#!/usr/bin/env python3
"""
Legatura cu ArduPilot: telemetrie intr-o parte, comenzi in cealalta.

Singura diferenta intre simulare si vehiculul real e sirul de conectare
(sectiunea 8 din CLAUDE.md):

    Vehicle('udpin:127.0.0.1:14552')            # SITL
    Vehicle('/dev/serial0', baud=921600)        # Raspberry Pi

Masina de stari vede doar interfata de aici, deci nu stie pe ce link merge.
"""

import collections
import time

from pymavlink import mavutil

from .detection import MARKER_SIZE_M

TELEM_HZ = 20
LANDED_HZ = 50        # EXTENDED_SYS_STATE: semnalul cu care prindem
                      # fereastra de ~0.5 s dinainte de dezarmare (5.6)
#: HEARTBEAT: ArduPilot il trimite implicit la **1 Hz**
#: (`GCS_Common.cpp`: `set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT,
#: 1000)`, tratat ca un caz special pentru ca nu e "streamed"). Cu
#: `safety.LINK_MAX_AGE_S = 1.0`, monitorul de legatura ar avea **marja
#: zero**: declara legatura cazuta exact cand soseste urmatorul heartbeat.
#: Orice jitter - o iteratie mai lenta, un pachet pierdut - il declanseaza,
#: iar actiunea lui e BRAKE, adica incercarea autonoma anulata.
#:
#: Masurat in Gazebo: secventa a pornit, a coborat 1.2 m si a fost oprita la
#: `fara HEARTBEAT de 1.02 s (prag 1.00 s)`. Nu e artefact de simulare -
#: pe vehicul raportul e identic, 1 Hz fata de un prag de 1 s.
#:
#: La 5 Hz pragul inseamna 5 heartbeat-uri pierdute la rand. Un test leaga
#: cele doua constante, ca sa nu poata fi schimbata una singura.
HEARTBEAT_HZ = 5

RC_HZ = 50            # 15.3.1: implicit RC_CHANNELS vine la 10 Hz, adica
                      # 100 ms consumate doar de streaming, dintr-un buget
                      # total de 250 ms. La 50 Hz raman 20 ms.

# --- Moduri ArduCopter ----------------------------------------------------
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9
MODE_BRAKE = 17

MODE_NAME = {MODE_GUIDED: 'GUIDED', MODE_LOITER: 'LOITER', MODE_RTL: 'RTL',
             MODE_LAND: 'LAND', MODE_BRAKE: 'BRAKE',
             2: 'ALT_HOLD', 0: 'STABILIZE'}

# Reincercarea PARAM_SET pana la confirmare prin PARAM_VALUE. Parametrii pe
# care ii comutam in zbor (PLND_ENABLED) sunt critici pentru siguranta, deci
# nu ii trimitem "si speram".
PARAM_RETRY_S = 0.2
PARAM_TRIES = 5

RANGE_MIN_M = 0.05
RANGE_MAX_M = 30.0

# --- Sanatatea legaturii cu FC-ul (H1) -----------------------------------
# Un cablu serial care se misca nu trebuie sa omoare procesul in mijlocul
# unei curse. FC-ul trimite HEARTBEAT la 1 Hz; 3 s inseamna trei batai
# pierdute, adica destul cat sa nu declansam pe o intarziere si destul de
# putin cat sa reactionam inainte sa conteze.
HEARTBEAT_TIMEOUT_S = 3.0

#: Backoff exponential intre incercarile de redeschidere: 1, 2, 4, 8, 8, ...
#: Plafonul exista pentru ca un cablu rebransat dupa zece minute trebuie sa
#: fie prins in cel mult 8 s, nu dupa un interval care a crescut la infinit.
RECONNECT_BACKOFF_S = (1.0, 2.0, 4.0, 8.0)
RECONNECT_BACKOFF_MAX_S = 8.0


class Vehicle:

    def __init__(self, conn, baud=None, telem_hz=TELEM_HZ, landed_hz=LANDED_HZ):
        self.conn_str = conn
        self.baud = baud
        self.telem_hz = telem_hz
        self.landed_hz = landed_hz

        # stare vehicul (NED, metri)
        self.x = self.y = self.z = 0.0
        self.vz = 0.0
        self.rel_alt = None          # deasupra home, din GLOBAL_POSITION_INT
        self.lat = self.lon = None   # grade; necesare pentru geofence (15.2.4)
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = False
        self.armed = False
        self.mode = None
        self.landed_state = mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED
        # Ceasul FC-ului, pentru loguri sincronizabile cu telemetria (6.2.1.30).
        # Ceasul Pi-ului nu e o referinta buna: nu e acelasi cu al FC-ului si
        # nu apare in .bin.
        self.time_boot_ms = None

        # RC brut, canalele 1..8, asa cum le raporteaza FC-ul. In GUIDED
        # ArduPilot ignora manetele, deci asta e singura cale prin care
        # companion-ul afla ca pilotul a pus mana pe ele (15.3.1).
        self.rc = None
        self.rc_t = None

        # parametri: cache din PARAM_VALUE + cereri neconfirmate inca
        self.params = {}
        #: Cand am vazut ultima data fiecare parametru (time.monotonic()).
        #: Fara asta, un consumator nu poate deosebi o valoare citita ACUM de
        #: una ramasa in cache de acum zece minute - iar intre timp cineva
        #: poate sa fi schimbat-o din GCS. Conteaza pentru oricine salveaza
        #: valori ca sa le restaureze (nova/authority.py).
        self.params_t = {}
        self._param_pending = {}     # nume -> [valoare, tries, last_send]

        # statistici de emisie
        self.n_lt = 0
        self.n_ds = 0

        self.ds_extended = True
        self.m = None

        # H1: sanatatea legaturii. `link_healthy` e False de la inceput si
        # devine True la primul HEARTBEAT - nu presupunem ca merge pana la
        # proba contrarie.
        #: Ultimul raspuns la SET_EKF_SOURCE_SET, sau None daca nu s-a cerut.
        self.ekf_src_ack = None
        self.hb_t = None              # time.monotonic() al ultimului HEARTBEAT
        #: Ultimele intervale intre heartbeat-uri, ca sa se poata verifica
        #: daca rata ceruta s-a aplicat (vezi heartbeat_interval).
        self.hb_gaps = collections.deque(maxlen=20)
        self.link_healthy = False
        self.reconnects = 0           # cate redeschideri au reusit
        self.reconnect_attempts = 0   # cate s-au incercat de la ultima reusita
        self._reconnect_next_t = None
        self.on_link_event = None     # callback(dict) pentru log
        self.link_verbose = True      # print pe stdout; testele il sting

        # Protocolul de misiune (fence) are nevoie de mesajele MISSION_*, dar
        # pump() le-ar consuma primul. Cine face upload/download isi pune aici
        # un handler si le primeste, fara sa oprim bucla principala.
        self.mission_handler = None

    # -- initializare ------------------------------------------------------
    def connect(self, verbose=True):
        if verbose:
            print(f"[vehicle] conectare la {self.conn_str} ...")
        kwargs = {'baud': self.baud} if self.baud else {}
        self.m = mavutil.mavlink_connection(self.conn_str, **kwargs)
        self.m.wait_heartbeat()
        self._note_heartbeat()
        if verbose:
            print(f"[vehicle] heartbeat sys={self.m.target_system} "
                  f"comp={self.m.target_component}")
        self._request_streams()
        self._probe_distance_api(verbose)
        return self

    # -- sanatatea legaturii (H1) ------------------------------------------
    def _note_heartbeat(self, now=None, detail='HEARTBEAT primit'):
        """Un HEARTBEAT de la FC. Emite `link_up` DOAR la tranzitie.

        Evenimentul e unul singur, aici: prima varianta il emitea si de aici,
        si de la sfarsitul lui `_try_reopen`, deci fiecare reconectare aparea
        de doua ori in log. Un log de siguranta care numara gresit
        evenimentele e mai rau decat unul absent."""
        now = now if now is not None else time.monotonic()
        # §5.10 aplicat unui STREAM: cererea de rata nu e aplicata pana nu a
        # fost observata. Nu putem citi inapoi un interval de mesaj, dar
        # putem masura ce soseste.
        if self.hb_t is not None and now > self.hb_t:
            self.hb_gaps.append(now - self.hb_t)
        self.hb_t = now
        if not self.link_healthy:
            self.link_healthy = True
            incercari = self.reconnect_attempts
            self.reconnect_attempts = 0
            self._reconnect_next_t = None
            if incercari:
                detail = (f"{detail} dupa {incercari} incercari "
                          f"(reconectari reusite: {self.reconnects})")
            self._link_event('link_up', now, detail)

    def _link_event(self, kind, now, detail):
        """Un eveniment de legatura, cu ceasul FC-ului cand exista.

        `time_boot_ms` e ultima valoare primita INAINTE de caderea legaturii,
        deci in logul de reconectare e o ancora spre .bin (6.2.1.30), nu o
        valoare curenta. Se noteaza ca atare."""
        ev = {'kind': kind, 't': now, 'time_boot_ms': self.time_boot_ms,
              'detail': detail, 'attempts': self.reconnect_attempts}
        if self.link_verbose:
            tb = '-' if self.time_boot_ms is None else str(self.time_boot_ms)
            print(f"[link {kind}] t={now:.3f} boot_ms={tb} {detail}")
        if self.on_link_event:
            self.on_link_event(ev)
        return ev

    def heartbeat_interval(self):
        """Intervalul MEDIAN observat intre heartbeat-uri, sau None.

        Cerut 1/HEARTBEAT_HZ; daca iese ~1 s, cererea nu s-a aplicat si
        monitorul de legatura ramane fara marja."""
        with_gaps = sorted(self.hb_gaps)
        if len(with_gaps) < 3:
            return None
        return with_gaps[len(with_gaps) // 2]

    def time_since_heartbeat(self, now=None):
        """Secunde de la ultimul HEARTBEAT, sau None daca nu a existat."""
        if self.hb_t is None:
            return None
        now = now if now is not None else time.monotonic()
        return now - self.hb_t

    def _backoff_s(self, attempt):
        """Pauza DUPA incercarea `attempt` (1-based): 1, 2, 4, 8, 8, ...

        Indexul e `attempt - 1`, nu `attempt`: prima incercare trebuie urmata
        de 1 s, nu de 2. Scris gresit, secventa pornea de la al doilea
        element si pierdeam prima secunda de reconectare - cel mai probabil
        moment in care cablul e deja inapoi la loc."""
        i = min(max(attempt - 1, 0), len(RECONNECT_BACKOFF_S) - 1)
        return min(RECONNECT_BACKOFF_S[i], RECONNECT_BACKOFF_MAX_S)

    def check_link(self, now=None):
        """De apelat din bucla, dupa pump(). NU blocheaza niciodata.

        Daca heartbeat-ul lipseste de peste HEARTBEAT_TIMEOUT_S, incearca sa
        redeschida portul - o singura incercare per apel, distantata prin
        backoff. Detectorul si restul buclei continua intre incercari; doar
        emisia catre FC e suspendata (vezi `link_healthy`).

        Intoarce True cat timp legatura e considerata buna."""
        now = now if now is not None else time.monotonic()
        age = self.time_since_heartbeat(now)
        if age is not None and age <= HEARTBEAT_TIMEOUT_S:
            return True

        if self.link_healthy:
            self.link_healthy = False
            self._reconnect_next_t = now          # prima incercare imediat
            self._link_event(
                'link_down', now,
                f"fara HEARTBEAT de {age:.1f} s (prag "
                f"{HEARTBEAT_TIMEOUT_S:.0f} s)" if age is not None
                else 'niciun HEARTBEAT de la pornire')

        if self._reconnect_next_t is None or now < self._reconnect_next_t:
            return False
        self._try_reopen(now)
        return self.link_healthy

    def _try_reopen(self, now):
        """O singura incercare de redeschidere. Orice esec e normal aici:
        portul poate lipsi cu totul daca s-a scos cablul."""
        self.reconnect_attempts += 1
        wait = self._backoff_s(self.reconnect_attempts)
        self._reconnect_next_t = now + wait
        self._link_event('reconnect_try', now,
                         f"incercarea {self.reconnect_attempts}, "
                         f"urmatoarea peste {wait:.0f} s")
        try:
            if self.m is not None:
                try:
                    self.m.close()
                except Exception:                           # noqa: BLE001
                    pass
            kwargs = {'baud': self.baud} if self.baud else {}
            self.m = mavutil.mavlink_connection(self.conn_str, **kwargs)
        except Exception as e:                              # noqa: BLE001
            self._link_event('reconnect_fail', now,
                             f"{type(e).__name__}: {e}")
            return False

        # Deschiderea portului nu inseamna ca FC-ul e acolo. Asteptam scurt
        # un HEARTBEAT - NU wait_heartbeat(), care blocheaza la nesfarsit si
        # ar ingheta bucla si supervizorul odata cu ea.
        hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is None:
            self._link_event('reconnect_fail', now,
                             'port deschis, dar niciun HEARTBEAT in 0.5 s')
            return False

        self.reconnects += 1
        self._note_heartbeat(now, 'reconectat')
        # Fluxurile se cer din nou: FC-ul nu retine intervalele peste o
        # reconectare daca a fost si el repornit.
        try:
            self._request_streams()
        except Exception as e:                              # noqa: BLE001
            self._link_event('reconnect_warn', now,
                             f"fluxurile nu s-au putut cere: {e}")
        return True

    def _request_streams(self):
        rates = [
            # Primul, fiindca de el atarna monitorul de legatura (H1).
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, HEARTBEAT_HZ),
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, self.telem_hz),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, self.telem_hz),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, self.telem_hz),
            # ArduPilot expune land_complete doar prin landed_state. E
            # singurul semnal care ne spune ca a inceput fereastra de
            # ~0.5 s pana la dezarmare, deci il cerem rapid.
            (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, self.landed_hz),
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, RC_HZ),
        ]
        for msg_id, hz in rates:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def _probe_distance_api(self, verbose=True):
        """Semnatura distance_sensor_send difera intre versiuni pymavlink."""
        self.ds_extended = True
        try:
            self.m.mav.distance_sensor_send(
                0, 5, 3000, 100,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 1,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0,
                horizontal_fov=0.0, vertical_fov=0.0,
                quaternion=[0.0, 0.0, 0.0, 0.0], signal_quality=0)
        except TypeError:
            self.ds_extended = False
            if verbose:
                print("[vehicle] pymavlink vechi: fara signal_quality")

    # -- telemetrie --------------------------------------------------------
    def pump(self):
        """Goleste coada de mesaje. De apelat la viteza buclei.

        Nu arunca daca portul a disparut sub noi: un cablu scos face
        `recv_match` sa ridice OSError, iar asta ar omori bucla exact in
        situatia pentru care exista reconectarea. Erorile de citire se
        trateaza ca "niciun mesaj"; `check_link()` decide ce se intampla."""
        while True:
            try:
                msg = self.m.recv_match(blocking=False)
            except Exception:                               # noqa: BLE001
                return
            if msg is None:
                return
            t = msg.get_type()
            if hasattr(msg, 'time_boot_ms'):
                self.time_boot_ms = msg.time_boot_ms
            if t == 'LOCAL_POSITION_NED':
                self.x, self.y, self.z = msg.x, msg.y, msg.z
                self.vz = msg.vz
                self.have_pos = True
            elif t == 'ATTITUDE':
                self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            elif t == 'GLOBAL_POSITION_INT':
                self.rel_alt = msg.relative_alt / 1000.0
                if msg.lat != 0 or msg.lon != 0:
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
            elif t == 'EXTENDED_SYS_STATE':
                self.landed_state = msg.landed_state
            elif t == 'HEARTBEAT':
                if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                    continue
                self._note_heartbeat()
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = msg.custom_mode
            elif t == 'RC_CHANNELS':
                self.rc = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                           msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                           msg.chan7_raw, msg.chan8_raw)
                self.rc_t = time.monotonic()
            elif t.startswith('MISSION_'):
                if self.mission_handler is not None:
                    self.mission_handler(msg)
            elif t == 'PARAM_VALUE':
                self._on_param(msg)
            elif t == 'COMMAND_ACK':
                self._on_ack(msg)

    def _on_param(self, msg):
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii', 'ignore')
        name = name.rstrip('\x00')
        self.params[name] = msg.param_value
        self.params_t[name] = time.monotonic()
        want = self._param_pending.get(name)
        if want is not None and abs(msg.param_value - want[0]) < 1e-6:
            del self._param_pending[name]

    def _on_ack(self, msg):
        if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"!! NAV_TAKEOFF respins (result={msg.result})")
        elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"!! DO_SET_MODE respins (result={msg.result})")
        elif msg.command == mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
            # 15.2.5: comutarea de surse e o afirmatie de conformitate, deci
            # raspunsul FC-ului se pastreaza, nu doar se tipareste.
            self.ekf_src_ack = msg.result
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"!! SET_EKF_SOURCE_SET respins (result={msg.result})")

    @property
    def alt(self):
        """Altitudine deasupra originii EKF, metri (z e NED, deci negativ)."""
        return -self.z

    def on_ground(self):
        """land_complete, singurul mod in care e expus prin MAVLink (5.6)."""
        return self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    def mode_name(self):
        return MODE_NAME.get(self.mode, str(self.mode))

    # -- parametri ---------------------------------------------------------
    def set_param(self, name, value, now=None):
        """PARAM_SET cu confirmare. Valoarea se considera aplicata abia cand
        FC-ul raspunde cu PARAM_VALUE; pana atunci update_params() reincearca."""
        now = now if now is not None else time.monotonic()
        value = float(value)
        self._param_pending[name] = [value, 1, now]
        self._send_param(name, value)

    def _send_param(self, name, value):
        return self._send(
            self.m.mav.param_set_send,
            self.m.target_system, self.m.target_component,
            name.encode('ascii'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def request_param(self, name):
        """Cere o citire; valoarea ajunge in self.params prin pump()."""
        return self._send(
            self.m.mav.param_request_read_send,
            self.m.target_system, self.m.target_component,
            name.encode('ascii'), -1)

    def update_params(self, now):
        """De apelat din bucla principala: reincearca ce nu s-a confirmat."""
        for name, rec in list(self._param_pending.items()):
            value, tries, last = rec
            if now - last < PARAM_RETRY_S:
                continue
            if tries >= PARAM_TRIES:
                print(f"!! PARAM_SET {name}={value:g} neconfirmat dupa "
                      f"{tries} incercari")
                del self._param_pending[name]
                continue
            rec[1] = tries + 1
            rec[2] = now
            self._send_param(name, value)

    def param_pending(self, name=None):
        if name is None:
            return bool(self._param_pending)
        return name in self._param_pending

    # -- comenzi -----------------------------------------------------------
    # Toate intorc True daca octetii chiar au plecat. Cand legatura e cazuta
    # NU arunca si NU trimit: bucla principala trebuie sa continue (H1), iar
    # apelantul trebuie sa poata deosebi "trimis" de "suspendat". Diferenta
    # conteaza pentru SafetySupervisor, care altfel si-ar consuma cele 5
    # reincercari de mod vorbind cu un port inchis, si ar declara mode_fail
    # fara sa fi trimis nimic.

    def _send(self, fn, *args, **kwargs):
        if not self.link_healthy:
            return False
        try:
            fn(*args, **kwargs)
            return True
        except Exception as e:                              # noqa: BLE001
            # Scrierea a esuat: portul tocmai a disparut. Marcam legatura
            # cazuta acum, ca sa nu asteptam expirarea heartbeat-ului.
            if self.link_healthy:
                self.link_healthy = False
                self._reconnect_next_t = time.monotonic()
                self._link_event('link_down', time.monotonic(),
                                 f"scriere esuata: {type(e).__name__}: {e}")
            return False

    def request_mode(self, mode):
        return self._send(
            self.m.mav.command_long_send,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode, 0, 0, 0, 0, 0)

    def send_ekf_source_set(self, n):
        """Comuta setul de surse al EKF3 (15.2.5). `n` e 1..3.

        `GCS_Common.cpp:5133`: comanda accepta 1..3 si scade 1 intern.
        Confirmarea vine prin `COMMAND_ACK`, citit in `_on_ack`."""
        if n not in (1, 2, 3):
            raise ValueError(f"setul de surse EKF e 1..3, nu {n}")
        self.ekf_src_ack = None
        return self._send(
            self.m.mav.command_long_send,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 0,
            float(n), 0, 0, 0, 0, 0, 0)

    def send_takeoff(self, alt_above_home_m):
        # Copter accepta NAV_TAKEOFF ca COMMAND_LONG; cadrul devine
        # GLOBAL_RELATIVE_ALT, deci param7 e altitudine deasupra HOME.
        return self._send(
            self.m.mav.command_long_send,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_above_home_m)

    def send_landing_target(self, angle_x, angle_y, dist):
        ok = self._send(
            self.m.mav.landing_target_send,
            int(time.time() * 1e6), 0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x, angle_y, dist,
            MARKER_SIZE_M, MARKER_SIZE_M)
        if ok:
            self.n_lt += 1
        return ok

    def send_distance(self, rng_m):
        cm = int(max(RANGE_MIN_M, min(rng_m, RANGE_MAX_M)) * 100)
        args = (int(time.monotonic() * 1000), 5, 3000, cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 1,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)
        if self.ds_extended:
            ok = self._send(self.m.mav.distance_sensor_send, *args,
                            horizontal_fov=0.0, vertical_fov=0.0,
                            quaternion=[0.0, 0.0, 0.0, 0.0],
                            signal_quality=100)
        else:
            ok = self._send(self.m.mav.distance_sensor_send, *args)
        if ok:
            self.n_ds += 1
        return ok
