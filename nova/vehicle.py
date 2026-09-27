#!/usr/bin/env python3
"""
Legatura cu ArduPilot: telemetrie intr-o parte, comenzi in cealalta.

Singura diferenta intre simulare si vehiculul real e sirul de conectare
(sectiunea 8 din CLAUDE.md):

    Vehicle('udpin:127.0.0.1:14552')                       # SITL, sincron
    Vehicle('/dev/serial0', baud=921600, threaded=True)    # Raspberry Pi

Masina de stari vede doar interfata de aici, deci nu stie pe ce link merge.

DOUA MODURI, O SINGURA INTERFATA (refactor/threads, faza 2)

  sincron (implicit; simulatorul si testele): `pump()` citeste portul,
    `check_link()` reconecteaza, `update_params()` reincearca, iar fiecare
    `send_*` scrie pe port pe loc - exact comportamentul de dinainte.

  cu fir I/O (`threaded=True`; aplicatia de bord): un singur fir detine
    portul si obiectul `mav`. El primeste (recv_match cu timeout 10 ms),
    tine starea vie intr-un VehicleState si o publica ca instantaneu in
    `Latest`; goleste intai coada URGENT, apoi TX; reincearca parametrii;
    reconecteaza. Firul principal vede prin `pump()` DOAR instantaneul
    (aceleasi atribute ca inainte: x, mode, rc, ...), consistent pe durata
    unei iteratii, si trimite DOAR prin cozi - `send_*` intoarce True cand
    comanda a intrat in coada. Nimeni altcineva nu atinge `mav`.

    Cozile: comenzile (mod, parametru, sursa EKF, decolare, STATUSTEXT) nu
    se pierd tacut - coada plina e eroare logata si False; mesajele
    periodice (LANDING_TARGET, DISTANCE_SENSOR, VISION_POSITION_ESTIMATE,
    consemnul GUIDED) au cate un loc pe tip si sunt inlocuite de cel mai
    nou. URGENT e pentru supervizor (`request_mode(..., urgent=True)`).

    Ce ramane pe atribute simple, scrise de firul I/O si citite atomic:
    starea legaturii (hb_t, link_healthy, contoarele de reconectare) si
    dictionarele de parametri (get / `in` sunt atomice sub GIL). Istoricul
    de atitudine si pozitie sta in AttitudeBuffer, sub lock propriu.
"""

import collections
import logging
import queue
import threading
import time

from pymavlink import mavutil

from .concurrency import AttitudeBuffer, Heartbeat, Latest, interpolate, run_thread
from .detection import MARKER_SIZE_M

log = logging.getLogger('nova')

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
#: EKF_STATUS_REPORT (ExtNav, 27.09.2026): the supervisor exits the
#: segment when the EKF loses a valid horizontal position. 5 Hz is enough
#: for a window measured in seconds, and cheap on the serial link.
EKF_HZ = 5
#: How much attitude / position history is kept for the ExtNav estimator,
#: which needs the attitude at the CAPTURE time of a frame (§4 of the
#: brief), not the latest one. At 20 Hz, 400 samples = 20 s.
HIST_SAMPLES = 400
#: Beyond this, a requested time is outside what the history can answer:
#: the estimator gets None, never the nearest sample presented as exact.
HIST_MAX_GAP_S = 0.25
#: SET_POSITION_TARGET_LOCAL_NED with position + yaw only: velocities,
#: accelerations and yaw rate ignored. Values from the MAVLink enum
#: POSITION_TARGET_TYPEMASK (checked in pymavlink, not from memory).
POS_TARGET_MASK_POS_YAW = (
    8 | 16 | 32          # VX, VY, VZ ignore
    | 64 | 128 | 256     # AX, AY, AZ ignore
    | 2048)              # YAW_RATE ignore

# --- Moduri ArduCopter ----------------------------------------------------
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9
MODE_BRAKE = 17

MODE_NAME = {MODE_GUIDED: 'GUIDED', MODE_LOITER: 'LOITER', MODE_RTL: 'RTL',
             MODE_LAND: 'LAND', MODE_BRAKE: 'BRAKE',
             MODE_ALT_HOLD: 'ALT_HOLD', MODE_STABILIZE: 'STABILIZE'}

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

# --- Firul I/O (faza 2) -------------------------------------------------------
#: recv_match(blocking=True, timeout=IO_RECV_TIMEOUT_S), apoi cel mult
#: IO_RECV_BURST mesaje fara blocare: o rafala se goleste intr-o iteratie,
#: iar cozile spre serial se golesc cel putin la fiecare 10 ms.
IO_RECV_TIMEOUT_S = 0.01
IO_RECV_BURST = 50
#: Cozi marginite. Comenzile nu se arunca tacut: plina = eroare logata.
QUEUE_URGENT = 32
QUEUE_TX = 64
#: Mesajele periodice, cate un loc pe tip; unul nou il inlocuieste pe cel
#: neplecat inca.
PERIODIC_KEYS = ('landing_target', 'distance', 'vpe', 'pos_target')

#: Telemetria pe care o vad consumatorii. Un singur obiect scris de firul
#: I/O (`_live`), publicat ca instantaneu in `Latest`, copiat in fatada de
#: `pump()`. In modul sincron `_live` E fatada, deci scrierile ajung direct.
STATE_FIELDS = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'rel_alt', 'lat', 'lon',
                'roll', 'pitch', 'yaw', 'have_pos', 'armed', 'mode',
                'landed_state', 'time_boot_ms', 'rc', 'rc_t', 'ekf_flags',
                'ekf_pos_horiz_var', 'ekf_t', 'ekf_src_ack',
                'n_lt', 'n_ds', 'n_vpe', 'n_pos_target')


class VehicleState:
    """Telemetria, ca obiect simplu cu campurile din STATE_FIELDS."""

    __slots__ = STATE_FIELDS

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.rel_alt = None
        self.lat = self.lon = None
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = False
        self.armed = False
        self.mode = None
        self.landed_state = mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED
        self.time_boot_ms = None
        self.rc = None
        self.rc_t = None
        self.ekf_flags = None
        self.ekf_pos_horiz_var = None
        self.ekf_t = None
        self.ekf_src_ack = None
        self.n_lt = self.n_ds = self.n_vpe = self.n_pos_target = 0

    def copy(self):
        c = VehicleState.__new__(VehicleState)
        for f in STATE_FIELDS:
            setattr(c, f, getattr(self, f))
        return c


class VehicleView:
    """Vederea unui ALT fir asupra vehiculului (faza 3: supervizorul).

    Aceleasi atribute ca fatada lui Vehicle (x, mode, rc, ...), copiate din
    ultimul instantaneu la `refresh()` - deci consistente pe durata unui
    pas al firului care o detine - si aceleasi metode de citire. Comenzile
    pleaca prin cozile vehiculului, cele de mod pe URGENT; nimic nu atinge
    portul. Fara instantaneu publicat (modul sincron, testele), refresh()
    copiaza fatada vehiculului."""

    def __init__(self, vehicle):
        self._v = vehicle
        for f in STATE_FIELDS:
            setattr(self, f, getattr(vehicle, f))
        self.link_healthy = vehicle.link_healthy
        self.hb_t = vehicle.hb_t
        self.last_snapshot_t = None

    def refresh(self):
        v = self._v
        snap, t = v.state.get()
        src = snap if snap is not None else v
        for f in STATE_FIELDS:
            setattr(self, f, getattr(src, f))
        self.link_healthy = v.link_healthy
        self.hb_t = v.hb_t
        self.last_snapshot_t = t
        return self

    # -- citiri, ca la Vehicle ----------------------------------------------
    @property
    def alt(self):
        return -self.z

    @property
    def params(self):
        return self._v.params

    @property
    def time_boot_ms_vehicle(self):
        return self.time_boot_ms

    def mode_name(self):
        return MODE_NAME.get(self.mode, str(self.mode))

    def on_ground(self):
        return self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    def time_since_heartbeat(self, now=None):
        hb = self.hb_t
        if hb is None:
            return None
        now = now if now is not None else time.monotonic()
        return now - hb

    def ekf_pos_horiz_ok(self):
        if self.ekf_flags is None:
            return None
        return bool(self.ekf_flags & mavutil.mavlink.EKF_POS_HORIZ_REL)

    def attitude_at(self, t):
        return self._v.attitude_at(t)

    def position_at(self, t):
        return self._v.position_at(t)

    # -- comenzi: prin cozile vehiculului, cele de mod pe URGENT ---------
    def request_mode(self, mode, urgent=True):
        return self._v.request_mode(mode, urgent=urgent)

    def send_ekf_source_set(self, n, urgent=True):
        return self._v.send_ekf_source_set(n, urgent=urgent)

    def send_statustext(self, severity, text):
        return self._v.send_statustext(severity, text)

    def request_param(self, name):
        return self._v.request_param(name)


class Vehicle:

    def __init__(self, conn, baud=None, telem_hz=TELEM_HZ, landed_hz=LANDED_HZ,
                 threaded=False):
        self.conn_str = conn
        self.baud = baud
        self.telem_hz = telem_hz
        self.landed_hz = landed_hz
        #: True = firul I/O detine portul; False = totul in firul apelantului.
        self.threaded = bool(threaded)

        # stare vehicul (NED, metri) - fatada citita de consumatori
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        #: EKF_STATUS_REPORT: flags bitmask and variances, or None until
        #: the first report. Unknown is not "ok".
        self.ekf_flags = None
        self.ekf_t = None
        self.ekf_pos_horiz_var = None
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

        #: Ultimul raspuns la SET_EKF_SOURCE_SET, sau None daca nu s-a cerut.
        self.ekf_src_ack = None

        # statistici de emisie (numarate cand octetii chiar pleaca)
        self.n_lt = 0
        self.n_ds = 0
        self.n_vpe = 0          # VISION_POSITION_ESTIMATE
        self.n_pos_target = 0   # SET_POSITION_TARGET_LOCAL_NED

        #: ExtNav (27.09.2026): short histories with the loop clock, so the
        #: estimator can ask "where was the vehicle / how was it tilted when
        #: this frame was captured" - see attitude_at() / position_at().
        #: Thread-safe buffers: the I/O thread pushes, the main thread reads.
        self.attitude = AttitudeBuffer(HIST_SAMPLES, HIST_MAX_GAP_S, wrap=(0, 1, 2))
        self.positions = AttitudeBuffer(HIST_SAMPLES, HIST_MAX_GAP_S, wrap=())

        # parametri: cache din PARAM_VALUE + cereri neconfirmate inca.
        # Scrise de firul I/O; get() si `in` sunt atomice sub GIL.
        self.params = {}
        #: Cand am vazut ultima data fiecare parametru (time.monotonic()).
        #: Fara asta, un consumator nu poate deosebi o valoare citita ACUM de
        #: una ramasa in cache de acum zece minute - iar intre timp cineva
        #: poate sa fi schimbat-o din GCS. Conteaza pentru oricine salveaza
        #: valori ca sa le restaureze (nova/authority.py).
        self.params_t = {}
        self._param_pending = {}     # nume -> [valoare, tries, last_send]

        #: Last mode asked for through request_mode(), sent or not. The
        #: state machine reads it in ACQUIRE (B3): a LAND retry must not
        #: follow a BRAKE the supervisor asked for in the same loop, before
        #: the FC even reported it. Intent, not delivery - hence set even
        #: when the link is down and nothing left the port. Main-thread
        #: owned (the caller's intent), never touched by the I/O thread.
        self.last_mode_req = None

        self.ds_extended = True
        self.m = None
        self._sysid = 1
        self._compid = 1

        # H1: sanatatea legaturii. `link_healthy` e False de la inceput si
        # devine True la primul HEARTBEAT - nu presupunem ca merge pana la
        # proba contrarie. Atribute simple, scrise de firul I/O.
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
        # un handler si le primeste, fara sa oprim bucla principala. Cu fir
        # I/O, handler-ul ruleaza IN firul I/O.
        self.mission_handler = None

        # --- firul I/O (faza 2) ---
        #: Starea vie: in modul sincron e chiar fatada (scrierile ajung
        #: direct, ca inainte); cu fir I/O e un VehicleState al firului.
        self._live = VehicleState() if self.threaded else self
        #: Ultimul instantaneu publicat de firul I/O.
        self.state = Latest()
        self._urgent = queue.Queue(maxsize=QUEUE_URGENT)
        self._tx = queue.Queue(maxsize=QUEUE_TX)
        self._periodic = {}
        self._periodic_lock = threading.Lock()
        self.io_heartbeat = Heartbeat('mav_io')
        self._stop = threading.Event()
        self._io_thread = None
        self.n_queue_full = 0

    # -- initializare ------------------------------------------------------
    def connect(self, verbose=True):
        if verbose:
            print(f"[vehicle] conectare la {self.conn_str} ...")
        kwargs = {'baud': self.baud} if self.baud else {}
        self.m = mavutil.mavlink_connection(self.conn_str, **kwargs)
        self.m.wait_heartbeat()
        self._sysid = self.m.target_system
        self._compid = self.m.target_component
        self._note_heartbeat()
        if verbose:
            print(f"[vehicle] heartbeat sys={self.m.target_system} "
                  f"comp={self.m.target_component}")
        self._request_streams()
        self._probe_distance_api(verbose)
        if self.threaded:
            self.start_io()
        return self

    def view(self):
        """O VehicleView pentru alt fir (supervizorul)."""
        return VehicleView(self)

    def start_io(self):
        """Porneste firul I/O (o singura data). De aici, portul e al lui."""
        if self._io_thread is None:
            self._stop.clear()
            self._io_thread = run_thread('mav_io', self._io_step, period=0,
                                         heartbeat=self.io_heartbeat,
                                         stop_event=self._stop)
        return self._io_thread

    def close(self, timeout_s=2.0):
        """Opreste firul I/O (daca exista) si inchide portul."""
        if self._io_thread is not None:
            self._stop.set()
            self._io_thread.join(timeout=timeout_s)
            self._io_thread = None
        if self.m is not None:
            try:
                self.m.close()
            except Exception:                               # noqa: BLE001
                pass

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
        tb = getattr(getattr(self, '_live', self), 'time_boot_ms', None)
        ev = {'kind': kind, 't': now, 'time_boot_ms': tb,
              'detail': detail, 'attempts': self.reconnect_attempts}
        if self.link_verbose:
            print(f"[link {kind}] t={now:.3f} "
                  f"boot_ms={'-' if tb is None else tb} {detail}")
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
        hb = self.hb_t
        if hb is None:
            return None
        now = now if now is not None else time.monotonic()
        return now - hb

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

        Sincron: daca heartbeat-ul lipseste de peste HEARTBEAT_TIMEOUT_S,
        incearca sa redeschida portul - o singura incercare per apel,
        distantata prin backoff. Cu fir I/O, reconectarea e treaba firului
        (`_check_link_io`) si aici se raporteaza doar starea.

        Intoarce True cat timp legatura e considerata buna."""
        if self.threaded:
            return self.link_healthy
        return self._check_link_io(now if now is not None else time.monotonic())

    def _check_link_io(self, now):
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
        self._sysid = getattr(self.m, 'target_system', self._sysid)
        self._compid = getattr(self.m, 'target_component', self._compid)
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
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, EKF_HZ),
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
        """De apelat la viteza buclei, din firul principal.

        Sincron: goleste coada de mesaje a portului (nu arunca daca portul a
        disparut sub noi - erorile de citire sunt "niciun mesaj", iar
        `check_link()` decide). Cu fir I/O: copiaza ultimul instantaneu
        publicat de fir in fatada - consumatorii vad aceeasi telemetrie pe
        toata iteratia."""
        if not self.threaded:
            self._recv_batch(blocking=False)
            return
        snap, _t = self.state.get()
        if snap is not None:
            for f in STATE_FIELDS:
                setattr(self, f, getattr(snap, f))

    def _recv_batch(self, blocking):
        """Un lot de mesaje: cu `blocking`, primul asteapta cel mult
        IO_RECV_TIMEOUT_S, apoi cel mult IO_RECV_BURST fara asteptare."""
        n = 0
        while True:
            try:
                if blocking and n == 0:
                    msg = self.m.recv_match(blocking=True,
                                            timeout=IO_RECV_TIMEOUT_S)
                else:
                    msg = self.m.recv_match(blocking=False)
            except Exception:                               # noqa: BLE001
                return
            if msg is None:
                return
            self._handle(msg)
            n += 1
            if blocking and n >= IO_RECV_BURST:
                return

    def _handle(self, msg):
        st = self._live
        t = msg.get_type()
        if hasattr(msg, 'time_boot_ms'):
            st.time_boot_ms = msg.time_boot_ms
        if t == 'LOCAL_POSITION_NED':
            self._on_position(time.monotonic(), msg.x, msg.y, msg.z,
                              msg.vx, msg.vy, msg.vz)
        elif t == 'ATTITUDE':
            self._on_attitude(time.monotonic(), msg.roll, msg.pitch, msg.yaw)
        elif t == 'EKF_STATUS_REPORT':
            st.ekf_flags = msg.flags
            st.ekf_pos_horiz_var = msg.pos_horiz_variance
            st.ekf_t = time.monotonic()
        elif t == 'GLOBAL_POSITION_INT':
            st.rel_alt = msg.relative_alt / 1000.0
            if msg.lat != 0 or msg.lon != 0:
                st.lat = msg.lat / 1e7
                st.lon = msg.lon / 1e7
        elif t == 'EXTENDED_SYS_STATE':
            st.landed_state = msg.landed_state
        elif t == 'HEARTBEAT':
            if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                return
            self._note_heartbeat()
            st.armed = bool(msg.base_mode &
                            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st.mode = msg.custom_mode
        elif t == 'RC_CHANNELS':
            st.rc = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                     msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                     msg.chan7_raw, msg.chan8_raw)
            st.rc_t = time.monotonic()
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
            self._param_pending.pop(name, None)

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
            self._live.ekf_src_ack = msg.result
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"!! SET_EKF_SOURCE_SET respins (result={msg.result})")

    def _on_position(self, t, x, y, z, vx, vy, vz):
        st = self._live
        st.x, st.y, st.z = x, y, z
        st.vx, st.vy, st.vz = vx, vy, vz
        st.have_pos = True
        self.positions.push(t, x, y, z)

    def _on_attitude(self, t, roll, pitch, yaw):
        st = self._live
        st.roll, st.pitch, st.yaw = roll, pitch, yaw
        self.attitude.push(t, roll, pitch, yaw)

    # Istoricul, ca liste (teste, unelte). Copii: nu se itereaza deque-ul
    # firului I/O.
    @property
    def att_hist(self):
        return self.attitude.snapshot()

    @property
    def pos_hist(self):
        return self.positions.snapshot()

    @staticmethod
    def _interp(hist, t, wrap):
        return interpolate(hist, t, wrap=wrap, max_gap_s=HIST_MAX_GAP_S)

    def attitude_at(self, t):
        """(roll, pitch, yaw) at loop time t, interpolated, or None."""
        return self.attitude.at(t)

    def position_at(self, t):
        """(x, y, z) NED at loop time t, interpolated, or None."""
        return self.positions.at(t)

    def ekf_pos_horiz_ok(self):
        """True/False from the last EKF_STATUS_REPORT; None if none yet.

        Relative horizontal position is what ExtNav gives (no GPS, no
        absolute frame): EKF_POS_HORIZ_REL. Unknown stays None - the
        supervisor decides what to do with "no report", not this method."""
        if self.ekf_flags is None:
            return None
        return bool(self.ekf_flags & mavutil.mavlink.EKF_POS_HORIZ_REL)

    @property
    def alt(self):
        """Altitudine deasupra originii EKF, metri (z e NED, deci negativ)."""
        return -self.z

    def on_ground(self):
        """land_complete, singurul mod in care e expus prin MAVLink (5.6)."""
        return self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    def mode_name(self):
        return MODE_NAME.get(self.mode, str(self.mode))

    # -- firul I/O ---------------------------------------------------------
    def _io_step(self, now):
        """O iteratie a firului I/O: primeste, verifica legatura, goleste
        URGENT apoi TX, reincearca parametrii, publica instantaneul. Nimic
        lung: cel mult 10 ms de asteptare pe port."""
        self._recv_batch(blocking=True)
        now = now if now is not None else time.monotonic()
        self._check_link_io(now)
        self._drain_queues(now)
        self._update_params_io(now)
        self.state.set(self._live.copy(), now)

    def _drain_queues(self, now):
        for q in (self._urgent, self._tx):
            while True:
                try:
                    item = q.get_nowait()
                except queue.Empty:
                    break
                self._dispatch(item, now)
        with self._periodic_lock:
            items = list(self._periodic.values())
            self._periodic.clear()
        for item in items:
            self._dispatch(item, now)

    def _dispatch(self, item, now):
        """Executa un element de coada, in firul I/O (sau inline, sincron)."""
        kind = item[0]
        if kind == 'send':
            _, name, args, kwargs, counter = item
            ok = self._write(name, args, kwargs)
            if ok and counter:
                setattr(self._live, counter, getattr(self._live, counter) + 1)
            return ok
        if kind == 'param_set':
            _, name, value = item
            self._param_pending[name] = [value, 1, now]
            return self._write('param_set_send',
                               (self._sysid, self._compid, name.encode('ascii'),
                                value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32), {})
        if kind == 'ekf_src':
            _, n = item
            self._live.ekf_src_ack = None
            return self._write('command_long_send',
                               (self._sysid, self._compid,
                                mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, 0,
                                float(n), 0, 0, 0, 0, 0, 0), {})
        return False

    def _write(self, name, args, kwargs):
        """Scrierea propriu-zisa pe port. Singurul loc care atinge `mav`
        pentru trimitere. Esecul marcheaza legatura cazuta pe loc.

        Cu firul I/O pornit, doar EL are voie aici: `mav` din pymavlink nu
        e sigur la acces din mai multe fire, iar o scriere din alt fir ar
        fi exact bug-ul pe care cozile exista sa-l faca imposibil."""
        if (self._io_thread is not None
                and threading.current_thread() is not self._io_thread):
            raise RuntimeError(
                f"{name} din firul {threading.current_thread().name}: "
                f"portul e al firului I/O; trimite prin cozi")
        if not self.link_healthy:
            return False
        try:
            getattr(self.m.mav, name)(*args, **kwargs)
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

    def _update_params_io(self, now):
        for name, rec in list(self._param_pending.items()):
            value, tries, last = rec
            if now - last < PARAM_RETRY_S:
                continue
            if tries >= PARAM_TRIES:
                print(f"!! PARAM_SET {name}={value:g} neconfirmat dupa "
                      f"{tries} incercari")
                self._param_pending.pop(name, None)
                continue
            rec[1] = tries + 1
            rec[2] = now
            self._write('param_set_send',
                        (self._sysid, self._compid, name.encode('ascii'),
                         value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32), {})

    # -- parametri ---------------------------------------------------------
    def set_param(self, name, value, now=None):
        """PARAM_SET cu confirmare. Valoarea se considera aplicata abia cand
        FC-ul raspunde cu PARAM_VALUE; pana atunci se reincearca (firul I/O,
        sau update_params() in modul sincron)."""
        now = now if now is not None else time.monotonic()
        value = float(value)
        return self._enqueue(('param_set', name, value), urgent=False,
                             now=now)

    def request_param(self, name):
        """Cere o citire; valoarea ajunge in self.params."""
        return self._send('param_request_read_send',
                          self._sysid, self._compid, name.encode('ascii'), -1)

    def update_params(self, now):
        """De apelat din bucla principala: reincearca ce nu s-a confirmat.
        Cu fir I/O e treaba firului; aici nu face nimic."""
        if not self.threaded:
            self._update_params_io(now)

    def param_pending(self, name=None):
        if name is None:
            return bool(self._param_pending)
        return name in self._param_pending

    # -- comenzi -----------------------------------------------------------
    # Toate intorc True daca octetii chiar au plecat (sincron) sau au intrat
    # in coada (cu fir I/O). Cand legatura e cazuta NU arunca si NU trimit:
    # bucla principala trebuie sa continue (H1), iar apelantul trebuie sa
    # poata deosebi "trimis" de "suspendat". Diferenta conteaza pentru
    # SafetySupervisor, care altfel si-ar consuma cele 5 reincercari de mod
    # vorbind cu un port inchis, si ar declara mode_fail fara sa fi emis
    # vreun octet.

    def _enqueue(self, item, urgent=False, periodic=None, now=None):
        if not self.link_healthy:
            return False
        if not self.threaded:
            return bool(self._dispatch(item, now if now is not None
                                       else time.monotonic()))
        if periodic is not None:
            with self._periodic_lock:
                self._periodic[periodic] = item
            return True
        q = self._urgent if urgent else self._tx
        try:
            q.put_nowait(item)
            return True
        except queue.Full:
            self.n_queue_full += 1
            ce = item[1] if len(item) > 1 else item
            log.error("[vehicle] coada %s plina: %s pierdut",
                      'URGENT' if urgent else 'TX', ce)
            if self.link_verbose:
                print(f"!! coada {'URGENT' if urgent else 'TX'} plina: "
                      f"{ce} pierdut")
            return False

    def _send(self, name, *args, urgent=False, periodic=None, counter=None,
              **kwargs):
        return self._enqueue(('send', name, args, kwargs, counter),
                             urgent=urgent, periodic=periodic)

    def request_mode(self, mode, urgent=False):
        """DO_SET_MODE. `urgent=True` e pentru supervizor: coada URGENT se
        goleste inaintea celei obisnuite."""
        self.last_mode_req = mode
        return self._send('command_long_send',
                          self._sysid, self._compid,
                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode, 0, 0, 0, 0, 0, urgent=urgent)

    def send_ekf_source_set(self, n, urgent=False):
        """Comuta setul de surse al EKF3 (15.2.5). `n` e 1..3.

        `GCS_Common.cpp:5133`: comanda accepta 1..3 si scade 1 intern.
        Confirmarea vine prin `COMMAND_ACK`, citit in `_on_ack`; `ekf_src_ack`
        se sterge in momentul trimiterii."""
        if n not in (1, 2, 3):
            raise ValueError(f"setul de surse EKF e 1..3, nu {n}")
        return self._enqueue(('ekf_src', n), urgent=urgent)

    def send_takeoff(self, alt_above_home_m):
        # Copter accepta NAV_TAKEOFF ca COMMAND_LONG; cadrul devine
        # GLOBAL_RELATIVE_ALT, deci param7 e altitudine deasupra HOME.
        return self._send('command_long_send',
                          self._sysid, self._compid,
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                          0, 0, 0, 0, 0, 0, alt_above_home_m)

    def send_statustext(self, severity, text):
        """STATUSTEXT catre GCS / OSD, prin FC. Nu e o comanda de zbor; se
        pierde fara efect daca nu exista telemetrie."""
        if isinstance(text, str):
            text = text[:50].encode('ascii', 'replace')
        return self._send('statustext_send', int(severity), text)

    def send_vision_position_estimate(self, usec, x, y, z, roll, pitch, yaw):
        """VISION_POSITION_ESTIMATE (ExtNav): the vehicle's pose in the
        marker frame, at capture time `usec`. Consumed by AP_VisualOdom
        with VISO_TYPE 1 and by EKF3 with EK3_SRC2_POSXY 6."""
        return self._send('vision_position_estimate_send',
                          int(usec), float(x), float(y), float(z),
                          float(roll), float(pitch), float(yaw),
                          periodic='vpe', counter='n_vpe')

    def send_position_target(self, x, y, z, yaw):
        """GUIDED setpoint: position (NED, metres) + yaw (rad), nothing
        else. Frame LOCAL_NED = the EKF origin - which under ExtNav is the
        marker (§3 of the brief: consigns (0, 0, z))."""
        return self._send('set_position_target_local_ned_send',
                          int(time.monotonic() * 1000) & 0xFFFFFFFF,
                          self._sysid, self._compid,
                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                          POS_TARGET_MASK_POS_YAW,
                          float(x), float(y), float(z), 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, float(yaw), 0.0,
                          periodic='pos_target', counter='n_pos_target')

    def send_landing_target(self, angle_x, angle_y, dist):
        return self._send('landing_target_send',
                          int(time.time() * 1e6), 0,
                          mavutil.mavlink.MAV_FRAME_BODY_FRD,
                          angle_x, angle_y, dist,
                          MARKER_SIZE_M, MARKER_SIZE_M,
                          periodic='landing_target', counter='n_lt')

    def send_distance(self, rng_m):
        cm = int(max(RANGE_MIN_M, min(rng_m, RANGE_MAX_M)) * 100)
        args = (int(time.monotonic() * 1000), 5, 3000, cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 1,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)
        if self.ds_extended:
            return self._send('distance_sensor_send', *args,
                              periodic='distance', counter='n_ds',
                              horizontal_fov=0.0, vertical_fov=0.0,
                              quaternion=[0.0, 0.0, 0.0, 0.0],
                              signal_quality=100)
        return self._send('distance_sensor_send', *args,
                          periodic='distance', counter='n_ds')
