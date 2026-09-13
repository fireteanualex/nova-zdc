#!/usr/bin/env python3
"""
Legatura cu ArduPilot: telemetrie intr-o parte, comenzi in cealalta.

Singura diferenta intre simulare si vehiculul real e sirul de conectare
(sectiunea 8 din CLAUDE.md):

    Vehicle('udpin:127.0.0.1:14552')            # SITL
    Vehicle('/dev/serial0', baud=921600)        # Raspberry Pi

Masina de stari vede doar interfata de aici, deci nu stie pe ce link merge.
"""

import time

from pymavlink import mavutil

from .detection import MARKER_SIZE_M

TELEM_HZ = 20
LANDED_HZ = 50        # EXTENDED_SYS_STATE: semnalul cu care prindem
                      # fereastra de ~0.5 s dinainte de dezarmare (5.6)
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
        self._param_pending = {}     # nume -> [valoare, tries, last_send]

        # statistici de emisie
        self.n_lt = 0
        self.n_ds = 0

        self.ds_extended = True
        self.m = None

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
        if verbose:
            print(f"[vehicle] heartbeat sys={self.m.target_system} "
                  f"comp={self.m.target_component}")
        self._request_streams()
        self._probe_distance_api(verbose)
        return self

    def _request_streams(self):
        rates = [
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
        """Goleste coada de mesaje. De apelat la viteza buclei."""
        while True:
            msg = self.m.recv_match(blocking=False)
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
        self.m.mav.param_set_send(
            self.m.target_system, self.m.target_component,
            name.encode('ascii'), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def request_param(self, name):
        """Cere o citire; valoarea ajunge in self.params prin pump()."""
        self.m.mav.param_request_read_send(
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
    def request_mode(self, mode):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode, 0, 0, 0, 0, 0)

    def send_takeoff(self, alt_above_home_m):
        # Copter accepta NAV_TAKEOFF ca COMMAND_LONG; cadrul devine
        # GLOBAL_RELATIVE_ALT, deci param7 e altitudine deasupra HOME.
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_above_home_m)

    def send_landing_target(self, angle_x, angle_y, dist):
        self.m.mav.landing_target_send(
            int(time.time() * 1e6), 0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x, angle_y, dist,
            MARKER_SIZE_M, MARKER_SIZE_M)
        self.n_lt += 1

    def send_distance(self, rng_m):
        cm = int(max(RANGE_MIN_M, min(rng_m, RANGE_MAX_M)) * 100)
        args = (int(time.monotonic() * 1000), 5, 3000, cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 1,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)
        if self.ds_extended:
            self.m.mav.distance_sensor_send(
                *args, horizontal_fov=0.0, vertical_fov=0.0,
                quaternion=[0.0, 0.0, 0.0, 0.0], signal_quality=100)
        else:
            self.m.mav.distance_sensor_send(*args)
        self.n_ds += 1
