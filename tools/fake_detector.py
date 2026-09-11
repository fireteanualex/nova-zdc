#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Faza 1.2/1.3 - detector sintetic de marker ArUco.

Simuleaza ce ar "vedea" camera si trimite catre ArduPilot:
  - LANDING_TARGET   (offset unghiular spre marker)
  - DISTANCE_SENSOR  (range derivat din dimensiunea cunoscuta a markerului)

Ambele provin din ACEEASI detectie, exact ca pe vehiculul real, unde
camera este si senzor de pozitie laterala si telemetru.

Parametri ArduPilot necesari (reboot dupa RNGFND1_TYPE):
    param set RNGFND1_TYPE 10
    param set RNGFND1_ORIENT 25
    param set RNGFND1_MIN 0.05
    param set RNGFND1_MAX 30
    param set RNGFND1_GNDCLR 0.0745
    param set PLND_ENABLED 1
    param set PLND_TYPE 1
    param set PLND_EST_TYPE 1
    param set PLND_STRICT 2
    param set PLND_ALT_MIN 0.35
    param set ARMING_CHECK 0        # doar pentru dezvoltare in SITL

Utilizare:
    # Terminal A
    gz sim -v4 -r iris_runway.sdf
    # Terminal B
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
        --console --map --out=udp:127.0.0.1:14552
    # Terminal C  (cu nova-venv activat)
    python3 fake_detector.py --north 2.0 --east 1.5

    apoi in MAVProxy:  mode guided / arm throttle / takeoff 10 / mode land
"""

import argparse
import math
import random
import time

from pymavlink import mavutil

# --- Model de camera: Raspberry Pi Camera Module 3 Wide -------------------
HFOV_DEG = 102.0          # camp vizual orizontal (axa Y corp / dreapta)
VFOV_DEG = 67.0           # camp vizual vertical  (axa X corp / inainte)
MARKER_SIZE_M = 0.48      # latura zonei codate
CAM_HEIGHT_M = 0.0745     # inaltimea camerei deasupra solului la contact

SEND_HZ = 20.0
TELEM_HZ = 20


class FakeDetector:

    def __init__(self, args):
        self.marker_n = args.north
        self.marker_e = args.east
        self.noise_px = args.noise_px
        self.dropout = args.dropout
        self.latency = args.latency_ms / 1000.0
        self.focal_px = args.focal_px
        self.ground_assist = args.ground_assist
        self.send_range = not args.no_range
        self.conv = args.conv        

        # stare vehicul
        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = False
        self.armed = False

        # coada pentru simularea latentei
        self.pending = []

        # statistici
        self.n_lt = 0
        self.n_ds = 0
        self.n_lost_fov = 0
        self.n_dropout = 0
        self.scoring_shot = None

        print(f"[detector] conectare la {args.conn} ...")
        self.m = mavutil.mavlink_connection(args.conn)
        self.m.wait_heartbeat()
        print(f"[detector] heartbeat sys={self.m.target_system} "
              f"comp={self.m.target_component}")
        self._request_streams()
        self._probe_distance_api()

    # -- initializare ------------------------------------------------------
    def _request_streams(self):
        interval_us = int(1e6 / TELEM_HZ)
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0)

    def _probe_distance_api(self):
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
            print("[detector] pymavlink vechi: fara signal_quality")

    # -- telemetrie --------------------------------------------------------
    def _pump(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == 'LOCAL_POSITION_NED':
                self.x, self.y, self.z = msg.x, msg.y, msg.z
                self.have_pos = True
            elif t == 'ATTITUDE':
                self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            elif t == 'HEARTBEAT':
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    # -- geometrie ---------------------------------------------------------
    def compute_detection(self):
        """
        (angle_x, angle_y, dist_3d, marker_px, raw_range) sau None.

        angle_x : unghi spre marker pe axa INAINTE a corpului (rad)
        angle_y : unghi spre marker pe axa DREAPTA a corpului (rad)
        raw_range : ce ar citi un telemetru orientat pe axa optica.
            ArduPilot aplica singur corectia de inclinare (inmulteste cu
            cos(tilt)), deci trimitem valoarea NEcorectata: alt / cos(tilt).
        """
        alt = -self.z
        if alt < 0.02:
            return None

        d_n = self.marker_n - self.x
        d_e = self.marker_e - self.y

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        fwd = d_n * c + d_e * s
        right = -d_n * s + d_e * c

        angle_x = math.atan2(fwd, alt)
        angle_y = math.atan2(right, alt)

        if abs(angle_x) > math.radians(VFOV_DEG / 2.0):
            return None
        if abs(angle_y) > math.radians(HFOV_DEG / 2.0):
            return None

        dist_3d = math.sqrt(fwd * fwd + right * right + alt * alt)
        marker_px = self.focal_px * MARKER_SIZE_M / dist_3d        
        # Markerul trebuie sa incapa INTREG in cadru, nu doar centrul lui.
        # Inaltimea cadrului la rezolutia de lucru:
        frame_h_px = 2.0 * self.focal_px * math.tan(math.radians(VFOV_DEG / 2))
        if marker_px > frame_h_px * 0.95:
            return None
        tilt = math.cos(self.roll) * math.cos(self.pitch)
        raw_range = alt / max(tilt, 0.3)

        return angle_x, angle_y, dist_3d, marker_px, raw_range

    def add_noise(self, angle_x, angle_y, dist, rng, marker_px):
        sigma_ang = self.noise_px / self.focal_px
        angle_x += random.gauss(0.0, sigma_ang)
        angle_y += random.gauss(0.0, sigma_ang)

        rel_err = self.noise_px / max(marker_px, 1.0)
        scale = 1.0 + random.gauss(0.0, rel_err)
        return angle_x, angle_y, dist * scale, rng * scale

    # -- emisie ------------------------------------------------------------
    def send_landing_target(self, angle_x, angle_y, dist):
        self.m.mav.landing_target_send(
            int(time.time() * 1e6), 0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x, angle_y, dist,
            MARKER_SIZE_M, MARKER_SIZE_M)
        self.n_lt += 1

    def send_distance(self, rng_m):
        if not self.send_range:
            return
        cm = int(max(0.05, min(rng_m, 30.0)) * 100)
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

    # -- bucla -------------------------------------------------------------
    def run(self):
        period = 1.0 / SEND_HZ
        next_tick = time.monotonic()
        last_report = time.monotonic()
        was_armed = False

        print("[detector] rulez. Ctrl-C pentru oprire.")
        while True:
            self._pump()
            now = time.monotonic()

            if was_armed and not self.armed and self.have_pos:
                self.report_landing()
                self.scoring_shot = None
            was_armed = self.armed

            if now >= next_tick:
                next_tick += period
                if self.have_pos:
                    self.tick(now)

            while self.pending and self.pending[0][0] <= now:
                _, ax, ay, d, rng = self.pending.pop(0)
                self.send_landing_target(ax, ay, d)
                self.send_distance(rng)

            if now - last_report > 2.0:
                last_report = now
                self.print_status()

            time.sleep(0.002)

    def tick(self, now):
        res = self.compute_detection()

        if res is None:
            self.n_lost_fov += 1
            alt = -self.z
            if alt < 1.0:
                # Aproape de sol: raportam altitudinea reala, altfel
                # detectorul de aterizare din ArduPilot nu confirma
                # starea "landed" si refuza urmatorul takeoff.
                self.send_distance(max(alt, 0.10))
            else:
                # In aer fara marker: valoare valida, departe de MIN/MAX.
                self.send_distance(20.0)
            return

        angle_x, angle_y, dist, marker_px, rng = res

        # captura de scoring: declansata pe dimensiunea markerului in pixeli,
        # nu pe altitudine (vezi 8.3.4)
        if self.scoring_shot is None and marker_px > 980:
            self.scoring_shot = (-self.z,
                                 math.hypot(self.marker_n - self.x,
                                            self.marker_e - self.y))
            print(f"  >> SCORING_CAPTURE la {self.scoring_shot[0]:.3f} m "
                  f"({marker_px:.0f} px)")

        if random.random() < self.dropout:
            self.n_dropout += 1
            return

        angle_x, angle_y, dist, rng = self.add_noise(
            angle_x, angle_y, dist, rng, marker_px)
        # Conventia de axe difera intre versiuni ArduPilot. Miscarea in
        # elipsa = corectie perpendiculara pe eroare = rotatie de 90 grade.
        if self.conv == 1:
            angle_x, angle_y = -angle_y, angle_x
        elif self.conv == 2:
            angle_x, angle_y = angle_y, -angle_x
        elif self.conv == 3:
            angle_x, angle_y = -angle_x, -angle_y

        self.pending.append((now + self.latency, angle_x, angle_y, dist, rng))            

    # -- raportare ---------------------------------------------------------
    def print_status(self):
        alt = -self.z
        res = self.compute_detection()
        vis = "MARKER PIERDUT" if res is None else \
              f"marker {res[3]:6.1f} px  range {res[4]:5.2f} m"
        err = math.hypot(self.marker_n - self.x, self.marker_e - self.y)
        print(f"alt {alt:6.2f} m | eroare {err*100:6.1f} cm | {vis} "
              f"| LT {self.n_lt} DS {self.n_ds}")

    def report_landing(self):
        err_n = self.marker_n - self.x
        err_e = self.marker_e - self.y
        err = math.hypot(err_n, err_e)
        pts = 5 if err < 0.10 else 3 if err < 0.25 else 1 if err < 0.50 else 0

        print("\n" + "=" * 60)
        print(f"ATERIZARE   eroare = {err*100:.1f} cm "
              f"(N {err_n*100:+.1f}, E {err_e*100:+.1f})")
        print(f"Scor estimat 8.3.4: {pts} puncte")
        if self.scoring_shot:
            alt_s, err_s = self.scoring_shot
            drift = abs(err - err_s) * 100
            print(f"Captura de scoring la {alt_s:.3f} m, "
                  f"eroare atunci {err_s*100:.1f} cm, "
                  f"deriva pana la contact {drift:.1f} cm")
        else:
            print("ATENTIE: nu s-a declansat SCORING_CAPTURE")
        print(f"LANDING_TARGET {self.n_lt} | DISTANCE_SENSOR {self.n_ds} | "
              f"in afara FOV {self.n_lost_fov} | dropout {self.n_dropout}")
        print("=" * 60 + "\n")


def main():
    p = argparse.ArgumentParser(description="Detector sintetic NOVA")
    p.add_argument('--conn', default='udpin:127.0.0.1:14552')
    p.add_argument('--north', type=float, default=2.0)
    p.add_argument('--east', type=float, default=1.5)
    p.add_argument('--noise-px', type=float, default=0.5)
    p.add_argument('--dropout', type=float, default=0.0)
    p.add_argument('--latency-ms', type=float, default=0.0)
    p.add_argument('--focal-px', type=float, default=933.0)
    p.add_argument('--no-range', action='store_true',
                   help='nu trimite DISTANCE_SENSOR')
    p.add_argument('--ground-assist', action='store_true', default=True,
                   help='trimite range la sol ca sa treaca pre-arm (SITL)')
    p.add_argument('--conv', type=int, default=0, choices=[0, 1, 2, 3],
                   help='conventie axe LANDING_TARGET (incearca 0..3)')                   
    args = p.parse_args()

    det = FakeDetector(args)
    try:
        det.run()
    except KeyboardInterrupt:
        print("\n[detector] oprit.")


if __name__ == '__main__':
    main()

