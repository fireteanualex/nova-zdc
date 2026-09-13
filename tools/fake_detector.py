#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Detector sintetic de marker ArUco (Faza 1) + harnasamentul de simulare.

Simuleaza ce ar "vedea" camera si PUBLICA doar detectii: offset unghiular,
distanta, dimensiunea markerului in pixeli, range si timestamp. Nu trimite
MAVLink si nu comanda nimic.

Tot ce inseamna control - LANDING_TARGET, DISTANCE_SENSOR, comutarea in
GUIDED, pauza pe sol, urcarea la 5 m - e in nova/state_machine.py si nu se
schimba cand acest fisier e inlocuit, pe Raspberry Pi, de detectorul ArUco
real. Pentru detectorul real, singurele obligatii sunt:

    poll(now) -> iterabil de nova.detection.Detection

si respectarea conventiei de axe documentate in nova/detection.py.

Ce ramane specific simularii:
  - geometria din pozitia CUNOSCUTA a markerului (aici citim si pozitia
    vehiculului din telemetrie; detectorul real nu are nevoie de ea)
  - zgomot, dropout, latenta artificiala
  - raportarea de la final, care compara cu adevarul din simulare

Atentie: detectorul sintetic e optimist. Nu modeleaza motion blur, detectii
false, variatii de expunere sau marker ocluzat.

Utilizare:
    # Terminal A
    gz sim -v4 -r iris_runway.sdf
    # Terminal B
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
        --console --map --out=udp:127.0.0.1:14552
    # Terminal C  (cu nova-venv activat)
    python3 fake_detector.py --north 2.0 --east 1.5

    apoi in MAVProxy:  mode guided / arm throttle / takeoff 10 / mode land
    Dupa contact secventa continua singura; NU da comenzi manual.
"""

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                              # noqa: E402

from nova.detection import (CAM_FOOTPRINT_MARGIN_M,        # noqa: E402
                            CameraModel, Detection)
from nova.handover import HandoverGate                     # noqa: E402
from nova.rc import OverrideMonitor                        # noqa: E402
from nova.safety import SafetySupervisor                   # noqa: E402
from nova.state_machine import (LandingStateMachine,       # noqa: E402
                                SequenceConfig, State, run_loop)
from nova.vehicle import Vehicle                           # noqa: E402

DETECT_HZ = 20.0


class FakeDetector:
    """Detector sintetic. Publica Detection, nimic altceva.

    Primeste obiectul Vehicle doar ca sa citeasca pozitia si atitudinea -
    e singurul mod de a sti ce "vede" camera intr-o simulare fara imagini.
    Detectorul ArUco real nu are nevoie de asa ceva: el are cadre.
    """

    def __init__(self, vehicle, args, cam=None):
        self.v = vehicle
        self.cam = cam or CameraModel(focal_px=args.focal_px)
        self.marker_n = args.north
        self.marker_e = args.east
        self.noise_px = args.noise_px
        self.dropout = args.dropout
        self.latency = args.latency_ms / 1000.0

        self.period = 1.0 / DETECT_HZ
        # Ceasul se ia din primul poll(now), nu din time.monotonic() aici:
        # altfel detectorul nu poate fi rulat in timp accelerat, cum e rulata
        # masina de stari in tools/test_state_machine.py.
        self.next_tick = None
        self.pending = []        # coada pentru simularea latentei

        self.n_lost_fov = 0
        self.n_dropout = 0

    # -- interfata detectorului -------------------------------------------
    def poll(self, now):
        """Zero sau mai multe detectii noi. Aceeasi semnatura o va avea si
        detectorul ArUco real."""
        if self.next_tick is None:
            self.next_tick = now
        if now >= self.next_tick:
            self.next_tick += self.period
            det = self._observe(now)
            if det is not None:
                self.pending.append((now + self.latency, det))

        out = []
        while self.pending and self.pending[0][0] <= now:
            out.append(self.pending.pop(0)[1])
        return out

    # -- geometrie ---------------------------------------------------------
    def _observe(self, now):
        """Ce ar extrage solvePnP din cadrul de acum, sau None."""
        if not self.v.have_pos:
            return None

        alt = self.v.alt
        if alt < 0.02:
            return None

        d_n = self.marker_n - self.v.x
        d_e = self.marker_e - self.v.y

        c, s = math.cos(self.v.yaw), math.sin(self.v.yaw)
        fwd = d_n * c + d_e * s
        right = -d_n * s + d_e * c

        angle_x = math.atan2(fwd, alt)
        angle_y = math.atan2(right, alt)

        if not self.cam.in_fov(angle_x, angle_y):
            self.n_lost_fov += 1
            return None

        dist_3d = math.sqrt(fwd * fwd + right * right + alt * alt)
        marker_px = self.cam.marker_px_at(dist_3d)
        if not self.cam.fits_in_frame(marker_px):
            self.n_lost_fov += 1
            return None

        # ArduPilot aplica singur corectia de inclinare (inmulteste cu
        # cos(tilt)), deci raportam valoarea NEcorectata: alt / cos(tilt).
        tilt = math.cos(self.v.roll) * math.cos(self.v.pitch)
        raw_range = alt / max(tilt, 0.3)

        if random.random() < self.dropout:
            self.n_dropout += 1
            return None

        angle_x, angle_y, dist_3d, raw_range = self._add_noise(
            angle_x, angle_y, dist_3d, raw_range, marker_px)

        return Detection(t=now, angle_x=angle_x, angle_y=angle_y,
                         distance_m=dist_3d, marker_px=marker_px,
                         range_m=raw_range)

    def _add_noise(self, angle_x, angle_y, dist, rng, marker_px):
        sigma_ang = self.noise_px / self.cam.focal_px
        angle_x += random.gauss(0.0, sigma_ang)
        angle_y += random.gauss(0.0, sigma_ang)

        rel_err = self.noise_px / max(marker_px, 1.0)
        scale = 1.0 + random.gauss(0.0, rel_err)
        return angle_x, angle_y, dist * scale, rng * scale

    # -- doar simulare -----------------------------------------------------
    def truth_error(self):
        """Eroarea reala fata de marker. Exista numai in simulare."""
        return (self.marker_n - self.v.x, self.marker_e - self.v.y)


class SimReport:
    """Raportarea de la final. Consuma evenimentele masinii de stari si le
    completeaza cu adevarul din simulare. Pe vehiculul real, locul ei il ia
    logul .bin/.tlog (6.2.1.30)."""

    def __init__(self, vehicle, detector, supervisor=None):
        self.v = vehicle
        self.det = detector
        self.sup = supervisor
        self.sm = None               # completat in main()
        self.scoring = None          # (alt, marker_px, eroare_reala)
        self.reported = False

    def on_event(self, name, info):
        if name == 'state':
            # Supervizorul se armeaza si se dezarmeaza SINGUR, din faza pe
            # care o primeste in update(). Aplicatia nu mai trebuie sa
            # ghiceasca tranzitia corecta - varianta anterioara arma pe
            # IDLE -> DESCEND_TRACK si a incetat sa mai functioneze tacut
            # cand intrarea a devenit IDLE -> HANDOVER_CHECK -> ACQUIRE ->
            # DESCEND_TRACK.
            if info['new'] == State.IDLE:
                # secventa s-a reluat (dezarmare, preluare de pilot)
                self.reset()
        elif name == 'scoring_capture':
            err = math.hypot(*self.det.truth_error())
            self.scoring = (info['alt'], info['marker_px'], err)
        elif name == 'touchdown':
            self.report_landing()
        elif name == 'ascent_done':
            self.report_ascent(info)
        elif name == 'disarm_early':
            if not self.reported:
                self.report_landing()

    def reset(self):
        self.scoring = None
        self.reported = False

    def status(self, now):
        err = math.hypot(*self.det.truth_error())
        extra = f" | {self.sup.status()}" if self.sup is not None else ''
        print(f"{self.sm.status_line()} | eroare {err * 100:6.1f} cm | "
              f"FOV- {self.det.n_lost_fov} drop {self.det.n_dropout}{extra}")

    def report_landing(self):
        self.reported = True
        err_n, err_e = self.det.truth_error()
        err = math.hypot(err_n, err_e)

        print("\n" + "=" * 60)
        # Eroarea e metrica interna, nu punctaj: 8.3.2 e binar (secventa
        # completa = 10 puncte), iar 8.3.3 cere doar ca centrul imaginii de
        # touchdown sa cada pe marker - satisfacut sub ~19 cm, la 74.5 mm.
        print(f"ATERIZARE   eroare = {err * 100:.1f} cm "
              f"(N {err_n * 100:+.1f}, E {err_e * 100:+.1f})")
        if err > CAM_FOOTPRINT_MARGIN_M:
            print(f"ATENTIE: peste {CAM_FOOTPRINT_MARGIN_M * 100:.0f} cm "
                  f"centrul imaginii poate cadea in afara markerului (8.3.3)")
        if self.scoring:
            alt_s, px_s, err_s = self.scoring
            drift = abs(err - err_s) * 100
            print(f"Captura de scoring la {alt_s:.3f} m ({px_s:.0f} px), "
                  f"eroare atunci {err_s * 100:.1f} cm, "
                  f"deriva pana la contact {drift:.1f} cm")
        else:
            print("ATENTIE: nu s-a declansat SCORING_CAPTURE")
        print(f"LANDING_TARGET {self.v.n_lt} | DISTANCE_SENSOR {self.v.n_ds} | "
              f"in afara FOV {self.det.n_lost_fov} | "
              f"dropout {self.det.n_dropout}")
        print("=" * 60 + "\n")

    def report_ascent(self, info):
        err = math.hypot(*self.det.truth_error())
        agl = info['agl']
        print("\n" + "=" * 60)
        print(f"15.2.7 INDEPLINIT   {agl:.2f} m deasupra markerului, "
              f"fara re-armare")
        print(f"Contact -> 5 m: {info['contact_to_agl_s']:.2f} s "
              f"(din care pauza {info['hold_s']:.2f} s)")
        print(f"Deriva laterala la {agl:.2f} m: {err * 100:.1f} cm")
        print("HANDBACK: comuta pe modul pilotului cand esti gata.")
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
    p.add_argument('--conv', type=int, default=0, choices=[0, 1, 2, 3],
                   help='conventie axe LANDING_TARGET (incearca 0..3)')
    # --- 15.2.7 ---
    p.add_argument('--no-ascent', action='store_true',
                   help='fara secventa de dupa contact (comportamentul Fazei 1)')
    p.add_argument('--ascent-alt', type=float, default=5.0,
                   help='inaltime ceruta deasupra markerului, m (15.2.7)')
    p.add_argument('--ascent-margin', type=float, default=0.3,
                   help='marja peste tinta la comanda de takeoff, m')
    p.add_argument('--hold-s', type=float, default=1.2,
                   help='pauza pe sol dupa contact, s (regulament: >= 1.0)')
    p.add_argument('--touchdown-alt', type=float, default=0.20,
                   help='prag de altitudine pentru contact, m')
    p.add_argument('--touchdown-vz', type=float, default=0.12,
                   help='viteza verticala reziduala acceptata la contact, m/s')
    # --- 15.2.9 / 15.2.10 ---
    p.add_argument('--no-safety', action='store_true',
                   help='fara Safety Supervisor (doar pentru comparatii)')
    p.add_argument('--detection-max-age', type=float, default=None,
                   help='varsta maxima a detectiei inainte de BRAKE, s')
    args = p.parse_args()

    cfg = SequenceConfig(
        do_ascent=not args.no_ascent,
        ascent_m=args.ascent_alt,
        ascent_margin=args.ascent_margin,
        hold_s=args.hold_s,
        touchdown_alt=args.touchdown_alt,
        touchdown_vz=args.touchdown_vz,
        conv=args.conv,
        send_range=not args.no_range,
    )

    vehicle = Vehicle(args.conn).connect()
    detector = FakeDetector(vehicle, args)

    # Un singur OverrideMonitor, folosit si de poarta (care memoreaza
    # neutrul la ACCEPT) si de supervizor (care il compara dupa aceea).
    override = OverrideMonitor(vehicle)

    sup = None
    if not args.no_safety:
        kw = {}
        if args.detection_max_age is not None:
            kw['detection_max_age_s'] = args.detection_max_age
        sup = SafetySupervisor(vehicle, override=override, **kw)
        print(f"[detector] Safety Supervisor activ: detectie mai veche de "
              f"{sup.detection_max_age_s:.2f} s -> BRAKE")
    else:
        print("[detector] Safety Supervisor DEZACTIVAT (--no-safety)")

    report = SimReport(vehicle, detector, sup)

    def signal_reject(reason):
        # Pe vehiculul real: buzzer/LED. Aici, si in telemetrie, ca pilotul
        # sa vada motivul - un handover refuzat costa secunde, o incercare
        # anulata costa 10 puncte.
        print(f"\n!! HANDOVER REFUZAT: {reason}\n")
        try:
            vehicle.m.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                f"NOVA handover refuzat: {reason}"[:50].encode('ascii',
                                                               'replace'))
        except Exception:                                   # noqa: BLE001
            pass

    # E0 in simulare: garda exista ca sa protejeze un vehicul real de un
    # detector nevalidat. Aici vehiculul e Gazebo si detectorul e sintetic,
    # deci o ocolim EXPLICIT - nu prin editarea config/nova.json, care
    # ramane pe false pana la E2. Pe Pi (nova/detector_pi.py) nu exista
    # aceasta optiune: acolo poarta citeste doar fisierul.
    print("[detector] E0: garda de autonomie OCOLITA in simulare "
          "(config/nova.json ramane sursa de adevar pentru zbor)")
    gate = HandoverGate(vehicle, override, on_reject=signal_reject,
                        autonomy_enabled=True)
    sm = LandingStateMachine(vehicle, cfg, on_event=report.on_event, gate=gate)
    report.sm = sm
    print(f"[detector] handover pe AUX canal {cfg.aux_channel} "
          f"(>= {cfg.aux_high_pwm} PWM); nu exista cale alternativa")

    if cfg.do_ascent:
        print(f"[detector] 15.2.7 activ: pauza {cfg.hold_s:.1f} s pe sol, "
              f"apoi urcare la {cfg.ascent_m:.1f} m peste marker")
    else:
        print("[detector] 15.2.7 dezactivat (--no-ascent): LAND dezarmeaza")

    print("[detector] rulez. Ctrl-C pentru oprire.")
    try:
        run_loop(vehicle, detector, sm, supervisor=sup,
                 on_status=report.status)
    except KeyboardInterrupt:
        print("\n[detector] oprit.")


if __name__ == '__main__':
    main()
