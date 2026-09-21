#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Bucla inchisa in Gazebo: pixeli reali in lantul de control (I4).

    ~/nova-sim-venv/bin/python tools/nova_sim.py
    ~/nova-sim-venv/bin/python tools/nova_sim.py --seconds 60 --csv run.csv

Aceeasi aplicatie ca `tools/nova_pi.py`, cu o singura piesa schimbata: sursa
de cadre e Gazebo in loc de picamera2. Detectorul, masina de stari,
supervizorul si poarta sunt IDENTICE - asta e tot rostul separarii din §3.

DE CE ARE BUCLA PROPRIE

`nova.state_machine.run_loop` foloseste `time.monotonic()`. Aici ceasul
trebuie sa fie cel de SIMULARE (§5.36): la RTF 0.56, o varsta calculata pe
ceasul de perete apare de ~1.8x mai mare decat e, iar `DETECTION_MAX_AGE_S`
(0.5 s) s-ar declansa fals in fiecare coborare.

`run_loop` e validat si nu se atinge (regula rundei). Deci bucla de aici o
oglindeste, pas cu pas, si un test compara ORDINEA celor doua - §5.14 spune
ca doua cablaje care diverg tacut sunt exact felul in care supervizorul a
ajuns inert.

VALIDAREA FATA DE ADEVAR

Gazebo stie unde sunt vehiculul si markerul. La fiecare detectie comparam
(`nova/sim_truth.py`) si strangem:

    eroare de range      prag 3%     (p95)
    eroare unghiulara    prag 0.5 deg (p95)
    rata de detectie     >95% intre 3 si 12 m
    latenta cadru -> LANDING_TARGET   masurata, p50/p99

Asta e verificarea pe care detectorul sintetic nu o putea da: acolo
geometria era corecta prin constructie.
"""

import argparse
import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                               # noqa: E402

from nova import config as nova_config                      # noqa: E402
from nova import sim_truth                                  # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector,          # noqa: E402
                              CameraCalibration,
                              GazeboFrameSource, PiDetector)
from nova.authority import (PROFIL_IMPLICIT,                # noqa: E402
                            PROFIL_RAPID, AuthorityScheduler)
from nova.handover import HandoverGate                      # noqa: E402
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.safety import SafetySupervisor                    # noqa: E402
from nova.state_machine import (LandingStateMachine,        # noqa: E402
                                SequenceConfig)
from nova.vehicle import Vehicle                            # noqa: E402

CSV_HEADER = [
    'sim_t', 'state', 'alt_m', 'detected', 'marker_px',
    'det_range_m', 'truth_range_m', 'range_rel',
    'angle_deg', 'angle_x_deg', 'angle_y_deg',
    'truth_north_off_m', 'truth_east_off_m', 'lat_ms',
]


class SimVehicle(Vehicle):
    """`Vehicle` cu ceasul buclei, nu cu cel de perete.

    `Vehicle.pump()` noteaza ora heartbeat-ului cu `time.monotonic()`, iar
    bucla de aici intreaba cu timp de SIMULARE. Cele doua difera cu ordine
    de marime, deci `time_since_heartbeat(now_sim)` iese **negativa** - si
    orice prag pe ea devine imposibil de atins.

    Consecinta: `_mon_link` din supervizor (H1, "legatura cazuta ->
    planeaza") nu s-ar declansa niciodata in simulare. Nu ar da nicio
    eroare; ar raporta legatura ca sanatoasa la nesfarsit. Exact forma
    §5.14: piesele merg, cablajul nu.

    Reparatia e aditiva, aici, nu in `nova/vehicle.py`: se schimba doar
    ceasul IMPLICIT al metodelor care isi noteaza singure ora.

    Retry-urile si backoff-ul devin astfel numarate in secunde de simulare.
    La RTF 0.56 asta inseamna ~1.8x mai mult timp de perete - corect, pentru
    ca tot ce masoara bucla e timp de simulare.
    """

    def __init__(self, *a, clock=None, **kw):
        super().__init__(*a, **kw)
        self._clock = clock or time.monotonic

    def _note_heartbeat(self, now=None, detail='HEARTBEAT primit'):
        return super()._note_heartbeat(
            self._clock() if now is None else now, detail)

    def set_param(self, name, value, now=None):
        return super().set_param(name, value,
                                 self._clock() if now is None else now)


class SimApp:
    """Cablajul, in aceeasi ordine ca `run_loop`, dar pe ceas de simulare."""

    def __init__(self, args, cfg):
        self.a = args
        self.cfg = cfg
        self.errors = []
        self.det_by_alt = []
        self.latencies = []
        self.rows = []
        self.last_det_t = None
        self.n_det_total = 0
        self._prev_frames = 0
        self.sim_clock_ok = False
        # urmarirea secventei: cat sta in fiecare stare si unde e vehiculul
        # la momentele care conteaza pentru 8.3.3
        self.t_state = {}
        self.prev_state = None
        self.prev_t = None
        self.alt_scoring_m = None
        self.pos_scoring = None
        self.pos_touchdown = None
        self.eroare_finala_m = None
        self.deriva_m = None
        self.tranzitii = []

        cal_path = args.calib or nova_config.resolve(cfg, 'camera_calibration')
        self.cal = CameraCalibration.load(cal_path,
                                          require_real=not args.provisional)
        print(f"[sim] calibrare: {self.cal}")

        aruco = ArucoMarkerDetector(
            self.cal, marker_id=cfg['marker_id'],
            marker_size_m=cfg['marker_size_m'],
            # ROI oprit: introduce dependenta de ordinea cadrelor, iar o
            # rulare de validare trebuie sa fie reproductibila.
            roi_below_m=0.0)
        self.source = GazeboFrameSource(args.topic, clock='sim',
                                        timeout_s=args.frame_timeout)
        self.detector = PiDetector(self.source, aruco, threaded=True).start()

        self.truth = None
        if not args.no_truth:
            try:
                self.truth = sim_truth.SimTruth(world=args.world,
                                                vehicle=args.vehicle_model,
                                                marker=args.marker_model)
            except (ImportError, RuntimeError) as e:
                print(f"[sim] fara adevar din simulare: {e}")

        baud = args.baud if not args.conn.startswith(('udp', 'tcp')) else None
        self.v = SimVehicle(args.conn, baud=baud, clock=self.now).connect()
        self.override = OverrideMonitor(self.v)
        self.sup = SafetySupervisor(self.v, override=self.override)
        # E0 in simulare: ocolita EXPLICIT, ca in fake_detector.py. Vehiculul
        # e Gazebo; config/nova.json ramane sursa de adevar pentru zbor.
        print("[sim] E0: garda de autonomie OCOLITA in simulare "
              "(config/nova.json ramane sursa de adevar pentru zbor)")
        self.gate = HandoverGate(self.v, self.override,
                                 on_reject=self._on_reject,
                                 autonomy_enabled=True)
        self.sm = LandingStateMachine(self.v, SequenceConfig(conv=args.conv),
                                      gate=self.gate)

        # Modularea de autoritate e OPRITA implicit: o campanie de validare
        # a perceptiei nu are voie sa schimbe si parametrii de control in
        # acelasi timp - altfel nu se mai stie ce a schimbat rezultatul.
        # Pornita, ordinea din bucla e cea din run_loop (§5.14).
        self.authority = None
        if args.authority:
            profil = PROFIL_RAPID if args.fast_descent else PROFIL_IMPLICIT
            self.authority = AuthorityScheduler(
                self.v, bands=profil,
                allow_fast_descent=args.fast_descent)
            print(f"[sim] autoritate: profil "
                  f"{'RAPID' if args.fast_descent else 'IMPLICIT'}")

    def _on_reject(self, reason):
        print(f"\n!! HANDOVER REFUZAT: {reason}\n")
        try:
            self.v.m.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                f"NOVA handover refuzat: {reason}"[:50].encode('ascii',
                                                               'replace'))
        except Exception:                                    # noqa: BLE001
            pass

    def now(self):
        """Ceasul buclei: timpul de SIMULARE.

        Pana la primul mesaj nu exista timp de simulare, iar `now_sim` cade
        pe `time.monotonic()` - o cifra de ordinul zecilor de mii. Cand
        sosesc cadrele, ceasul SARE INAPOI la cateva secunde. Tot ce s-a
        calculat pe diferente pana atunci devine absurd: `--seconds` nu s-ar
        mai indeplini niciodata, iar rularea ar atarna pana la omorare.

        Deci se spune deschis cand ceasul a devenit real (`sim_clock_ok`),
        iar bucla isi ia originea de-abia atunci."""
        t = sim_truth.latest_sim_t([self.source, self.truth])
        if t is None:
            self.sim_clock_ok = False
            return time.monotonic()
        self.sim_clock_ok = True
        return t

    def step(self):
        """Un ciclu, in ACEEASI ordine ca nova.state_machine.run_loop."""
        self.v.pump()
        now = self.now()
        if hasattr(self.v, 'check_link'):
            self.v.check_link(now)
        self.v.update_params(now)

        dets = self.detector.poll(now)
        for det in dets:
            self.last_det_t = (det.t if self.last_det_t is None
                               else max(self.last_det_t, det.t))
            self._record(det, now)
        self.n_det_total += len(dets)
        self._note_frames(len(dets))

        age = None if self.last_det_t is None else (now - self.last_det_t)
        self.sup.update(now, age, self.sm.state)

        if self.authority is not None:
            lat = None
            st = getattr(self.detector, 'stats', None)
            if st is not None:
                try:
                    p50 = st().get('latency_p50_ms')
                    lat = None if p50 is None else p50 / 1000.0
                except Exception:                            # noqa: BLE001
                    lat = None
            self.authority.update(now, self.v.alt, self.sm.state,
                                  latency_s=lat)

        for det in dets:
            self.sm.on_detection(det, now)
        self.sm.update(now)
        self._track(now)
        return now

    def _offset_fata_de_marker(self):
        """(nord, est) vehicul - marker, din adevarul simularii. None daca
        nu avem pozitiile: o eroare de aterizare calculata din altceva decat
        adevar ar fi exact cifra pe care nu vrem sa o raportam."""
        if self.truth is None:
            return None
        tr = self.truth.truth()
        if tr is None:
            return None
        return (-tr['north_off_m'], -tr['east_off_m'])

    def _track(self, now):
        """Timp per stare + pozitiile de la SCORING_CAPTURE si contact."""
        st = self.sm.state
        if self.prev_t is not None and self.prev_state is not None:
            self.t_state[self.prev_state] = (
                self.t_state.get(self.prev_state, 0.0) + (now - self.prev_t))
        self.prev_t = now
        if st == self.prev_state:
            return
        self.tranzitii.append((round(now, 3), self.prev_state, st))
        self.prev_state = st
        if st == 'SCORING_CAPTURE':
            self.alt_scoring_m = self.v.alt if self.v.have_pos else None
            self.pos_scoring = self._offset_fata_de_marker()
        elif st == 'TOUCHDOWN_CONFIRM':
            self.pos_touchdown = self._offset_fata_de_marker()
            if self.pos_touchdown is not None:
                n, e = self.pos_touchdown
                self.eroare_finala_m = (n * n + e * e) ** 0.5
                if self.pos_scoring is not None:
                    dn = n - self.pos_scoring[0]
                    de = e - self.pos_scoring[1]
                    self.deriva_m = (dn * dn + de * de) ** 0.5

    def _record(self, det, now):
        n_lt = self.v.n_lt
        err = None
        if self.truth is not None:
            err = self.truth.error_vs(det)
            self.errors.append(err)
        alt = self.v.alt
        self.det_by_alt.append((alt, True))
        # latenta cadru -> LANDING_TARGET: cat trece de la CAPTURA pana cand
        # mesajul chiar pleaca. Ceasul de simulare pentru captura, cel de
        # perete pentru emisie, ar amesteca doua lumi - deci masuram in
        # timp de simulare, care e ce vede controlerul.
        lat_ms = (now - det.t) * 1000.0
        self.latencies.append(lat_ms)
        rand = {
            'sim_t': round(now, 4), 'state': self.sm.state,
            'alt_m': round(alt, 4), 'detected': 1,
            'marker_px': round(det.marker_px, 2),
            'det_range_m': round(det.range_m, 4),
            'lat_ms': round(lat_ms, 3),
        }
        if err:
            rand.update({
                'truth_range_m': round(err['truth_range_m'], 4),
                'range_rel': round(err['range_rel'], 6)
                if err['range_rel'] is not None else '',
                'angle_deg': round(err['angle_deg'], 4),
                'angle_x_deg': round(err['angle_x_deg'], 4),
                'angle_y_deg': round(err['angle_y_deg'], 4),
                'truth_north_off_m': round(err['truth_north_off_m'], 4),
                'truth_east_off_m': round(err['truth_east_off_m'], 4),
            })
        self.rows.append(rand)
        del n_lt

    def _note_frames(self, n_det):
        """Cadrele PROCESATE de la ultimul ciclu, minus cele cu detectie.

        Fara asta, `det_by_alt` primeste numai `True` si rata de detectie
        raporteaza 100% orice s-ar intampla - o metrica ce nu poate esua nu
        e metrica (§5.11). Contorul de cadre e al detectorului, nu al
        buclei: el stie cate a citit din sursa.

        Altitudinea atribuita unei ratari e cea de acum, nu cea de la
        capturarea cadrului ratat. Intre doua cicluri vehiculul face
        centimetri, iar ferestrele de altitudine sunt de metri."""
        nf = getattr(self.detector, 'n_frames', None)
        if nf is None or not self.v.have_pos:
            return
        noi = nf - self._prev_frames
        self._prev_frames = nf
        # max(0, ...): contorul creste in firul detectorului, deci o detectie
        # poate ajunge aici inaintea incrementarii lui.
        for _ in range(max(0, noi - n_det)):
            self.det_by_alt.append((self.v.alt, False))

    def report(self):
        s = sim_truth.summarize(self.errors)
        rata, n_fer = sim_truth.detection_rate(self.det_by_alt)
        lat = sorted(self.latencies)

        def pct(v, p):
            if not v:
                return None
            k = (len(v) - 1) * p / 100.0
            import math as _m
            lo, hi = int(_m.floor(k)), int(_m.ceil(k))
            return v[lo] + (v[hi] - v[lo]) * (k - lo)

        return {
            'n_detectii': s.get('n', 0),
            'n_cadre': self._prev_frames,
            'range_p50': s.get('range_p50'), 'range_p95': s.get('range_p95'),
            'angle_p50': s.get('angle_p50'), 'angle_p95': s.get('angle_p95'),
            'range_ok': s.get('range_ok'), 'angle_ok': s.get('angle_ok'),
            'rata_detectie': rata, 'n_in_fereastra': n_fer,
            'lat_p50_ms': pct(lat, 50), 'lat_p99_ms': pct(lat, 99),
            'stare_finala': self.sm.state,
            'succes': self.sm.state == 'HANDBACK',
            'alt_scoring_m': self.alt_scoring_m,
            'eroare_finala_cm': (None if self.eroare_finala_m is None
                                 else self.eroare_finala_m * 100.0),
            'deriva_cm': (None if self.deriva_m is None
                          else self.deriva_m * 100.0),
            't_state': {k: round(v, 2) for k, v in self.t_state.items()},
            'tranzitii': self.tranzitii,
        }

    def close(self):
        try:
            self.detector.stop()
        except Exception:                                    # noqa: BLE001
            pass
        if self.truth is not None:
            self.truth.close()


def print_report(r, praguri=(0.03, 0.5, 0.95)):
    p_r, p_a, p_d = praguri
    print("\n  --- validare fata de adevarul din simulare ---")
    print(f"    detectii comparate : {r['n_detectii']}")

    def linie(eticheta, p50, p95, prag, unit, ok):
        if p95 is None:
            print(f"    {eticheta:<19}: -")
            return
        semn = 'OK ' if ok else 'PESTE PRAG'
        print(f"    {eticheta:<19}: p50 {p50:.3f}{unit}  p95 {p95:.3f}{unit}"
              f"  (prag {prag:g}{unit})  {semn}")

    linie('eroare de range', (r['range_p50'] or 0) * 100,
          (r['range_p95'] or 0) * 100 if r['range_p95'] is not None else None,
          p_r * 100, '%', r['range_ok'])
    linie('eroare unghiulara', r['angle_p50'], r['angle_p95'], p_a, ' deg',
          r['angle_ok'])
    if r['rata_detectie'] is not None:
        ok = r['rata_detectie'] >= p_d
        print(f"    {'rata detectie 3-12m':<19}: {r['rata_detectie']:.1%}"
              f" din {r['n_in_fereastra']} cadre  (prag {p_d:.0%})  "
              f"{'OK ' if ok else 'SUB PRAG'}")
    if r['lat_p99_ms'] is not None:
        print(f"    {'latenta cadru->LT':<19}: p50 {r['lat_p50_ms']:.1f} ms  "
              f"p99 {r['lat_p99_ms']:.1f} ms  (timp de simulare)")
    print(f"    stare finala       : {r['stare_finala']}")
    if r.get('eroare_finala_cm') is not None:
        print(f"    {'eroare la contact':<19}: "
              f"{r['eroare_finala_cm']:.1f} cm"
              + ('' if r.get('deriva_cm') is None
                 else f"   deriva captura->contact {r['deriva_cm']:.1f} cm"))
    if r.get('alt_scoring_m') is not None:
        print(f"    {'SCORING_CAPTURE la':<19}: {r['alt_scoring_m']:.2f} m")
    if r.get('t_state'):
        parti = '  '.join(f"{k} {v:g}s" for k, v in r['t_state'].items())
        print(f"    timp per stare     : {parti}")


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='udpin:127.0.0.1:14552')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--config', default=None)
    p.add_argument('--calib', default=None)
    p.add_argument('--provisional', action='store_true',
                   help='accepta o calibrare care nu e reala (simulare)')
    p.add_argument('--topic', default='/down_cam/image')
    p.add_argument('--world', default='nova_marker')
    p.add_argument('--vehicle-model', default='iris_with_gimbal')
    p.add_argument('--marker-model', default='aruco_26')
    p.add_argument('--no-truth', action='store_true')
    p.add_argument('--conv', type=int, default=2, choices=[0, 1, 2, 3])
    p.add_argument('--authority', action='store_true',
                   help='modularea de autoritate pe praguri (I5)')
    p.add_argument('--fast-descent', action='store_true',
                   help='PROFIL_RAPID; cere --authority si distanta de '
                        'franare masurata pe fiecare treapta (§6/15.2.9)')
    p.add_argument('--seconds', type=float, default=0.0,
                   help='0 = pana la Ctrl-C; altfel secunde de SIMULARE')
    p.add_argument('--frame-timeout', type=float, default=10.0)
    p.add_argument('--status-s', type=float, default=2.0)
    p.add_argument('--csv', default=None)
    p.add_argument('--json', default=None,
                   help='raportul, pentru tools/batch_sim.py')
    a = p.parse_args(argv)

    if a.fast_descent and not a.authority:
        print("  --fast-descent cere --authority")
        return 1
    cfg = nova_config.load(a.config)
    try:
        app = SimApp(a, cfg)
    except (FileNotFoundError, ValueError) as e:
        print(f"\n[sim] NU PORNESC: {e}\n"
              f"  Pentru simulare: --calib config/camera_sim.yaml "
              f"--provisional\n")
        return 2
    except ImportError as e:
        print(f"\n[sim] NU PORNESC: {e}\n")
        return 2
    except RuntimeError as e:
        print(f"\n[sim] NU PORNESC: {e}\n")
        return 3

    print("[sim] rulez. Ctrl-C pentru oprire.")
    t0 = None
    last_status = 0.0
    perete0 = time.monotonic()
    # Cat asteptam primul cadru inainte sa renuntam. Fara asta, o lume fara
    # camera (sau un server care nu randeaza, §5.33) arata identic cu o
    # rulare care merge: procesul traieste, nu tipareste nimic si nu se
    # termina niciodata. O campanie ar astepta pana la omorare si ar scrie
    # "timeout" in loc de cauza reala.
    rabdare_s = max(30.0, a.frame_timeout * 3.0)
    fara_ceas = False
    try:
        while True:
            now = app.step()
            if t0 is None and time.monotonic() - perete0 > rabdare_s:
                fara_ceas = True
                break
            if t0 is None and app.sim_clock_ok:
                t0 = now
                last_status = now
                print(f"[sim] ceas de simulare: t = {now:.2f} s")
            if a.seconds and t0 and now - t0 >= a.seconds:
                print(f"\n[sim] --seconds {a.seconds:g} atins "
                      f"(timp de simulare)")
                break
            if now - last_status > a.status_s:
                last_status = now
                print(f"  t={now:8.2f}  {app.sm.status_line()} | "
                      f"{app.detector.status_line()}")
            if getattr(app.detector, 'exhausted', False):
                print("\n[sim] sursa de cadre s-a terminat")
                break
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n[sim] oprire.")
    finally:
        app.close()

    if fara_ceas:
        print(f"\n[sim] niciun cadru in {rabdare_s:.0f} s de asteptare.")
        print(f"  Verifica in ordine:")
        print(f"    gz topic -l | grep {a.topic}          # exista topicul?")
        print(f"    gz topic -e -t /down_cam/camera_info -n 1   # SOSESC date?")
        print(f"  Un topic ANUNTAT nu inseamna date (§5.33): fara EGL")
        print(f"  functional, senzorul e declarat si nu randeaza nimic.")

    r = app.report()
    print_report(r)

    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CSV_HEADER, extrasaction='ignore')
            w.writeheader()
            for rand in app.rows:
                w.writerow({k: rand.get(k, '') for k in CSV_HEADER})
        print(f"\n  CSV: {a.csv}  ({len(app.rows)} randuri)")
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(r, f, indent=2, sort_keys=True)
        print(f"  JSON: {a.json}")
    print()
    return 3 if fara_ceas else 0


if __name__ == '__main__':
    sys.exit(main())
