#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Aplicatia de bord (Raspberry Pi): detectorul real + poarta de handover +
Safety Supervisor + masina de stari, in bucla din nova/state_machine.run_loop.

    python3 tools/nova_pi.py                       # /dev/serial0 @ 921600
    python3 tools/nova_pi.py --conn udpin:127.0.0.1:14552   # pe desktop, cu SITL
    python3 tools/nova_pi.py --camera-check        # doar camera + calibrare, 10 s
    python3 tools/nova_pi.py --stop-service        # opreste nova-monitor intai

Acelasi cablaj ca tools/fake_detector.py, cu doua diferente deliberate:

  1. Sursa de Detection e nova/detector_pi.py (picamera2 + ArUco + solvePnP),
     nu geometria sintetica. Masina de stari si supervizorul sunt identice.
  2. NU exista ocolirea gardei E0. Poarta citeste config/nova.json si refuza
     handover-ul cat timp autonomy_enabled e false. Simularea are voie sa o
     ocoleasca (nu are ce distruge); bordul nu.

Refuza sa porneasca fara calibrare reala a camerei (E1.2).

**H2: portul serial.** `nova-monitor.service` si aplicatia asta nu pot
folosi simultan /dev/serial0. Fara verificare, a doua pornire da
`SerialException: ... [Errno 16] Device or resource busy` - un mesaj care nu
spune nici cine ocupa portul, nici ce sa faci. Verificam INAINTE de a-l
deschide (nova/serial_guard.py) si iesim cu comanda de reparare.
`--stop-service` o executa singur, dupa confirmare.

**H3: previzualizarea e OPRITA implicit aici.** Vezi `--show-window`.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                              # noqa: E402

from nova import config as nova_config                     # noqa: E402
from nova.authority import AuthorityScheduler              # noqa: E402
from nova import preview as preview_mod                    # noqa: E402
from nova import race_screen                              # noqa: E402
from nova import serial_guard                             # noqa: E402
from nova.detector_pi import (CameraCalibration, PiCameraSource,  # noqa: E402
                              ArucoMarkerDetector, PiDetector,
                              build_pi_detector, MAX_REPROJ_ERR_PX)
from nova.handover import HandoverGate
from nova.scoring import ScoringRecorder                     # noqa: E402
from nova.rc import OverrideMonitor                        # noqa: E402
from nova.safety import SafetySupervisor                   # noqa: E402
from nova.state_machine import (AUX_CHANNEL,               # noqa: E402
                                LandingStateMachine,
                                SequenceConfig, run_loop)
from nova.vehicle import Vehicle                           # noqa: E402


def banner(cfg):
    flag = cfg.get('autonomy_enabled') is True
    print("=" * 64)
    print(f"  NOVA bord | config: {cfg['_path']}"
          f"{'' if cfg['_exists'] else '  (LIPSA - valori implicite)'}")
    print(f"  E0 autonomy_enabled = {flag}"
          + ("" if flag else "   -> orice handover va fi REFUZAT"))
    print(f"  marker ID {cfg['marker_id']}, {cfg['marker_size_m']} m | "
          f"calibrare {cfg['camera_calibration']}")
    print("=" * 64)


def camera_check(cfg, seconds, show_window=False, preview_scale=0.5,
                 max_rms=None):
    """Camera + calibrare, fara MAVLink. Pentru banc si preflight.

    Asta e singurul loc din aplicatia de bord unde fereastra are sens
    implicit: --camera-check se ruleaza pe banc, nu in cursa. Chiar si aici
    ramane pe fals, ca sa mearga si prin SSH fara X."""
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    calib = CameraCalibration.load(
        cal_path, require_real=True,
        max_rms=MAX_REPROJ_ERR_PX if max_rms is None else float(max_rms))
    print(f"[camera-check] calibrare: {calib}")
    aruco = ArucoMarkerDetector(calib, marker_id=cfg['marker_id'],
                                marker_size_m=cfg['marker_size_m'],
                                roi_below_m=cfg['roi_below_m'],
                                roi_size_px=cfg['roi_size_px'])
    src = PiCameraSource(verbose=True)
    if src.control_problems:
        print("[camera-check] ATENTIE: controale neaplicate, vezi mai sus")
    det = PiDetector(src, aruco, threaded=True).start()
    pv = preview_mod.bench_preview('NOVA camera-check', enabled=show_window,
                                   scale=preview_scale)
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < seconds:
            for d in det.poll(time.monotonic()):
                n += 1
                if n % 10 == 1:
                    print(f"  detectie: {d.distance_m:.2f} m, "
                          f"{d.marker_px:.0f} px, range {d.range_m:.2f} m")
            if pv.enabled:
                cadru = src.read()
                if cadru is not None and not pv.show(cadru[0]):
                    print("  [camera-check] iesire ceruta de la tastatura")
                    break
            time.sleep(0.05)
    finally:
        pv.close()
        det.stop()
    print(f"[camera-check] {det.status_line()}")
    print(f"[camera-check] {n} detectii in {seconds:.0f} s")
    return 0 if det.stats()['fps'] else 1


class LastDetection:
    """Invelis peste detector care retine ultima Detection publicata.

    Deleaga tot restul. Exista doar pentru ecran: `poll()` goleste coada, deci
    fara asta ultima detectie s-ar pierde intre doua redesenari."""

    def __init__(self, inner):
        self._inner = inner
        self.last_detection = None

    def poll(self, now):
        dets = self._inner.poll(now)
        if dets:
            self.last_detection = dets[-1]
        return dets

    def __getattr__(self, name):
        return getattr(self._inner, name)


def run_preflight(a):
    """H4: preflight OBLIGATORIU inainte de modul de concurs.

    Ruleaza exact aceleasi verificari ca `tools/preflight_check.py` - prin
    import, nu reimplementate, ca sa nu poata diverge. Cod diferit de 0
    inseamna ca NU pornim: o verificare sarita nu e o verificare trecuta, si
    o cursa pornita cu preflight-ul picat e o cursa pierduta plus un risc.
    """
    import argparse as _ap
    import preflight_check as pf

    args = _ap.Namespace(
        config=a.config, conn=a.conn, baud=a.baud,
        parm=os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'config', 'nova_flight.parm'),
        frames=30, mavlink_timeout=10.0, no_camera=False,
        no_mavlink=a.no_mavlink, json=False, os_release='/etc/os-release')
    print("\n  [race] preflight...\n")
    rezultate = pf.run_checks(args)
    for r in rezultate:
        print(r)
    picate = [r for r in rezultate if r.status == pf.ESEC]
    sarite = [r for r in rezultate if r.status == pf.SARIT]
    return rezultate, picate, sarite


def race_mode(a, cfg):
    """Modul de concurs: preflight -> RACE_MONITOR -> un singur ecran."""
    rezultate, picate, sarite = run_preflight(a)
    if picate or sarite:
        print("\n" + race_screen.bar(
            race_screen.spaced('NU PORNESC'), 'rosu'))
        for r in picate:
            print(f"  PICAT  {r.name}: {r.detail}")
        for r in sarite:
            print(f"  SARIT  {r.name}: {r.detail} "
                  f"(o verificare sarita nu e trecuta)")
        print()
        return 4
    print("\n  [race] preflight trecut integral.\n")
    return None


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='/dev/serial0')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--config', default=None, help='implicit config/nova.json')
    p.add_argument('--camera-check', action='store_true',
                   help='doar camera si calibrarea, fara MAVLink')
    p.add_argument('--seconds', type=float, default=10.0,
                   help='durata pentru --camera-check')
    p.add_argument('--conv', type=int, default=2, choices=[0, 1, 2, 3],
                   help='conventia LANDING_TARGET (5.1); 2 validat in SITL')
    p.add_argument('--stop-service', action='store_true',
                   help=f'opreste {serial_guard.SERVICE_NAME} daca ocupa '
                        f'portul (cere confirmare)')
    p.add_argument('--yes', action='store_true',
                   help='raspunde da la confirmari (pentru scripturi)')
    # H3: fereastra e OPRITA implicit pe bord. Fara vc4-kms-v3d nu exista
    # accelerare grafica, deci fiecare imshow consuma CPU direct din bugetul
    # detectiei. Se aprinde explicit, si logul o spune.
    p.add_argument('--show-window', action='store_true',
                   help='previzualizare pe ecran (DEGRADEAZA performanta)')
    p.add_argument('--race', action='store_true',
                   help='mod de concurs: preflight obligatoriu + ecran unic')
    p.add_argument('--no-mavlink', action='store_true',
                   help='preflight fara FC (banc); NU trece in --race')
    p.add_argument('--screen-hz', type=float, default=4.0,
                   help='cat de des se redeseneaza ecranul de concurs')
    p.add_argument('--no-authority', action='store_true',
                   help='fara modulare de autoritate (reglaj nominal)')
    p.add_argument('--fast-descent', action='store_true',
                   help='permite viteze peste 0.5 m/s; cere intai '
                        'masuratoarea de distanta de franare (§6/15.2.9)')
    p.add_argument('--ring-frames', type=int, default=30,
                   help='cate cadre tine ringul pentru 8.3.3. Pe Pi un cadru '
                        'e ~3 MB, deci 30 inseamna ~90 MB. 0 = oprit, si '
                        'atunci NU se produce imaginea predata juriului')
    p.add_argument('--scoring-dir', default='data/scoring',
                   help='unde se scriu imaginea de scoring, cea de contact '
                        'si evidenta lor (6.2.1.30)')
    p.add_argument('--aux-channel', type=int, default=AUX_CHANNEL,
                   metavar='N',
                   help=f'canalul RC pe care pilotul CERE segmentul autonom, '
                        f'pe frontul crescator (implicit {AUX_CHANNEL}). '
                        f'Modul LAND NU declanseaza nimic - intrarea e doar '
                        f'prin canalul asta')
    p.add_argument('--no-ascent', action='store_true',
                   help='opreste urcarea de dupa contact (15.2.7). Secventa '
                        'se incheie pe sol. Pentru primele coborari de test, '
                        'unde o urcare automata dupa touchdown e o surpriza')
    p.add_argument('--max-rms', type=float, default=None,
                   help='ridica pragul de reproiectie al calibrarii DOAR '
                        'pentru rularea asta. Pentru bring-up la banc cu o '
                        'calibrare provizorie; se anunta zgomotos. Pragul de '
                        'zbor din cod ramane neatins (§5.34)')
    p.add_argument('--fullscreen', action='store_true',
                   help='fereastra pe tot ecranul (implica --show-window). '
                        'Pentru bring-up la sol; iesire pe q sau Escape')
    p.add_argument('--preview-scale', type=float, default=0.5,
                   help='scara ferestrei; detectia ruleaza pe cadrul plin')
    a = p.parse_args()

    cfg = nova_config.load(a.config)
    banner(cfg)

    # H2: portul INAINTE de orice altceva. Daca e ocupat, aflam acum, nu
    # dupa ce am pornit camera si am asteptat calibrarea.
    def confirma(serviciu):
        if a.yes:
            return True
        raspuns = input(f"  Opresc {serviciu}? [d/N] ").strip().lower()
        return raspuns in ('d', 'da', 'y', 'yes')

    try:
        serial_guard.ensure_port_free(a.conn, stop_service_ok=a.stop_service,
                                      confirm=confirma)
    except serial_guard.PortBusy as e:
        print(f"\n[bord] EROARE: {e}\n")
        return 3
    except (KeyboardInterrupt, EOFError):
        print("\n[bord] anulat.\n")
        return 3

    # H4: preflight INAINTE de a construi detectorul.
    #
    # Prima varianta il rula DUPA, cu motivarea ca "asa un esec de camera se
    # vede in preflight, nu ca exceptie". Gresit, si greseala se vedea doar
    # pe Pi: detectorul tine deja camera, iar `check_camera` incearca sa
    # deschida a doua oara acelasi senzor. Pe desktop nu se manifesta (nu
    # exista picamera2), deci ar fi ajuns pe teren. Preflight-ul e oricum
    # scris ca sa deschida SI sa inchida singur camera - exact ca sa poata
    # rula primul.
    if a.race:
        rc = race_mode(a, cfg)
        if rc is not None:
            return rc

    try:
        if a.camera_check:
            return camera_check(cfg, a.seconds, a.show_window,
                                a.preview_scale, max_rms=a.max_rms)
        # Detectorul intai: daca lipseste calibrarea, ne oprim inainte sa
        # deschidem legatura cu FC-ul.
        detector = build_pi_detector(cfg, verbose=True,
                                     ring_frames=a.ring_frames,
                                     max_rms=a.max_rms)
    except (FileNotFoundError, ValueError) as e:
        # Refuz DELIBERAT (E1.2), nu crash: mesaj scurt, cod de iesire
        # distinct, ca serviciul/preflight-ul sa il poata deosebi de o
        # eroare de program.
        print(f"\n[bord] NU PORNESC: {e}\n")
        return 2
    except ModuleNotFoundError as e:
        print(f"\n[bord] NU PORNESC: {e} - picamera2 exista doar pe Pi.\n")
        return 2

    baud = a.baud if not a.conn.startswith(('udp', 'tcp')) else None
    vehicle = Vehicle(a.conn, baud=baud).connect()

    override = OverrideMonitor(vehicle)
    sup = SafetySupervisor(vehicle, override=override)
    # Modularea de autoritate pe praguri de altitudine. `None` o dezactiveaza
    # complet: fara ea, vehiculul zboara cu reglajul lui nominal, ceea ce e
    # exact comportamentul de dinainte.
    autoritate = (None if a.no_authority
                  else AuthorityScheduler(
                      vehicle, allow_fast_descent=a.fast_descent))

    def signal_reject(reason):
        if ecran is not None:
            ecran.note_handover(False, reason, time.time())
        print(f"\n!! HANDOVER REFUZAT: {reason}\n")
        try:
            vehicle.m.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                f"NOVA handover refuzat: {reason}"[:50].encode('ascii',
                                                               'replace'))
        except Exception:                                   # noqa: BLE001
            pass

    # Fara autonomy_enabled= aici: poarta citeste config/nova.json (E0).
    gate = HandoverGate(vehicle, override, on_reject=signal_reject)
    # Ecranul are nevoie de ultima detectie (px si varsta). O ia dintr-un
    # invelis peste detector, nu dintr-o modificare in run_loop: bucla e
    # validata si nu vrem sa o atingem pentru afisare.
    detector = LastDetection(detector)

    # 8.3.3: imaginea predata juriului. Cadrele se scot din ringul
    # detectorului dupa timestamp-ul CAPTURII purtat de eveniment, nu dupa
    # cel al deciziei si nici dupa "ultimul cadru de acum" - intre ele sunt
    # zeci de milisecunde de coborare (§5.55).
    # LastDetection deleaga prin __getattr__, deci ringul detectorului
    # se vede direct prin invelis.
    ring = getattr(detector, 'ring', None)
    rec = ScoringRecorder(a.scoring_dir, ring, vehicle=vehicle)
    if ring is None:
        print("[bord] ATENTIE: detectorul nu are ring buffer; 8.3.3 NU va "
              "avea imagine. Vezi --ring-frames.")
    seq = SequenceConfig(conv=a.conv, do_ascent=not a.no_ascent,
                         aux_channel=a.aux_channel)
    if a.aux_channel != AUX_CHANNEL:
        print(f"[bord] handover pe canalul RC {a.aux_channel} "
              f"(implicit {AUX_CHANNEL})")
    if a.no_ascent:
        # 15.2.7 oprit: secventa se incheie pe sol, fara NAV_TAKEOFF. Pentru
        # PRIMA coborare autonoma pe un vehicul real asta e ce vrei - o
        # urcare automata imediat dupa contact e exact genul de surpriza
        # care te face sa tragi de manse. ArduPilot dezarmeaza singur din
        # LAND dupa contact (§5.6).
        print("[bord] 15.2.7 OPRIT (--no-ascent): secventa se incheie pe "
              "sol, fara urcare la 5 m")
    sm = LandingStateMachine(vehicle, seq, gate=gate, on_event=rec.on_event)

    ecran = race_screen.RaceScreen() if a.race else None

    def status(now):
        if ecran is None:
            print(f"{sm.status_line()} | {detector.status_line()} | "
                  f"{sup.status()}"
                  + ('' if autoritate is None else f" | {autoritate.status()}"))
            return
        # Verdictul de handover se CITESTE din poarta la fiecare redesenare,
        # nu se tine intr-o variabila proprie actualizata prin callback. Un
        # ecran care isi tine propria copie a starii poate ramane in urma,
        # si exact asta nu are voie sa se intample cu un refuz de handover.
        if gate.decided is not None:
            ecran.note_handover(bool(gate.decided), gate.reason, now)
        snap = race_screen.snapshot(cfg, sm=sm, detector=detector,
                                    vehicle=vehicle, preflight_ok=True,
                                    last_det=detector.last_detection, now=now)
        ecran.draw(snap)

    pv = preview_mod.onboard_preview('NOVA bord',
                                     enabled=a.show_window or a.fullscreen,
                                     scale=a.preview_scale,
                                     fullscreen=a.fullscreen)
    if pv.enabled:
        # Fereastra pe bord nu e interzisa, dar nu are ce cauta intr-o cursa.
        # E pusa aici pentru depanare la sol, cu avertismentul de rigoare.
        print("[bord] ATENTIE: fereastra pornita - vezi avertismentul de mai sus")
    print("[bord] rulez. Ctrl-C pentru oprire.")
    try:
        run_loop(vehicle, detector, sm, supervisor=sup, on_status=status,
                 authority=autoritate)
    except KeyboardInterrupt:
        print("\n[bord] oprire.")
    finally:
        pv.close()
        detector.stop()
        r = rec.raport()
        if r['salvate']:
            print(f"[bord] 8.3.3: {', '.join(sorted(r['salvate'].values()))}")
        if r['lipsa']:
            print(f"[bord] 8.3.3 INCOMPLET: lipsesc {', '.join(r['lipsa'])}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
