#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Aplicatia de bord (Raspberry Pi): detectorul real + poarta de handover +
Safety Supervisor + masina de stari, in bucla din nova/state_machine.run_loop.

    python3 tools/nova_pi.py                       # /dev/serial0 @ 921600
    python3 tools/nova_pi.py --conn udpin:127.0.0.1:14552   # pe desktop, cu SITL
    python3 tools/nova_pi.py --camera-check        # doar camera + calibrare, 10 s
    python3 tools/nova_pi.py --stop-service        # opreste nova-monitor intai

Sursa de Detection e nova/detector_pi.py (picamera2 + ArUco + solvePnP).
Poarta citeste config/nova.json si refuza handover-ul cat timp
autonomy_enabled e false; pe bord garda E0 nu se ocoleste.

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
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                              # noqa: E402

from nova import config as nova_config                     # noqa: E402
from nova.authority import AuthorityScheduler              # noqa: E402
from nova.board_window import FereastraBord, OsdMesaje    # noqa: E402
from nova.touchdown_capture import TouchdownCapture        # noqa: E402
from nova.concurrency import (Heartbeat, Latest,           # noqa: E402
                              install_excepthook)
from nova.supervisor_thread import SupervisorThread        # noqa: E402
from nova.ekf_source import EkfSourceManager               # noqa: E402
from nova.extnav import ExtNavEstimator                    # noqa: E402
from nova.extnav_landing import (ExtNavConfig, ExtNavLanding,  # noqa: E402
                                 SRC2_PHASES)
from nova import preview as preview_mod                    # noqa: E402
from nova import race_screen                              # noqa: E402
from nova import serial_guard                             # noqa: E402
from nova.detector_pi import (CameraCalibration, PiCameraSource,  # noqa: E402
                              ArucoMarkerDetector, PiDetector,
                              build_pi_detector, MAX_REPROJ_ERR_PX)
from nova.handover import HandoverGate
from nova.scoring import ScoringRecorder                     # noqa: E402
from nova.rc import OverrideMonitor                        # noqa: E402
from nova.safety import ExtNavSupervisor, SafetySupervisor  # noqa: E402
from nova.state_machine import (AUX_CHANNEL, AUX_HIGH_PWM,  # noqa: E402
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
    print(f"  ghidare: {nova_config.guidance(cfg).upper()}")
    print(f"  marker ID {cfg['marker_id']}, {cfg['marker_size_m']} m | "
          f"calibrare {cfg['camera_calibration']}")
    print("=" * 64)


def camera_check(cfg, seconds, show_window=False, preview_scale=0.5,
                 max_rms=None, preset=None):
    """Camera + calibrare, fara MAVLink. Pentru banc si preflight.

    Asta e singurul loc din aplicatia de bord unde fereastra are sens
    implicit: --camera-check se ruleaza pe banc, nu in cursa. Chiar si aici
    ramane pe fals, ca sa mearga si prin SSH fara X."""
    # The same construction as the flight (27.09.2026): sensor mode from the
    # config, geometry read back, calibration derived / scaled for it. Its
    # own copy used to open the camera without a mode and scale blindly.
    det = build_pi_detector(cfg, verbose=True, max_rms=max_rms, preset=preset)
    src = det.source
    if src.control_problems:
        print("[camera-check] ATENTIE: controale neaplicate, vezi mai sus")
    pv = preview_mod.bench_preview('NOVA camera-check', enabled=show_window,
                                   scale=preview_scale,
                                   rotate_deg=cfg['camera_rotation_deg'])
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

    Deleaga tot restul. Exista pentru ecran: `poll()` goleste coada, deci
    fara asta ultima detectie s-ar pierde intre doua redesenari. Si pentru
    `on_poll(now)`: singurul carlig per ciclu pe care il avem fara sa
    atingem `run_loop` (validat, comun cu simularea) - B6 il foloseste ca
    ScoringRecorder sa isi rezolve cererea de cadru de contact."""

    def __init__(self, inner, on_poll=None):
        self._inner = inner
        self.last_detection = None
        self.on_poll = on_poll

    def poll(self, now):
        dets = self._inner.poll(now)
        if dets:
            self.last_detection = dets[-1]
        if self.on_poll is not None:
            self.on_poll(now)
        return dets

    def __getattr__(self, name):
        return getattr(self._inner, name)


def frame_log_path(arg, now=None):
    """The per-frame detector log (27.09.2026): the given path, None for
    "none", else a timestamped file next to the other run logs."""
    if arg is not None:
        return None if arg.lower() == 'none' else arg
    d = os.environ.get('NOVA_LOG_DIR') or os.path.expanduser('~/nova-logs')
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now))
    return os.path.join(d, f"cadre-{stamp}.csv")


#: Ring frames on extnav: the touchdown descent from 1 m at 0.4 m/s
#: (2.5 s) + the FC's land detection (1.5 s) + 1 s after + margin, 30 fps.
TOUCHDOWN_RING_FRAMES = 200


def ring_frames_for(a, cfg):
    if a.ring_frames is not None:
        return a.ring_frames
    return TOUCHDOWN_RING_FRAMES if nova_config.guidance(cfg) == 'extnav' else 30


def camera_info(detector):
    """Preset, geometry and calibration kind, for the capture's meta."""
    src = getattr(detector, 'source', None)
    g = getattr(src, 'geometry', None)
    s = getattr(src, 'settings', None)
    cal = getattr(getattr(detector, 'det', None), 'calib', None)
    return {'preset': getattr(s, 'preset', None),
            'sensor_mode': list(g.sensor_mode) if g else None,
            'scaler_crop': list(g.scaler_crop) if g else None,
            'calibration': getattr(cal, 'kind', None)}


class Recordere:
    """The recorders updated once per main-loop iteration (PasPrincipal)."""

    def __init__(self, *recs):
        self.recs = [r for r in recs if r is not None]

    def update(self, now):
        for r in self.recs:
            r.update(now)


def vedere(vehicle):
    """The supervisor's view of the vehicle (phase 3): snapshots, commands
    through the queues. A vehicle without one (tests, stubs) is used as is."""
    fn = getattr(vehicle, 'view', None)
    return fn() if callable(fn) else vehicle


class PasPrincipal:
    """Phase 5: what the main thread publishes / checks once per loop
    iteration, hung on LastDetection.on_poll (the one per-cycle hook we
    have without touching run_loop): the phase for the supervisor thread,
    its own heartbeat, the watchdog over the supervisor, and the scoring
    recorder's pending frame request (B6)."""

    def __init__(self, sm, faza, heartbeat, supervisor_thread=None,
                 recorder=None):
        self.sm = sm
        self.faza = faza
        self.heartbeat = heartbeat
        self.st = supervisor_thread
        self.rec = recorder
        self.n = 0

    def __call__(self, now):
        self.faza.set(self.sm.state, now)
        self.heartbeat.beat(now)
        if self.st is not None:
            self.st.watchdog(now)
        if self.rec is not None:
            self.rec.update(now)
        self.n += 1


def opreste_firele(supervizor, detector, vehicle):
    """Ordered shutdown, the reverse of the start: the state machine has
    already stopped (the main loop returned), then the supervisor (so it
    queues nothing into a vehicle that is closing), the detection thread,
    and the I/O thread last (it drains what is still queued, then closes
    the port). Every step is guarded: one that fails must not skip the
    next."""
    ramase = []
    for nume, fn in (('supervizor', getattr(supervizor, 'stop', None)),
                     ('detectie', getattr(detector, 'stop', None)),
                     ('mav_io', getattr(vehicle, 'close', None))):
        if fn is None:
            continue
        try:
            r = fn()
            if r:
                ramase.extend(r if isinstance(r, list) else [nume])
        except Exception as e:                              # noqa: BLE001
            print(f"[bord] oprirea firului {nume} a picat: "
                  f"{type(e).__name__}: {e}")
    if ramase:
        print(f"[bord] fire inca in viata la iesire: {', '.join(ramase)}")
    return ramase


def mesaj_mod(gate, canal, prag):
    """Textul STATUSTEXT de la pornire: ce face comutatorul, daca il ridici.
    Sub 50 de caractere, limita campului."""
    if gate.monitor:
        return f"NOVA MONITOR: {gate.monitor_reason}"[:50]
    return f"NOVA ZBOR gata: AUX{canal} > {prag} = handover"[:50]


def anunta_modul(vehicle, gate, canal, prag):
    """STATUSTEXT o data la pornire. Nu e o comanda: FC-ul il transmite mai
    departe spre GCS / OSD, daca exista telemetrie. Fara ea, se pierde fara
    niciun efect - de aceea nu opreste nimic cand esueaza."""
    text = mesaj_mod(gate, canal, prag)
    print(f"[bord] {text}")
    sev = (mavutil.mavlink.MAV_SEVERITY_WARNING if gate.monitor
           else mavutil.mavlink.MAV_SEVERITY_NOTICE)
    try:
        vehicle.send_statustext(sev, text)
    except Exception:                                           # noqa: BLE001
        pass


def extnav_config(cfg, canal, prag):
    """ExtNavConfig from config/nova.json: the switch, and the lateral
    tolerance (extnav_tol_min_m / extnav_tol_frac; absent = the brief's)."""
    kw = {}
    for key, field in (('extnav_tol_min_m', 'tol_min_m'),
                       ('extnav_tol_frac', 'tol_frac')):
        if cfg.get(key) is not None:
            kw[field] = float(cfg[key])
    # touchdown and the climb back: validated (ValueError on a refusal)
    td, _warnings = nova_config.touchdown_settings(cfg)
    kw.update(td)
    return ExtNavConfig(aux_channel=canal, aux_high_pwm=prag, **kw)


def cablaj_extnav(a, cfg, vehicle, override, gate_kw, canal, prag,
                  signal_reject, on_sm_event):
    """The flight configuration since 27.09.2026 (REPROIECTARE_EXTNAV.md):
    camera -> VISION_POSITION_ESTIMATE -> EKF3 SRC2 (no GNSS), GUIDED in
    steps of h/2 down to 1 m, LAND vertical. No PLND, no LANDING_TARGET, no
    DISTANCE_SENSOR, no authority modulation (4.8 names anyway). The EKF
    source manager is instantiated HERE - it is the central piece now
    (open item 36 closed on the onboard path)."""
    if 'alt_min_m' not in gate_kw:
        gate_kw = dict(gate_kw, alt_min_m=1.0)          # brief D6: 1-12 m
    gate = HandoverGate(vehicle, override, on_reject=signal_reject,
                        dist_max_m=None, detection_max_age_s=None,
                        **gate_kw, monitor=a.monitor_motiv or a.monitor)
    # Phase 5 (refactor/threads): the supervisor lives in its own thread
    # and reads the vehicle through a VehicleView (snapshots); its verdict
    # reaches the state machine through `abort`, read by the main thread
    # each step - no on_exit callback across threads.
    sup = ExtNavSupervisor(vedere(vehicle), override=override)
    est = ExtNavEstimator(vehicle, verbose=False)
    ekf = EkfSourceManager(vehicle, faze=SRC2_PHASES)
    sm = ExtNavLanding(vehicle, est, ekf, gate,
                       extnav_config(cfg, canal, prag),
                       on_event=on_sm_event)
    sm.attach_supervisor(sup)
    print("[bord] ghidare EXTNAV: camera -> VISION_POSITION_ESTIMATE -> EKF3 "
          "SRC2; GUIDED in trepte h/2 pana la 1 m; LAND vertical. PLND 0.")
    print(f"[bord] poarta: {gate_kw.get('alt_min_m', 1.0):.1f}-12 m, fara "
          f"raza; segmentul porneste la o detectie in <= 5 s (un retry)")
    if a.no_ascent:
        print("[bord] --no-ascent: fara efect pe extnav (secventa se "
              "incheie la contact oricum)")
    if not a.no_authority:
        print("[bord] modularea de autoritate NU exista pe extnav")
    return gate, sup, sm, None


def cablaj_plnd(a, cfg, vehicle, override, gate_kw, canal, prag,
                signal_reject, on_sm_event):
    """The previous path: LAND + precision landing (LANDING_TARGET +
    DISTANCE_SENSOR). Kept for a comparison flight; the simulator flies it."""
    monitor = a.monitor_motiv or a.monitor
    gate = HandoverGate(vehicle, override, on_reject=signal_reject,
                        **gate_kw, monitor=monitor)
    sup = SafetySupervisor(vedere(vehicle), override=override)
    # Modularea de autoritate pe praguri de altitudine. `None` o dezactiveaza
    # complet: fara ea, vehiculul zboara cu reglajul lui nominal, ceea ce e
    # exact comportamentul de dinainte.
    autoritate = (None if a.no_authority
                  else AuthorityScheduler(
                      vehicle, allow_fast_descent=a.fast_descent))
    seq = SequenceConfig(conv=a.conv, do_ascent=not a.no_ascent,
                         aux_channel=canal, aux_high_pwm=prag)
    if a.no_ascent:
        # 15.2.7 oprit: secventa se incheie pe sol, fara NAV_TAKEOFF. Pentru
        # PRIMA coborare autonoma pe un vehicul real asta e ce vrei - o
        # urcare automata imediat dupa contact e exact genul de surpriza
        # care te face sa tragi de manse. ArduPilot dezarmeaza singur din
        # LAND dupa contact (§5.6).
        print("[bord] 15.2.7 OPRIT (--no-ascent): secventa se incheie pe "
              "sol, fara urcare la 5 m")
    sm = LandingStateMachine(vehicle, seq, gate=gate, on_event=on_sm_event)
    sm.attach_supervisor(sup)
    print("[bord] ghidare PLND (calea veche): LAND + precision landing")
    return gate, sup, sm, autoritate


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
    # A thread that dies outside run_loop is logged with its name, not
    # lost on the stderr of a headless service (phase 5).
    install_excepthook()
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
    p.add_argument('--ring-frames', type=int, default=None,
                   help='cate cadre tine ringul pentru 8.3.3. Implicit 200 pe '
                        'extnav (~6.7 s la 30 fps: coborarea de la 1 m, contactul, '
                        '1 s dupa; ~185 MB la 1280x720) si 30 pe plnd. 0 = oprit, '
                        'si atunci NU se produce imaginea predata juriului')
    p.add_argument('--scoring-dir', default='data/scoring',
                   help='unde se scriu imaginea de scoring, cea de contact '
                        'si evidenta lor (6.2.1.30)')
    p.add_argument('--aux-channel', type=int, default=None,
                   metavar='N',
                   help=f'canalul RC pe care pilotul CERE segmentul autonom, '
                        f'pe frontul crescator. Implicit `aux_channel` din '
                        f'config/nova.json ({AUX_CHANNEL} daca lipseste). '
                        f'Modul LAND NU declanseaza nimic - intrarea e doar '
                        f'prin canalul asta')
    p.add_argument('--monitor', action='store_true',
                   help='poarta de handover INCHISA pentru rularea asta, '
                        'oricare ar fi config/nova.json. Poate doar inchide, '
                        'niciodata deschide. Folosit de pornirea automata')
    p.add_argument('--monitor-motiv', default=None, metavar='TEXT',
                   help='ca --monitor, cu motivul spus pilotului la refuz si '
                        'in STATUSTEXT la pornire. Folosit de pornirea '
                        'automata cand nu poate porni in modul de zbor')
    p.add_argument('--no-ascent', action='store_true',
                   help='opreste urcarea de dupa contact (15.2.7). Secventa '
                        'se incheie pe sol. Pentru primele coborari de test, '
                        'unde o urcare automata dupa touchdown e o surpriza')
    p.add_argument('--etape', type=float, default=10.0, metavar='S',
                   help='tipareste timpii pe etape ai detectorului la '
                        'fiecare S secunde (0 = oprit). Doar masurare')
    p.add_argument('--frame-log', default=None, metavar='CALE',
                   help='jurnalul per cadru al detectorului (CSV: metadatele '
                   'camerei - ExposureTime, AnalogueGain, LensPosition, '
                   'SensorTimestamp - si ce a facut detectia). Implicit '
                   '$NOVA_LOG_DIR sau ~/nova-logs/cadre-<data>.csv; "none" '
                   '= fara')
    p.add_argument('--camera-preset', default=None,
                   metavar='NUME', help='presetul camerei pentru rularea asta '
                   '(crop1280, crop1536, full1280, trackerv2): inlocuieste '
                   'cheile camerei din config/nova.json. Implicit '
                   '`camera_preset` din config')
    p.add_argument('--max-rms', type=float, default=None,
                   help='ridica pragul de reproiectie al calibrarii DOAR '
                        'pentru rularea asta. Pentru bring-up la banc cu o '
                        'calibrare provizorie; se anunta zgomotos. Pragul de '
                        'zbor din cod ramane neatins (§5.34)')
    p.add_argument('--fullscreen', action='store_true',
                   help='fereastra pe tot ecranul (implica --show-window). '
                        'Pentru bring-up la sol; iesire pe q sau Escape')
    p.add_argument('--preview-scale', type=float, default=0.5,
                   help='doar pentru --camera-check; fereastra de bord e '
                        'compusa fix la 720x480 (OSD analog)')
    a = p.parse_args()

    cfg = nova_config.load(a.config)
    banner(cfg)
    # touchdown and the climb back (27.09.2026): a refused setting stops
    # here, before the port and the camera; a warning is said and kept
    try:
        _td, _td_warn = nova_config.touchdown_settings(cfg)
    except ValueError as e:
        print(f"\n[bord] NU PORNESC: {e}\n")
        return 2
    for w in _td_warn:
        print(f"[bord] ATENTIE: {w}")

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
                                a.preview_scale, max_rms=a.max_rms,
                                preset=a.camera_preset)
        # Detectorul intai: daca lipseste calibrarea, ne oprim inainte sa
        # deschidem legatura cu FC-ul.
        detector = build_pi_detector(cfg, verbose=True,
                                     ring_frames=ring_frames_for(a, cfg),
                                     keep_color=nova_config.guidance(cfg) == 'extnav',
                                     max_rms=a.max_rms,
                                     keep_last_frame=(a.show_window or
                                                      a.fullscreen),
                                     preset=a.camera_preset,
                                     frame_log=frame_log_path(a.frame_log))
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
    # Faza 2 (refactor/threads): portul e al firului I/O al lui Vehicle;
    # bucla principala vede instantanee si trimite prin cozi.
    vehicle = Vehicle(a.conn, baud=baud, threaded=True).connect()

    override = OverrideMonitor(vehicle)

    def signal_reject(reason):
        if ecran is not None:
            ecran.note_handover(False, reason, time.time())
        print(f"\n!! HANDOVER REFUZAT: {reason}\n")
        try:
            vehicle.send_statustext(mavutil.mavlink.MAV_SEVERITY_WARNING,
                                    f"NOVA refuz: {reason}")
        except Exception:                                   # noqa: BLE001
            pass

    # Fara autonomy_enabled= aici: poarta citeste config/nova.json (E0).
    # --monitor INCHIDE poarta pentru rularea asta, oricare ar fi fisierul.
    # Doar in directia asta - E0 nu se deschide din linia de comanda
    # (§5.16). Folosit de pornirea automata: altfel, dupa deschiderea lui E0,
    # fiecare boot ar porni un sistem autonom viu, cu alte setari decat
    # proba de coborare.
    monitor = a.monitor_motiv or a.monitor
    # Altitude floor from config, if set; otherwise the gate's default.
    # Passed as a kwarg only when present, so a missing key changes nothing.
    gate_kw = {}
    if cfg.get('handover_alt_min_m') is not None:
        gate_kw['alt_min_m'] = float(cfg['handover_alt_min_m'])
        print(f"[bord] ATENTIE: handover acceptat de la "
              f"{gate_kw['alt_min_m']:.1f} m (config handover_alt_min_m)")
    # Ecranul are nevoie de ultima detectie (px si varsta). O ia dintr-un
    # invelis peste detector, nu dintr-o modificare in run_loop: bucla e
    # validata si nu vrem sa o atingem pentru afisare.
    # 8.3.3: imaginea predata juriului. Cadrele se scot din ringul
    # detectorului dupa timestamp-ul CAPTURII purtat de eveniment, nu dupa
    # cel al deciziei si nici dupa "ultimul cadru de acum" - intre ele sunt
    # zeci de milisecunde de coborare (§5.55). Cadrul de contact vine DUPA
    # eveniment, deci recorder-ul e chemat la fiecare ciclu (B6).
    ring = getattr(detector, 'ring', None)
    rec = ScoringRecorder(a.scoring_dir, ring, vehicle=vehicle)
    # 27.09.2026: on extnav the 8.3.3 image is taken at CONTACT (on the
    # ground, ON_GROUND from the FC), written by nova/touchdown_capture in
    # its own low-priority thread, in the format the PC tool fetches
    cap = None
    if nova_config.guidance(cfg) == 'extnav' and ring is not None:
        cap = TouchdownCapture(
            os.path.join(nova_config.REPO_ROOT, 'scoring'), ring,
            camera_info=lambda d=detector: camera_info(d),
            on_event=lambda n, i: on_sm_event(n, i),
            color_for=getattr(detector.source, 'color_for', None),
            meta_for=getattr(detector, 'metadata_at', None))
        print(f"[bord] captura de touchdown: {cap.root}/{cap.date}/{cap.session}/"
              f" (ring {ring.buf.maxlen} cadre)")
    detector = LastDetection(detector)      # on_poll: PasPrincipal, below
    if ring is None:
        print("[bord] ATENTIE: detectorul nu are ring buffer; 8.3.3 NU va "
              "avea imagine. Vezi --ring-frames.")
    canal = int(a.aux_channel if a.aux_channel is not None
                else cfg.get('aux_channel', AUX_CHANNEL))
    prag = int(cfg.get('aux_high_pwm', AUX_HIGH_PWM))
    print(f"[bord] handover: frontul crescator pe canalul RC {canal} "
          f"(sus = peste {prag} PWM)")

    osd = OsdMesaje()          # ce vede pilotul in banda ferestrei

    def on_sm_event(name, info):
        rec.on_event(name, info)
        osd.note(name, info, time.monotonic())
        if name == 'contact' and cap is not None:
            cap.on_contact(info)
        if name == 'abort' and info.get('action') == 'LOITER':
            # Pilot abort (AUX down): the sticks are live again. Said through
            # the FC, like the reject, so it reaches the OSD/GCS if there is
            # telemetry; nothing depends on it arriving.
            try:
                vehicle.send_statustext(
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                    "NOVA ABORT pilot: LOITER, throttle la mijloc")
            except Exception:                               # noqa: BLE001
                pass

    # The guidance is decided in the versioned config, like E0 (§5.16):
    # nothing on the command line can switch it.
    cablaj = (cablaj_extnav if nova_config.guidance(cfg) == 'extnav'
              else cablaj_plnd)
    gate, sup, sm, autoritate = cablaj(a, cfg, vehicle, override, gate_kw,
                                       canal, prag, signal_reject, on_sm_event)
    if monitor:
        print("[bord] MONITOR: poarta inchisa pentru rularea asta, oricare "
              f"ar fi config/nova.json. Motiv: {gate.monitor_reason}")
    # Pe teren, fara retea, pilotul nu are alt ecran decat OSD-ul / GCS-ul:
    # modul in care a pornit companion-ul se spune o data, prin FC.
    anunta_modul(vehicle, gate, canal, prag)

    # Phase 5 (refactor/threads): the supervisor in its own thread, fed
    # from Latest[Detection] (the detection thread), Latest[phase] and the
    # heartbeats (the main thread, through PasPrincipal); it monitors the
    # three other threads and the main thread watches it back (watchdog).
    # run_loop gets supervisor=None: the supervisor no longer runs inside
    # the main loop.
    faza = Latest(sm.state)
    hb_principal = Heartbeat('principal')
    st = SupervisorThread(sup, vehicle, getattr(detector, 'latest', None),
                          faza,
                          miss_streak_fn=lambda: getattr(detector,
                                                         'miss_streak', None))
    sup.set_thread_heartbeats(detectie=getattr(detector, 'heartbeat', None),
                              mav_io=getattr(vehicle, 'io_heartbeat', None),
                              principal=hb_principal)
    detector.on_poll = PasPrincipal(sm, faza, hb_principal,
                                    supervisor_thread=st,
                                    recorder=Recordere(rec, cap))
    # 2a: with the supervisor's abort up, the I/O thread drops the TX queue
    # and the periodic slots (counted, logged); URGENT still goes.
    vehicle.set_abort_event(sup.abort)

    ecran = race_screen.RaceScreen() if a.race else None

    # Step 0 (§5.65): stage timings every --etape seconds, from the timer
    # that PiDetector attaches to the source and the detector. Measurement
    # only; nothing in the loop changes.
    etape_la = {'t': None}

    def status(now):
        # Display only. Nothing here may stop the flight: run_loop does
        # not guard on_status, and one exception in a status line would
        # exit the app mid-descent (B8) - then systemd restarts it in
        # flight mode. Say the error, keep flying.
        try:
            _status(now)
        except Exception as e:                              # noqa: BLE001
            print(f"[bord] afisarea a picat ({type(e).__name__}: {e}); "
                  f"zborul continua")

    def _status(now):
        if ecran is None:
            print(f"{sm.status_line()} | {detector.status_line()} | "
                  f"{sup.status()} | {st.status()}"
                  + ('' if autoritate is None else f" | {autoritate.status()}"))
            if a.etape > 0 and (etape_la['t'] is None
                                or now - etape_la['t'] >= a.etape):
                etape_la['t'] = now
                linie = getattr(detector, 'stage_line', None)
                if linie is not None:
                    print(f"[etape] {linie()}")
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

    # The OSD window (27.09.2026): composed at `osd.size` (720x480 NTSC by
    # default, 720x576 PAL) by nova/board_window for the analog OSD - frame
    # + marker outline + three status lines, redrawn at every camera frame,
    # its own cost logged every 5 s. No scaling here (scale 1.0) and NO
    # mounting rotation: the 270 degrees are applied to the axes for
    # guidance, never to the pixels shown.
    pv = preview_mod.onboard_preview('NOVA bord',
                                     enabled=a.show_window or a.fullscreen,
                                     scale=1.0, fullscreen=a.fullscreen,
                                     rotate_deg=0)
    if pv.enabled:
        osd_size = (cfg.get('osd') or {}).get('size', [720, 480])
        try:
            detector = FereastraBord(detector, pv, sm=sm, sup=sup,
                                     vehicle=vehicle, mesaje=osd, size=osd_size)
            print(f"[bord] fereastra OSD {detector.size[0]}x{detector.size[1]} "
                  f"pornita (stare + evenimente, la fiecare cadru; inchiderea "
                  f"ei NU opreste zborul)")
        except ValueError as e:
            # here the vehicle is connected and the detector runs: a bad
            # display setting costs the window, never the app
            print(f"[bord] ATENTIE: fereastra OSD NU porneste ({e}); zborul "
                  f"merge fara ea")
            pv.close()
            pv.enabled = False
    # Start order: I/O (connect, above), detection (build_pi_detector,
    # above), supervisor, then the state machine (run_loop). The two
    # producers run before anyone consumes them; the supervisor watches
    # the state machine's thread from its first step.
    st.start()
    print(f"[bord] rulez: fire mav_io, detectie, supervizor "
          f"({1.0 / st.period_s:.0f} Hz), principal. Ctrl-C pentru oprire.")
    try:
        run_loop(vehicle, detector, sm, supervisor=None, on_status=status,
                 authority=autoritate)
    except KeyboardInterrupt:
        print("\n[bord] oprire.")
    finally:
        pv.close()
        opreste_firele(st, detector, vehicle)
        if cap is not None:
            cap.stop()              # a capture is evidence: flushed, not dropped
            for d, ok, why in cap.done:
                print(f"[bord] 8.3.3 captura {'OK' if ok else 'ESUATA'}: {d}"
                      + ('' if ok else f" ({why})"))
        r = rec.raport()
        if r['salvate']:
            print(f"[bord] 8.3.3: {', '.join(sorted(r['salvate'].values()))}")
        if r['lipsa']:
            print(f"[bord] 8.3.3 INCOMPLET: lipsesc {', '.join(r['lipsa'])}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
