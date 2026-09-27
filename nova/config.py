#!/usr/bin/env python3
"""
Configuratia companion-ului: config/nova.json.

Un singur fisier, citit de aplicatia de bord si de unelte. Valorile implicite
sunt aici, in cod, ca fisierul sa poata lipsi fara sa se schimbe nimic
periculos - si ca sa fie evident ce inseamna fiecare cheie.

E0 - garda de siguranta a rundei 3:

    "autonomy_enabled": false

Cat timp e false, poarta de handover REFUZA orice cerere, cu motiv explicit.
Se pune pe true manual, cu un commit separat, DUPA ce E2 (validarea offline
a detectorului) a trecut criteriile de acceptare. Detectia nevalidata intr-o
bucla care comanda coborare e modul cel mai direct de a distruge vehiculul.
Garda e in cod, nu in documentatie, tocmai ca sa nu depinda de cine isi
aduce aminte.
"""

import json
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(REPO_ROOT, 'config', 'nova.json')

DEFAULTS = {
    # E0. Implicit FALS. Vezi docstring-ul modulului.
    'autonomy_enabled': False,

    # Marker ArUco (E1.3): dictionar 4x4, ID-ul markerului de concurs, latura
    # zonei codate in metri (aceeasi cu nova.detection.MARKER_SIZE_M).
    'marker_id': 26,
    'marker_size_m': 0.48,

    # Calibrarea camerei (E1.2). Fara ea detectorul REFUZA sa porneasca.
    'camera_calibration': 'config/camera_pi.yaml',

    # Sub aceasta distanta detectia se face intr-un ROI centrat pe ultima
    # pozitie a markerului (E1.3). Markerul are > 90 px acolo, deci 640x480
    # ajunge, iar saltul de FPS pe Pi 4 e mare.
    # Zborul din 24.09.2026 (§5.62): drumul cu ROI dadea 66 ms / det 77%,
    # cadrul intreg 320 ms / det sub 15% - iar fereastra de handover (5-12 m)
    # cadea integral pe al doilea. ROI la orice distanta din fereastra, plus
    # cautare pe imaginea redusa de `search_downscale` ori (1 = dezactivat).
    'roi_below_m': 15.0,
    'search_downscale': 2,
    'roi_size_px': [640, 480],

    # Camera preset (27.09.2026): ONE key selects sensor mode, stream,
    # exposure, AWB and focus - nova.detector_pi.CAMERA_PRESETS:
    #   crop1280   1536x864 crop -> 1280x720, exposure measured then locked
    #              (what flew on 27.09.2026), calibration derived + scaled
    #   crop1536   1536x864 crop, no scaling, calibration derived
    #   full1280   2304x1296 full field -> 1280x720, calibration scaled
    #   trackerv2  like crop1280, continuous AE, AWB on, LensPosition 1.0
    # The keys below (non-null) refine the file's preset; a preset given on
    # the command line (--preset / --camera-preset) replaces all of them.
    'camera_preset': None,

    # IMX708 sensor mode [w, h], MANDATORY - no default on purpose. Without
    # it libcamera picks the mode itself: asked for a 1280x720 stream it
    # picked the 1536x864 CROPPED mode while the calibration was scaled as
    # full-field 2304x1296 (fx 577 instead of ~865; found 27.09.2026).
    # One of 4608x2592, 2304x1296 (binned, full field), 1536x864 (binned,
    # central crop). The camera reads it back and refuses a different one.
    'sensor_mode': None,

    # Detection stream [w, h]. None = the sensor mode's own size. The ISP
    # scales the mode down to it; the calibration follows through
    # nova.detector_pi.calibration_for (scale / derive, nothing else).
    # `track_size` is the older name, still read when this one is unset.
    'output_size': None,
    'track_size': None,

    # Exposure: 'fixed' (exposure_us / analogue_gain), 'auto_lock'
    # (measured on the scene, then locked, capped at 2000 us - flown since
    # 24.09.2026), 'auto' (continuous), 'auto_capped' (continuous inside
    # exposure_max_us / gain_max, both mandatory then). Unset = the
    # preset's; without a preset 'fixed', 2000 us, gain 8.
    'exposure': None,
    'exposure_us': None,
    'analogue_gain': None,
    'exposure_max_us': None,
    'gain_max': None,
    # Auto white balance (we work on luminance) and the manual focus, in
    # dioptres (1.63 = hyperfocal of the Wide lens). Unset = the preset's.
    'awb': None,
    'lens_position': None,

    # Cum e montata camera pe VEHICULUL REAL: cu cate grade trebuie rotita
    # imaginea bruta SPRE STANGA (pe ecran) ca nasul dronei sa ajunga sus.
    # 0, 90, 180 sau 270. Se aplica pe maparea axelor, nu pe pixeli - vezi
    # nova.detector_pi.axe_corp.
    'camera_rotation_deg': 0,

    # Fisierul de parametri ArduPilot care corespunde FIRMWARE-ULUI de pe
    # FC. Numele si unitatile difera intre 4.6 si 4.7 (CLAUDE.md §5.4), deci
    # preflight-ul trebuie sa verifice fisierul potrivit versiunii - altfel
    # pica pe nume care pur si simplu nu exista pe placa.
    #   4.7.0+  config/nova_flight.parm
    #   4.5.x   config/nova_flight_4.5.parm  (generat, nu editat de mana)
    'flight_parm': 'config/nova_flight.parm',

    # Canalul RC pe care pilotul CERE segmentul autonom (frontul crescator).
    # Intr-un singur loc: pornirea automata si proba de coborare trebuie sa
    # asculte ACELASI canal. Cand erau parametri separati, monitorul asculta
    # 7 iar comutatorul era pe 6 - deci in log nu aparea nicio cerere, si
    # parea ca handover-ul nu ajunge la Pi.
    'aux_channel': 7,
    # Peste ce PWM consideram comutatorul "sus". 1700 e pentru comutatoare
    # de 3 pozitii (~1000/1500/2000: clar in treapta de sus, departe de
    # mijloc). Pentru unul de 2 pozitii (~1000/2000), 1500 e chiar mijlocul.
    'aux_high_pwm': 1700,

    # Which guidance the onboard app flies (27.09.2026 redesign):
    #   "extnav"  camera -> VISION_POSITION_ESTIMATE -> EKF3 SRC2, GUIDED in
    #             steps of h/2 down to 1 m, then LAND vertical. PLND stays 0,
    #             nothing is sent as LANDING_TARGET / DISTANCE_SENSOR.
    #   "plnd"    the previous path (LAND + precision landing), kept for a
    #             comparison flight; the simulator is on it regardless.
    # Anything else = extnav. Written in the versioned file, like E0.
    'guidance': 'extnav',

    # Handover gate altitude floor, metres. None = the gate's own default
    # (nova/handover.py: HANDOVER_ALT_MIN_M, 5 m). A number here REPLACES
    # the floor for the onboard app only - the gate code stays untouched.
    # Exists for descent tests from any height (team, 26.09.2026); the
    # 12 m ceiling is not configurable. Remove the key to get 5 m back.
    'handover_alt_min_m': None,

    # ExtNav lateral tolerance tol(h) = max(extnav_tol_min_m,
    # extnav_tol_frac * h), metres (MOVE exit, CENTER_CHECK, FINAL_ALIGN).
    # None = the brief's 0.15 m / 0.10 h (nova/extnav.py).
    'extnav_tol_min_m': None,
    'extnav_tol_frac': None,

    # Touchdown without disarming and the climb back (ZDC rules 15.1.2,
    # 15.1.3, 15.2.7, 8.2.2): after FINAL_ALIGN the vehicle descends in
    # GUIDED to contact, stays armed on the ground, then climbs in GUIDED
    # to h_ref + alt_riseup above the marker (h_ref = baro height at
    # contact). Validated by touchdown_settings(): alt_riseup < 5 m is a
    # WARNING (15.1.2 asks for at least 5 m; 5.5 leaves 0.5 m for baro
    # error), touchdown_hold_s < 1 s is REFUSED (15.1.3: stable >= 1 s).
    'alt_riseup': 5.5,
    'touchdown_speed': 0.4,
    'touchdown_hold_s': 1.5,
    'riseup_timeout_s': 15.0,
    'hover_confirm_s': 1.0,

    # OLD KEY, replaced by `exposure` (27.09.2026): true = 'auto_lock',
    # false = 'fixed'. Read only when `exposure` is unset. None = not set:
    # the preset decides (a default of True here would override every
    # preset's own exposure mode).
    'camera_auto_expose': None,
    # Step 5 (§5.65): take the NEWEST completed frame (capture_request
    # flush=True) instead of the oldest queued one. Measured before: frame
    # already 50-90 ms old when it left the camera. False = old queue.
    'camera_fresh_capture': True,

    # Ce porneste la boot (pi/bringup.sh, din pornirea automata):
    #   "monitor" - detector + fereastra, ZERO comenzi, oricare ar fi E0
    #   "zbor"    - proba de coborare (pi/descent_test.sh --auto): aceleasi
    #               verificari, fara confirmarea tastata. Doar cu E0 deschis.
    # Orice alta valoare = monitor. Vezi autostart_mode().
    'autostart': 'monitor',

    # Onboard OSD window (nova/board_window.py): [width, height] of the
    # analog video it feeds. [720, 480] = NTSC, [720, 576] = PAL. The text
    # band stays 75 px; the camera frame is stretched over the rest.
    'osd': {'size': [720, 480]},
}


def load(path=None):
    """Dictionar complet: DEFAULTS peste care se aseaza fisierul, daca exista.
    Cheile necunoscute din fisier sunt pastrate (nu le validam aici), ca sa
    poata fi folosite de unelte fara sa modifice modulul asta."""
    path = path or DEFAULT_PATH
    cfg = dict(DEFAULTS)
    if os.path.exists(path):
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError(f"{path}: se astepta un obiect JSON")
        cfg.update({k: v for k, v in data.items() if not k.startswith('_')})
    cfg['_path'] = path
    cfg['_exists'] = os.path.exists(path)
    return cfg


def autonomy_enabled(path=None):
    """E0. Fals daca fisierul lipseste, daca cheia lipseste sau daca valoarea
    e orice altceva decat literalul `true`. Nu exista cale de a-l activa din
    linia de comanda sau din variabile de mediu - doar din fisierul versionat."""
    return load(path).get('autonomy_enabled') is True


def autostart_mode(path=None):
    """(mod, motiv) pentru pornirea automata: ('zbor', '') sau
    ('monitor', de_ce).

    'zbor' cere DOUA lucruri, amandoua din fisierul versionat: `autostart`
    exact "zbor" si E0 deschis (literalul `true`, ca in autonomy_enabled()).
    Nu exista cale din linia de comanda sau din mediu - la fel ca E0 (§5.16):
    pe teren, fara retea, fisierul de pe Pi e singurul loc unde s-a decis.

    Motivul e scurt deliberat: ajunge in STATUSTEXT (50 de caractere cu tot
    cu prefix), deci pe OSD / in Mission Planner, unde pilotul il poate citi
    fara laptop."""
    cfg = load(path)
    if cfg.get('autostart') != 'zbor':
        return 'monitor', 'autostart=monitor'
    if cfg.get('autonomy_enabled') is not True:
        return 'monitor', 'autostart=zbor dar E0 inchis'
    return 'zbor', ''


#: Keys of the touchdown sequence and their accepted ranges.
TOUCHDOWN_KEYS = ('alt_riseup', 'touchdown_speed', 'touchdown_hold_s',
                  'riseup_timeout_s', 'hover_confirm_s')


def touchdown_settings(cfg):
    """(values dict, warnings list) for the touchdown sequence, or
    ValueError. touchdown_hold_s < 1.0 is refused: rule 15.1.3 wants a
    stable touchdown of at least 1 s in the FC log. alt_riseup < 5.0 is
    only a warning: 15.1.2 asks for >= 5 m above the marker, and the
    default 5.5 keeps 0.5 m of margin for the barometer."""
    v = {}
    for k in TOUCHDOWN_KEYS:
        raw = cfg.get(k, DEFAULTS[k])
        try:
            v[k] = float(DEFAULTS[k] if raw is None else raw)
        except (TypeError, ValueError):
            raise ValueError(f"config: `{k}` = {raw!r} nu e un numar")
        if v[k] <= 0:
            raise ValueError(f"config: `{k}` = {v[k]} trebuie sa fie pozitiv")
    warnings = []
    if v['touchdown_hold_s'] < 1.0:
        raise ValueError(
            f"config: touchdown_hold_s = {v['touchdown_hold_s']} s < 1.0 s: "
            f"regula 15.1.3 cere contact stabil >= 1 s in logul FC")
    if v['alt_riseup'] < 5.0:
        warnings.append(
            f"alt_riseup = {v['alt_riseup']} m < 5.0 m: regula 15.1.2 cere "
            f"urcare la cel putin 5 m deasupra markerului")
    if v['touchdown_speed'] > 1.0:
        warnings.append(f"touchdown_speed = {v['touchdown_speed']} m/s: contact dur")
    return v, warnings


def guidance(cfg):
    """'plnd' only when the file says exactly "plnd"; otherwise 'extnav'."""
    return 'plnd' if cfg.get('guidance') == 'plnd' else 'extnav'


def resolve(cfg, key):
    """Cai relative la radacina repo-ului."""
    val = cfg[key]
    if os.path.isabs(val):
        return val
    return os.path.join(REPO_ROOT, val)
