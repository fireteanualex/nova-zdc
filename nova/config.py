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

    # Cum e montata camera pe VEHICULUL REAL: cu cate grade trebuie rotita
    # imaginea bruta SPRE STANGA (pe ecran) ca nasul dronei sa ajunga sus.
    # 0, 90, 180 sau 270. Se aplica pe maparea axelor, nu pe pixeli - vezi
    # nova.detector_pi.axe_corp. Simularea NU o citeste: camera din Gazebo
    # e montata drept, indiferent cum e cea de pe vehicul.
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

    # Expunerea camerei: masurata pe scena la pornire (AE converge ~1 s),
    # apoi BLOCATA, cu plafon la 2000 us (limita de blur). False = valorile
    # fixe de banc (2000 us / gain 8) - care pe teren, la lumina zilei, s-au
    # dovedit cu ~5-6 trepte prea luminoase (zborul din 24.09.2026).
    'camera_auto_expose': True,
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


def resolve(cfg, key):
    """Cai relative la radacina repo-ului."""
    val = cfg[key]
    if os.path.isabs(val):
        return val
    return os.path.join(REPO_ROOT, val)
