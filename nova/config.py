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
    'roi_below_m': 5.0,
    'roi_size_px': [640, 480],
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


def resolve(cfg, key):
    """Cai relative la radacina repo-ului."""
    val = cfg[key]
    if os.path.isabs(val):
        return val
    return os.path.join(REPO_ROOT, val)
