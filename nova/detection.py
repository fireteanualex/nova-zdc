#!/usr/bin/env python3
"""
Contractul dintre viziune si control.

Un detector - sintetic sau ArUco real - produce obiecte Detection si atat.
Nu trimite MAVLink, nu cunoaste starile, nu comanda vehiculul. Tot ce are
nevoie masina de stari trebuie sa fie in Detection; daca apare ceva nou,
se adauga aici, nu pe o cale laterala.

Conventia de axe (corp, FRD - Front/Right/Down):
    angle_x  unghi spre marker pe axa INAINTE a corpului, rad
    angle_y  unghi spre marker pe axa DREAPTA a corpului, rad

Rotatia de compatibilitate cu ArduPilot (5.1 din CLAUDE.md) NU se aplica
aici: e o proprietate a mesajului LANDING_TARGET, deci sta in
state_machine.py. Asa, detectorul real cu solvePnP publica in conventia de
mai sus si nu redescopera fixul.
"""

import math
from dataclasses import dataclass

# --- Marker ---------------------------------------------------------------
MARKER_SIZE_M = 0.48      # latura zonei codate

# --- Camera: Raspberry Pi Camera Module 3 Wide (IMX708) -------------------
HFOV_DEG = 102.0          # camp vizual orizontal (axa Y corp / dreapta)
VFOV_DEG = 67.0           # camp vizual vertical  (axa X corp / inainte)
FOCAL_PX = 933.0          # la rezolutia de lucru 2304x1296
CAM_HEIGHT_M = 0.0745     # inaltimea camerei deasupra solului la contact

# 8.3.3 cere doar ca centrul imaginii de touchdown sa contina un pixel de pe
# marker. La 74.5 mm amprenta camerei e 184x99 mm pe un marker de 480 mm,
# deci conditia e satisfacuta pentru orice eroare sub ~19 cm. Nu e un prag de
# punctaj - regulamentul nu puncteaza precizia - ci limita de valabilitate a
# imaginii obligatorii.
CAM_FOOTPRINT_MARGIN_M = 0.19


@dataclass(frozen=True)
class Detection:
    """O observatie a markerului, la un moment dat.

    t           time.monotonic() la capturarea cadrului (nu la publicare)
    angle_x     offset unghiular pe axa INAINTE, rad
    angle_y     offset unghiular pe axa DREAPTA, rad
    distance_m  distanta 3D pana la marker (din solvePnP / dimensiune)
    marker_px   latura markerului in imagine, px - criteriul de scoring (8.3.3)
    range_m     citirea echivalenta de telemetru, NEcorectata de inclinare.
                ArduPilot inmulteste singur cu cos(tilt), deci aici se pune
                alt / cos(tilt), nu alt (5.3 din CLAUDE.md).
    """
    t: float
    angle_x: float
    angle_y: float
    distance_m: float
    marker_px: float
    range_m: float


@dataclass(frozen=True)
class CameraModel:
    """Geometria comuna detectorului sintetic si celui real."""
    focal_px: float = FOCAL_PX
    hfov_deg: float = HFOV_DEG
    vfov_deg: float = VFOV_DEG
    marker_size_m: float = MARKER_SIZE_M

    @property
    def frame_h_px(self):
        return 2.0 * self.focal_px * math.tan(math.radians(self.vfov_deg / 2.0))

    def marker_px_at(self, distance_m):
        return self.focal_px * self.marker_size_m / max(distance_m, 1e-6)

    def in_fov(self, angle_x, angle_y):
        return (abs(angle_x) <= math.radians(self.vfov_deg / 2.0) and
                abs(angle_y) <= math.radians(self.hfov_deg / 2.0))

    def fits_in_frame(self, marker_px):
        """5.2: markerul trebuie sa incapa INTREG in cadru, nu doar centrul
        lui. Cu focal 933 si VFOV 67 grade limita e ~1173 px, adica 0.38 m.
        Fara verificarea asta, SCORING_CAPTURE se declansa la 0.19 m cu
        2344 px (fizic imposibil) si takeoff-ul ramanea blocat."""
        return marker_px <= self.frame_h_px * 0.95
