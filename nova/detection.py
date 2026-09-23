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
FRAME_W_PX = 2304.0       # rezolutia de lucru, in pixeli reali
FRAME_H_PX = 1296.0       # NU `frame_h_px`, care e derivata din VFOV
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
    fill        cat din cadru ocupa CUTIA DE INCADRARE a markerului, pe axa
                mai stramta: max(cutie_lat / latime, cutie_inalt / inaltime).
                None daca detectorul nu o poate raporta.

`fill` si `marker_px` nu sunt acelasi lucru, si diferenta decide 8.3.3.
`marker_px` e LATURA; ce trebuie sa incapa in cadru e cutia unui patrat
rotit, mai mare cu `|cos| + |sin|` - pana la 41% la 45 grade (§5.49). Un
prag fix in pixeli e deci o conditie a carei marja se prabuseste exact la
rotatiile pe care nu le controlam: masurat, 980 px e de neatins peste ~18
grade (§5.51) si 800 px a picat la 41 grade (§5.54).

`fill` nu se calculeaza din `marker_px` si un unghi presupus - se ia din
colturile detectate, deci include rotatia, perspectiva si distorsiunea asa
cum sunt, nu cum ar fi la un patrat perfect vazut de sus.

**None inseamna necunoscut, nu zero.** Un detector care nu raporteaza
incadrarea trimite None si consumatorul cade pe pragul in pixeli; un 0.0
implicit ar spune "markerul e mic" tocmai cand e pe cale sa iasa din cadru
(§5.53: un termen lipsa produce refuz, nu acceptare tacuta).
    """
    t: float
    angle_x: float
    angle_y: float
    distance_m: float
    marker_px: float
    range_m: float
    fill: float = None


@dataclass(frozen=True)
class CameraModel:
    """Geometria comuna detectorului sintetic si celui real."""
    focal_px: float = FOCAL_PX
    hfov_deg: float = HFOV_DEG
    vfov_deg: float = VFOV_DEG
    marker_size_m: float = MARKER_SIZE_M
    #: Dimensiunea cadrului in PIXELI. Deliberat separata de `frame_h_px`,
    #: care se deduce din VFOV: la 2304x1296 cele doua difera cu ~5%, pentru
    #: ca VFOV-ul de fisa tehnica (67 grade) nu e cel geometric al decupajului
    #: (69.6 grade) - vezi §2. Ce se compara cu o cutie masurata in pixeli
    #: trebuie sa fie tot in pixeli.
    width_px: float = FRAME_W_PX
    height_px: float = FRAME_H_PX

    @property
    def frame_h_px(self):
        return 2.0 * self.focal_px * math.tan(math.radians(self.vfov_deg / 2.0))

    def marker_px_at(self, distance_m):
        return self.focal_px * self.marker_size_m / max(distance_m, 1e-6)

    def in_fov(self, angle_x, angle_y):
        return (abs(angle_x) <= math.radians(self.vfov_deg / 2.0) and
                abs(angle_y) <= math.radians(self.hfov_deg / 2.0))

    def fits_in_frame(self, marker_px, yaw_deg=0.0):
        """5.2: markerul trebuie sa incapa INTREG in cadru, nu doar centrul
        lui. Fara verificarea asta, SCORING_CAPTURE se declansa la 0.19 m cu
        2344 px (fizic imposibil) si takeoff-ul ramanea blocat.

        `yaw_deg` e rotatia markerului IN CADRU. Ce trebuie sa incapa nu e
        latura, ci **cutia de incadrare** a unui patrat rotit, mai mare cu
        `|cos| + |sin|` - pana la 41% la 45 grade (§5.49).

        Masurat: la range 0.42 m si 1057 px, criteriul pe latura spune "DA"
        la orice rotatie, dar detectia merge la 10 grade si pica la 20.
        Implicitul 0 pastreaza comportamentul vechi pentru apelantii care nu
        stiu rotatia; cine o stie trebuie sa o dea."""
        factor = abs(math.cos(math.radians(yaw_deg))) + \
            abs(math.sin(math.radians(yaw_deg)))
        return marker_px * factor <= self.frame_h_px * 0.95

    def fill_at(self, marker_px, yaw_deg=0.0):
        """`fill` pentru un patrat IDEAL de latura `marker_px`, rotit cu
        `yaw_deg` in cadru.

        Pentru detectorul sintetic, care nu are colturi de masurat. Cel real
        foloseste `fill_from_corners`, care ia forma asa cum e."""
        factor = abs(math.cos(math.radians(yaw_deg))) + \
            abs(math.sin(math.radians(yaw_deg)))
        return max(marker_px * factor / self.width_px,
                   marker_px * factor / self.height_px)


    def tilt_budget_deg(self, alt_m, lateral_m=0.0):
        """Cat se poate inclina vehiculul fara ca markerul sa iasa din cadru.

        §5.48, scris ca inegalitate:

            tan(inclinare) <= tan(jumatate_de_cadru) - (lateral + 0.24) / h

        Termenul din dreapta e perfid: vehiculul se inclina TOCMAI ca sa
        corecteze lateral, deci exact cand eroarea e mare, cadrul se muta in
        directia gresita. Cu cat eroarea laterala e mai mare, cu atat ai voie
        sa te inclini mai putin.

        Jumatatea de cadru se ia din PIXELI (`height_px`, axa scurta), nu din
        VFOV-ul de fisa tehnica: cele doua difera cu ~1.3 grade (§2).

        Masurat, si reprodus de functia asta:
          h 7.17 m, lateral 2.95 m -> 14.0 grade, iar tranzitoriul a atins
          19.3: markerul a iesit din cadru cu 72.7 cm (§5.48)
          h 1.00 m, lateral 0.10 m -> 19.5 grade, adica sub pragul de
          integritate de 30 al supervizorului (elementul deschis 30)

        **Nu e acelasi lucru cu `safety.MAX_TILT_DEG`.** Acela e plafonul de
        integritate al vehiculului, o constanta. Asta e limita camerei, si e
        functie de altitudine si de eroarea laterala. Doua praguri distincte
        care pana acum imparteau un numar."""
        if alt_m is None or alt_m <= 0.0:
            return 0.0
        jumatate = math.atan2(self.height_px / 2.0, self.focal_px)
        t = math.tan(jumatate) - \
            (abs(lateral_m) + self.marker_size_m / 2.0) / alt_m
        if t <= 0.0:
            return 0.0
        return math.degrees(math.atan(t))


def fill_from_corners(corners, frame_w_px, frame_h_px):
    """Cat din cadru ocupa cutia de incadrare a colturilor detectate.

    `corners` e orice iterabil de perechi (x, y) in pixeli. Rezultatul e
    fractia de pe axa mai STRAMTA - cea care se atinge prima:

        max(latime_cutie / frame_w, inaltime_cutie / frame_h)

    Se ia din colturi, nu din `marker_px` si un unghi presupus. Diferenta
    nu e academica: latura e ce masoara `side_px`, dar ce iese din cadru e
    cutia, mai mare cu pana la 41% la 45 grade (§5.49). Iar colturile
    poarta si perspectiva si distorsiunea, pe care un `|cos| + |sin|`
    aplicat unui patrat ideal nu le vede.

    Intoarce None daca dimensiunile cadrului nu sunt utilizabile: necunoscut
    nu inseamna zero, iar un 0.0 aici ar spune "markerul e mic" exact cand e
    pe cale sa iasa din cadru."""
    if not frame_w_px or not frame_h_px:
        return None
    xs = [float(c[0]) for c in corners]
    ys = [float(c[1]) for c in corners]
    if not xs or not ys:
        return None
    return max((max(xs) - min(xs)) / float(frame_w_px),
               (max(ys) - min(ys)) / float(frame_h_px))
