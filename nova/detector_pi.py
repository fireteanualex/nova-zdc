#!/usr/bin/env python3
"""
Detectorul real (E1): Camera Module 3 Wide -> ArUco -> solvePnP -> Detection.

Inlocuieste tools/fake_detector.py ca sursa de Detection. Interfata e cea din
nova/detection.py (`poll(now)` -> lista de Detection), deci masina de stari si
supervizorul nu se modifica deloc.

Trei straturi, separate deliberat:

    FrameSource        de unde vin pixelii: picamera2 (bord), director de
                       imagini (E2), cadre din memorie (teste). O singura
                       interfata: read() -> (gray, t_captura) sau None.
    ArucoMarkerDetector cadru -> Detection. Numai OpenCV, fara camera, fara
                       fire. Se testeaza pe desktop cu markere sintetice.
    PiDetector         leaga sursa de detector intr-un fir separat si expune
                       poll(now) + instrumentare (E1.4).

Ce NU face detectorul:
  - nu aplica rotatia de axe din CLAUDE.md §5.1 - aceea e o proprietate a
    mesajului LANDING_TARGET si sta in nova/state_machine.py;
  - nu comanda nimic si nu stie de stari.

Conventia de montaj (E5, docs/MONTAJ.md): camera priveste in jos, cu
**varful imaginii spre nasul dronei**. Deci, in cadrul camerei (x dreapta,
y in jos pe imagine, z pe axa optica):

    inainte (body X) = -y_cam        dreapta (body Y) = +x_cam

Asta e conventia de montaj, nu rotatia din §5.1. O eroare de montaj aici se
manifesta ca orbitare eliptica (acelasi simptom ca in §5.1), dar de origine
mecanica - verificarea e in procedura de receptie.
"""

import collections
import math
import os
import logging
import threading
import time
import traceback

import cv2
import numpy as np

from .concurrency import Heartbeat, Latest
from .detection import (Detection, CameraModel, MARKER_SIZE_M,
                        fill_from_corners)
from .frame_ring import FrameRing

log = logging.getLogger('nova')

# --- Camera: rezolutii si controale (E1.1) ---------------------------------

#: Fluxul de tracking. Modul de senzor binned 2x2 al IMX708 (2304x1296) da
#: pana la ~56 fps; il rulam la 30.
TRACK_SIZE = (2304, 1296)
TRACK_FPS = 30

#: Cadrul de scoring (grupul C). Rezolutia nativa e un ALT mod de senzor
#: (~14 fps), deci nu poate curge simultan cu tracking-ul la 30 fps - se
#: obtine prin comutare de mod, la cerere (capture_scoring_frame). Vezi
#: nota din PiCameraSource.
SCORING_SIZE = (4608, 2592)

#: Controale fixate la pornire. Toate obligatorii, fiecare cu motivul lui.
#:
#: AfMode Manual + LensPosition 1.63 (dioptrii, ~0.61 m): hiperfocala
#:   obiectivului Wide - clar de la ~0.31 m la infinit. PDAF-ul, lasat pe
#:   automat, ar cauta focus exact in timpul coborarii, cand contrastul
#:   markerului se schimba cel mai repede.
#: ExposureTime 2000 us: blur-ul de miscare e v * t_exp * f / Z. La 2 ms,
#:   cu f = 933 px, ramane sub 1 px pe tot profilul de coborare
#:   (2 m/s la 8 m = 0.47 px; 0.5 m/s la 1 m = 0.93 px). Markerul are
#:   contrast maxim si tolereaza zgomot mult mai bine decat blur.
#: AnalogueGain 8.0: compenseaza expunerea scurta. Zgomotul rezultat e
#:   acceptabil pentru un patrat alb-negru.
#: AeEnable False: expunerea automata ar oscila cand markerul alb-negru
#:   intra in cadru si ar schimba pragul de binarizare intre cadre.
#: AwbEnable False: lucram pe luminanta; balansul de alb nu are ce cauta
#:   intr-o bucla determinista.
CAMERA_CONTROLS = {
    'LensPosition': 1.63,
    'ExposureTime': 2000,
    'AnalogueGain': 8.0,
    'AeEnable': False,
    'AwbEnable': False,
}

#: Plafonul de expunere al blocarii automate = valoarea de banc de mai sus:
#: sub 2000 us blur-ul de miscare ramane sub 1 px pe tot profilul de
#: coborare. Auto-expunerea are voie sa aleaga ORICAT de scurt (afara va
#: alege sute de us la gain mic), dar niciodata mai lung.
AUTOEXP_MAX_US = CAMERA_CONTROLS['ExposureTime']
#: Cate cadre lasam AE-ul sa convearga inainte de blocare (~1 s la 30 fps).
AUTOEXP_FRAMES = 30
#: Cate cadre asteptam sa se APLICE un control dupa set_controls. Prima
#: varianta astepta fix 5: pe vehicul, seara, driverul inca raporta vechiul
#: 32680 us, verificarea dadea ESEC fals, preflight-ul pica si pornirea
#: automata cadea in monitor - trei boot-uri la rand (§5.61). Se asteapta
#: VALOAREA, nu un numar de cadre.
CONTROL_APPLY_FRAMES = 30


def asteapta_valoare(citeste, cheie, tinta, max_incercari=CONTROL_APPLY_FRAMES,
                     tol_rel=0.05):
    """Apeleaza `citeste()` (dict de metadate) pana cand `cheie` ajunge la
    `tinta` (toleranta 5%%). True daca s-a aplicat; False la epuizare -
    atunci nepotrivirea e reala, nu metadate vechi."""
    for _ in range(max_incercari):
        md = citeste() or {}
        got = md.get(cheie)
        if got is not None and \
                abs(float(got) - float(tinta)) <= tol_rel * abs(float(tinta)) + 1e-6:
            return True
    return False


def expunere_blocata(exp_us, gain, exp_max_us=AUTOEXP_MAX_US,
                     gain_min=1.0, gain_max=16.0):
    """(exp_us, gain, avertisment) - valorile de FIXAT dupa convergenta AE.

    Regula: expunerea nu depaseste plafonul de blur; ce lipseste se muta in
    gain, la aceeasi luminozitate (exp*gain constant). Zgomotul de gain e
    tolerabil pe un patrat alb-negru; blur-ul, nu (comentariul de la
    CAMERA_CONTROLS).

    Prima valoare fixa (2000 us + gain 8, aleasa pe banc) s-a dovedit pe
    teren o presupunere despre LUMINA: afara e cu ~5-6 trepte prea multa,
    iar imaginea aproape alba pierde markerul intermitent - simptomul
    zborului din 24.09.2026 (det 30-44%%). De aceea expunerea se MASOARA la
    pornire si abia apoi se blocheaza, in loc sa fie o constanta."""
    exp_us = float(exp_us)
    gain = float(gain)
    avert = None
    if exp_us > exp_max_us:
        gain = gain * exp_us / exp_max_us
        exp_us = float(exp_max_us)
    if gain > gain_max:
        avert = (f"prea intuneric pentru plafonul de blur: ar trebui gain "
                 f"{gain:.1f}, senzorul da maxim {gain_max:.0f}. Fixez "
                 f"maximul; imaginea va fi mai intunecata decat ideal.")
        gain = gain_max
    if gain < gain_min:
        gain = gain_min
    return exp_us, gain, avert

#: Calibrare (E1.2): peste asta calibrarea e proasta si se refuza.
#:
#: 0.85 px, DECIZIA ECHIPEI (23.09.2026), ridicat de la 0.5. Calibrarea
#: reala a camerei de pe vehicul (60 de poze ChArUco) da 0.829 px.
#:
#: Ce se stie si ce se accepta: RMS-ul nu e criteriu de valabilitate
#: (§5.22 - 24 de poze identice dau 0.061 cu fx gresit cu +754%), dar e
#: indicator de calitate. O camera bine calibrata sta tipic la 0.2-0.5;
#: 0.83 sugereaza tinta neplana, poze miscate sau colturi neacoperite
#: (§5.34). Garzile care chiar prind o calibrare degenerata - focala fata
#: de cea geometrica si acoperirea cadrului, in calibrate_camera.py - raman
#: neatinse.
#:
#: Consecinta de urmarit la E2: distanta din solvePnP comanda coborarea,
#: deci eroarea de range fata de ruleta e masuratoarea care spune daca
#: 0.83 e destul.
MAX_REPROJ_ERR_PX = 0.85

#: Detectie (E1.3)
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 26
#: Sub ce distanta se urmareste markerul intr-un ROI centrat pe ultima
#: pozitie. Era 5.0 ("markerul are > 90 px acolo"); zborul din 24.09.2026
#: (§5.62) a aratat ca exact FEREASTRA DE HANDOVER (5-12 m) ramanea pe
#: cautarea in cadrul intreg: 320 ms si det sub 15%, fata de 66 ms si det
#: 77% pe drumul cu ROI. La 12 m markerul are ~40 px si incape lejer in
#: 640x480; ratarea ROI cade oricum pe cadrul intreg, in acelasi apel.
ROI_BELOW_M = 15.0
#: Cautarea in cadrul intreg se face pe imaginea redusa de k ori
#: (INTER_AREA), cu rafinarea colturilor la rezolutia plina intr-un ROI in
#: jurul gasirii. Sub ~5 px pe modul decodarea ArUco pica, deci la 2x
#: limita practica de achizitie e ~65 px pe latura (8-9 m); pentru capatul
#: de sus al portii exista plasa de mai jos. 1 = comportamentul vechi.
SEARCH_DOWNSCALE = 2
#: Plasa de achizitie: la fiecare al N-lea cadru FARA gasire pe imaginea
#: redusa se cauta si la rezolutia plina. Achizitia la 10-12 m dureaza deci
#: pana la N cadre; dupa prima gasire preia ROI-ul, la orice distanta.
SEARCH_FULLRES_EVERY = 4
#: Step 1 (§5.65): above this side (px) the marker is "big" - it no longer
#: fits reliably in the 640x480 ROI once rotated (at 1 m it has 350-500 px),
#: and a ROI miss used to cost 90-330 ms per frame. Big markers are found
#: on the whole frame reduced so the marker lands at ~SEARCH_TARGET_PX,
#: no ROI involved, corners refined at full resolution.
BIG_MARKER_PX = 150
SEARCH_TARGET_PX = 110
#: After this many consecutive misses the remembered marker size is
#: dropped and the search returns to acquisition.
FORGET_SIZE_AFTER_MISSES = 10
#: Peste cat din cadru consideram ca markerul nu mai incape (§5.2, pe cutie).
#: Masurat in Gazebo, detectia tine pana la fill ~0.91 la rotatie zero si
#: ~0.83 la 41 grade, deci pragul asta nu e cel care leaga - `detectMarkers`
#: pica primul. Ramane ca garda explicita: o detectie cu markerul pe jumatate
#: afara nu e o detectie, e o extrapolare a lui solvePnP.
FILL_MAX = 0.95

#: Rotatiile de montaj acceptate - vezi `axe_corp`.
ROTATII_MONTAJ = (0, 90, 180, 270)


def axe_corp(tx, ty, rotatie_deg=0):
    """(inainte, dreapta) in cadrul CORPULUI, din componentele x, y ale
    vectorului spre marker in cadrul CAMEREI (x dreapta pe imagine, y in jos).

    `rotatie_deg` spune cum e montata camera, definit prin ce vezi: cu cate
    grade trebuie rotita imaginea bruta SPRE STANGA (invers acelor de
    ceasornic, pe ecran) ca nasul dronei sa ajunga SUS.

        rotatie   inainte   dreapta
        0         -ty       +tx       conventia initiala: varful imaginii = nas
        90        +tx       +ty       nasul apare in DREAPTA imaginii brute
        180       +ty       -tx       nasul apare JOS
        270       -tx       -ty       nasul apare in STANGA

    DE CE AICI si nu prin rotirea pixelilor. Un cadru 2304x1296 rotit cu 90
    grade devine 1296x2304, iar calibrarea (K, cx, cy, dimensiunea) nu se mai
    potriveste: `build_pi_detector` refuza pornirea, `fill` s-ar calcula fata
    de alt cadru, si fiecare cadru ar plati o copie integrala pe Pi 4. Ce e
    rotit e MONTAJUL, deci se roteste maparea axelor - zero cost, calibrarea
    neatinsa. Doar fereastra de afisare roteste pixelii, pe o copie.

    DE CE AICI si nu prin PLND_YAW_ALIGN. Parametrul ArduPilot ar roti doar
    LANDING_TARGET. Dar `angle_x`/`angle_y` mai intra in poarta de handover,
    in supervizor si in comparatiile cu adevarul din simulare - toate
    presupun cadrul corpului. Rotatia se face o singura data, la sursa, iar
    PLND_YAW_ALIGN RAMANE 0: ambele ar insemna rotatie dubla.

    Nu are legatura cu rotatia din §5.1 (`conv`), care e conventia de axe a
    mesajului LANDING_TARGET, independenta de montaj."""
    r = int(rotatie_deg) % 360
    if r == 0:
        return -ty, tx
    if r == 90:
        return tx, ty
    if r == 180:
        return ty, -tx
    if r == 270:
        return -tx, -ty
    raise ValueError(f"rotatie de montaj {rotatie_deg}: se accepta doar "
                     f"{ROTATII_MONTAJ} (camera e montata in trepte de 90 grade)")
ROI_SIZE_PX = (640, 480)

#: Instrumentare (E1.4): fereastra pe care se calculeaza percentilele.
STATS_WINDOW = 300
#: Faza 4 (refactor/threads): o exceptie in pasul firului de detectie e
#: logata si pasul se reia (dupa o pauza scurta); dupa atatea ESECURI
#: CONSECUTIVE firul se declara mort (B9) si se opreste - o camera care
#: arunca la fiecare cadru nu e "vie" doar pentru ca bucla mai ruleaza.
DETECT_FAIL_MAX = 5
DETECT_FAIL_BACKOFF_S = 0.05
#: Log per fereastra: cadre capturate, procesate, detectii, timp mediu.
DETECT_LOG_WINDOW_S = 5.0
#: Exposure statistics per window (27.09.2026): every frame's gray plane is
#: sampled 1 pixel in SAT_SAMPLE x SAT_SAMPLE (57 600 of 921 600 at
#: 1280x720): the share of saturated (255) and near-black (< DARK_LEVEL)
#: pixels. An estimate, deliberately: counting the whole frame costs
#: detection time on a Pi 4 (the preflight lesson of §5.40).
SAT_SAMPLE = 4
DARK_LEVEL = 10
#: The per-frame detector log (CSV): camera metadata + what detection did.
FRAME_LOG_FIELDS = ('seq', 't_capture', 'SensorTimestamp', 'ExposureTime',
                    'AnalogueGain', 'LensPosition', 'detected', 'marker_px',
                    'proc_ms')


# --- Calibrare ---------------------------------------------------------------

class StageTimer:
    """Per-stage wall-clock timings for the detection pipeline (step 0 of
    the Pi 4 detection work, §5.65). Records durations only; it changes no
    behaviour. Each stage keeps the last `window` samples; `stats()` gives
    p50/p99 in ms. Attached by PiDetector to the source and the detector,
    which record into it only if it is present (`timer is not None`)."""

    STAGES = ('achizitie', 'gri', 'varsta', 'roi', 'scalare', 'detect_mare',
              'detect_redus', 'rafinare', 'detect_plin', 'geometrie', 'ring',
              'total')

    def __init__(self, window=300):
        self.window = window
        self.d = {}

    def add(self, stage, dt_s):
        self.d.setdefault(stage, collections.deque(maxlen=self.window)).append(dt_s)

    @staticmethod
    def _pct(vals, p):
        s = sorted(vals)
        k = (len(s) - 1) * p
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return s[lo] + (s[hi] - s[lo]) * (k - lo)

    def stats(self):
        """{stage: (p50_ms, p99_ms, n)}, stages in pipeline order.

        Called from the main loop while the worker thread adds samples,
        and adds KEYS the first time a stage runs (`detect_mare`,
        `rafinare` on the first big-marker frame, ~1.5-2 m). The old
        generator over `self.d` then raised "dictionary changed size
        during iteration" - out of on_status, which run_loop does not
        guard, so the app would exit mid-descent (B8, 26.09.2026).
        `list(dict)` and `list(deque)` are single C calls under the GIL:
        atomic snapshots, no generator in between."""
        out = {}
        keys = list(self.d)
        for st in self.STAGES + tuple(k for k in keys if k not in self.STAGES):
            v = self.d.get(st)
            if v:
                vals = list(v)
                if vals:
                    out[st] = (1000.0 * self._pct(vals, 0.5),
                               1000.0 * self._pct(vals, 0.99), len(vals))
        return out

    def line(self):
        return 'etape ' + ' | '.join(f"{st} {p50:.0f}/{p99:.0f}"
                                    for st, (p50, p99, _) in self.stats().items())

    def table(self):
        rows = ["etapa           p50 ms   p99 ms      n"]
        for st, (p50, p99, n) in self.stats().items():
            rows.append(f"{st:<14} {p50:8.1f} {p99:8.1f} {n:6d}")
        return '\n'.join(rows)


class CameraCalibration:
    """Matricea intrinseca si coeficientii de distorsiune, cu proveniența.

    Se salveaza/incarca prin cv2.FileStorage (YAML), ca sa nu adaugam pyyaml.
    `load()` REFUZA o calibrare cu eroare de reproiectie peste
    MAX_REPROJ_ERR_PX: la 102 grade FOV distorsiunea radiala e severa la
    margini, iar solvePnP cu coeficienti prosti da erori de pozitie care cresc
    exact acolo unde se afla markerul in timpul apropierii.
    """

    def __init__(self, K, dist, width, height, rms=None, n_images=0,
                 source='necunoscut', meta=None, kind='nativa'):
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.dist = np.asarray(dist, dtype=np.float64).reshape(-1)
        self.width = int(width)
        self.height = int(height)
        self.rms = None if rms is None else float(rms)
        self.n_images = int(n_images)
        self.source = source
        #: How this calibration relates to the file it came from, for the
        #: geometry actually running: 'nativa' (the file itself), 'scalata'
        #: (ISP scale only), 'derivata' (crop mode at the same binning) or
        #: 'derivata+scalata'. Set by calibration_for().
        self.kind = kind
        #: Trasabilitate pentru Compliance Matrix: tipul de tinta, latura
        #: MASURATA a patratului, cate poze au fost acceptate si cate
        #: respinse, LensPosition-ul folosit. Fara ele, fisierul de calibrare
        #: e un set de numere fara proveniența.
        self.meta = dict(meta or {})

    @property
    def fx(self):
        return float(self.K[0, 0])

    @property
    def fy(self):
        return float(self.K[1, 1])

    @property
    def cx(self):
        return float(self.K[0, 2])

    @property
    def cy(self):
        return float(self.K[1, 2])

    def hfov_deg(self):
        return math.degrees(2.0 * math.atan(self.width / (2.0 * self.fx)))

    def vfov_deg(self):
        return math.degrees(2.0 * math.atan(self.height / (2.0 * self.fy)))

    def camera_model(self, marker_size_m=MARKER_SIZE_M):
        """CameraModel din nova/detection.py, derivat din calibrare. Asa
        verificarea de incadrare (§5.2) e aceeasi ca in detectorul sintetic,
        dar cu focala si FOV-ul reale, nu cu cele din fisa tehnica.

        Dimensiunile in PIXELI se dau explicit, nu se lasa pe implicit.
        `fill` si `tilt_budget_deg` se raporteaza la ele, iar implicitul
        (2304x1296) se potriveste doar cu rezolutia de lucru actuala. O
        calibrare facuta la alta rezolutie ar produce incadrari calculate
        fata de un cadru care nu exista - si nimic nu ar semnala asta."""
        return CameraModel(focal_px=self.fy, hfov_deg=self.hfov_deg(),
                           vfov_deg=self.vfov_deg(), marker_size_m=marker_size_m,
                           width_px=float(self.width), height_px=float(self.height))

    def is_real(self):
        return self.rms is not None and self.n_images > 0

    def scaled_to(self, width, height, tol=0.005):
        """The same calibration for a frame of another size, SAME aspect.

        Detection on 1280x720 (team decision 27.09.2026): the sensor still
        runs its 2304x1296 binned mode, the ISP scales the stream down.
        Intrinsics scale with the image (fx, cx by width, fy, cy by
        height); the distortion coefficients act on normalised coordinates
        and stay. A different aspect ratio would mean a crop, not a scale
        - refused, because it would silently move the principal point."""
        width, height = int(width), int(height)
        if (width, height) == (self.width, self.height):
            return self
        sx = width / float(self.width)
        sy = height / float(self.height)
        if abs(sx / sy - 1.0) > tol:
            raise ValueError(
                f"{width}x{height} nu are raportul calibrarii "
                f"{self.width}x{self.height} ({sx:.4f} vs {sy:.4f}): ar fi "
                f"un decupaj, nu o scalare. Recalibreaza la rezolutia asta.")
        K = self.K.copy()
        K[0, 0] *= sx
        K[0, 2] *= sx
        K[1, 1] *= sy
        K[1, 2] *= sy
        meta = dict(self.meta)
        meta['scalata_din'] = f"{self.width}x{self.height}"
        return CameraCalibration(
            K, self.dist, width, height,
            rms=None if self.rms is None else self.rms * sx,
            n_images=self.n_images,
            source=f"{self.source} (scalata {self.width}x{self.height} -> "
                   f"{width}x{height})", meta=meta,
            kind='scalata' if self.kind == 'nativa' else f"{self.kind}+scalata")

    # -- geometry the calibration was made in (27.09.2026) -----------------
    def recorded_geometry(self):
        """(sensor_mode, scaler_crop, legacy): the sensor mode and the
        ScalerCrop the calibration images were taken in. Files written
        before 27.09.2026 do not say; they were all made in the 2304x1296
        binned, full-field mode, so that is what `legacy` = True means."""
        m, c = self.meta.get('sensor_mode'), self.meta.get('scaler_crop')
        if m is None and c is None:
            return LEGACY_CAL_MODE, LEGACY_CAL_CROP, True
        if m is None or c is None:
            raise ValueError(
                f"calibrarea are doar jumatate din geometrie (sensor_mode="
                f"{m}, scaler_crop={c}): refac calibrarea sau completeaza "
                f"fisierul")
        return parse_size(m), parse_rect(c), False

    def set_geometry(self, sensor_mode, scaler_crop):
        """Record the geometry in the meta (saved as meta_sensor_mode,
        meta_scaler_crop next to image_width/height)."""
        w, h = parse_size(sensor_mode)
        self.meta['sensor_mode'] = f"{w}x{h}"
        self.meta['scaler_crop'] = ','.join(str(v) for v in parse_rect(scaler_crop))
        return self

    def cropped(self, dx, dy, width, height, note):
        """The same lens and pixel pitch, a window of the image: the
        principal point moves by the window's offset, nothing else does
        (focal length and distortion are properties of the lens and of the
        pixel size, both unchanged)."""
        K = self.K.copy()
        K[0, 2] -= dx
        K[1, 2] -= dy
        meta = dict(self.meta)
        meta['derivata_din'] = f"{self.width}x{self.height}"
        return CameraCalibration(K, self.dist, width, height, rms=self.rms,
                                 n_images=self.n_images,
                                 source=f"{self.source} ({note})", meta=meta,
                                 kind='derivata')

    @classmethod
    def geometric(cls, width, height, hfov_deg=102.0):
        """Punct de plecare din fisa tehnica: f = W/2 / tan(HFOV/2), fara
        distorsiune. E0/E1.2: NU e un substitut pentru calibrarea reala si
        `load()`-ul de bord nu o accepta. Exista pentru teste si pentru
        unealta de masurare, unde e marcata ca atare."""
        f = (width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        K = [[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]]
        return cls(K, np.zeros(5), width, height, rms=None, n_images=0,
                   source='geometric (fisa tehnica) - NU e calibrare')

    def save(self, path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        fs.write('camera_matrix', self.K)
        fs.write('dist_coeffs', self.dist.reshape(1, -1))
        fs.write('image_width', self.width)
        fs.write('image_height', self.height)
        fs.write('rms_reprojection_px', -1.0 if self.rms is None else self.rms)
        fs.write('n_images', self.n_images)
        fs.write('source', self.source)
        fs.write('hfov_deg', self.hfov_deg())
        fs.write('vfov_deg', self.vfov_deg())
        for key in sorted(self.meta):
            val = self.meta[key]
            if val is None:
                continue
            name = 'meta_' + str(key)
            if isinstance(val, bool):
                fs.write(name, str(val))
            elif isinstance(val, (int, float)):
                fs.write(name, float(val))
            else:
                fs.write(name, str(val))
        fs.release()

    @classmethod
    def load(cls, path, require_real=True, max_rms=MAX_REPROJ_ERR_PX):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"lipseste calibrarea camerei: {path}\n"
                f"  Ruleaza tools/calibrate_camera.py. Fara calibrare reala "
                f"detectorul nu porneste (E1.2).")
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise ValueError(f"{path}: nu se poate citi ca YAML OpenCV")
        K = fs.getNode('camera_matrix').mat()
        dist = fs.getNode('dist_coeffs').mat()
        w = int(fs.getNode('image_width').real())
        h = int(fs.getNode('image_height').real())
        rms = fs.getNode('rms_reprojection_px').real()
        n = int(fs.getNode('n_images').real())
        src = fs.getNode('source').string()
        meta = {}
        root = fs.root()
        for key in (root.keys() if hasattr(root, 'keys') else []):
            if not key.startswith('meta_'):
                continue
            node = fs.getNode(key)
            if node.isString():
                meta[key[5:]] = node.string()
            elif node.isReal() or node.isInt():
                meta[key[5:]] = node.real()
        fs.release()
        if K is None or dist is None or w <= 0 or h <= 0:
            raise ValueError(f"{path}: campuri lipsa sau invalide")
        cal = cls(K, dist, w, h, rms=None if rms < 0 else rms, n_images=n,
                  source=src or path, meta=meta)
        if require_real:
            if not cal.is_real():
                raise ValueError(
                    f"{path}: nu e o calibrare reala (sursa: {cal.source}). "
                    f"E1.2: fara calibrare reala nu rula detectorul.")
            if cal.rms > max_rms:
                raise ValueError(
                    f"{path}: eroare de reproiectie {cal.rms:.3f} px > "
                    f"{max_rms} px. Calibrare proasta, refac.")
        return cal

    def __str__(self):
        rms = '-' if self.rms is None else f"{self.rms:.3f} px"
        return (f"{self.width}x{self.height} fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f} HFOV={self.hfov_deg():.1f} "
                f"VFOV={self.vfov_deg():.1f} rms={rms} n={self.n_images} "
                f"[{self.source}]")


# --- Camera geometry: sensor mode, ScalerCrop, stream (27.09.2026) -----------
#
# Found by `descent_test.sh --check` on the vehicle: the camera was asked for
# a 1280x720 stream without a sensor mode, libcamera picked the 1536x864
# CROPPED mode (the central 3072x1728 of the array, binned 2x2), and the
# calibration was scaled as if the sensor ran the 2304x1296 full-field mode.
# Same 16:9 aspect, so the old "same aspect -> scale" rule let it through:
# fx 577 instead of ~865, angles 1.5x too large, solvePnP distance 1.5x too
# small. Nothing is implicit any more: the mode comes from the config, is
# read back after configure, the ScalerCrop is read from the first frame,
# and a calibration is used only through calibration_for(), which knows
# exactly two legal transforms and refuses everything else.

#: IMX708 active pixel array. ScalerCrop rectangles are in its coordinates.
SENSOR_ARRAY = (4608, 2592)
#: The IMX708 (Camera Module 3) sensor modes, 10 bit, and the region of the
#: array each one reads. Only used to validate the config offline and to
#: say what is expected; at run time the camera's own list (sensor_modes)
#: and the ScalerCrop read back are the truth.
IMX708_MODES = {
    (4608, 2592): (0, 0, 4608, 2592),      # full resolution, ~14 fps
    (2304, 1296): (0, 0, 4608, 2592),      # binned 2x2, full field, ~56 fps
    (1536, 864): (768, 432, 3072, 1728),   # binned 2x2, central crop, ~120 fps
}
SENSOR_BIT_DEPTH = 10
#: What a calibration file without recorded geometry was made in: every
#: calibration before 27.09.2026 (tools/calibrate_camera.py opened the
#: camera at 2304x1296, which libcamera serves from the full-field mode).
LEGACY_CAL_MODE = (2304, 1296)
LEGACY_CAL_CROP = (0, 0, 4608, 2592)


class CameraModeError(ValueError):
    """The camera is not (or cannot be) in the sensor mode the config asks
    for. A ValueError: the onboard app refuses cleanly, it does not crash."""


class CalibrationMismatch(ValueError):
    """The calibration file cannot be used for the geometry running."""


def parse_size(v):
    """(w, h) from [w, h], (w, h) or 'WxH'."""
    if isinstance(v, str):
        parts = v.lower().split('x')
    else:
        parts = list(v)
    if len(parts) != 2:
        raise ValueError(f"dimensiune invalida: {v!r} (se astepta [latime, inaltime])")
    w, h = (int(float(p)) for p in parts)
    if w <= 0 or h <= 0:
        raise ValueError(f"dimensiune invalida: {v!r}")
    return w, h


def parse_rect(v):
    """(x, y, w, h) from a 4-sequence or 'x,y,w,h'."""
    parts = v.split(',') if isinstance(v, str) else list(v)
    if len(parts) != 4:
        raise ValueError(f"dreptunghi invalid: {v!r} (se astepta x, y, w, h)")
    x, y, w, h = (int(float(p)) for p in parts)
    if w <= 0 or h <= 0 or x < 0 or y < 0:
        raise ValueError(f"dreptunghi invalid: {v!r}")
    return x, y, w, h


class CameraGeometry:
    """What the detector's pixels are: the sensor mode (binned pixels), the
    region of the array the image covers (ScalerCrop, array pixels) and the
    size of the stream the ISP delivers."""

    __slots__ = ('sensor_mode', 'scaler_crop', 'output_size')

    def __init__(self, sensor_mode, scaler_crop, output_size):
        self.sensor_mode = parse_size(sensor_mode)
        self.scaler_crop = parse_rect(scaler_crop)
        self.output_size = parse_size(output_size)

    @property
    def binning(self):
        """Array pixels per sensor-mode pixel, (x, y)."""
        return (self.scaler_crop[2] / float(self.sensor_mode[0]),
                self.scaler_crop[3] / float(self.sensor_mode[1]))

    def __eq__(self, other):
        return (isinstance(other, CameraGeometry)
                and (self.sensor_mode, self.scaler_crop, self.output_size)
                == (other.sensor_mode, other.scaler_crop, other.output_size))

    def __repr__(self):
        return f"CameraGeometry({self.describe()})"

    def describe(self):
        m, c, o = self.sensor_mode, self.scaler_crop, self.output_size
        return (f"mod {m[0]}x{m[1]}, ScalerCrop ({c[0]}, {c[1]}, {c[2]}, "
                f"{c[3]}), flux {o[0]}x{o[1]}")


def _check_sensor_mode(raw, where='config'):
    if raw is None:
        raise CameraModeError(
            f"{where}: lipseste `sensor_mode` (modul senzorului, ex. [1536, 864]). "
            "Fara el libcamera alege singur modul - asa a zburat 1536x864 cu "
            "calibrarea de 2304x1296 (27.09.2026).")
    try:
        mode = parse_size(raw)
    except (TypeError, ValueError) as e:
        raise CameraModeError(f"{where}: `sensor_mode` invalid: {e}") from e
    if mode not in IMX708_MODES:
        disp = ', '.join(f"{w}x{h}" for w, h in IMX708_MODES)
        raise CameraModeError(
            f"{where}: modul {mode[0]}x{mode[1]} nu exista pe IMX708 "
            f"(moduri: {disp})")
    return mode


def sensor_mode_from_config(cfg):
    """The sensor mode the config asks for (camera_settings: preset + keys).
    Mandatory: without it libcamera chooses, and that is exactly how
    1536x864 flew with a 2304x1296 calibration. Checked against the IMX708
    modes (a typo must fail here, not as 'the camera would not start' on
    the field)."""
    return camera_settings(cfg).sensor_mode


def geometric_focal_px(image_width, scaler_crop=LEGACY_CAL_CROP,
                       hfov_full_deg=102.0):
    """The datasheet focal length, in pixels of an image `image_width` wide
    that covers `scaler_crop` of the array. The lens's nominal field spans
    the WHOLE array; a crop mode sees less of it with the same pixels, so
    its focal length in pixels is the full field's, not a narrower lens's.
    A plausibility reference for calibrate_camera, never a calibration."""
    f_array = (SENSOR_ARRAY[0] / 2.0) / math.tan(math.radians(hfov_full_deg / 2.0))
    return f_array * image_width / float(parse_rect(scaler_crop)[2])


def output_size_from_config(cfg, sensor_mode):
    """The stream size of camera_settings(cfg) - `output_size` (or the old
    `track_size`), else the mode's own size."""
    try:
        s = camera_settings(cfg)
        if s.sensor_mode == tuple(sensor_mode):
            return s.output_size
    except CameraModeError:
        pass
    ts = cfg.get('output_size') or cfg.get('track_size')
    return parse_size(ts) if ts else tuple(sensor_mode)


# --- Camera settings: config keys and presets (27.09.2026) --------------------

#: Exposure modes.
#:   fixed        ExposureTime / AnalogueGain as given (AE off)
#:   auto_lock    AE converges ~1 s on the scene, then is LOCKED, exposure
#:                capped at AUTOEXP_MAX_US (moving the rest into gain) - the
#:                behaviour flown since 24.09.2026 (`camera_auto_expose`)
#:   auto         continuous AE, libcamera's own limits (trackerV2.py)
#:   auto_capped  continuous AE held inside [exposure_max_us, gain_max] by
#:                a custom AGC exposure mode in the camera tuning (libcamera
#:                has no plain "max exposure" control; see _capped_tuning)
EXPOSURE_MODES = ('fixed', 'auto_lock', 'auto', 'auto_capped')

#: The keys (config/nova.json) and their defaults when neither a preset nor
#: the file sets them. `sensor_mode` has NO default: it is mandatory.
CAMERA_KEY_DEFAULTS = {
    'sensor_mode': None,
    'output_size': None,          # None = the sensor mode's own size
    'exposure': 'fixed',
    'exposure_us': 2000,
    'analogue_gain': 8.0,
    'exposure_max_us': None,      # auto_capped only, mandatory there
    'gain_max': None,             # auto_capped only, mandatory there
    'awb': False,
    'lens_position': 1.63,
}
#: Proposed limits for auto_capped, said in the refusal when they are
#: missing: 2000 us is the blur ceiling (CAMERA_CONTROLS comment: < 1 px of
#: motion over the whole descent profile), 16 the IMX708's analogue maximum.
AUTO_CAPPED_PROPOSAL = {'exposure_max_us': 2000, 'gain_max': 16.0}

#: One key (config `camera_preset`) or one option (--preset / --camera-preset)
#: selects all of it. crop1280 is what flew on 27.09.2026 - same mode, same
#: stream, same exposure behaviour - now with the calibration derived right.
CAMERA_PRESETS = {
    'crop1280': {'sensor_mode': [1536, 864], 'output_size': [1280, 720],
                 'exposure': 'auto_lock', 'awb': False, 'lens_position': 1.63},
    'crop1536': {'sensor_mode': [1536, 864], 'output_size': [1536, 864],
                 'exposure': 'auto_lock', 'awb': False, 'lens_position': 1.63},
    'full1280': {'sensor_mode': [2304, 1296], 'output_size': [1280, 720],
                 'exposure': 'auto_lock', 'awb': False, 'lens_position': 1.63},
    'trackerv2': {'sensor_mode': [1536, 864], 'output_size': [1280, 720],
                  'exposure': 'auto', 'awb': True, 'lens_position': 1.0},
}


class CameraSettings:
    """Everything the camera is opened with, validated, and where each value
    came from (for the log: a value nobody chose must be visible)."""

    FIELDS = tuple(CAMERA_KEY_DEFAULTS)

    def __init__(self, values, preset=None, origin=None):
        self.preset = preset
        self.origin = dict(origin or {})
        v = dict(values)
        self.sensor_mode = _check_sensor_mode(v.get('sensor_mode'),
                                              f"preset {preset}" if preset else 'config')
        out = v.get('output_size')
        self.output_size = parse_size(out) if out else self.sensor_mode
        self.exposure = v.get('exposure')
        if self.exposure not in EXPOSURE_MODES:
            raise CameraModeError(
                f"`exposure` = {self.exposure!r}; valori: {', '.join(EXPOSURE_MODES)}")
        self.exposure_us = int(v.get('exposure_us'))
        self.analogue_gain = float(v.get('analogue_gain'))
        self.exposure_max_us = v.get('exposure_max_us')
        self.gain_max = v.get('gain_max')
        if self.exposure == 'auto_capped':
            lipsa = [k for k in ('exposure_max_us', 'gain_max')
                     if not v.get(k) or float(v.get(k)) <= 0]
            if lipsa:
                raise CameraModeError(
                    f"`exposure: auto_capped` fara limite ({', '.join(lipsa)}): "
                    f"fara ele e doar `auto`. Propunere: "
                    f"exposure_max_us {AUTO_CAPPED_PROPOSAL['exposure_max_us']} "
                    f"(plafonul de blur), gain_max {AUTO_CAPPED_PROPOSAL['gain_max']:g}")
            self.exposure_max_us = int(v['exposure_max_us'])
            self.gain_max = float(v['gain_max'])
        self.awb = bool(v.get('awb'))
        self.lens_position = float(v.get('lens_position'))
        if self.exposure_us <= 0 or self.analogue_gain <= 0 or self.lens_position < 0:
            raise CameraModeError("expunere / gain / focus invalide")

    def describe(self):
        m, o = self.sensor_mode, self.output_size
        if self.exposure == 'fixed':
            exp = f"fixa {self.exposure_us} us gain {self.analogue_gain:g}"
        elif self.exposure == 'auto_lock':
            exp = f"masurata apoi blocata (plafon {AUTOEXP_MAX_US} us)"
        elif self.exposure == 'auto':
            exp = 'automata continua'
        else:
            exp = (f"automata <= {self.exposure_max_us} us, gain <= "
                   f"{self.gain_max:g}")
        return (f"preset {self.preset or '-'}: mod {m[0]}x{m[1]} -> flux "
                f"{o[0]}x{o[1]}, expunere {exp}, AWB {'da' if self.awb else 'nu'}, "
                f"LensPosition {self.lens_position:g}")


def camera_settings(cfg, preset=None):
    """The camera settings for this run.

    `preset` (command line) selects a preset EXACTLY: the file's camera
    keys do not apply - what was asked for on the command line is what
    runs. Otherwise: key defaults <- the file's `camera_preset` <- the
    file's own keys (non-null). The old keys still work: `track_size` for
    `output_size`, `camera_auto_expose` true/false for auto_lock/fixed."""
    values = dict(CAMERA_KEY_DEFAULTS)
    origin = {k: 'implicit' for k in values}

    def apply(d, who):
        for k, val in d.items():
            if k in values and val is not None:
                values[k] = val
                origin[k] = who

    if preset is not None:
        if preset not in CAMERA_PRESETS:
            raise CameraModeError(
                f"presetul {preset!r} nu exista ({', '.join(CAMERA_PRESETS)})")
        apply(CAMERA_PRESETS[preset], f"preset {preset}")
        return CameraSettings(values, preset=preset, origin=origin)

    name = cfg.get('camera_preset')
    if name is not None:
        if name not in CAMERA_PRESETS:
            raise CameraModeError(
                f"config: presetul {name!r} nu exista ({', '.join(CAMERA_PRESETS)})")
        apply(CAMERA_PRESETS[name], f"preset {name}")
    legacy = {}
    if cfg.get('output_size') is None and cfg.get('track_size'):
        legacy['output_size'] = cfg['track_size']
    if cfg.get('exposure') is None and cfg.get('camera_auto_expose') is not None:
        legacy['exposure'] = 'auto_lock' if cfg['camera_auto_expose'] else 'fixed'
    apply(legacy, 'config (cheie veche)')
    apply({k: cfg.get(k) for k in CAMERA_KEY_DEFAULTS}, 'config')
    return CameraSettings(values, preset=name, origin=origin)


def cap_agc_tuning(agc, exposure_max_us, gain_max):
    """Add a 'custom' AGC exposure mode to the rpi.agc parameters of a
    camera tuning: exposure up to `exposure_max_us` at gain 1, then gain up
    to `gain_max`. The Raspberry Pi AGC never goes beyond the last entries
    of the mode it runs, so with AeExposureMode = Custom continuous AE stays
    inside the caps. Handles both tuning layouts (one AGC, or 'channels' in
    the newer libcamera) and both key names ('shutter' / 'exposure').
    Returns how many channels were changed."""
    chans = agc['channels'] if 'channels' in agc else [agc]
    n = 0
    for ch in chans:
        modes = ch.get('exposure_modes')
        if not modes:
            continue
        ref = modes.get('normal') or next(iter(modes.values()))
        key = next((k for k in ('shutter', 'exposure') if k in ref), None)
        if key is None:
            raise CameraModeError(
                f"tuning: modul de expunere nu are 'shutter'/'exposure' ({sorted(ref)})")
        lo = min(float(ref[key][0]), float(exposure_max_us))
        modes['custom'] = {key: [lo, float(exposure_max_us), float(exposure_max_us)],
                           'gain': [1.0, 1.0, float(gain_max)]}
        n += 1
    if n == 0:
        raise CameraModeError("tuning: rpi.agc fara exposure_modes")
    return n


def capped_tuning(Picamera2, exposure_max_us, gain_max, model=None):
    """The camera's own tuning file with the capped 'custom' exposure mode
    (for Picamera2(tuning=...)). The file is the sensor's (imx708_wide.json
    for the Camera Module 3 Wide), found by picamera2 itself."""
    if model is None:
        info = Picamera2.global_camera_info() or [{}]
        model = info[0].get('Model')
    if not model:
        raise CameraModeError("auto_capped: nu stiu modelul senzorului (tuning)")
    try:
        tuning = Picamera2.load_tuning_file(f"{model}.json")
    except RuntimeError as e:
        raise CameraModeError(f"auto_capped: tuning {model}.json negasit: {e}") from e
    cap_agc_tuning(Picamera2.find_tuning_algo(tuning, 'rpi.agc'),
                   exposure_max_us, gain_max)
    return tuning


def calibration_for(cal, geom):
    """The calibration to use for `geom`, or CalibrationMismatch.

    Two transforms are legal, nothing else:

      scale    ISP scaling only - same sensor mode AND same ScalerCrop as
               the calibration, same aspect ratio. Intrinsics scale with
               the image, distortion stays.
      derive   a CROP mode at the same binning, from a calibration made
               over the full field at its mode's own size: the pixels are
               the same pixels, so fx, fy and the distortion stay; the
               principal point moves by the crop offset, READ from the
               ScalerCrop (in binned pixels: offset / binning). Then, if
               the stream is smaller, a scale as above.

    The result carries `kind` ('nativa', 'scalata', 'derivata',
    'derivata+scalata') and the geometry it is valid for."""
    cmode, ccrop, legacy = cal.recorded_geometry()
    csize = (cal.width, cal.height)
    mode, crop, out = geom.sensor_mode, geom.scaler_crop, geom.output_size
    orig = (f"calibrarea {csize[0]}x{csize[1]} (mod {cmode[0]}x{cmode[1]}, "
            f"ScalerCrop {ccrop}{', implicit - fisier fara geometrie' if legacy else ''})")
    refa = (f"Refa calibrarea in modul {mode[0]}x{mode[1]} (tools/"
            f"calibrate_camera.py --live cu acelasi `sensor_mode`).")

    if mode == cmode and crop == ccrop:
        # `kind` is relative to the calibration handed in, whatever it was
        # derived from before: its own geometry = native to it.
        res = CameraCalibration(cal.K, cal.dist, cal.width, cal.height,
                                rms=cal.rms, n_images=cal.n_images,
                                source=cal.source, meta=cal.meta,
                                kind='nativa')
        if out != csize:
            try:
                res = res.scaled_to(*out)
            except ValueError as e:
                raise CalibrationMismatch(
                    f"{orig} nu se poate scala la {out[0]}x{out[1]}: {e}") from e
        return res.set_geometry(mode, crop)

    bx, by = geom.binning
    cbx, cby = ccrop[2] / float(cmode[0]), ccrop[3] / float(cmode[1])
    motive = []
    if ccrop != (0, 0) + SENSOR_ARRAY:
        motive.append("calibrarea nu e facuta pe campul intreg")
    if csize != cmode:
        motive.append("calibrarea e deja scalata (nu e la dimensiunea modului ei)")
    if abs(bx - cbx) > 1e-6 or abs(by - cby) > 1e-6:
        motive.append(f"binning diferit ({bx:g}x{by:g} fata de {cbx:g}x{cby:g}"
                      f": ScalerCrop {crop} pe modul {mode[0]}x{mode[1]})")
    if not (crop[0] >= ccrop[0] and crop[1] >= ccrop[1]
            and crop[0] + crop[2] <= ccrop[0] + ccrop[2]
            and crop[1] + crop[3] <= ccrop[1] + ccrop[3]):
        motive.append(f"decupajul {crop} iese din campul calibrarii {ccrop}")
    if motive:
        raise CalibrationMismatch(
            f"{orig} nu se poate folosi pentru {geom.describe()}: "
            f"{'; '.join(motive)}. {refa}")

    dx = (crop[0] - ccrop[0]) / bx
    dy = (crop[1] - ccrop[1]) / by
    res = cal.cropped(dx, dy, mode[0], mode[1],
                      f"derivata pentru modul {mode[0]}x{mode[1]}, "
                      f"ScalerCrop {crop}: centrul optic -{dx:g}, -{dy:g} px")
    res.set_geometry(mode, crop)
    if out != mode:
        try:
            res = res.scaled_to(*out)
        except ValueError as e:
            raise CalibrationMismatch(
                f"calibrarea derivata {mode[0]}x{mode[1]} nu se poate scala "
                f"la {out[0]}x{out[1]}: {e}") from e
    return res.set_geometry(mode, crop)


# --- Detectia ------------------------------------------------------------------

class ArucoMarkerDetector:
    """Un cadru gri -> Detection sau None. Numai OpenCV.

    solvePnP cu SOLVEPNP_IPPE_SQUARE: solutie analitica pentru un patrat
    plan, fara ambiguitatea metodei iterative si mai rapida. Cere punctele
    obiect in ordinea colturilor ArUco (stanga-sus, dreapta-sus,
    dreapta-jos, stanga-jos), centrate in origine, in planul z=0.
    """

    def __init__(self, calib, marker_id=MARKER_ID, marker_size_m=MARKER_SIZE_M,
                 roi_below_m=ROI_BELOW_M, roi_size_px=ROI_SIZE_PX,
                 camera_rotation_deg=0, search_downscale=SEARCH_DOWNSCALE):
        self.calib = calib
        # Validat aici, la constructie, nu la primul cadru: o valoare gresita
        # trebuie sa opreasca pornirea, nu sa apara in mijlocul unui zbor.
        axe_corp(0.0, 0.0, camera_rotation_deg)
        self.camera_rotation_deg = int(camera_rotation_deg) % 360
        self.marker_id = int(marker_id)
        self.marker_size_m = float(marker_size_m)
        self.roi_below_m = float(roi_below_m)
        self.search_downscale = int(search_downscale)
        self._search_tick = 0
        #: StageTimer, attached by PiDetector; None = no timing overhead.
        self.timer = None
        self.roi_size_px = tuple(int(x) for x in roi_size_px)
        self.cam = calib.camera_model(self.marker_size_m)

        # Step 4 (§5.65): a second detector for TRACKING, when the expected
        # marker size is known. One adaptive-threshold window instead of
        # three (adaptiveThreshWinSizeMin == Max) and perimeter limits set
        # from the expected side, so the candidate list is short. The
        # acquisition detector below keeps default parameters: with an
        # unknown size, narrowing would cost detections, not time.
        self._track_params = cv2.aruco.DetectorParameters()
        self._track_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._track_params.adaptiveThreshWinSizeMin = 23
        self._track_params.adaptiveThreshWinSizeMax = 23
        self.detector_track = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT), self._track_params)
        self.last_side_px = None      # step 1: size of the previous marker
        self.n_big = 0                # step 1: hits on the reduced whole frame
        params = cv2.aruco.DetectorParameters()
        # Colturi sub-pixel: la 30 px, o eroare de 0.5 px pe colt inseamna
        # ~1.7% pe latura, deci ~1.7% pe distanta. Merita costul.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        s = self.marker_size_m / 2.0
        self.obj_points = np.array([[-s, s, 0], [s, s, 0],
                                    [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

        # starea pentru ROI
        self.last_center = None
        self.last_range_m = None
        #: Colturile ultimei detectii (cadru intreg, ordinea ArUco). Nu intra
        #: in Detection - contractul cu masina de stari ramane neschimbat -
        #: dar tools/compare_detectors.py are nevoie de ele ca sa compare
        #: cele doua implementari pe colturi, nu doar pe distanta.
        self.last_corners = None

        # contoare
        #: (cadre procesate, cadre cu detectie), ca o SINGURA valoare.
        #:
        #: Doua intregi actualizate separat nu se pot citi consistent: intre
        #: `n_frames += 1` la intrarea in `detect()` si `n_detected += 1` la
        #: iesire trece tot calculul, ~4 ms, si orice citire cazuta acolo
        #: vede un cadru fara detectie care de fapt are una. Bucla din
        #: `nova_sim` citeste de sute de ori pe secunda, deci cade acolo
        #: des - si asa fiecare detectie primea o ratare fantoma, cu rata
        #: raportata la 50% pe un detector care vedea markerul mereu.
        #:
        #: Perechea se inlocuieste printr-o singura atribuire, dupa ce
        #: rezultatul e cunoscut. Cititorul vede ori vechea valoare, ori pe
        #: cea noua, niciodata o combinatie.
        self._counts = (0, 0)
        self.n_roi = 0
        self.n_roi_miss = 0
        self.miss_streak = 0     # cadre consecutive fara marker
        self.n_half = 0          # gasiri pe imaginea redusa
        self.n_full = 0          # gasiri pe plasa de rezolutie plina
        self.last_lum = None
        self.n_rejected_fit = 0
        self.n_rejected_id = 0

    # -- ROI ---------------------------------------------------------------
    def _roi(self, shape):
        """(x0, y0, x1, y1) daca merita ROI, altfel None."""
        if (self.last_center is None or self.last_range_m is None
                or self.last_range_m >= self.roi_below_m
                or (self.last_side_px is not None
                    and self.last_side_px >= BIG_MARKER_PX)):
            return None
        h, w = shape[:2]
        rw, rh = self.roi_size_px
        cx, cy = self.last_center
        x0 = int(min(max(cx - rw / 2, 0), max(w - rw, 0)))
        y0 = int(min(max(cy - rh / 2, 0), max(h - rh, 0)))
        return x0, y0, min(x0 + rw, w), min(y0 + rh, h)

    def _t(self, stage, t0):
        """Record one stage duration, if a timer is attached."""
        if self.timer is not None:
            self.timer.add(stage, time.perf_counter() - t0)

    def _find(self, gray):
        """(colturi 4x2 in coordonatele cadrului intreg, folosit_roi)"""
        # Step 1 (§5.65): big marker -> whole frame reduced to ~target px,
        # no ROI. Cost is independent of how far the marker moved between
        # frames, which is exactly what a ROI could not offer at 1 m.
        side = self.last_side_px
        if side is not None and side >= BIG_MARKER_PX:
            k = max(2, min(8, int(round(side / SEARCH_TARGET_PX))))
            t0 = time.perf_counter()
            small = cv2.resize(gray, None, fx=1.0 / k, fy=1.0 / k,
                               interpolation=cv2.INTER_AREA)
            self._t('scalare', t0)
            t0 = time.perf_counter()
            c_small = self._detect_id(small, expected_side_px=side / k)
            self._t('detect_mare', t0)
            if c_small is not None:
                self.n_big += 1
                t0 = time.perf_counter()
                c = self._refine_subpix(gray, c_small * float(k), k)
                self._t('rafinare', t0)
                return c, False
            # not where it was expected: fall through to acquisition

        roi = self._roi(gray.shape)
        if roi is not None:
            x0, y0, x1, y1 = roi
            t0 = time.perf_counter()
            corners = self._detect_id(gray[y0:y1, x0:x1],
                                      expected_side_px=self.last_side_px)
            self._t('roi', t0)
            if corners is not None:
                self.n_roi += 1
                corners = corners + np.array([x0, y0], dtype=np.float32)
                return corners, True
            self.n_roi_miss += 1
            self.last_center = None          # cadrul intreg data viitoare

        # Cautarea in cadrul intreg. La rezolutia plina (2304x1296) ia
        # ~320 ms pe Pi 4 si in zbor a ratat majoritatea cadrelor la 5-7 m
        # (§5.62) - exact fereastra de handover. Redusa de k ori, cautarea
        # scade la ~80 ms, iar colturile se rafineaza la rezolutia plina
        # intr-un decupaj in jurul gasirii - acelasi drum care in zbor a
        # dat 66 ms si det 77%.
        k = self.search_downscale
        if k <= 1:
            t0 = time.perf_counter()
            c = self._detect_id(gray)
            self._t('detect_plin', t0)
            return c, False
        t0 = time.perf_counter()
        small = cv2.resize(gray, None, fx=1.0 / k, fy=1.0 / k,
                           interpolation=cv2.INTER_AREA)
        self._t('scalare', t0)
        t0 = time.perf_counter()
        c_small = self._detect_id(small)
        self._t('detect_redus', t0)
        if c_small is not None:
            self.n_half += 1
            c_scaled = c_small * float(k)
            t0 = time.perf_counter()
            refined = self._refine_full(gray, c_scaled)
            self._t('rafinare', t0)
            # Rafinarea poate rata (rar: marginea cadrului). Colturile
            # scalate au eroare ~k/2 px - acceptabil ca rezerva, si tot
            # trece prin solvePnP si prin gardurile de incadrare.
            return (refined if refined is not None else c_scaled), False
        self._search_tick += 1
        if self._search_tick >= SEARCH_FULLRES_EVERY:
            self._search_tick = 0
            t0 = time.perf_counter()
            c = self._detect_id(gray)
            self._t('detect_plin', t0)
            if c is not None:
                self.n_full += 1
            return c, False
        return None, False

    def _refine_full(self, gray, c_scaled):
        """Colturile la rezolutia plina, dintr-un decupaj in jurul gasirii
        de pe imaginea redusa. Decupajul acopera markerul cu marja, oricat
        de mare ar fi el in cadru."""
        h, w = gray.shape[:2]
        x_min, y_min = c_scaled.min(axis=0)
        x_max, y_max = c_scaled.max(axis=0)
        rw = max(self.roi_size_px[0], int((x_max - x_min) * 1.6))
        rh = max(self.roi_size_px[1], int((y_max - y_min) * 1.6))
        cx, cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        x0 = int(min(max(cx - rw / 2, 0), max(w - rw, 0)))
        y0 = int(min(max(cy - rh / 2, 0), max(h - rh, 0)))
        c = self._detect_id(gray[y0:min(y0 + rh, h), x0:min(x0 + rw, w)])
        if c is None:
            return None
        return c + np.array([x0, y0], dtype=np.float32)

    @staticmethod
    def _refine_subpix(gray, corners, k):
        """Corners found on a 1/k image, refined on the full-resolution
        frame with cornerSubPix (what CORNER_REFINE_SUBPIX does inside
        ArUco). Window ~ the scale factor: the scaled corner is within
        ~k/2 px of the true one."""
        c = np.ascontiguousarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        win = int(max(3, k + 2))
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        cv2.cornerSubPix(gray, c, (win, win), (-1, -1), crit)
        return c.reshape(4, 2)

    def _detect_id(self, img, expected_side_px=None):
        if expected_side_px is not None and expected_side_px > 0:
            # Step 4: tracking parameters from the expected size. Perimeter
            # rates are relative to the larger image dimension.
            maxdim = float(max(img.shape[:2]))
            perim = 4.0 * float(expected_side_px)
            p = self._track_params
            p.minMarkerPerimeterRate = max(0.01, 0.4 * perim / maxdim)
            p.maxMarkerPerimeterRate = min(4.0, max(0.1, 2.5 * perim / maxdim))
            self.detector_track.setDetectorParameters(p)
            corners, ids, _ = self.detector_track.detectMarkers(img)
        else:
            corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if int(i) == self.marker_id:
                return c.reshape(4, 2).astype(np.float32)
        self.n_rejected_id += 1
        return None

    # -- geometrie ---------------------------------------------------------
    @staticmethod
    def side_px(corners):
        """Latura medie a patrulaterului detectat (E1.3: marker_px)."""
        d = np.roll(corners, -1, axis=0) - corners
        return float(np.mean(np.linalg.norm(d, axis=1)))

    def detect(self, gray, t_capture):
        """Detection sau None. `gray` e cadrul intreg, uint8, un canal.

        Contoarele se actualizeaza AICI, dupa ce rezultatul e cunoscut, si
        printr-o singura atribuire - vezi `_counts`."""
        # Luminozitatea medie, subesantionata (~12k px, cost neglijabil).
        # Zborul din 24.09.2026 nu avea NICIO cifra despre expunere in log,
        # iar cauza probabila a detectiei intermitente era chiar imaginea
        # supraexpusa. ~250 = alb ars; ~10 = beznav; util 60-180.
        if gray is not None:
            self.last_lum = int(gray[::16, ::16].mean())
        det = self._detect(gray, t_capture)
        cadre, gasite = self._counts
        self._counts = (cadre + 1, gasite + (det is not None))
        # Ratari CONSECUTIVE - regula de abort a echipei (safety,
        # DETECTION_MAX_MISSES). O singura atribuire, citita din alt fir.
        self.miss_streak = 0 if det is not None else self.miss_streak + 1
        if self.miss_streak >= FORGET_SIZE_AFTER_MISSES:
            self.last_side_px = None      # back to acquisition search
        return det

    def _detect(self, gray, t_capture):
        self.last_corners = None
        corners, used_roi = self._find(gray)
        if corners is None:
            return None
        self.last_corners = corners
        t_geo = time.perf_counter()
        try:
            return self._geometry(gray, corners, t_capture)
        finally:
            self._t('geometrie', t_geo)

    def _geometry(self, gray, corners, t_capture):
        """Fill check, solvePnP and axis convention - everything after the
        corners are known. Split out only so it can be timed as one stage."""

        marker_px = self.side_px(corners)
        self.last_side_px = marker_px
        # Cat din cadru ocupa CUTIA colturilor. Se ia din forma reala a
        # cadrului (gray.shape), nu din VFOV-ul de fisa tehnica: cele doua
        # difera cu ~5% la 2304x1296, iar aici masuram pixeli, nu unghiuri
        # (§2 - "valoarea care conteaza operational e cea din calibrare").
        h_px, w_px = gray.shape[:2]
        fill = fill_from_corners(corners, w_px, h_px)
        # §5.2: markerul trebuie sa incapa INTREG in cadru. Sub ~0.38 m nu
        # mai incape, si atunci detectorul tace, prin constructie - de aici
        # exceptia FINAL_DESCENT din supervizor.
        #
        # Criteriul se ia din `fill` cand exista, adica din cutia masurata,
        # nu din latura: un patrat rotit iese din cadru cu pana la 41% mai
        # devreme decat spune latura lui (§5.49). `fits_in_frame(marker_px)`
        # ramane pentru cazul in care forma cadrului nu e cunoscuta.
        if fill is not None:
            incape = fill <= FILL_MAX
        else:
            incape = self.cam.fits_in_frame(marker_px)
        if not incape:
            self.n_rejected_fit += 1
            self.last_center = None
            return None

        ok, rvec, tvec = cv2.solvePnP(self.obj_points, corners, self.calib.K,
                                      self.calib.dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        t = tvec.reshape(3)
        if t[2] <= 0.05:
            return None                       # in spatele camerei / degenerat

        # Conventia de montaj: cu rotatie 0, varful imaginii e nasul dronei,
        # deci inainte = -y_cam, dreapta = +x_cam. Altfel vezi `axe_corp`.
        inainte, dreapta = axe_corp(t[0], t[1], self.camera_rotation_deg)
        angle_x = math.atan2(inainte, t[2])
        angle_y = math.atan2(dreapta, t[2])
        distance = float(np.linalg.norm(t))

        # range_m = ce ar citi un telemetru pe axa optica pana la PLANUL
        # markerului (nu pana la marker). Planul: normala n = R[:,2], trece
        # prin t. Raza pe axa z loveste planul la d = (t.n) / n_z. ArduPilot
        # inmulteste apoi cu cos(tilt) al vehiculului (§5.3), deci nu
        # corectam noi. Daca planul e aproape paralel cu axa (n_z ~ 0), ceva
        # e foarte gresit; cadem pe adancimea markerului.
        R, _ = cv2.Rodrigues(rvec)
        n = R[:, 2]
        if abs(n[2]) > 0.2:
            range_m = float(np.dot(t, n) / n[2])
        else:
            range_m = float(t[2])
        if range_m <= 0.0:
            range_m = float(t[2])

        # ExtNav: the marker's orientation in the body frame, from the
        # direction of its top edge (corner 0 -> corner 1 in ArUco order)
        # in the image, mapped through the same mounting rotation as the
        # position. Angle from the body RIGHT axis, positive clockwise seen
        # from above. Only the yaw setpoint uses it (nova/extnav.py).
        dx, dy = (corners[1] - corners[0]).tolist()
        e_fwd, e_right = axe_corp(dx, dy, self.camera_rotation_deg)
        marker_yaw_deg = math.degrees(math.atan2(-e_fwd, e_right))

        self.last_center = tuple(corners.mean(axis=0))
        self.last_range_m = range_m
        return Detection(t=t_capture, angle_x=angle_x, angle_y=angle_y,
                         distance_m=distance, marker_px=marker_px,
                         range_m=range_m, fill=fill,
                         marker_yaw_deg=marker_yaw_deg)

    def counts(self):
        """(cadre procesate, cadre cu detectie), citite ATOMIC."""
        return self._counts

    @property
    def n_frames(self):
        return self._counts[0]

    @property
    def n_detected(self):
        return self._counts[1]

    def stats(self):
        cadre, gasite = self._counts
        return {
            'frames': cadre,
            'detected': gasite,
            'detection_rate': (gasite / cadre if cadre else 0.0),
            'roi_hits': self.n_roi,
            'roi_misses': self.n_roi_miss,
            'half_hits': self.n_half,
            'full_hits': self.n_full,
            'big_hits': self.n_big,
            'rejected_fit': self.n_rejected_fit,
            'rejected_other_id': self.n_rejected_id,
            'lum': self.last_lum,
        }


# --- Surse de cadre ------------------------------------------------------------

class FrameSource:
    """Interfata: read() -> (gray uint8 HxW, t_captura in secunde monotonic)
    sau None cand nu (mai) exista cadre. close() elibereaza resursele."""

    nominal_fps = None

    def read(self):
        raise NotImplementedError

    def close(self):
        pass


class ArraySource(FrameSource):
    """Cadre din memorie. Pentru teste si masuratori de banc.

    timestamps=None -> fiecare cadru primeste time.monotonic() la citire, ca
    o sursa live; latenta masurata de PiDetector e atunci timpul de detectie
    propriu-zis. Cu timestamps date, se folosesc ca atare."""

    def __init__(self, frames, timestamps=None, fps=30.0):
        self.frames = list(frames)
        self.nominal_fps = fps
        self.timestamps = None if timestamps is None else list(timestamps)
        self.i = 0

    def read(self):
        if self.i >= len(self.frames):
            return None
        f = self.frames[self.i]
        t = (time.monotonic() if self.timestamps is None
             else self.timestamps[self.i])
        self.i += 1
        return f, t


class ImageDirSource(FrameSource):
    """Director de imagini (E2: tools/run_e2.py). Timestamp-urile
    sunt sintetice, echidistante, doar ca sa existe."""

    EXT = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')

    def __init__(self, path, fps=30.0):
        self.paths = sorted(os.path.join(path, f) for f in os.listdir(path)
                            if f.lower().endswith(self.EXT))
        if not self.paths:
            raise FileNotFoundError(f"nicio imagine in {path}")
        self.nominal_fps = fps
        self.i = 0

    timer = None

    def read(self):
        if self.i >= len(self.paths):
            return None
        t0 = time.perf_counter()
        img = cv2.imread(self.paths[self.i], cv2.IMREAD_GRAYSCALE)
        if self.timer is not None:
            self.timer.add('achizitie', time.perf_counter() - t0)
        t = self.i / self.nominal_fps
        self.i += 1
        if img is None:
            return self.read()
        return img, t


class GazeboFrameSource(FrameSource):
    """Cadre din Gazebo, prin gz-transport (I3).

        src = GazeboFrameSource('/down_cam/image')
        gray, t = src.read()

    **Calea 1 din doua, si a mers.** Alternativa era GstCameraPlugin cu flux
    UDP citit prin cv2.VideoCapture; H.264 introduce artefacte de compresie
    care degradeaza exact localizarea colturilor pe care vrem sa o masuram.
    Aici cadrele vin NECOMPRIMATE, iar timestamp-ul e cel din simulare.

    CEASUL

    `clock='sim'` (implicit) intoarce timpul de SIMULARE, din antetul
    mesajului. Cu el, o rulare accelerata sau incetinita nu strica nici
    masuratorile de latenta, nici varsta detectiei - dar bucla care consuma
    sursa trebuie sa foloseasca ACELASI ceas, altfel `now - det.t` compara
    doua lumi. `clock='wall'` exista pentru cand se amesteca cu cod care
    foloseste `time.monotonic()`; atunci masuratorile de latenta masoara
    altceva.

    DE UNDE VIN DEPENDENTELE

    `gz.transport13` si `gz.msgs10` vin din apt (python3 de sistem), iar
    OpenCV >= 4.7 din venv. Nu coexista intr-un venv obisnuit: vezi
    tools/setup_sim_venv.sh si §5.36.

    ATENTIE: un senzor de camera din Gazebo **nu randeaza fara abonat**
    (§5.33). Clasa asta E abonatul, deci randarea porneste cand o
    instantiezi - si se opreste cand o inchizi.
    """

    #: Cate cadre tinem in coada. 1 = mereu cel mai nou. Detectia care nu
    #: tine pasul trebuie sa piarda cadre vechi, nu sa ramana in urma.
    QUEUE = 1

    def __init__(self, topic='/down_cam/image', clock='sim',
                 timeout_s=5.0, verbose=True):
        if clock not in ('sim', 'wall'):
            raise ValueError(f"clock necunoscut: {clock}")
        self.topic = topic
        self.clock = clock
        self.timeout_s = timeout_s
        self.verbose = verbose
        self.nominal_fps = None          # o aflam din cadrele primite
        self.n_received = 0
        self.n_dropped = 0
        self.last_sim_t = None

        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest = None              # (gray, t)
        self._seq = 0
        self._seen = 0
        self._closed = False
        self._t_prev = None

        Node, self._Image = _gz_imports()
        self._node = Node()
        if not self._node.subscribe(self._Image, topic, self._on_msg):
            raise RuntimeError(
                f"nu ma pot abona la {topic}.\n"
                f"  Ruleaza `gz topic -l | grep {topic}`; daca topicul nu "
                f"apare, serverul nu e pornit sau senzorul nu e in lume.")
        if verbose:
            print(f"[gazebo] abonat la {topic} (ceas: {clock})")

    # -- receptie ----------------------------------------------------------
    def _on_msg(self, msg):
        try:
            gray = _image_to_gray(msg)
        except Exception as e:                              # noqa: BLE001
            if self.verbose:
                print(f"[gazebo] cadru ignorat: {type(e).__name__}: {e}")
            return
        t_sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        t = t_sim if self.clock == 'sim' else time.monotonic()
        with self._cond:
            if self._latest is not None and self._seq > self._seen:
                # cadrul anterior nu a fost consumat: il pierdem DELIBERAT,
                # ca detectia sa lucreze pe cel mai nou (vezi QUEUE)
                self.n_dropped += 1
            self._latest = (gray, t)
            self._seq += 1
            self.n_received += 1
            if self._t_prev is not None and t_sim > self._t_prev:
                dt = t_sim - self._t_prev
                self.nominal_fps = 1.0 / dt if dt > 0 else self.nominal_fps
            self._t_prev = t_sim
            self.last_sim_t = t_sim
            self._cond.notify_all()

    # -- interfata FrameSource ---------------------------------------------
    def read(self):
        """Urmatorul cadru NOU, sau None dupa `timeout_s` fara niciunul.

        Blocheaza pana soseste un cadru pe care nu l-am mai dat. None
        inseamna "sursa s-a terminat" pentru PiDetector, deci timeout-ul
        trebuie sa fie mai lung decat orice pauza normala intre cadre."""
        with self._cond:
            if self._closed:
                return None
            if self._seq <= self._seen:
                self._cond.wait(timeout=self.timeout_s)
            if self._closed or self._seq <= self._seen or self._latest is None:
                return None
            self._seen = self._seq
            return self._latest

    def sim_time(self):
        """Timpul de simulare al ultimului cadru, sau None.

        Bucla care consuma sursa cu `clock='sim'` are nevoie de el ca sa
        foloseasca acelasi ceas."""
        with self._lock:
            return self.last_sim_t

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        # gz.transport nu expune dezabonare; nodul se elibereaza cand e
        # colectat. Il scoatem din calea noastra explicit.
        self._node = None

    def stats(self):
        with self._lock:
            return {'received': self.n_received, 'dropped': self.n_dropped,
                    'fps': self.nominal_fps, 'sim_t': self.last_sim_t}


def _gz_imports():
    """(Node, Image) din gz-transport. Mesaj util cand lipsesc.

    Versiunea pachetelor urmeaza versiunea de Gazebo: Harmonic = transport13
    + msgs10. Incercam in ordine descrescatoare, ca sa mearga si pe altceva."""
    perechi = ((13, 10), (12, 9), (11, 8))
    erori = []
    for tv, mv in perechi:
        try:
            transport = __import__(f'gz.transport{tv}', fromlist=['Node'])
            msgs = __import__(f'gz.msgs{mv}.image_pb2', fromlist=['Image'])
            return transport.Node, msgs.Image
        except ImportError as e:                            # noqa: PERF203
            erori.append(f"transport{tv}/msgs{mv}: {e}")
    raise ImportError(
        "gz-transport pentru Python nu e disponibil.\n"
        "  Vine din apt (python3-gz-*), deci NU se vede dintr-un venv "
        "obisnuit.\n"
        "  Creeaza mediul de simulare:  tools/setup_sim_venv.sh\n"
        "  Incercat: " + '; '.join(erori))


#: Formatele pe care le stim converti. L_INT8 e ce cere senzorul nostru;
#: restul exista pentru cand cineva schimba <format> in SDF si se intreaba
#: de ce nu mai merge.
_GZ_L8 = 1
_GZ_RGB8 = 3
_GZ_BGR8 = 9


def _image_to_gray(msg):
    """gz.msgs.Image -> numpy gri (H, W), fara copie inutila."""
    w, h = int(msg.width), int(msg.height)
    if w <= 0 or h <= 0:
        raise ValueError(f"dimensiuni invalide {w}x{h}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    fmt = int(msg.pixel_format_type)
    if fmt == _GZ_L8:
        canale = 1
    elif fmt in (_GZ_RGB8, _GZ_BGR8):
        canale = 3
    else:
        raise ValueError(
            f"format {fmt} neacceptat; senzorul nostru cere L8 "
            f"(tools/make_camera_model.py: IMAGE_FORMAT)")
    asteptat = w * h * canale
    if buf.size < asteptat:
        raise ValueError(f"{buf.size} octeti, asteptati {asteptat}")
    img = buf[:asteptat].reshape(h, w, canale)
    if canale == 1:
        # copie: buf-ul protobuf poate fi reciclat de sub noi
        return np.ascontiguousarray(img[:, :, 0])
    cod = cv2.COLOR_RGB2GRAY if fmt == _GZ_RGB8 else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(img, cod)


class PiCameraSource(FrameSource):
    """Camera Module 3 Wide prin picamera2 (E1.1). Importul e lenes, ca
    modulul sa se poata incarca si pe desktop.

    Nota despre "dual-stream". IMX708 are moduri de senzor distincte:
    4608x2592 la ~14 fps si 2304x1296 (binned) la ~56 fps. Tracking-ul la
    30 fps cere modul binned, iar in acel mod NU se poate scoate simultan un
    cadru la rezolutie nativa - ISP-ul poate doar sa scaleze in jos. Deci
    cadrul de scoring se obtine prin comutare de mod la cerere
    (`capture_scoring_frame`), care opreste fluxul de tracking ~0.3-0.5 s.
    De aceea 8.3.3 se rezolva cu ring buffer (grupul C), nu cu o captura
    sincrona la contact. Cifrele de mai sus sunt din fisa senzorului; se
    confirma pe hardware (E2, temperatura + FPS efectiv).
    """

    nominal_fps = TRACK_FPS

    def __init__(self, size=None, sensor_mode=None, fps=TRACK_FPS,
                 controls=None, verbose=True, auto_expose=True, fresh=True,
                 settings=None):
        """`settings` (CameraSettings, from camera_settings(cfg, preset))
        says everything: sensor mode, stream, exposure mode, AWB, focus.
        Without it, `sensor_mode` + `size` + `auto_expose` (auto_lock or
        fixed) - the older call. Either way the sensor mode is MANDATORY
        (27.09.2026): the camera never lets libcamera choose. After start,
        `geometry` says what the pixels are - the mode read back from the
        configuration and the ScalerCrop of the first frame; a mode other
        than the one asked for is a CameraModeError."""
        if settings is None:
            if sensor_mode is None:
                raise CameraModeError(
                    "PiCameraSource: modul senzorului e obligatoriu (config "
                    "`sensor_mode`). Fara el libcamera il alege singur.")
            settings = CameraSettings(
                dict(CAMERA_KEY_DEFAULTS, sensor_mode=sensor_mode,
                     output_size=size,
                     exposure='auto_lock' if auto_expose else 'fixed'))
        from picamera2 import Picamera2               # noqa: import lenes
        from libcamera import controls as lc

        self.verbose = verbose
        self.settings = settings
        self.sensor_mode = settings.sensor_mode
        self.size = settings.output_size
        self.geometry = None
        self.mode_info = None
        # Step 5 (§5.65): `capture_request()` returns the OLDEST completed
        # request, so with buffer_count=4 a frame was already 50-90 ms old
        # when it left the source (measured: 'varsta' p50 50 ms). With
        # flush=True picamera2 returns a frame captured AFTER the call:
        # no queue, one ISP latency. If this picamera2 has no `flush`, the
        # old behaviour stays and the log says so.
        self.fresh = bool(fresh)
        self._flush_ok = None
        tuning = None
        if settings.exposure == 'auto_capped':
            tuning = capped_tuning(Picamera2, settings.exposure_max_us,
                                   settings.gain_max)
        try:
            self.picam2 = Picamera2() if tuning is None else Picamera2(tuning=tuning)
        except RuntimeError as e:
            # libcamera spune CE ("Camera __init__ sequence did not
            # complete"), nu CINE. Pe vehicul: a doua instanta a aplicatiei,
            # pornita peste serviciul de pornire automata. Se numeste
            # ocupantul inainte de a lasa eroarea sa urce.
            from . import serial_guard
            cine = serial_guard.describe_camera_conflict()
            if cine:
                raise RuntimeError(f"{e}\n  {cine}") from e
            raise
        try:
            self._start(fps, controls, auto_expose, lc)
        except Exception:
            # a camera left open holds the sensor until the process exits:
            # the next attempt (preflight, then the app) would fail on it
            self.close()
            raise

    def _start(self, fps, controls, auto_expose, lc):
        self._check_mode_exists()
        sensor = {'output_size': self.sensor_mode, 'bit_depth': SENSOR_BIT_DEPTH}
        self.video_cfg = self.picam2.create_video_configuration(
            main={'size': self.size, 'format': 'YUV420'},
            sensor=sensor,
            controls={'FrameRate': float(fps)},
            buffer_count=4)
        self.still_cfg = self.picam2.create_still_configuration(
            main={'size': SCORING_SIZE, 'format': 'YUV420'},
            sensor={'output_size': SCORING_SIZE, 'bit_depth': SENSOR_BIT_DEPTH})
        self.picam2.configure(self.video_cfg)
        self._check_configured_mode()

        ctrl = self._controls_for(self.settings, lc)
        ctrl.update(controls or {})
        self.picam2.set_controls(ctrl)
        self.picam2.start()
        if self.settings.exposure == 'auto_lock':
            ctrl = self._lock_exposure(ctrl)

        # Ceasul senzorului e CLOCK_BOOTTIME (ns); time.monotonic() e
        # CLOCK_MONOTONIC. Pe un Pi care nu suspenda, coincid - dar calculam
        # offsetul o data, ca timestamp-ul de captura sa fie comparabil cu
        # ceasul buclei si al supervizorului.
        self._boot_offset = (time.monotonic()
                             - time.clock_gettime(time.CLOCK_BOOTTIME))
        self._verify_controls(ctrl)
        self._read_geometry()

    def _controls_for(self, s, lc):
        """The libcamera controls for the settings. Focus is always manual
        (PDAF hunting during the descent is exactly when the marker's
        contrast changes fastest). For auto_lock and fixed this is, key for
        key, what the camera got before the presets existed."""
        ctrl = {'LensPosition': s.lens_position, 'AwbEnable': s.awb,
                'AfMode': lc.AfModeEnum.Manual}
        if s.exposure == 'fixed':
            ctrl.update(ExposureTime=s.exposure_us,
                        AnalogueGain=s.analogue_gain, AeEnable=False)
        else:
            # auto_lock: converges on the scene, then locked (_lock_exposure)
            ctrl['AeEnable'] = True
        if s.exposure == 'auto_capped':
            if 'AeExposureMode' not in (self.picam2.camera_controls or {}):
                raise CameraModeError(
                    "auto_capped: libcamera nu anunta AeExposureMode pe "
                    "aceasta camera; plafonul nu se poate aplica")
            ctrl['AeExposureMode'] = lc.AeExposureModeEnum.Custom
        return ctrl

    # -- the sensor mode: asked, validated, read back (27.09.2026) ---------
    def _check_mode_exists(self):
        """The mode must be one the sensor has (picam2.sensor_modes, which
        reconfigures the camera once per mode to read them - so BEFORE our
        own configure)."""
        modes = self.picam2.sensor_modes or []
        found = [m for m in modes
                 if tuple(m.get('size') or ()) == self.sensor_mode
                 and m.get('bit_depth') == SENSOR_BIT_DEPTH]
        if not found:
            disp = ', '.join(sorted({f"{m['size'][0]}x{m['size'][1]}/"
                                     f"{m.get('bit_depth')}bit"
                                     for m in modes if m.get('size')}))
            raise CameraModeError(
                f"modul {self.sensor_mode[0]}x{self.sensor_mode[1]}/"
                f"{SENSOR_BIT_DEPTH}bit nu exista pe senzor (moduri: {disp})")
        self.mode_info = found[0]

    def _check_configured_mode(self):
        """Read back what libcamera accepted: the sensor configuration and
        the stream size. Different from what was asked = the camera does
        not start."""
        cc = self.picam2.camera_configuration() or {}
        sensor = cc.get('sensor') or {}
        got = tuple(sensor.get('output_size') or ())
        bits = sensor.get('bit_depth')
        main = tuple((cc.get('main') or {}).get('size') or ())
        if self.verbose:
            print(f"[camera] mod cerut {self.sensor_mode[0]}x{self.sensor_mode[1]}"
                  f"/{SENSOR_BIT_DEPTH}bit, configurat {got} {bits}bit, "
                  f"flux {main}")
        if got != self.sensor_mode or bits != SENSOR_BIT_DEPTH:
            raise CameraModeError(
                f"modul efectiv al senzorului {got} / {bits} bit difera de "
                f"cel cerut {self.sensor_mode} / {SENSOR_BIT_DEPTH} bit: "
                f"camera nu porneste")
        if main and main != self.size:
            raise CameraModeError(
                f"fluxul configurat {main} difera de cel cerut {self.size}")

    def _read_geometry(self):
        """ScalerCrop from the metadata of a frame: the region of the array
        the image covers. With the mode, it decides which calibration
        transform is legal (calibration_for)."""
        md = self.picam2.capture_metadata() or {}
        crop = md.get('ScalerCrop')
        if crop is None:
            raise CameraModeError(
                "metadatele cadrului nu au ScalerCrop: geometria imaginii nu "
                "se poate verifica, camera nu porneste")
        self.geometry = CameraGeometry(self.sensor_mode, tuple(crop), self.size)
        if self.verbose:
            asteptat = IMX708_MODES.get(self.sensor_mode)
            nota = ('' if asteptat is None or tuple(crop) == asteptat
                    else f"  (ATENTIE: modul citeste de obicei {asteptat})")
            print(f"[camera] geometrie: {self.geometry.describe()}{nota}")
        return self.geometry

    def _lock_exposure(self, ctrl):
        """Lasa AE-ul sa convearga, apoi fixeaza ce a masurat (plafonat).
        Intoarce dictionarul de controale FIXATE, pentru citirea inapoi."""
        limits = (self.picam2.camera_controls or {}).get('AnalogueGain')
        gain_min, gain_max = (float(limits[0]), float(limits[1])) \
            if limits else (1.0, 16.0)
        md = {}
        for _ in range(AUTOEXP_FRAMES):
            md = self.picam2.capture_metadata()
        exp = md.get('ExposureTime')
        gain = md.get('AnalogueGain')
        if exp is None or gain is None:
            # fara metadate nu blocam pe ghicite: cad pe valorile de banc
            fixed = dict(ctrl)
            fixed.update(ExposureTime=CAMERA_CONTROLS['ExposureTime'],
                         AnalogueGain=CAMERA_CONTROLS['AnalogueGain'],
                         AeEnable=False)
            if self.verbose:
                print("[camera] ATENTIE: AE fara metadate; folosesc "
                      "valorile de banc (2000 us / gain 8)")
            self.picam2.set_controls(fixed)
            asteapta_valoare(self.picam2.capture_metadata,
                             'ExposureTime', fixed['ExposureTime'])
            return fixed
        exp_f, gain_f, avert = expunere_blocata(exp, gain,
                                                gain_min=gain_min,
                                                gain_max=gain_max)
        fixed = dict(ctrl)
        fixed.update(ExposureTime=int(round(exp_f)),
                     AnalogueGain=gain_f, AeEnable=False)
        self.picam2.set_controls(fixed)
        if self.verbose:
            print(f"[camera] expunere masurata pe scena: {exp:.0f} us "
                  f"gain {gain:.2f} -> BLOCATA la {exp_f:.0f} us "
                  f"gain {gain_f:.2f} (plafon blur {AUTOEXP_MAX_US} us)")
            if avert:
                print(f"[camera] ATENTIE: {avert}")
        # Se asteapta ca valoarea sa se APLICE, nu un numar fix de cadre:
        # driverul o propaga abia dupa cateva cadre, iar verificarea citita
        # prea devreme vede inca AE-ul si pica preflight-ul (ESEC fals).
        if not asteapta_valoare(self.picam2.capture_metadata,
                                'ExposureTime', fixed['ExposureTime']) \
                and self.verbose:
            print(f"[camera] ATENTIE: ExposureTime {fixed['ExposureTime']} us "
                  f"nu s-a aplicat in {CONTROL_APPLY_FRAMES} cadre")
        return fixed

    def _verify_controls(self, wanted):
        """§5.10 aplicat camerei: un control poate fi acceptat si apoi
        limitat tacut de driver. Citim inapoi din metadate."""
        md = self.picam2.capture_metadata()
        problems = []
        for key in ('LensPosition', 'ExposureTime', 'AnalogueGain'):
            got = md.get(key)
            want = wanted.get(key)
            if got is None:
                problems.append(f"{key}: neraportat")
            elif want is not None and abs(float(got) - float(want)) > 0.05 * abs(float(want)) + 1e-6:
                problems.append(f"{key}: cerut {want}, aplicat {got}")
        s = getattr(self, 'settings', None)
        if s is not None and s.exposure == 'auto_capped':
            # the cap lives in the tuning, not in a control: read it back
            exp, gain = md.get('ExposureTime'), md.get('AnalogueGain')
            if exp is not None and float(exp) > 1.05 * s.exposure_max_us:
                problems.append(f"ExposureTime {exp} us peste plafonul "
                                f"{s.exposure_max_us} us (auto_capped)")
            if gain is not None and float(gain) > 1.05 * s.gain_max:
                problems.append(f"AnalogueGain {gain} peste plafonul "
                                f"{s.gain_max:g} (auto_capped)")
        if self.verbose:
            print(f"[camera] {self.size[0]}x{self.size[1]} @ {TRACK_FPS} fps, "
                  f"LensPosition={md.get('LensPosition')} "
                  f"ExposureTime={md.get('ExposureTime')} us "
                  f"AnalogueGain={md.get('AnalogueGain')}")
            for p in problems:
                print(f"[camera] ATENTIE control neaplicat: {p}")
        self.control_problems = problems

    #: StageTimer attached by PiDetector (None = no timing).
    timer = None
    #: Metadata of the last frame (ExposureTime, AnalogueGain,
    #: LensPosition, SensorTimestamp), for the recording/bench tools.
    last_metadata = None

    def read(self):
        t0 = time.perf_counter()
        req = self._capture()
        t1 = time.perf_counter()
        try:
            # make_array() copiaza; request-ul se ELIBEREAZA imediat dupa,
            # altfel bufferele camerei (buffer_count=4) se epuizeaza si
            # captura se blocheaza (faza 4, regula 6).
            arr = req.make_array('main')
            md = req.get_metadata()
        finally:
            req.release()
        h = self.size[1]
        gray = np.ascontiguousarray(arr[:h, :self.size[0]])   # planul Y
        t2 = time.perf_counter()
        self.last_metadata = md
        ts = md.get('SensorTimestamp')
        t = (ts * 1e-9 + self._boot_offset) if ts else time.monotonic()
        if self.timer is not None:
            self.timer.add('achizitie', t1 - t0)   # wait for a request
            self.timer.add('gri', t2 - t1)         # Y-plane copy, no cvtColor
            # Age of the frame when it leaves the source: queue depth shows
            # up here (step 5 of §5.65), not in the detection stages.
            self.timer.add('varsta', time.monotonic() - t)
        return gray, t

    def _capture(self):
        """One completed request: the newest (flush) when supported."""
        if self.fresh and self._flush_ok is not False:
            try:
                req = self.picam2.capture_request(flush=True)
                self._flush_ok = True
                return req
            except TypeError:
                self._flush_ok = False
                if self.verbose:
                    print("[camera] picamera2 fara capture_request(flush=): "
                          "raman pe coada veche (varsta cadrului ramane)")
        return self.picam2.capture_request()

    def capture_scoring_frame(self):
        """Cadru la rezolutie nativa, prin comutare de mod (~0.3-0.5 s fara
        tracking). Grupul C decide cand se apeleaza; aici doar exista."""
        arr = self.picam2.switch_mode_and_capture_array(self.still_cfg, 'main')
        h = SCORING_SIZE[1]
        return np.ascontiguousarray(arr[:h, :SCORING_SIZE[0]])

    def close(self):
        # separately: a stop() failing on a camera in an error state must
        # not skip the close() that releases the sensor
        for fn in ('stop', 'close'):
            try:
                getattr(self.picam2, fn)()
            except Exception:                               # noqa: BLE001
                pass


# --- Detectorul de bord ----------------------------------------------------------

class PiDetector:
    """Sursa + detectie intr-un fir separat, `poll(now)` pentru bucla.

    De ce fir separat: read() pe picamera2 blocheaza ~33 ms la 30 fps. Bucla
    din nova/state_machine.run_loop ruleaza supervizorul la sute de Hz si
    trebuie sa prinda fereastra de spool-down din §5.6 (sub 100 ms); nu o
    lasam sa astepte dupa camera.

    Instrumentare (E1.4): latenta captura->publicare ca percentile (p50,
    p99), nu ca medie - o medie ascunde exact cozile care ne intereseaza.
    FPS efectiv si rata de detectie pe aceeasi fereastra.
    """

    def __init__(self, source, detector, threaded=True, max_queue=8,
                 clock=None, keep_last_frame=False, ring_frames=0,
                 frame_log=None):
        """`clock` e ceasul in care se masoara LATENTA captura->publicare.

        Trebuie sa fie ACELASI cu cel in care sursa stampileaza cadrele. Pe
        Pi ambele sunt `time.monotonic()`, deci implicitul e corect. Cu
        `GazeboFrameSource`, cadrele poarta timp de SIMULARE, iar
        `time.monotonic()` e timpul de la pornirea masinii: diferenta lor nu
        e o latenta, e uptime-ul.

        Masurat in prima campanie: `lat p50 3385850 ms`, adica 3386 s -
        exact cat rula masina. A treia oara aceeasi forma ca §5.39, acum in
        instrumentare. Aditiv: nimic nu se schimba pe vehicul."""
        self.source = source
        self.det = detector
        self.clock = clock or time.monotonic
        #: Stage timings (step 0, §5.65). One timer shared by the source,
        #: the detector and this loop; attached here so production wiring
        #: gets it without any caller knowing about it.
        self.timer = StageTimer()
        try:
            self.det.timer = self.timer
            self.source.timer = self.timer
        except AttributeError:
            pass
        #: Ultimul cadru citit, pastrat DOAR daca cineva cere explicit.
        #: Serveste la diagnostic: cand detectia se pierde, vrei pixelii
        #: care au picat, nu o teorie despre ei. Oprit implicit - pe Pi
        #: ar fi o copie de 3 MB pe fiecare cadru, din bugetul detectiei.
        self.keep_last_frame = keep_last_frame
        self.last_frame = None
        #: Ring buffer pentru 8.3.3 (J2). Oprit implicit: pe Pi fiecare cadru
        #: e ~3 MB, deci 30 de cadre inseamna 90 MB de RAM. Se porneste
        #: explicit de aplicatia care preda imaginea juriului.
        self.ring = FrameRing(ring_frames) if ring_frames else None
        self.threaded = threaded
        self.queue = collections.deque(maxlen=max_queue)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.exhausted = False
        #: B9 (26.09.2026): the worker died on an exception (text), and
        #: when. Before, a picamera2 timeout or any error in detect() ended
        #: the thread with a traceback on stderr and NOTHING else: the
        #: miss counter froze (at 0 if the last frame had the marker), fps
        #: and det% kept showing the last 300 frames, and only the 1.5 s
        #: hard age cap in the supervisor would stop a descent.
        self.died = None
        self.died_t = None
        self._dead_polls = 0
        #: Faza 4: ce citesc celelalte fire. `latest` = ultima Detection cu
        #: timpul CAPTURII (supervizorul masoara varsta fata de el);
        #: `heartbeat` bate DOAR dupa un cadru procesat, deci o camera
        #: blocata in captura apare ca heartbeat stagnat, nu ca "viu";
        #: `active` e citit de fir: stins, cadrele se citesc (ringul si
        #: expunerea continua), dar nu se detecteaza. Aprins implicit -
        #: comportamentul de azi.
        self.latest = Latest()
        self.heartbeat = Heartbeat('detectie')
        self.active = threading.Event()
        self.active.set()
        self.fail_streak = 0
        self.n_fail = 0
        self.window_log_s = DETECT_LOG_WINDOW_S
        self.window_log = True
        self.last_window = None
        self._win_t0 = None
        self._win_frames = 0
        self._win_processed = 0
        self._win_dets = 0
        self._win_time = 0.0
        #: 27.09.2026 (point 3): frames read by the detection thread, ever -
        #: the OSD window redraws when it changes. The camera's metadata per
        #: frame (ExposureTime, AnalogueGain, LensPosition, SensorTimestamp)
        #: goes to `frame_log` (a CSV path or an open text file; None = off),
        #: and per window: exposure / gain seen, saturated and near-black
        #: share of the gray plane.
        self.frame_seq = 0
        self.n_no_timestamp = 0
        self._win_sat = self._win_dark = self._win_px = 0
        self._win_exp = []
        self._win_gain = []
        self._win_lens = None
        self._frame_log = None
        self._frame_log_own = False
        self.frame_log_path = None
        if isinstance(frame_log, str):
            # A diagnostic file must never stop the detector from starting
            # (read-only disk, permissions), and never truncate an earlier
            # flight's file: on a Pi without RTC two boots can share a
            # timestamp. Exclusive create, a suffix on collision.
            try:
                os.makedirs(os.path.dirname(os.path.abspath(frame_log)), exist_ok=True)
                base, ext = os.path.splitext(frame_log)
                for i in range(100):
                    cand = frame_log if i == 0 else f"{base}-{i}{ext}"
                    try:
                        self._frame_log = open(cand, 'x', buffering=1 << 16)
                        self.frame_log_path = cand
                        self._frame_log_own = True
                        break
                    except FileExistsError:
                        continue
            except OSError as e:
                print(f"[detector] ATENTIE: jurnalul per cadru nu se poate "
                      f"deschide ({e}); detectia merge fara el", flush=True)
                self._frame_log = None
        elif frame_log is not None:
            self._frame_log = frame_log
        if self._frame_log is not None:
            try:
                self._frame_log.write(','.join(FRAME_LOG_FIELDS) + '\n')
            except OSError:
                self._drop_frame_log('scrierea antetului')
        #: The last frame with ITS detection outcome, published together at
        #: the end of the frame (review 27.09.2026): the OSD draws from this,
        #: never from `last_frame` + `det.last_corners`, which the detection
        #: clears and refills while it runs.
        self.last_view = None
        self._diag_off = False

        self.latencies = collections.deque(maxlen=STATS_WINDOW)
        self.frame_times = collections.deque(maxlen=STATS_WINDOW)
        self.frame_detected = collections.deque(maxlen=STATS_WINDOW)
        self.n_published = 0
        self.n_dropped = 0

    @property
    def miss_streak(self):
        """Cadre consecutive fara marker, pentru supervizor. Delegat, ca
        n_frames: un atribut care lipseste pe clasa din productie ar face
        regula de abort inerta, fara nicio eroare (§5.56).

        A dead worker (B9) processes no frames, so the counter would never
        grow again: every poll() after death counts as a miss instead. A
        detector that died must look like a lost marker, not healthy."""
        inner = getattr(self.det, 'miss_streak', None)
        if self.died is not None:
            return (inner or 0) + self._dead_polls
        return inner

    @property
    def n_frames(self):
        """Cate cadre au fost PROCESATE, cu sau fara detectie.

        Contorul creste in ArucoMarkerDetector, care vede fiecare cadru.
        Aici e doar delegat, si asta nu e cosmetic: `nova_sim._note_frames`
        il cauta prin `getattr(detector, 'n_frames', None)`, iar detectorul
        pe care il primeste e un PiDetector. Cat timp proprietatea a lipsit,
        `getattr` intorcea None, numararea ratarilor se oprea din prima
        instructiune si `rata_detectie` raporta **1.000 in orice conditii**.

        A patra oara aceeasi forma ca §5.39: metrica moarta care raporteaza
        sanatate. De data asta testul care ar fi trebuit sa o prinda isi
        injecta un obiect fals cu `n_frames`, deci verifica aritmetica lui
        `_note_frames` pe o clasa care nu e cea din productie (§5.40)."""
        return self.det.n_frames

    @property
    def n_detected(self):
        """Cate cadre au produs o detectie.

        Perechea lui `n_frames`, si ASTA e ce conteaza: amandoua cresc in
        `ArucoMarkerDetector.detect()`, deci sunt consistente intre ele
        oricand le-ai citi. Diferenta lor e numarul de ratari reale.

        Numarul de detectii POLLED de bucla nu e acelasi lucru: intre
        `detect()` si `poll()` trece coada, deci o detectie poate fi
        numarata intr-un ciclu si consumata in urmatorul."""
        return self.det.n_detected

    @property
    def cam(self):
        """Modelul de camera al detectorului interior.

        Delegat din acelasi motiv ca `n_frames`: cine are PiDetector nu
        trebuie sa stie ca inauntru e un ArucoMarkerDetector. Un
        `detector.det.cam` scris prin lantul de atribute se rupe tacut la
        prima schimbare de structura - exact cum s-a rupt numararea
        cadrelor."""
        return self.det.cam

    # -- ciclu de viata ----------------------------------------------------
    def start(self):
        if self.threaded and self._thread is None:
            self._thread = threading.Thread(target=self._worker, daemon=True,
                                            name='nova-detector')
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.source.close()
        f, self._frame_log = self._frame_log, None
        if f is not None:
            try:
                f.flush()
                if self._frame_log_own:
                    # a battery pull right after landing must not lose the
                    # descent's rows; here, not in the detection thread
                    os.fsync(f.fileno())
                    f.close()
            except (OSError, ValueError, AttributeError):
                pass

    def _drop_frame_log(self, why):
        """Disk full / I/O error: close, say it once, detection goes on."""
        f, self._frame_log = self._frame_log, None
        if f is not None and self._frame_log_own:
            try:
                f.close()
            except (OSError, ValueError):
                pass
        print(f"[detector] ATENTIE: jurnalul per cadru oprit ({why}); "
              f"detectia continua", flush=True)

    def _diag(self, fn, *args):
        """Run a diagnostic step (stats, frame log, window line). A failure
        in it must not drop a detection or count toward DETECT_FAIL_MAX:
        the first one switches diagnostics off, with one line."""
        if self._diag_off:
            return None
        try:
            return fn(*args)
        except Exception as e:                              # noqa: BLE001
            self._diag_off = True
            print(f"[detector] ATENTIE: diagnosticul a picat ({type(e).__name__}: "
                  f"{e}); oprit, detectia continua", flush=True)
            return None

    # -- procesare ---------------------------------------------------------
    def _process_one(self):
        """Un cadru: citeste, detecteaza, publica. False cand sursa s-a
        terminat."""
        t_start = time.perf_counter()
        item = self.source.read()
        if item is None:
            self.exhausted = True
            return False
        gray, t_cap = item
        md = self._diag(self._frame_stats, gray)
        if self.keep_last_frame:
            self.last_frame = gray
        if self.ring is not None:
            # Impins INAINTE de detectie: cadrul trebuie sa fie in ring chiar
            # daca detectia pe el esueaza. Cadrul de contact e tocmai unul pe
            # care markerul nu mai incape in cadru (§5.2).
            t0 = time.perf_counter()
            self.ring.push(gray, t_cap)
            self.timer.add('ring', time.perf_counter() - t0)
        self._win_frames += 1
        if not self.active.is_set():
            # detectie inactiva: cadrul a fost citit (ring, expunere), atat
            self._publish_view(gray, None)
            self._diag(self._log_frame, t_cap, md, None, None)
            self._diag(self._window_log, time.monotonic())
            return True
        det = self.det.detect(gray, t_cap)
        t_pub = time.monotonic()
        dt = time.perf_counter() - t_start
        self.timer.add('total', dt)
        self._win_processed += 1
        self._win_time += dt
        with self.lock:
            self.frame_times.append(t_pub)
            self.frame_detected.append(det is not None)
            if det is not None:
                # Ceasul SURSEI, nu cel de perete: altfel scadem doua lumi.
                self.latencies.append(self.clock() - t_cap)
                if len(self.queue) == self.queue.maxlen:
                    self.n_dropped += 1
                self.queue.append(det)
                self.n_published += 1
        if det is not None:
            self._win_dets += 1
            self.latest.set(det, t_cap)
        self._publish_view(gray, det)
        # diagnostics AFTER the detection is published: a slow disk or a
        # bad value delays / loses a log row, never a detection
        self._diag(self._log_frame, t_cap, md, det, dt)
        self._diag(self._window_log, t_pub)
        return True

    def _publish_view(self, gray, det):
        """The frame and its outcome, one tuple, one assignment (atomic for
        the reader): (seq, gray or None, corners or None). `frame_seq` moves
        here, at the END of the frame, so the OSD draws in the camera-wait
        gap, not while detection runs."""
        corners = None
        if det is not None:
            c = getattr(self.det, 'last_corners', None)
            corners = None if c is None else np.array(c, copy=True)
        seq = self.frame_seq + 1
        self.last_view = (seq, gray if self.keep_last_frame else None, corners)
        self.frame_seq = seq

    def _frame_stats(self, gray):
        """Per frame: the camera's metadata (from the source, when it has
        any) into the window, and the sampled saturation of the gray plane.
        Returns the metadata dict (or None) for the per-frame log."""
        md = getattr(self.source, 'last_metadata', None)
        if md:
            e, g = md.get('ExposureTime'), md.get('AnalogueGain')
            if e is not None:
                self._win_exp.append(float(e))
            if g is not None:
                self._win_gain.append(float(g))
            if md.get('LensPosition') is not None:
                self._win_lens = float(md['LensPosition'])
            if md.get('SensorTimestamp') is None:
                # t_capture fell back to the time read() returned: count
                # it, a capture time that is not one must be visible
                self.n_no_timestamp += 1
        if getattr(gray, 'ndim', 0) < 2:
            return md                    # sources without pixels (tests, sim)
        s = gray[::SAT_SAMPLE, ::SAT_SAMPLE]
        self._win_sat += int(np.count_nonzero(s >= 255))
        self._win_dark += int(np.count_nonzero(s < DARK_LEVEL))
        self._win_px += int(s.size)
        return md

    def _log_frame(self, t_cap, md, det, dt):
        f = self._frame_log
        if f is None:
            return
        md = md or {}

        def v(x, fmt='{}'):
            return '' if x is None else fmt.format(x)
        row = (self.frame_seq, f"{t_cap:.6f}", v(md.get('SensorTimestamp')),
               v(md.get('ExposureTime')), v(md.get('AnalogueGain'), '{:.3f}'),
               v(md.get('LensPosition'), '{:.3f}'),
               '' if dt is None else int(det is not None),
               '' if getattr(det, 'marker_px', None) is None else f"{det.marker_px:.1f}",
               '' if dt is None else f"{1000.0 * dt:.1f}")
        try:
            f.write(','.join(str(x) for x in row) + '\n')
        except (OSError, ValueError) as e:
            # a full disk must not take the detection down with it
            self._drop_frame_log(f"{type(e).__name__}: {e}")

    def _window_log(self, now):
        """Faza 4: la fiecare `window_log_s`, o linie cu cadrele capturate,
        procesate, detectiile si timpul mediu per cadru procesat."""
        if self._win_t0 is None:
            self._win_t0 = now
            return
        if now - self._win_t0 < self.window_log_s:
            return
        span = now - self._win_t0
        medie = (1000.0 * self._win_time / self._win_processed
                 if self._win_processed else None)
        px = self._win_px
        sat = 100.0 * self._win_sat / px if px else None
        dark = 100.0 * self._win_dark / px if px else None
        exp = self._percentile(self._win_exp, 0.5)
        gain = self._percentile(self._win_gain, 0.5)
        self.last_window = {'s': span, 'captured': self._win_frames,
                            'processed': self._win_processed,
                            'detections': self._win_dets, 'mean_ms': medie,
                            'sat_pct': sat, 'dark_pct': dark,
                            'exposure_us': exp,
                            'exposure_max_us': max(self._win_exp) if self._win_exp else None,
                            'gain': gain, 'lens': self._win_lens,
                            'no_timestamp': self.n_no_timestamp}
        if self.window_log:
            m = '-' if medie is None else f"{medie:.0f}"
            cam = ''
            if sat is not None:
                cam += f" | sat {sat:.1f}% <{DARK_LEVEL} {dark:.1f}%"
            if exp is not None:
                cam += (f" | exp {exp:.0f}us (max {max(self._win_exp):.0f})"
                        f" gain {'-' if gain is None else f'{gain:.2f}'}"
                        f" lens {'-' if self._win_lens is None else f'{self._win_lens:.2f}'}")
            if self.n_no_timestamp:
                cam += f" | FARA SensorTimestamp: {self.n_no_timestamp} cadre"
            print(f"[detectie {span:.0f}s] cadre {self._win_frames} "
                  f"({self._win_frames / span:.1f}/s) | procesate "
                  f"{self._win_processed} | detectii {self._win_dets} | "
                  f"{m} ms/cadru" + cam + ('' if self.active.is_set()
                                           else ' | INACTIVA'), flush=True)
            if self._frame_log is not None:
                try:
                    self._frame_log.flush()
                except (OSError, ValueError) as e:
                    self._drop_frame_log(f"{type(e).__name__}: {e}")
        self._win_t0 = now
        self._win_frames = self._win_processed = self._win_dets = 0
        self._win_time = 0.0
        self._win_sat = self._win_dark = self._win_px = 0
        self._win_exp = []
        self._win_gain = []

    def _worker(self):
        """Bucla firului de detectie (faza 4). Fiecare pas e in try/except
        cu logare; un esec trecator se reia dupa o pauza scurta, iar
        DETECT_FAIL_MAX esecuri consecutive inseamna firul mort (B9): se
        spune, se marcheaza, si miss_streak / stats o arata. Heartbeat-ul
        bate doar dupa un cadru procesat."""
        while not self._stop.is_set():
            try:
                ok = self._process_one()
            except Exception as e:                          # noqa: BLE001
                self.n_fail += 1
                self.fail_streak += 1
                log.error("[detectie] pasul a picat (%d/%d): %s: %s\n%s",
                          self.fail_streak, DETECT_FAIL_MAX,
                          type(e).__name__, e, traceback.format_exc())
                if self.fail_streak >= DETECT_FAIL_MAX:
                    self.died = f"{type(e).__name__}: {e}"
                    self.died_t = self.clock()
                    print(f"[detector] MORT: firul de detectie a picat de "
                          f"{self.fail_streak} ori la rand, ultima cu "
                          f"{self.died}. Fara cadre de acum; supervizorul "
                          f"vede ratari.", flush=True)
                    return
                time.sleep(DETECT_FAIL_BACKOFF_S)
                continue
            self.fail_streak = 0
            if not ok:
                return                       # sursa epuizata: nu e o bataie
            self.heartbeat.beat()

    def poll(self, now):
        """Interfata din nova/detection.py. Fara fir, proceseaza un cadru
        aici (teste, E2); cu fir, doar goleste coada."""
        if not self.threaded:
            self._process_one()
        elif self.died is not None:
            self._dead_polls += 1
        with self.lock:
            out = list(self.queue)
            self.queue.clear()
        return out

    # -- instrumentare -----------------------------------------------------
    @staticmethod
    def _percentile(vals, p):
        if not vals:
            return None
        s = sorted(vals)
        k = (len(s) - 1) * p
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return s[lo] + (s[hi] - s[lo]) * (k - lo)

    def stats(self):
        with self.lock:
            lat = list(self.latencies)
            times = list(self.frame_times)
            dets = list(self.frame_detected)
        fps = None
        if len(times) >= 2 and times[-1] > times[0]:
            # Live (threaded): measured up to NOW, not up to the last frame.
            # A worker stuck in capture, or dead, then shows a decaying fps
            # instead of the last 300 healthy frames forever (B9).
            end = max(times[-1], time.monotonic()) if self.threaded else times[-1]
            fps = (len(times) - 1) / (end - times[0])
        if self.died is not None:
            fps = 0.0
        d = {
            'latency_p50_ms': None,
            'latency_p99_ms': None,
            'fps': fps,
            'detection_rate': (0.0 if self.died is not None else
                               (sum(dets) / len(dets)) if dets else None),
            'died': self.died,
            'published': self.n_published,
            'dropped': self.n_dropped,
            'window': len(times),
        }
        if lat:
            d['latency_p50_ms'] = 1000.0 * self._percentile(lat, 0.50)
            d['latency_p99_ms'] = 1000.0 * self._percentile(lat, 0.99)
        d.update({'aruco_' + k: v for k, v in self.det.stats().items()})
        return d

    def stage_line(self):
        """One line with p50/p99 ms per pipeline stage (step 0, §5.65)."""
        return self.timer.line()

    def status_line(self):
        s = self.stats()
        f = lambda v, fmt: '-' if v is None else format(v, fmt)     # noqa: E731
        mort = f"DETECTOR MORT ({s['died']}) | " if s['died'] else ''
        return (f"{mort}cam {f(s['fps'], '4.1f')} fps | det "
                f"{f(s['detection_rate'], '.0%')} | lat p50 "
                f"{f(s['latency_p50_ms'], '4.0f')} p99 "
                f"{f(s['latency_p99_ms'], '4.0f')} ms | lum "
                f"{f(s.get('aruco_lum'), '3d')} | roi "
                f"{s['aruco_roi_hits']}/{s['aruco_roi_misses']}")


# --- Constructor de bord -------------------------------------------------------------

def build_pi_detector(cfg, verbose=True, ring_frames=0, max_rms=None,
                      keep_last_frame=False, preset=None, frame_log=None):
    """Detectorul complet pentru aplicatia de bord, din config/nova.json.
    Refuza sa porneasca fara calibrare reala (E1.2).

    `ring_frames` > 0 porneste ringul cerut de 8.3.3. Oprit implicit: pe Pi
    un cadru e ~3 MB.

    `max_rms` ridica pragul de reproiectie DOAR pentru apelul asta. Exista
    pentru bring-up la banc, unde o calibrare provizorie e mai buna decat
    niciuna, si se anunta zgomotos. Nu se schimba `MAX_REPROJ_ERR_PX`, care
    e citit de tot codul: ridicat global, ar slabi tacut exact garda care
    decide daca se zboara (§5.34)."""
    from . import config as nova_config
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    prag = MAX_REPROJ_ERR_PX if max_rms is None else float(max_rms)
    calib = CameraCalibration.load(cal_path, require_real=True, max_rms=prag)
    if verbose:
        print(f"[detector] calibrare: {calib}")
    if max_rms is not None and calib.rms is not None \
            and calib.rms > MAX_REPROJ_ERR_PX:
        print(f"[detector] ATENTIE: calibrarea are rms {calib.rms:.3f} px, "
              f"peste pragul de zbor de {MAX_REPROJ_ERR_PX} px.")
        print(f"[detector]           acceptata doar pentru rularea asta "
              f"(--max-rms {prag}). De refacut inainte de zbor.")
    # 27.09.2026: the sensor mode comes from the config (mandatory, checked
    # before the camera opens), the stream from `camera_settings`. The camera is
    # opened FIRST: the calibration can only be chosen once the geometry
    # actually running is known (mode read back + ScalerCrop).
    # `preset` (command line) overrides the config's camera keys entirely.
    settings = camera_settings(cfg, preset)
    if verbose:
        print(f"[detector] camera: {settings.describe()}")
    source = PiCameraSource(settings=settings, verbose=verbose,
                            fresh=cfg.get('camera_fresh_capture', True))
    try:
        return _detector_on(source, calib, cfg, verbose, ring_frames,
                            keep_last_frame, frame_log)
    except BaseException:
        # whatever fails after the camera opened (calibration, a config
        # key, the frame log) releases the sensor: the next attempt -
        # preflight, the app restarted by systemd - must find it free
        source.close()
        raise


def _detector_on(source, calib, cfg, verbose, ring_frames, keep_last_frame,
                 frame_log):
    calib = calibration_for(calib, source.geometry)
    if verbose:
        print(f"[detector] {source.geometry.describe()} | calibrare "
              f"{calib.kind}: fx={calib.fx:.1f} fy={calib.fy:.1f} "
              f"cx={calib.cx:.1f} cy={calib.cy:.1f} HFOV={calib.hfov_deg():.1f}")
    aruco = ArucoMarkerDetector(calib, marker_id=cfg['marker_id'],
                                marker_size_m=cfg['marker_size_m'],
                                roi_below_m=cfg['roi_below_m'],
                                roi_size_px=cfg['roi_size_px'],
                                camera_rotation_deg=cfg['camera_rotation_deg'],
                                search_downscale=cfg['search_downscale'])
    if verbose and aruco.camera_rotation_deg:
        print(f"[detector] camera montata rotit: imaginea se roteste cu "
              f"{aruco.camera_rotation_deg} grade la stanga (axe, nu pixeli)")
    if verbose and frame_log:
        print(f"[detector] jurnal per cadru (metadate camera + detectie): "
              f"{frame_log}")
    return PiDetector(source, aruco, ring_frames=ring_frames, threaded=True,
                      keep_last_frame=keep_last_frame,
                      frame_log=frame_log).start()
