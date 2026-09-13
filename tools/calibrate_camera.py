#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Calibrarea camerei (E1.2): tabla de sah -> cv2.calibrateCamera ->
config/camera_pi.yaml.

    # ChArUco (recomandat), pe Pi:
    python3 tools/calibrate_camera.py --live --target charuco \
        --cols 9 --rows 6 --square-mm 37 --marker-mm 27.75

    # tabla de sah clasica, dintr-un director de imagini:
    python3 tools/calibrate_camera.py --from-dir ~/calib_imgs \
        --target checker --cols 8 --rows 5 --square-mm 37

**ATENTIE la ce inseamna --cols/--rows.** Pentru `checker` sunt COLTURI
INTERIOARE; pentru `charuco` sunt PATRATE, ca in constructorul
`cv2.aruco.CharucoBoard`. `tools/make_calib_target.py` scrie langa PNG un
JSON cu ambele si afiseaza comanda gata formata - foloseste-o de acolo.

**De ce ChArUco.** La 102 grade FOV vrem tinta mare in cadru, inclusiv poze
de aproape unde depaseste marginile. Tabla clasica pierde poza intreaga daca
un singur colt iese din cadru; ChArUco tolereaza vederi partiale si tot
produce puncte utile, pentru ca fiecare colt e identificat de markerii din
jur. Masurat: vezi §5.19 din CLAUDE.md.

Latura patratului trebuie sa fie cea MASURATA cu rigla dupa tipar, nu cea
nominala. Ea da scara pozelor de calibrare; o valoare gresita nu schimba
intrinsecii, dar strica orice distanta raportata.

De ce nu merge fara asta: la 102 grade FOV distorsiunea radiala e severa la
margini, iar solvePnP cu coeficienti zero da erori de pozitie care cresc
exact acolo unde se afla markerul in timpul apropierii. Focala derivata
geometric (W/2 / tan(HFOV/2)) e un punct de plecare, nu un substitut.

Criteriu: eroare de reproiectie RMS peste 0.5 px = calibrare proasta.
Unealta REFUZA sa salveze. Cauze tipice: tabla indoita, cadre cu blur,
acoperire slaba a marginilor, prea putine poze.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.detector_pi import (CameraCalibration, MAX_REPROJ_ERR_PX,  # noqa: E402
                              TRACK_SIZE, ImageDirSource)

DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'camera_pi.yaml')

#: Acoperirea cadrului se urmareste pe o grila 3x3: distorsiunea se
#: calibreaza acolo unde exista puncte, deci marginile si colturile trebuie
#: sa fie vizitate explicit.
COVERAGE_GRID = (3, 3)

#: In modul live, un cadru nou se accepta doar daca tabla s-a mutat destul
#: fata de ultimul acceptat si a trecut un interval minim - altfel 20 de
#: poze aproape identice nu aduc nimic.
LIVE_MIN_MOVE_PX = 40.0
LIVE_MIN_INTERVAL_S = 0.7


#: Doua garzi impotriva calibrarii DEGENERATE, adica a unui set de poze care
#: nu constrange geometric intrinsecii. Masurat: 24 de poze identice dau
#: RMS 0.061 px - mai bun decat un set bun! - cu fx gresit cu +754% si k1 cu
#: +429%. RMS-ul NU e un criteriu de valabilitate: masoara cat de bine se
#: potriveste modelul cu punctele date, nu daca punctele spun ceva despre
#: camera. Vezi §5.22 din CLAUDE.md.
#:
#: 1. Acoperirea cadrului: sub atatea celule din 9, refuzam.
MIN_COVERAGE_CELLS = 5
#: 2. Focala fata de cea geometrica (W/2 / tan(HFOV/2)). Obiectivul e de
#: ~102 grade; o focala care se abate atat de mult nu descrie aceasta camera,
#: indiferent ce spune RMS-ul.
FOCAL_SANITY_REL = 0.30

#: ChArUco: sub atatea colturi intr-o vedere, poza nu merita pastrata.
#: `matchImagePoints` cere minimum 4; 6 lasa marja pentru o poza stabila.
CHARUCO_MIN_CORNERS = 6

#: Peste asta o poza e considerata aberanta (colt localizat gresit, cadru
#: miscat) si e scoasa inainte de calibrarea finala. Pragul efectiv e
#: max(OUTLIER_ABS_PX, OUTLIER_REL * mediana), ca sa nu taiem poze bune
#: doar pentru ca setul e in general foarte curat.
OUTLIER_ABS_PX = 1.0
OUTLIER_REL = 2.5


def find_corners(gray, pattern):
    """Colturile interioare, sau None.

    Preferam detectorul sector-based (findChessboardCornersSB): localizeaza
    colturile direct la sub-pixel, fara cornerSubPix, si e mult mai robust la
    perspectiva puternica si la marginile cadrului - exact unde ne trebuie
    puncte pentru distorsiune. Cel clasic ramane ca rezerva."""
    if hasattr(cv2, 'findChessboardCornersSB'):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | getattr(cv2, 'CALIB_CB_ACCURACY', 0)
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        if ok:
            return corners.reshape(-1, 2).astype(np.float32)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    return corners.reshape(-1, 2)


def object_points(pattern, square_mm):
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_mm)
    return objp


# --- tinte -------------------------------------------------------------------
# Fiecare tinta stie sa se gaseasca intr-un cadru si sa intoarca perechile
# (puncte-obiect, puncte-imagine) pentru acel cadru. ChArUco intoarce un
# subset diferit la fiecare poza; tabla clasica intoarce mereu tot tiparul.

class CheckerTarget:
    """Tabla de sah clasica. `pattern` = (colturi_x, colturi_y)."""

    name = 'checker'

    def __init__(self, pattern, square_mm):
        self.pattern = tuple(pattern)
        self.square_mm = float(square_mm)
        self.objp = object_points(self.pattern, self.square_mm)

    def describe(self):
        return (f"tabla de sah {self.pattern[0]}x{self.pattern[1]} colturi "
                f"interioare, patrat {self.square_mm:g} mm")

    def detect(self, gray):
        """(objp, imgp, colturi) sau None."""
        c = find_corners(gray, self.pattern)
        if c is None:
            return None
        return self.objp, c.reshape(-1, 2).astype(np.float32), c

    def expected_corners(self):
        return self.pattern[0] * self.pattern[1]

    def meta(self):
        return {
            'target_type': 'checker',
            'inner_corners': f"{self.pattern[0]}x{self.pattern[1]}",
            'square_mm_measured': self.square_mm,
        }


class CharucoTarget:
    """ChArUco. `squares` = (coloane, linii) de PATRATE, ca in CharucoBoard.

    Nu exista `cv2.aruco.calibrateCameraCharuco` in OpenCV 5 - a fost
    eliminat. Calea moderna, si cea folosita aici, e
    `CharucoDetector.detectBoard` -> `board.matchImagePoints` ->
    `cv2.calibrateCamera`, care e echivalenta si merge cu vederi partiale.
    Vezi §5.19 din CLAUDE.md.
    """

    name = 'charuco'

    def __init__(self, squares, square_mm, marker_mm=None,
                 dict_name='DICT_5X5_250', min_corners=CHARUCO_MIN_CORNERS):
        self.squares = tuple(squares)
        self.square_mm = float(square_mm)
        self.marker_mm = float(marker_mm if marker_mm else square_mm * 0.75)
        if self.marker_mm >= self.square_mm:
            raise ValueError(
                f"markerul ({self.marker_mm:g} mm) trebuie sa fie mai mic "
                f"decat patratul ({self.square_mm:g} mm)")
        self.dict_name = dict_name
        self.min_corners = int(min_corners)
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.board = cv2.aruco.CharucoBoard(self.squares, self.square_mm,
                                            self.marker_mm, d)
        self.detector = cv2.aruco.CharucoDetector(self.board)

    def describe(self):
        return (f"ChArUco {self.squares[0]}x{self.squares[1]} patrate "
                f"({self.squares[0] - 1}x{self.squares[1] - 1} colturi), "
                f"patrat {self.square_mm:g} mm, marker {self.marker_mm:g} mm, "
                f"{self.dict_name}")

    def detect(self, gray):
        cor, ids, _m_cor, _m_ids = self.detector.detectBoard(gray)
        if cor is None or ids is None or len(cor) < self.min_corners:
            return None
        objp, imgp = self.board.matchImagePoints(cor, ids)
        if objp is None or len(objp) < self.min_corners:
            return None
        return (objp.reshape(-1, 3).astype(np.float32),
                imgp.reshape(-1, 2).astype(np.float32),
                cor.reshape(-1, 2))

    def expected_corners(self):
        return (self.squares[0] - 1) * (self.squares[1] - 1)

    def meta(self):
        return {
            'target_type': 'charuco',
            'squares': f"{self.squares[0]}x{self.squares[1]}",
            'inner_corners': f"{self.squares[0] - 1}x{self.squares[1] - 1}",
            'square_mm_measured': self.square_mm,
            'marker_mm_measured': self.marker_mm,
            'aruco_dict': self.dict_name,
        }


def make_target(kind, cols, rows, square_mm, marker_mm=None):
    if kind == 'charuco':
        return CharucoTarget((cols, rows), square_mm, marker_mm)
    return CheckerTarget((cols, rows), square_mm)


def coverage(image_size, corner_sets, grid=COVERAGE_GRID):
    """Matrice booleana grid[r][c]: celula a fost vizitata de vreun colt."""
    w, h = image_size
    gr, gc = grid
    seen = [[False] * gc for _ in range(gr)]
    for pts in corner_sets:
        for x, y in pts:
            c = min(int(x / w * gc), gc - 1)
            r = min(int(y / h * gr), gr - 1)
            seen[r][c] = True
    return seen


def coverage_text(seen):
    rows = ['  ' + ''.join('[X]' if v else '[ ]' for v in row) for row in seen]
    n = sum(sum(row) for row in seen)
    total = len(seen) * len(seen[0])
    return '\n'.join(rows) + f"\n  acoperire: {n}/{total} celule"


def _per_image_errors(obj_pts, img_pts, K, dist, rvecs, tvecs):
    """Eroarea de reproiectie RMS a fiecarei poze. `obj_pts` e o lista - la
    ChArUco fiecare vedere are alt subset de colturi."""
    errs = []
    for op, ip, rv, tv in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        d = proj.reshape(-1, 2) - ip.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(d * d, axis=1)))))
    return errs


def calibrate_points(detections, image_size, reject_outliers=True,
                     source_note=''):
    """CameraCalibration din perechi (obj_pts, img_pts), cate una per poza.

    Cu reject_outliers, pozele a caror eroare de reproiectie e aberanta fata
    de restul (colt localizat gresit, cadru miscat) sunt scoase si se
    recalibreaza o data. O singura poza proasta din 24 poate duce RMS-ul
    de la 0.15 la 1.8 px si ar face unealta sa refuze un set altfel bun -
    sau, mai rau, sa il accepte cu coeficienti trasi de un punct fals."""
    obj = [np.asarray(o, np.float32).reshape(-1, 1, 3) for o, _ in detections]
    img = [np.asarray(i, np.float32).reshape(-1, 1, 2) for _, i in detections]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, image_size,
                                                     None, None)
    errs = _per_image_errors(obj, img, K, dist, rvecs, tvecs)
    dropped = []
    if reject_outliers and len(img) > 3:
        thr = max(OUTLIER_ABS_PX, OUTLIER_REL * float(np.median(errs)))
        keep = [i for i, e in enumerate(errs) if e <= thr]
        dropped = [i for i in range(len(img)) if i not in keep]
        if dropped and len(keep) >= 3:
            obj = [obj[i] for i in keep]
            img = [img[i] for i in keep]
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj, img, image_size, None, None)
            errs = _per_image_errors(obj, img, K, dist, rvecs, tvecs)
    cal = CameraCalibration(K, dist.reshape(-1), image_size[0],
                            image_size[1], rms=rms, n_images=len(img),
                            source=f"calibrateCamera, {source_note}"
                                   f"{len(dropped)} poze respinse, "
                                   f"{time.strftime('%Y-%m-%d %H:%M')}")
    cal.per_image_errors = errs
    cal.dropped = dropped
    return cal


def calibrate(corner_sets, image_size, pattern, square_mm,
              reject_outliers=True):
    """Forma clasica, pentru tabla de sah: acelasi tipar in toate pozele."""
    objp = object_points(pattern, square_mm)
    dets = [(objp, np.asarray(c, np.float32).reshape(-1, 2))
            for c in corner_sets]
    cal = calibrate_points(dets, image_size, reject_outliers,
                           source_note=f"tabla {pattern[0]}x{pattern[1]} @ "
                                       f"{square_mm} mm, ")
    cal.meta.update({'target_type': 'checker',
                     'inner_corners': f"{pattern[0]}x{pattern[1]}",
                     'square_mm_measured': float(square_mm)})
    return cal


def save_if_acceptable(cal, path, max_rms=MAX_REPROJ_ERR_PX, min_images=20,
                       seen=None):
    """True daca s-a salvat. Refuza explicit, cu motiv, altfel."""
    if cal.n_images < min_images:
        print(f"  REFUZ: {cal.n_images} imagini, minimum {min_images}.")
        return False
    if cal.rms is None or cal.rms > max_rms:
        print(f"  REFUZ: eroare de reproiectie {cal.rms:.3f} px > {max_rms} px.")
        print("  Cauze tipice: tabla indoita, cadre cu blur, acoperire slaba"
              " a marginilor, prea putine poze. Reia captura.")
        return False

    # Garzile impotriva calibrarii degenerate. Se verifica DUPA RMS tocmai
    # pentru ca un set degenerat trece pragul de RMS fara probleme.
    geo = CameraCalibration.geometric(cal.width, cal.height)
    dev = abs(cal.fx / geo.fx - 1)
    if dev > FOCAL_SANITY_REL:
        print(f"  REFUZ: focala {cal.fx:.0f} px se abate cu {dev:+.0%} de la "
              f"cea geometrica ({geo.fx:.0f} px).")
        print(f"  RMS-ul de {cal.rms:.3f} px nu salveaza asta: un set de poze "
              f"care nu constrange\n  geometria da RMS mic si parametri "
              f"absurzi. Vezi §5.22 din CLAUDE.md.")
        return False
    if seen is not None:
        n_cells = sum(sum(r) for r in seen)
        if n_cells < MIN_COVERAGE_CELLS:
            print(f"  REFUZ: acoperire {n_cells}/9 celule, minimum "
                  f"{MIN_COVERAGE_CELLS}.")
            print("  Fara poze in zone diferite ale cadrului, distorsiunea e "
                  "extrapolata,\n  iar focala si centrul optic nu sunt "
                  "constranse. Misca tinta prin tot cadrul.")
            return False
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cal.save(path)
    print(f"  salvat: {path}")
    return True


def report(cal, seen, target=None, n_total=None):
    print(f"\n  {cal}")
    if target is not None:
        print(f"  tinta: {target.describe()}")
    if n_total:
        print(f"  poze: {n_total} incercate, {cal.n_images} folosite, "
              f"{len(getattr(cal, 'dropped', []))} respinse ca aberante")
    print(f"  eroare de reproiectie RMS: {cal.rms:.3f} px "
          f"(prag {MAX_REPROJ_ERR_PX})")
    errs = getattr(cal, 'per_image_errors', None)
    if errs:
        print(f"  per poza: min {min(errs):.3f}  mediana "
              f"{float(np.median(errs)):.3f}  max {max(errs):.3f} px")
    dropped = getattr(cal, 'dropped', [])
    if dropped:
        print(f"  poze respinse ca aberante: {len(dropped)} "
              f"(indici {dropped}) - colt localizat gresit sau cadru miscat")
    geo = CameraCalibration.geometric(cal.width, cal.height)
    print(f"  fata de focala geometrica {geo.fx:.1f} px: "
          f"fx {cal.fx / geo.fx - 1:+.1%}, fy {cal.fy / geo.fy - 1:+.1%}")
    print(f"  centru optic la ({cal.cx - cal.width / 2:+.1f}, "
          f"{cal.cy - cal.height / 2:+.1f}) px de centrul cadrului")
    print(f"  distorsiune k1={cal.dist[0]:+.4f} k2={cal.dist[1]:+.4f}"
          + (f" k3={cal.dist[4]:+.4f}" if len(cal.dist) > 4 else ''))
    print("  acoperirea cadrului:\n" + coverage_text(seen))
    if not all(all(r) for r in seen):
        print("  ATENTIE: celule nevizitate - distorsiunea de acolo e "
              "extrapolata, nu masurata. Marginile conteaza cel mai mult.")


def collect_from_dir(path, target):
    """(detectii, seturi_de_colturi, dimensiune)."""
    src = ImageDirSource(path)
    dets, sets, size = [], [], None
    n = 0
    total = target.expected_corners()
    while True:
        item = src.read()
        if item is None:
            break
        gray, _ = item
        n += 1
        size = (gray.shape[1], gray.shape[0])
        got = target.detect(gray)
        if got is None:
            print(f"    [{n}] tinta negasita")
            continue
        objp, imgp, cor = got
        dets.append((objp, imgp))
        sets.append(cor)
        partial = '' if len(objp) == total else f" (partiala: {len(objp)}/{total})"
        print(f"    [{n}] ok, {len(objp)} puncte{partial} "
              f"({len(dets)} acceptate)")
    return dets, sets, size


def collect_live(target, min_images, max_images):
    from nova.detector_pi import PiCameraSource
    src = PiCameraSource(verbose=True)
    print(f"\n  Misca tabla prin TOT cadrul, mai ales pe margini si colturi."
          f"\n  Accept un cadru cand tabla e gasita si s-a mutat >= "
          f"{LIVE_MIN_MOVE_PX:.0f} px. Ctrl-C cand ai destule.\n")
    dets, sets, last_c, last_t = [], [], None, 0.0
    size = TRACK_SIZE
    lens = None
    total = target.expected_corners()
    try:
        while len(dets) < max_images:
            gray, _ = src.read()
            size = (gray.shape[1], gray.shape[0])
            got = target.detect(gray)
            now = time.monotonic()
            if got is None:
                print("\r  tinta: negasita           ", end='', flush=True)
                continue
            objp, imgp, c = got
            moved = (last_c is None
                     or np.linalg.norm(c.mean(axis=0) - last_c.mean(axis=0))
                     >= LIVE_MIN_MOVE_PX)
            if moved and now - last_t >= LIVE_MIN_INTERVAL_S:
                dets.append((objp, imgp))
                sets.append(c)
                last_c, last_t = c, now
                seen = coverage(size, sets)
                cov = sum(sum(r) for r in seen)
                print(f"\r  acceptat #{len(dets)}  {len(objp)}/{total} puncte"
                      f"  acoperire {cov}/9  "
                      f"{'(minim atins)' if len(dets) >= min_images else ''}"
                      f"      ", flush=True)
            else:
                print("\r  tinta: gasita, misc-o     ", end='', flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        # LensPosition-ul e parte din calibrare: focusul schimba intrinsecii.
        try:
            lens = src.picam2.capture_metadata().get('LensPosition')
        except Exception:                                   # noqa: BLE001
            lens = None
        src.close()
    return dets, sets, size, lens


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--from-dir', help='director cu imagini ale tintei')
    p.add_argument('--live', action='store_true',
                   help='captura asistata de la picamera2')
    p.add_argument('--target', choices=('charuco', 'checker'),
                   default='charuco')
    p.add_argument('--cols', type=int, default=9,
                   help='charuco: PATRATE | checker: COLTURI interioare')
    p.add_argument('--rows', type=int, default=6,
                   help='charuco: PATRATE | checker: COLTURI interioare')
    p.add_argument('--square-mm', type=float, default=None,
                   help='latura MASURATA a patratului, dupa tipar')
    p.add_argument('--marker-mm', type=float, default=None,
                   help='charuco: latura masurata a markerului (implicit 75%%)')
    p.add_argument('--min-images', type=int, default=20)
    p.add_argument('--max-images', type=int, default=60)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--max-rms', type=float, default=MAX_REPROJ_ERR_PX)
    a = p.parse_args()

    if a.square_mm is None:
        p.error('--square-mm e obligatoriu: latura MASURATA cu rigla dupa '
                'tipar, nu cea nominala')
    try:
        target = make_target(a.target, a.cols, a.rows, a.square_mm,
                             a.marker_mm)
    except ValueError as e:
        print(f"\n  REFUZ: {e}\n")
        return 1
    print(f"  tinta: {target.describe()}")

    lens = None
    if a.from_dir:
        dets, sets, size = collect_from_dir(a.from_dir, target)
    elif a.live:
        dets, sets, size, lens = collect_live(target, a.min_images,
                                              a.max_images)
    else:
        p.error('alege --from-dir sau --live')

    n_total = len(dets)
    if n_total < a.min_images:
        print(f"\n  {n_total} imagini utile, minimum {a.min_images}. "
              f"Nu calibrez.")
        return 1
    print(f"\n  calibrez din {n_total} imagini la {size[0]}x{size[1]} ...")
    cal = calibrate_points(dets, size,
                           source_note=f"{target.describe()}, ")
    # Trasabilitate pentru Compliance Matrix, scrisa in camera_pi.yaml.
    cal.meta.update(target.meta())
    cal.meta.update({
        'n_images_accepted': cal.n_images,
        'n_images_rejected': len(cal.dropped),
        'n_images_attempted': n_total,
        'rms_px': round(float(cal.rms), 4),
        'resolution': f"{size[0]}x{size[1]}",
        'lens_position': lens,
        'calibrated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    })
    seen = coverage(size, sets)
    report(cal, seen, target, n_total)
    return 0 if save_if_acceptable(cal, a.out, a.max_rms, a.min_images,
                                   seen) else 1


if __name__ == '__main__':
    sys.exit(main())
