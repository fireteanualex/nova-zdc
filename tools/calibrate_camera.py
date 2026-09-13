#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Calibrarea camerei (E1.2): tabla de sah -> cv2.calibrateCamera ->
config/camera_pi.yaml.

    # pe Pi, captura asistata de la camera:
    python3 tools/calibrate_camera.py --live --min-images 20

    # de oriunde, dintr-un director de imagini deja capturate:
    python3 tools/calibrate_camera.py --from-dir ~/calib_imgs

Tabla implicita: 9x6 colturi INTERIOARE (adica 10x7 patrate), patrat de 25 mm.
Verifica cu --rows/--cols/--square-mm ce ai printat. Latura reala a
patratului conteaza doar pentru scara pozelor de calibrare, nu pentru
solvePnP pe marker - dar o valoare gresita strica statisticile de
acoperire, deci pune-o corect.

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


def _per_image_errors(objp, img_pts, K, dist, rvecs, tvecs):
    errs = []
    for pts, rv, tv in zip(img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp, rv, tv, K, dist)
        d = proj.reshape(-1, 2) - pts.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(d * d, axis=1)))))
    return errs


def calibrate(corner_sets, image_size, pattern, square_mm,
              reject_outliers=True):
    """CameraCalibration din seturile de colturi. Nu salveaza nimic.

    Cu reject_outliers, pozele a caror eroare de reproiectie e aberanta fata
    de restul (colt localizat gresit, cadru miscat) sunt scoase si se
    recalibreaza o data. O singura poza proasta din 24 poate duce RMS-ul
    de la 0.2 la 1.8 px si ar face unealta sa refuze un set altfel bun -
    sau, mai rau, sa il accepte cu coeficienti trasi de un punct fals."""
    objp = object_points(pattern, square_mm)
    img = [np.asarray(c, dtype=np.float32).reshape(-1, 1, 2)
           for c in corner_sets]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera([objp] * len(img), img,
                                                     image_size, None, None)
    errs = _per_image_errors(objp, img, K, dist, rvecs, tvecs)
    dropped = []
    if reject_outliers and len(img) > 3:
        thr = max(OUTLIER_ABS_PX, OUTLIER_REL * float(np.median(errs)))
        keep = [i for i, e in enumerate(errs) if e <= thr]
        dropped = [i for i in range(len(img)) if i not in keep]
        if dropped and len(keep) >= 3:
            img = [img[i] for i in keep]
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                [objp] * len(img), img, image_size, None, None)
            errs = _per_image_errors(objp, img, K, dist, rvecs, tvecs)
    cal = CameraCalibration(K, dist.reshape(-1), image_size[0],
                            image_size[1], rms=rms, n_images=len(img),
                            source=f"calibrateCamera, tabla {pattern[0]}x"
                                   f"{pattern[1]} @ {square_mm} mm, "
                                   f"{len(dropped)} poze respinse, "
                                   f"{time.strftime('%Y-%m-%d %H:%M')}")
    cal.per_image_errors = errs
    cal.dropped = dropped
    return cal


def save_if_acceptable(cal, path, max_rms=MAX_REPROJ_ERR_PX, min_images=20):
    """True daca s-a salvat. Refuza explicit, cu motiv, altfel."""
    if cal.n_images < min_images:
        print(f"  REFUZ: {cal.n_images} imagini, minimum {min_images}.")
        return False
    if cal.rms is None or cal.rms > max_rms:
        print(f"  REFUZ: eroare de reproiectie {cal.rms:.3f} px > {max_rms} px.")
        print("  Cauze tipice: tabla indoita, cadre cu blur, acoperire slaba"
              " a marginilor, prea putine poze. Reia captura.")
        return False
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cal.save(path)
    print(f"  salvat: {path}")
    return True


def report(cal, seen):
    print(f"\n  {cal}")
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


def collect_from_dir(path, pattern):
    src = ImageDirSource(path)
    sets, size = [], None
    n = 0
    while True:
        item = src.read()
        if item is None:
            break
        gray, _ = item
        n += 1
        size = (gray.shape[1], gray.shape[0])
        c = find_corners(gray, pattern)
        if c is None:
            print(f"    [{n}] tabla negasita")
            continue
        sets.append(c)
        print(f"    [{n}] ok ({len(sets)} acceptate)")
    return sets, size


def collect_live(pattern, min_images, max_images):
    from nova.detector_pi import PiCameraSource
    src = PiCameraSource(verbose=True)
    print(f"\n  Misca tabla prin TOT cadrul, mai ales pe margini si colturi."
          f"\n  Accept un cadru cand tabla e gasita si s-a mutat >= "
          f"{LIVE_MIN_MOVE_PX:.0f} px. Ctrl-C cand ai destule.\n")
    sets, last_c, last_t = [], None, 0.0
    size = TRACK_SIZE
    try:
        while len(sets) < max_images:
            gray, _ = src.read()
            size = (gray.shape[1], gray.shape[0])
            c = find_corners(gray, pattern)
            now = time.monotonic()
            if c is None:
                print("\r  tabla: negasita           ", end='', flush=True)
                continue
            moved = (last_c is None
                     or np.linalg.norm(c.mean(axis=0) - last_c.mean(axis=0))
                     >= LIVE_MIN_MOVE_PX)
            if moved and now - last_t >= LIVE_MIN_INTERVAL_S:
                sets.append(c)
                last_c, last_t = c, now
                seen = coverage(size, sets)
                cov = sum(sum(r) for r in seen)
                print(f"\r  acceptat #{len(sets)}  acoperire {cov}/9  "
                      f"{'(minim atins)' if len(sets) >= min_images else ''}"
                      f"      ", flush=True)
            else:
                print("\r  tabla: gasita, misc-o     ", end='', flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        src.close()
    return sets, size


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--from-dir', help='director cu imagini de tabla')
    p.add_argument('--live', action='store_true',
                   help='captura asistata de la picamera2')
    p.add_argument('--cols', type=int, default=9, help='colturi interioare pe orizontala')
    p.add_argument('--rows', type=int, default=6, help='colturi interioare pe verticala')
    p.add_argument('--square-mm', type=float, default=25.0)
    p.add_argument('--min-images', type=int, default=20)
    p.add_argument('--max-images', type=int, default=60)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--max-rms', type=float, default=MAX_REPROJ_ERR_PX)
    a = p.parse_args()

    pattern = (a.cols, a.rows)
    if a.from_dir:
        sets, size = collect_from_dir(a.from_dir, pattern)
    elif a.live:
        sets, size = collect_live(pattern, a.min_images, a.max_images)
    else:
        p.error('alege --from-dir sau --live')

    if len(sets) < a.min_images:
        print(f"\n  {len(sets)} imagini utile, minimum {a.min_images}. "
              f"Nu calibrez.")
        return 1
    print(f"\n  calibrez din {len(sets)} imagini la {size[0]}x{size[1]} ...")
    cal = calibrate(sets, size, pattern, a.square_mm)
    report(cal, coverage(size, sets))
    return 0 if save_if_acceptable(cal, a.out, a.max_rms, a.min_images) else 1


if __name__ == '__main__':
    sys.exit(main())
