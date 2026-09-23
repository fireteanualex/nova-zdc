#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verificator INDEPENDENT de detectie ArUco (F3).

Pornit din tools/script_cristi.py, adaptat pentru E2. Nu inlocuieste
nova/detector_pi.py - exista tocmai ca a doua implementare, scrisa separat,
cu care se verifica incrucisat rezultatele de pe imagini statice.

    python3 tools/verify_detection.py --images ~/e2/d05_soare --csv d05.csv

**Nu importa nimic din nova/.** Asta e tot rostul uneltei: doua implementari
care dau aceleasi colturi si acelasi tvec sunt evidenta mult mai puternica
decat una singura care se autoconfirma. Daca ar partaja cod, verificarea
incrucisata nu ar mai dovedi nimic. Deci si incarcarea calibrarii, si
verificarea de incadrare in cadru sunt rescrise aici, deliberat.

Trei schimbari fata de scriptul original, toate necesare:

1. **Citeste dintr-un director de imagini**, nu `cv2.VideoCapture`. E2
   lucreaza pe seturi capturate la distante masurate cu ruleta, in trei
   conditii de lumina; un flux live nu e reproductibil.
2. **Calibrarea vine din config/camera_pi.yaml**, nu din formula geometrica
   `f = W * d_full / s`. Formula presupune proiectie liniara si e gresita cu
   zeci de procente la 102 grade FOV; `dist_coeffs` zero e la fel de gresit.
   Unealta REFUZA sa ruleze fara calibrare reala.
3. **`MARKER_SIZE_M = 0.48`** - markerul de concurs, nu cel de 6.7 cm de pe
   banc.
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np

# --- configuratie -----------------------------------------------------------

MARKER_SIZE_M = 0.48
ARUCO_DICT = cv2.aruco.DICT_4X4_50
TARGET_MARKER_ID = 26

#: §5.2 din CLAUDE.md, reimplementat aici ca sa nu depindem de nova/:
#: markerul trebuie sa incapa INTREG in cadru, nu doar centrul lui. Limita e
#: 95% din inaltimea cadrului.
FRAME_FILL_LIMIT = 0.95

IMAGE_EXT = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')

DEFAULT_CALIB = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'camera_pi.yaml')

CSV_HEADER = [
    'file', 'detected', 'marker_id',
    'c0x', 'c0y', 'c1x', 'c1y', 'c2x', 'c2y', 'c3x', 'c3y',
    'tvec_x', 'tvec_y', 'tvec_z', 'distance_m', 'marker_px', 'fits_in_frame',
]


def load_calibration(path):
    """(K, dist, (w, h)) din YAML-ul scris de tools/calibrate_camera.py.

    Citire proprie, prin cv2.FileStorage - nu importam incarcatorul din
    nova/. Refuzam o calibrare care nu e reala: fara numarul de imagini si
    fara eroarea de reproiectie, fisierul poate fi o focala geometrica pusa
    acolo ca sa treaca ceva, si atunci verificarea incrucisata compara doua
    presupuneri, nu doua masuratori."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"lipseste calibrarea: {path}\n"
            f"  Ruleaza tools/calibrate_camera.py. Formula geometrica NU e "
            f"un substitut la 102 grade FOV.")
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"{path}: nu se poate citi")
    K = fs.getNode('camera_matrix').mat()
    dist = fs.getNode('dist_coeffs').mat()
    w = int(fs.getNode('image_width').real())
    h = int(fs.getNode('image_height').real())
    rms = fs.getNode('rms_reprojection_px').real()
    n = int(fs.getNode('n_images').real())
    fs.release()
    if K is None or dist is None or w <= 0 or h <= 0:
        raise ValueError(f"{path}: campuri lipsa")
    if n <= 0 or rms < 0:
        raise ValueError(
            f"{path}: nu e o calibrare reala (n_images={n}, rms={rms}). "
            f"Vezi tools/calibrate_camera.py.")
    return (np.asarray(K, np.float64), np.asarray(dist, np.float64).reshape(-1),
            (w, h), {'rms': rms, 'n_images': n})


def build_marker_object_points(marker_size_m):
    """Colturile markerului in metri, in ordinea ArUco: stanga-sus,
    dreapta-sus, dreapta-jos, stanga-jos."""
    half = marker_size_m / 2.0
    return np.array([
        [-half, half, 0],
        [half, half, 0],
        [half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)


def marker_side_px(corners):
    """Latura medie a patrulaterului detectat."""
    d = np.roll(corners, -1, axis=0) - corners
    return float(np.mean(np.linalg.norm(d, axis=1)))


def fits_in_frame(marker_px, frame_height, limit=FRAME_FILL_LIMIT):
    return bool(marker_px <= limit * frame_height)


class Detector:
    """ArUco + solvePnP. Nicio dependenta de nova/."""

    # DECIZIE DESCHISA: `refine` implicit False.
    # Scriptul original nu rafina colturile, iar rostul acestei unelte e sa fie
    # a doua implementare, nu o copie a celei de bord - daca ii dau aceiasi
    # parametri, comparatia nu mai poate gasi nimic. Masurat pe adevar
    # sintetic, diferenta e reala: fara rafinare eroarea de distanta 1.76%,
    # cu rafinare 0.68%. Deci pentru DATE REALE (E2) varianta corecta e
    # probabil --refine, dar alegerea schimba ce dovedeste comparatia si
    # o las la latitudinea ta. compare_detectors.py ruleaza oricum ambele
    # moduri si le raporteaza separat.
    def __init__(self, K, dist, marker_size_m=MARKER_SIZE_M,
                 marker_id=TARGET_MARKER_ID, refine=False):
        self.K = K
        self.dist = dist
        self.marker_id = int(marker_id)
        self.object_points = build_marker_object_points(marker_size_m)
        params = cv2.aruco.DetectorParameters()
        if refine:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.refine = refine
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params)

    def detect(self, gray):
        """dict cu rezultatele, sau None daca markerul nu e in cadru."""
        corners, ids, _rejected = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) != self.marker_id:
                continue
            pts = marker_corners.reshape((4, 2)).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points, pts, self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
            t = tvec.reshape(3)
            side = marker_side_px(pts)
            return {
                'marker_id': int(marker_id),
                'corners': pts,
                'rvec': rvec.reshape(3),
                'tvec': t,
                'distance_m': float(np.linalg.norm(t)),
                'marker_px': side,
                'fits_in_frame': fits_in_frame(side, gray.shape[0]),
            }
        return None


def list_images(path):
    if os.path.isfile(path):
        return [path]
    files = sorted(os.path.join(path, f) for f in os.listdir(path)
                   if f.lower().endswith(IMAGE_EXT))
    if not files:
        raise FileNotFoundError(f"nicio imagine in {path}")
    return files


def row_for(path, res):
    name = os.path.basename(path)
    if res is None:
        return [name, 0, ''] + [''] * 8 + [''] * 5
    c = res['corners']
    t = res['tvec']
    return ([name, 1, res['marker_id']]
            + [f"{v:.3f}" for v in c.reshape(-1)]
            + [f"{t[0]:.5f}", f"{t[1]:.5f}", f"{t[2]:.5f}",
               f"{res['distance_m']:.5f}", f"{res['marker_px']:.3f}",
               int(res['fits_in_frame'])])


def run(images, detector, csv_path=None, verbose=True):
    """[(cale, rezultat_sau_None)] si, optional, CSV."""
    out = []
    writer = f = None
    if csv_path:
        f = open(csv_path, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
    try:
        for path in images:
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                if verbose:
                    print(f"  {os.path.basename(path)}: necitibila")
                continue
            res = detector.detect(gray)
            out.append((path, res))
            if writer:
                writer.writerow(row_for(path, res))
            if verbose:
                if res is None:
                    print(f"  {os.path.basename(path):<32} nedetectat")
                else:
                    inc = 'da' if res['fits_in_frame'] else 'NU'
                    print(f"  {os.path.basename(path):<32} "
                          f"{res['distance_m']:6.3f} m  "
                          f"{res['marker_px']:7.1f} px  incape={inc}")
    finally:
        if f:
            f.close()
    return out


def summarize(results):
    det = [r for _, r in results if r is not None]
    n = len(results)
    print(f"\n  {len(det)}/{n} detectate ({len(det) / n:.0%})" if n else
          "\n  niciun fisier")
    if not det:
        return
    d = np.array([r['distance_m'] for r in det])
    px = np.array([r['marker_px'] for r in det])
    print(f"  distanta : {d.min():.3f} - {d.max():.3f} m "
          f"(mediana {np.median(d):.3f})")
    print(f"  marker_px: {px.min():.1f} - {px.max():.1f} "
          f"(mediana {np.median(px):.1f})")
    nu_incap = [r for r in det if not r['fits_in_frame']]
    if nu_incap:
        print(f"  {len(nu_incap)} cadre in care markerul NU incape intreg "
              f"(sub ~0.38 m) - vezi §5.2")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--images', required=True,
                   help='director cu imagini, sau o singura imagine')
    p.add_argument('--calib', default=DEFAULT_CALIB)
    p.add_argument('--csv', default=None)
    p.add_argument('--marker-size-m', type=float, default=MARKER_SIZE_M)
    p.add_argument('--marker-id', type=int, default=TARGET_MARKER_ID)
    p.add_argument('--refine', action='store_true',
                   help='rafinare sub-pixel a colturilor (implicit: fara, ca '
                        'in implementarea originala)')
    a = p.parse_args()

    try:
        K, dist, (w, h), info = load_calibration(a.calib)
    except (FileNotFoundError, ValueError) as e:
        print(f"\n  NU RULEZ: {e}\n")
        return 2
    print(f"  calibrare: {a.calib}  {w}x{h}  fx={K[0, 0]:.1f} "
          f"fy={K[1, 1]:.1f}  rms={info['rms']:.3f} px  "
          f"n={info['n_images']}")
    print(f"  marker: ID {a.marker_id}, {a.marker_size_m} m, "
          f"rafinare {'da' if a.refine else 'nu'}\n")

    try:
        images = list_images(a.images)
    except FileNotFoundError as e:
        print(f"  {e}")
        return 1
    det = Detector(K, dist, a.marker_size_m, a.marker_id, a.refine)
    results = run(images, det, a.csv)
    summarize(results)
    if a.csv:
        print(f"\n  CSV: {a.csv}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
