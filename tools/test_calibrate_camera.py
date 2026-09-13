#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru tools/calibrate_camera.py (E1.2), fara camera.

    python3 tools/test_calibrate_camera.py

Tabla de sah e RANDATA prin intrinseci cunoscuti (K + distorsiune), la poze
cunoscute, apoi unealta trebuie sa recupereze K-ul. Verificam cifre (focala
in 1.5%, RMS sub prag), nu doar ca "a rulat".
"""

import math
import os
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova.detector_pi import CameraCalibration  # noqa: E402
import calibrate_camera as cc  # noqa: E402

#: Rezolutie redusa (raport 16:9, aceeasi geometrie) ca testul sa dureze
#: secunde, nu minute. Unealta nu depinde de rezolutie.
W, H = 1152, 648
PATTERN = (9, 6)          # colturi interioare (cols, rows)
SQUARE_MM = 25.0
SQ_PX = 60                # patratul in imaginea-sursa a tablei
MARGIN_PX = 60            # bordura alba din jurul tablei (obligatorie)


def truth_calibration():
    geo = CameraCalibration.geometric(W, H, 102.0)
    return CameraCalibration(geo.K, [-0.05, 0.008, 0.0, 0.0, 0.0], W, H,
                             rms=0.0, n_images=1, source='adevar (test)')


def board_image():
    cols, rows = PATTERN
    nx, ny = cols + 1, rows + 1                        # patrate
    img = np.full((ny * SQ_PX + 2 * MARGIN_PX, nx * SQ_PX + 2 * MARGIN_PX),
                  255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                y0, x0 = MARGIN_PX + j * SQ_PX, MARGIN_PX + i * SQ_PX
                img[y0:y0 + SQ_PX, x0:x0 + SQ_PX] = 0
    return img


def px_to_mm(px, py):
    """Pixel din imaginea-sursa -> mm in planul tablei, cu originea in primul
    colt interior (aceeasi conventie ca cc.object_points)."""
    o = MARGIN_PX + SQ_PX
    return ((px - o) / SQ_PX * SQUARE_MM, (py - o) / SQ_PX * SQUARE_MM)


_GRID = None


def _pixel_grid():
    """Toate pixelii cadrului, o singura data."""
    global _GRID
    if _GRID is None:
        u, v = np.meshgrid(np.arange(W, dtype=np.float64),
                           np.arange(H, dtype=np.float64))
        _GRID = np.stack([u.ravel(), v.ravel()], axis=1)
    return _GRID


def render(cal, R, t, board):
    """Randare FIDELA prin modelul de distorsiune, pixel cu pixel.

    O homografie e exacta doar in cele 4 colturi prin care e definita; cu
    distorsiune radiala, colturile interioare ale tablei ar cadea in alta
    parte decat unde le-ar vedea camera reala. Prima varianta a acestui test
    facea exact asta si dadea RMS 1.9 px - un artefact de randare, nu al
    uneltei. Aici: pentru fiecare pixel al cadrului, il dezdistorsionam,
    intersectam raza cu planul tablei si citim culoarea din imaginea-sursa."""
    h, w = board.shape
    pts = _pixel_grid().reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pts, cal.K, cal.dist).reshape(-1, 2)
    d = np.concatenate([norm, np.ones((norm.shape[0], 1))], axis=1)   # raze
    Rt = np.asarray(R, np.float64).T
    t = np.asarray(t, np.float64)
    d_b = d @ Rt.T                       # razele in cadrul tablei
    t_b = Rt @ t
    with np.errstate(divide='ignore', invalid='ignore'):
        lam = t_b[2] / d_b[:, 2]
    P = lam[:, None] * d_b - t_b         # punctul pe planul tablei (mm), z=0
    o = MARGIN_PX + SQ_PX
    map_x = (P[:, 0] / SQUARE_MM * SQ_PX + o).astype(np.float32)
    map_y = (P[:, 1] / SQUARE_MM * SQ_PX + o).astype(np.float32)
    bad = ~np.isfinite(lam) | (lam <= 0)
    map_x[bad] = -1
    map_y[bad] = -1
    out = cv2.remap(board, map_x.reshape(H, W), map_y.reshape(H, W),
                    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                    borderValue=128)
    return out


def poses(n, seed=3):
    """Poze care acopera grila 3x3 a cadrului, cu inclinari variate."""
    rng = np.random.default_rng(seed)
    out = []
    cells = [(c, r) for r in range(3) for c in range(3)]
    for k in range(n):
        c, r = cells[k % 9]
        ax = math.radians(rng.uniform(-28, 28))
        ay = math.radians(rng.uniform(-28, 28))
        az = math.radians(rng.uniform(-15, 15))
        R, _ = cv2.Rodrigues(np.array([ax, ay, az]))
        z = rng.uniform(420, 800)
        # centrul tablei (~100 mm, ~62 mm) dus in celula (c, r) a cadrului
        fx = truth_calibration().fx
        u = (c + 0.5) / 3 * W - W / 2 + rng.uniform(-40, 40)
        v = (r + 0.5) / 3 * H - H / 2 + rng.uniform(-30, 30)
        tx = u * z / fx - 100.0
        ty = v * z / fx - 62.5
        out.append((R, np.array([tx, ty, z])))
    return out


def rendered_set(n=24):
    cal = truth_calibration()
    board = board_image()
    frames = [render(cal, R, t, board) for R, t in poses(n)]
    return cal, frames


# --- teste -----------------------------------------------------------------

def test_recupereaza_intrinsecii():
    truth, frames = rendered_set(24)
    sets = []
    for f in frames:
        c = cc.find_corners(f, PATTERN)
        if c is not None:
            sets.append(c)
    assert len(sets) >= 20, f"tabla gasita in doar {len(sets)}/24 cadre"
    cal = cc.calibrate(sets, (W, H), PATTERN, SQUARE_MM)
    assert cal.rms < cc.MAX_REPROJ_ERR_PX, f"rms {cal.rms:.3f}"
    e_fx = abs(cal.fx / truth.fx - 1)
    e_fy = abs(cal.fy / truth.fy - 1)
    assert e_fx < 0.015 and e_fy < 0.015, f"fx {e_fx:.2%} fy {e_fy:.2%}"
    assert abs(cal.cx - truth.cx) < 0.01 * W and abs(cal.cy - truth.cy) < 0.01 * H
    assert abs(cal.dist[0] - truth.dist[0]) < 0.02, f"k1 {cal.dist[0]:+.4f} vs {truth.dist[0]:+.4f}"
    return (f"{len(sets)} imagini, rms {cal.rms:.3f} px, fx {e_fx:+.2%}, "
            f"k1 {cal.dist[0]:+.4f} (adevar {truth.dist[0]:+.3f})")


def test_acoperire_grila():
    truth, frames = rendered_set(24)
    sets = [cc.find_corners(f, PATTERN) for f in frames]
    sets = [s for s in sets if s is not None]
    seen = cc.coverage((W, H), sets)
    assert all(all(r) for r in seen), cc.coverage_text(seen)
    # o singura poza in centru NU acopera marginile - asta e ce vrem sa vedem
    one = cc.coverage((W, H), sets[4:5])
    assert not all(all(r) for r in one)
    return "24 poze: 9/9 celule; 1 poza: incompleta, cum trebuie"


def test_NEGATIV_refuza_rms_mare():
    tmp = os.path.join(tempfile.mkdtemp(), 'cam.yaml')
    truth = truth_calibration()
    bad = CameraCalibration(truth.K, truth.dist, W, H, rms=0.9, n_images=30)
    assert cc.save_if_acceptable(bad, tmp) is False
    assert not os.path.exists(tmp), "a scris fisierul desi a refuzat"
    return "rms 0.9 px: refuzat, nimic scris"


def test_NEGATIV_refuza_prea_putine():
    tmp = os.path.join(tempfile.mkdtemp(), 'cam.yaml')
    truth = truth_calibration()
    few = CameraCalibration(truth.K, truth.dist, W, H, rms=0.2, n_images=9)
    assert cc.save_if_acceptable(few, tmp) is False
    assert not os.path.exists(tmp)
    return "9 imagini: refuzat"


def test_salveaza_si_reincarca():
    tmp = os.path.join(tempfile.mkdtemp(), 'cam.yaml')
    truth = truth_calibration()
    good = CameraCalibration(truth.K, truth.dist, W, H, rms=0.2, n_images=25,
                             source='test')
    assert cc.save_if_acceptable(good, tmp) is True
    back = CameraCalibration.load(tmp)          # require_real=True
    assert np.allclose(back.K, truth.K) and back.n_images == 25
    return "salvat si reincarcat cu require_real"


def test_NEGATIV_tabla_gresita_nu_e_gasita():
    """Cerem 7x5 pe o tabla 9x6: nicio detectie, nicio calibrare falsa."""
    truth, frames = rendered_set(6)
    found = [cc.find_corners(f, (7, 5)) for f in frames]
    assert all(c is None for c in found), "a 'gasit' un tipar gresit"
    return "tipar 7x5 pe tabla 9x6: negasit"


TESTS = [
    ('recupereaza intrinsecii din tabla sintetica', test_recupereaza_intrinsecii),
    ('acoperirea grilei 3x3', test_acoperire_grila),
    ('NEGATIV: refuza rms > 0.5', test_NEGATIV_refuza_rms_mare),
    ('NEGATIV: refuza sub 20 imagini', test_NEGATIV_refuza_prea_putine),
    ('salveaza si reincarca', test_salveaza_si_reincarca),
    ('NEGATIV: tipar gresit negasit', test_NEGATIV_tabla_gresita_nu_e_gasita),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            note = fn()
            print(f"  OK    {name}" + (f"   ({note})" if note else ""))
        except AssertionError as e:
            fails += 1
            print(f"  ESEC  {name}\n        {e}")
        except Exception as e:                      # noqa: BLE001
            fails += 1
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
