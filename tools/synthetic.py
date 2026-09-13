#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Adevar sintetic pentru validarea uneltelor de viziune (rundele E si F).

Randeaza o tinta PLANA (tabla de sah, ChArUco, marker ArUco) vazuta de o
camera cu matrice si distorsiune CUNOSCUTE, dintr-o poza CUNOSCUTA. Apoi
unealta testata trebuie sa recupereze ce am pus noi acolo. Fara asta, un
test de viziune nu poate spune decat "a rulat", nu "e corect".

**Randarea trece prin acelasi model ca detectia.** Pentru fiecare pixel al
cadrului de iesire: il dezdistorsionam, construim raza, o intersectam cu
planul tintei si citim culoarea din imaginea-sursa. O homografie (varianta
naiva) ar fi exacta doar in cele 4 colturi prin care e definita - punctele
interioare nu ar urma distorsiunea radiala, iar testul ar masura eroarea
randarii, nu a uneltei. Vezi CLAUDE.md §5.11: prima varianta a testului de
calibrare raporta RMS 1.9 px pe date perfecte exact din acest motiv.

Conventii:
  - tinta sta in planul z=0 al cadrului ei propriu, in MILIMETRI
  - (x_mm, y_mm) -> pixel in imaginea-sursa prin `px_per_mm` si `origin_px`
  - R, t duc un punct din cadrul tintei in cadrul camerei: p_cam = R @ p + t

**Axa y a cadrului tintei e IN JOS**, ca in imagine: un punct cu y_mm mai
mare cade mai jos in imaginea-sursa. Deci o tinta plana, dreapta, vazuta de
sus, e `R = identitate` - normala ei iese dinspre camera, ca a unei foi pe
masa privita de deasupra.

Capcana pe care am calcat-o o data: daca definesti colturile obiectului cu y
in SUS (conventia uzuala in 3D) si le randezi prin maparea de aici, iese o
imagine OGLINDITA. Un marker ArUco oglindit nu se decodeaza deloc - nu da
nici macar o detectie gresita, pur si simplu tace. Vezi §5.21 din CLAUDE.md.
"""

import math

import cv2
import numpy as np

_RAY_CACHE = {}


def _rays(w, h, K, dist):
    """Razele dezdistorsionate ale fiecarui pixel, in cadrul camerei.

    Depind doar de (w, h, K, dist), nu de poza tintei, deci se calculeaza o
    singura data pentru o campanie de randari. Fara cache, cele 25 de vederi
    din F2 ar reface de 25 de ori acelasi undistortPoints pe 3 milioane de
    puncte."""
    key = (w, h, np.asarray(K).tobytes(), np.asarray(dist).tobytes())
    if key not in _RAY_CACHE:
        u, v = np.meshgrid(np.arange(w, dtype=np.float64),
                           np.arange(h, dtype=np.float64))
        pts = np.stack([u.ravel(), v.ravel()], axis=1).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, np.asarray(K, np.float64),
                                   np.asarray(dist, np.float64)).reshape(-1, 2)
        _RAY_CACHE[key] = np.concatenate(
            [norm, np.ones((norm.shape[0], 1))], axis=1)
    return _RAY_CACHE[key]


def render_planar_target(src_img, px_per_mm, origin_px, K, dist, R, t,
                         out_wh, bg=128, interp=cv2.INTER_LINEAR,
                         antialias=True):
    """Cadrul vazut de camera. `src_img` e imaginea tintei (gri, uint8).

    px_per_mm, origin_px: un punct (x_mm, y_mm) din planul tintei se afla la
    pixelul (origin_px[0] + x_mm*px_per_mm, origin_px[1] + y_mm*px_per_mm)
    in imaginea-sursa.

    **antialias**: o pagina A3 la 300 DPI are ~4961 px latime; vazuta de la
    700 mm ocupa ~450 px in cadru, deci minificare de ~9x. `cv2.remap`
    esantioneaza punctual, fara prefiltrare, deci produce aliasing masiv.
    Efectul nu e cosmetic: masurat, reproiectia unei tinte ChArUco a scazut
    de la 0.498 px la 0.070 px doar prin prefiltrarea sursei. Fara asta,
    testul masoara aliasingul randorului, nu unealta - exact capcana din
    §5.11. Prefiltram sursa cu un Gaussian potrivit minificarii.
    """
    w, h = out_wh
    K = np.asarray(K, np.float64)
    dist = np.asarray(dist, np.float64).reshape(-1)
    R = np.asarray(R, np.float64).reshape(3, 3)
    t = np.asarray(t, np.float64).reshape(3)

    rays = _rays(w, h, K, dist)

    Rt = R.T
    rays_b = rays @ Rt.T          # razele, in cadrul tintei
    t_b = Rt @ t                  # camera, in cadrul tintei
    with np.errstate(divide='ignore', invalid='ignore'):
        lam = t_b[2] / rays_b[:, 2]
    P = lam[:, None] * rays_b - t_b        # punctul pe planul z=0 (mm)

    map_x = (P[:, 0] * px_per_mm + origin_px[0]).astype(np.float32)
    map_y = (P[:, 1] * px_per_mm + origin_px[1]).astype(np.float32)
    bad = ~np.isfinite(lam) | (lam <= 0)   # in spatele camerei sau paralel
    map_x[bad] = -1
    map_y[bad] = -1

    if antialias:
        good = ~bad
        if np.any(good):
            # Adancimea mediana a planului in cadru; minificarea e
            # px_per_mm * adancime / focala. Un singur Gaussian pe toata
            # sursa: pentru inclinarile noastre (sub ~40 grade) variatia de
            # scara in cadru e mica. La inclinari extreme ar trebui blur
            # variabil - de retinut daca se adauga astfel de teste.
            depth = np.median((lam[good] * rays[good, 2]))
            f = 0.5 * (K[0, 0] + K[1, 1])
            minif = float(px_per_mm * depth / f)
            if minif > 1.5:
                src_img = cv2.GaussianBlur(src_img, (0, 0), minif / 2.0)

    return cv2.remap(src_img, map_x.reshape(h, w), map_y.reshape(h, w),
                     interp, borderMode=cv2.BORDER_CONSTANT, borderValue=bg)


def project(K, dist, R, t, pts_mm):
    """Punctele tintei (mm, z=0 implicit daca sunt 2D) -> pixeli."""
    pts = np.asarray(pts_mm, np.float64)
    if pts.shape[1] == 2:
        pts = np.concatenate([pts, np.zeros((len(pts), 1))], axis=1)
    rvec, _ = cv2.Rodrigues(np.asarray(R, np.float64))
    img, _ = cv2.projectPoints(pts, rvec, np.asarray(t, np.float64),
                               np.asarray(K, np.float64),
                               np.asarray(dist, np.float64))
    return img.reshape(-1, 2)


def rot(rx_deg=0.0, ry_deg=0.0, rz_deg=0.0):
    """Rotatie din unghiuri in grade (aplicate ca vector Rodrigues)."""
    v = np.array([math.radians(rx_deg), math.radians(ry_deg),
                  math.radians(rz_deg)], np.float64)
    R, _ = cv2.Rodrigues(v)
    return R


#: Camera tinta: IMX708 Wide la rezolutia de tracking. Focala geometrica
#: (W/2 / tan(HFOV/2)) si o distorsiune radiala moderata, plauzibila pentru
#: un obiectiv de 102 grade. NU sunt masurate - sunt adevarul sintetic fata
#: de care se verifica uneltele.
IMX708_W, IMX708_H = 2304, 1296
IMX708_FOCAL_PX = (IMX708_W / 2.0) / math.tan(math.radians(102.0 / 2.0))
IMX708_K = np.array([[IMX708_FOCAL_PX, 0, IMX708_W / 2.0],
                     [0, IMX708_FOCAL_PX, IMX708_H / 2.0],
                     [0, 0, 1]], np.float64)
IMX708_DIST = np.array([-0.05, 0.008, 0.0, 0.0, 0.0], np.float64)


def imx708(width=IMX708_W, height=IMX708_H, k1=-0.05, k2=0.008):
    """(K, dist, (w, h)) pentru camera sintetica de referinta."""
    f = (width / 2.0) / math.tan(math.radians(102.0 / 2.0))
    K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]],
                 np.float64)
    return K, np.array([k1, k2, 0.0, 0.0, 0.0], np.float64), (width, height)


def aruco_marker_image(marker_id, dictionary, module_px=50, quiet_modules=1):
    """Imaginea unui marker ArUco cu zona linistita, si latura zonei codate
    in pixeli-sursa. Zona linistita e obligatorie pentru detectie."""
    n = dictionary.markerSize + 2                  # module, cu bordura neagra
    coded = n * module_px
    q = quiet_modules * module_px
    img = np.full((coded + 2 * q, coded + 2 * q), 255, np.uint8)
    img[q:q + coded, q:q + coded] = cv2.aruco.generateImageMarker(
        dictionary, marker_id, coded)
    return img, coded, q


#: Colturile markerului in cadrul lui propriu, cu y IN JOS, in ordinea pe
#: care o raporteaza `detectMarkers`: stanga-sus, dreapta-sus, dreapta-jos,
#: stanga-jos. Cu y in jos, "sus" inseamna y negativ.
def marker_corners_mm(size_mm):
    s = size_mm / 2.0
    return np.array([[-s, -s], [s, -s], [s, s], [-s, s]], np.float64)


def render_marker(K, dist, out_wh, R, t_m, marker_id=26, size_m=0.48,
                  dictionary=None, bg=110, module_px=50):
    """Marker ArUco la poza (R, t_m) in METRI. Intoarce (cadru, colturi_px).

    Colturile intoarse sunt ale markerului intreg (date + bordura neagra),
    adica exact ce raporteaza `detectMarkers`, in aceeasi ordine.
    """
    if dictionary is None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    src, coded_px, q = aruco_marker_image(marker_id, dictionary, module_px)
    # Scara: latura markerului (size_m) ocupa coded_px pixeli-sursa.
    size_mm = size_m * 1000.0
    px_per_mm = coded_px / size_mm
    origin_px = (q + coded_px / 2.0, q + coded_px / 2.0)   # centrul markerului
    t_mm = np.asarray(t_m, np.float64) * 1000.0
    frame = render_planar_target(src, px_per_mm, origin_px, K, dist, R, t_mm,
                                 out_wh, bg=bg)
    corners_px = project(K, dist, R, t_mm, marker_corners_mm(size_mm))
    return frame, corners_px


#: Marker plan, drept, vazut de deasupra. Cu conventia y-in-jos de mai sus,
#: asta e pur si simplu identitatea - vezi nota din capul modulului.
R_MARKER_FLAT = np.eye(3)
