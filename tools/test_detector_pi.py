#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru nova/detector_pi.py (E1), fara camera si fara Pi.

    python3 tools/test_detector_pi.py

Markerul e RANDAT sintetic: generam imaginea ArUco, o proiectam printr-o
calibrare cunoscuta (K + distorsiune) la o pozitie cunoscuta (R, t), apoi
cerem detectorului sa recupereze pozitia. Adevarul e cunoscut la fiecare
test, deci verificam cifre, nu "a detectat ceva".

Ce nu acopera: picamera2 (nu exista pe desktop), lumina reala, blur,
hartie reflectorizanta. Alea sunt E2, cu marker printat si ruleta.
"""

import math
import os
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.detection import Detection  # noqa: E402
from nova.detector_pi import (ArraySource, ArucoMarkerDetector,  # noqa: E402
                              CameraCalibration, PiDetector, ARUCO_DICT)

W, H = 2304, 1296
MARKER_M = 0.48
DICT = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

#: Calibrare sintetica "reala": focala geometrica + distorsiune radiala
#: moderata, ca sa exercitam si coeficientii. Marcata ca reala (rms, n) ca sa
#: treaca de load(); in productie o astfel de valoare vine din
#: tools/calibrate_camera.py.
def synthetic_calibration(k1=-0.06, k2=0.01):
    geo = CameraCalibration.geometric(W, H, 102.0)
    return CameraCalibration(geo.K, [k1, k2, 0.0, 0.0, 0.0], W, H,
                             rms=0.21, n_images=25, source='sintetic (test)')


#: Marker pe sol, camera priveste in jos, varful imaginii spre nas: normala
#: markerului e spre camera (-z), iar "sus"-ul markerului e sus in imagine.
R_FLAT = np.diag([1.0, -1.0, -1.0])


def obj_corners(size_m=MARKER_M):
    s = size_m / 2.0
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                    dtype=np.float64)


def project(cal, R, t, pts):
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    img, _ = cv2.projectPoints(np.asarray(pts, dtype=np.float64), rvec,
                               np.asarray(t, dtype=np.float64), cal.K,
                               cal.dist)
    return img.reshape(-1, 2)


def render(cal, R, t, marker_id=26, size_m=MARKER_M, bg=110, module_px=50):
    """Cadru gri HxW cu markerul la pozitia (R, t). Intoarce (imagine,
    colturile proiectate ale zonei codate)."""
    marker = cv2.aruco.generateImageMarker(DICT, marker_id, 4 * module_px)
    q = module_px                                   # zona linistita: 1 modul
    canvas = np.full((4 * module_px + 2 * q, 4 * module_px + 2 * q), 255,
                     np.uint8)
    canvas[q:q + 4 * module_px, q:q + 4 * module_px] = marker
    src = np.array([[q, q], [q + 4 * module_px, q],
                    [q + 4 * module_px, q + 4 * module_px],
                    [q, q + 4 * module_px]], dtype=np.float32)
    dst = project(cal, R, t, obj_corners(size_m)).astype(np.float32)
    Hm = cv2.getPerspectiveTransform(src, dst)
    frame = cv2.warpPerspective(canvas, Hm, (W, H), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=bg)
    return frame, dst


def truth(t, R=R_FLAT):
    t = np.asarray(t, dtype=np.float64)
    n = np.asarray(R)[:, 2]
    return dict(
        angle_x=math.atan2(-t[1], t[2]),
        angle_y=math.atan2(t[0], t[2]),
        distance=float(np.linalg.norm(t)),
        range_m=float(np.dot(t, n) / n[2]),
    )


def check_det(det, t, R=R_FLAT, corners_px=None, tol_dist=0.02, tol_deg=0.3):
    assert det is not None, "nicio detectie"
    tr = truth(t, R)
    e_d = abs(det.distance_m / tr['distance'] - 1)
    assert e_d < tol_dist, f"distanta {det.distance_m:.3f} vs {tr['distance']:.3f} ({e_d:.1%})"
    e_ax = abs(math.degrees(det.angle_x - tr['angle_x']))
    e_ay = abs(math.degrees(det.angle_y - tr['angle_y']))
    assert e_ax < tol_deg, f"angle_x {math.degrees(det.angle_x):.2f} vs {math.degrees(tr['angle_x']):.2f}"
    assert e_ay < tol_deg, f"angle_y {math.degrees(det.angle_y):.2f} vs {math.degrees(tr['angle_y']):.2f}"
    e_r = abs(det.range_m / tr['range_m'] - 1)
    assert e_r < tol_dist, f"range {det.range_m:.3f} vs {tr['range_m']:.3f} ({e_r:.1%})"
    if corners_px is not None:
        side = float(np.mean(np.linalg.norm(
            np.roll(corners_px, -1, axis=0) - corners_px, axis=1)))
        e_px = abs(det.marker_px / side - 1)
        assert e_px < 0.03, f"marker_px {det.marker_px:.1f} vs {side:.1f}"
    return tr


# --- teste -----------------------------------------------------------------

def test_direct_dedesubt_8m():
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (0.0, 0.0, 8.0)
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 12.5)
    check_det(det, t, corners_px=corners)
    assert det.t == 12.5, "timestamp-ul nu e cel al capturii"
    return (f"dist {det.distance_m:.3f} m, {det.marker_px:.1f} px, "
            f"angle ({math.degrees(det.angle_x):+.2f}, "
            f"{math.degrees(det.angle_y):+.2f}) deg")


def test_offset_lateral_conventia_de_montaj():
    """Marker in fata (nas) si la dreapta: angle_x > 0 (inainte = -y_cam),
    angle_y > 0 (dreapta = +x_cam)."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (1.5, -1.0, 6.0)               # x_cam dreapta 1.5, y_cam -1.0 = in fata
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 0.0)
    tr = check_det(det, t, corners_px=corners)
    assert det.angle_x > 0 and det.angle_y > 0
    return (f"inainte {math.degrees(tr['angle_x']):.1f} deg, "
            f"dreapta {math.degrees(tr['angle_y']):.1f} deg, recuperate")


def test_departe_12m_37px():
    """Fereastra operationala cere detectie la 12 m (37 px). Sintetic trebuie
    sa mearga; pe hartie reala e criteriul E2 (>= 95%)."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (0.8, 0.5, 12.0)
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 0.0)
    check_det(det, t, corners_px=corners, tol_dist=0.03)
    assert det.marker_px < 45, det.marker_px
    return f"{det.marker_px:.1f} px la 12 m, dist {det.distance_m:.2f} m"


def test_aproape_045m_incape():
    """La 0.45 m markerul are ~995 px si inca incape (limita e 0.95*H=1231)."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (0.0, 0.0, 0.45)
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 0.0)
    check_det(det, t, corners_px=corners)
    assert 900 < det.marker_px < 1231, det.marker_px
    return f"{det.marker_px:.0f} px la 0.45 m, acceptat"


def test_NEGATIV_nu_incape_in_cadru():
    """§5.2: marker detectat, dar mai mare decat 0.95*H -> None, si contorul
    de respingeri creste. Colturile sunt injectate, ca sa testam decizia
    indiferent daca ArUco ar mai gasi un patrat asa mare."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    big = np.array([[20, 18], [1280, 18], [1280, 1278], [20, 1278]],
                   dtype=np.float32)
    d._find = lambda gray: (big, False)
    det = d.detect(np.zeros((H, W), np.uint8), 0.0)
    assert det is None, "a publicat un marker care nu incape in cadru"
    assert d.n_rejected_fit == 1
    return f"{ArucoMarkerDetector.side_px(big):.0f} px > {0.95 * H:.0f}: respins"


def test_NEGATIV_cadru_gol():
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    assert d.detect(np.full((H, W), 110, np.uint8), 0.0) is None
    rng = np.random.default_rng(1)
    noise = rng.integers(0, 255, (H, W), dtype=np.uint8)
    assert d.detect(noise, 0.0) is None, "a detectat un marker in zgomot"
    return "gol si zgomot: nimic"


def test_NEGATIV_alt_id_ignorat():
    """Doar ID 26 conteaza. Un alt marker din acelasi dictionar e ignorat."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    frame, _ = render(cal, R_FLAT, (0.0, 0.0, 5.0), marker_id=7)
    assert d.detect(frame, 0.0) is None
    assert d.n_rejected_id == 1
    return "ID 7 respins, contorizat"


def test_plan_inclinat_range_pe_axa_optica():
    """Marker inclinat 12 grade: range_m e distanta pe axa optica pana la
    PLAN, nu pana la marker. Verificam formula (t.n)/n_z fata de adevar."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    tilt, _ = cv2.Rodrigues(np.array([math.radians(12.0), 0.0, 0.0]))
    R = tilt @ R_FLAT
    t = (0.3, -0.2, 4.0)
    frame, corners = render(cal, R, t)
    det = d.detect(frame, 0.0)
    tr = check_det(det, t, R=R, corners_px=corners, tol_dist=0.03)
    assert abs(tr['range_m'] - 4.0) > 0.01, "testul nu inclina destul planul"
    return f"range {det.range_m:.3f} m vs adevar {tr['range_m']:.3f} m (t_z=4.000)"


def test_roi_sub_5m():
    """Dupa o detectie sub 5 m, urmatorul cadru se cauta intr-un ROI de
    640x480 centrat pe ultima pozitie. Rezultatul trebuie sa fie identic."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (0.4, 0.3, 3.0)
    frame, corners = render(cal, R_FLAT, t)
    d1 = d.detect(frame, 0.0)
    assert d1 is not None and d.n_roi == 0
    d2 = d.detect(frame, 0.1)
    assert d.n_roi == 1, "al doilea cadru nu a folosit ROI"
    check_det(d2, t, corners_px=corners)
    assert abs(d2.distance_m - d1.distance_m) < 0.005
    return f"ROI folosit, dist {d1.distance_m:.3f} -> {d2.distance_m:.3f} m"


def test_roi_miss_cade_pe_cadrul_intreg():
    """Markerul a sarit in afara ROI-ului: se cauta in tot cadrul, nu se
    pierde detectia."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    f1, _ = render(cal, R_FLAT, (0.0, 0.0, 3.0))
    assert d.detect(f1, 0.0) is not None
    t2 = (1.4, 0.9, 3.0)                       # ~450 px mai incolo
    f2, c2 = render(cal, R_FLAT, t2)
    det = d.detect(f2, 0.1)
    assert d.n_roi_miss == 1, "nu a inregistrat ratarea ROI"
    check_det(det, t2, corners_px=c2)
    return "ratare ROI -> cadru intreg -> detectie corecta"


def test_roi_gating_pe_prag_si_implicitele_de_zbor():
    """Mecanismul de prag se testeaza cu pragul INJECTAT (§5.40), nu cu
    implicitul - implicitul e o decizie care se schimba, si s-a schimbat:
    5 m -> 15 m dupa zborul din 24.09.2026, in care fereastra de handover
    (5-12 m) cadea integral pe cautarea lenta din cadrul intreg (§5.62)."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal, roi_below_m=5.0)
    f, _ = render(cal, R_FLAT, (0.0, 0.0, 7.0))
    d.detect(f, 0.0)
    d.detect(f, 0.1)
    assert d.n_roi == 0, "ROI folosit peste pragul injectat de 5 m"
    # cu implicitul de zbor, aceeasi scena de 7 m FOLOSESTE ROI
    d2 = ArucoMarkerDetector(cal)
    assert d2.detect(f, 0.0) is not None
    assert d2.detect(f, 0.1) is not None and d2.n_roi == 1, (
        "la 7 m, in fereastra portii, al doilea cadru trebuia sa fie pe ROI")
    # paritate: config-ul de zbor = implicitele clasei (§5.56)
    from nova import config as nova_config
    from nova.detector_pi import ROI_BELOW_M, SEARCH_DOWNSCALE
    assert ROI_BELOW_M == 15.0 and nova_config.DEFAULTS['roi_below_m'] == 15.0
    assert nova_config.DEFAULTS['search_downscale'] == SEARCH_DOWNSCALE == 2
    return "prag injectat: gating ok; implicit 15 m: ROI la 7 m; paritate"


def test_cautarea_redusa_gaseste_si_rafineaza():
    """Cadrul intreg la rezolutia plina lua ~320 ms pe Pi 4 si rata
    majoritatea cadrelor la 5-7 m (§5.62). Cautarea pe imaginea redusa
    trebuie sa gaseasca ACELASI marker cu ACEEASI precizie - rafinarea la
    rezolutia plina e cea care o garanteaza."""
    cal = synthetic_calibration()
    t = (0.3, -0.2, 6.0)
    frame, corners = render(cal, R_FLAT, t)
    d1 = ArucoMarkerDetector(cal, search_downscale=1)   # drumul vechi
    det1 = d1.detect(frame, 0.0)
    d2 = ArucoMarkerDetector(cal)                       # drumul de zbor
    det2 = d2.detect(frame, 0.0)
    assert det1 is not None and det2 is not None
    assert d2.n_half == 1 and d1.n_half == 0
    check_det(det2, t, corners_px=corners)
    # rafinarea la rezolutia plina: colturile celor doua drumuri coincid
    assert float(np.max(np.abs(d1.last_corners - d2.last_corners))) < 0.5, (
        "colturile de pe drumul redus difera de cele de la rezolutia plina")
    assert abs(det1.distance_m - det2.distance_m) < 0.01
    return (f"redus+rafinat: dist {det2.distance_m:.3f} m vs "
            f"{det1.distance_m:.3f} m, colturi identice sub 0.5 px")


def test_pidetector_timestamps_si_statistici():
    """PiDetector fara fir: fiecare Detection poarta timestamp-ul CAPTURII,
    iar statisticile sunt percentile, nu medii."""
    cal = synthetic_calibration()
    frames, stamps = [], []
    for i in range(12):
        f, _ = render(cal, R_FLAT, (0.05 * i, 0.0, 6.0))
        frames.append(f)
        stamps.append(1000.0 + i / 30.0)
    frames.insert(6, np.full((H, W), 110, np.uint8))   # un cadru fara marker
    stamps.insert(6, 1000.0 + 5.5 / 30.0)
    pd = PiDetector(ArraySource(frames, stamps), ArucoMarkerDetector(cal),
                    threaded=False)
    got = []
    for _ in range(len(frames) + 2):
        got.extend(pd.poll(time.monotonic()))
    assert len(got) == 12, f"{len(got)} detectii din 12 cadre cu marker"
    assert all(isinstance(g, Detection) for g in got)
    assert [g.t for g in got] == [s for i, s in enumerate(stamps) if i != 6]
    st = pd.stats()
    assert abs(st['detection_rate'] - 12 / 13) < 1e-9, st['detection_rate']
    assert pd.exhausted
    return f"12/13 detectii, rata {st['detection_rate']:.0%}, timestamp pastrat"


def test_pidetector_latenta_reala_pe_desktop():
    """Sursa stampileaza la citire, deci latenta = timpul de detectie. Nu e
    Pi 4, dar e prima cifra reala pentru 2304x1296 si pentru ROI."""
    cal = synthetic_calibration()
    far = [render(cal, R_FLAT, (0.3 * (i % 3), 0.0, 9.0))[0] for i in range(15)]
    pd = PiDetector(ArraySource(far), ArucoMarkerDetector(cal), threaded=False)
    while pd.poll(0.0) or not pd.exhausted:
        pass
    st_far = pd.stats()
    near = [render(cal, R_FLAT, (0.0, 0.0, 3.0))[0]] * 15
    pd2 = PiDetector(ArraySource(near), ArucoMarkerDetector(cal), threaded=False)
    while pd2.poll(0.0) or not pd2.exhausted:
        pass
    st_near = pd2.stats()
    for st in (st_far, st_near):
        assert st['latency_p50_ms'] is not None
        assert st['latency_p99_ms'] >= st['latency_p50_ms']
        assert st['fps'] is not None and st['fps'] > 0
    assert st_near['aruco_roi_hits'] >= 13, st_near['aruco_roi_hits']
    return (f"cadru intreg p50/p99 {st_far['latency_p50_ms']:.0f}/"
            f"{st_far['latency_p99_ms']:.0f} ms; cu ROI "
            f"{st_near['latency_p50_ms']:.0f}/{st_near['latency_p99_ms']:.0f} ms"
            f" (desktop, nu Pi 4)")


def test_pidetector_fir_separat():
    cal = synthetic_calibration()
    frames = [render(cal, R_FLAT, (0.0, 0.0, 6.0))[0]] * 5
    pd = PiDetector(ArraySource(frames), ArucoMarkerDetector(cal),
                    threaded=True).start()
    got = []
    t0 = time.time()
    while time.time() - t0 < 5.0:
        got.extend(pd.poll(time.monotonic()))
        if pd.exhausted and len(got) == 5:
            break
        time.sleep(0.01)
    pd.stop()
    assert len(got) == 5, f"{len(got)} detectii din fir"
    return "5/5 detectii adunate din firul separat"


def test_calibrare_salvare_incarcare_refuz():
    cal = synthetic_calibration()
    tmp = os.path.join(tempfile.mkdtemp(), 'cam.yaml')
    cal.save(tmp)
    back = CameraCalibration.load(tmp)
    assert np.allclose(back.K, cal.K) and np.allclose(back.dist, cal.dist)
    assert (back.width, back.height) == (W, H) and back.rms == cal.rms
    # refuz: geometric (nu e calibrare)
    geo = CameraCalibration.geometric(W, H)
    geo.save(tmp)
    try:
        CameraCalibration.load(tmp)
        raise AssertionError("a acceptat o focala geometrica drept calibrare")
    except ValueError:
        pass
    # refuz: rms prea mare
    bad = CameraCalibration(cal.K, cal.dist, W, H, rms=0.9, n_images=30)
    bad.save(tmp)
    try:
        CameraCalibration.load(tmp)
        raise AssertionError("a acceptat rms 0.9 px")
    except ValueError:
        pass
    # refuz: lipsa
    try:
        CameraCalibration.load(tmp + '.nu')
        raise AssertionError("a acceptat un fisier inexistent")
    except FileNotFoundError:
        pass
    return "roundtrip ok; geometric, rms>0.5 si lipsa -> refuzate"


# --- regresie pentru adaugirile cerute de F2 si F3 ---------------------------
# PROMPT_RUNDA4_AUTONOM cere sa nu se atinga nova/detector_pi.py, cu motivul
# ca o regresie n-ar putea fi verificata. Cele doua adaugiri (meta pe
# CameraCalibration, last_corners pe ArucoMarkerDetector) au fost necesare
# pentru F2 si F3; testele de mai jos exista tocmai ca sa acopere motivul.

def test_regresie_meta_nu_strica_calibrarea():
    """meta e optional: o calibrare fara el se salveaza si se incarca la fel
    ca inainte, iar require_real ramane la fel de strict."""
    cal = synthetic_calibration()
    assert cal.meta == {}, cal.meta
    tmp = os.path.join(tempfile.mkdtemp(), 'cam.yaml')
    cal.save(tmp)
    back = CameraCalibration.load(tmp)
    assert back.meta == {} and np.allclose(back.K, cal.K)
    assert back.rms == cal.rms and back.n_images == cal.n_images
    cal.meta = {'target_type': 'charuco', 'square_mm_measured': 37.0}
    cal.save(tmp)
    b2 = CameraCalibration.load(tmp)
    assert b2.meta['target_type'] == 'charuco'
    assert abs(b2.meta['square_mm_measured'] - 37.0) < 1e-9
    assert np.allclose(b2.K, cal.K) and np.allclose(b2.dist, cal.dist)
    return "fara meta: identic cu inainte; cu meta: dus-intors corect"


def test_regresie_last_corners_nu_schimba_Detection():
    """last_corners e doar observabilitate: Detection si contorii raman
    neschimbati, iar la nedetectie e None."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    assert d.last_corners is None
    t = (0.2, -0.1, 7.0)
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 3.5)
    assert det is not None and d.last_corners is not None
    assert d.last_corners.shape == (4, 2)
    e = float(np.max(np.linalg.norm(d.last_corners - corners, axis=1)))
    assert e < 1.0, f"last_corners la {e:.2f} px de adevar"
    # Ce apara testul asta: colturile NU ajung in contract. Sunt observabile
    # pentru tools/compare_detectors.py, nu ceva ce consuma masina de stari.
    campuri = set(det.__dataclass_fields__)
    for interzis in ('last_corners', 'corners', 'rvec', 'tvec'):
        assert interzis not in campuri, (
            f"{interzis} a ajuns in Detection: contractul cu masina de stari "
            f"creste cu detalii de implementare ale detectorului")
    # iar ce E in contract e acolo deliberat, nu din inertie
    assert campuri == {'t', 'angle_x', 'angle_y', 'distance_m', 'marker_px',
                       'range_m', 'fill'}, campuri
    assert det.t == 3.5
    # dupa un cadru gol, last_corners revine la None
    assert d.detect(np.full((H, W), 110, np.uint8), 4.0) is None
    assert d.last_corners is None, 'last_corners a ramas din cadrul anterior'
    return f"colturi la {e:.3f} px de adevar; Detection neschimbat; None la ratare"


def test_incadrarea_tine_cont_de_rotatia_markerului():
    """§5.49: criteriul masura LATURA, dar ce trebuie sa incapa e CUTIA DE
    INCADRARE a unui patrat rotit - mai mare cu |cos|+|sin|, pana la 41% la
    45 grade.

    Masurat in Gazebo pe 18 rulari: 11 esecuri, toate intre 0.46 si 0.56 m,
    toate cu centrarea perfecta. Randat sintetic, la range 0.42 m detectia
    merge la 10 grade si pica la 20 - iar criteriul pe latura spunea "incape"
    la ambele."""
    import math as _m
    from nova.detection import CameraModel
    cam = CameraModel(focal_px=933.7, vfov_deg=69.52, marker_size_m=0.48)
    h = cam.frame_h_px
    # o latura care incape lejer stand drept
    px = h * 0.90
    assert cam.fits_in_frame(px, 0.0), "drept trebuie sa incapa"
    assert not cam.fits_in_frame(px, 45.0), (
        "rotit la 45 grade, cutia e cu 41% mai mare: NU incape")
    # monotonie: cu cat mai rotit, cu atat mai putin permisiv
    limite = [max(p for p in range(100, 2000)
                  if cam.fits_in_frame(p, yaw)) for yaw in (0, 15, 30, 45)]
    assert limite == sorted(limite, reverse=True), limite
    assert limite[0] / limite[-1] > 1.35, (
        f"raportul dintre limita dreapta si cea la 45 deg e {limite[0]/limite[-1]:.2f}, "
        f"asteptat ~1.41")
    # implicitul pastreaza comportamentul vechi
    assert cam.fits_in_frame(px) == cam.fits_in_frame(px, 0.0)
    return (f"limita {limite[0]} px drept -> {limite[-1]} px la 45 deg "
            f"(x{limite[0]/limite[-1]:.2f})")


def test_detectorul_raporteaza_incadrarea_din_colturi():
    """`Detection.fill` se ia din COLTURI, nu din latura si un unghi presupus.

    Diferenta decide 8.3.3: latura e ce masoara `side_px`, dar ce iese din
    cadru e cutia, mai mare cu pana la 41% la 45 grade (§5.49). Un prag fix
    in pixeli are deci o marja care se prabuseste exact la rotatiile pe care
    nu le controlam - masurat, 980 px e de neatins peste ~18 grade (§5.51),
    iar 800 px a picat la 41 grade (§5.54)."""
    import math as _m
    from nova.detection import fill_from_corners

    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    t = (0.0, 0.0, 2.0)
    frame, corners = render(cal, R_FLAT, t)
    det = d.detect(frame, 0.0)
    assert det is not None, "markerul ar trebui detectat la 2 m"
    assert det.fill is not None, (
        "detectorul nu raporteaza incadrarea, deci masina de stari cade pe "
        "pragul in pixeli chiar si cand ar putea sti mai bine")

    # e chiar cutia colturilor raportate, fata de forma reala a cadrului
    asteptat = fill_from_corners(d.last_corners, W, H)
    assert abs(det.fill - asteptat) < 1e-6, (det.fill, asteptat)

    # si creste cu rotatia, la aceeasi distanta - ce un prag pe latura nu vede
    a = _m.radians(45.0)
    Rz = np.array([[_m.cos(a), -_m.sin(a), 0.0],
                   [_m.sin(a), _m.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    R_rot = Rz @ R_FLAT
    frame_r, _ = render(cal, R_rot, t)
    det_r = d.detect(frame_r, 0.0)
    assert det_r is not None, "markerul rotit la 45 grade ar trebui detectat"
    assert abs(det_r.marker_px - det.marker_px) / det.marker_px < 0.05, (
        f"latura nu trebuie sa se schimbe cu rotatia: "
        f"{det.marker_px:.0f} -> {det_r.marker_px:.0f} px")
    assert det_r.fill > det.fill * 1.30, (
        f"incadrarea trebuie sa creasca cu rotatia: "
        f"{det.fill:.3f} -> {det_r.fill:.3f} (asteptat ~x1.41)")
    return (f"latura {det.marker_px:.0f} px neschimbata; incadrare "
            f"{det.fill:.3f} -> {det_r.fill:.3f} la 45 deg")


def test_rotatia_de_montaj_din_cv2_rotate():
    """Camera montata rotit: adevarul vine din `cv2.rotate`, nu din tabelul
    din `axe_corp`.

    Altfel testul ar verifica tabelul cu el insusi - daca amandoua gresesc
    in acelasi fel, trece. Aici definitia e operatia pe care o face omul:
    "imaginea trebuie rotita 90 grade la STANGA ca nasul sa fie sus" inseamna
    ca `cv2.rotate(bruta, ROTATE_90_COUNTERCLOCKWISE)` da imaginea corecta.

    Cadrul e PATRAT, cu punctul principal exact in centru si fara
    distorsiune tangentiala, ca rotirea pixelilor sa nu schimbe calibrarea -
    altfel am testa si o calibrare gresita, nu doar maparea.

    Cazul negativ conteaza la fel de mult: aceeasi imagine bruta, citita cu
    rotatia 0, trebuie sa dea axele GRESITE. Asa arata o camera montata
    rotit si nedeclarata - vehiculul ar corecta perpendicular pe eroare si
    ar orbita, exact semnatura din §5.1."""
    S = 1296
    c = (S - 1) / 2.0
    f = 933.0
    K = np.array([[f, 0, c], [0, f, c], [0, 0, 1]], dtype=np.float64)
    cal = CameraCalibration(K, [0.0, 0.0, 0.0, 0.0, 0.0], S, S,
                            rms=0.2, n_images=25, source='patrat (test)')

    # markerul: 1.2 m IN FATA nasului si 0.5 m la DREAPTA, la 6 m dedesubt.
    # Asimetric deliberat: cu fata == dreapta, o inversare ar trece.
    FATA, DREAPTA, Z = 1.2, 0.5, 6.0
    # imaginea CORECTA (nas sus): inainte = -y_cam, dreapta = +x_cam
    t_drept = (DREAPTA, -FATA, Z)

    marker = cv2.aruco.generateImageMarker(DICT, 26, 200)
    q = 50
    pansa = np.full((300, 300), 255, np.uint8)
    pansa[q:q + 200, q:q + 200] = marker
    src = np.array([[q, q], [q + 200, q], [q + 200, q + 200], [q, q + 200]],
                   dtype=np.float32)
    dst = project(cal, R_FLAT, t_drept, obj_corners()).astype(np.float32)
    dreapta_img = cv2.warpPerspective(
        pansa, cv2.getPerspectiveTransform(src, dst), (S, S),
        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
        borderValue=110)

    # Ce ar da o camera care trebuie rotita cu R la STANGA ca sa fie dreapta:
    # inversul lui R, adica rotirea imaginii corecte la DREAPTA.
    montaj = {
        0: None,
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }
    la_stanga = {90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_CLOCKWISE}

    rezultate = []
    for rot, cod in montaj.items():
        bruta = dreapta_img if cod is None else cv2.rotate(dreapta_img, cod)
        # definitia: rotita la stanga cu `rot`, bruta redevine imaginea corecta
        if rot:
            assert np.array_equal(cv2.rotate(bruta, la_stanga[rot]),
                                  dreapta_img), f"definitia pentru {rot}"

        det = ArucoMarkerDetector(cal, camera_rotation_deg=rot).detect(
            bruta, 0.0)
        assert det is not None, f"nicio detectie la rotatia {rot}"
        f_m = Z * math.tan(det.angle_x)
        d_m = Z * math.tan(det.angle_y)
        assert abs(f_m - FATA) < 0.03 and abs(d_m - DREAPTA) < 0.03, (
            f"rotatie {rot}: inainte {f_m:.2f} m (asteptat {FATA}), "
            f"dreapta {d_m:.2f} m (asteptat {DREAPTA})")
        rezultate.append(f"{rot}:{f_m:+.2f}/{d_m:+.2f}")

    # NEGATIV: camera rotita 90, dar nedeclarata
    bruta = cv2.rotate(dreapta_img, cv2.ROTATE_90_CLOCKWISE)
    det = ArucoMarkerDetector(cal, camera_rotation_deg=0).detect(bruta, 0.0)
    f_m = Z * math.tan(det.angle_x)
    d_m = Z * math.tan(det.angle_y)
    assert abs(f_m - FATA) > 0.3 or abs(d_m - DREAPTA) > 0.3, (
        "rotatia nedeclarata da totusi axele bune - testul nu masoara nimic")

    # valorile care nu sunt montaje reale se refuza la constructie
    for rau in (45, 30, -15):
        try:
            ArucoMarkerDetector(cal, camera_rotation_deg=rau)
        except ValueError:
            continue
        raise AssertionError(f"rotatia {rau} a fost acceptata")
    return (f"fata/dreapta recuperate (m): {' '.join(rezultate)}; "
            f"nedeclarat: {f_m:+.2f}/{d_m:+.2f}")


def test_expunerea_se_masoara_apoi_se_blocheaza():
    """Zborul din 24.09.2026: 2000 us + gain 8, valori de banc, au ajuns
    afara cu ~5-6 trepte peste - imagine spalata, detectie 30-44%. Regula
    blocarii: luminozitatea masurata de AE se pastreaza, expunerea nu
    depaseste plafonul de blur, restul se muta in gain."""
    from nova.detector_pi import expunere_blocata, AUTOEXP_MAX_US
    assert AUTOEXP_MAX_US == 2000, "plafonul de blur s-a mutat fara calcul"
    # afara: AE alege scurt -> ramane exact ce a masurat, gain mic
    e, g, w = expunere_blocata(400, 1.0)
    assert (e, g, w) == (400.0, 1.0, None), (e, g, w)
    # interior: AE cere 10 ms -> plafonat la 2 ms, gain x5 (lumina egala)
    e, g, w = expunere_blocata(10000, 1.0)
    assert (e, g) == (2000.0, 5.0) and w is None, (e, g, w)
    assert abs(10000 * 1.0 - e * g) < 1e-6, "luminozitatea nu s-a pastrat"
    # bezna: gain-ul cerut depaseste senzorul -> maxim + AVERTISMENT
    e, g, w = expunere_blocata(60000, 4.0)
    assert (e, g) == (2000.0, 16.0) and w is not None, (e, g, w)
    # gain sub minimul senzorului se ridica la minim
    assert expunere_blocata(1500, 0.5)[1] == 1.0
    return "afara scurt; interior plafonat cu gain compensat; bezna avertizata"


def test_verificarea_asteapta_aplicarea_controlului():
    """Pe vehicul, seara (24.09.2026, b1-b3): blocarea a calculat corect
    2000 us, dar verificarea a citit metadatele dupa fix 5 cadre, a vazut
    inca vechiul 32680 al AE-ului, si ESEC-ul fals a trimis pornirea
    automata in monitor - trei boot-uri, 12 handovere refuzate. Se asteapta
    VALOAREA, cu un maxim dupa care nepotrivirea e reala."""
    from nova.detector_pi import asteapta_valoare
    # driverul raporteaza vechea valoare inca 12 cadre, apoi o aplica
    valori = [32680] * 12 + [2000] * 30
    it = iter(valori)
    assert asteapta_valoare(lambda: {'ExposureTime': next(it)},
                            'ExposureTime', 2000) is True
    # NEGATIV: nu se aplica niciodata -> False, nepotrivirea e reala
    assert asteapta_valoare(lambda: {'ExposureTime': 32680},
                            'ExposureTime', 2000, max_incercari=8) is False
    # metadate lipsa nu arunca si nu trec drept aplicare
    assert asteapta_valoare(lambda: {}, 'ExposureTime', 2000,
                            max_incercari=3) is False
    # toleranta de driver: 1990 pentru 2000 cerut e aplicat (ca la banc)
    assert asteapta_valoare(lambda: {'ExposureTime': 1990},
                            'ExposureTime', 2000) is True
    return "12 cadre vechi tolerate; neaplicat -> False; 1990~2000 ok"


def test_ratarile_consecutive_pe_clasa_din_productie():
    """Regula de abort a echipei atarna de contorul de ratari. Verificat pe
    PiDetector, nu pe un dublu: un atribut care lipseste pe clasa care
    zboara face monitorul inert fara nicio eroare (§5.56)."""
    cal = synthetic_calibration()
    frame, _ = render(cal, R_FLAT, (0.0, 0.0, 6.0))
    gol = np.full_like(frame, 110)
    src = ArraySource([frame, gol, gol, gol, frame, gol])
    pid = PiDetector(src, ArucoMarkerDetector(cal), threaded=False)
    urme = []
    for _ in range(6):
        pid.poll(0.1)
        urme.append(pid.miss_streak)
    assert urme == [0, 1, 2, 3, 0, 1], urme
    return f"contor pe cadre: {urme}"


def test_timpii_pe_etape_se_masoara_pe_clasele_din_productie():
    """Step 0 (§5.65): per-stage timings must come from the production
    wiring - PiDetector attaches one StageTimer to the source and the
    detector - and must not change any result. Measured on ArraySource
    frames: stages present, p50 <= p99, and the same Detection as without
    a timer."""
    from nova.detector_pi import StageTimer
    tm = StageTimer(window=10)
    for v in (0.001, 0.003, 0.002):
        tm.add('x', v)
    st = tm.stats()['x']
    assert abs(st[0] - 2.0) < 1e-6 and st[1] <= 3.0 + 1e-6 and st[2] == 3, st
    assert 'x 2/3' in tm.line() and 'etapa' in tm.table()

    cal = synthetic_calibration()
    frame, _ = render(cal, R_FLAT, (0.2, -0.1, 6.0))
    gol = np.full_like(frame, 110)
    src = ArraySource([frame, gol, frame])
    det = ArucoMarkerDetector(cal)
    pid = PiDetector(src, det, threaded=False, ring_frames=5)
    assert det.timer is pid.timer and src.timer is pid.timer, (
        "timer-ul nu ajunge la detector/sursa prin cablajul din productie")
    dets = []
    for i in range(3):
        dets += pid.poll(0.1 * i)
    assert len(dets) == 2
    s = pid.timer.stats()
    for st in ('geometrie', 'ring', 'total'):
        assert st in s, f"etapa {st} lipseste din {list(s)}"
    assert 'detect_redus' in s or 'detect_plin' in s
    assert s['total'][0] <= s['total'][1]
    # timing changes nothing: same corners as a detector without timer
    d2 = ArucoMarkerDetector(cal)
    ref = d2.detect(frame, 0.0)
    assert abs(ref.distance_m - dets[0].distance_m) < 1e-9
    return f"etape: {', '.join(s)}; rezultat identic cu/fara timer"


def test_bench_offline_ruleaza_cele_trei_pipeline_uri():
    """tools/bench_detect.py on a directory of rendered frames: the three
    pipelines (plain cv2.aruco, ArucoMarkerDetector, PiDetector) all see
    the same frames and report a detection rate that matches what was
    rendered - 4 frames with marker out of 6."""
    import json
    import tempfile
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bench_detect as bd
    cal = synthetic_calibration()
    d = tempfile.mkdtemp()
    frame, _ = render(cal, R_FLAT, (0.1, 0.1, 5.0))
    gol = np.full_like(frame, 110)
    for i, g in enumerate([frame, gol, frame, frame, gol, frame]):
        cv2.imwrite(os.path.join(d, f"f{i:05d}.png"), g)
    json.dump({'frames': [{'LensPosition': 1.63}], 'fps_measured': 30.0},
              open(os.path.join(d, 'meta.json'), 'w'))
    cfg = {'marker_id': 26, 'marker_size_m': MARKER_M, 'roi_below_m': 15.0,
           'roi_size_px': (640, 480), 'camera_rotation_deg': 0,
           'search_downscale': 2}
    r = bd.run_bench(d, cal, cfg, variants=True)
    assert r['n'] == 6
    for k in ('plain', 'aruco'):
        assert abs(r[k]['rate'] - 4 / 6) < 1e-9, (k, r[k])
        assert r[k]['p99'] >= r[k]['p50'] > 0
    assert abs(r['pi']['rate'] - 4 / 6) < 1e-9, r['pi']
    assert 'achizitie' in r['pi']['stats'] and 'geometrie' in r['pi']['stats']
    assert len(r['variants']) == 4
    assert all(abs(v['rate'] - 4 / 6) < 1e-9 for v in r['variants'].values())
    return "plain/aruco/pi: 4/6 fiecare; 4 variante; etape cu achizitie"


def test_luminozitatea_ajunge_in_statistici():
    """In zbor nu exista NICIO cifra despre expunere in log - cauza
    probabila (imagine supraexpusa) a ramas o ipoteza. `lum` o face
    masurabila: media cadrului, in fiecare linie de stare."""
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    assert d.stats()['lum'] is None, "inainte de primul cadru: None, nu 0"
    frame, _ = render(cal, R_FLAT, (0.0, 0.0, 8.0))
    d.detect(frame, 1.0)
    lum = d.stats()['lum']
    assert lum is not None and abs(lum - frame[::16, ::16].mean()) < 2, lum
    # un cadru ars (alb saturat) trebuie sa se VADA in cifra
    d.detect(np.full_like(frame, 254), 2.0)
    assert d.stats()['lum'] >= 250, d.stats()['lum']
    src = ArraySource([frame])
    pid = PiDetector(src, ArucoMarkerDetector(cal), threaded=False)
    pid.poll(0.1)
    assert 'lum' in pid.status_line(), pid.status_line()
    return f"lum {lum} pe cadrul sintetic; 254 pe cadrul ars; in status_line"


TESTS = [
    ('expunerea se masoara apoi se blocheaza',
     test_expunerea_se_masoara_apoi_se_blocheaza),
    ('luminozitatea ajunge in statistici',
     test_luminozitatea_ajunge_in_statistici),
    ('verificarea asteapta aplicarea controlului',
     test_verificarea_asteapta_aplicarea_controlului),
    ('ratarile consecutive pe clasa din productie',
     test_ratarile_consecutive_pe_clasa_din_productie),
    ('timpii pe etape, pe clasele din productie',
     test_timpii_pe_etape_se_masoara_pe_clasele_din_productie),
    ('bench offline: cele trei pipeline-uri',
     test_bench_offline_ruleaza_cele_trei_pipeline_uri),
    ('incadrarea tine cont de rotatia markerului',
     test_incadrarea_tine_cont_de_rotatia_markerului),
    ('direct dedesubt, 8 m', test_direct_dedesubt_8m),
    ('offset lateral, conventia de montaj', test_offset_lateral_conventia_de_montaj),
    ('12 m, ~37 px', test_departe_12m_37px),
    ('0.45 m, inca incape', test_aproape_045m_incape),
    ('ROI: gating pe prag + implicitele de zbor',
     test_roi_gating_pe_prag_si_implicitele_de_zbor),
    ('cautarea redusa gaseste si rafineaza',
     test_cautarea_redusa_gaseste_si_rafineaza),
    ('NEGATIV: nu incape in cadru (5.2)', test_NEGATIV_nu_incape_in_cadru),
    ('NEGATIV: cadru gol / zgomot', test_NEGATIV_cadru_gol),
    ('NEGATIV: alt ID ignorat', test_NEGATIV_alt_id_ignorat),
    ('plan inclinat: range pe axa optica', test_plan_inclinat_range_pe_axa_optica),
    ('ROI sub 5 m', test_roi_sub_5m),
    ('ratare ROI -> cadru intreg', test_roi_miss_cade_pe_cadrul_intreg),
    ('PiDetector: timestamp captura pastrat', test_pidetector_timestamps_si_statistici),
    ('PiDetector: latenta reala (desktop)', test_pidetector_latenta_reala_pe_desktop),
    ('PiDetector: fir separat', test_pidetector_fir_separat),
    ('calibrare: salvare/incarcare/refuz', test_calibrare_salvare_incarcare_refuz),
    ('REGRESIE: meta nu strica calibrarea', test_regresie_meta_nu_strica_calibrarea),
    ('REGRESIE: last_corners nu schimba Detection',
     test_regresie_last_corners_nu_schimba_Detection),
    ('detectorul raporteaza incadrarea din colturi',
     test_detectorul_raporteaza_incadrarea_din_colturi),
    ('rotatia de montaj, cu adevarul din cv2.rotate',
     test_rotatia_de_montaj_din_cv2_rotate),
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
