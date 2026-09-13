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


def test_fara_roi_peste_5m():
    cal = synthetic_calibration()
    d = ArucoMarkerDetector(cal)
    f, _ = render(cal, R_FLAT, (0.0, 0.0, 7.0))
    d.detect(f, 0.0)
    d.detect(f, 0.1)
    assert d.n_roi == 0, "ROI folosit peste pragul de 5 m"
    return "la 7 m se cauta in tot cadrul"


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


TESTS = [
    ('direct dedesubt, 8 m', test_direct_dedesubt_8m),
    ('offset lateral, conventia de montaj', test_offset_lateral_conventia_de_montaj),
    ('12 m, ~37 px', test_departe_12m_37px),
    ('0.45 m, inca incape', test_aproape_045m_incape),
    ('NEGATIV: nu incape in cadru (5.2)', test_NEGATIV_nu_incape_in_cadru),
    ('NEGATIV: cadru gol / zgomot', test_NEGATIV_cadru_gol),
    ('NEGATIV: alt ID ignorat', test_NEGATIV_alt_id_ignorat),
    ('plan inclinat: range pe axa optica', test_plan_inclinat_range_pe_axa_optica),
    ('ROI sub 5 m', test_roi_sub_5m),
    ('ratare ROI -> cadru intreg', test_roi_miss_cade_pe_cadrul_intreg),
    ('fara ROI peste 5 m', test_fara_roi_peste_5m),
    ('PiDetector: timestamp captura pastrat', test_pidetector_timestamps_si_statistici),
    ('PiDetector: latenta reala (desktop)', test_pidetector_latenta_reala_pe_desktop),
    ('PiDetector: fir separat', test_pidetector_fir_separat),
    ('calibrare: salvare/incarcare/refuz', test_calibrare_salvare_incarcare_refuz),
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
