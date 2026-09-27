#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Metadatele camerei si timpul capturii (27.09.2026, punctul 3).

    python3 tools/test_camera_metadata.py

Ce conteaza: t_capture e SensorTimestamp convertit in ceasul buclei, nu
momentul in care read() s-a intors (cu camera_fresh_capture false cadrul
poate fi deja la coada); jurnalul detectorului are, per cadru, ExposureTime,
AnalogueGain, LensPosition, SensorTimestamp; fereastra de 5 s spune cat din
planul de gri e saturat (255) si cat e aproape negru (< 10).
"""

import contextlib
import csv
import io
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova.detector_pi import (ArraySource, ArucoMarkerDetector,  # noqa: E402
                              DARK_LEVEL, FRAME_LOG_FIELDS, PiDetector,
                              camera_settings)
from test_camera_geometry import FakePicamera2, fake_camera, quiet  # noqa: E402
import test_detector_pi as tdp                                  # noqa: E402


def test_t_capture_e_SensorTimestamp_nu_momentul_citirii():
    """Cadrul capturat cu 40 ms inainte de read() (la coada): t_capture e cu
    ~40 ms in urma ceasului, nu 'acum'. Fara SensorTimestamp, read() cade pe
    momentul intoarcerii - iar detectorul numara si spune asta."""
    from nova.detector_pi import PiCameraSource
    with fake_camera() as cam, quiet():
        cam.ts_lag_s = 0.040
        src = PiCameraSource(settings=camera_settings({}, 'crop1280'))
        lags = []
        for _ in range(5):
            _gray, t = src.read()
            lags.append(time.monotonic() - t)
        src.close()
    assert all(0.035 < x < 0.2 for x in lags), lags
    with fake_camera() as cam, quiet():
        src = PiCameraSource(settings=camera_settings({}, 'crop1280'))
        cam.no_timestamp = True
        pd = PiDetector(src, ArucoMarkerDetector(tdp.synthetic_calibration()),
                        threaded=False)
        pd.window_log = False
        for _ in range(3):
            pd.poll(0.0)
        src.close()
    assert pd.n_no_timestamp == 3, pd.n_no_timestamp
    return f"varsta cadrului {1000 * min(lags):.0f}-{1000 * max(lags):.0f} ms (lag 40 ms); fara timestamp -> numarat"


def test_jurnalul_per_cadru_are_metadatele_camerei():
    """Detectorul real, camera stub: fiecare cadru o linie in CSV cu
    metadatele lui si ce a facut detectia."""
    from nova.detector_pi import PiCameraSource
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, 'sub', 'cadre.csv')
    with fake_camera(), quiet():
        src = PiCameraSource(settings=camera_settings({}, 'crop1280'))
        pd = PiDetector(src, ArucoMarkerDetector(tdp.synthetic_calibration()),
                        threaded=False, frame_log=path)
        pd.window_log = False
        for _ in range(4):
            pd.poll(0.0)
        pd.stop()
    rows = list(csv.DictReader(open(path)))
    assert tuple(rows[0].keys()) == FRAME_LOG_FIELDS, rows[0].keys()
    assert [int(r['seq']) for r in rows] == [1, 2, 3, 4]
    for r in rows:
        for k in ('ExposureTime', 'AnalogueGain', 'LensPosition', 'SensorTimestamp',
                  't_capture', 'proc_ms'):
            assert r[k] != '', (k, r)
        assert r['detected'] == '0' and r['marker_px'] == ''
    assert abs(float(rows[0]['LensPosition']) - 1.63) < 1e-6
    assert float(rows[0]['ExposureTime']) <= 2000        # auto_lock, plafonat
    return f"{len(rows)} linii, campuri: {', '.join(FRAME_LOG_FIELDS)}"


def test_fereastra_spune_saturatia_si_expunerea():
    """Cadre cu 25% alb saturat si 10% negru: fereastra le raporteaza (pe
    esantion, deci aproximativ); expunerea mediana si maxima, gain-ul,
    focusul vin din metadate; linia de log le contine."""
    h, w = 720, 1280
    frame = np.full((h, w), 120, np.uint8)
    frame[:, :w // 4] = 255
    frame[:h // 10, w // 4:] = 3            # 10% din rest -> 7.5% din cadru
    frames = [frame] * 6

    class Sursa(ArraySource):
        def __init__(self):
            super().__init__(frames)
            self.n = 0

        def read(self):
            item = super().read()
            if item is not None:
                self.n += 1
                self.last_metadata = {'ExposureTime': 1000 + 100 * self.n,
                                      'AnalogueGain': 2.0, 'LensPosition': 1.63,
                                      'SensorTimestamp': 123}
            return item

    t = [0.0]
    pd = PiDetector(Sursa(), ArucoMarkerDetector(tdp.synthetic_calibration()),
                    threaded=False)
    pd.window_log_s = 1.0
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        for i in range(6):
            pd._window_log(t[0])     # ancora ferestrei inainte de cadre
            pd.poll(0.0)
            t[0] += 0.3
        pd._window_log(t[0])
    lw = pd.last_window
    assert lw is not None and abs(lw['sat_pct'] - 25.0) < 1.0, lw
    assert abs(lw['dark_pct'] - 7.5) < 1.0, lw
    assert lw['exposure_max_us'] >= lw['exposure_us'] > 1000 and lw['gain'] == 2.0
    assert lw['lens'] == 1.63 and lw['no_timestamp'] == 0
    line = [x for x in out.getvalue().splitlines() if x.startswith('[detectie')][-1]
    assert f"<{DARK_LEVEL}" in line and 'sat ' in line and 'exp ' in line and 'lens 1.63' in line, line
    return f"sat {lw['sat_pct']:.1f}% negru {lw['dark_pct']:.1f}%; {line.split('|', 5)[-1].strip()}"


def test_frame_seq_numara_fiecare_cadru():
    """Contorul pe care il urmareste fereastra OSD: fiecare cadru citit,
    si cu detectia inactiva."""
    frame, _ = tdp.render(tdp.synthetic_calibration(), tdp.R_FLAT, (0.0, 0.0, 6.0))
    pd = PiDetector(ArraySource([frame] * 5),
                    ArucoMarkerDetector(tdp.synthetic_calibration()), threaded=False)
    pd.window_log = False
    pd.poll(0.0)
    pd.poll(0.0)
    pd.active.clear()
    pd.poll(0.0)
    assert pd.frame_seq == 3, pd.frame_seq
    return "3 cadre -> frame_seq 3 (unul cu detectia inactiva)"


def test_calea_jurnalului_in_aplicatie_si_in_proba():
    import nova_pi
    assert nova_pi.frame_log_path('none') is None
    assert nova_pi.frame_log_path('/x/y.csv') == '/x/y.csv'
    vechi = os.environ.get('NOVA_LOG_DIR')
    os.environ['NOVA_LOG_DIR'] = '/tmp/loguri'
    try:
        p = nova_pi.frame_log_path(None, now=0)
    finally:
        if vechi is None:
            os.environ.pop('NOVA_LOG_DIR')
        else:
            os.environ['NOVA_LOG_DIR'] = vechi
    assert p.startswith('/tmp/loguri/cadre-') and p.endswith('.csv'), p
    src = open(os.path.join(HERE, 'nova_pi.py')).read()
    assert 'frame_log=frame_log_path(a.frame_log)' in src
    dt = open(os.path.join(REPO, 'pi', 'descent_test.sh')).read()
    assert 'ARGS+=(--frame-log "$LOG_DIR/cadre-$STAMP.csv")' in dt
    return "none / cale / implicit in NOVA_LOG_DIR; proba: acelasi stamp ca logul"


def test_diagnosticul_nu_pierde_detectii_si_nu_blocheaza_pornirea():
    """Review 27.09: jurnalul si statisticile ruleaza DUPA publicarea
    detectiei; o eroare in ele opreste diagnosticul, nu detectia; un disc
    plin inchide jurnalul, spus o data; o cale nescriibila nu opreste
    pornirea; doua porniri cu acelasi nume nu se suprascriu."""
    cal = tdp.synthetic_calibration()
    frame, _ = tdp.render(cal, tdp.R_FLAT, (0.0, 0.0, 6.0))

    class Plin(io.StringIO):
        def write(self, s):
            if 'seq' in s:
                return super().write(s)
            raise OSError(28, 'No space left on device')

    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        pd = PiDetector(ArraySource([frame] * 3), ArucoMarkerDetector(cal),
                        threaded=False, frame_log=Plin())
        pd.window_log = False
        got = pd.poll(0.0) + pd.poll(0.0)
    assert len(got) == 2 and pd._frame_log is None, (len(got), pd._frame_log)
    assert out.getvalue().count('jurnalul per cadru oprit') == 1, out.getvalue()
    # o eroare in statistici: detectia se publica, diagnosticul se opreste
    pd2 = PiDetector(ArraySource([frame] * 3), ArucoMarkerDetector(cal), threaded=False)
    pd2.window_log = False

    def rau(*a):
        raise ValueError('metadate ciudate')
    pd2._frame_stats = rau
    with contextlib.redirect_stdout(io.StringIO()) as o2:
        got2 = pd2.poll(0.0) + pd2.poll(0.0)
    assert len(got2) == 2 and pd2._diag_off and pd2.fail_streak == 0
    assert 'diagnosticul a picat' in o2.getvalue()
    # cale nescriibila: porneste fara jurnal
    with contextlib.redirect_stdout(io.StringIO()) as o3:
        pd3 = PiDetector(ArraySource([frame]), ArucoMarkerDetector(cal),
                         threaded=False, frame_log='/proc/nu/se/poate/cadre.csv')
    assert pd3._frame_log is None and 'nu se poate deschide' in o3.getvalue()
    # acelasi nume de doua ori: al doilea primeste sufix, primul ramane
    tmp = tempfile.mkdtemp()
    p = os.path.join(tmp, 'cadre-x.csv')
    a = PiDetector(ArraySource([frame]), ArucoMarkerDetector(cal), threaded=False, frame_log=p)
    b = PiDetector(ArraySource([frame]), ArucoMarkerDetector(cal), threaded=False, frame_log=p)
    assert a.frame_log_path == p and b.frame_log_path == os.path.join(tmp, 'cadre-x-1.csv')
    a.stop()
    b.stop()
    return "disc plin -> jurnal oprit, detectii publicate; statistica rea -> oprita; cale rea -> pornire; fara suprascriere"


def test_last_view_are_cadrul_si_conturul_lui():
    """Instantaneul pentru OSD: (seq, cadru, colturi) publicat la sfarsitul
    cadrului, colturile ale ACELUI cadru; frame_seq se misca odata cu el."""
    cal = tdp.synthetic_calibration()
    frame, corners = tdp.render(cal, tdp.R_FLAT, (0.3, -0.2, 6.0))
    gol = np.full_like(frame, 110)
    pd = PiDetector(ArraySource([frame, gol]), ArucoMarkerDetector(cal),
                    threaded=False, keep_last_frame=True)
    pd.window_log = False
    pd.poll(0.0)
    seq, img, col = pd.last_view
    assert seq == pd.frame_seq == 1 and img is frame and col is not None
    assert float(np.max(np.abs(col.reshape(-1, 2) - corners))) < 1.0
    pd.poll(0.0)
    seq, img, col = pd.last_view
    assert seq == 2 and img is gol and col is None, "colturile cadrului vechi pe cel nou"
    return "cadru cu marker -> colturile lui; cadru gol -> fara contur"


def test_camera_se_inchide_la_orice_eroare_dupa_deschidere():
    """build_pi_detector: o cheie lipsa din config (dupa ce camera s-a
    deschis) elibereaza senzorul. close(): stop() care pica nu sare close()."""
    from nova import config as nova_config
    from nova.detector_pi import PiCameraSource, build_pi_detector
    cfg = dict(nova_config.load())
    cfg.pop('roi_size_px')
    with fake_camera() as cam, quiet():
        try:
            build_pi_detector(cfg, verbose=False)
            assert False, "config stricat acceptat"
        except KeyError:
            pass
        assert cam.instances[-1].closed, "camera ramasa deschisa"
    with fake_camera() as cam, quiet():
        src = PiCameraSource(settings=camera_settings({}, 'crop1280'))
        pc = cam.instances[-1]

        def stop_rau():
            raise RuntimeError('camera in eroare')
        pc.stop = stop_rau
        src.close()
        assert pc.closed, "close() sarit dupa un stop() picat"
    return "KeyError dupa deschidere -> camera inchisa; stop() picat -> close() tot se face"


def test_distanta_solvepnp_nu_decide_nimic_pe_extnav():
    """Retus 27.09: cu roi_below_m null (nova.json), ROI-ul nu mai depinde
    de distanta solvePnP (care scaleaza cu marker_size_m): acelasi cadru,
    acelasi marker, declarat 0.48 sau 0.336 -> exact aceleasi decizii de
    ROI si aceleasi unghiuri; doar distanta difera, de 1.43x. roi_below_m
    numeric pastreaza comportamentul vechi (0 = ROI oprit)."""
    from nova import config as nova_config
    cal = tdp.synthetic_calibration()
    assert nova_config.load()['roi_below_m'] is None
    frames = [tdp.render(cal, tdp.R_FLAT, (0.3 * k, -0.2, 12.0))[0] for k in range(5)]

    def ruleaza(size_m, roi):
        d = ArucoMarkerDetector(cal, marker_size_m=size_m, roi_below_m=roi)
        rez = []
        for i, f in enumerate(frames):
            roi_box = d._roi(f.shape)
            det = d.detect(f, float(i))
            rez.append((roi_box, None if det is None else (det.angle_x, det.angle_y,
                                                            det.distance_m)))
        return rez

    a, b = ruleaza(0.48, None), ruleaza(0.336, None)
    assert [x[0] for x in a] == [x[0] for x in b], "ROI diferit dupa marimea declarata"
    assert a[1][0] is not None, "fara ROI la 12 m cu roi_below_m null"
    for (_, da), (_, db) in zip(a, b):
        assert abs(da[0] - db[0]) < 1e-9 and abs(da[1] - db[1]) < 1e-9
        assert abs(da[2] / db[2] - 0.48 / 0.336) < 1e-6
    # pragul numeric: comportamentul vechi (la 12 m, sub 15 -> ROI; 0 -> niciodata)
    assert ruleaza(0.48, 15.0)[1][0] is not None
    assert all(x[0] is None for x in ruleaza(0.48, 0.0))
    return "ROI si unghiuri identice la 0.48 / 0.336 m; doar distanta x1.43"


TESTS = [
    ('distanta solvePnP nu decide nimic pe ExtNav',
     test_distanta_solvepnp_nu_decide_nimic_pe_extnav),
    ('diagnosticul nu pierde detectii si nu blocheaza pornirea',
     test_diagnosticul_nu_pierde_detectii_si_nu_blocheaza_pornirea),
    ('last_view are cadrul si conturul lui', test_last_view_are_cadrul_si_conturul_lui),
    ('camera se inchide la orice eroare dupa deschidere',
     test_camera_se_inchide_la_orice_eroare_dupa_deschidere),
    ('t_capture = SensorTimestamp, nu momentul citirii',
     test_t_capture_e_SensorTimestamp_nu_momentul_citirii),
    ('jurnalul per cadru are metadatele camerei',
     test_jurnalul_per_cadru_are_metadatele_camerei),
    ('fereastra spune saturatia si expunerea',
     test_fereastra_spune_saturatia_si_expunerea),
    ('frame_seq numara fiecare cadru', test_frame_seq_numara_fiecare_cadru),
    ('calea jurnalului in aplicatie si in proba',
     test_calea_jurnalului_in_aplicatie_si_in_proba),
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
            import traceback
            traceback.print_exc()
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
