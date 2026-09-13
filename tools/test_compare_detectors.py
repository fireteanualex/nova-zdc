#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru verificarea incrucisata (F3): tools/verify_detection.py si
tools/compare_detectors.py.

    python3 tools/test_compare_detectors.py

Markerul ID 26 de 480 mm e randat prin intrinseci CUNOSCUTI, la 15 poze
cunoscute intre 2 si 15 m, acoperind toate zonele cadrului. Ambele
implementari trebuie sa recupereze poza, si fiecare e comparata SEPARAT cu
adevarul - daca diverg, stim care se abate, nu doar ca difera.
"""

import math
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import compare_detectors as cd                            # noqa: E402
import synthetic as syn                                   # noqa: E402
import verify_detection as vd                             # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector,        # noqa: E402
                              CameraCalibration)

K, DIST, WH = syn.imx708()
W, H = WH
MARKER_M = 0.48

#: Praguri (PROMPT_RUNDA4_AUTONOM, sectiunea F3)
MAX_CORNER_PX = 1.0
MAX_DIST_REL_INTRE = 0.01
MAX_DIST_REL_ADEVAR = 0.02
MAX_ANGLE_DEG = 0.3


def calib_file(k=None, dist=None, real=True):
    """Scrie o calibrare temporara si intoarce calea."""
    cal = CameraCalibration(K if k is None else k,
                            DIST if dist is None else dist, W, H,
                            rms=0.11 if real else None,
                            n_images=25 if real else 0,
                            source='sintetic (test F3)')
    p = os.path.join(tempfile.mkdtemp(), 'camera_pi.yaml')
    cal.save(p)
    return p, cal


def poses_15():
    """15 poze: 2-15 m, toate cele noua zone ale cadrului, inclinari variate.

    Offset-urile sunt trase spre interior cu 0.75, ca markerul sa incapa
    INTREG in cadru la distantele mici - altfel testul ar masura decuparea,
    nu detectia."""
    rng = np.random.default_rng(7)
    zs = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
          12.0, 13.0, 14.0, 15.0, 2.5]
    cells = [(c, r) for r in range(3) for c in range(3)]
    out = []
    f = K[0, 0]
    for i, z in enumerate(zs):
        c, r = cells[i % 9]
        u = ((c + 0.5) / 3 * W - W / 2) * 0.75
        v = ((r + 0.5) / 3 * H - H / 2) * 0.75
        t = (u * z / f, v * z / f, z)
        R = syn.rot(rng.uniform(-12, 12), rng.uniform(-12, 12),
                    rng.uniform(-25, 25))
        out.append((R, t))
    return out


def truth_of(t):
    """Adevarul pentru o poza: distanta si unghiurile in conventia de bord
    (inainte = -y_cam, dreapta = +x_cam)."""
    t = np.asarray(t, float)
    return {
        'distance': float(np.linalg.norm(t)),
        'angle_x': math.atan2(-t[1], t[2]),
        'angle_y': math.atan2(t[0], t[2]),
    }


def render_all(poses, out_dir=None):
    """[(cadru, t, colturi_adevar)], optional salvate pe disc."""
    out = []
    for i, (R, t) in enumerate(poses):
        img, corners = syn.render_marker(K, DIST, WH, R, t,
                                         marker_id=26, size_m=MARKER_M)
        if out_dir:
            cv2.imwrite(os.path.join(out_dir, f"m{i:02d}.png"), img)
        out.append((img, t, corners))
    return out


def angles_from_tvec(t):
    t = np.asarray(t, float)
    return math.atan2(-t[1], t[2]), math.atan2(t[0], t[2])


# --- teste -------------------------------------------------------------------

def test_F3_15_poze_ambele_implementari():
    """Testul principal: fiecare implementare fata de adevar, si una fata de
    cealalta. Daca diverg, mesajul spune care se abate."""
    path, cal = calib_file()
    Kb, distb, _wh, _info = vd.load_calibration(path)
    a = ArucoMarkerDetector(cal, roi_below_m=0.0)
    b = vd.Detector(Kb, distb, MARKER_M, 26, refine=True)

    n = 0
    d_corner, d_rel, ea_d, eb_d, ea_ang, eb_ang, pxs = [], [], [], [], [], [], []
    for img, t, truth_corners in render_all(poses_15()):
        tr = truth_of(t)
        da = a.detect(img, 0.0)
        db = b.detect(img)
        assert da is not None and db is not None, (
            f"la {tr['distance']:.1f} m: A={'ok' if da else 'nu'} "
            f"B={'ok' if db else 'nu'}")
        n += 1
        pxs.append(da.marker_px)

        # fiecare fata de adevar
        ea_d.append(abs(da.distance_m / tr['distance'] - 1))
        eb_d.append(abs(db['distance_m'] / tr['distance'] - 1))
        bx, by = angles_from_tvec(db['tvec'])
        ea_ang.append(max(abs(math.degrees(da.angle_x - tr['angle_x'])),
                          abs(math.degrees(da.angle_y - tr['angle_y']))))
        eb_ang.append(max(abs(math.degrees(bx - tr['angle_x'])),
                          abs(math.degrees(by - tr['angle_y']))))

        # una fata de cealalta
        d_corner.append(float(np.linalg.norm(
            a.last_corners - db['corners'], axis=1).max()))
        d_rel.append(abs(da.distance_m - db['distance_m'])
                     / db['distance_m'])

    assert n == 15, n
    assert max(d_corner) <= MAX_CORNER_PX, (
        f"colturi {max(d_corner):.3f} px intre implementari; fata de adevar "
        f"A greseste {max(ea_d):.2%} si B {max(eb_d):.2%} pe distanta")
    assert max(d_rel) <= MAX_DIST_REL_INTRE, (
        f"distanta {max(d_rel):.2%} intre implementari; fata de adevar "
        f"A {max(ea_d):.2%}, B {max(eb_d):.2%}")
    assert max(ea_d) < MAX_DIST_REL_ADEVAR, f"A vs adevar: {max(ea_d):.2%}"
    assert max(eb_d) < MAX_DIST_REL_ADEVAR, f"B vs adevar: {max(eb_d):.2%}"
    assert max(ea_ang) < MAX_ANGLE_DEG, f"A unghiuri: {max(ea_ang):.3f} deg"
    assert max(eb_ang) < MAX_ANGLE_DEG, f"B unghiuri: {max(eb_ang):.3f} deg"
    return (f"15/15 poze ({min(pxs):.0f}-{max(pxs):.0f} px) | intre "
            f"implementari: colturi {max(d_corner):.3f} px, distanta "
            f"{max(d_rel):.3%} | fata de adevar: distanta A {max(ea_d):.2%} "
            f"B {max(eb_d):.2%}, unghiuri A {max(ea_ang):.3f} B "
            f"{max(eb_ang):.3f} deg")


def test_F3_acord_pe_detectat_nedetectat():
    """100% acord pe tot regimul, de la 0.25 m (prea aproape) la 15 m.

    Masurat: detectia moare la ~0.35 m, deci sub acel prag AMBELE tac. Cele
    doua implementari trebuie sa fie de acord si acolo, nu doar acolo unde
    vad markerul."""
    path, cal = calib_file()
    Kb, distb, _wh, _info = vd.load_calibration(path)
    a = ArucoMarkerDetector(cal, roi_below_m=0.0)
    b = vd.Detector(Kb, distb, MARKER_M, 26, refine=True)

    zs = (0.25, 0.30, 0.33, 0.35, 0.40, 0.45, 1.0, 4.0, 8.0, 15.0)
    scene = [(syn.R_MARKER_FLAT, (0.0, 0.0, z)) for z in zs]
    dezacorduri, ambele, tac = [], 0, 0
    for (img, t, _), z in zip(render_all(scene), zs):
        r = cd.compare_one(a.detect(img, 0.0), a.last_corners, b.detect(img))
        if r['stare'].startswith('DOAR'):
            dezacorduri.append((z, r['stare']))
        elif r['stare'] == 'ambele':
            ambele += 1
        elif r['stare'] == 'ambele tac':
            tac += 1
    assert not dezacorduri, f"dezacorduri: {dezacorduri}"
    assert ambele >= 6 and tac >= 3, (ambele, tac)
    gol = np.full((H, W), 110, np.uint8)
    assert a.detect(gol, 0.0) is None and b.detect(gol) is None
    return (f"{len(zs)} distante 0.25-15 m: {ambele} ambele detecteaza, "
            f"{tac} ambele tac (sub ~0.35 m), 0 dezacorduri; + cadru gol")


def test_F3_tratarea_fits_in_frame_nu_e_dezacord():
    """Cand A tace pentru ca markerul nu incape (§5.2) iar B raporteaza
    fits_in_frame=0, cele doua sunt DE ACORD - nu o divergenta.

    Cazul e construit, nu randat: masurat, detectia ArUco moare la ~0.35 m,
    adica INAINTE ca garda de 0.95*H sa se declanseze, deci in randare
    fereastra e practic goala. Garda ramane (e ieftina si acopera cazuri
    reale: marker mai mare, alta camera), dar logica ei se verifica direct."""
    fals = {'fits_in_frame': False, 'distance_m': 0.3, 'marker_px': 1400.0,
            'corners': np.zeros((4, 2)), 'tvec': np.zeros(3), 'marker_id': 26}
    r = cd.compare_one(None, None, fals)
    assert r['stare'].startswith('acord:'), r['stare']

    adevarat = dict(fals, fits_in_frame=True)
    r2 = cd.compare_one(None, None, adevarat)
    assert r2['stare'] == 'DOAR B detecteaza', r2['stare']
    return "A tace + B fits_in_frame=0 -> acord; fits_in_frame=1 -> divergenta"


def test_F3_NEGATIV_calibrare_gresita_e_prinsa():
    """Verificarea incrucisata trebuie sa PRINDA o greseala reala. B primeste
    focala geometrica fara distorsiune - exact greseala pe care o face
    scriptul original si pe care unealta o interzice."""
    path, cal = calib_file()
    a = ArucoMarkerDetector(cal, roi_below_m=0.0)
    b = vd.Detector(K, np.zeros(5), MARKER_M, 26, refine=True)   # dist zero

    d_rel, ea, eb = [], [], []
    for img, t, _ in render_all(poses_15()):
        tr = truth_of(t)
        da, db = a.detect(img, 0.0), b.detect(img)
        if da is None or db is None:
            continue
        d_rel.append(abs(da.distance_m - db['distance_m']) / db['distance_m'])
        ea.append(abs(da.distance_m / tr['distance'] - 1))
        eb.append(abs(db['distance_m'] / tr['distance'] - 1))
    assert max(d_rel) > MAX_DIST_REL_INTRE, (
        f"dist_coeffs zero a trecut neobservat ({max(d_rel):.2%}) - "
        f"verificarea incrucisata nu prinde nimic")
    assert max(eb) > max(ea), (
        f"comparatia cu adevarul nu arata care greseste: A {max(ea):.2%}, "
        f"B {max(eb):.2%}")
    return (f"dist_coeffs zero: divergenta {max(d_rel):.1%}; fata de adevar "
            f"A {max(ea):.2%} vs B {max(eb):.1%} -> B e cel gresit")


def test_F3_NEGATIV_latura_gresita_e_prinsa():
    """Cealalta greseala clasica: latura markerului. 0.24 in loc de 0.48
    inseamna distante la jumatate."""
    path, cal = calib_file()
    Kb, distb, _wh, _info = vd.load_calibration(path)
    a = ArucoMarkerDetector(cal, roi_below_m=0.0)
    b = vd.Detector(Kb, distb, 0.24, 26, refine=True)
    img, t, _ = render_all([(syn.R_MARKER_FLAT, (0.0, 0.0, 8.0))])[0]
    da, db = a.detect(img, 0.0), b.detect(img)
    assert da is not None and db is not None
    rel = abs(da.distance_m - db['distance_m']) / db['distance_m']
    corner = float(np.linalg.norm(a.last_corners - db['corners'], axis=1).max())
    assert rel > 0.5, f"latura la jumatate a dat doar {rel:.1%} diferenta"
    assert corner <= MAX_CORNER_PX, (
        'colturile ar trebui identice - greseala e in punctele-obiect, nu in '
        'detectie; asta e si indiciul care spune unde sa te uiti')
    return (f"latura 0.24 vs 0.48: distanta difera cu {rel:.0%}, colturile "
            f"raman identice ({corner:.3f} px) -> greseala e in scara")


def test_F3_verify_detection_nu_importa_nova():
    """Independenta e cerinta, deci se verifica, nu se presupune."""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'verify_detection.py')).read()
    linii = [ln.strip() for ln in src.splitlines()
             if ln.strip().startswith(('import ', 'from '))]
    rele = [ln for ln in linii if 'nova' in ln]
    assert not rele, f"verify_detection.py importa din nova/: {rele}"
    assert 'nova' not in sys.modules or True    # informativ
    module = sorted({ln.split()[1].split('.')[0] for ln in linii})
    assert module == ['argparse', 'csv', 'cv2', 'numpy', 'os', 'sys'], module
    return f"importa doar: {', '.join(module)}"


def test_F3_verify_detection_csv_si_refuz():
    """CSV-ul are coloanele cerute si valorile corecte; calibrarea falsa e
    refuzata."""
    path, cal = calib_file()
    Kb, distb, _wh, _info = vd.load_calibration(path)
    tmpdir = tempfile.mkdtemp()
    scene = [(syn.R_MARKER_FLAT, (0.3, -0.2, 6.0)),
             (syn.R_MARKER_FLAT, (0.0, 0.0, 0.30))]   # a doua: prea aproape
    render_all(scene, out_dir=tmpdir)
    csv_path = os.path.join(tmpdir, 'out.csv')
    det = vd.Detector(Kb, distb, MARKER_M, 26, refine=True)
    res = vd.run(vd.list_images(tmpdir), det, csv_path, verbose=False)
    assert len(res) == 2

    import csv as _csv
    with open(csv_path) as f:
        rows = list(_csv.DictReader(f))
    assert list(rows[0].keys()) == vd.CSV_HEADER, list(rows[0].keys())
    assert rows[0]['detected'] == '1' and rows[0]['marker_id'] == '26'
    assert abs(float(rows[0]['tvec_z']) - 6.0) < 0.12, rows[0]['tvec_z']
    assert rows[0]['fits_in_frame'] == '1'
    # La 0.30 m markerul depaseste cadrul si ArUco nu il mai gaseste deloc,
    # deci randul e de nedetectie, cu campuri goale.
    assert rows[1]['detected'] == '0', rows[1]
    assert rows[1]['marker_px'] == '' and rows[1]['tvec_z'] == '' 

    # calibrare falsa (focala geometrica): refuz
    fake, _ = calib_file(real=False)
    try:
        vd.load_calibration(fake)
        raise AssertionError('a acceptat o calibrare care nu e reala')
    except ValueError:
        pass
    return (f"CSV cu {len(vd.CSV_HEADER)} coloane, tvec_z "
            f"{float(rows[0]['tvec_z']):.3f} m, fits_in_frame 1/0 corect; "
            f"calibrare falsa refuzata")


def test_F3_unealta_de_comparare_end_to_end():
    """compare_detectors.py rulat ca proces, pe imagini pe disc."""
    path, cal = calib_file()
    tmpdir = tempfile.mkdtemp()
    render_all(poses_15()[:8], out_dir=tmpdir)
    tool = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'compare_detectors.py')
    r = subprocess.run([sys.executable, tool, '--images', tmpdir,
                        '--calib', path, '--mod', 'ambele'],
                       capture_output=True, text=True, timeout=600)
    out = r.stdout
    assert 'MOD: identic' in out and 'MOD: fidel' in out, out[-400:]
    assert 'DE ACORD' in out, out[-800:]
    assert r.returncode == 0, f"cod {r.returncode}\\n{out[-800:]}"
    linii = [ln.strip() for ln in out.splitlines() if 'colturi :' in ln]
    return f"ruleaza ca proces, ambele moduri; {linii[0] if linii else 'ok'}"


TESTS = [
    ('F3: 15 poze, ambele implementari', test_F3_15_poze_ambele_implementari),
    ('F3: acord pe detectat/nedetectat', test_F3_acord_pe_detectat_nedetectat),
    ('F3: fits_in_frame nu e dezacord',
     test_F3_tratarea_fits_in_frame_nu_e_dezacord),
    ('F3 NEGATIV: calibrare gresita prinsa',
     test_F3_NEGATIV_calibrare_gresita_e_prinsa),
    ('F3 NEGATIV: latura gresita prinsa',
     test_F3_NEGATIV_latura_gresita_e_prinsa),
    ('F3: verify_detection nu importa nova',
     test_F3_verify_detection_nu_importa_nova),
    ('F3: CSV si refuzul calibrarii false',
     test_F3_verify_detection_csv_si_refuz),
    ('F3: compare_detectors end-to-end',
     test_F3_unealta_de_comparare_end_to_end),
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
