#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru tools/make_calib_target.py (F1), fara imprimanta si fara camera.

    python3 tools/test_make_calib_target.py

Metoda: tinta generata e randata printr-o proiectie CUNOSCUTA (camera
sintetica IMX708 din tools/synthetic.py), apoi detectorul real trebuie sa
gaseasca in ea exact ce am pus. Adevarul e cunoscut la fiecare test, deci
verificam cifre - numar de colturi, permutare, potrivire in pixeli - nu doar
ca "a rulat".

Ce NU acopera: tiparul propriu-zis (scalare, hartie lucioasa, tabla indoita)
si iluminarea reala. Alea sunt E2, cu ruleta si trei conditii de lumina.
"""

import os
import struct
import sys
import tempfile
import zlib

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import calibrate_camera as cc            # noqa: E402
import make_calib_target as mt           # noqa: E402
import synthetic as syn                  # noqa: E402

K, DIST, WH = syn.imx708()
W, H = WH


# --- ajutoare: adevarul geometric al paginii generate ------------------------

def build_page(kind='checker', cols=9, rows=6, paper='A3', square_mm=None):
    """(pagina, metadate, px_per_mm, origin_px). Originea planului tintei e
    centrul paginii."""
    page, meta = mt.build(kind, paper, cols, rows, square_mm)
    ph, pw = page.shape
    return page, meta, meta['dpi'] / mt.MM_PER_INCH, (pw / 2.0, ph / 2.0)


def checker_truth_mm(meta):
    """Colturile interioare, in mm fata de centrul paginii, in ordinea
    canonica a lui findChessboardCorners (pe linii, de sus in jos)."""
    sq = meta['square_mm_nominal']
    ic, ir = meta['inner_corners_cols'], meta['inner_corners_rows']
    bw, bh = meta['board_mm']
    pts = []
    for j in range(ir):
        for i in range(ic):
            pts.append([(i + 1) * sq - bw / 2.0, (j + 1) * sq - bh / 2.0])
    return np.array(pts, np.float64)


def charuco_truth_mm(meta):
    """Idem, pentru ChArUco: coordonatele din board.getChessboardCorners(),
    exprimate in mm si mutate in centrul paginii."""
    sq = meta['square_mm_nominal']
    d = cv2.aruco.getPredefinedDictionary(mt.CHARUCO_DICT)
    board = cv2.aruco.CharucoBoard(
        (meta['squares_cols'], meta['squares_rows']), sq,
        sq * meta['marker_ratio'], d)
    cor = board.getChessboardCorners()[:, :2].astype(np.float64)
    bw, bh = meta['board_mm']
    return cor - np.array([bw / 2.0, bh / 2.0]), board


def render(page, px_per_mm, origin_px, R, t_mm):
    return syn.render_planar_target(page, px_per_mm, origin_px, K, DIST, R,
                                    np.asarray(t_mm, np.float64), WH, bg=128)


def permutation(detected, truth_px):
    """Pentru fiecare colt detectat, indexul coltului fizic cel mai apropiat,
    plus distanta maxima de potrivire."""
    d = np.linalg.norm(truth_px[None, :, :] - detected[:, None, :], axis=2)
    idx = d.argmin(axis=1)
    return idx, float(d[np.arange(len(detected)), idx].max())


# --- teste -------------------------------------------------------------------

def test_numar_colturi_checker():
    """Numarul de colturi detectate e exact cel asteptat, si fiecare se
    potriveste cu adevarul sub 1 px."""
    page, meta, ppm, org = build_page('checker', 9, 6)
    pat = (meta['inner_corners_cols'], meta['inner_corners_rows'])
    assert pat == (8, 5), pat
    R, t = syn.rot(0, 0, 0), [0, 0, 700.0]
    img = render(page, ppm, org, R, t)
    got = cc.find_corners(img, pat)
    assert got is not None, 'tabla negasita'
    assert len(got) == pat[0] * pat[1] == 40, len(got)
    truth = syn.project(K, DIST, R, t, checker_truth_mm(meta))
    _, resid = permutation(got, truth)
    assert resid < 1.0, f"potrivire max {resid:.2f} px"
    return f"{len(got)} colturi (9x6 patrate -> 8x5), potrivire max {resid:.2f} px"


def test_numar_colturi_charuco():
    """ChArUco: toate colturile de sah SI toti markerii."""
    page, meta, ppm, org = build_page('charuco', 9, 6)
    truth_mm, board = charuco_truth_mm(meta)
    n_expect = (meta['squares_cols'] - 1) * (meta['squares_rows'] - 1)
    assert len(truth_mm) == n_expect == 40, len(truth_mm)
    R, t = syn.rot(0, 0, 0), [0, 0, 700.0]
    img = render(page, ppm, org, R, t)
    det = cv2.aruco.CharucoDetector(board)
    cor, ids, m_cor, m_ids = det.detectBoard(img)
    assert cor is not None and len(cor) == n_expect, \
        f"{0 if cor is None else len(cor)} colturi din {n_expect}"
    assert m_ids is not None and len(m_ids) == meta['n_markers'], \
        f"{0 if m_ids is None else len(m_ids)} markeri din {meta['n_markers']}"
    truth = syn.project(K, DIST, R, t, truth_mm)
    got = cor.reshape(-1, 2)
    resid = float(np.max(np.linalg.norm(
        got - truth[ids.flatten()], axis=1)))
    assert resid < 1.0, f"potrivire max {resid:.2f} px"
    return (f"{len(cor)}/{n_expect} colturi, {len(m_ids)}/{meta['n_markers']} "
            f"markeri, potrivire max {resid:.2f} px")


def test_ordine_stabila_la_rotatie():
    """Grila asimetrica: permutarea colt-detectat -> colt-fizic e ACEEASI la
    0/90/180/270 grade. Adica ordinea e legata de tabla, nu de cum cade ea
    in cadru."""
    page, meta, ppm, org = build_page('checker', 9, 6)
    pat = (meta['inner_corners_cols'], meta['inner_corners_rows'])
    truth_mm = checker_truth_mm(meta)
    perms = {}
    for rz in (0, 90, 180, 270):
        R, t = syn.rot(0, 0, rz), [0, 0, 700.0]
        img = render(page, ppm, org, R, t)
        got = cc.find_corners(img, pat)
        assert got is not None, f"negasit la {rz} grade"
        idx, resid = permutation(got, syn.project(K, DIST, R, t, truth_mm))
        assert resid < 1.0, f"{rz} grade: potrivire {resid:.2f} px"
        perms[rz] = idx
    ref = perms[0]
    for rz, idx in perms.items():
        assert np.array_equal(idx, ref), (
            f"la {rz} grade permutarea difera de cea de la 0 grade")
    return "aceeasi permutare la 0/90/180/270 grade"


def test_NEGATIV_grila_patrata_are_ordine_instabila():
    """CAZUL NEGATIV care justifica grila asimetrica implicita.

    Pe o grila patrata (8x8 colturi) permutarea SE SCHIMBA cu rotatia: acelasi
    colt fizic primeste alt index. De aceea 9x6 e implicit.

    Atentie la ce NU spune testul: masurat, asta nu strica intrinsecii - vezi
    §5.17 din CLAUDE.md."""
    page, meta, ppm, org = build_page('checker', 9, 9)
    pat = (meta['inner_corners_cols'], meta['inner_corners_rows'])
    assert pat == (8, 8), pat
    truth_mm = checker_truth_mm(meta)
    perms = {}
    for rz in (0, 90):
        R, t = syn.rot(0, 0, rz), [0, 0, 700.0]
        img = render(page, ppm, org, R, t)
        got = cc.find_corners(img, pat)
        assert got is not None, f"negasit la {rz} grade"
        idx, resid = permutation(got, syn.project(K, DIST, R, t, truth_mm))
        assert resid < 1.0
        perms[rz] = idx
    assert not np.array_equal(perms[0], perms[90]), (
        'grila patrata a dat aceeasi permutare; testul nu mai demonstreaza '
        'nimic si trebuie regandit')
    n_diff = int(np.sum(perms[0] != perms[90]))
    return (f"{n_diff}/{len(perms[0])} indici se schimba la 90 grade "
            f"(primul colt: fizic #{perms[0][0]} -> #{perms[90][0]})")


def test_charuco_ids_stabile_la_rotatie():
    """ChArUco nu are ambiguitatea de mai sus deloc: fiecare colt isi
    pastreaza ID-ul, pentru ca markerii din jur il identifica."""
    page, meta, ppm, org = build_page('charuco', 9, 6)
    truth_mm, board = charuco_truth_mm(meta)
    det = cv2.aruco.CharucoDetector(board)
    for rz in (0, 90, 180, 270):
        R, t = syn.rot(0, 0, rz), [0, 0, 700.0]
        img = render(page, ppm, org, R, t)
        cor, ids, _, _ = det.detectBoard(img)
        assert cor is not None and len(cor) == len(truth_mm), \
            f"{rz} grade: {0 if cor is None else len(cor)} colturi"
        truth = syn.project(K, DIST, R, t, truth_mm)
        resid = float(np.max(np.linalg.norm(
            cor.reshape(-1, 2) - truth[ids.flatten()], axis=1)))
        assert resid < 1.0, f"{rz} grade: ID-ul {resid:.2f} px de coltul fizic"
    return "ID-ul fiecarui colt corespunde aceluiasi colt fizic la toate 4 rotatiile"


def test_margine_suficienta_pana_la_40_grade():
    """Marginea alba trebuie sa tina detectia la inclinari mari - acolo tabla
    umple cadrul si colturile exterioare ajung langa margine."""
    page, meta, ppm, org = build_page('checker', 9, 6)
    pat = (meta['inner_corners_cols'], meta['inner_corners_rows'])
    truth_mm = checker_truth_mm(meta)
    rezultate = []
    for tilt in (0, 10, 20, 30, 40):
        R, t = syn.rot(tilt, 0, 0), [0, 0, 700.0]
        img = render(page, ppm, org, R, t)
        got = cc.find_corners(img, pat)
        assert got is not None, f"negasit la {tilt} grade inclinare"
        assert len(got) == pat[0] * pat[1]
        _, resid = permutation(got, syn.project(K, DIST, R, t, truth_mm))
        assert resid < 1.5, f"{tilt} grade: potrivire {resid:.2f} px"
        rezultate.append(f"{tilt}:{resid:.2f}")
    return "toate gasite; potrivire max per inclinare " + " ".join(rezultate)


def _crop_board(page, meta):
    """Tabla decupata exact pe patrate: varianta fara margine alba."""
    spx = mt.mm_to_px(meta['square_mm_nominal'], meta['dpi'])
    ph, pw = page.shape
    x0 = (pw - meta['squares_cols'] * spx) // 2
    y0 = (ph - meta['squares_rows'] * spx) // 2
    crop = page[y0:y0 + meta['squares_rows'] * spx,
                x0:x0 + meta['squares_cols'] * spx]
    return crop, (crop.shape[1] / 2.0, crop.shape[0] / 2.0)


def _classic_corners(gray, pattern):
    """findChessboardCorners clasic, fara rafinare - exact ce foloseste orice
    unealta care nu a trecut la varianta SB."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, pattern, flags=flags)
    return c.reshape(-1, 2) if ok else None


def test_NEGATIV_fara_margine_pe_fundal_inchis():
    """CAZUL NEGATIV care justifica marginea alba, masurat precis.

    Conditia in care conteaza: detectorul CLASIC, tabla decupata pe patrate,
    fundal inchis. Atunci patratele negre de pe margine fuzioneaza cu fundalul
    si conturul tablei nu mai poate fi delimitat - 0 din 5 poze.

    Cu marginea alba: 5 din 5, pe acelasi fundal.

    Testul verifica si ca `findChessboardCornersSB` NU are aceasta
    sensibilitate. Pastram marginea oricum: unealta noastra cade pe detectorul
    clasic daca SB lipseste, alte unelte il pot folosi direct, si marginea nu
    costa nimic. Vezi §5.18 din CLAUDE.md."""
    page, meta, ppm, org = build_page('checker', 9, 6)
    pat = (meta['inner_corners_cols'], meta['inner_corners_rows'])
    crop, org_c = _crop_board(page, meta)
    tilts = (0, 10, 20, 30, 40)

    def numara(fn, src, orgx, bg):
        n = 0
        for tilt in tilts:
            img = syn.render_planar_target(src, ppm, orgx, K, DIST,
                                           syn.rot(tilt, 0, 0),
                                           np.array([0, 0, 700.0]), WH, bg=bg)
            if fn(img, pat) is not None:
                n += 1
        return n

    cu = numara(_classic_corners, page, org, 0)
    fara = numara(_classic_corners, crop, org_c, 0)
    assert cu == len(tilts), f"clasic, cu margine, fundal negru: {cu}/{len(tilts)}"
    assert fara == 0, (
        f"clasic, fara margine, fundal negru: {fara}/{len(tilts)} - "
        f"asteptam 0; testul nu mai demonstreaza ca marginea conteaza")

    sb_fara = numara(lambda g, p: cc.find_corners(g, p), crop, org_c, 0)
    assert sb_fara == len(tilts), (
        f"SB fara margine: {sb_fara}/{len(tilts)} - daca si SB a devenit "
        f"sensibil, nota din §5.18 trebuie corectata")
    return (f"clasic: {cu}/5 cu margine, {fara}/5 fara (fundal negru); "
            f"SB: {sb_fara}/5 fara margine")


def test_NEGATIV_refuza_daca_nu_incape():
    ok, why = mt.check_fits(mt.PAPERS['A4'], 9, 6, 40.0)
    assert ok is False and 'nu incape' in why, why
    assert '--square-mm' in why, 'refuzul nu spune ce sa faci'
    try:
        mt.build('checker', 'A4', 9, 6, 40.0)
        raise AssertionError('a generat o tinta care nu incape pe hartie')
    except ValueError:
        pass
    ok2, _ = mt.check_fits(mt.PAPERS['A3'], 9, 6, 37.0)
    assert ok2 is True, 'a refuzat o tinta care incape'
    biggest = mt.fit_square_mm(mt.PAPERS['A4'], 9, 6)
    assert mt.check_fits(mt.PAPERS['A4'], 9, 6, biggest)[0] is True
    return (f"A4 cu 40 mm: refuzat, sugereaza {biggest:g} mm; "
            f"A3 cu 37 mm: acceptat")


def test_margine_si_alb_negru():
    """Marginea reala e cel putin o latime de patrat pe toate laturile, si
    pagina nu contine nimic in afara de alb si negru."""
    page, meta, _, _ = build_page('charuco', 9, 6)
    sq_px = mt.mm_to_px(meta['square_mm_nominal'], meta['dpi'])
    ph, pw = page.shape
    assert page.dtype == np.uint8 and page.ndim == 2, 'pagina nu e gri 8 biti'
    # chenarul de latimea unui patrat, pe toate laturile, trebuie sa fie alb
    for banda in (page[:sq_px, :], page[-sq_px:, :],
                  page[:, :sq_px], page[:, -sq_px:]):
        assert banda.min() == 255, 'marginea alba e invadata de tipar'
    vals = np.unique(page)
    assert set(vals.tolist()) <= {0, 255}, \
        f"pagina are {len(vals)} niveluri, nu doar alb si negru: {vals[:8]}"
    mx, my = meta['margin_mm_actual']
    assert mx >= meta['square_mm_nominal'] and my >= meta['square_mm_nominal']
    return (f"margine {mx:.1f}x{my:.1f} mm >= patrat "
            f"{meta['square_mm_nominal']:g} mm; doar 2 niveluri de gri")


def test_dimensiuni_px_exacte():
    """mm x 300/25.4, eroare 0 px: pagina, tabla si patratul."""
    for kind in ('checker', 'charuco'):
        page, meta, _, _ = build_page(kind, 9, 6)
        dpi = meta['dpi']
        ph, pw = page.shape
        for mm, got, eticheta in (
                (meta['paper_mm'][0], pw, 'latime pagina'),
                (meta['paper_mm'][1], ph, 'inaltime pagina')):
            astept = mt.mm_to_px(mm, dpi)
            assert got == astept, (f"{kind} {eticheta}: {got} px, astept "
                                   f"{astept} px pentru {mm} mm")
        spx = mt.mm_to_px(meta['square_mm_nominal'], dpi)
        for n, mm, eticheta in (
                (meta['squares_cols'], meta['board_mm'][0], 'latime tabla'),
                (meta['squares_rows'], meta['board_mm'][1], 'inaltime tabla')):
            assert n * spx == mt.mm_to_px(mm, dpi), eticheta
    sq = mt.mm_to_px(37.0, 300)
    assert sq == round(37.0 / 25.4 * 300), sq
    return f"pagina, tabla si patratul: 0 px eroare (patrat 37 mm = {sq} px)"


def test_png_are_dpi():
    """PNG-ul trebuie sa poarte rezolutia fizica (chunk pHYs), altfel
    driverul de imprimanta poate scala si latura masurata nu mai corespunde."""
    page, meta, _, _ = build_page('checker', 9, 6)
    tmp = os.path.join(tempfile.mkdtemp(), 't.png')
    ppm = mt.write_png_with_dpi(tmp, page, meta['dpi'])
    raw = open(tmp, 'rb').read()
    i = raw.find(b'pHYs')
    assert i > 0, 'PNG fara chunk pHYs'
    x, y, unit = struct.unpack('>IIB', raw[i + 4:i + 13])
    assert (x, y, unit) == (ppm, ppm, 1), (x, y, unit)
    crc = struct.unpack('>I', raw[i + 13:i + 17])[0]
    assert crc == zlib.crc32(raw[i:i + 13]) & 0xFFFFFFFF, 'CRC gresit'
    back = cv2.imread(tmp, cv2.IMREAD_GRAYSCALE)
    assert back is not None and back.shape == page.shape, 'PNG necitibil'
    assert np.array_equal(back, page), 'pixelii s-au schimbat'
    dpi_back = x * mt.MM_PER_INCH / 1000.0
    return f"pHYs {x} px/m = {dpi_back:.1f} DPI, CRC ok, imaginea intacta"


def test_metadate_geometrie():
    """Sidecar-ul JSON trebuie sa spuna explicit colturile interioare -
    conversia patrate -> colturi e exact locul unde se greseste."""
    page, meta, _, _ = build_page('charuco', 9, 6)
    assert meta['inner_corners_cols'] == meta['squares_cols'] - 1
    assert meta['inner_corners_rows'] == meta['squares_rows'] - 1
    assert meta['aruco_dict'] == 'DICT_5X5_250', meta['aruco_dict']
    assert meta['n_chessboard_corners'] == 40
    txt = mt.instructions(meta, 'x.png', 'x.json')
    assert '--target charuco' in txt and '--cols 9 --rows 6' in txt
    assert '100%' in txt and 'MASOARA' in txt
    return (f"{meta['squares_cols']}x{meta['squares_rows']} patrate -> "
            f"{meta['inner_corners_cols']}x{meta['inner_corners_rows']} "
            f"colturi; dictionar {meta['aruco_dict']}")


def test_dictionarul_nu_se_confunda_cu_markerul_de_misiune():
    """Tinta de calibrare nu are voie sa produca o detectie in DICT_4X4_50,
    ca sa nu poata fi luata drept markerul ID 26."""
    page, meta, ppm, org = build_page('charuco', 9, 6)
    img = render(page, ppm, org, syn.rot(0, 0, 0), [0, 0, 700.0])
    d4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(d4, cv2.aruco.DetectorParameters())
    _, ids, _ = det.detectMarkers(img)
    n = 0 if ids is None else len(ids)
    assert n == 0, f"tinta de calibrare a produs {n} detectii in DICT_4X4_50: {ids}"
    return "zero detectii in DICT_4X4_50 pe tinta ChArUco"


TESTS = [
    ('checker: numar de colturi', test_numar_colturi_checker),
    ('charuco: colturi si markeri', test_numar_colturi_charuco),
    ('ordine stabila la rotatie (asimetric)', test_ordine_stabila_la_rotatie),
    ('NEGATIV: grila patrata, ordine instabila',
     test_NEGATIV_grila_patrata_are_ordine_instabila),
    ('charuco: ID-uri stabile la rotatie', test_charuco_ids_stabile_la_rotatie),
    ('margine suficienta pana la 40 grade',
     test_margine_suficienta_pana_la_40_grade),
    ('NEGATIV: fara margine, detector clasic, fundal inchis',
     test_NEGATIV_fara_margine_pe_fundal_inchis),
    ('NEGATIV: refuza daca nu incape pe hartie',
     test_NEGATIV_refuza_daca_nu_incape),
    ('margine alba si doar alb-negru', test_margine_si_alb_negru),
    ('dimensiuni px exacte', test_dimensiuni_px_exacte),
    ('PNG poarta DPI (pHYs)', test_png_are_dpi),
    ('metadate de geometrie', test_metadate_geometrie),
    ('dictionar diferit de markerul de misiune',
     test_dictionarul_nu_se_confunda_cu_markerul_de_misiune),
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
