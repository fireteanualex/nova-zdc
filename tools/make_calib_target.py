#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Generator de tinte de calibrare (F1), PNG la 300 DPI, pregatit de tipar.

    python3 tools/make_calib_target.py --type charuco --paper A3
    python3 tools/make_calib_target.py --type checker --paper A3 --cols 9 --rows 6 --square-mm 40

De ce nu o poza de pe internet: rezolutie de ecran, grile patrate simetrice,
borduri colorate care ating patratele de la margine. Toate trei strica
detectia sau calibrarea, tacut.

**`--cols` si `--rows` sunt PATRATE, nu colturi.** O tabla 9x6 patrate are
8x5 colturi interioare - aceea e valoarea pe care o cere
`tools/calibrate_camera.py`. Unealta scrie langa PNG un fisier `.json` cu
geometria exacta si afiseaza comanda de calibrare gata formata, ca sa nu fie
nevoie de conversia asta din cap.

Trei decizii de proiectare, cu motivele lor:

- **Margine alba de cel putin o latime de patrat**, pe toate laturile.
  `findChessboardCorners` are nevoie de zona alba ca sa delimiteze tabla;
  fara ea colturile exterioare sunt deplasate sau ratate. Daca hartia
  permite, marginea e mai mare - centram tabla si lasam tot restul alb.
- **Fara borduri colorate sau linii de taiere.** Orice tranzitie care nu e
  alb-negru poate produce un contur care confunda detectorul.
- **Grila asimetrica implicit** (9x6 patrate). O grila patrata e ambigua la
  rotatie; vezi §5.17 din CLAUDE.md pentru ce inseamna asta in practica.

Pentru ChArUco, dictionarul e `DICT_5X5_250` - deliberat ALTUL decat
`DICT_4X4_50` al markerului de misiune (ID 26), ca sa nu existe nicio sansa
ca o tinta de calibrare aflata in cadru sa fie confundata cu markerul.
"""

import argparse
import json
import math
import os
import struct
import sys
import zlib

import cv2
import numpy as np

DPI = 300
MM_PER_INCH = 25.4

#: Hartie in landscape, milimetri.
PAPERS = {
    'A4': (297.0, 210.0),
    'A3': (420.0, 297.0),
    'A2': (594.0, 420.0),
}

#: ChArUco: dictionar DIFERIT de cel al markerului de misiune (DICT_4X4_50).
CHARUCO_DICT = cv2.aruco.DICT_5X5_250

#: Markerul ocupa ~75% din latura patratului: destul de mare ca sa fie
#: decodabil la distanta, destul de mic ca sa ramana alb in jur pentru
#: localizarea coltului de sah.
MARKER_RATIO = 0.75

#: Marginea alba minima, in latimi de patrat.
MARGIN_SQUARES = 1.0

DEFAULT_OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs')


def mm_to_px(mm, dpi=DPI):
    return int(round(mm / MM_PER_INCH * dpi))


def fit_square_mm(paper_mm, cols, rows, margin_squares=MARGIN_SQUARES):
    """Cel mai mare patrat INTREG in mm care incape cu marginea ceruta."""
    pw, ph = paper_mm
    s = min(pw / (cols + 2 * margin_squares), ph / (rows + 2 * margin_squares))
    return float(math.floor(s))


def check_fits(paper_mm, cols, rows, square_mm,
               margin_squares=MARGIN_SQUARES):
    """(ok, mesaj). Refuza explicit, cu cifre, daca nu incape."""
    pw, ph = paper_mm
    need_w = (cols + 2 * margin_squares) * square_mm
    need_h = (rows + 2 * margin_squares) * square_mm
    if need_w <= pw and need_h <= ph:
        return True, ''
    biggest = fit_square_mm(paper_mm, cols, rows, margin_squares)
    return False, (
        f"nu incape: {cols}x{rows} patrate de {square_mm:g} mm plus marginea "
        f"de {margin_squares:g} patrate cer {need_w:.0f}x{need_h:.0f} mm, "
        f"hartia are {pw:.0f}x{ph:.0f} mm.\n"
        f"  Optiuni: --square-mm {biggest:g} (maximul care incape), "
        f"hartie mai mare, sau mai putine patrate.")


def render_checker(cols, rows, square_px):
    """Tabla de sah, doar patratele (fara margine). Primul patrat (0,0) e
    NEGRU, ca sa fie clar ce colt e primul colt interior."""
    img = np.full((rows * square_px, cols * square_px), 255, np.uint8)
    for j in range(rows):
        for i in range(cols):
            if (i + j) % 2 == 0:
                img[j * square_px:(j + 1) * square_px,
                    i * square_px:(i + 1) * square_px] = 0
    return img


def render_charuco(cols, rows, square_px, marker_ratio=MARKER_RATIO):
    """Tabla ChArUco, doar patratele. `generateImage` pastreaza proportiile
    si centreaza; cerem exact dimensiunea tablei, cu marginea 0, si o lipim
    noi pe pagina - asa controlam marimea fizica la pixel."""
    d = cv2.aruco.getPredefinedDictionary(CHARUCO_DICT)
    board = cv2.aruco.CharucoBoard((cols, rows), 1.0, marker_ratio, d)
    img = board.generateImage((cols * square_px, rows * square_px),
                              marginSize=0)
    return img, board


def compose_page(board_img, paper_mm, dpi=DPI):
    """Tabla centrata pe pagina alba. Marginea reala e cel putin cea ceruta,
    plus tot ce ramane din hartie."""
    pw, ph = mm_to_px(paper_mm[0], dpi), mm_to_px(paper_mm[1], dpi)
    bh, bw = board_img.shape[:2]
    if bw > pw or bh > ph:
        raise ValueError(f"tabla {bw}x{bh} px nu incape pe {pw}x{ph} px")
    page = np.full((ph, pw), 255, np.uint8)
    x0, y0 = (pw - bw) // 2, (ph - bh) // 2
    page[y0:y0 + bh, x0:x0 + bw] = board_img
    return page, (x0, y0)


def write_png_with_dpi(path, img, dpi=DPI):
    """PNG cu chunk pHYs corect, ca tiparirea la 100% sa dea marimea reala.

    cv2.imwrite nu scrie rezolutia fizica, iar fara ea driverul de imprimanta
    nu are de unde sti ca imaginea e la 300 DPI - poate o scala ca sa umple
    pagina, si atunci latura masurata cu rigla nu mai corespunde cu ce dam
    calibrarii. Inseram chunk-ul manual (PIL nu e in stack).
    """
    ok, buf = cv2.imencode('.png', img)
    if not ok:
        raise IOError('cv2.imencode a esuat')
    data = buf.tobytes()
    ppm = int(round(dpi / MM_PER_INCH * 1000))      # pixeli pe metru
    phys = struct.pack('>IIB', ppm, ppm, 1)         # unitate 1 = metru
    chunk = (struct.pack('>I', len(phys)) + b'pHYs' + phys
             + struct.pack('>I', zlib.crc32(b'pHYs' + phys) & 0xFFFFFFFF))
    # dupa semnatura (8) + IHDR (4 lungime + 4 tip + 13 date + 4 CRC = 25)
    cut = 8 + 25
    if data[12:16] != b'IHDR':
        raise IOError('PNG neasteptat: IHDR lipsa')
    with open(path, 'wb') as f:
        f.write(data[:cut] + chunk + data[cut:])
    return ppm


def build(kind, paper, cols, rows, square_mm=None, dpi=DPI,
          marker_ratio=MARKER_RATIO):
    """(pagina, metadate). Ridica ValueError daca nu incape."""
    paper_mm = PAPERS[paper]
    if square_mm is None:
        square_mm = fit_square_mm(paper_mm, cols, rows)
        if square_mm < 10:
            raise ValueError(
                f"patratul ar iesi de {square_mm:g} mm pe {paper}; sub 10 mm "
                f"colturile devin imprecise. Foloseste hartie mai mare sau "
                f"mai putine patrate.")
    ok, why = check_fits(paper_mm, cols, rows, square_mm)
    if not ok:
        raise ValueError(why)

    square_px = mm_to_px(square_mm, dpi)
    board = None
    if kind == 'checker':
        board_img = render_checker(cols, rows, square_px)
    else:
        board_img, board = render_charuco(cols, rows, square_px, marker_ratio)
    page, (x0, y0) = compose_page(board_img, paper_mm, dpi)

    meta = {
        'type': kind,
        'paper': paper,
        'paper_mm': list(paper_mm),
        'dpi': dpi,
        'squares_cols': cols,
        'squares_rows': rows,
        'inner_corners_cols': cols - 1,
        'inner_corners_rows': rows - 1,
        'square_mm_nominal': square_mm,
        'board_mm': [cols * square_mm, rows * square_mm],
        'margin_mm_actual': [round(x0 / dpi * MM_PER_INCH, 1),
                             round(y0 / dpi * MM_PER_INCH, 1)],
        'page_px': [page.shape[1], page.shape[0]],
    }
    if kind == 'charuco':
        meta.update({
            'aruco_dict': 'DICT_5X5_250',
            'marker_ratio': marker_ratio,
            'marker_mm_nominal': round(square_mm * marker_ratio, 2),
            'n_markers': len(board.getIds()),
            'n_chessboard_corners': len(board.getChessboardCorners()),
        })
    return page, meta


def instructions(meta, png_path, json_path):
    m = meta
    ic = f"{m['inner_corners_cols']}x{m['inner_corners_rows']}"
    lines = [
        "",
        "=" * 68,
        f"  {m['type'].upper()} {m['squares_cols']}x{m['squares_rows']} patrate"
        f"  ->  {ic} COLTURI INTERIOARE",
        "=" * 68,
        f"  hartie        : {m['paper']} landscape "
        f"({m['paper_mm'][0]:.0f} x {m['paper_mm'][1]:.0f} mm)",
        f"  patrat nominal: {m['square_mm_nominal']:g} mm",
    ]
    if m['type'] == 'charuco':
        lines.append(f"  marker nominal: {m['marker_mm_nominal']:g} mm "
                     f"({m['marker_ratio']:.0%} din patrat), "
                     f"{m['aruco_dict']}, {m['n_markers']} markeri")
    lines += [
        f"  tabla         : {m['board_mm'][0]:.0f} x {m['board_mm'][1]:.0f} mm",
        f"  margine alba  : {m['margin_mm_actual'][0]:.1f} mm pe laterale, "
        f"{m['margin_mm_actual'][1]:.1f} mm sus/jos "
        f"(minim cerut {m['square_mm_nominal']:g} mm)",
        f"  fisier        : {png_path}  ({m['page_px'][0]}x{m['page_px'][1]} px "
        f"@ {m['dpi']} DPI)",
        f"  geometrie     : {json_path}",
        "",
        "  TIPAR:",
        "    1. Scalare 100%. NU 'fit to page', NU 'shrink to fit'.",
        "       Daca driverul scaleaza, latura masurata nu mai corespunde si",
        "       toate distantele din calibrare ies proportional gresite.",
        "    2. Hartie mata. Lucioasa da reflexii care sterg colturile.",
        "    3. Dupa tipar MASOARA cu rigla latura mai multor patrate",
        "       (de ex. 5 patrate, apoi imparte) si foloseste valoarea",
        "       MASURATA, nu cea nominala, la calibrare.",
        "    4. Lipeste pe carton rigid sau pe placa. O tabla indoita cu 1 mm",
        "       strica reproiectia mai mult decat orice altceva.",
        "",
        "  CALIBRARE (cu latura masurata, nu cea nominala):",
    ]
    if m['type'] == 'charuco':
        lines.append("    python3 tools/calibrate_camera.py --live \\")
        lines.append(f"        --target charuco "
                     f"--cols {m['squares_cols']} --rows {m['squares_rows']} \\")
        lines.append(f"        --square-mm {m['square_mm_nominal']:g} "
                     f"--marker-mm {m['marker_mm_nominal']:g}")
    else:
        lines.append("    python3 tools/calibrate_camera.py --live \\")
        lines.append(f"        --target checker "
                     f"--cols {m['inner_corners_cols']} "
                     f"--rows {m['inner_corners_rows']} \\")
        lines.append(f"        --square-mm {m['square_mm_nominal']:g}")
        lines.append("    (atentie: --cols/--rows acolo sunt COLTURI "
                     "INTERIOARE)")
    lines += ["", "=" * 68, ""]
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--type', choices=('charuco', 'checker'), default='charuco')
    p.add_argument('--paper', choices=sorted(PAPERS), default='A3')
    p.add_argument('--cols', type=int, default=9,
                   help='PATRATE pe orizontala (nu colturi)')
    p.add_argument('--rows', type=int, default=6,
                   help='PATRATE pe verticala (nu colturi)')
    p.add_argument('--square-mm', type=float, default=None,
                   help='implicit: cel mai mare patrat intreg care incape')
    p.add_argument('--marker-ratio', type=float, default=MARKER_RATIO)
    p.add_argument('--dpi', type=int, default=DPI)
    p.add_argument('--out', default=None)
    a = p.parse_args()

    if a.cols == a.rows:
        print(f"  ATENTIE: grila {a.cols}x{a.rows} e patrata, deci ambigua la "
              f"rotatie cu 90 de grade.\n  Foloseste o grila asimetrica "
              f"(implicit 9x6). Vezi §5.17 din CLAUDE.md.")
    if a.cols < 3 or a.rows < 3:
        p.error('minimum 3x3 patrate')

    try:
        page, meta = build(a.type, a.paper, a.cols, a.rows, a.square_mm,
                           a.dpi, a.marker_ratio)
    except ValueError as e:
        print(f"\n  REFUZ: {e}\n")
        return 1

    out = a.out or os.path.join(
        DEFAULT_OUT_DIR,
        f"calib_{a.type}_{a.paper}_{meta['squares_cols']}x"
        f"{meta['squares_rows']}_{meta['square_mm_nominal']:g}mm.png")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    write_png_with_dpi(out, page, a.dpi)
    json_path = os.path.splitext(out)[0] + '.json'
    with open(json_path, 'w') as f:
        json.dump(meta, f, indent=2)
        f.write('\n')
    print(instructions(meta, out, json_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
