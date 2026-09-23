#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verificare incrucisata a celor doua implementari de detectie (F3).

    python3 tools/compare_detectors.py --images ~/e2/d05_soare

Ruleaza pe acelasi director:
  A) nova/detector_pi.py      - implementarea de bord
  B) tools/verify_detection.py - implementarea independenta

si raporteaza unde nu sunt de acord. Doua implementari scrise separat care
dau aceleasi colturi si aceeasi distanta sunt evidenta mult mai puternica
decat una singura care se autoconfirma. Daca diverg, avem un bug gasit pe
imagini statice, nu in zbor.

Praguri de acceptare: **sub 1 px pe colturi, sub 1% pe distanta.**

**Ce dovedeste si ce NU dovedeste comparatia.** Ambele implementari apeleaza
acelasi `cv2.aruco.ArucoDetector`, deci cu aceiasi parametri produc colturi
IDENTICE bit cu bit (masurat: 0.000 px). Comparatia verifica deci
**cablajul** - dictionar, ID, latura markerului, incarcarea calibrarii,
punctele-obiect, flag-ul de solvePnP, unitatile, ordinea colturilor - adica
exact clasa de greseli care omoara proiectele. NU verifica detectorul OpenCV
in sine; pentru asta ar trebui o a treia implementare, independenta de
OpenCV, ceea ce nu merita.

De aceea unealta ruleaza DOUA moduri si le raporteaza pe amandoua:

  identic  B cu rafinare sub-pixel, ca A -> verificarea de cablaj (prag strict)
  fidel    B fara rafinare, ca scriptul original -> masoara diferenta reala
           de acuratete intre cele doua alegeri de algoritm

Verdictul se da pe modul `identic`. Modul `fidel` e informativ: pe date
sintetice, fara rafinare eroarea de distanta a fost 1.76% fata de 0.68% cu
rafinare, deci alegerea chiar conteaza.

Doua lucruri pe care unealta le trateaza explicit, ca sa nu raporteze
divergente false:

- **`fits_in_frame`.** Implementarea de bord NU publica o detectie daca
  markerul nu incape intreg in cadru (§5.2); cea independenta o publica, cu
  un indicator. Cand B raporteaza `fits_in_frame=0` iar A tace, ele sunt DE
  ACORD - se numara separat, nu ca divergenta.
- **ROI.** Implementarea de bord cauta intr-un ROI dupa o detectie sub 5 m,
  ceea ce introduce dependenta de ordinea imaginilor. Pentru comparatia
  algoritmilor ROI-ul e oprit implicit; `--roi` il porneste, ca sa se poata
  verifica si calea de productie.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import verify_detection as vd                             # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector,        # noqa: E402
                              CameraCalibration)

#: Praguri de acceptare.
MAX_CORNER_PX = 1.0
MAX_DISTANCE_REL = 0.01


def compare_one(a_det, a_corners, b_res):
    """Diferentele pentru o imagine, sau o eticheta de dezacord."""
    if a_det is None and b_res is None:
        return {'stare': 'ambele tac'}
    if a_det is None and b_res is not None:
        # Cazul asteptat: markerul nu incape intreg in cadru.
        if not b_res['fits_in_frame']:
            return {'stare': 'acord: nu incape in cadru'}
        return {'stare': 'DOAR B detecteaza', 'b': b_res}
    if a_det is not None and b_res is None:
        return {'stare': 'DOAR A detecteaza', 'a': a_det}

    d_corners = np.linalg.norm(a_corners - b_res['corners'], axis=1)
    d_dist = abs(a_det.distance_m - b_res['distance_m'])
    rel = d_dist / max(b_res['distance_m'], 1e-9)
    return {
        'stare': 'ambele',
        'corner_max': float(d_corners.max()),
        'corner_mean': float(d_corners.mean()),
        'dist_a': a_det.distance_m,
        'dist_b': b_res['distance_m'],
        'dist_rel': float(rel),
        'px_a': a_det.marker_px,
        'px_b': b_res['marker_px'],
    }


def run(images, calib_path, marker_id, marker_size_m, use_roi, refine,
        verbose=True, eticheta=''):
    # Fiecare implementare isi incarca singura calibrarea - asta verifica
    # incrucisat si citirea fisierului, nu doar detectia.
    cal_a = CameraCalibration.load(calib_path, require_real=True)
    K, dist, (w, h), info = vd.load_calibration(calib_path)
    if not np.allclose(cal_a.K, K) or not np.allclose(cal_a.dist[:len(dist)],
                                                      dist[:len(cal_a.dist)]):
        print("  ATENTIE: cele doua incarcari ale calibrarii difera")

    a = ArucoMarkerDetector(cal_a, marker_id=marker_id,
                            marker_size_m=marker_size_m,
                            roi_below_m=(5.0 if use_roi else 0.0))
    b = vd.Detector(K, dist, marker_size_m, marker_id, refine=refine)

    rows = []
    for path in images:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        a_det = a.detect(gray, 0.0)
        a_corners = a.last_corners
        b_res = b.detect(gray)
        r = compare_one(a_det, a_corners, b_res)
        r['file'] = os.path.basename(path)
        rows.append(r)
        if verbose and eticheta:
            if r['stare'] == 'ambele':
                print(f"  {r['file']:<32} colturi max {r['corner_max']:5.2f} "
                      f"medie {r['corner_mean']:5.2f} px | dist "
                      f"{r['dist_a']:6.3f} vs {r['dist_b']:6.3f} m "
                      f"({r['dist_rel']:+.2%})")
            else:
                print(f"  {r['file']:<32} {r['stare']}")
    return rows


def summarize(rows, max_corner=MAX_CORNER_PX, max_rel=MAX_DISTANCE_REL):
    ambele = [r for r in rows if r['stare'] == 'ambele']
    doar_a = [r for r in rows if r['stare'] == 'DOAR A detecteaza']
    doar_b = [r for r in rows if r['stare'] == 'DOAR B detecteaza']
    tac = [r for r in rows if r['stare'] == 'ambele tac']
    acord_incadrare = [r for r in rows if r['stare'].startswith('acord:')]

    print(f"\n  {len(rows)} imagini: {len(ambele)} detectate de ambele, "
          f"{len(tac)} de niciuna, {len(acord_incadrare)} acord pe "
          f"'nu incape in cadru'")
    if doar_a or doar_b:
        print(f"  DEZACORD: {len(doar_a)} doar A (bord), "
              f"{len(doar_b)} doar B (independent)")
        for r in (doar_a + doar_b)[:10]:
            print(f"    {r['file']}: {r['stare']}")

    if not ambele:
        print("  nicio imagine detectata de ambele - nimic de comparat")
        return False

    cmax = np.array([r['corner_max'] for r in ambele])
    cmean = np.array([r['corner_mean'] for r in ambele])
    rel = np.array([r['dist_rel'] for r in ambele])
    print(f"\n  colturi : max {cmax.max():.3f} px, mediu "
          f"{cmean.mean():.3f} px  (prag {max_corner} px)")
    print(f"  distanta: max {rel.max():.3%}, mediu {rel.mean():.3%}  "
          f"(prag {max_rel:.0%})")

    worst = ambele[int(cmax.argmax())]
    print(f"  cea mai mare diferenta pe colturi: {worst['file']} "
          f"({worst['corner_max']:.3f} px, dist {worst['dist_a']:.3f} vs "
          f"{worst['dist_b']:.3f} m)")

    ok = (cmax.max() <= max_corner and rel.max() <= max_rel
          and not doar_a and not doar_b)
    print(f"\n  VERDICT: {'DE ACORD' if ok else 'DIVERG - investigheaza'}")
    if not ok:
        print("  Peste praguri inseamna un bug intr-una dintre implementari, "
              "nu o toleranta de reglat.")
    return ok


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--images', required=True)
    p.add_argument('--calib', default=vd.DEFAULT_CALIB)
    p.add_argument('--marker-id', type=int, default=vd.TARGET_MARKER_ID)
    p.add_argument('--marker-size-m', type=float, default=vd.MARKER_SIZE_M)
    p.add_argument('--roi', action='store_true',
                   help='lasa ROI-ul activ in implementarea de bord')
    p.add_argument('--mod', choices=('ambele', 'identic', 'fidel'),
                   default='ambele',
                   help='identic = B cu rafinare (cablaj); fidel = B fara '
                        'rafinare (ca scriptul original)')
    p.add_argument('--max-corner-px', type=float, default=MAX_CORNER_PX)
    p.add_argument('--max-dist-rel', type=float, default=MAX_DISTANCE_REL)
    a = p.parse_args()

    try:
        images = vd.list_images(a.images)
    except FileNotFoundError as e:
        print(f"  {e}")
        return 1

    moduri = []
    if a.mod in ('ambele', 'identic'):
        moduri.append(('identic (B cu rafinare) - verificare de cablaj', True))
    if a.mod in ('ambele', 'fidel'):
        moduri.append(('fidel (B fara rafinare) - diferenta de algoritm', False))

    verdict = None
    for eticheta, refine in moduri:
        print(f"\n{'=' * 68}\n  MOD: {eticheta}\n{'=' * 68}")
        try:
            rows = run(images, a.calib, a.marker_id, a.marker_size_m, a.roi,
                       refine, eticheta=eticheta)
        except (FileNotFoundError, ValueError) as e:
            print(f"\n  NU RULEZ: {e}\n")
            return 2
        ok = summarize(rows, a.max_corner_px, a.max_dist_rel)
        if refine:
            verdict = ok          # verdictul se da pe modul de cablaj
        elif not ok:
            print("  (informativ: in modul fidel o diferenta peste prag e "
                  "asteptata - masoara rafinarea sub-pixel, nu un bug)")
    return 0 if (verdict is None or verdict) else 1


if __name__ == '__main__':
    sys.exit(main())
