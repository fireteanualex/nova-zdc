#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Chiar se vede markerul de la handover? Masurat, nu estimat (§5.42).

    ~/nova-sim-venv/bin/python tools/check_handover_fov.py
    ~/nova-sim-venv/bin/python tools/check_handover_fov.py --calib config/camera_pi.yaml

Randeaza markerul la pozele pe care le planifica `batch_sim.py` si pune
detectorul REAL pe imagine. Raspunde la doua intrebari pe care altfel le-ai
estima: incape markerul in cadru, si il vede detectorul.

Markerul e deplasat pe axa SCURTA a cadrului - cazul cel mai strans, fiindca
VFOV e mai ingust decat HFOV si azimutul din campanie e aleator.

CE NU SPUNE: randarea nu are blur de miscare, zgomot de senzor sau
iluminare reala (§5.20). Raspunsul e ferm pentru GEOMETRIE; rata de detectie
in conditii reale se masoara la E2.
"""
import argparse
import os
import sys
import math

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'tools'))
import numpy as np, cv2
import synthetic
import batch_sim
from nova.detector_pi import ArucoMarkerDetector, CameraCalibration

_p = argparse.ArgumentParser(description=__doc__,
                            formatter_class=argparse.RawDescriptionHelpFormatter)
_p.add_argument('--calib', default=os.path.join(REPO, 'config',
                                                'camera_sim.yaml'))
_a = _p.parse_args()

cal = CameraCalibration.load(_a.calib, require_real=False)
K, dist, (W, H) = cal.K, cal.dist, (cal.width, cal.height)
det = ArucoMarkerDetector(cal, marker_id=26, marker_size_m=0.48,
                          roi_below_m=0.0)

def incearca(alt, lateral):
    """Camera nadir la `alt`; marker deplasat `lateral` pe axa SCURTA."""
    t = (0.0, lateral, alt)          # x dreapta, y jos (axa scurta), z optic
    frame, colturi = synthetic.render_marker(
        K, dist, (W, H), synthetic.R_MARKER_FLAT, t)
    d = det.detect(frame, 0.0)
    # unde cade centrul markerului in cadru, si cat de mare e
    c = colturi.reshape(-1, 2)
    cx, cy = c[:, 0].mean(), c[:, 1].mean()
    latura = max(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4))
    in_cadru = (c[:, 0].min() >= 0 and c[:, 0].max() < W
                and c[:, 1].min() >= 0 and c[:, 1].max() < H)
    err = None if d is None else (d.range_m - alt) / alt * 100.0
    return dict(alt=alt, lat=lateral, px=latura, cx=cx, cy=cy,
                in_cadru=in_cadru, detectat=d is not None, err=err,
                marja_y=min(c[:, 1].min(), H - 1 - c[:, 1].max()))

print(f"  calibrare: {W}x{H} fx={K[0,0]:.0f} fy={K[1,1]:.0f}")
print(f"  jumatate VFOV geometric: "
      f"{math.degrees(math.atan(H / 2 / K[1,1])):.1f} deg\n")
print(f"  {'alt':>5} {'lateral':>8} {'unghi':>7} {'px':>6} {'marja_y':>8} "
      f"{'in cadru':>9} {'detectat':>9} {'err range':>10}")
def altitudine_minima(lat_m, tilt_deg, vfov_deg):
    """Sub ce altitudine markerul nu mai incape INTREG in cadru.

    §5.2 da 0.38 m, dar aia e cifra de NADIR, cu eroare laterala zero.
    Bugetul real are trei termeni:

        h * tan(VFOV/2)  >=  lateral  +  h * tan(inclinare)  +  0.24

    Al doilea termen conteaza pentru ca la coborare vehiculul se inclina
    tocmai ca sa corecteze lateral - deci exact cand eroarea e mare, si
    cadrul se muta in directia gresita."""
    k = math.tan(math.radians(vfov_deg / 2.0))
    den = k - math.tan(math.radians(tilt_deg))
    return None if den <= 0 else (lat_m + MARKER_HALF_M) / den


def tabel_prag_jos(vfov_deg):
    print(f"\n  --- altitudinea MINIMA la care markerul mai incape intreg ---")
    print(f"  h * tan({vfov_deg / 2:.1f} deg) >= lateral + h*tan(inclinare) "
          f"+ {MARKER_HALF_M:.2f} m\n")
    tilturi = (0, 5, 10, 15, 20)
    print("  " + f"{'lateral':>9} " + ' '.join(f"{t:>6} deg" for t in tilturi))
    for lat in (0.0, 0.05, 0.10, 0.20, 0.30):
        rand = []
        for t in tilturi:
            h = altitudine_minima(lat, t, vfov_deg)
            rand.append('     -  ' if h is None else f"{h:6.2f} m")
        print(f"  {lat * 100:7.0f} cm " + ' '.join(rand))
    print("\n  Cifra din §5.2 (0.38 m) e coltul din stanga sus: nadir, eroare")
    print("  zero. Cu 20 cm lateral si 15 deg inclinare pragul urca la ~1 m.")


MARKER_HALF_M = 0.24

for alt in (5.0, 8.0, 10.9, 12.0):
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        # limita din batch_sim.raza_max
        rmax = batch_sim.raza_max(alt)
        lat = rmax * frac
        r = incearca(alt, lat)
        unghi = math.degrees(math.atan2(lat, alt))
        e = '-' if r['err'] is None else f"{r['err']:+.2f}%"
        print(f"  {alt:5.1f} {lat:8.2f} {unghi:6.1f}d {r['px']:6.0f} "
              f"{r['marja_y']:8.0f} {str(r['in_cadru']):>9} "
              f"{str(r['detectat']):>9} {e:>10}")
    print()

tabel_prag_jos(math.degrees(2 * math.atan(H / 2 / K[1, 1])))
