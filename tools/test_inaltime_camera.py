#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Inaltimea din camera (16.10.2026), fara FC si fara camera.

    python3 tools/test_inaltime_camera.py

CameraHeight learns bias = EKF height - real height from the camera's
perpendicular distance to the marker plane; the sequence (h_now, targets)
and the estimator's lateral scaling use it. Negatives: small marker, a
single outlier, before any detection (config fallback).
"""

import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import extnav as ex                               # noqa: E402
from nova.detection import CAM_HEIGHT_M, Detection          # noqa: E402
from nova.extnav_landing import ExtNavConfig, ExtNavLanding  # noqa: E402


def det(h_real, px=80.0, t=1.0, ax=0.0, ay=0.0):
    return Detection(t=t, angle_x=ax, angle_y=ay, distance_m=h_real,
                     marker_px=px, range_m=h_real,
                     plane_dist_m=h_real + CAM_HEIGHT_M)


def test_invata_corectia_si_respinge_aberantele():
    ch = ex.CameraHeight(fallback_bias=2.0)
    assert ch.height(7.5) == 5.5 and not ch.learned     # fallback first
    assert ch.update(det(5.0, px=20.0), 6.0) is None    # marker too small
    assert not ch.learned
    for _ in range(4):
        ch.update(det(5.0), 6.0)                         # baro +1 m
    assert ch.learned and abs(ch.bias - 1.0) < 1e-9, ch.bias
    assert abs(ch.height(6.0) - 5.0) < 1e-9
    assert ch.update(det(2.0), 6.0) is None              # 3 m jump: outlier
    assert abs(ch.bias - 1.0) < 1e-9 and ch.n_rejected == 1
    for _ in range(ex.CAM_HEIGHT_RESET_N):               # it persists: re-anchor
        ch.update(det(4.0), 6.0)
    assert abs(ch.bias - 2.0) < 1e-9, ch.bias
    return "fallback 2.0 -> invatat 1.0; o aberatie ignorata; persistenta -> re-ancorare"


def test_secventa_foloseste_corectia_invatata():
    v = types.SimpleNamespace(z=-6.0)
    ch = ex.CameraHeight(fallback_bias=0.0)
    ch.update(det(5.0), 6.0)
    sm = ExtNavLanding.__new__(ExtNavLanding)
    sm.v, sm.cfg, sm.est = v, ExtNavConfig(alt_offset_m=3.0), types.SimpleNamespace(height=ch)
    assert abs(sm.h_now() - 5.0) < 1e-9, sm.h_now()       # camera, not the 3.0
    assert abs(sm._z_of(1.25) - (-2.25)) < 1e-9, sm._z_of(1.25)
    sm.est = types.SimpleNamespace(height=None)          # switch off: config offset
    assert abs(sm.h_now() - 3.0) < 1e-9
    return "h_now = 5.0 din camera; tinta 1.25 m -> z -2.25 in cadrul EKF"


def test_estimatorul_scaleaza_cu_inaltimea_reala():
    class V:
        z = -6.0
        rel_alt = 6.0

        def attitude_at(self, t):
            return (0.0, 0.0, 0.0)

        def position_at(self, t):
            return (0.0, 0.0, -6.0)
    ax = 0.1                                             # rad, forward
    e_cam = ex.ExtNavEstimator(V(), height=ex.CameraHeight()).estimate(
        det(5.0, ax=ax))
    e_baro = ex.ExtNavEstimator(V()).estimate(det(5.0, ax=ax))
    assert abs(e_cam.h - 5.0) < 1e-9 and abs(e_baro.h - 6.0) < 1e-9
    assert e_cam.lateral_m < e_baro.lateral_m * 0.9, (e_cam.lateral_m, e_baro.lateral_m)
    return f"lateral {e_cam.lateral_m:.3f} m cu camera vs {e_baro.lateral_m:.3f} m cu baro (+1 m)"


TESTS = [
    ('invata corectia si respinge aberantele', test_invata_corectia_si_respinge_aberantele),
    ('secventa foloseste corectia invatata', test_secventa_foloseste_corectia_invatata),
    ('estimatorul scaleaza cu inaltimea reala', test_estimatorul_scaleaza_cu_inaltimea_reala),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            print(f"  OK    {name}   ({fn()})")
        except Exception as e:                          # noqa: BLE001
            fails += 1
            print(f"  ESEC  {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
