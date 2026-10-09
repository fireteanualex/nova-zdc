#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Reparatiile dupa zborul din 09.10.2026, fara FC si fara camera.

    python3 tools/test_reparatii_0910.py

1. The camera exposure is re-measured and re-locked on request (each AUX
   request), not only at boot - non-blocking, inside read().
2. Touchdown descent: once the camera is blind (no detection for
   TD_BLIND_S), the EKF gets the position frozen at that moment; the ramp
   goes TOUCHDOWN_BELOW_M (1.5 m) below the ground so the FC sees contact.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import detector_pi as dp                          # noqa: E402
from nova import extnav_landing as el                       # noqa: E402


class FakeCam:
    camera_controls = {'AnalogueGain': (1.0, 16.0, 1.0)}

    def __init__(self):
        self.calls = []

    def set_controls(self, c):
        self.calls.append(dict(c))


def _source():
    s = dp.PiCameraSource.__new__(dp.PiCameraSource)
    s.settings = type('S', (), {'exposure': 'auto_lock'})()
    s.picam2 = FakeCam()
    s.verbose = False
    s._relock_req = False
    s._relock_left = 0
    s._locked_ctrl = {'ExposureTime': 2000, 'AnalogueGain': 16.0,
                      'AeEnable': False, 'LensPosition': 1.63}
    return s


def test_expunerea_se_re_blocheaza_la_cerere():
    s = _source()
    assert s.request_exposure_relock() is True
    md = {'ExposureTime': 300, 'AnalogueGain': 1.2}
    s._relock_step(md)                       # starts: AE on
    assert s.picam2.calls == [{'AeEnable': True}], s.picam2.calls
    for _ in range(dp.AUTOEXP_FRAMES - 1):
        s._relock_step(md)
    assert len(s.picam2.calls) == 1, "blocat prea devreme"
    s._relock_step(md)                       # the last AE frame: lock
    last = s.picam2.calls[-1]
    assert last['AeEnable'] is False and last['ExposureTime'] == 300, last
    assert last['LensPosition'] == 1.63, "controalele de focus pierdute"
    assert s._relock_left == 0 and not s._relock_req
    # negative: 'fixed' exposure is never re-measured
    s2 = _source()
    s2.settings.exposure = 'fixed'
    assert s2.request_exposure_relock() is False and not s2._relock_req
    return f"AE {dp.AUTOEXP_FRAMES} cadre, apoi blocat la 300 us; 'fixed' neatins"


class V:
    def __init__(self):
        self.x, self.y, self.z = 0.03, -0.02, -0.4
        self.roll = self.pitch = self.yaw = 0.0
        self.vpe = []

    def send_vision_position_estimate(self, usec, x, y, z, r, p, yaw):
        self.vpe.append((x, y))
        return True


def _sm(v):
    sm = el.ExtNavLanding.__new__(el.ExtNavLanding)
    sm.v = v
    sm.last_det_t = None
    sm._td_blind_xy = None
    sm._ground_vpe_t = float('-inf')
    sm.n_ground_vpe = 0
    return sm


def test_coborarea_oarba_tine_EKF_viu():
    v = V()
    sm = _sm(v)
    sm.last_det_t = 10.0
    sm._blind_descent_vpe(10.2)              # detection 0.2 s old: not blind
    assert v.vpe == [] and sm._td_blind_xy is None
    sm._blind_descent_vpe(10.4)              # 0.4 s: blind, frozen here
    v.x, v.y = 0.5, 0.5                      # the EKF moves: frozen ignores it
    sm._blind_descent_vpe(10.45)             # < GROUND_VPE_S: nothing
    sm._blind_descent_vpe(10.55)
    assert v.vpe == [(0.03, -0.02), (0.03, -0.02)], v.vpe
    sm.last_det_t = 10.6                     # a new detection ends it
    sm._blind_descent_vpe(10.65)
    assert sm._td_blind_xy is None and len(v.vpe) == 2
    assert el.TOUCHDOWN_BELOW_M >= 1.5
    return f"orb dupa {el.TD_BLIND_S} s: VPE inghetat la 10 Hz; detectie noua -> oprit"


TESTS = [
    ('expunerea se re-blocheaza la cerere', test_expunerea_se_re_blocheaza_la_cerere),
    ('coborarea oarba tine EKF-ul viu', test_coborarea_oarba_tine_EKF_viu),
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
