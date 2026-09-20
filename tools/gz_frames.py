#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verificarea sursei de cadre din Gazebo (I3).

    ~/nova-sim-venv/bin/python tools/gz_frames.py
    ~/nova-sim-venv/bin/python tools/gz_frames.py --save /tmp/cadre --n 10
    ~/nova-sim-venv/bin/python tools/gz_frames.py --detect

Ce raspunde, in ordinea in care conteaza:

  1. **Sosesc cadre?** Un senzor Gazebo nu randeaza fara abonat (§5.33), iar
     clasa asta e abonatul - deci daca nu sosesc nimic, problema e in lume
     sau in server, nu in noi.
  2. **Ceasul e cel de simulare?** Timpii trebuie sa creasca odata cu
     simularea, nu cu ceasul de perete. Unealta le tipareste pe amandoua si
     raporteaza raportul - care e chiar RTF-ul.
  3. **Se vede markerul?** Cu `--detect`, ruleaza detectorul real pe cadre.
     Asta inchide lantul randare -> detectie fara sa mai fie nevoie de bucla
     de control.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import config as nova_config                      # noqa: E402
from nova.detector_pi import GazeboFrameSource              # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/down_cam/image')
    p.add_argument('--n', type=int, default=30, help='cate cadre')
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--clock', choices=('sim', 'wall'), default='sim')
    p.add_argument('--save', default=None, help='director pentru PNG-uri')
    p.add_argument('--detect', action='store_true',
                   help='ruleaza detectorul real pe cadre')
    p.add_argument('--calib', default=None)
    a = p.parse_args(argv)

    try:
        src = GazeboFrameSource(a.topic, clock=a.clock, timeout_s=a.timeout)
    except (ImportError, RuntimeError) as e:
        print(f"\n  NU POT PORNI: {e}\n")
        return 2

    det = None
    if a.detect:
        from nova.detector_pi import (ArucoMarkerDetector,
                                      CameraCalibration)
        cfg = nova_config.load()
        calib = a.calib or nova_config.resolve(cfg, 'camera_calibration')
        try:
            cal = CameraCalibration.load(calib, require_real=False)
        except (FileNotFoundError, ValueError) as e:
            print(f"\n  --detect are nevoie de calibrare: {e}\n"
                  f"  Pentru simulare: --calib config/camera_sim.yaml\n")
            src.close()
            return 2
        det = ArucoMarkerDetector(cal, marker_id=cfg['marker_id'],
                                  marker_size_m=cfg['marker_size_m'],
                                  roi_below_m=0.0)
        print(f"  detector: {cal.width}x{cal.height} fx={cal.fx:.1f} "
              f"marker ID {cfg['marker_id']}")

    if a.save:
        os.makedirs(a.save, exist_ok=True)

    print(f"\n  astept cadre pe {a.topic} (timeout {a.timeout:g} s)...\n")
    t_perete0 = time.monotonic()
    t_sim0 = None
    n = 0
    n_det = 0
    try:
        while n < a.n:
            item = src.read()
            if item is None:
                print(f"\n  NICIUN CADRU in {a.timeout:g} s.")
                print(f"    Verifica: gz topic -l | grep {a.topic}")
                print("    Serverul ruleaza? Senzorul e in lume?")
                break
            gray, t = item
            if t_sim0 is None:
                t_sim0 = t
            n += 1
            linie = (f"  {n:>3}  {gray.shape[1]}x{gray.shape[0]}  "
                     f"t={t:.3f}  dt_sim={t - t_sim0:6.3f}  "
                     f"dt_perete={time.monotonic() - t_perete0:6.3f}")
            if det is not None:
                d = det.detect(gray, t)
                if d is not None:
                    n_det += 1
                    linie += (f"  | marker {d.marker_px:5.1f} px  "
                              f"{d.distance_m:5.2f} m")
                else:
                    linie += "  | fara marker"
            print(linie)
            if a.save:
                import cv2 as _cv
                _cv.imwrite(os.path.join(a.save, f"{n:04d}.png"), gray)
    except KeyboardInterrupt:
        print("\n  intrerupt")
    finally:
        st = src.stats()
        src.close()

    perete = time.monotonic() - t_perete0
    print(f"\n  {n} cadre, {st['dropped']} pierdute (coada de 1: pastram cel "
          f"mai nou)")
    if n >= 2 and t_sim0 is not None:
        sim = st['sim_t'] - t_sim0
        print(f"  timp simulat {sim:.2f} s in {perete:.2f} s de perete  ->  "
              f"RTF {sim / perete:.2f}")
        print(f"  rata cadrelor in simulare: {st['fps']:.1f} fps"
              if st['fps'] else "")
        if a.clock == 'sim' and sim <= 0:
            print("  ATENTIE: timpul de simulare nu creste - lumea e pe pauza?")
    if det is not None and n:
        print(f"  detectie: {n_det}/{n} ({n_det / n:.0%})")
    if a.save:
        print(f"  cadre scrise in {a.save}")
    print()
    return 0 if n else 1


if __name__ == '__main__':
    sys.exit(main())
