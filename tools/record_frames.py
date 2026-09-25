#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Record raw camera frames on the Pi, for offline detection benchmarks
(step 0 of the Pi 4 detection work, §5.65).

    ~/nova-venv/bin/python tools/record_frames.py --seconds 30
    ~/nova-venv/bin/python tools/record_frames.py --seconds 30 --out ~/nova-frames/1m

Uses the SAME camera setup as the flight app (PiCameraSource: mode, size,
locked exposure, LensPosition), so the frames are what the detector sees.
Frames are kept in RAM during the recording (writing PNG at 30 fps on a
Pi 4 would throttle the camera and bias the measurement) and written as
lossless PNG afterwards, with a meta.json carrying per-frame timestamps and
the camera metadata (ExposureTime, AnalogueGain, LensPosition).

The camera is exclusive: stop the autostart first
(`systemctl --user stop nova-bringup`) or pass --stop-service.
"""

import argparse
import json
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import config as nova_config                          # noqa: E402
from nova import serial_guard                                   # noqa: E402
from nova.detector_pi import PiCameraSource                     # noqa: E402

META_KEYS = ('ExposureTime', 'AnalogueGain', 'LensPosition', 'SensorTimestamp',
             'FrameDuration', 'Lux', 'ColourTemperature')


def mem_available_bytes():
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 1 << 30


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--out', default=None,
                   help='output directory (default ~/nova-frames/<stamp>)')
    p.add_argument('--mem-frac', type=float, default=0.5,
                   help='fraction of MemAvailable the buffer may use')
    p.add_argument('--stop-service', action='store_true',
                   help='stop nova-bringup first (it holds the camera)')
    p.add_argument('--config', default=None)
    a = p.parse_args()

    if a.stop_service:
        os.system('systemctl --user stop nova-bringup 2>/dev/null')
    cine = serial_guard.describe_camera_conflict()
    if cine:
        print(f"[record] camera is held by another process:\n  {cine}\n"
              f"  stop it (or pass --stop-service) and retry")
        return 3

    cfg = nova_config.load(a.config)
    out = a.out or os.path.expanduser(
        time.strftime('~/nova-frames/%Y%m%d-%H%M%S'))
    os.makedirs(out, exist_ok=True)

    src = PiCameraSource(verbose=True,
                         auto_expose=cfg.get('camera_auto_expose', True))
    budget = int(mem_available_bytes() * a.mem_frac)
    print(f"[record] {a.seconds:.0f} s into RAM (budget {budget / 1e6:.0f} MB), "
          f"then PNG into {out}")
    print("[record] move the drone by hand over the marker now")

    frames, used = [], 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.seconds:
            gray, t = src.read()
            md = {k: src.last_metadata.get(k) for k in META_KEYS
                  if src.last_metadata and k in src.last_metadata}
            frames.append((gray.copy(), t, md))
            used += gray.nbytes
            if used >= budget:
                print(f"[record] memory budget reached after "
                      f"{len(frames)} frames; stopping early")
                break
    finally:
        src.close()
    dur = frames[-1][1] - frames[0][1] if len(frames) > 1 else 0.0
    fps = (len(frames) - 1) / dur if dur > 0 else 0.0
    print(f"[record] {len(frames)} frames in {dur:.1f} s = {fps:.1f} fps "
          f"(camera nominal {src.nominal_fps})")

    meta = []
    for i, (gray, t, md) in enumerate(frames):
        name = f"f{i:05d}.png"
        cv2.imwrite(os.path.join(out, name),
                    gray, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        meta.append({'file': name, 't': t, **md})
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump({'frames': meta, 'fps_measured': fps,
                   'size': list(src.size), 'config': {
                       k: cfg.get(k) for k in ('camera_rotation_deg',
                                               'camera_auto_expose',
                                               'marker_size_m')}}, f, indent=1)
    md0 = frames[0][2] if frames else {}
    print(f"[record] LensPosition={md0.get('LensPosition')} "
          f"ExposureTime={md0.get('ExposureTime')} us "
          f"AnalogueGain={md0.get('AnalogueGain')}")
    print(f"[record] written: {out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
