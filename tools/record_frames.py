#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Record raw camera frames on the Pi, for offline detection benchmarks
(step 0 of the Pi 4 detection work, §5.65).

    ~/nova-venv/bin/python tools/record_frames.py --seconds 30
    ~/nova-venv/bin/python tools/record_frames.py --seconds 30 --out ~/nova-frames/1m
    ~/nova-venv/bin/python tools/record_frames.py --seconds 30 --preset full1280

Uses the SAME camera setup as the flight app: camera_settings(cfg, preset)
(the config's `camera_preset` + camera keys, or exactly the preset given
with --preset) -> PiCameraSource. So the frames are what the detector sees.
Frames are kept in RAM during the recording (writing PNG at 30 fps on a
Pi 4 would throttle the camera and bias the measurement) and written as
lossless PNG afterwards, with a meta.json carrying:

  per frame    t (capture time, monotonic), ExposureTime, AnalogueGain,
               LensPosition, SensorTimestamp (always present, null if the
               camera did not report one) + FrameDuration, Lux,
               ColourTemperature when reported
  per session  sensor_mode and scaler_crop (they decide which calibration
               applies: calibration_for), size (the stream), preset, the
               full camera settings, and the config's marker_id,
               marker_size_m, camera_rotation_deg

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
from nova.detector_pi import (CAMERA_PRESETS, CameraSettings,  # noqa: E402
                              PiCameraSource, camera_settings)

#: Written for EVERY frame (null when the camera did not report it): the
#: exposure and focus the frame was taken with, and the sensor's own clock.
REQUIRED_META_KEYS = ('ExposureTime', 'AnalogueGain', 'LensPosition',
                      'SensorTimestamp')
#: Written when the camera reports them.
OPTIONAL_META_KEYS = ('FrameDuration', 'Lux', 'ColourTemperature')
META_KEYS = REQUIRED_META_KEYS + OPTIONAL_META_KEYS
#: Config keys the offline tools need to interpret the frames.
CONFIG_KEYS = ('marker_id', 'marker_size_m', 'camera_rotation_deg',
               'camera_preset')


def _jsonable(v):
    """Tuples (sizes, rectangles) as lists; anything else json can't take
    (numpy scalars) as float, else str."""
    if isinstance(v, (tuple, list)):
        return [_jsonable(x) for x in v]
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    try:
        return float(v)
    except (TypeError, ValueError):
        return str(v)


def frame_metadata(md):
    """The per-frame subset of the camera metadata: the required keys always
    (None if missing), the optional ones when present."""
    md = md or {}
    out = {k: _jsonable(md.get(k)) for k in REQUIRED_META_KEYS}
    out.update({k: _jsonable(md[k]) for k in OPTIONAL_META_KEYS if k in md})
    return out


def settings_dict(settings):
    """CameraSettings -> plain dict (every field), for meta.json."""
    return {k: _jsonable(getattr(settings, k)) for k in CameraSettings.FIELDS}


def session_meta(src, cfg, settings):
    """What is true for the whole recording: the geometry the frames were
    taken in, the settings that produced them, the marker they are for."""
    g = src.geometry
    return {'size': list(src.size),
            'sensor_mode': list(g.sensor_mode),
            'scaler_crop': list(g.scaler_crop),
            'preset': settings.preset,
            'settings': settings_dict(settings),
            'settings_origin': dict(settings.origin),
            'config': {k: _jsonable(cfg.get(k)) for k in CONFIG_KEYS}}


def mem_available_bytes():
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 1 << 30


def main(argv=None):
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
    p.add_argument('--preset', choices=sorted(CAMERA_PRESETS), default=None,
                   help='camera preset, exactly as the flight app takes it '
                        '(default: the config\'s camera_preset + camera keys)')
    p.add_argument('--format', choices=('png', 'jpg'), default='png',
                   help='png = lossless, ~0.15 s/frame on a Pi 4; '
                        'jpg (quality 97) ~10x faster, tiny lossy cost')
    p.add_argument('--max-frames', type=int, default=0,
                   help='stop the recording after N frames (0 = by time/RAM)')
    a = p.parse_args(argv)

    if a.stop_service:
        os.system('systemctl --user stop nova-bringup 2>/dev/null')
    cine = serial_guard.describe_camera_conflict()
    if cine:
        print(f"[record] camera is held by another process:\n  {cine}\n"
              f"  stop it (or pass --stop-service) and retry")
        return 3

    cfg = nova_config.load(a.config)
    # 27.09.2026: the flight's settings (sensor mode, stream, exposure mode,
    # AWB, focus), never the camera's own choice - the frames must be what
    # the detector sees. Same call as build_pi_detector.
    try:
        settings = camera_settings(cfg, a.preset)
    except ValueError as e:                    # CameraModeError: bad config
        print(f"[record] REFUZ: {e}")
        return 2
    print(f"[record] camera: {settings.describe()}")
    out = a.out or os.path.expanduser(
        time.strftime('~/nova-frames/%Y%m%d-%H%M%S'))
    os.makedirs(out, exist_ok=True)

    src = PiCameraSource(settings=settings, verbose=True,
                         fresh=cfg.get('camera_fresh_capture', True))
    budget = int(mem_available_bytes() * a.mem_frac)
    print(f"[record] {a.seconds:.0f} s into RAM (budget {budget / 1e6:.0f} MB), "
          f"then PNG into {out}")
    print("[record] move the drone by hand over the marker now")

    frames, used = [], 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.seconds:
            gray, t = src.read()
            frames.append((gray.copy(), t, frame_metadata(src.last_metadata)))
            used += gray.nbytes
            if a.max_frames and len(frames) >= a.max_frames:
                break
            if used >= budget:
                print(f"[record] memory budget reached after "
                      f"{len(frames)} frames; stopping early")
                break
    except KeyboardInterrupt:
        print(f"\n[record] interrupted: keeping {len(frames)} frames")
    finally:
        src.close()
    session = session_meta(src, cfg, settings)
    dur = frames[-1][1] - frames[0][1] if len(frames) > 1 else 0.0
    fps = (len(frames) - 1) / dur if dur > 0 else 0.0
    print(f"[record] {len(frames)} frames in {dur:.1f} s = {fps:.1f} fps "
          f"(camera nominal {src.nominal_fps})")

    # meta.json FIRST: writing 500+ PNGs takes over a minute on a Pi 4 and a
    # Ctrl-C in the middle must not lose the timestamps and camera metadata
    # of the frames that did get written.
    ext = '.png' if a.format == 'png' else '.jpg'
    flags = ([cv2.IMWRITE_PNG_COMPRESSION, 1] if a.format == 'png'
             else [cv2.IMWRITE_JPEG_QUALITY, 97])
    meta = [{'file': f"f{i:05d}{ext}", 't': t, **md}
            for i, (_, t, md) in enumerate(frames)]
    meta_path = os.path.join(out, 'meta.json')

    def write_meta(n_written):
        with open(meta_path, 'w') as f:
            json.dump({'frames': meta, 'written': n_written, 'fps_measured': fps,
                       'format': a.format, **session}, f, indent=1)
    write_meta(0)
    print(f"[record] writing {len(frames)} {a.format} files "
          f"(png ~0.15 s each on a Pi 4; --format jpg is ~10x faster)")
    n = 0
    try:
        for i, (gray, _, _) in enumerate(frames):
            cv2.imwrite(os.path.join(out, meta[i]['file']), gray, flags)
            n += 1
            if n % 50 == 0:
                print(f"[record]   {n}/{len(frames)}")
    except KeyboardInterrupt:
        print(f"\n[record] interrupted: {n} frames written, meta kept")
    write_meta(n)
    md0 = frames[0][2] if frames else {}
    print(f"[record] LensPosition={md0.get('LensPosition')} "
          f"ExposureTime={md0.get('ExposureTime')} us "
          f"AnalogueGain={md0.get('AnalogueGain')}")
    print(f"[record] written: {out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
