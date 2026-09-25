#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Offline detection benchmark on recorded frames (step 0, §5.65).

    ~/nova-venv/bin/python tools/bench_detect.py --frames ~/nova-frames/1m
    ~/nova-venv/bin/python tools/bench_detect.py --frames DIR --variants

Runs, on the SAME frames, three pipelines and reports detection rate and
per-frame time for each:

  1. plain    cv2.aruco.ArucoDetector, default parameters, full frame -
              the "standalone script that works" baseline
  2. aruco    nova.detector_pi.ArucoMarkerDetector as configured for flight
              (ROI, reduced search, refinement, ID filter, fill guard)
  3. pi       the full PiDetector loop (source -> ring -> detect -> publish),
              with the per-stage timer table

--variants also runs pipeline 2 with search_downscale in {1, 2} and ROI
on/off, to see which knob costs what on this Pi.

Frames come from tools/record_frames.py (meta.json is optional).
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import config as nova_config                          # noqa: E402
from nova.detector_pi import (ARUCO_DICT, ArucoMarkerDetector,  # noqa: E402
                              CameraCalibration, ImageDirSource, PiDetector,
                              SEARCH_DOWNSCALE, ROI_BELOW_M)


def _pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * p
    lo, hi = int(k), int(np.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _make_detector(cal, cfg, **over):
    kw = dict(marker_id=cfg['marker_id'], marker_size_m=cfg['marker_size_m'],
              roi_below_m=cfg['roi_below_m'], roi_size_px=cfg['roi_size_px'],
              camera_rotation_deg=cfg['camera_rotation_deg'],
              search_downscale=cfg['search_downscale'])
    kw.update(over)
    return ArucoMarkerDetector(cal, **kw)


def bench_plain(frames, marker_id):
    """Pipeline 1: default ArucoDetector on the full frame."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
                                  cv2.aruco.DetectorParameters())
    hits, ms, sides = 0, [], []
    for g in frames:
        t0 = time.perf_counter()
        corners, ids, _ = det.detectMarkers(g)
        ms.append(1000.0 * (time.perf_counter() - t0))
        if ids is not None and marker_id in ids.flatten():
            hits += 1
            c = corners[list(ids.flatten()).index(marker_id)].reshape(4, 2)
            d = np.roll(c, -1, axis=0) - c
            sides.append(float(np.mean(np.linalg.norm(d, axis=1))))
    return {'rate': hits / len(frames), 'p50': _pct(ms, .5), 'p99': _pct(ms, .99),
            'side_px': _pct(sides, .5) if sides else None}


def bench_aruco(frames, cal, cfg, **over):
    """Pipeline 2: the production detector class, one frame at a time."""
    d = _make_detector(cal, cfg, **over)
    hits, ms, sides = 0, [], []
    for i, g in enumerate(frames):
        t0 = time.perf_counter()
        det = d.detect(g, float(i) / 30.0)
        ms.append(1000.0 * (time.perf_counter() - t0))
        if det is not None:
            hits += 1
            sides.append(det.marker_px)
    st = d.stats()
    return {'rate': hits / len(frames), 'p50': _pct(ms, .5), 'p99': _pct(ms, .99),
            'side_px': _pct(sides, .5) if sides else None,
            'roi': f"{st['roi_hits']}/{st['roi_misses']}",
            'half': st['half_hits'], 'full': st['full_hits'],
            'rejected_fit': st['rejected_fit']}


def bench_pi(frames_dir, cal, cfg):
    """Pipeline 3: the full PiDetector loop, unthreaded, on ImageDirSource."""
    src = ImageDirSource(frames_dir)
    pid = PiDetector(src, _make_detector(cal, cfg), threaded=False,
                     ring_frames=30)
    n = len(src.paths)
    for i in range(n):
        pid.poll(float(i) / 30.0)
    s = pid.stats()
    return {'rate': s['detection_rate'], 'lat_p50': s['latency_p50_ms'],
            'lat_p99': s['latency_p99_ms'], 'stages': pid.timer.table(),
            'stats': pid.timer.stats()}


def load_frames(frames_dir):
    src = ImageDirSource(frames_dir)
    frames = []
    while True:
        item = src.read()
        if item is None:
            break
        frames.append(item[0])
    meta = None
    mp = os.path.join(frames_dir, 'meta.json')
    if os.path.exists(mp):
        with open(mp) as f:
            meta = json.load(f)
    return frames, meta


def run_bench(frames_dir, cal, cfg, variants=False):
    frames, meta = load_frames(frames_dir)
    out = {'n': len(frames), 'meta': meta,
           'plain': bench_plain(frames, cfg['marker_id']),
           'aruco': bench_aruco(frames, cal, cfg),
           'pi': bench_pi(frames_dir, cal, cfg)}
    if variants:
        out['variants'] = {}
        for k in (1, 2):
            for roi in (0.0, ROI_BELOW_M):
                key = f"downscale={k} roi={'on' if roi else 'off'}"
                out['variants'][key] = bench_aruco(frames, cal, cfg,
                                                   search_downscale=k,
                                                   roi_below_m=roi)
    return out


def fmt(r):
    side = '-' if r.get('side_px') is None else f"{r['side_px']:.0f}"
    extra = ''
    if 'roi' in r:
        extra = (f"  roi {r['roi']}  half {r['half']}  full {r['full']}"
                 f"  rejected_fit {r['rejected_fit']}")
    return (f"det {100 * r['rate']:5.1f}%  p50 {r['p50']:6.1f} ms  "
            f"p99 {r['p99']:6.1f} ms  marker ~{side} px{extra}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--frames', required=True)
    p.add_argument('--config', default=None)
    p.add_argument('--variants', action='store_true')
    a = p.parse_args()

    cfg = nova_config.load(a.config)
    cal = CameraCalibration.load(nova_config.resolve(cfg, 'camera_calibration'),
                                 require_real=True)
    r = run_bench(a.frames, cal, cfg, variants=a.variants)
    print(f"frames: {r['n']} from {a.frames}")
    if r['meta']:
        m0 = (r['meta'].get('frames') or [{}])[0]
        print(f"camera: LensPosition={m0.get('LensPosition')} "
              f"ExposureTime={m0.get('ExposureTime')} "
              f"AnalogueGain={m0.get('AnalogueGain')} "
              f"fps_measured={r['meta'].get('fps_measured')}")
    print(f"config: rotation={cfg['camera_rotation_deg']} "
          f"marker_size_m={cfg['marker_size_m']} roi_below_m={cfg['roi_below_m']} "
          f"search_downscale={cfg['search_downscale']}")
    print()
    print(f"1. plain cv2.aruco   {fmt(r['plain'])}")
    print(f"2. ArucoMarkerDet.   {fmt(r['aruco'])}")
    pi = r['pi']
    print(f"3. PiDetector        det {100 * pi['rate']:5.1f}%  "
          f"lat p50 {pi['lat_p50'] or float('nan'):6.1f} ms  "
          f"p99 {pi['lat_p99'] or float('nan'):6.1f} ms")
    print()
    print(pi['stages'])
    if a.variants:
        print()
        for key, v in r['variants'].items():
            print(f"variant {key:<24} {fmt(v)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
