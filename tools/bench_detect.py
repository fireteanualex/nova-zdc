#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Offline detection benchmark on recorded frames (step 0, §5.65).

    ~/nova-venv/bin/python tools/bench_detect.py --frames ~/nova-frames/1m
    ~/nova-venv/bin/python tools/bench_detect.py --frames DIR --variants
    ~/nova-venv/bin/python tools/bench_detect.py --frames DIR --rescale 960 540
    ~/nova-venv/bin/python tools/bench_detect.py --frames DIR \\
        --sensor-mode 1536 864 --scaler-crop 768 432 3072 1728

Runs, on the SAME frames, three pipelines and reports, for each, the
detection rate of the config's marker (`marker_id`), the median marker side
in pixels and the per-frame time:

  1. plain    cv2.aruco.ArucoDetector, default parameters, full frame -
              the "standalone script that works" baseline
  2. aruco    nova.detector_pi.ArucoMarkerDetector as configured for flight
              (ROI, reduced search, refinement, ID filter, fill guard)
  3. pi       the full PiDetector loop (source -> ring -> detect -> publish),
              with the per-stage timer table

--variants also runs pipeline 2 with search_downscale in {1, 2} and ROI
on/off, to see which knob costs what on this Pi.

--rescale W H runs the same frames scaled (INTER_AREA) to W x H - what the
ISP would deliver for the same sensor mode and ScalerCrop at another stream
size - with the calibration scaled to match, next to the original. Only a
pure scale is meaningful: another aspect ratio would be a crop, refused.

The calibration MUST match the frames' geometry (27.09.2026: a calibration
for another sensor mode gives distances silently off by 1.5x). The geometry
comes from the meta.json tools/record_frames.py writes (sensor_mode,
scaler_crop, size), or from --sensor-mode / --scaler-crop; without either
the bench refuses. The calibration file is then used only through
calibration_for(), and the kind (nativa / scalata / derivata / ...) and fx
used are printed.
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
from nova.detector_pi import (ARUCO_DICT, ArraySource,          # noqa: E402
                              ArucoMarkerDetector, CalibrationMismatch,
                              CameraCalibration, CameraGeometry,
                              ImageDirSource, PiDetector, ROI_BELOW_M,
                              calibration_for, parse_rect, parse_size)

#: Offline frames get synthetic capture times at this rate.
BENCH_FPS = 30.0


class BenchRefused(ValueError):
    """The bench cannot give honest numbers for these frames (unknown
    geometry, calibration that does not fit, rescale that is a crop)."""


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
    hits, ms, sides, dists = 0, [], [], []
    for i, g in enumerate(frames):
        t0 = time.perf_counter()
        det = d.detect(g, float(i) / BENCH_FPS)
        ms.append(1000.0 * (time.perf_counter() - t0))
        if det is not None:
            hits += 1
            sides.append(det.marker_px)
            dists.append(det.distance_m)
    st = d.stats()
    # The median distance makes a calibration of the wrong geometry visible
    # (1536x864 frames with a 2304x1296-scaled calibration: 1.5x too short).
    return {'rate': hits / len(frames), 'p50': _pct(ms, .5), 'p99': _pct(ms, .99),
            'side_px': _pct(sides, .5) if sides else None,
            'dist_m': _pct(dists, .5) if dists else None,
            'roi': f"{st['roi_hits']}/{st['roi_misses']}",
            'half': st['half_hits'], 'full': st['full_hits'],
            'rejected_fit': st['rejected_fit']}


def bench_pi(frames, cal, cfg):
    """Pipeline 3: the full PiDetector loop, unthreaded. `frames` is a
    directory (ImageDirSource: 'achizitie' = image decode) or a list of
    gray frames already in memory (ArraySource: no decode stage)."""
    if isinstance(frames, str):
        src = ImageDirSource(frames)
        n = len(src.paths)
    else:
        n = len(frames)
        src = ArraySource(frames, timestamps=[i / BENCH_FPS for i in range(n)],
                          fps=BENCH_FPS)
    # Offline there is no capture clock: the sources stamp frames with
    # synthetic times, so PiDetector's capture->publish latency would be
    # "now minus a synthetic stamp" = uptime (the §5.43 clock mix, seen as
    # lat p50 448938 ms on the first Pi run). Latency is a flight number;
    # offline the honest figures are the per-stage times below.
    pid = PiDetector(src, _make_detector(cal, cfg), threaded=False,
                     ring_frames=30, clock=lambda: 0.0)
    sides = []
    for i in range(n):
        sides.extend(d.marker_px for d in pid.poll(float(i) / BENCH_FPS))
    s = pid.stats()
    st = pid.timer.stats()
    tot = st.get('total')
    return {'rate': s['detection_rate'], 'stages': pid.timer.table(),
            'stats': st,
            'p50': tot[0] if tot else float('nan'),
            'p99': tot[1] if tot else float('nan'),
            'side_px': _pct(sides, .5) if sides else None}


def load_meta(frames_dir):
    mp = os.path.join(frames_dir, 'meta.json')
    if not os.path.exists(mp):
        return None
    with open(mp) as f:
        return json.load(f)


def load_frames(frames_dir):
    src = ImageDirSource(frames_dir)
    frames = []
    while True:
        item = src.read()
        if item is None:
            break
        frames.append(item[0])
    return frames, load_meta(frames_dir)


def first_frame_size(frames_dir):
    """(w, h) of the first image of the directory, without loading them all."""
    item = ImageDirSource(frames_dir).read()
    if item is None:
        raise BenchRefused(f"nicio imagine lizibila in {frames_dir}")
    return item[0].shape[1], item[0].shape[0]


def frames_geometry(meta, frame_size, sensor_mode=None, scaler_crop=None):
    """(CameraGeometry, where it came from) of a recorded set.

    Sensor mode and ScalerCrop: the command line, else meta.json. Neither
    is a refusal: a calibration applied to frames of another geometry gives
    distances and angles that are wrong without any sign of it (the
    1536x864 / 2304x1296 mix-up of 27.09.2026). The stream size is the
    frames' own; meta.json's `size`, when present, must agree with it."""
    meta = meta or {}
    mode = sensor_mode if sensor_mode is not None else meta.get('sensor_mode')
    crop = scaler_crop if scaler_crop is not None else meta.get('scaler_crop')
    if mode is None or crop is None:
        raise BenchRefused(
            "nu stiu in ce geometrie sunt cadrele (meta.json fara "
            "sensor_mode / scaler_crop): da --sensor-mode W H si "
            "--scaler-crop X Y W H. O calibrare pentru alt mod da distante "
            "gresite fara niciun semn (27.09.2026).")
    size = tuple(frame_size)
    if meta.get('size') is not None and tuple(parse_size(meta['size'])) != size:
        raise BenchRefused(
            f"meta.json spune {meta['size']} dar cadrele sunt "
            f"{size[0]}x{size[1]}: alt director sau cadre modificate")
    who = ('linia de comanda' if sensor_mode is not None or scaler_crop is not None
           else 'meta.json')
    return CameraGeometry(parse_size(mode), parse_rect(crop), size), who


def calibration_info(cal):
    return {'kind': cal.kind, 'fx': cal.fx, 'fy': cal.fy, 'cx': cal.cx,
            'cy': cal.cy, 'size': (cal.width, cal.height),
            'source': cal.source}


def _run_pipelines(frames, frames_or_dir, cal, cfg):
    return {'plain': bench_plain(frames, cfg['marker_id']),
            'aruco': bench_aruco(frames, cal, cfg),
            'pi': bench_pi(frames_or_dir, cal, cfg)}


def run_bench(frames_dir, cal, cfg, variants=False, geometry=None,
              rescale=None):
    """The numbers for one directory of frames.

    `geometry` (CameraGeometry of the frames): `cal` is the calibration FILE
    and the one used is calibration_for(cal, geometry) - what main() does.
    Without it, `cal` is used as given: the caller vouches that it is
    already the calibration of these frames (tests with a synthetic one).
    Either way its size must be the frames' size.

    `rescale` (W, H): the same frames scaled to W x H (INTER_AREA), with
    calibration_for(cal, same mode and ScalerCrop at W x H) - the ISP-scale
    case. Needs `geometry`. Results under out['rescaled']."""
    frames, meta = load_frames(frames_dir)
    size = (frames[0].shape[1], frames[0].shape[0])
    if geometry is not None:
        try:
            used = calibration_for(cal, geometry)
        except CalibrationMismatch as e:
            raise BenchRefused(f"calibrarea nu se potriveste cadrelor: {e}") from e
    else:
        used = cal
    if (used.width, used.height) != size:
        raise BenchRefused(
            f"calibrarea e pentru {used.width}x{used.height}, cadrele sunt "
            f"{size[0]}x{size[1]}")
    rescaled = None
    if rescale is not None:
        rw, rh = parse_size(rescale)
        if geometry is None:
            raise BenchRefused("--rescale cere geometria cadrelor (mod + "
                               "ScalerCrop) ca sa scaleze calibrarea")
        rgeom = CameraGeometry(geometry.sensor_mode, geometry.scaler_crop,
                               (rw, rh))
        try:
            rcal = calibration_for(cal, rgeom)
        except CalibrationMismatch as e:
            raise BenchRefused(
                f"--rescale {rw}x{rh}: nu e o scalare a cadrelor de "
                f"{size[0]}x{size[1]} (alt raport de aspect = decupaj, "
                f"alta geometrie, nu aceeasi imagine mai mica). {e}") from e
        rescaled = (rgeom, rcal)

    out = {'n': len(frames), 'meta': meta, 'size': size, 'geometry': geometry,
           'calibration': calibration_info(used)}
    out.update(_run_pipelines(frames, frames_dir, used, cfg))
    if variants:
        out['variants'] = {}
        for k in (1, 2):
            for roi in (0.0, ROI_BELOW_M):
                key = f"downscale={k} roi={'on' if roi else 'off'}"
                out['variants'][key] = bench_aruco(frames, used, cfg,
                                                   search_downscale=k,
                                                   roi_below_m=roi)
    if rescaled is not None:
        rgeom, rcal = rescaled
        small = [cv2.resize(g, rgeom.output_size, interpolation=cv2.INTER_AREA)
                 for g in frames]
        r = {'size': rgeom.output_size, 'geometry': rgeom,
             'calibration': calibration_info(rcal)}
        r.update(_run_pipelines(small, small, rcal, cfg))
        out['rescaled'] = r
    return out


def fmt(r):
    side = '-' if r.get('side_px') is None else f"{r['side_px']:.0f}"
    extra = ''
    if r.get('dist_m') is not None:
        extra += f"  dist ~{r['dist_m']:.2f} m"
    if 'roi' in r:
        extra += (f"  roi {r['roi']}  half {r['half']}  full {r['full']}"
                 f"  rejected_fit {r['rejected_fit']}")
    return (f"det {100 * r['rate']:5.1f}%  p50 {r['p50']:6.1f} ms  "
            f"p99 {r['p99']:6.1f} ms  marker ~{side} px{extra}")


def fmt_cal(c):
    return (f"calibrare {c['kind']} {c['size'][0]}x{c['size'][1]}: "
            f"fx={c['fx']:.1f} fy={c['fy']:.1f} cx={c['cx']:.1f} "
            f"cy={c['cy']:.1f}")


def print_pipelines(r, title):
    print(title)
    print(f"  1. plain cv2.aruco   {fmt(r['plain'])}")
    print(f"  2. ArucoMarkerDet.   {fmt(r['aruco'])}")
    pi = r['pi']
    side = '-' if pi.get('side_px') is None else f"{pi['side_px']:.0f}"
    print(f"  3. PiDetector        det {100 * (pi['rate'] or 0):5.1f}%  "
          f"total/frame p50 {pi['p50']:6.1f} ms  p99 {pi['p99']:6.1f} ms  "
          f"marker ~{side} px")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--frames', required=True)
    p.add_argument('--config', default=None)
    p.add_argument('--variants', action='store_true')
    p.add_argument('--sensor-mode', type=int, nargs=2, metavar=('W', 'H'),
                   default=None, help='the sensor mode the frames were taken '
                   'in (default: meta.json)')
    p.add_argument('--scaler-crop', type=int, nargs=4,
                   metavar=('X', 'Y', 'W', 'H'), default=None,
                   help='the ScalerCrop of the frames (default: meta.json)')
    p.add_argument('--rescale', type=int, nargs=2, metavar=('W', 'H'),
                   default=None, help='also run the frames scaled to W x H '
                   '(same aspect), with the calibration scaled to match')
    a = p.parse_args(argv)

    cfg = nova_config.load(a.config)
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    try:
        cal = CameraCalibration.load(cal_path, require_real=True)
        geom, who = frames_geometry(load_meta(a.frames),
                                    first_frame_size(a.frames),
                                    a.sensor_mode, a.scaler_crop)
        print(f"frames: {a.frames}")
        print(f"geometrie ({who}): {geom.describe()}")
        r = run_bench(a.frames, cal, cfg, variants=a.variants, geometry=geom,
                      rescale=a.rescale)
    except (BenchRefused, FileNotFoundError, ValueError) as e:
        print(f"\nREFUZ: {e}\n")
        return 2
    print(f"{fmt_cal(r['calibration'])}  (din {cal_path})")
    print(f"frames: {r['n']} x {r['size'][0]}x{r['size'][1]}")
    if r['meta']:
        m = r['meta']
        m0 = (m.get('frames') or [{}])[0]
        print(f"camera: preset={m.get('preset')} "
              f"LensPosition={m0.get('LensPosition')} "
              f"ExposureTime={m0.get('ExposureTime')} "
              f"AnalogueGain={m0.get('AnalogueGain')} "
              f"fps_measured={m.get('fps_measured')}")
        rec = m.get('config') or {}
        for k in ('marker_id', 'marker_size_m', 'camera_rotation_deg'):
            if rec.get(k) is not None and rec.get(k) != cfg.get(k):
                print(f"ATENTIE: cadrele au fost inregistrate cu {k}="
                      f"{rec[k]}, config-ul de acum are {cfg.get(k)}")
    print(f"config: marker_id={cfg['marker_id']} "
          f"rotation={cfg['camera_rotation_deg']} "
          f"marker_size_m={cfg['marker_size_m']} roi_below_m={cfg['roi_below_m']} "
          f"search_downscale={cfg['search_downscale']}")
    print()
    print_pipelines(r, f"{r['size'][0]}x{r['size'][1]} (cadrele inregistrate)")
    rr = r.get('rescaled')
    if rr:
        print()
        print_pipelines(rr, f"{rr['size'][0]}x{rr['size'][1]} (scalate "
                            f"INTER_AREA, {fmt_cal(rr['calibration'])})")
    print("\n  (latency is a flight number: see [etape] in the flight log)")
    print()
    print("per-stage (offline: 'achizitie' = image decode, not the camera)")
    print(r['pi']['stages'])
    if a.variants:
        print()
        for key, v in r['variants'].items():
            print(f"variant {key:<24} {fmt(v)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
