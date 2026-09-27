#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Uneltele de masurare ale camerei (punctul 5, 27.09.2026): record_frames,
bench_detect, calibrate_camera - aceleasi chei si preseturi ca zborul,
geometria inregistrata si respectata.

    python3 tools/test_camera_tools.py

Ce conteaza:
  - record_frames scrie in meta.json modul senzorului, ScalerCrop-ul,
    presetul, fluxul si toate setarile, iar la fiecare cadru ExposureTime,
    AnalogueGain, LensPosition, SensorTimestamp si t;
  - bench_detect foloseste calibrarea DOAR prin calibration_for pe
    geometria cadrelor, refuza fara geometrie si refuza o "scalare" care
    schimba raportul de aspect;
  - calibrate_camera --live deschide camera in modul presetului, la
    dimensiunea NATIVA a modului, si scrie presetul si setarile in calibrare.

picamera2 nu exista pe desktop: camera e stub-ul din test_camera_geometry.
"""

import contextlib
import io
import json
import os
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import config as nova_config                          # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector, CameraCalibration,  # noqa: E402
                              CameraGeometry, CameraSettings, IMX708_MODES,
                              calibration_for)
import bench_detect as bd                                       # noqa: E402
import calibrate_camera as cc                                   # noqa: E402
import record_frames as rf                                      # noqa: E402
import test_detector_pi as tdp                                  # noqa: E402
from test_camera_geometry import (CROP, FULL, fake_camera,      # noqa: E402
                                  quiet, real_like_calibration)

MARKER_M = 0.48


def write_config(d, **extra):
    """A config file of our own: the tests must not depend on what
    config/nova.json says today."""
    cfg = {'marker_id': 26, 'marker_size_m': MARKER_M, 'camera_rotation_deg': 0,
           'search_downscale': 1, 'camera_preset': 'crop1280',
           'camera_fresh_capture': False}
    cfg.update(extra)
    path = os.path.join(d, 'nova.json')
    with open(path, 'w') as f:
        json.dump(cfg, f)
    return path


@contextlib.contextmanager
def no_camera_conflict():
    """record_frames asks systemd / fuser who holds the camera; on the
    desktop the answer must not depend on the machine."""
    old = rf.serial_guard.describe_camera_conflict
    rf.serial_guard.describe_camera_conflict = lambda *a, **k: None
    try:
        yield
    finally:
        rf.serial_guard.describe_camera_conflict = old


def captured():
    buf = io.StringIO()
    return buf, contextlib.redirect_stdout(buf)


# --- record_frames ----------------------------------------------------------------

def record(tmp, *args):
    out = os.path.join(tmp, 'cadre')
    with fake_camera(), quiet(), no_camera_conflict():
        rc = rf.main(['--out', out, '--max-frames', '3', '--seconds', '5',
                      '--config', write_config(tmp)] + list(args))
    assert rc == 0, rc
    with open(os.path.join(out, 'meta.json')) as f:
        return out, json.load(f)


def test_record_frames_geometria_presetul_si_metadatele_per_cadru():
    """crop1280 si full1280 (--preset), plus presetul config-ului fara
    --preset: acelasi mod, acelasi ScalerCrop si acelasi flux ca zborul,
    scrise o data pe sesiune; metadatele camerei la fiecare cadru."""
    asteptat = {
        'crop1280': ([1536, 864], list(CROP), [1280, 720]),
        'full1280': ([2304, 1296], list(FULL), [1280, 720]),
        'crop1536': ([1536, 864], list(CROP), [1536, 864]),
    }
    note = []
    for preset, args in (('crop1280', ['--preset', 'crop1280']),
                         ('full1280', ['--preset', 'full1280']),
                         ('crop1536', [])):
        tmp = tempfile.mkdtemp()
        if not args:
            # no --preset: the config's camera_preset is what runs
            write_config(tmp, camera_preset='crop1536')
            out = os.path.join(tmp, 'cadre')
            with fake_camera(), quiet(), no_camera_conflict():
                rc = rf.main(['--out', out, '--max-frames', '3',
                              '--config', os.path.join(tmp, 'nova.json')])
            assert rc == 0, rc
            with open(os.path.join(out, 'meta.json')) as f:
                meta = json.load(f)
        else:
            out, meta = record(tmp, *args)
        mode, crop, size = asteptat[preset]
        assert meta['preset'] == preset, (preset, meta['preset'])
        assert meta['sensor_mode'] == mode, (preset, meta['sensor_mode'])
        assert meta['scaler_crop'] == crop, (preset, meta['scaler_crop'])
        assert meta['size'] == size, (preset, meta['size'])
        s = meta['settings']
        assert set(s) == set(CameraSettings.FIELDS), sorted(s)
        assert s['sensor_mode'] == mode and s['output_size'] == size, s
        assert s['exposure'] == 'auto_lock' and s['lens_position'] == 1.63, s
        assert meta['config']['marker_id'] == 26, meta['config']
        assert meta['config']['marker_size_m'] == MARKER_M
        assert meta['config']['camera_rotation_deg'] == 0
        assert meta['written'] == 3 and len(meta['frames']) == 3, meta['written']
        for fr in meta['frames']:
            for k in ('t', 'ExposureTime', 'AnalogueGain', 'LensPosition',
                      'SensorTimestamp'):
                assert fr.get(k) is not None, (preset, k, fr)
            assert fr['LensPosition'] == 1.63, fr
            img = cv2.imread(os.path.join(out, fr['file']), cv2.IMREAD_GRAYSCALE)
            assert img is not None and img.shape == (size[1], size[0]), fr['file']
        ts = [fr['t'] for fr in meta['frames']]
        assert ts == sorted(ts), ts
        note.append(f"{preset} {mode[0]}x{mode[1]}->{size[0]}x{size[1]}")
    return '; '.join(note)


def test_record_frames_cheile_obligatorii_exista_si_fara_raport():
    """O camera care nu raporteaza o cheie: cheia apare cu null (nu lipseste
    tacut); cheile optionale apar doar cand exista."""
    md = rf.frame_metadata({'ExposureTime': 1500, 'Lux': 300.0,
                            'ScalerCrop': (1, 2, 3, 4)})
    assert md == {'ExposureTime': 1500, 'AnalogueGain': None,
                  'LensPosition': None, 'SensorTimestamp': None,
                  'Lux': 300.0}, md
    assert rf.frame_metadata(None)['LensPosition'] is None
    json.dumps(rf.frame_metadata({'ExposureTime': np.int64(7)}))
    return "cheile obligatorii mereu, null daca lipsesc"


def test_record_frames_preset_inexistent_refuzat():
    tmp = tempfile.mkdtemp()
    with fake_camera(), quiet(), no_camera_conflict(), \
            contextlib.redirect_stderr(io.StringIO()):
        try:
            rf.main(['--out', tmp, '--preset', 'crop9999',
                     '--config', write_config(tmp)])
            assert False, "preset inexistent acceptat"
        except SystemExit as e:
            assert e.code == 2
        # a config whose preset does not exist: clean refusal, camera untouched
        rc = rf.main(['--out', tmp, '--config',
                      write_config(tmp, camera_preset='nu_exista')])
        from test_camera_geometry import FakePicamera2
        assert rc == 2 and not FakePicamera2.instances, rc
    return "argparse refuza --preset necunoscut; config cu preset gresit -> 2"


# --- bench_detect ------------------------------------------------------------------

#: The flight geometry: 1536x864 crop mode, ISP-scaled to 1280x720.
GEOM = CameraGeometry((1536, 864), CROP, (1280, 720))
#: Marker positions (camera frame, m): 4 frames with the marker, 2 without.
POSES = [(0.1, 0.1, 5.0), None, (-0.3, 0.2, 5.0), (0.2, -0.2, 5.0), None,
         (0.0, 0.0, 5.0)]


def render_at(cal, t, size):
    """tdp.render at any frame size (tdp.render draws at 2304x1296)."""
    marker = cv2.aruco.generateImageMarker(tdp.DICT, 26, 200)
    q = 50
    canvas = np.full((300, 300), 255, np.uint8)
    canvas[q:q + 200, q:q + 200] = marker
    src = np.array([[q, q], [q + 200, q], [q + 200, q + 200], [q, q + 200]],
                   np.float32)
    dst = tdp.project(cal, tdp.R_FLAT, t, tdp.obj_corners(MARKER_M)).astype(np.float32)
    Hm = cv2.getPerspectiveTransform(src, dst)
    frame = cv2.warpPerspective(canvas, Hm, tuple(size), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=110)
    return frame, dst


def bench_fixture(with_geometry=True):
    """(dir, cal file, config path, rendered side px, true distance m):
    frames rendered through the calibration of the flight geometry, as
    record_frames would have written them, and the vehicle-like FILE
    calibration (2304x1296 full field) the bench must derive from."""
    tmp = tempfile.mkdtemp()
    d = os.path.join(tmp, 'cadre')
    os.makedirs(d)
    cal_file = real_like_calibration()
    cal_path = os.path.join(tmp, 'camera_pi.yaml')
    cal_file.save(cal_path)
    used = calibration_for(cal_file, GEOM)
    sides, dists = [], []
    for i, t in enumerate(POSES):
        if t is None:
            frame = np.full((720, 1280), 110, np.uint8)
        else:
            frame, c = render_at(used, t, GEOM.output_size)
            sides.append(ArucoMarkerDetector.side_px(c))
            dists.append(tdp.truth(t)['distance'])
        cv2.imwrite(os.path.join(d, f"f{i:05d}.png"), frame)
    meta = {'frames': [{'LensPosition': 1.63}], 'fps_measured': 30.0,
            'preset': 'crop1280',
            'config': {'marker_id': 26, 'marker_size_m': MARKER_M}}
    if with_geometry:
        meta.update(size=[1280, 720], sensor_mode=[1536, 864],
                    scaler_crop=list(CROP))
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    cfg_path = write_config(tmp, camera_calibration=cal_path)
    return d, cal_path, cfg_path, float(np.median(sides)), float(np.median(dists))


def test_bench_calibrarea_geometriei_cadrelor_rata_si_latura():
    d, cal_path, cfg_path, side, dist = bench_fixture()
    cfg = nova_config.load(cfg_path)
    cal = CameraCalibration.load(cal_path)
    geom, who = bd.frames_geometry(bd.load_meta(d), bd.first_frame_size(d))
    assert geom == GEOM and who == 'meta.json', (geom, who)
    r = bd.run_bench(d, cal, cfg, geometry=geom)
    c = r['calibration']
    assert c['kind'] == 'derivata+scalata' and abs(c['fx'] - 864.9) < 0.1, c
    for k in ('plain', 'aruco', 'pi'):
        assert abs(r[k]['rate'] - 4 / 6) < 1e-9, (k, r[k]['rate'])
        assert abs(r[k]['side_px'] / side - 1) < 0.03, (k, r[k]['side_px'], side)
        assert r[k]['p50'] > 0, (k, r[k])
    # the distance is right only with the derived calibration: the old
    # scaled one would have read ~1.5x too short
    assert abs(r['aruco']['dist_m'] / dist - 1) < 0.03, (r['aruco']['dist_m'], dist)
    # main(): the same, with the geometry and the calibration kind printed
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path])
    txt = buf.getvalue()
    assert rc == 0, txt
    assert 'geometrie (meta.json): mod 1536x864' in txt, txt
    assert 'calibrare derivata+scalata 1280x720: fx=864.9' in txt, txt
    return (f"derivata+scalata fx {c['fx']:.1f}; det 4/6 in toate; latura "
            f"{r['aruco']['side_px']:.1f} px (randat {side:.1f}); distanta "
            f"{r['aruco']['dist_m']:.2f} m (adevar {dist:.2f})")


def test_bench_fara_geometrie_refuza_cu_steaguri_merge():
    d, cal_path, cfg_path, side, dist = bench_fixture(with_geometry=False)
    try:
        bd.frames_geometry(bd.load_meta(d), bd.first_frame_size(d))
        assert False, "cadre fara geometrie acceptate"
    except bd.BenchRefused as e:
        assert '--sensor-mode' in str(e), e
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path])
    assert rc == 2 and 'REFUZ' in buf.getvalue(), buf.getvalue()
    # geometry given on the command line
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path,
                      '--sensor-mode', '1536', '864',
                      '--scaler-crop'] + [str(v) for v in CROP])
    txt = buf.getvalue()
    assert rc == 0, txt
    assert 'geometrie (linia de comanda)' in txt and 'fx=864.9' in txt, txt
    # geometry that does not fit the calibration: the 1536x864 mode with
    # a ScalerCrop at another binning cannot be derived -> refused
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path,
                      '--sensor-mode', '1536', '864',
                      '--scaler-crop', '0', '0', '4608', '2592'])
    assert rc == 2 and 'binning diferit' in buf.getvalue(), buf.getvalue()
    # meta.json size disagreeing with the frames
    meta = bd.load_meta(d)
    meta.update(size=[1536, 864], sensor_mode=[1536, 864], scaler_crop=list(CROP))
    try:
        bd.frames_geometry(meta, (1280, 720))
        assert False, "dimensiune diferita acceptata"
    except bd.BenchRefused:
        pass
    return "fara geometrie -> REFUZ; cu --sensor-mode/--scaler-crop -> fx 864.9"


def test_bench_rescale_acelasi_raport_merge_alt_raport_refuzat():
    d, cal_path, cfg_path, side, dist = bench_fixture()
    cfg = nova_config.load(cfg_path)
    cal = CameraCalibration.load(cal_path)
    r = bd.run_bench(d, cal, cfg, geometry=GEOM, rescale=(960, 540))
    rr = r['rescaled']
    assert tuple(rr['size']) == (960, 540)
    c = rr['calibration']
    assert c['kind'] == 'derivata+scalata' and abs(c['fx'] - 864.9 * 0.75) < 0.1, c
    for k in ('plain', 'aruco', 'pi'):
        assert abs(rr[k]['rate'] - 4 / 6) < 1e-9, (k, rr[k]['rate'])
        ratio = rr[k]['side_px'] / r[k]['side_px']
        assert abs(ratio - 0.75) < 0.03, (k, ratio)
    assert abs(rr['aruco']['dist_m'] / dist - 1) < 0.03, (rr['aruco']['dist_m'], dist)
    try:
        bd.run_bench(d, cal, cfg, geometry=GEOM, rescale=(1024, 768))
        assert False, "rescale cu alt raport de aspect acceptat"
    except bd.BenchRefused as e:
        assert 'decupaj' in str(e), e
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path, '--rescale', '960', '540'])
    txt = buf.getvalue()
    assert rc == 0 and '960x540 (scalate INTER_AREA' in txt and 'fx=648.7' in txt, txt
    buf, redir = captured()
    with redir:
        rc = bd.main(['--frames', d, '--config', cfg_path, '--rescale', '1024', '768'])
    assert rc == 2 and 'REFUZ' in buf.getvalue(), buf.getvalue()
    return (f"960x540: fx {c['fx']:.1f}, latura {rr['aruco']['side_px']:.1f} px "
            f"(x{rr['aruco']['side_px'] / r['aruco']['side_px']:.3f}); "
            f"1024x768 refuzat")


# --- calibrate_camera ----------------------------------------------------------------

def test_calibrate_setarile_live_native_implicit():
    cfg = {'camera_preset': 'crop1280'}
    s = cc.live_settings(cfg)
    assert s.preset == 'crop1280' and s.sensor_mode == (1536, 864), s.describe()
    assert s.output_size == (1536, 864), "calibrarea nu e nativa implicit"
    assert s.exposure == 'auto_lock' and s.lens_position == 1.63 and not s.awb
    s = cc.live_settings(cfg, 'full1280')
    assert (s.sensor_mode, s.output_size) == ((2304, 1296), (2304, 1296)), s.describe()
    s = cc.live_settings(cfg, 'trackerv2')
    assert s.output_size == (1536, 864) and s.lens_position == 1.0 and s.awb
    s = cc.live_settings(cfg, None, size=(1280, 720))
    assert (s.sensor_mode, s.output_size) == ((1536, 864), (1280, 720))
    s = cc.live_settings(cfg, None, sensor_mode=(2304, 1296))
    assert (s.sensor_mode, s.output_size) == ((2304, 1296), (2304, 1296))
    assert s.origin['sensor_mode'] == '--sensor-mode'
    m = cc.camera_meta(cc.live_settings(cfg))
    assert m['preset'] == 'crop1280' and m['camera_sensor_mode'] == '1536x864'
    assert m['camera_output_size'] == '1536x864' and m['camera_lens_position'] == 1.63
    assert all(v is None or isinstance(v, (str, int, float)) for v in m.values()), m
    return "crop1280 -> 1536x864 nativ; full1280 -> 2304x1296; --size / --sensor-mode"


class _StopAfter:
    """A target that never finds the board and ends the capture (Ctrl-C)
    after n frames: only the camera plumbing is under test."""
    def __init__(self, n=2):
        self.n = n
        self.sizes = []

    def detect(self, gray):
        self.sizes.append((gray.shape[1], gray.shape[0]))
        if len(self.sizes) >= self.n:
            raise KeyboardInterrupt
        return None

    def expected_corners(self):
        return 40


def test_calibrate_collect_live_deschide_modul_presetului_nativ():
    note = []
    for preset, size, mode, stream, crop in (
            ('crop1280', None, (1536, 864), (1536, 864), CROP),
            ('full1280', None, (2304, 1296), (2304, 1296), FULL),
            ('crop1280', (1280, 720), (1536, 864), (1280, 720), CROP)):
        s = cc.live_settings({}, preset, size=size)
        tgt = _StopAfter()
        with fake_camera() as cam, quiet():
            dets, sets, got_size, lens, geom = cc.collect_live(
                tgt, 20, 60, show_window=False, settings=s)
            conf = cam.instances[-1].configured[0]
            assert tuple(conf['sensor']['output_size']) == mode, conf
            assert tuple(conf['main']['size']) == stream, conf
            assert cam.instances[-1].closed
        assert geom == CameraGeometry(mode, crop, stream), geom
        assert got_size == stream and set(tgt.sizes) == {stream}, (got_size, tgt.sizes)
        assert lens == 1.63 and dets == [], (lens, dets)
        note.append(f"{preset}{'' if size is None else ' --size'} -> "
                    f"{mode[0]}x{mode[1]}/{stream[0]}x{stream[1]}")
    return '; '.join(note)


class _Replay:
    """Replays synthetic board detections, one per camera frame, then
    Ctrl-C: the real collect_live / main path, a known board."""
    def __init__(self, target, dets, sets):
        self.target = target
        self.items = [(o, i, c) for (o, i), c in zip(dets, sets)]

    def detect(self, gray):
        if not self.items:
            raise KeyboardInterrupt
        return self.items.pop(0)

    def describe(self):
        return self.target.describe()

    def expected_corners(self):
        return self.target.expected_corners()

    def meta(self):
        return self.target.meta()


def test_calibrate_main_live_scrie_presetul_setarile_si_geometria():
    """cc.main --live --preset full1280: camera in 2304x1296 nativ (stub),
    25 de vederi ChArUco sintetice la 2304x1296 -> calibrarea salvata are
    presetul, setarile, geometria si LensPosition-ul in meta, si
    calibration_for o poate scala la fluxul de zbor al presetului."""
    import test_calibrate_camera as tcc
    target, dets, sets, meta = tcc.synth_views('charuco', 9, 6, n=25)
    assert tcc.SYN_WH == (2304, 1296) and len(dets) >= 20, (tcc.SYN_WH, len(dets))
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, 'cam.yaml')
    old = cc.make_target, cc.LIVE_MIN_INTERVAL_S
    cc.make_target = lambda *a, **k: _Replay(target, dets, sets)
    cc.LIVE_MIN_INTERVAL_S = 0.0
    try:
        with fake_camera(), quiet():
            rc = cc.main(['--live', '--preset', 'full1280', '--no-window',
                          '--square-mm', str(meta['square_mm_nominal']),
                          '--out', out, '--config', write_config(tmp)])
    finally:
        cc.make_target, cc.LIVE_MIN_INTERVAL_S = old
    assert rc == 0, rc
    back = CameraCalibration.load(out)
    m = back.meta
    assert m['preset'] == 'full1280', m
    assert m['sensor_mode'] == '2304x1296' and m['scaler_crop'] == '0,0,4608,2592', m
    assert m['camera_sensor_mode'] == '2304x1296', m
    assert m['camera_output_size'] == '2304x1296' and m['resolution'] == '2304x1296', m
    assert m['camera_exposure'] == 'auto_lock', m
    assert abs(m['lens_position'] - 1.63) < 1e-6 and abs(m['camera_lens_position'] - 1.63) < 1e-6
    assert (back.width, back.height) == (2304, 1296)
    fly = cc.live_settings({}, 'full1280')        # only for the mode
    c = calibration_for(back, CameraGeometry(fly.sensor_mode,
                                             IMX708_MODES[fly.sensor_mode],
                                             (1280, 720)))
    assert c.kind == 'scalata', c.kind
    return (f"{back.n_images} poze, fx {back.fx:.1f}, meta preset "
            f"{m['preset']} mod {m['sensor_mode']} LensPosition "
            f"{m['lens_position']:g}; la zbor: {c.kind}")


def test_calibrate_din_director_citeste_ce_a_scris_record_frames():
    """Cadrele de la record_frames sunt si poze de calibrare: geometria si
    presetul/setarile/LensPosition-ul lor ajung in calibrare."""
    tmp = tempfile.mkdtemp()
    out, meta = record(tmp, '--preset', 'crop1536')
    mode, crop = cc.geometry_for_dir(out)
    assert mode == (1536, 864) and crop == CROP, (mode, crop)
    m = cc.dir_camera_meta(out)
    assert m['preset'] == 'crop1536' and m['camera_sensor_mode'] == '1536x864', m
    assert m['camera_output_size'] == '1536x864' and abs(m['lens_position'] - 1.63) < 1e-9, m
    assert cc.dir_camera_meta(tempfile.mkdtemp()) == {}
    return "geometrie 1536x864 + preset crop1536 + LensPosition 1.63 din meta.json"


TESTS = [
    ('record_frames: geometria, presetul si metadatele per cadru',
     test_record_frames_geometria_presetul_si_metadatele_per_cadru),
    ('record_frames: cheile obligatorii exista si fara raport',
     test_record_frames_cheile_obligatorii_exista_si_fara_raport),
    ('record_frames: preset inexistent refuzat',
     test_record_frames_preset_inexistent_refuzat),
    ('bench: calibrarea geometriei cadrelor, rata si latura',
     test_bench_calibrarea_geometriei_cadrelor_rata_si_latura),
    ('bench: fara geometrie refuza, cu steaguri merge',
     test_bench_fara_geometrie_refuza_cu_steaguri_merge),
    ('bench: --rescale acelasi raport merge, alt raport refuzat',
     test_bench_rescale_acelasi_raport_merge_alt_raport_refuzat),
    ('calibrate: setarile live, native implicit',
     test_calibrate_setarile_live_native_implicit),
    ('calibrate: collect_live deschide modul presetului, nativ',
     test_calibrate_collect_live_deschide_modul_presetului_nativ),
    ('calibrate: main --live scrie presetul, setarile si geometria',
     test_calibrate_main_live_scrie_presetul_setarile_si_geometria),
    ('calibrate: din director citeste ce a scris record_frames',
     test_calibrate_din_director_citeste_ce_a_scris_record_frames),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            note = fn()
            print(f"  OK    {name}" + (f"   ({note})" if note else ""))
        except AssertionError as e:
            fails += 1
            print(f"  ESEC  {name}\n        {e}")
        except Exception as e:                      # noqa: BLE001
            fails += 1
            import traceback
            traceback.print_exc()
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
