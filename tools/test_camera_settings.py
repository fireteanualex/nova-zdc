#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Parametrii camerei in config si preseturile (27.09.2026).

    python3 tools/test_camera_settings.py

Ce conteaza: presetul implicit (crop1280) e exact ce s-a zburat - acelasi
mod, acelasi flux, aceeasi expunere masurata-apoi-blocata, acelasi focus -
cu calibrarea derivata corect; fiecare preset ajunge la geometria si
calibrarea asteptate; validarea prinde un mod inexistent si auto_capped
fara limite; fiecare mod de expunere da controalele lui (stub picamera2).
"""

import argparse
import contextlib
import io
import json
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import config as nova_config                          # noqa: E402
from nova.detector_pi import (AUTOEXP_MAX_US, CAMERA_PRESETS,   # noqa: E402
                              CameraCalibration, CameraGeometry,
                              CameraModeError, IMX708_MODES,
                              cap_agc_tuning, calibration_for,
                              camera_settings)
import preflight_check as pf                                    # noqa: E402
from test_camera_geometry import (FakePicamera2, REPO_CAL,      # noqa: E402
                                  fake_camera, quiet)

#: fx each preset must end with, from the vehicle's calibration
#: (config/camera_pi.yaml: 2304x1296 full field, fx 1037.89)
FX = {'crop1280': 864.9, 'crop1536': 1037.9, 'full1280': 576.6,
      'trackerv2': 864.9}
KIND = {'crop1280': 'derivata+scalata', 'crop1536': 'derivata',
        'full1280': 'scalata', 'trackerv2': 'derivata+scalata'}


def test_fiecare_preset_geometria_si_calibrarea_lui():
    cal = CameraCalibration.load(REPO_CAL)
    out = []
    assert set(CAMERA_PRESETS) == set(FX), sorted(CAMERA_PRESETS)
    for name in CAMERA_PRESETS:
        s = camera_settings({}, name)
        geom = CameraGeometry(s.sensor_mode, IMX708_MODES[s.sensor_mode],
                              s.output_size)
        c = calibration_for(cal, geom)
        assert c.kind == KIND[name] and abs(c.fx - FX[name]) < 0.1, (name, c.kind, c.fx)
        out.append(f"{name} {c.kind} fx {c.fx:.1f}")
    t = camera_settings({}, 'trackerv2')
    assert (t.exposure, t.awb, t.lens_position) == ('auto', True, 1.0)
    return '; '.join(out)


def test_presetul_implicit_e_ce_s_a_zburat():
    """config/nova.json: crop1280 = modul 1536x864, flux 1280x720, expunere
    masurata apoi blocata (camera_auto_expose true de pana acum), fara AWB,
    LensPosition 1.63. Nimic din comportamentul camerei nu se schimba; se
    schimba doar geometria declarata (commit-ul de la punctul 1)."""
    cfg = nova_config.load()
    s = camera_settings(cfg)
    assert s.preset == 'crop1280'
    assert (s.sensor_mode, s.output_size) == ((1536, 864), (1280, 720))
    assert (s.exposure, s.awb, s.lens_position) == ('auto_lock', False, 1.63)
    raw = json.load(open(nova_config.DEFAULT_PATH))
    for vechi in ('camera_auto_expose', 'track_size', 'sensor_mode'):
        assert vechi not in raw, f"{vechi} ramas in nova.json langa preset"
    assert cfg['camera_fresh_capture'] is False, "camera_fresh_capture schimbat"
    return s.describe()


def test_validarea_configului():
    def refuz(cfg, preset=None, text=''):
        try:
            camera_settings(cfg, preset)
        except CameraModeError as e:
            assert text in str(e), (text, str(e))
            return str(e)
        raise AssertionError(f"acceptat: {cfg} {preset}")

    refuz({}, text='lipseste `sensor_mode`')
    refuz({'sensor_mode': [1920, 1080]}, text='nu exista pe IMX708')
    refuz({'camera_preset': 'hd'}, text="presetul 'hd' nu exista")
    refuz({}, 'hd', text="presetul 'hd' nu exista")
    refuz({'sensor_mode': [1536, 864], 'exposure': 'manual'}, text='valori')
    m = refuz({'camera_preset': 'crop1280', 'exposure': 'auto_capped'},
              text='fara limite')
    assert 'exposure_max_us 2000' in m and 'gain_max 16' in m, m
    refuz({'camera_preset': 'crop1280', 'exposure': 'auto_capped',
           'exposure_max_us': 2000}, text='gain_max')
    ok = camera_settings({'camera_preset': 'crop1280', 'exposure': 'auto_capped',
                          'exposure_max_us': 1500, 'gain_max': 12})
    assert (ok.exposure_max_us, ok.gain_max) == (1500, 12.0)
    return "fara mod / mod inexistent / preset necunoscut / expunere / auto_capped fara limite"


def test_ordinea_preset_chei_linie_de_comanda():
    """Fisierul: presetul lui, rafinat de cheile lui. Linia de comanda:
    presetul cerut, EXACT - cheile camerei din fisier nu se aplica. Cheile
    vechi (track_size, camera_auto_expose) se citesc cand cele noi lipsesc."""
    cfg = {'camera_preset': 'crop1280', 'lens_position': 2.0, 'awb': True}
    s = camera_settings(cfg)
    assert (s.preset, s.lens_position, s.awb, s.sensor_mode) == \
        ('crop1280', 2.0, True, (1536, 864)), s.describe()
    assert s.origin['lens_position'] == 'config' and s.origin['sensor_mode'] == 'preset crop1280'
    f = camera_settings(cfg, 'full1280')
    assert (f.preset, f.sensor_mode, f.lens_position, f.awb) == \
        ('full1280', (2304, 1296), 1.63, False), f.describe()
    v = camera_settings({'sensor_mode': [2304, 1296], 'track_size': [1280, 720],
                         'camera_auto_expose': False})
    assert (v.output_size, v.exposure) == ((1280, 720), 'fixed')
    v2 = camera_settings({'sensor_mode': [2304, 1296], 'camera_auto_expose': True})
    assert v2.exposure == 'auto_lock' and v2.output_size == (2304, 1296)
    # cheile vechi nu calca peste presetul din fisier daca nu sunt in fisier
    t = camera_settings(dict(nova_config.DEFAULTS, camera_preset='trackerv2'))
    assert t.exposure == 'auto', "DEFAULTS au suprascris expunerea presetului"
    return "fisier: preset <- chei; --preset: exact presetul; chei vechi citite"


def _deschide(settings):
    from nova.detector_pi import PiCameraSource
    with fake_camera() as cam, quiet():
        src = PiCameraSource(settings=settings)
        pc = cam.instances[-1]
        src.close()
    return src, pc


def test_controalele_fiecarui_mod_de_expunere_si_focusul():
    """Stub picamera2: ce controale ajung la camera, pe mod. auto_lock si
    fixed sunt, cheie cu cheie, cele de dinainte de preseturi."""
    base = {'sensor_mode': [1536, 864], 'output_size': [1280, 720]}
    # fixed
    src, pc = _deschide(camera_settings(dict(base, exposure='fixed',
                                             exposure_us=1500, analogue_gain=4)))
    assert pc.controls_log[0] == {'LensPosition': 1.63, 'AwbEnable': False,
                                  'AfMode': 0, 'ExposureTime': 1500,
                                  'AnalogueGain': 4.0, 'AeEnable': False}, pc.controls_log
    assert len(pc.controls_log) == 1 and pc.tuning is None and not src.control_problems
    # auto_lock: AE pornit, apoi blocat pe ce a masurat, plafonat la 2 ms
    FakePicamera2.auto_exposure_us = 1200
    src, pc = _deschide(camera_settings({'camera_preset': 'crop1280'}))
    assert pc.controls_log[0] == {'LensPosition': 1.63, 'AwbEnable': False,
                                  'AfMode': 0, 'AeEnable': True}, pc.controls_log[0]
    locked = pc.controls_log[1]
    assert locked['AeEnable'] is False and locked['ExposureTime'] == 1200
    assert locked['ExposureTime'] <= AUTOEXP_MAX_US
    # auto (trackerv2): AE continuu, AWB, LensPosition 1.0, fara blocare
    src, pc = _deschide(camera_settings({}, 'trackerv2'))
    assert pc.controls_log == [{'LensPosition': 1.0, 'AwbEnable': True,
                                'AfMode': 0, 'AeEnable': True}], pc.controls_log
    assert pc.tuning is None
    # auto_capped: tuning cu modul 'custom' plafonat + AeExposureMode Custom
    s = camera_settings({'camera_preset': 'crop1280', 'exposure': 'auto_capped',
                         'exposure_max_us': 2000, 'gain_max': 16})
    src, pc = _deschide(s)
    assert pc.controls_log[0]['AeExposureMode'] == 3 and pc.controls_log[0]['AeEnable']
    assert len(pc.controls_log) == 1, "auto_capped nu se blocheaza"
    for ch in pc.tuning['algorithms'][1]['rpi.agc']['channels']:
        c = ch['exposure_modes']['custom']
        assert c['shutter'] == [100.0, 2000.0, 2000.0] and c['gain'] == [1.0, 1.0, 16.0], c
    assert not src.control_problems, src.control_problems
    # si daca AE-ul iese totusi peste plafon, citirea inapoi o spune
    FakePicamera2.auto_exposure_us = 4000
    src, pc = _deschide(s)
    assert any('peste plafonul 2000' in p for p in src.control_problems), src.control_problems
    return "fixed / auto_lock / auto / auto_capped: controalele lor; focus manual in toate"


def test_plafonarea_in_tuning_pe_ambele_formate():
    vechi = {'exposure_modes': {'normal': {'exposure': [100, 10000], 'gain': [1.0, 8.0]}}}
    assert cap_agc_tuning(vechi, 1800, 12) == 1
    assert vechi['exposure_modes']['custom'] == {'exposure': [100.0, 1800.0, 1800.0],
                                                 'gain': [1.0, 1.0, 12.0]}
    nou = json.loads(json.dumps(FakePicamera2.TUNING))['algorithms'][1]['rpi.agc']
    assert cap_agc_tuning(nou, 2000, 16) == 2
    try:
        cap_agc_tuning({'channels': [{}]}, 2000, 16)
        assert False
    except CameraModeError:
        pass
    return "AGC plat sau pe canale, 'shutter' sau 'exposure'"


def test_preflight_si_scripturile_poarta_presetul():
    """--preset ajunge in preflight (linia 'rezolutie' il spune) si, prin
    descent_test.sh --preset=, si in preflight si in aplicatie - ce s-a
    verificat e ce zboara."""
    tmp = tempfile.mkdtemp()
    cfg_path = os.path.join(tmp, 'nova.json')
    json.dump({'camera_calibration': REPO_CAL, 'camera_preset': 'crop1280'},
              open(cfg_path, 'w'))

    class Sursa:
        nominal_fps = 30.0
        control_problems = []

        def __init__(self, geom):
            self.geometry, self.size, self.i = geom, geom.output_size, 0

        def read(self):
            if self.i >= 4:
                return None
            self.i += 1
            return np.full((self.size[1], self.size[0]), 128, np.uint8), time.monotonic()

        def close(self):
            pass

    def ruleaza(preset, geom):
        args = argparse.Namespace(config=cfg_path, conn='/dev/null', baud=1,
                                  parm='x', frames=4, mavlink_timeout=0.1,
                                  no_camera=False, no_mavlink=True, json=False,
                                  os_release='/etc/os-release', preset=preset)
        return {r.name: r for r in pf.run_checks(args, source_factory=lambda: Sursa(geom))}['rezolutie']

    full = CameraGeometry((2304, 1296), (0, 0, 4608, 2592), (1280, 720))
    crop = CameraGeometry((1536, 864), (768, 432, 3072, 1728), (1280, 720))
    r = ruleaza('full1280', full)
    assert r.status == pf.OK and r.detail.startswith('preset full1280 |') \
        and 'calibrare scalata' in r.detail and abs(r.data['fx'] - 576.6) < 0.1, r.detail
    r = ruleaza(None, crop)
    assert r.status == pf.OK and r.detail.startswith('preset crop1280 |'), r.detail
    r = ruleaza('full1280', crop)
    assert r.status == pf.ESEC and 'difera de cel cerut 2304x1296' in r.detail, r.detail

    dt = open(os.path.join(REPO, 'pi', 'descent_test.sh')).read()
    assert '--preset=*)      PRESET="${arg#*=}"' in dt
    assert '${PRESET:+--preset "$PRESET"}' in dt, "preflight fara preset"
    assert 'ARGS+=(--camera-preset "$PRESET")' in dt, "aplicatia fara preset"
    pi = open(os.path.join(HERE, 'nova_pi.py')).read()
    assert 'preset=a.camera_preset' in pi and "'--camera-preset'" in pi
    return "preflight --preset; descent_test --preset= -> preflight si aplicatie"


def test_build_pi_detector_cu_preset():
    from nova.detector_pi import build_pi_detector
    cfg = nova_config.load()
    for name in ('crop1536', 'full1280'):
        with fake_camera(), quiet():
            det = build_pi_detector(cfg, verbose=True, preset=name)
            try:
                c = det.det.calib
                assert c.kind == KIND[name] and abs(c.fx - FX[name]) < 0.1, (name, c.kind, c.fx)
                assert det.source.settings.preset == name
            finally:
                det.stop()
    return "crop1536 derivata fx 1037.9; full1280 scalata fx 576.6"


TESTS = [
    ('fiecare preset: geometria si calibrarea lui',
     test_fiecare_preset_geometria_si_calibrarea_lui),
    ('presetul implicit e ce s-a zburat', test_presetul_implicit_e_ce_s_a_zburat),
    ('validarea config-ului', test_validarea_configului),
    ('ordinea preset / chei / linie de comanda',
     test_ordinea_preset_chei_linie_de_comanda),
    ('controalele fiecarui mod de expunere si focusul',
     test_controalele_fiecarui_mod_de_expunere_si_focusul),
    ('plafonarea in tuning pe ambele formate', test_plafonarea_in_tuning_pe_ambele_formate),
    ('preflight si scripturile poarta presetul',
     test_preflight_si_scripturile_poarta_presetul),
    ('build_pi_detector cu preset', test_build_pi_detector_cu_preset),
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
