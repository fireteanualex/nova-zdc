#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Geometria camerei: modul senzorului fixat explicit, ScalerCrop citit inapoi,
calibrarea folosita doar prin calibration_for (scalare / derivare / refuz).

    python3 tools/test_camera_geometry.py

Situatia gasita pe vehicul (27.09.2026, `descent_test.sh --check`): fluxul
de 1280x720 cerut fara mod de senzor, libcamera a ales modul 1536x864
DECUPAT, iar calibrarea de 2304x1296 camp intreg a fost scalata ca si cum
senzorul ar fi fost in 2304x1296 - fx 577 in loc de ~865. Acelasi 16:9, deci
regula "acelasi raport de aspect -> scalare" nu a prins nimic.

picamera2 nu exista pe desktop: camera e un stub care imita ce face
picamera2 + libcamera (moduri, `sensor=` in configuratie, citirea inapoi,
ScalerCrop in metadate). Verificarea pe camera reala e in raport.
"""

import argparse
import contextlib
import io
import json
import math
import os
import sys
import tempfile
import time
import types

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import config as nova_config                          # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector, CalibrationMismatch,  # noqa: E402
                              CameraCalibration, CameraGeometry,
                              CameraModeError, LEGACY_CAL_CROP,
                              LEGACY_CAL_MODE, calibration_for,
                              geometric_focal_px, output_size_from_config,
                              sensor_mode_from_config)
import preflight_check as pf                                    # noqa: E402
import test_detector_pi as tdp                                  # noqa: E402

FULL = (0, 0, 4608, 2592)
CROP = (768, 432, 3072, 1728)
REPO_CAL = os.path.join(REPO, 'config', 'camera_pi.yaml')


def real_like_calibration():
    """The vehicle's calibration, in numbers (config/camera_pi.yaml, 13.09):
    2304x1296 full field, fx 1037.9, principal point off-centre - so the
    derivation's shift is visible, not hidden by a centred cx."""
    K = [[1037.89, 0, 1188.87], [0, 1038.68, 647.17], [0, 0, 1]]
    return CameraCalibration(K, [-0.03, 0.103, 0.0012, 0.0089, -0.027],
                             2304, 1296, rms=0.829, n_images=60,
                             source='ca pe vehicul (test)')


# --- stub picamera2 / libcamera ------------------------------------------------

MODES = [
    {'format': 'SRGGB10_CSI2P', 'size': (1536, 864), 'bit_depth': 10,
     'crop_limits': CROP, 'fps': 120.13},
    {'format': 'SRGGB10_CSI2P', 'size': (2304, 1296), 'bit_depth': 10,
     'crop_limits': FULL, 'fps': 56.03},
    {'format': 'SRGGB10_CSI2P', 'size': (4608, 2592), 'bit_depth': 10,
     'crop_limits': FULL, 'fps': 14.35},
]


def libcamera_choice(main_size):
    """What libcamera / picamera2 pick when NO sensor mode is given: the
    score of picamera2._make_libcamera_config (0.3.37). For a 1280x720
    stream it is 1536x864 - the finding."""
    def score(m):
        def sf(desired, actual):
            d = desired - actual
            return -d / 4 if d < 0 else d * 2
        ow, oh = main_size
        mw, mh = m['size']
        return sf(ow, mw) + sf(oh, mh) + 1500 * sf(ow / oh, mw / mh)
    return min(MODES, key=score)['size']


class FakeRequest:
    def __init__(self, cam):
        self.cam = cam

    def make_array(self, name):
        w, h = self.cam._config['main']['size']
        return np.full((h * 3 // 2, w), 100, np.uint8)

    def get_metadata(self):
        return self.cam.capture_metadata()

    def release(self):
        self.cam.released += 1


class FakePicamera2:
    #: knobs, per test
    force_mode = None          # libcamera "picks" this whatever is asked
    force_crop = None          # ScalerCrop reported instead of the mode's
    no_crop = False            # metadata without ScalerCrop
    instances = []

    camera_controls = {'AnalogueGain': (1.0, 16.0, 1.0),
                       'ExposureTime': (9, 1000000, 20000)}

    def __init__(self, camera_num=0, tuning=None):
        self.closed = False
        self.started = False
        self.stopped = False
        self._ctrl = {}
        self.controls_log = []
        self.configured = []
        self.released = 0
        self._config = None
        self.modes_read = 0
        FakePicamera2.instances.append(self)

    @property
    def sensor_modes(self):
        self.modes_read += 1
        return [dict(m) for m in MODES]

    def create_video_configuration(self, main=None, sensor=None, controls=None,
                                   buffer_count=4, **kw):
        return {'main': dict(main), 'sensor': dict(sensor) if sensor else None,
                'controls': controls, 'kind': 'video'}

    def create_still_configuration(self, main=None, sensor=None, **kw):
        return {'main': dict(main), 'sensor': dict(sensor) if sensor else None,
                'kind': 'still'}

    def configure(self, cfg):
        self.configured.append(cfg)
        if self.force_mode is not None:
            mode = tuple(self.force_mode)
        elif cfg['sensor']:
            mode = tuple(cfg['sensor']['output_size'])
        else:
            mode = libcamera_choice(tuple(cfg['main']['size']))
        self._config = {'main': {'size': tuple(cfg['main']['size'])},
                        'sensor': {'output_size': mode, 'bit_depth': 10}}
        self._crop = next(m['crop_limits'] for m in MODES if m['size'] == mode)

    def camera_configuration(self):
        return self._config

    def set_controls(self, c):
        self.controls_log.append(dict(c))
        self._ctrl.update(c)

    def start(self):
        self.started = True

    def capture_metadata(self):
        ae = self._ctrl.get('AeEnable', False)
        md = {'ExposureTime': 1200 if ae else self._ctrl.get('ExposureTime', 2000),
              'AnalogueGain': 1.5 if ae else self._ctrl.get('AnalogueGain', 8.0),
              'LensPosition': self._ctrl.get('LensPosition', 0.0),
              'SensorTimestamp': time.clock_gettime_ns(time.CLOCK_BOOTTIME)}
        if not self.no_crop:
            md['ScalerCrop'] = self.force_crop or self._crop
        return md

    def capture_request(self, flush=None):
        time.sleep(1.0 / 60.0)
        return FakeRequest(self)

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


@contextlib.contextmanager
def fake_camera(force_mode=None, force_crop=None, no_crop=False):
    FakePicamera2.force_mode = force_mode
    FakePicamera2.force_crop = force_crop
    FakePicamera2.no_crop = no_crop
    FakePicamera2.instances = []
    mods = {'picamera2': types.SimpleNamespace(Picamera2=FakePicamera2),
            'libcamera': types.SimpleNamespace(controls=types.SimpleNamespace(
                AfModeEnum=types.SimpleNamespace(Manual=0)))}
    old = {k: sys.modules.get(k) for k in mods}
    sys.modules.update(mods)
    try:
        yield FakePicamera2
    finally:
        for k, v in old.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        FakePicamera2.force_mode = FakePicamera2.force_crop = None
        FakePicamera2.no_crop = False


def quiet():
    return contextlib.redirect_stdout(io.StringIO())


# --- punctul 1: situatia gasita --------------------------------------------------

def test_situatia_gasita_pe_Pi_preflight_PICAT():
    """Reproducerea exacta: senzorul efectiv in 1536x864 (decupat), fluxul
    1280x720, calibrarea de 2304x1296 camp intreg. Vechea verificare zicea
    "OK 1280x720 (calibrare 2304x1296 scalata)". Acum:
      - config care presupune campul intreg (sensor_mode 2304x1296) si o
        camera care ruleaza 1536x864 -> preflight PICAT pe 'rezolutie';
      - pentru geometria 1536x864 calibrarea NU se mai poate scala (alt
        mod, alt ScalerCrop): se deriva, fx ~865, niciodata 577;
      - libcamera, lasat sa aleaga, chiar alege 1536x864 pentru 1280x720
        (scorul din picamera2) - de aceea modul nu se mai lasa implicit."""
    assert libcamera_choice((1280, 720)) == (1536, 864)
    cal = real_like_calibration()
    geom = CameraGeometry((1536, 864), CROP, (1280, 720))
    vechi = cal.scaled_to(1280, 720)                  # ce se folosea
    nou = calibration_for(cal, geom)
    assert abs(vechi.fx - 576.6) < 0.1, vechi.fx
    assert nou.kind == 'derivata+scalata' and abs(nou.fx - 864.9) < 0.1, (nou.kind, nou.fx)
    assert abs(nou.fx / vechi.fx - 1.5) < 1e-6

    tmp = tempfile.mkdtemp()
    cfg_path = os.path.join(tmp, 'nova.json')

    class Sursa:
        nominal_fps = 30.0
        control_problems = []
        size = (1280, 720)
        geometry = geom

        def __init__(self):
            self.i = 0

        def read(self):
            if self.i >= 4:
                return None
            self.i += 1
            return np.full((720, 1280), 128, np.uint8), time.monotonic()

        def close(self):
            pass

    def ruleaza(sensor_mode):
        with open(cfg_path, 'w') as f:
            json.dump({'camera_calibration': REPO_CAL, 'track_size': [1280, 720],
                       'sensor_mode': sensor_mode}, f)
        args = argparse.Namespace(config=cfg_path, conn='/dev/null', baud=1,
                                  parm='x', frames=4, mavlink_timeout=0.1,
                                  no_camera=False, no_mavlink=True, json=False,
                                  os_release='/etc/os-release')
        return {r.name: r for r in pf.run_checks(args, source_factory=Sursa)}

    r = ruleaza([2304, 1296])['rezolutie']
    assert r.status == pf.ESEC, r.detail
    assert '1536x864 difera de cel cerut 2304x1296' in r.detail, r.detail
    r = ruleaza([1536, 864])['rezolutie']
    assert r.status == pf.OK and 'derivata+scalata' in r.detail, r.detail
    assert 'ScalerCrop (768, 432, 3072, 1728)' in r.detail and 'flux 1280x720' in r.detail
    assert abs(r.data['fx'] - 864.9) < 0.1, r.data
    return (f"mod efectiv 1536 cu config 2304 -> PICAT; fx scalat gresit "
            f"{vechi.fx:.1f}, derivat {nou.fx:.1f} (x1.5)")


def test_scalarea_acceptata_doar_in_acelasi_mod_si_decupaj():
    """full1280: modul 2304x1296 camp intreg, fluxul 1280x720 -> scalare,
    fx ~577. Aceeasi calibrare, alt raport de aspect -> refuz."""
    cal = real_like_calibration()
    c = calibration_for(cal, CameraGeometry((2304, 1296), FULL, (1280, 720)))
    assert c.kind == 'scalata' and abs(c.fx - 576.6) < 0.1, (c.kind, c.fx)
    assert c.meta['sensor_mode'] == '2304x1296' and c.meta['scaler_crop'] == '0,0,4608,2592'
    n = calibration_for(cal, CameraGeometry((2304, 1296), FULL, (2304, 1296)))
    assert n.kind == 'nativa' and n.fx == cal.fx and n is not cal
    assert cal.meta.get('sensor_mode') is None, "calibrarea incarcata a fost modificata"
    try:
        calibration_for(cal, CameraGeometry((2304, 1296), FULL, (1024, 600)))
        assert False, "alt raport de aspect acceptat"
    except CalibrationMismatch as e:
        assert 'decupaj' in str(e)
    return f"2304 -> 1280x720: scalata, fx {c.fx:.1f}; 1024x600 refuzat"


def test_derivarea_pentru_modul_decupat_pe_adevar():
    """crop1536 / crop1280: aceeasi lentila, aceiasi pixeli binned, doar
    fereastra e alta. Adevarul: markerul randat in 2304x1296 camp intreg,
    decupat cum il citeste modul 1536x864 (offset-ul ScalerCrop / 2), apoi
    redus la 1280x720 de "ISP". Calibrarea derivata da unghiurile si
    distanta adevarate; cea scalata (vechea regula) le da gresit cu ~1.5x."""
    cal = real_like_calibration()
    g1536 = CameraGeometry((1536, 864), CROP, (1536, 864))
    g1280 = CameraGeometry((1536, 864), CROP, (1280, 720))
    d = calibration_for(cal, g1536)
    assert d.kind == 'derivata' and d.fx == cal.fx and d.fy == cal.fy
    assert abs(d.cx - (cal.cx - 384)) < 1e-9 and abs(d.cy - (cal.cy - 216)) < 1e-9
    assert np.allclose(d.dist, cal.dist) and (d.width, d.height) == (1536, 864)
    assert d.meta['sensor_mode'] == '1536x864' and d.meta['scaler_crop'] == '768,432,3072,1728'
    ds = calibration_for(cal, g1280)
    assert ds.kind == 'derivata+scalata' and abs(ds.fx - cal.fx * 1280 / 1536) < 1e-9
    assert abs(ds.hfov_deg() - d.hfov_deg()) < 1e-6 and 72 < d.hfov_deg() < 74
    gresit = cal.scaled_to(1280, 720)
    worst = 0.0
    for t in ((0.0, 0.0, 6.0), (0.9, -0.5, 7.0), (-1.2, 0.6, 9.0)):
        full, _ = tdp.render(cal, tdp.R_FLAT, t, module_px=60)
        mode_img = full[216:216 + 864, 384:384 + 1536]          # ScalerCrop / 2
        tr = tdp.truth(t)
        for img, c in ((mode_img, d),
                       (cv2.resize(mode_img, (1280, 720), interpolation=cv2.INTER_AREA), ds)):
            det = ArucoMarkerDetector(c, roi_below_m=0.0).detect(img, 1.0)
            assert det is not None, (t, c.kind)
            assert abs(det.distance_m / tr['distance'] - 1) < 0.03, (c.kind, det.distance_m, tr)
            for k in ('angle_x', 'angle_y'):
                err = abs(math.degrees(getattr(det, k) - tr[k]))
                worst = max(worst, err)
                assert err < 0.4, (c.kind, k, err)
        small = cv2.resize(mode_img, (1280, 720), interpolation=cv2.INTER_AREA)
        rau = ArucoMarkerDetector(gresit, roi_below_m=0.0).detect(small, 1.0)
        assert rau is not None
        assert abs(rau.distance_m / tr['distance'] - 1 / 1.5) < 0.05, (rau.distance_m, tr)
        if abs(t[0]) > 0.5:
            # 1.5 from the focal length, plus the principal point the old
            # scaling also put ~10 px off (660.5 instead of 670.7)
            ratio = math.tan(rau.angle_y) / math.tan(tr['angle_y'])
            assert 1.35 < ratio < 1.75, ratio
    return (f"derivata: fx {d.fx:.1f}, cx {d.cx:.1f} (-384), unghiuri <= "
            f"{worst:.2f} grade; scalata gresit: distanta x0.67, unghiuri x1.5")


def test_derivarea_refuzata_in_orice_alt_caz():
    """Orice altceva decat scalare sau derivare la acelasi binning -> refuz
    cu mesaj care spune sa se refaca calibrarea in modul respectiv."""
    cal = real_like_calibration()
    cazuri = {
        'alt ScalerCrop (zoom ISP in modul 1536)':
            CameraGeometry((1536, 864), (1000, 500, 2560, 1440), (1280, 720)),
        'camp intreg citit de modul 1536 (binning 3)':
            CameraGeometry((1536, 864), FULL, (1536, 864)),
        'modul nativ 4608 (binning 1)':
            CameraGeometry((4608, 2592), FULL, (1280, 720)),
        'decupaj 4:3 pe modul 2304':
            CameraGeometry((2304, 1296), (576, 0, 3456, 2592), (1024, 768)),
    }
    for nume, g in cazuri.items():
        try:
            calibration_for(cal, g)
            assert False, f"{nume}: acceptat"
        except CalibrationMismatch as e:
            assert 'Refa calibrarea in modul' in str(e), (nume, str(e))
    # o calibrare deja scalata nu mai e baza pentru o derivare
    try:
        calibration_for(cal.scaled_to(1280, 720), CameraGeometry((1536, 864), CROP, (1280, 720)))
        assert False, "derivare dintr-o calibrare scalata"
    except CalibrationMismatch as e:
        assert 'deja scalata' in str(e)
    # o calibrare facuta nativ in 1536 nu se poate folosi pe campul intreg
    nat = calibration_for(cal, CameraGeometry((1536, 864), CROP, (1536, 864)))
    try:
        calibration_for(nat, CameraGeometry((2304, 1296), FULL, (1280, 720)))
        assert False, "calibrare de decupaj folosita pe campul intreg"
    except CalibrationMismatch as e:
        assert 'campul intreg' in str(e)
    # dar aceeasi calibrare, in geometria ei, e nativa (si scalabila)
    assert calibration_for(nat, CameraGeometry((1536, 864), CROP, (1536, 864))).kind == 'nativa'
    assert calibration_for(nat, CameraGeometry((1536, 864), CROP, (1280, 720))).kind == 'scalata'
    return f"{len(cazuri) + 2} geometrii refuzate, fiecare cu motiv"


def test_calibrarea_fara_geometrie_e_2304_camp_intreg_si_se_salveaza():
    """Fisierele de dinainte (fara camp) = 2304x1296, ScalerCrop (0, 0,
    4608, 2592). Unul nou scrie modul si ScalerCrop-ul; jumatate de
    geometrie = refuz. Si fisierul din repo e unul vechi."""
    cal = real_like_calibration()
    assert cal.recorded_geometry() == (LEGACY_CAL_MODE, LEGACY_CAL_CROP, True)
    tmp = tempfile.mkdtemp()
    p = os.path.join(tmp, 'c.yaml')
    cal.set_geometry((1536, 864), CROP).save(p)
    back = CameraCalibration.load(p)
    assert back.recorded_geometry() == ((1536, 864), CROP, False), back.meta
    back.meta.pop('scaler_crop')
    try:
        back.recorded_geometry()
        assert False, "geometrie pe jumatate acceptata"
    except ValueError as e:
        assert 'jumatate' in str(e)
    repo = CameraCalibration.load(REPO_CAL)
    assert repo.recorded_geometry()[2] is True, "fisierul din repo are deja geometrie?"
    return "fara camp -> 2304 camp intreg; salvat/reincarcat; jumatate -> refuz"


def test_focala_geometrica_de_referinta_tine_cont_de_decupaj():
    """Garda de plauzibilitate din calibrate_camera: modul decupat are
    ACEEASI focala in pixeli ca 2304 camp intreg (aceiasi pixeli binned).
    Comparata cu 102 grade pe 1536 px ar parea +67% si ar fi refuzata."""
    f_full = geometric_focal_px(2304, FULL)
    f_crop = geometric_focal_px(1536, CROP)
    assert abs(f_full - f_crop) < 1e-9 and 930 < f_full < 936, (f_full, f_crop)
    assert abs(geometric_focal_px(2304) - CameraCalibration.geometric(2304, 1296).fx) < 1e-9
    return f"{f_full:.0f} px in ambele moduri (1038 masurat: +11%)"


# --- camera: modul fixat, citit inapoi ------------------------------------------

def test_camera_fixeaza_modul_explicit_si_citeste_geometria():
    """PiCameraSource cere modul prin `sensor=` (output_size + bit_depth),
    il cauta intai in sensor_modes, il citeste inapoi din
    camera_configuration() si ScalerCrop-ul din metadate. Si cadrul de
    scoring are modul fixat."""
    from nova.detector_pi import PiCameraSource
    with fake_camera() as cam, quiet():
        src = PiCameraSource(size=(1280, 720), sensor_mode=(1536, 864))
        pc = cam.instances[-1]
        video = pc.configured[-1]
        assert video['sensor'] == {'output_size': (1536, 864), 'bit_depth': 10}, video
        assert src.still_cfg['sensor'] == {'output_size': (4608, 2592), 'bit_depth': 10}
        assert pc.modes_read == 1 and src.mode_info['crop_limits'] == CROP
        assert src.geometry == CameraGeometry((1536, 864), CROP, (1280, 720))
        gray, t = src.read()
        assert gray.shape == (720, 1280) and abs(time.monotonic() - t) < 1.0
        src.close()
        assert pc.closed
    with fake_camera(), quiet():
        full = PiCameraSource(size=(1280, 720), sensor_mode=(2304, 1296))
        assert full.geometry.scaler_crop == FULL
        full.close()
    return "sensor={output_size, bit_depth} trimis; mod si ScalerCrop citite inapoi"


def test_camera_refuza_modul_diferit_inexistent_sau_fara_ScalerCrop():
    from nova.detector_pi import PiCameraSource
    # 1. libcamera ruleaza alt mod decat cel cerut -> eroare, camera inchisa
    with fake_camera(force_mode=(1536, 864)) as cam, quiet():
        try:
            PiCameraSource(size=(1280, 720), sensor_mode=(2304, 1296))
            assert False, "mod diferit acceptat"
        except CameraModeError as e:
            assert 'difera de cel cerut' in str(e), str(e)
        assert cam.instances[-1].closed, "camera ramasa deschisa dupa refuz"
    # 2. mod care nu exista pe senzor -> eroare inainte de configure
    with fake_camera() as cam, quiet():
        try:
            PiCameraSource(size=(1280, 720), sensor_mode=(1920, 1080))
            assert False, "mod inexistent acceptat"
        except CameraModeError as e:
            assert 'nu exista pe senzor' in str(e) and '1536x864/10bit' in str(e)
        assert cam.instances[-1].configured == [] and cam.instances[-1].closed
    # 3. fara ScalerCrop in metadate -> nu se poate verifica -> eroare
    with fake_camera(no_crop=True) as cam, quiet():
        try:
            PiCameraSource(size=(1280, 720), sensor_mode=(1536, 864))
            assert False, "fara ScalerCrop acceptat"
        except CameraModeError as e:
            assert 'ScalerCrop' in str(e)
        assert cam.instances[-1].closed
    # 4. fara mod deloc -> eroare, camera nici nu se deschide
    with fake_camera() as cam:
        try:
            PiCameraSource(size=(1280, 720))
            assert False, "fara mod acceptat"
        except CameraModeError:
            pass
        assert cam.instances == []
    return "mod diferit / inexistent / fara ScalerCrop / lipsa -> CameraModeError, camera inchisa"


def test_config_modul_e_obligatoriu_si_validat():
    try:
        sensor_mode_from_config({})
        assert False
    except CameraModeError as e:
        assert 'lipseste `sensor_mode`' in str(e)
    for rau in ([1920, 1080], [1536], 'mare', [0, 864]):
        try:
            sensor_mode_from_config({'sensor_mode': rau})
            assert False, rau
        except CameraModeError:
            pass
    assert sensor_mode_from_config({'sensor_mode': '2304x1296'}) == (2304, 1296)
    assert output_size_from_config({}, (1536, 864)) == (1536, 864)
    assert nova_config.DEFAULTS['sensor_mode'] is None, "modul are implicit in cod"
    cfg = nova_config.load()
    mode = sensor_mode_from_config(cfg)
    out = output_size_from_config(cfg, mode)
    assert (mode, out) == ((1536, 864), (1280, 720)), (mode, out)
    c = calibration_for(CameraCalibration.load(REPO_CAL),
                        CameraGeometry(mode, CROP, out))
    assert c.kind == 'derivata+scalata' and abs(c.fx - 864.9) < 0.1, (c.kind, c.fx)
    return f"repo: 1536x864 -> 1280x720, calibrarea din repo derivata+scalata fx {c.fx:.1f}"


def test_build_pi_detector_foloseste_calibrarea_geometriei_efective():
    """Constructorul de bord: config -> camera in modul cerut -> calibrarea
    rezolvata pe geometria CITITA (nu pe cea presupusa). Cu o calibrare
    care nu se poate folosi, camera se inchide si eroarea e ValueError
    (refuz curat al aplicatiei, nu crash)."""
    from nova.detector_pi import build_pi_detector
    cfg = nova_config.load()
    with fake_camera() as cam, quiet():
        det = build_pi_detector(cfg, verbose=True)
        try:
            c = det.det.calib
            assert c.kind == 'derivata+scalata' and abs(c.fx - 864.9) < 0.1, (c.kind, c.fx)
            assert det.det.cam.width_px == 1280.0
            assert det.source.geometry.sensor_mode == (1536, 864)
        finally:
            det.stop()
    tmp = tempfile.mkdtemp()
    nat = os.path.join(tmp, 'crop.yaml')
    calibration_for(real_like_calibration(),
                    CameraGeometry((1536, 864), CROP, (1536, 864))).save(nat)
    cfg2 = dict(cfg, camera_calibration=nat, sensor_mode=[2304, 1296])
    with fake_camera() as cam, quiet():
        try:
            build_pi_detector(cfg2, verbose=False)
            assert False, "calibrare de decupaj folosita pe camp intreg"
        except ValueError as e:
            assert isinstance(e, CalibrationMismatch)
        assert cam.instances[-1].closed
    return f"geometrie citita -> {c.kind}, fx {c.fx:.1f}; nepotrivire -> refuz, camera inchisa"


TESTS = [
    ('situatia gasita pe Pi: preflight PICAT', test_situatia_gasita_pe_Pi_preflight_PICAT),
    ('scalarea doar in acelasi mod si decupaj',
     test_scalarea_acceptata_doar_in_acelasi_mod_si_decupaj),
    ('derivarea pentru modul decupat, pe adevar',
     test_derivarea_pentru_modul_decupat_pe_adevar),
    ('derivarea refuzata in orice alt caz', test_derivarea_refuzata_in_orice_alt_caz),
    ('calibrare fara geometrie = 2304 camp intreg',
     test_calibrarea_fara_geometrie_e_2304_camp_intreg_si_se_salveaza),
    ('focala geometrica tine cont de decupaj',
     test_focala_geometrica_de_referinta_tine_cont_de_decupaj),
    ('camera: modul fixat explicit, geometria citita',
     test_camera_fixeaza_modul_explicit_si_citeste_geometria),
    ('camera: mod diferit / inexistent / fara ScalerCrop -> eroare',
     test_camera_refuza_modul_diferit_inexistent_sau_fara_ScalerCrop),
    ('config: sensor_mode obligatoriu si validat', test_config_modul_e_obligatoriu_si_validat),
    ('build_pi_detector: calibrarea geometriei efective',
     test_build_pi_detector_foloseste_calibrarea_geometriei_efective),
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
