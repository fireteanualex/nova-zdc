#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru senzorul de camera din simulare (I2).

    python3 tools/test_camera_model.py

Modelul de vehicul NU e in repo: depinde de `config/camera_pi.yaml`, care nu
exista pana la calibrarea reala. Deci testele genereaza in directoare
temporare, dintr-o calibrare sintetica, si verifica **generatorul** - nu un
artefact commit-uit.

Doua lucruri fac cea mai mare parte din munca:

  - intrinsecii din SDF trebuie sa fie cei din calibrare, iar `horizontal_fov`
    derivat din fx. Scrise independent, ar putea sa nu se potriveasca, iar
    rezultatul depinde de ordinea in care le citeste Gazebo.
  - la `--scale`, intrinsecii se scaleaza si distorsiunea NU. Invers, ar fi
    o eroare tacuta care arata ca o calibrare proasta.
"""

import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                          # noqa: E402

import make_camera_model as mc                              # noqa: E402
import make_marker_model as mm                              # noqa: E402
from nova.detector_pi import CameraCalibration              # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Calibrare sintetica, cu valori ASIMETRICE pe fiecare axa: fx != fy,
#: cx != w/2, cy != h/2. Cu valori simetrice, o inversare x/y ar trece
#: neobservata.
K = np.array([[937.0, 0.0, 1139.4],
              [0.0, 933.7, 649.0],
              [0.0, 0.0, 1.0]])
DIST = np.array([-0.0529, 0.0696, 0.0011, -0.0007, -0.0295])
W, H = 2304, 1296


def calib_file(tmp, **kw):
    d = dict(rms=0.21, n_images=25, source='sintetic pentru test')
    d.update(kw)
    cal = CameraCalibration(K, DIST, W, H, **d)
    cale = os.path.join(tmp, 'cam.yaml')
    cal.save(cale)
    return cale


def genereaza(tmp, scale=1.0, **kw):
    """(dir_model, radacina_sdf, intrinseci)."""
    cale = kw.pop('calib', None) or calib_file(tmp)
    intr, _cal = mc.load_intrinsics(cale, scale)
    base = open(mc.find_base_model()).read()
    out = os.path.join(tmp, 'models', mc.MODEL_NAME)
    mc.write_model(out, base, intr, 'test', sursa='test', **kw)
    return out, ET.parse(os.path.join(out, 'model.sdf')).getroot(), intr


def _cam(root):
    for link in root.find('model').findall('link'):
        s = link.find('sensor')
        if s is not None and s.get('name') == mc.SENSOR_NAME:
            return link, s
    raise AssertionError('senzorul down_cam nu exista in model')


# --- calibrarea e obligatorie ------------------------------------------------

def test_NEGATIV_refuza_fara_calibrare_reala():
    """E1.2, acelasi prag ca detectorul de bord: o focala geometrica pusa ca
    sa treaca ceva ar face simularea sa masoare alta camera."""
    tmp = tempfile.mkdtemp()
    for eticheta, kw in (
            ('lipsa', None),
            ('geometrica', 'GEO'),
            ('rms mare', dict(rms=1.4)),
            ('n_images 0', dict(n_images=0, rms=0.2))):
        if kw is None:
            cale = os.path.join(tmp, 'nu_exista.yaml')
        elif kw == 'GEO':
            cale = os.path.join(tmp, 'geo.yaml')
            CameraCalibration.geometric(W, H).save(cale)
        else:
            cale = calib_file(tmp, **kw)
        try:
            mc.load_intrinsics(cale)
            raise AssertionError(f"{eticheta}: acceptata")
        except (FileNotFoundError, ValueError):
            pass

    # iar CLI-ul iese cu 2, nu cu un traceback
    import subprocess
    r = subprocess.run(
        [sys.executable, os.path.join(REPO, 'tools', 'make_camera_model.py'),
         '--calib', os.path.join(tmp, 'nu_exista.yaml'), '--no-patch-world'],
        capture_output=True, text=True, timeout=60)
    assert r.returncode == 2, r.returncode
    assert 'NU GENEREZ' in r.stdout and 'E1.2' in r.stdout
    return "4 calibrari refuzate; CLI iese cu 2, nu cu traceback"


# --- intrinseci --------------------------------------------------------------

def test_intrinsecii_vin_din_calibrare():
    """Nicio valoare scrisa de mana: ce e in YAML ajunge in SDF."""
    tmp = tempfile.mkdtemp()
    _d, root, intr = genereaza(tmp)
    _link, sen = _cam(root)
    lens = sen.find('camera/lens/intrinsics')
    for camp, astept in (('fx', K[0, 0]), ('fy', K[1, 1]),
                         ('cx', K[0, 2]), ('cy', K[1, 2])):
        got = float(lens.find(camp).text)
        assert abs(got - astept) < 1e-3, f"{camp}: {got} != {astept}"

    img = sen.find('camera/image')
    assert int(img.find('width').text) == W
    assert int(img.find('height').text) == H
    assert img.find('format').text.strip() == 'L8', (
        "format color: randare si transport de 3x, degeaba")

    dist = sen.find('camera/distortion')
    for camp, astept in (('k1', DIST[0]), ('k2', DIST[1]),
                         ('p1', DIST[2]), ('p2', DIST[3]), ('k3', DIST[4])):
        got = float(dist.find(camp).text)
        assert abs(got - astept) < 1e-6, f"{camp}: {got} != {astept}"
    del intr
    return "fx/fy/cx/cy si k1,k2,k3,p1,p2 identice cu YAML-ul; format L8"


def test_hfov_derivat_din_fx():
    """Scrise independent, `<horizontal_fov>` si `<intrinsics>` pot sa nu se
    potriveasca, iar rezultatul depinde de ordinea de citire din Gazebo."""
    tmp = tempfile.mkdtemp()
    _d, root, _i = genereaza(tmp)
    _link, sen = _cam(root)
    hfov = float(sen.find('camera/horizontal_fov').text)
    fx = float(sen.find('camera/lens/intrinsics/fx').text)
    w = int(sen.find('camera/image/width').text)
    astept = 2.0 * math.atan(w / (2.0 * fx))
    assert abs(hfov - astept) < 1e-6, f"{hfov} != {astept}"
    # si e aproape de fisa tehnica (102 deg), fara sa fie luat din ea
    assert 95.0 < math.degrees(hfov) < 110.0, math.degrees(hfov)
    return (f"hfov {math.degrees(hfov):.2f} deg = 2*atan(w/2fx), "
            f"nu preluat din fisa")


def test_scale_muta_intrinsecii_dar_NU_distorsiunea():
    """k1..k3 sunt definiti pe coordonate normalizate: adimensionali."""
    tmp = tempfile.mkdtemp()
    _d, r1, _i1 = genereaza(tmp, scale=1.0)
    tmp2 = tempfile.mkdtemp()
    _d2, r2, _i2 = genereaza(tmp2, scale=0.5, calib=calib_file(tmp2))

    _l1, s1 = _cam(r1)
    _l2, s2 = _cam(r2)
    for camp in ('fx', 'fy', 'cx', 'cy'):
        a = float(s1.find(f'camera/lens/intrinsics/{camp}').text)
        b = float(s2.find(f'camera/lens/intrinsics/{camp}').text)
        assert abs(b - a * 0.5) < 1e-3, f"{camp}: {b} != {a}*0.5"
    assert int(s2.find('camera/image/width').text) == W // 2
    assert int(s2.find('camera/image/height').text) == H // 2

    for camp in ('k1', 'k2', 'k3', 'p1', 'p2'):
        a = float(s1.find(f'camera/distortion/{camp}').text)
        b = float(s2.find(f'camera/distortion/{camp}').text)
        assert abs(a - b) < 1e-9, (
            f"{camp} s-a scalat: {a} -> {b}. Coeficientii de distorsiune "
            f"sunt adimensionali.")

    # hfov nu se schimba: si w, si fx s-au injumatatit
    h1 = float(s1.find('camera/horizontal_fov').text)
    h2 = float(s2.find('camera/horizontal_fov').text)
    assert abs(h1 - h2) < 1e-6, f"hfov s-a schimbat: {h1} -> {h2}"
    return "fx/fy/cx/cy x0.5, w/h x0.5, distorsiune neschimbata, hfov identic"


def test_pragurile_se_muta_la_alta_rezolutie():
    """Capcana principala a lui --scale: pragurile sunt in PIXELI, deci la
    alta rezolutie corespund altor altitudini."""
    tmp = tempfile.mkdtemp()
    cale = calib_file(tmp)
    i1, _ = mc.load_intrinsics(cale, 1.0)
    i2, _ = mc.load_intrinsics(cale, 0.5)
    t1 = mc.threshold_table(i1)
    t2 = mc.threshold_table(i2)

    # la rezolutie plina, cifrele trebuie sa se potriveasca cu §2 / §5.2
    assert abs(t1['px_at'][5.0] - 90) < 2, t1['px_at'][5.0]
    assert abs(t1['px_at'][12.0] - 37) < 2, t1['px_at'][12.0]
    assert 0.35 < t1['frame_limit_alt_m'] < 0.40, t1['frame_limit_alt_m']

    # la jumatate: acelasi prag in pixeli, alta altitudine
    assert abs(t2['scoring_alt_m'] - t1['scoring_alt_m'] * 0.5) < 1e-6, (
        f"{t2['scoring_alt_m']} vs {t1['scoring_alt_m']}")
    assert abs(t2['px_at'][5.0] - t1['px_at'][5.0] * 0.5) < 1e-6
    return (f"plin: 90 px la 5 m, incadrare sub {t1['frame_limit_alt_m']:.2f} m; "
            f"la scale 0.5, scoring se muta de la "
            f"{t1['scoring_alt_m']:.2f} la {t2['scoring_alt_m']:.2f} m")


# --- modelul ----------------------------------------------------------------

def test_modelul_e_DERIVAT_nu_rescris():
    """§5.31: plugin-ul ArduPilot, lift-drag pe fiecare rotor si referintele
    de link trebuie sa ramana EXACT ce foloseste ardupilot_gazebo."""
    tmp = tempfile.mkdtemp()
    _d, root, _i = genereaza(tmp)
    baza = ET.parse(mc.find_base_model()).getroot()

    p_baza = sorted(p.get('filename') or p.get('name')
                    for p in baza.find('model').findall('plugin'))
    p_nou = sorted(p.get('filename') or p.get('name')
                   for p in root.find('model').findall('plugin'))
    assert p_baza == p_nou, f"plugin-uri diferite:\n  {p_baza}\n  {p_nou}"
    assert any('ArduPilot' in (x or '') for x in p_nou), p_nou

    # includerile raman aceleasi
    u_baza = {i.find('uri').text.strip()
              for i in baza.find('model').findall('include')}
    u_nou = {i.find('uri').text.strip()
             for i in root.find('model').findall('include')}
    assert u_baza == u_nou, f"{u_baza} vs {u_nou}"

    # exact un link si un joint in plus
    l_baza = {l.get('name') for l in baza.find('model').findall('link')}
    l_nou = {l.get('name') for l in root.find('model').findall('link')}
    assert l_nou - l_baza == {mc.CAM_LINK}, l_nou - l_baza
    j_baza = {j.get('name') for j in baza.find('model').findall('joint')}
    j_nou = {j.get('name') for j in root.find('model').findall('joint')}
    assert j_nou - j_baza == {mc.CAM_LINK + '_joint'}, j_nou - j_baza
    return (f"{len(p_nou)} plugin-uri identice, aceleasi includeri, "
            f"+1 link si +1 joint")


def test_montajul_si_orientarea():
    tmp = tempfile.mkdtemp()
    _d, root, _i = genereaza(tmp, mount_z=0.0745)
    link, _s = _cam(root)
    poza = [float(x) for x in link.find('pose').text.split()]
    assert abs(poza[2] + 0.0745) < 1e-9, f"z = {poza[2]}, asteptat -0.0745"
    assert abs(poza[4] - math.pi / 2) < 1e-6, (
        f"pitch = {poza[4]}, asteptat +pi/2 (axa optica pe -Z)")
    assert poza[0] == poza[1] == 0.0

    # parametrizabil
    tmp2 = tempfile.mkdtemp()
    _d2, r2, _i2 = genereaza(tmp2, mount_z=0.12, calib=calib_file(tmp2))
    l2, _ = _cam(r2)
    assert abs(float(l2.find('pose').text.split()[2]) + 0.12) < 1e-9

    # jointul prinde de base_link, ca gimbalul
    joint = [j for j in root.find('model').findall('joint')
             if j.get('name') == mc.CAM_LINK + '_joint'][0]
    assert joint.get('type') == 'fixed', joint.get('type')
    assert joint.find('parent').text.strip() == 'iris_with_standoffs::base_link'
    assert joint.find('child').text.strip() == mc.CAM_LINK
    return "z = -0.0745 (parametrizabil), pitch +90 deg, joint fix pe base_link"


def test_modelul_nu_e_gol():
    """Bug prins in dezvoltare: `open(f,'w').write(open(f).read())`
    trunchiaza fisierul INAINTE de a-l citi, deci iesea de zero octeti."""
    tmp = tempfile.mkdtemp()
    d, _r, _i = genereaza(tmp)
    cale = os.path.join(d, 'model.sdf')
    n = os.path.getsize(cale)
    assert n > 5000, f"model.sdf are {n} octeti"
    text = open(cale).read()
    assert '{sursa}' not in text, "sablonul {sursa} a ramas neinlocuit"
    assert 'ArduPilotPlugin' in text and mc.SENSOR_NAME in text
    return f"{n} octeti, fara sabloane neinlocuite"


def test_patch_world_e_idempotent():
    tmp = tempfile.mkdtemp()
    sim = os.path.join(tmp, 'sim')
    os.makedirs(os.path.join(sim, 'worlds'))
    base = open(mm.find_base_world()).read()
    lume = os.path.join(sim, 'worlds', 'nova_marker.sdf')
    open(lume, 'w').write(
        mm.build_world(base, 2.0, 1.5, 135.0, 45.0,
                       uri='/x/aruco_26').replace('{sursa}', 'test'))

    t1, m1 = mc.patch_world(lume, os.path.join(sim, 'models'))
    assert 'inlocuit' in m1, m1
    assert 'model://iris_with_gimbal' not in t1
    assert '<name>iris_with_gimbal</name>' in t1, (
        "fara <name>, modelul si-ar schimba numele in lume si plugin-ul "
        "ArduPilot ar cauta alte entitati")
    t2, m2 = mc.patch_world(lume, os.path.join(sim, 'models'))
    assert t1 == t2, "a doua rulare a schimbat lumea"
    assert 'deja' in m2, m2

    # lume inexistenta: mesaj, nu exceptie
    _t, m3 = mc.patch_world(os.path.join(tmp, 'nimic.sdf'),
                            os.path.join(sim, 'models'))
    assert 'nu exista' in m3 and 'make_marker_model' in m3
    return "prima rulare inlocuieste, a doua nu schimba nimic; lipsa = mesaj"


def test_nu_e_commit_uit_cu_intrinseci_provizorii():
    """`sim/models/iris_nova/` depinde de o calibrare care inca nu exista.
    Daca ar fi in repo, ar contine intrinseci inventati si nimeni nu ar
    observa - deci e ignorat de git pana la calibrarea reala."""
    cale = os.path.join(REPO, 'sim', 'models', mc.MODEL_NAME)
    gitignore = open(os.path.join(REPO, '.gitignore')).read()
    assert 'iris_nova' in gitignore, (
        "sim/models/iris_nova nu e in .gitignore; s-ar putea commit-ui cu "
        "intrinseci provizorii")
    if os.path.exists(cale):
        import subprocess
        r = subprocess.run(['git', '-C', REPO, 'ls-files', '--error-unmatch',
                            os.path.relpath(os.path.join(cale, 'model.sdf'),
                                            REPO)],
                           capture_output=True, text=True)
        assert r.returncode != 0, "model.sdf e urmarit de git"
    return "ignorat de git pana cand exista config/camera_pi.yaml"



def test_calibrarea_provizorie_e_sim_only():
    """Ocolirea gardii E1.2 trebuie ceruta EXPLICIT si sa nu atinga zborul.

    Acelasi tipar ca E0 in fake_detector.py: garda exista ca sa protejeze un
    vehicul real; in simulare vehiculul e Gazebo, deci se ocoleste - dar din
    linia de comanda, anuntat, nu prin editarea unui config."""
    cale = os.path.join(REPO, 'config', 'camera_sim.yaml')
    if not os.path.exists(cale):
        return 'config/camera_sim.yaml nu exista (optional)'

    # fara --provisional: refuzata
    try:
        mc.load_intrinsics(cale)
        raise AssertionError('calibrarea provizorie a trecut fara --provisional')
    except ValueError:
        pass
    # cu --provisional: merge
    intr, cal = mc.load_intrinsics(cale, provisional=True)
    assert intr['w'] == 2304 and intr['h'] == 1296, intr
    assert not cal.is_real(), (
        'calibrarea provizorie se declara reala; pusa in config/camera_pi.yaml '
        'ar fi acceptata pentru ZBOR')
    assert 'PROVIZORIE' in (cal.source or '').upper(), cal.source

    # si NU e fisierul pe care il citeste zborul
    from nova import config as nova_config
    cfg = nova_config.load()
    zbor = nova_config.resolve(cfg, 'camera_calibration')
    assert os.path.basename(zbor) != 'camera_sim.yaml', (
        f"config/nova.json arata spre {zbor}: calea de ZBOR foloseste "
        f"calibrarea provizorie")

    # CLI: fara flag, cod 2 si indicatie
    import subprocess
    r = subprocess.run(
        [sys.executable, os.path.join(REPO, 'tools', 'make_camera_model.py'),
         '--calib', cale, '--no-patch-world'],
        capture_output=True, text=True, timeout=60)
    assert r.returncode == 2, r.returncode
    assert '--provisional' in r.stdout, 'refuzul nu spune cum se continua'
    return "refuzata fara flag; cu flag merge; zborul citeste alt fisier"


def test_pragul_de_rms_nu_se_schimba_pentru_zbor():
    """0.85 px depaseste pragul. Ridicat global, ar slabi si calea de zbor."""
    from nova.detector_pi import MAX_REPROJ_ERR_PX
    assert MAX_REPROJ_ERR_PX == 0.5, (
        f"pragul global e {MAX_REPROJ_ERR_PX}; ridicat, detectorul de bord ar "
        f"accepta calibrari mai proaste fara ca cineva sa decida asta")

    tmp = tempfile.mkdtemp()
    slaba = calib_file(tmp, rms=0.85)
    try:
        mc.load_intrinsics(slaba)
        raise AssertionError('rms 0.85 acceptat la pragul implicit')
    except ValueError:
        pass
    # ridicat DOAR pentru aceasta rulare
    intr, _c = mc.load_intrinsics(slaba, max_rms=0.9)
    assert intr['w'] == 2304
    return "prag global 0.5 neatins; --max-rms 0.9 ridica doar rularea curenta"


TESTS = [
    ('NEGATIV: refuza fara calibrare reala',
     test_NEGATIV_refuza_fara_calibrare_reala),
    ('intrinsecii vin din calibrare', test_intrinsecii_vin_din_calibrare),
    ('hfov derivat din fx', test_hfov_derivat_din_fx),
    ('scale muta intrinsecii, NU distorsiunea',
     test_scale_muta_intrinsecii_dar_NU_distorsiunea),
    ('pragurile se muta la alta rezolutie',
     test_pragurile_se_muta_la_alta_rezolutie),
    ('modelul e DERIVAT, nu rescris', test_modelul_e_DERIVAT_nu_rescris),
    ('montajul si orientarea', test_montajul_si_orientarea),
    ('modelul nu e gol', test_modelul_nu_e_gol),
    ('patch_world e idempotent', test_patch_world_e_idempotent),
    ('nu e commit-uit cu intrinseci provizorii',
     test_nu_e_commit_uit_cu_intrinseci_provizorii),
    ('calibrarea provizorie e sim-only',
     test_calibrarea_provizorie_e_sim_only),
    ('pragul de rms nu se schimba pentru zbor',
     test_pragul_de_rms_nu_se_schimba_pentru_zbor),
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
        except Exception as e:                              # noqa: BLE001
            fails += 1
            import traceback
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
            traceback.print_exc(limit=3)
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
