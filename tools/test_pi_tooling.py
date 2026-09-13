#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru uneltele de Pi (runda 5: G1-G4).

    python3 tools/test_pi_tooling.py

Ruleaza pe desktop, fara Pi si fara FC. Ce se poate verifica aici e logica:
gardurile de pornire, bariera de comanda, structura datelor, codurile de
iesire, matematica rezumatelor. Ce NU se poate - camera reala, temperatura
reala, systemd care chiar porneste ceva - e marcat explicit in
RAPORT_RUNDA5.md, nu simulat cu un test care trece.

Fiecare unealta are cel putin un caz NEGATIV. Un test care nu poate esua nu
e test (§5.11).
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                          # noqa: E402

import nova_service as svc                                  # noqa: E402
import preflight_check as pf                                # noqa: E402
import run_e2                                               # noqa: E402
import synthetic as syn                                     # noqa: E402
from check_params import parse_parm                         # noqa: E402
from nova.detector_pi import (ArraySource, CameraCalibration)  # noqa: E402
from nova.vehicle import Vehicle                            # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SETUP = os.path.join(REPO, 'tools', 'setup_pi.sh')

K, DIST, _WH = syn.imx708(2304, 1296)
W, H = 2304, 1296


# --- ajutoare ---------------------------------------------------------------

def real_calibration(**kw):
    """O calibrare care trece toate gardurile."""
    d = dict(rms=0.105, n_images=25, source='charuco 25 poze',
             meta={'target_type': 'charuco', 'square_mm_measured': 37.0})
    d.update(kw)
    return CameraCalibration(K, DIST, W, H, **d)


def marker_frames(depths, size=(W, H)):
    out = []
    for i, z in enumerate(depths):
        img, _ = syn.render_marker(K, DIST, size,
                                   syn.rot(0.02 * i, 0.01 * i, 0.0),
                                   (0.0, 0.0, float(z)))
        out.append(img)
    return out


def make_cfg(tmp, cal=None):
    """(cale_config, cale_calibrare). Scrie ambele pe disc."""
    cal = cal or real_calibration()
    cal_path = os.path.join(tmp, 'camera_pi.yaml')
    cal.save(cal_path)
    cfg_path = os.path.join(tmp, 'nova.json')
    with open(cfg_path, 'w') as f:
        json.dump({'autonomy_enabled': False, 'marker_id': 26,
                   'marker_size_m': 0.48, 'camera_calibration': cal_path,
                   'roi_below_m': 5.0, 'roi_size_px': [640, 480]}, f)
    return cfg_path, cal_path


def run_setup(args, env=None):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(['bash', SETUP] + args, capture_output=True,
                          text=True, env=e, timeout=120)


def os_release_fixture(tmp, name, ident, codename, pretty):
    p = os.path.join(tmp, name)
    with open(p, 'w') as f:
        f.write(f'PRETTY_NAME="{pretty}"\nID={ident}\n'
                f'VERSION_CODENAME={codename}\n')
    return p


# --- G1: setup_pi.sh + requirements ----------------------------------------

def test_G1_poarta_de_platforma():
    """Bookworm trece; Bullseye si Ubuntu sunt refuzate cu motiv."""
    tmp = tempfile.mkdtemp()
    model = os.path.join(tmp, 'model')
    with open(model, 'w') as f:
        f.write('Raspberry Pi 4 Model B Rev 1.5\x00')
    cazuri = [
        ('trixie', 'debian', 'trixie', 'Debian GNU/Linux 13 (trixie)', 0),
        ('bookworm', 'debian', 'bookworm', 'Debian GNU/Linux 12 (bookworm)', 0),
        ('bullseye', 'raspbian', 'bullseye', 'Raspbian GNU/Linux 11', 1),
        ('forky', 'debian', 'forky', 'Debian GNU/Linux 14', 1),
        ('ubuntu', 'ubuntu', 'jammy', 'Ubuntu 22.04.5 LTS', 1),
    ]
    vazute = []
    for name, ident, code, pretty, want_rc in cazuri:
        osr = os_release_fixture(tmp, name, ident, code, pretty)
        r = run_setup(['--dry-run'], {
            'NOVA_OS_RELEASE': osr, 'NOVA_MODEL_FILE': model,
            'NOVA_VENV': os.path.join(tmp, f'venv-{name}')})
        assert r.returncode == want_rc, (
            f"{name}: exit {r.returncode}, asteptat {want_rc}\n{r.stdout}\n{r.stderr}")
        if want_rc:
            assert 'OPRIT' in r.stderr, f"{name}: refuz fara motiv explicit"
        vazute.append(f"{name}->{r.returncode}")
    return ', '.join(vazute)


def test_G1_ordinea_pasilor():
    """apt INAINTE de venv, venv cu --system-site-packages, pip DUPA."""
    tmp = tempfile.mkdtemp()
    model = os.path.join(tmp, 'model')
    open(model, 'w').write('Raspberry Pi 4 Model B\x00')
    osr = os_release_fixture(tmp, 'os', 'debian', 'bookworm', 'Debian 12')
    r = run_setup(['--dry-run'], {
        'NOVA_OS_RELEASE': osr, 'NOVA_MODEL_FILE': model,
        'NOVA_VENV': os.path.join(tmp, 'venv')})
    out = r.stdout
    i_apt = out.find('apt-get install')
    i_venv = out.find('venv --system-site-packages')
    i_pip = out.find('pip install -r')
    assert i_apt > 0, f"nu instaleaza pachete apt:\n{out}"
    assert i_venv > i_apt, "venv-ul se creeaza inaintea pachetelor apt"
    assert i_pip > i_venv, "pip ruleaza inaintea venv-ului"
    assert 'python3-picamera2' in out and 'python3-libcamera' in out
    return "apt -> venv(--system-site-packages) -> pip, in ordine"


def test_G1_NEGATIV_venv_fara_system_site_packages():
    """Un venv existent creat gresit e refuzat, nu refolosit tacit."""
    tmp = tempfile.mkdtemp()
    bad = os.path.join(tmp, 'bad')
    subprocess.run([sys.executable, '-m', 'venv', bad], check=True,
                   capture_output=True)
    model = os.path.join(tmp, 'model')
    open(model, 'w').write('Raspberry Pi 4\x00')
    osr = os_release_fixture(tmp, 'os', 'debian', 'bookworm', 'Debian 12')
    r = run_setup(['--dry-run'], {'NOVA_OS_RELEASE': osr,
                                  'NOVA_MODEL_FILE': model, 'NOVA_VENV': bad})
    assert r.returncode == 1, f"venv gresit acceptat (exit {r.returncode})"
    assert 'system-site-packages' in r.stderr
    # si varianta corecta e acceptata
    good = os.path.join(tmp, 'good')
    subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages',
                    good], check=True, capture_output=True)
    r2 = run_setup(['--dry-run'], {'NOVA_OS_RELEASE': osr,
                                   'NOVA_MODEL_FILE': model,
                                   'NOVA_VENV': good})
    assert r2.returncode == 0, f"venv corect refuzat:\n{r2.stderr}"
    return "venv fara flag: refuzat cu motiv; cu flag: acceptat"


def test_G1_NEGATIV_verificarea_finala_chiar_verifica():
    """--verify-only pica pe un venv fara picamera2.

    Fara cazul asta, "verificarea a trecut" ar putea insemna la fel de bine
    ca verificarea nu masoara nimic."""
    tmp = tempfile.mkdtemp()

    # (a) venv gol: picamera2 lipseste, deci verificarea trebuie sa pice.
    gol = os.path.join(tmp, 'gol')
    subprocess.run([sys.executable, '-m', 'venv', gol], check=True,
                   capture_output=True)
    r = run_setup(['--verify-only'], {'NOVA_VENV': gol})
    assert r.returncode == 1, "verificarea a trecut pe un venv fara picamera2"
    assert 'picamera2' in r.stdout

    # (b) capcana propriu-zisa: numpy INSTALAT IN VENV peste cel din sistem.
    # Nu instalam nimic (ar cere retea) - punem un modul-momeala in
    # site-packages-ul venv-ului, exact unde ar ajunge pip.
    ssp = os.path.join(tmp, 'ssp')
    subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages',
                    ssp], check=True, capture_output=True)
    sp = subprocess.run(
        [os.path.join(ssp, 'bin', 'python'), '-c',
         'import sysconfig; print(sysconfig.get_paths()["purelib"])'],
        capture_output=True, text=True, check=True).stdout.strip()
    os.makedirs(sp, exist_ok=True)
    with open(os.path.join(sp, 'numpy.py'), 'w') as f:
        f.write("__version__ = '2.9.9-momeala'\n")
    r2 = run_setup(['--verify-only'], {'NOVA_VENV': ssp})
    assert 'numpy e instalat IN venv' in r2.stdout, (
        f"numpy din venv nu a fost semnalat - exact capcana pe care o "
        f"cauta verificarea:\n{r2.stdout}")
    assert r2.returncode == 1, "numpy in venv nu a dus la cod de iesire 1"
    return ("venv fara picamera2 -> ESEC; numpy pus in venv -> semnalat "
            "si ESEC")


def _requirements(name):
    linii = [l.split('#')[0].strip()
             for l in open(os.path.join(REPO, name))]
    return [l for l in linii if l and not l.startswith('-r ')]


def test_H0_requirements_pe_doua_cai():
    """Trixie ia OpenCV din apt; Bookworm din pip, 4.x. numpy niciodata."""
    comun = _requirements('requirements-pi.txt')
    bw = _requirements('requirements-pi-bookworm.txt')
    for eticheta, pachete in (('comun', comun), ('bookworm', bw)):
        nume = {p.split('==')[0].lower() for p in pachete}
        assert 'picamera2' not in nume, f"{eticheta}: picamera2 vine din apt"
        assert 'numpy' not in nume, (
            f"{eticheta}: numpy vine din apt. O versiune pip peste cea de "
            f"sistem sparge simplejpeg (§5.24).")
        for pk in pachete:
            assert '==' in pk, f"{eticheta}: versiune nefixata: {pk}"

    assert not [p for p in comun if p.startswith('opencv')], (
        "requirements-pi.txt nu are voie sa contina OpenCV: pe Trixie vine "
        "din apt")
    ocv = [p for p in bw if p.startswith('opencv')]
    assert len(ocv) == 1, f"asteptam exact un opencv pe Bookworm: {ocv}"
    ver = ocv[0].split('==')[1]
    assert ver.startswith('4.'), (
        f"OpenCV {ver} pe Bookworm: 5.x cere numpy>=2, iar apt da 1.24.2, "
        f"deci pip ar instala numpy in venv si ar sparge picamera2 (§5.24).")
    major, minor = (int(x) for x in ver.split('.')[:2])
    assert (major, minor) >= (4, 7), (
        f"OpenCV {ver} < 4.7: fara cv2.aruco.ArucoDetector")

    # requirements-pi-bookworm.txt trebuie sa includa fisierul comun, altfel
    # pe Bookworm ar lipsi pymavlink.
    assert any(l.strip().startswith('-r ') and 'requirements-pi.txt' in l
               for l in open(os.path.join(REPO,
                                          'requirements-pi-bookworm.txt'))), \
        "requirements-pi-bookworm.txt nu include requirements-pi.txt"
    assert 'system-site-packages' in open(
        os.path.join(REPO, 'requirements-pi.txt')).read()
    return (f"comun: {len(comun)} pachete fara opencv; "
            f"bookworm: +opencv {ver}")


def test_H0_setup_alege_calea_dupa_distributie():
    """Trixie -> apt opencv + requirements-pi.txt;
    Bookworm -> pip opencv prin requirements-pi-bookworm.txt."""
    tmp = tempfile.mkdtemp()
    model = os.path.join(tmp, 'model')
    open(model, 'w').write('Raspberry Pi 4 Model B\x00')
    vazute = {}
    for code in ('trixie', 'bookworm'):
        osr = os_release_fixture(tmp, code, 'debian', code, f"Debian ({code})")
        r = run_setup(['--dry-run'], {
            'NOVA_OS_RELEASE': osr, 'NOVA_MODEL_FILE': model,
            'NOVA_VENV': os.path.join(tmp, f'v-{code}')})
        assert r.returncode == 0, f"{code} refuzat:\n{r.stderr}"
        apt = [l for l in r.stdout.splitlines() if 'apt-get install' in l][0]
        pip = [l for l in r.stdout.splitlines() if 'pip install -r' in l][0]
        vazute[code] = (('python3-opencv' in apt), os.path.basename(pip.split()[-1]))

    assert vazute['trixie'] == (True, 'requirements-pi.txt'), vazute['trixie']
    assert vazute['bookworm'] == (False, 'requirements-pi-bookworm.txt'), \
        vazute['bookworm']
    return (f"trixie: opencv din apt + {vazute['trixie'][1]}; "
            f"bookworm: pip + {vazute['bookworm'][1]}")


def test_H0_verificarea_de_stiva():
    """preflight raporteaza CALEA fiecarui pachet; pe Pi, numpy din venv e ESEC."""
    r = pf.check_stack()
    assert r.status == pf.OK, (
        f"pe desktop, numpy din venv nu e o problema (nu exista picamera2), "
        f"dar a fost raportat ca {r.status}: {r.detail}")
    assert 'numpy' in r.detail and 'cv2' in r.detail
    assert '/' in r.detail, "nu raporteaza CALEA, doar versiunea"

    # acelasi mediu, dar pretinzand ca suntem pe Pi -> devine ESEC
    vechi = pf._model
    try:
        pf._model = lambda path=None: 'Raspberry Pi 4 Model B Rev 1.5'
        r2 = pf.check_stack()
    finally:
        pf._model = vechi
    assert r2.status == pf.ESEC, (
        "pe Pi, numpy incarcat din venv trebuie sa pice: picamera2 si "
        "simplejpeg sunt compilate impotriva celui de sistem")
    assert 'setup_pi.sh' in r2.detail, "refuzul nu spune cum se repara"
    return "desktop: OK cu cale; acelasi mediu pretinzand Pi: ESEC"


# --- G2: serviciul ----------------------------------------------------------

def test_G2_calibrarea_ceruta_la_pornire():
    """Cinci fisiere: unul bun, patru care trebuie refuzate."""
    tmp = tempfile.mkdtemp()
    bun = os.path.join(tmp, 'bun.yaml')
    real_calibration().save(bun)
    cal = svc.check_calibration(bun)
    assert cal.n_images == 25

    geo = os.path.join(tmp, 'geo.yaml')
    CameraCalibration.geometric(W, H).save(geo)

    # Pare reala (n_images si rms plauzibile) dar are focala geometrica si
    # distorsiune zero - cazul pe care CameraCalibration.load NU il prinde.
    fals = os.path.join(tmp, 'fals.yaml')
    CameraCalibration(K, np.zeros(5), W, H, rms=0.2, n_images=25,
                      source='calibrare').save(fals)

    prost = os.path.join(tmp, 'prost.yaml')
    real_calibration(rms=1.4).save(prost)

    refuzate = []
    for eticheta, p in (('lipsa', os.path.join(tmp, 'nuexista.yaml')),
                        ('geometrica', geo), ('dist zero', fals),
                        ('rms mare', prost)):
        try:
            svc.check_calibration(p)
            raise AssertionError(f"{eticheta}: ACCEPTATA, trebuia refuzata")
        except svc.StartupRefusal as e:
            refuzate.append(f"{eticheta}: {str(e).splitlines()[0][:40]}")
    return f"1 acceptata, {len(refuzate)} refuzate (inclusiv dist zero)"


def test_G2_bariera_de_comanda():
    """ReadOnlyVehicle: lista alba, tot restul blocat, inclusiv `m`."""
    scrie = svc.ReadOnlyVehicle.write_paths_allowed(Vehicle)
    assert not scrie, f"metode permise care totusi scriu pe FC: {scrie}"

    class Spion:
        def __init__(self):
            self.comenzi = []
        m = 'conexiune bruta'
        def pump(self):
            return 'telemetrie'
        def alt(self):
            return 4.2
        def request_mode(self, mode):
            self.comenzi.append(mode)
        def send_takeoff(self, a):
            self.comenzi.append('takeoff')
        def update_params(self, now):
            self.comenzi.append('param')
        def send_landing_target(self, *a):
            self.comenzi.append('lt')

    spion = Spion()
    ro = svc.ReadOnlyVehicle(spion)
    assert ro.pump() == 'telemetrie' and ro.alt() == 4.2

    blocate = []
    for nume in ('request_mode', 'send_takeoff', 'send_landing_target',
                 'update_params', 'm', 'set_param', 'send_distance'):
        try:
            getattr(ro, nume)
            raise AssertionError(f"{nume} NU a fost blocat")
        except PermissionError:
            blocate.append(nume)
    try:
        ro.something = 1
        raise AssertionError("scrierea in Vehicle nu a fost blocata")
    except PermissionError:
        pass
    assert not spion.comenzi, f"au ajuns comenzi la vehicul: {spion.comenzi}"
    return (f"{len(blocate)} cai blocate (inclusiv `m` si update_params); "
            f"citirea trece")


def test_G2_ring_buffer():
    """Marginit, si tine si cadrele NEdetectate."""
    ring = svc.FrameRing(maxlen=4)
    frames = marker_frames([3.0, 4.0, 5.0])
    frames.append(np.full((H, W), 100, np.uint8))       # fara marker
    frames += marker_frames([6.0, 7.0])
    src = svc.RingTapSource(ArraySource(frames, fps=30.0), ring)
    while src.read() is not None:
        pass
    assert len(ring) == 4, f"buffer de {len(ring)}, maxlen 4"
    mb = ring.nbytes() / 1e6
    assert 11.0 < mb < 13.0, f"{mb:.1f} MB pentru 4 cadre 2304x1296"
    # ultimele 4 din 6 includ cadrul gol (indexul 3)
    goale = sum(1 for _t, f in ring.buf if float(np.std(f)) < 1.0)
    assert goale == 1, (
        f"{goale} cadre uniforme in buffer; cel fara marker trebuie pastrat "
        f"- 8.3.3 cere cadrul de la CONTACT, unde markerul nu mai incape")
    assert len(ring.since(-1e9)) == 4 and ring.since(1e9) == []
    return f"4/6 cadre pastrate, {mb:.1f} MB, inclusiv cel fara marker"


def test_G2_monitorul_ruleaza_fara_sa_comande():
    """RACE_MONITOR end-to-end pe sursa sintetica."""
    tmp = tempfile.mkdtemp()
    cal = real_calibration()
    cfg = {'marker_id': 26, 'marker_size_m': 0.48, 'roi_below_m': 5.0,
           'roi_size_px': [640, 480], 'autonomy_enabled': False}
    log = svc.setup_logging(os.path.join(tmp, 'logs'), verbose=False)
    ring = svc.FrameRing(maxlen=8)
    args = argparse.Namespace(conn='', baud=0, buffer_frames=8,
                              wait_serial=0.1, status_every=0.0,
                              loop_sleep=0.005, max_seconds=10.0)
    src = ArraySource(marker_frames([3, 4, 5, 6, 7, 8, 9, 10]), fps=30.0)
    det = svc.build_monitor_detector(cfg, cal, ring, source=src)
    rc = svc.run_monitor(cfg, cal, args, log, detector=det, ring=ring)
    assert rc == 0
    assert len(ring) == 8, f"ring {len(ring)}"
    logfile = os.path.join(tmp, 'logs', svc.LOG_NAME)
    txt = open(logfile).read()
    assert 'RACE_MONITOR pornit' in txt
    assert 'NU comanda vehiculul' in txt
    return f"8 cadre procesate, log scris ({len(txt)} octeti)"


def test_G2_logul_se_roteste():
    """Rotatia chiar taie fisierul, nu doar e configurata."""
    tmp = tempfile.mkdtemp()
    logdir = os.path.join(tmp, 'logs')
    vechi = svc.LOG_MAX_BYTES
    try:
        svc.LOG_MAX_BYTES = 2048
        log = svc.setup_logging(logdir, verbose=False)
        for i in range(400):
            log.info("linie de test %d cu ceva continut ca sa ocupe loc", i)
    finally:
        svc.LOG_MAX_BYTES = vechi
    fisiere = sorted(os.listdir(logdir))
    assert len(fisiere) > 1, f"nu s-a rotit: {fisiere}"
    assert len(fisiere) <= svc.LOG_BACKUPS + 1, f"prea multe: {fisiere}"
    for f in fisiere:
        size = os.path.getsize(os.path.join(logdir, f))
        assert size < 8192, f"{f}: {size} octeti, peste limita"
    return f"{len(fisiere)} fisiere, toate sub limita (max {svc.LOG_BACKUPS}+1)"


def _unit_sections(text):
    """{sectiune: {cheie: valoare}}, ignorand comentariile.

    Prima varianta a testului taia textul cu `text.split('[Service]')` si a
    dat un rezultat fals: un COMENTARIU din unitate contine literalul
    "[Service]", deci sectiunea [Unit] parea sa se termine acolo. Testul
    raporta ca StartLimitIntervalSec lipseste din [Unit] si, mai rau,
    verificarea "nu e in [Service]" trecea din acelasi motiv gresit.
    Sectiunile se determina pe linii ancorate, nu prin cautare de subsir."""
    out, cur = {}, None
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith(('#', ';')):
            continue
        if line.startswith('[') and line.endswith(']'):
            cur = line[1:-1]
            out.setdefault(cur, {})
            continue
        if cur and '=' in line:
            k, _, v = line.partition('=')
            out[cur][k.strip()] = v.strip()
    return out


def test_G2_unitatea_systemd():
    """Unitatea contine ce trebuie, in SECTIUNEA care trebuie."""
    unit = svc.render_unit(repo='/home/pi/nova-zdc',
                           python='/home/pi/nova-venv/bin/python', user='pi',
                           conn='/dev/serial0', baud=921600)
    sec = _unit_sections(unit)
    assert set(sec) == {'Unit', 'Service', 'Install'}, sorted(sec)

    srv = sec['Service']
    assert srv['Restart'] == 'on-failure', srv.get('Restart')
    assert srv['User'] == 'pi'
    assert srv['ExecStart'].startswith('/home/pi/nova-venv/bin/python')
    assert 'nova_service.py' in srv['ExecStart']
    assert 'video' in srv['SupplementaryGroups']
    assert 'dialout' in srv['SupplementaryGroups']
    assert sec['Install']['WantedBy'] == 'multi-user.target'

    # StartLimit* sunt chei de [Unit]. Puse in [Service], systemd le ignora
    # TACUT - inca o instanta din §5.10.
    assert 'StartLimitIntervalSec' in sec['Unit'], (
        "StartLimitIntervalSec lipseste din [Unit]")
    assert 'StartLimitIntervalSec' not in srv, (
        "StartLimitIntervalSec e in [Service], unde e ignorat tacut")
    assert 'StartLimitBurst' in sec['Unit'] and 'StartLimitBurst' not in srv

    # Caile de scriere permise acopera exact ce scrie serviciul.
    rw = srv.get('ReadWritePaths', '')
    assert '/logs' in rw and '/data' in rw, rw

    assert 'DEZACTIVAT' in unit, "unitatea nu spune ca e implicit dezactivata"
    return (f"{len(sec)} sectiuni; StartLimit* in [Unit], nu in [Service]; "
            f"ReadWritePaths corect")


# --- G3: run_e2 -------------------------------------------------------------

def test_G3_sesiune_completa_pe_adevar_sintetic():
    """Statii la distante cunoscute: eroarea trebuie sa iasa mica."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)
    adevar = [10.0, 5.0, 2.0, 1.0]
    stare = {'d': None}

    def source_factory():
        return ArraySource(marker_frames([stare['d']] * 6), fps=30.0)

    raspunsuri = iter([f"{d}" for d in adevar] + ['q'])

    def input_fn(_prompt):
        a = next(raspunsuri)
        if a != 'q':
            stare['d'] = float(a)
        return a

    args = argparse.Namespace(
        config=cfg_path, lumina='sintetic', nota='test', frames=6,
        stations=adevar, out_root=os.path.join(tmp, 'e2'), no_frames=False,
        prag_incadrare=0.38, images=None)
    rc = run_e2.run_session(args, source_factory, input_fn=input_fn)
    assert rc == 0

    ses_dir = os.path.join(tmp, 'e2', sorted(os.listdir(os.path.join(tmp, 'e2')))[0])
    man = json.load(open(os.path.join(ses_dir, 'manifest.json')))
    assert man['calibrare']['n_images'] == 25
    assert man['mediu']['opencv']

    import csv as _csv
    with open(os.path.join(ses_dir, 'stations.csv')) as f:
        randuri = list(_csv.DictReader(f))
    assert len(randuri) == 4, f"{len(randuri)} statii"
    erori = []
    for r, truth in zip(randuri, adevar):
        assert float(r['rata_detectie']) == 1.0, f"{r['statie']}: rata < 1"
        e = abs(float(r['eroare_rel']))
        erori.append(e)
        assert e < 0.02, f"{r['statie']}: eroare {e:.2%} peste 2%"
        d = os.path.join(ses_dir, r['statie'], 'frames')
        assert len(os.listdir(d)) == 6, f"{r['statie']}: cadre brute lipsa"
    return (f"4 statii 1-10 m, 100% detectie, eroare max {max(erori):.2%}, "
            f"cadre brute salvate")


def test_G3_NEGATIV_sub_pragul_de_incadrare():
    """Sub 0.38 m absenta detectiei e REZULTATUL CORECT; prezenta ei e bug."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)

    def source_factory():
        return ArraySource(marker_frames([0.30] * 5), fps=30.0)

    raspunsuri = iter(['0.30', 'q'])
    args = argparse.Namespace(
        config=cfg_path, lumina='sintetic', nota='', frames=5, stations=[0.3],
        out_root=os.path.join(tmp, 'e2'), no_frames=False,
        prag_incadrare=0.38, images=None)
    run_e2.run_session(args, source_factory, input_fn=lambda _p: next(raspunsuri))

    ses_dir = os.path.join(tmp, 'e2', sorted(os.listdir(os.path.join(tmp, 'e2')))[0])
    import csv as _csv
    r = list(_csv.DictReader(open(os.path.join(ses_dir, 'stations.csv'))))[0]
    assert float(r['rata_detectie']) == 0.0, "a detectat sub pragul de incadrare"
    assert r['nota'] == 'OK', f"verdict {r['nota']}, asteptat OK"

    # iar verdictul se inverseaza: detectie sub prag = PROBLEMA
    stare, motiv = run_e2.verdict({'rata_detectie': 0.8, 'eroare_rel': 0.0},
                                  0.30, sub_prag=True)
    assert stare == 'PROBLEMA', f"{stare}: detectia sub prag trebuie semnalata"
    return "0/5 sub prag = OK; 4/5 sub prag = PROBLEMA"


def test_G3_rezumatul_rezista_la_o_poza_proasta():
    """Mediana, nu media: o detectie aberanta nu trebuie sa mute cifra."""
    randuri = [{'detectat': True, 'distanta_m': 5.0, 'marker_px': 90.0,
                'latenta_ms': 10.0, 'temp_c': 50.0} for _ in range(9)]
    randuri.append({'detectat': True, 'distanta_m': 25.0, 'marker_px': 18.0,
                    'latenta_ms': 10.0, 'temp_c': 55.0})
    s = run_e2.summarize_station(randuri, 5.0)
    assert abs(s['distanta_estimata_mediana_m'] - 5.0) < 1e-9, s
    assert abs(s['eroare_rel']) < 1e-9
    medie = sum(r['distanta_m'] for r in randuri) / len(randuri)
    assert abs(medie / 5.0 - 1) > 0.35, "cazul de test nu e destul de aberant"
    assert s['temp_max_c'] == 55.0
    return (f"mediana 5.000 m (eroare 0.00%) vs medie {medie:.1f} m "
            f"(eroare {medie/5-1:+.0%})")


def test_G3_scrie_dupa_fiecare_statie():
    """O sesiune intrerupta lasa pe disc statiile deja facute."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)
    stare = {'d': None}

    def source_factory():
        return ArraySource(marker_frames([stare['d']] * 4), fps=30.0)

    raspunsuri = iter(['5.0', '3.0', 'BOOM'])

    def input_fn(_prompt):
        a = next(raspunsuri)
        if a == 'BOOM':
            raise KeyboardInterrupt
        stare['d'] = float(a)
        return a

    args = argparse.Namespace(
        config=cfg_path, lumina='sintetic', nota='', frames=4,
        stations=[5.0, 3.0], out_root=os.path.join(tmp, 'e2'),
        no_frames=False, prag_incadrare=0.38, images=None)
    try:
        run_e2.run_session(args, source_factory, input_fn=input_fn)
        raise AssertionError('KeyboardInterrupt nu s-a propagat')
    except KeyboardInterrupt:
        pass

    ses_dir = os.path.join(tmp, 'e2', sorted(os.listdir(os.path.join(tmp, 'e2')))[0])
    import csv as _csv
    randuri = list(_csv.DictReader(open(os.path.join(ses_dir, 'stations.csv'))))
    assert len(randuri) == 2, f"{len(randuri)} statii pe disc dupa intrerupere"
    assert os.path.exists(os.path.join(ses_dir, 'manifest.json'))
    for r in randuri:
        assert len(os.listdir(os.path.join(ses_dir, r['statie'], 'frames'))) == 4
    return "2 statii complete pe disc dupa Ctrl-C in a treia"


def test_G3_citirea_distantei():
    """Validarea intrarii de la operator."""
    raspunsuri = iter(['abc', '-3', '500', '5,02', ''])
    v = run_e2.prompt_distance(lambda _p: next(raspunsuri))
    assert abs(v - 5.02) < 1e-9, v
    assert run_e2.prompt_distance(lambda _p: 'q') is None
    assert run_e2.prompt_distance(lambda _p: '', sugestie=7.5) == 7.5
    return "text, negativ si 500 m respinse; virgula acceptata; Enter = sugestie"


# --- G4: preflight ----------------------------------------------------------

def test_G4_totul_trece_da_zero():
    """Singurul caz in care codul de iesire e 0."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)

    class SursaBuna:
        nominal_fps = 30.0
        size = (W, H)
        control_problems = []
        def __init__(self):
            self.f = marker_frames([5.0] * 12)
            self.i = 0
        def read(self):
            import time as _t
            if self.i >= len(self.f):
                return None
            _t.sleep(0.001)
            self.i += 1
            return self.f[self.i - 1], _t.monotonic()
        def close(self):
            pass

    args = argparse.Namespace(config=cfg_path, conn='/dev/null', baud=921600,
                              parm=os.path.join(REPO, 'config',
                                                'nova_flight.parm'),
                              frames=12, mavlink_timeout=1.0, no_camera=False,
                              no_mavlink=True, json=False,
                              os_release='/etc/os-release')
    res = pf.run_checks(args, source_factory=SursaBuna)
    dupa = {r.name: r.status for r in res}
    for nume in ('stiva', 'calibrare', 'rezolutie', 'camera', 'imagine',
                 'controale'):
        assert dupa[nume] == pf.OK, f"{nume}: {dupa[nume]}"
    # Cu --no-mavlink raman doua SARIT, deci codul NU e 0 - asta e regula.
    sarite = [r.name for r in res if r.status == pf.SARIT]
    assert sarite == ['mavlink', 'parametri'], sarite
    return (f"6 verificari locale OK; sarite: {', '.join(sarite)} "
            f"-> cod de iesire diferit de 0, cum trebuie")


def test_G4_NEGATIV_fps_mic_si_capac_pe_obiectiv():
    """Doua moduri de esec care trec orice test de 'camera se deschide'."""
    class Lenta:
        nominal_fps = 30.0
        def __init__(self, frames, delay):
            self.f, self.delay, self.i = frames, delay, 0
        def read(self):
            import time as _t
            if self.i >= len(self.f):
                return None
            _t.sleep(self.delay)
            self.i += 1
            return self.f[self.i - 1], _t.monotonic()
        def close(self):
            pass

    bune = marker_frames([5.0] * 8)
    r_fps, r_img = pf.check_camera(Lenta(bune, 0.05), n_frames=8)   # ~20 fps
    assert r_fps.status == pf.ESEC, f"20 fps acceptat: {r_fps.detail}"
    assert r_img.status == pf.OK

    negre = [np.zeros((240, 320), np.uint8) for _ in range(8)]
    r_fps2, r_img2 = pf.check_camera(Lenta(negre, 0.001), n_frames=8)
    assert r_fps2.status == pf.OK, r_fps2.detail
    assert r_img2.status == pf.ESEC, "cadre negre acceptate ca valide"
    return (f"{r_fps.data['fps']:.0f} fps -> ESEC; cadre uniforme -> ESEC "
            f"(desi rata e buna)")


def test_G4_NEGATIV_rezolutie_nepotrivita():
    """Calibrare pentru alta rezolutie: eroare tacuta de distanta."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)

    class SursaMica:
        nominal_fps = 30.0
        size = (1280, 720)
        control_problems = []
        def __init__(self):
            self.i = 0
        def read(self):
            import time as _t
            if self.i >= 4:
                return None
            self.i += 1
            return np.full((720, 1280), 128, np.uint8), _t.monotonic()
        def close(self):
            pass

    args = argparse.Namespace(config=cfg_path, conn='/dev/null', baud=1,
                              parm='x', frames=4, mavlink_timeout=0.1,
                              no_camera=False, no_mavlink=True, json=False,
                              os_release='/etc/os-release')
    res = {r.name: r for r in pf.run_checks(args, source_factory=SursaMica)}
    assert res['rezolutie'].status == pf.ESEC, res['rezolutie'].detail
    assert '1280x720' in res['rezolutie'].detail
    return f"prins: {res['rezolutie'].detail[:52]}..."


def test_G4_NEGATIV_controale_neaplicate():
    """§5.10 aplicat camerei: cerut != aplicat trebuie sa pice."""
    class Sursa:
        control_problems = ['LensPosition: cerut 1.63, aplicat 0.0']
    r = pf.check_controls(Sursa())
    assert r.status == pf.ESEC and 'LensPosition' in r.detail

    class Buna:
        control_problems = []
    assert pf.check_controls(Buna()).status == pf.OK

    class Necunoscuta:
        pass
    assert pf.check_controls(Necunoscuta()).status == pf.SARIT
    return "control limitat de driver -> ESEC; sursa fara metadate -> SARIT"


def test_G4_parametrii_de_zbor():
    """nova_flight.parm se parseaza si difera de SITL doar unde trebuie."""
    flight = os.path.join(REPO, 'config', 'nova_flight.parm')
    sitl = os.path.join(REPO, 'config', 'nova_sitl.parm')
    f = {n: v for n, v, _ in parse_parm(flight)}
    s = {n: v for n, v, _ in parse_parm(sitl)}
    assert f.get('FS_THR_ENABLE') == 1.0, (
        "FS_THR_ENABLE trebuie 1 pe vehiculul real: acolo pierderea "
        "emitatorului e exact evenimentul pentru care exista failsafe-ul")
    assert s.get('FS_THR_ENABLE') == 0.0

    comune = set(f) & set(s)
    diferite = {n for n in comune if f[n] != s[n]}
    assert diferite == {'FS_THR_ENABLE'}, (
        f"diferente neasteptate fata de SITL: {sorted(diferite)}")

    # Numele care ne-au costat deja o data (§5.4/§5.10).
    assert 'WP_RFND_USE' in f and 'WPNAV_RFND_USE' not in f
    assert f['ARMING_SKIPCHK'] == 32768.0

    # FLTMODE_* depinde de emitator: mai bine lipsa decat inventat.
    assert not any(n.startswith('FLTMODE') for n in f), (
        "FLTMODE_* nu se inventeaza; depinde de emitatorul de concurs")
    txt = open(flight).read()
    assert 'DECIZII DESCHISE' in txt and 'NEVERIFICAT PE HARDWARE' in txt
    return (f"{len(f)} parametri; singura diferenta fata de SITL: "
            f"FS_THR_ENABLE 0 -> 1")


def test_G4_codul_de_iesire_ca_poarta():
    """Codul de iesire prin CLI, pe cele trei cazuri."""
    tmp = tempfile.mkdtemp()
    cfg_path, _ = make_cfg(tmp)
    exe = [sys.executable, os.path.join(REPO, 'tools', 'preflight_check.py'),
           '--config', cfg_path]

    r1 = subprocess.run(exe + ['--no-camera', '--no-mavlink'],
                        capture_output=True, text=True, timeout=60)
    assert r1.returncode == 1, "tot sarit a dat 0"
    assert 'NU sunt trecute' in r1.stdout

    r2 = subprocess.run(exe + ['--no-mavlink', '--frames', '2'],
                        capture_output=True, text=True, timeout=60)
    assert r2.returncode == 1, "camera indisponibila a dat 0"

    r3 = subprocess.run(exe + ['--no-camera', '--no-mavlink', '--json'],
                        capture_output=True, text=True, timeout=60)
    data = json.loads(r3.stdout)
    assert {d['name'] for d in data} >= {'calibrare', 'camera', 'mavlink'}
    return "sarit -> 1, esec -> 1, --json valid"


TESTS = [
    ('H0: poarta de platforma (trixie+bookworm)', test_G1_poarta_de_platforma),
    ('G1: ordinea apt -> venv -> pip', test_G1_ordinea_pasilor),
    ('G1 NEGATIV: venv fara --system-site-packages',
     test_G1_NEGATIV_venv_fara_system_site_packages),
    ('G1 NEGATIV: verificarea finala chiar verifica',
     test_G1_NEGATIV_verificarea_finala_chiar_verifica),
    ('H0: requirements pe doua cai', test_H0_requirements_pe_doua_cai),
    ('H0: setup alege calea dupa distributie',
     test_H0_setup_alege_calea_dupa_distributie),
    ('H0: verificarea de stiva', test_H0_verificarea_de_stiva),
    ('G2: calibrarea ceruta la pornire', test_G2_calibrarea_ceruta_la_pornire),
    ('G2: bariera de comanda', test_G2_bariera_de_comanda),
    ('G2: ring buffer', test_G2_ring_buffer),
    ('G2: monitorul ruleaza fara sa comande',
     test_G2_monitorul_ruleaza_fara_sa_comande),
    ('G2: logul se roteste', test_G2_logul_se_roteste),
    ('G2: unitatea systemd', test_G2_unitatea_systemd),
    ('G3: sesiune completa pe adevar sintetic',
     test_G3_sesiune_completa_pe_adevar_sintetic),
    ('G3 NEGATIV: sub pragul de incadrare',
     test_G3_NEGATIV_sub_pragul_de_incadrare),
    ('G3: rezumatul rezista la o poza proasta',
     test_G3_rezumatul_rezista_la_o_poza_proasta),
    ('G3: scrie dupa fiecare statie', test_G3_scrie_dupa_fiecare_statie),
    ('G3: citirea distantei', test_G3_citirea_distantei),
    ('G4: totul trece da zero', test_G4_totul_trece_da_zero),
    ('G4 NEGATIV: fps mic si capac pe obiectiv',
     test_G4_NEGATIV_fps_mic_si_capac_pe_obiectiv),
    ('G4 NEGATIV: rezolutie nepotrivita',
     test_G4_NEGATIV_rezolutie_nepotrivita),
    ('G4 NEGATIV: controale neaplicate',
     test_G4_NEGATIV_controale_neaplicate),
    ('G4: parametrii de zbor', test_G4_parametrii_de_zbor),
    ('G4: codul de iesire ca poarta', test_G4_codul_de_iesire_ca_poarta),
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
            fails += 0
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
            traceback.print_exc(limit=3)
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
