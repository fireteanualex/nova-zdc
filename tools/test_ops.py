#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru H2 (conflictul de port serial) si H3 (previzualizare).

    python3 tools/test_ops.py

Ambele sunt lucruri care se manifesta doar pe teren: un serviciu care tine
portul, un ecran care nu exista prin SSH. Testele le produc pe desktop
inlocuind cele doua cai de iesire spre sistem - `subprocess` pentru
systemctl/fuser, si variabilele de mediu pentru sesiunea grafica.
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                          # noqa: E402

from nova import preview as pv_mod                          # noqa: E402
from nova import serial_guard as sg                         # noqa: E402


def runner_for(is_active=None, fuser=None, stop_ok=True):
    """Un `runner` fals: raspunde ca systemctl/fuser, fara sa atinga sistemul."""
    stare = {'activ': is_active}

    def run(cmd, timeout=3.0):
        if cmd[:2] == ['systemctl', 'is-active']:
            if stare['activ'] is None:
                return None, ''
            return (0 if stare['activ'] == 'active' else 3), stare['activ']
        if cmd[:3] == ['sudo', 'systemctl', 'stop']:
            if stop_ok:
                stare['activ'] = 'inactive'
                return 0, ''
            return 1, 'Failed to stop'
        if cmd[0] == 'fuser':
            return (0, fuser) if fuser else (1, '')
        return None, ''
    return run


# --- H2 ---------------------------------------------------------------------

def test_H2_serviciul_activ_e_identificat():
    run = runner_for(is_active='active')
    motiv = sg.describe_conflict('/dev/serial0', runner=run)
    assert motiv is not None, "serviciu activ, dar portul pare liber"
    assert sg.SERVICE_NAME in motiv
    assert 'systemctl stop' in motiv, "mesajul nu contine comanda de reparare"
    assert '--stop-service' in motiv
    try:
        sg.ensure_port_free('/dev/serial0', runner=run)
        raise AssertionError('ensure_port_free nu a ridicat PortBusy')
    except sg.PortBusy as e:
        assert e.service == sg.SERVICE_NAME
    return "identificat, cu `sudo systemctl stop` in mesaj"


def test_H2_port_liber_trece():
    run = runner_for(is_active='inactive')
    assert sg.describe_conflict('/dev/ttyINEXISTENT', runner=run) is None
    assert sg.ensure_port_free('/dev/ttyINEXISTENT', runner=run) is True
    return "port liber: trece fara sa deschida nimic"


def test_H2_SITL_nu_are_conflict():
    """Pe udp/tcp nu exista port exclusiv; verificarea nu are ce cauta."""
    run = runner_for(is_active='active')      # serviciul RULEAZA
    assert sg.ensure_port_free('udpin:127.0.0.1:14552', runner=run) is True
    assert sg.ensure_port_free('tcp:127.0.0.1:5760', runner=run) is True
    return "udp/tcp trec chiar si cu serviciul activ"


def test_H2_alt_proces_e_identificat():
    run = runner_for(is_active='inactive', fuser=f"{os.getpid()} 999")
    motiv = sg.describe_conflict('/dev/serial0', runner=run)
    assert motiv is not None and 'PID 999' in motiv, motiv
    assert str(os.getpid()) not in motiv, (
        "s-a raportat pe sine ca ocupant al portului")
    return "alt PID raportat cu linia de comanda; propriul PID exclus"


def test_H2_stop_service_dupa_confirmare():
    run = runner_for(is_active='active')
    intrebari = []

    def confirma(serviciu):
        intrebari.append(serviciu)
        return True

    ok = sg.ensure_port_free('/dev/serial0', stop_service_ok=True,
                             confirm=confirma, runner=run,
                             printer=lambda _m: None)
    assert ok is True and intrebari == [sg.SERVICE_NAME], intrebari
    return "--stop-service: intreaba o data, apoi opreste"


def test_H2_NEGATIV_refuzul_confirmarii_opreste():
    """Un 'nu' la confirmare nu are voie sa porneasca oricum."""
    run = runner_for(is_active='active')
    try:
        sg.ensure_port_free('/dev/serial0', stop_service_ok=True,
                            confirm=lambda _s: False, runner=run,
                            printer=lambda _m: None)
        raise AssertionError('a pornit desi confirmarea a fost refuzata')
    except sg.PortBusy as e:
        assert 'refuzata' in str(e)
    return "raspuns negativ -> PortBusy, nu pornire"


def test_H2_NEGATIV_stop_esuat_nu_porneste():
    run = runner_for(is_active='active', stop_ok=False)
    try:
        sg.ensure_port_free('/dev/serial0', stop_service_ok=True,
                            confirm=lambda _s: True, runner=run,
                            printer=lambda _m: None)
        raise AssertionError('a pornit desi oprirea serviciului a esuat')
    except sg.PortBusy:
        pass
    return "stop esuat -> PortBusy, nu pornire"


def test_H2_fara_systemd_nu_blocheaza():
    """Un diagnostic care nu se poate face nu are voie sa opreasca pornirea."""
    run = runner_for(is_active=None)          # systemctl lipseste
    assert sg.service_active(runner=run) is None
    assert sg.ensure_port_free('/dev/ttyINEXISTENT', runner=run) is True
    return "fara systemctl: 'nu stiu', nu 'refuz'"


def test_H2_nova_pi_iese_cu_cod_distinct():
    """Codul de iesire 3 deosebeste conflictul de port de restul."""
    import subprocess
    r = subprocess.run(
        [sys.executable, os.path.join(os.path.dirname(__file__), 'nova_pi.py'),
         '--conn', '/dev/serial0', '--config', '/nu/exista.json'],
        capture_output=True, text=True, timeout=60,
        env=dict(os.environ, NOVA_FAKE='1'))
    # Pe desktop, serviciul nu ruleaza si portul nu exista, deci trebuie sa
    # treaca de garda si sa pice mai incolo (calibrare lipsa = 2).
    assert r.returncode in (2, 3), f"exit {r.returncode}:\n{r.stdout}{r.stderr}"
    return f"exit {r.returncode} (2 = calibrare, 3 = port ocupat)"


# --- H3 ---------------------------------------------------------------------

def test_H3_fara_sesiune_grafica_se_stinge_singura():
    vechi = {v: os.environ.pop(v, None) for v in pv_mod.GUI_ENV_VARS}
    try:
        p = pv_mod.bench_preview('t', enabled=True, logger=lambda _m: None)
        assert p.enabled is False
        assert 'sesiune grafica' in p.disabled_reason
        assert p.show(np.zeros((8, 8), np.uint8)) is True, (
            "show() trebuie sa fie no-op, nu sa arunce")
        p.close()
    finally:
        for k, v in vechi.items():
            if v is not None:
                os.environ[k] = v
    return "fara DISPLAY: se opreste singura, cu motiv, si nu arunca"


def test_H3_implicit_oprita_pe_bord_pornita_pe_banc():
    os.environ.setdefault('DISPLAY', ':0')
    bord = pv_mod.onboard_preview('bord', logger=lambda _m: None)
    assert bord.enabled is False, (
        "fereastra pornita implicit pe bord: fara vc4-kms-v3d consuma CPU "
        "din bugetul detectiei")
    banc = pv_mod.bench_preview('banc', logger=lambda _m: None)
    assert banc.enabled is True and banc.fullscreen is True
    assert bord.fullscreen is False
    return "bord: oprita, nefullscreen; banc: pornita, fullscreen"


def test_H3_avertisment_la_pornirea_pe_bord():
    os.environ.setdefault('DISPLAY', ':0')
    mesaje = []
    p = pv_mod.onboard_preview('bord', enabled=True, logger=mesaje.append)
    assert p.enabled is True
    assert any('vc4-kms-v3d' in m for m in mesaje), mesaje
    assert any('bugetul detectiei' in m for m in mesaje), mesaje
    return "pornita explicit -> avertisment despre CPU in log"


def test_H3_redimensionarea_e_doar_pentru_afisare():
    """Cadrul original nu se modifica; scara e o copie."""
    aratate = []

    class FalsCv2:
        WINDOW_NORMAL = 0
        WND_PROP_FULLSCREEN = 1
        WINDOW_FULLSCREEN = 1
        INTER_AREA = 3
        error = Exception

        @staticmethod
        def namedWindow(*a):
            pass

        @staticmethod
        def setWindowProperty(*a):
            pass

        @staticmethod
        def imshow(_t, img):
            aratate.append(img)

        @staticmethod
        def waitKey(_ms):
            return 255           # nicio tasta

        @staticmethod
        def resize(img, _d, fx, fy, interpolation):
            h, w = img.shape[:2]
            return np.zeros((int(h * fy), int(w * fx)), img.dtype)

        @staticmethod
        def destroyWindow(_t):
            pass

    os.environ.setdefault('DISPLAY', ':0')
    vechi = pv_mod.cv2
    try:
        pv_mod.cv2 = FalsCv2
        p = pv_mod.Preview('t', enabled=True, scale=0.5)
        cadru = np.full((1296, 2304), 77, np.uint8)
        assert p.show(cadru) is True
        assert aratate[0].shape == (648, 1152), aratate[0].shape
        assert cadru.shape == (1296, 2304) and cadru[0, 0] == 77, (
            "cadrul original a fost modificat")
    finally:
        pv_mod.cv2 = vechi
    return "afisat 1152x648, original intact 2304x1296"


def test_H3_iesire_pe_q_si_pe_escape():
    """Fereastra fullscreen nu are buton de inchidere: Escape e obligatoriu."""
    taste = {'k': 0}

    class FalsCv2:
        WINDOW_NORMAL = 0
        WND_PROP_FULLSCREEN = 1
        WINDOW_FULLSCREEN = 1
        error = Exception
        namedWindow = staticmethod(lambda *a: None)
        setWindowProperty = staticmethod(lambda *a: None)
        imshow = staticmethod(lambda *a: None)
        destroyWindow = staticmethod(lambda *a: None)

        @staticmethod
        def waitKey(_ms):
            return taste['k']

    os.environ.setdefault('DISPLAY', ':0')
    vechi = pv_mod.cv2
    rezultate = {}
    try:
        pv_mod.cv2 = FalsCv2
        p = pv_mod.Preview('t', enabled=True)
        cadru = np.zeros((8, 8), np.uint8)
        for eticheta, cod in (('nimic', 255), ('q', ord('q')),
                              ('Q', ord('Q')), ('Esc', 27), ('a', ord('a'))):
            taste['k'] = cod
            rezultate[eticheta] = p.show(cadru)
    finally:
        pv_mod.cv2 = vechi
    assert rezultate['nimic'] is True and rezultate['a'] is True
    for t in ('q', 'Q', 'Esc'):
        assert rezultate[t] is False, f"{t} nu a inchis fereastra"
    return "q, Q si Escape inchid; alte taste nu"


def test_H3_fullscreen_cere_WINDOW_NORMAL():
    """WINDOW_AUTOSIZE ignora TACIT cererea de fullscreen."""
    apeluri = []

    class FalsCv2:
        WINDOW_NORMAL = 0
        WINDOW_AUTOSIZE = 1
        WND_PROP_FULLSCREEN = 1
        WINDOW_FULLSCREEN = 1
        error = Exception
        imshow = staticmethod(lambda *a: None)
        waitKey = staticmethod(lambda _m: 255)
        destroyWindow = staticmethod(lambda *a: None)

        @staticmethod
        def namedWindow(titlu, flag):
            apeluri.append(('named', flag))

        @staticmethod
        def setWindowProperty(titlu, prop, val):
            apeluri.append(('prop', prop, val))

    os.environ.setdefault('DISPLAY', ':0')
    vechi = pv_mod.cv2
    try:
        pv_mod.cv2 = FalsCv2
        p = pv_mod.Preview('t', enabled=True, fullscreen=True)
        p.show(np.zeros((8, 8), np.uint8))
    finally:
        pv_mod.cv2 = vechi
    assert apeluri[0] == ('named', FalsCv2.WINDOW_NORMAL), (
        f"fereastra creata cu {apeluri[0]}, nu cu WINDOW_NORMAL - cererea de "
        f"fullscreen ar fi ignorata tacut")
    assert apeluri[1][0] == 'prop', apeluri
    return "namedWindow(WINDOW_NORMAL) INAINTE de setWindowProperty"



# --- H5: collect_session ----------------------------------------------------

class FakeLogEntry:
    def __init__(self, i, size, num_logs):
        self.id, self.size, self.num_logs = i, size, num_logs
        self.time_utc = 1700000000

    def get_type(self):
        return 'LOG_ENTRY'


class FakeLogData:
    def __init__(self, i, ofs, data):
        self.id, self.ofs = i, ofs
        self.count = len(data)
        self.data = list(data) + [0] * (90 - len(data))

    def get_type(self):
        return 'LOG_DATA'


class FakeParam:
    def __init__(self, name, val):
        self.param_id, self.param_value = name, val

    def get_type(self):
        return 'PARAM_VALUE'


class FakeFC:
    """Un FC de laborator: parametri, lista de loguri, si un .bin.

    `pierde` sare peste al n-lea bloc, ca sa se poata verifica raportarea
    golurilor - un .bin cu gauri e inutilizabil ca dovada si trebuie sa se
    stie pe loc, nu la scrutineering."""

    def __init__(self, continut=b'', params=None, pierde=()):
        self.target_system = 1
        self.target_component = 1
        self.continut = continut
        self.params = params or {}
        self.pierde = set(pierde)
        self.coada = []
        self.mav = self
        self._cerut_lista = False

    # -- emisie (interfata mav) --
    def param_request_read_send(self, sysid, comp, name, idx):
        name = name.decode() if isinstance(name, bytes) else name
        if name in self.params:
            self.coada.append(FakeParam(name, self.params[name]))

    def log_request_list_send(self, sysid, comp, start, end):
        self.coada.append(FakeLogEntry(7, len(self.continut), 1))

    def log_request_data_send(self, sysid, comp, log_id, ofs, count):
        n = 90
        for k, i in enumerate(range(0, len(self.continut), n)):
            if k in self.pierde:
                continue
            self.coada.append(FakeLogData(log_id, i, self.continut[i:i + n]))

    # -- receptie --
    def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
        while self.coada:
            msg = self.coada.pop(0)
            if type is None or msg.get_type() == type:
                return msg
        return None

    def wait_heartbeat(self, timeout=None):
        return True


def _collect_args(tmp, **kw):
    import argparse as _ap
    d = dict(conn='/dev/fake', baud=921600, config=None,
             out_root=os.path.join(tmp, 'sessions'),
             logs_dir=os.path.join(tmp, 'logs'),
             frames_dir=os.path.join(tmp, 'frames'),
             nota='test', stamp='20260913-000000', timeout=1.0,
             no_fc=False, no_fc_log=False, log_id=None, list_only=False)
    d.update(kw)
    return _ap.Namespace(**d)


def test_H5_sesiune_completa():
    """Manifest, loguri, cadre, parametri si .bin, intr-un singur director."""
    import collect_session as cs
    tmp = tempfile.mkdtemp()
    os.makedirs(os.path.join(tmp, 'logs'))
    open(os.path.join(tmp, 'logs', 'nova-monitor.log'), 'w').write('x' * 100)
    os.makedirs(os.path.join(tmp, 'frames'))
    open(os.path.join(tmp, 'frames', 'ring_000000001000500.png'), 'wb').write(b'\x89PNG')

    continut = bytes(range(256)) * 12          # 3072 octeti
    fc = FakeFC(continut, params={'PLND_ENABLED': 0.0, 'WP_RFND_USE': 0.0,
                                  'ARMING_SKIPCHK': 32768.0})
    a = _collect_args(tmp)
    ses, manifest, raport, _ = cs.collect(a, connect=lambda: fc)

    fisiere = sorted(os.listdir(ses))
    assert 'manifest.json' in fisiere and 'params.txt' in fisiere, fisiere
    assert os.path.isdir(os.path.join(ses, 'logs'))
    assert os.path.isdir(os.path.join(ses, 'frames'))

    binar = os.path.join(ses, 'fc', 'log_7.bin')
    assert os.path.exists(binar), fisiere
    assert open(binar, 'rb').read() == continut, ".bin descarcat gresit"
    assert manifest['fc']['log_descarcat']['complet'] is True

    assert manifest['git']['commit'], "hash-ul commit-ului lipseste"
    assert 'dirty' in manifest['git']
    assert manifest['software']['python']
    assert manifest['fc']['parametri']['ARMING_SKIPCHK'] == 32768.0
    assert 'time_boot_ms' in manifest['sincronizare']

    # parametrul care nu exista pe firmware se noteaza ca lipsa, nu se omite
    assert manifest['fc']['parametri']['FENCE_TYPE'] is None
    txt = open(os.path.join(ses, 'params.txt')).read()
    assert 'FENCE_TYPE,-' in txt, txt
    return (f"{len(fisiere)} intrari, .bin {len(continut)} octeti verificat "
            f"bit cu bit, {len(manifest['fc']['parametri'])} parametri")


def test_H5_NEGATIV_bin_cu_goluri_e_semnalat():
    """Un .bin incomplet trebuie sa se vada ACUM, nu la scrutineering."""
    import collect_session as cs
    tmp = tempfile.mkdtemp()
    continut = bytes(range(256)) * 12
    fc = FakeFC(continut, pierde=(3, 9))
    ses, manifest, raport, _ = cs.collect(_collect_args(tmp),
                                          connect=lambda: fc)
    info = manifest['fc']['log_descarcat']
    assert info['complet'] is False, "gaurile nu au fost detectate"
    assert info['lipsa'] == 180, info
    assert any('LIPSA' in r for r in raport), raport
    return f"{info['lipsa']} octeti lipsa, raportati explicit"


def test_H5_NEGATIV_fara_FC_produce_tot_ce_poate():
    """FC-ul lipsa nu anuleaza colectarea: restul evidentei tot se aduna."""
    import collect_session as cs
    tmp = tempfile.mkdtemp()
    os.makedirs(os.path.join(tmp, 'logs'))
    open(os.path.join(tmp, 'logs', 'a.log'), 'w').write('y')

    def explodeaza():
        raise OSError(2, 'No such file or directory')

    ses, manifest, raport, _ = cs.collect(_collect_args(tmp),
                                          connect=explodeaza)
    assert manifest['fc']['conectat'] is False
    assert 'eroare' in manifest['fc']
    assert os.path.exists(os.path.join(ses, 'logs', 'a.log'))
    assert os.path.exists(os.path.join(ses, 'manifest.json'))
    assert any('neconectat' in r for r in raport), raport
    return "FC inaccesibil: manifest si loguri scrise, eroarea inregistrata"


def test_H5_arborele_murdar_se_declara():
    """Un arbore git murdar inseamna ca nu stim ce cod a zburat."""
    import collect_session as cs
    g = cs.git_info()
    assert g['commit'] and len(g['commit']) == 40, g
    assert isinstance(g['dirty'], bool)
    assert 'describe' in g
    # repo inexistent: nu arunca, doar nu stie
    g2 = cs.git_info('/nu/exista/nicaieri')
    assert g2['commit'] is None and g2['dirty'] is None, g2
    return f"commit {g['describe']}, dirty={g['dirty']}; repo absent -> None"


def test_H5_dump_ring_buffer_poarta_timpul_capturii():
    """Numele fisierului trebuie sa permita alinierea cu .bin (6.2.1.30)."""
    import nova_service as svc
    tmp = tempfile.mkdtemp()
    ring = svc.FrameRing(maxlen=4)
    for i in range(4):
        ring.push(np.full((20, 30), 40 + i, np.uint8), 1234.500 + i * 0.033)
    scrise = ring.dump(tmp)
    assert len(scrise) == 4, scrise
    nume = [os.path.basename(x) for x in scrise]
    assert nume[0] == 'ring_000000001234500.png', nume[0]
    assert nume == sorted(nume), "ordinea alfabetica nu e cea cronologica"
    ms = [int(n.split('_')[1].split('.')[0]) for n in nume]
    assert [b - a for a, b in zip(ms, ms[1:])] == [33, 33, 33], ms
    return f"{len(nume)} cadre, nume cu timestamp de captura in ms"



# --- H4: modul de concurs ---------------------------------------------------

from nova import race_screen as rs                          # noqa: E402


def _snap(**kw):
    d = {'preflight_ok': True, 'link_healthy': True, 'state': 'RACE_MONITOR',
         'marker_px': 148.0, 'det_age_s': 0.08, 'hb_age_s': 0.3, 'fps': 29.4,
         'det_rate': 0.97, 'temp_c': 55.0, 'disk_mb': 12000.0,
         'autonomy_enabled': True}
    d.update(kw)
    return d


def _text(linii):
    """Liniile fara secvente ANSI - ce citeste de fapt operatorul."""
    import re
    return re.sub(r'\x1b\[[0-9;?]*[a-zA-Z]', '', '\n'.join(linii))


def test_H4_verdictul_raporteaza_cel_mai_grav():
    """Ordinea conteaza: cine vede GATA nu trebuie sa mai citeasca restul."""
    cazuri = [
        ({}, 'GATA', 'verde'),
        ({'autonomy_enabled': False}, 'GATA', 'albastru'),
        ({'temp_c': 72.0}, 'ATENTIE', 'galben'),
        ({'disk_mb': 300.0}, 'ATENTIE', 'galben'),
        ({'det_rate': 0.0}, 'ATENTIE', 'galben'),
        ({'disk_mb': 100.0}, 'NU ZBURA', 'rosu'),
        ({'temp_c': 85.0}, 'NU ZBURA', 'rosu'),
        ({'link_healthy': False}, 'NU ZBURA', 'rosu'),
        ({'preflight_ok': False}, 'NU ZBURA', 'rosu'),
        # cel mai grav castiga, chiar daca vine dupa in dictionar
        ({'temp_c': 85.0, 'disk_mb': 100.0, 'preflight_ok': False},
         'NU ZBURA - PREFLIGHT', 'rosu'),
    ]
    for kw, asteptat, culoare in cazuri:
        txt, cul = rs.RaceScreen.verdict(_snap(**kw))
        assert txt.startswith(asteptat), f"{kw}: {txt!r}, astept {asteptat}"
        assert cul == culoare, f"{kw}: culoare {cul}, astept {culoare}"
    return f"{len(cazuri)} combinatii; cel mai grav castiga"


def test_H4_verdictul_e_scris_nu_doar_colorat():
    """Un ecran spalat de soare, sau un operator daltonist, tot trebuie sa
    poata citi verdictul."""
    for kw in ({'link_healthy': False}, {'temp_c': 90.0},
               {'preflight_ok': False}, {'disk_mb': 50.0}):
        txt, _ = rs.RaceScreen.verdict(_snap(**kw))
        assert 'NU ZBURA' in txt, txt
        assert '-' in txt, f"verdictul nu spune DE CE: {txt!r}"
    txt, _ = rs.RaceScreen.verdict(_snap())
    assert 'GATA' in txt
    return "fiecare verdict contine cuvintele, si motivul dupa liniuta"


def test_H4_ecranul_e_rar_si_are_unitati():
    sc = rs.RaceScreen(width=80)
    linii = sc.render(_snap())
    txt = _text(linii)
    utile = [x for x in txt.splitlines() if x.strip()]
    assert len(utile) <= 10, f"{len(utile)} linii - prea dens pentru un metru"
    for unitate in ('px', 's', 'fps', 'C', 'MB'):
        assert unitate in txt, f"lipseste unitatea {unitate}"
    assert 'G A T A' in txt, "banda de sus nu e cu litere rarite"
    return f"{len(utile)} linii, toate cifrele cu unitate"


def test_H4_lipsa_se_arata_cu_liniuta_nu_cu_zero():
    """FPS 0 si FPS necunoscut sunt lucruri diferite."""
    sc = rs.RaceScreen(width=80)
    txt = _text(sc.render(_snap(fps=None, temp_c=None, disk_mb=None,
                                det_rate=None, marker_px=None,
                                det_age_s=None)))
    assert '- fps' in txt, txt
    assert '- C' in txt and '- MB' in txt, txt
    assert '0 fps' not in txt, "necunoscut raportat ca zero"
    assert 'FARA MARKER' in txt
    # si eticheta nu are voie sa se lipeasca de valoare
    for linie in txt.splitlines():
        if not linie.startswith('  ') or not linie.strip():
            continue
        eticheta = linie[2:2 + rs.LABEL_W]
        assert eticheta.endswith(' '), (
            f"eticheta lipita de valoare: {linie!r}")
    return "necunoscut -> '- fps', nu 0; etichetele nu se lipesc"


def test_H4_handover_refuzat_arata_motivul():
    sc = rs.RaceScreen(width=80)
    txt = _text(sc.render(_snap()))
    assert 'HANDOVER' not in txt, "verdict afisat inainte sa existe"

    sc.note_handover(False, 'altitudine 3.2 m (cerut 5-12 m)')
    txt = _text(sc.render(_snap()))
    assert 'H A N D O V E R   R E F U Z A T' in txt, txt
    assert 'altitudine 3.2 m' in txt, (
        "refuzul nu arata motivul - nu se poate repara in cateva secunde")

    sc.note_handover(True, '')
    txt = _text(sc.render(_snap()))
    assert 'H A N D O V E R   A C C E P T A T' in txt
    return "refuz cu motiv literal; acceptare distincta"


def test_H4_snapshot_tolereaza_piese_lipsa():
    """Ecranul trebuie sa se deseneze si cand detectorul inca porneste."""
    snap = rs.snapshot({'autonomy_enabled': False})
    assert snap['state'] == '-'
    assert snap['autonomy_enabled'] is False
    sc = rs.RaceScreen(width=80)
    linii = sc.render(snap)               # nu trebuie sa arunce
    assert len(linii) > 5

    class DetectorRupt:
        def stats(self):
            raise RuntimeError('inca porneste')

    class VehiculRupt:
        @property
        def link_healthy(self):
            raise RuntimeError('neconectat')

    snap2 = rs.snapshot({}, detector=DetectorRupt(), vehicle=VehiculRupt())
    assert 'fps' not in snap2 or snap2['fps'] is None
    sc.render(snap2)
    return "piese lipsa sau care arunca -> liniute, nu exceptie"


def test_H4_preflight_picat_refuza_pornirea():
    """Cerinta centrala a lui H4: nu pornim cu preflight-ul picat."""
    import argparse as _ap
    import nova_pi
    import preflight_check as pf

    apeluri = {'n': 0}

    def fals(a):
        apeluri['n'] += 1
        return ([pf.Result('camera', pf.ESEC, 'capac pe obiectiv'),
                 pf.Result('mavlink', pf.SARIT, '--no-mavlink')],
                [pf.Result('camera', pf.ESEC, 'capac pe obiectiv')],
                [pf.Result('mavlink', pf.SARIT, '--no-mavlink')])

    vechi = nova_pi.run_preflight
    iesire = []
    try:
        nova_pi.run_preflight = fals
        rc = nova_pi.race_mode(_ap.Namespace(config=None, conn='/dev/x',
                                             baud=1, no_mavlink=True),
                               {'autonomy_enabled': False})
    finally:
        nova_pi.run_preflight = vechi
    assert rc == 4, f"a pornit cu preflight picat (rc={rc})"
    assert apeluri['n'] == 1
    del iesire
    return "preflight picat -> cod 4, nu pornire"


def test_H4_o_verificare_sarita_opreste_pornirea():
    """Sarit != trecut, si in modul de concurs asta inseamna refuz."""
    import argparse as _ap
    import nova_pi
    import preflight_check as pf

    def doar_sarite(a):
        r = [pf.Result('mavlink', pf.SARIT, '--no-mavlink'),
             pf.Result('camera', pf.OK, '30 fps')]
        return r, [], [r[0]]

    vechi = nova_pi.run_preflight
    try:
        nova_pi.run_preflight = doar_sarite
        rc = nova_pi.race_mode(_ap.Namespace(config=None, conn='/dev/x',
                                             baud=1, no_mavlink=True), {})
    finally:
        nova_pi.run_preflight = vechi
    assert rc == 4, f"a pornit cu o verificare sarita (rc={rc})"
    return "nimic picat, o verificare sarita -> tot refuz"


def test_H4_preflight_curat_lasa_sa_porneasca():
    import argparse as _ap
    import nova_pi
    import preflight_check as pf

    def tot_ok(a):
        return [pf.Result('camera', pf.OK, '30 fps')], [], []

    vechi = nova_pi.run_preflight
    try:
        nova_pi.run_preflight = tot_ok
        rc = nova_pi.race_mode(_ap.Namespace(config=None, conn='/dev/x',
                                             baud=1, no_mavlink=False), {})
    finally:
        nova_pi.run_preflight = vechi
    assert rc is None, f"a refuzat desi preflight-ul a trecut (rc={rc})"
    return "tot verde -> None, adica porneste"


def test_H4_preflight_ruleaza_INAINTEA_detectorului():
    """Ordinea din main() - conflict de resursa, nu de stil.

    Preflight-ul deschide singur camera si serialul. Daca ar rula DUPA
    `build_pi_detector`, ar incerca sa deschida a doua oara acelasi senzor,
    iar asta esueaza DOAR pe Pi (pe desktop nu exista picamera2, deci nu se
    manifesta si ar ajunge pe teren). Testul verifica ordinea in sursa,
    fiindca aici nu avem camera cu care sa o reproducem."""
    cale = os.path.join(os.path.dirname(__file__), 'nova_pi.py')
    src = open(cale).read()
    i_race = src.index('rc = race_mode(a, cfg)')
    i_det = src.index('detector = build_pi_detector(')
    assert i_race < i_det, (
        "race_mode() ruleaza DUPA build_pi_detector(): preflight-ul ar "
        "incerca sa deschida camera a doua oara")

    # si preflight-ul trebuie sa elibereze portul serial dupa verificare
    pf_src = open(os.path.join(os.path.dirname(__file__),
                               'preflight_check.py')).read()
    bloc = pf_src[pf_src.index('def check_mavlink'):
                  pf_src.index('def check_params')]
    assert 'm.close()' in bloc, (
        "check_mavlink nu inchide conexiunea: portul ar ramane ocupat de "
        "propriul nostru proces pana la GC")
    return "race_mode inainte de detector; check_mavlink elibereaza portul"


def test_H4_race_mode_nu_isi_face_propriul_cablaj():
    """§5.14: un al doilea punct de intrare care si-ar construi singur
    piesele ar reintroduce exact bugetul de supervizor inert."""
    cale = os.path.join(os.path.dirname(__file__), 'race_mode.py')
    src = open(cale).read()
    for interzis in ('SafetySupervisor(', 'LandingStateMachine(',
                     'HandoverGate(', 'run_loop('):
        assert interzis not in src, (
            f"race_mode.py construieste {interzis} - trebuie sa delege catre "
            f"nova_pi.py, altfel exista doua cablaje (§5.14)")
    assert 'nova_pi' in src and '--race' in src
    return "race_mode.py deleaga integral catre nova_pi.py --race"


def test_H4_checklistul_acopera_simptomele():
    """Checklistul e util doar daca gasesti in el ce ti s-a intamplat."""
    cale = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'docs', 'CHECKLIST_TEREN.md')
    txt = open(cale, encoding='utf-8').read()
    for simptom in ('resource busy', '_ARRAY_API not found',
                    'Rangefinder 1: No Data', 'NAV_TAKEOFF result=4',
                    'capac pe obiectiv', 'octeți lipsă',
                    'autonomie dezactivată', 'LEGATURA FC CAZUTA'):
        assert simptom in txt, f"checklistul nu acopera: {simptom}"
    for sectiune in ('Înainte de plecare', 'La sosire', 'Între curse',
                     'Când ceva nu merge', 'ELICELE DEMONTATE'):
        assert sectiune in txt, f"lipseste sectiunea: {sectiune}"
    assert 'kill switch' in txt.lower()
    assert txt.count('|') > 150, "tabelul de depanare e prea sarac"
    return "toate sectiunile + 8 simptome-cheie prezente"


TESTS = [
    ('H2: serviciul activ e identificat', test_H2_serviciul_activ_e_identificat),
    ('H2: port liber trece', test_H2_port_liber_trece),
    ('H2: SITL nu are conflict', test_H2_SITL_nu_are_conflict),
    ('H2: alt proces e identificat', test_H2_alt_proces_e_identificat),
    ('H2: --stop-service dupa confirmare',
     test_H2_stop_service_dupa_confirmare),
    ('H2 NEGATIV: refuzul confirmarii opreste',
     test_H2_NEGATIV_refuzul_confirmarii_opreste),
    ('H2 NEGATIV: stop esuat nu porneste',
     test_H2_NEGATIV_stop_esuat_nu_porneste),
    ('H2: fara systemd nu blocheaza', test_H2_fara_systemd_nu_blocheaza),
    ('H2: nova_pi iese cu cod distinct',
     test_H2_nova_pi_iese_cu_cod_distinct),
    ('H3: fara sesiune grafica se stinge singura',
     test_H3_fara_sesiune_grafica_se_stinge_singura),
    ('H3: implicit oprita pe bord, pornita pe banc',
     test_H3_implicit_oprita_pe_bord_pornita_pe_banc),
    ('H3: avertisment la pornirea pe bord',
     test_H3_avertisment_la_pornirea_pe_bord),
    ('H3: redimensionarea e doar pentru afisare',
     test_H3_redimensionarea_e_doar_pentru_afisare),
    ('H3: iesire pe q si pe Escape', test_H3_iesire_pe_q_si_pe_escape),
    ('H3: fullscreen cere WINDOW_NORMAL',
     test_H3_fullscreen_cere_WINDOW_NORMAL),
    ('H5: sesiune completa', test_H5_sesiune_completa),
    ('H5 NEGATIV: .bin cu goluri e semnalat',
     test_H5_NEGATIV_bin_cu_goluri_e_semnalat),
    ('H5 NEGATIV: fara FC produce tot ce poate',
     test_H5_NEGATIV_fara_FC_produce_tot_ce_poate),
    ('H5: arborele murdar se declara', test_H5_arborele_murdar_se_declara),
    ('H5: dump-ul ring bufferului poarta timpul capturii',
     test_H5_dump_ring_buffer_poarta_timpul_capturii),
    ('H4: verdictul raporteaza cel mai grav',
     test_H4_verdictul_raporteaza_cel_mai_grav),
    ('H4: verdictul e scris, nu doar colorat',
     test_H4_verdictul_e_scris_nu_doar_colorat),
    ('H4: ecranul e rar si are unitati', test_H4_ecranul_e_rar_si_are_unitati),
    ('H4: lipsa se arata cu liniuta, nu cu zero',
     test_H4_lipsa_se_arata_cu_liniuta_nu_cu_zero),
    ('H4: handover refuzat arata motivul',
     test_H4_handover_refuzat_arata_motivul),
    ('H4: snapshot tolereaza piese lipsa',
     test_H4_snapshot_tolereaza_piese_lipsa),
    ('H4: preflight picat refuza pornirea',
     test_H4_preflight_picat_refuza_pornirea),
    ('H4: o verificare sarita opreste pornirea',
     test_H4_o_verificare_sarita_opreste_pornirea),
    ('H4: preflight curat lasa sa porneasca',
     test_H4_preflight_curat_lasa_sa_porneasca),
    ('H4: preflight ruleaza INAINTEA detectorului',
     test_H4_preflight_ruleaza_INAINTEA_detectorului),
    ('H4: race_mode nu isi face propriul cablaj',
     test_H4_race_mode_nu_isi_face_propriul_cablaj),
    ('H4: checklistul acopera simptomele',
     test_H4_checklistul_acopera_simptomele),
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
