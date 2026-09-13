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
