#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru tools/sim_handover.py (I0).

    python3 tools/test_sim_handover.py

Fara SITL: legatura mavutil e inlocuita cu una falsa care raspunde la
PARAM_REQUEST_READ si inregistreaza fiecare RC_CHANNELS_OVERRIDE.

Testul care conteaza cel mai mult trece canalele injectate prin **poarta
reala** (`nova/handover.py`) si prin monitorul de override, si verifica ca
poarta ACCEPTA. Un script care "pare ca trimite ce trebuie" dar pe care
poarta il refuza nu ajuta la nimic; un script care trece prin poarta dar
declanseaza override fals imediat dupa - cu atat mai putin.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim_handover as sh                                   # noqa: E402
from nova.handover import HandoverGate                      # noqa: E402
from nova.rc import OverrideMonitor, HANDOVER_SETTLE_S      # noqa: E402

#: Configuratia din config/nova_sitl.parm: throttle cu trim-ul JOS, ca pe un
#: emitator real. Daca scriptul ar trimite RC3_TRIM, ar comanda coborare.
PARAMS = {
    'MAV_GCS_SYSID': 255.0,
    'RC1_TRIM': 1500.0, 'RC2_TRIM': 1500.0,
    'RC3_TRIM': 1100.0, 'RC4_TRIM': 1500.0,
    'RC3_MIN': 1100.0, 'RC3_MAX': 1900.0,
}


class FakeParam:
    def __init__(self, name, val):
        self.param_id, self.param_value = name, val

    def get_type(self):
        return 'PARAM_VALUE'


class FakeMav:
    def __init__(self, link):
        self.link = link

    def param_request_read_send(self, sysid, comp, name, idx):
        name = name.decode() if isinstance(name, bytes) else name
        if name in self.link.params:
            self.link.coada.append(FakeParam(name, self.link.params[name]))

    def rc_channels_override_send(self, sysid, comp, *ch):
        self.link.trimise.append(tuple(ch))


class FakeLink:
    def __init__(self, params=None):
        self.params = dict(PARAMS if params is None else params)
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav(self)
        self.trimise = []
        self.coada = []

    def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
        while self.coada:
            msg = self.coada.pop(0)
            if type is None or msg.get_type() == type:
                return msg
        return None

    def wait_heartbeat(self, timeout=None):
        return True


def args(**kw):
    d = dict(after=0.0, hold=3.0, settle_low=0.5, then_low=False,
             then_low_after=5.0, mode_pwm=None)
    d.update(kw)
    return argparse.Namespace(**d)


class Ceas:
    """Ceas si sleep injectate: bucla ruleaza in timp simulat, instant."""

    def __init__(self, dt=1.0 / sh.RATE_HZ):
        self.t = 0.0
        self.dt = dt

    def now(self):
        return self.t

    def sleep(self, _s):
        self.t += self.dt


def ruleaza(link, a=None, **kw):
    c = Ceas()
    rc = sh.run(link, a or args(**kw), printer=lambda *_a, **_k: None,
                input_fn=lambda _p: '', now_fn=c.now, sleep_fn=c.sleep)
    return rc, c


# --- precontitii ------------------------------------------------------------

def test_NEGATIV_sysid_nepotrivit_e_refuzat():
    """ArduPilot respinge TACIT override-ul de la alt sysid. Daca scriptul
    n-ar verifica, testele ar esua fara sa spuna de ce."""
    link = FakeLink()
    ok, mesaj = sh.check_gcs_sysid(link, 42)
    assert ok is False, "sysid nepotrivit acceptat"
    assert 'MAV_GCS_SYSID' in mesaj and 'TACIT' in mesaj
    assert '--source-system 255' in mesaj, "mesajul nu spune cum se repara"

    ok2, m2 = sh.check_gcs_sysid(FakeLink(), 255)
    assert ok2 is True, m2

    # parametrul lipsa: nu blocam, dar spunem ca e primul de verificat
    fara = FakeLink({k: v for k, v in PARAMS.items() if k != 'MAV_GCS_SYSID'})
    ok3, m3 = sh.check_gcs_sysid(fara, 255)
    assert ok3 is True and 'primul lucru de verificat' in m3
    return "nepotrivit -> refuz cu comanda de reparare; lipsa -> avertisment"


def test_throttle_e_mijlocul_cursei_nu_trim():
    """Cea mai periculoasa greseala posibila in unealta asta.

    Poarta masoara AMPLITUDINEA throttle-ului, nu distanta fata de trim (§8),
    deci si trim-ul ar trece poarta. Dar in LOITER, throttle la 1100 comanda
    coborare rapida - unealta de test ar face vehiculul sa cada."""
    link = FakeLink()
    ch, info = sh.neutral_channels(link, printer=lambda *_a: None)
    assert ch[2] == 1500, f"throttle {ch[2]}, asteptat mijlocul 1100..1900"
    assert ch[2] != PARAMS['RC3_TRIM'], "throttle trimis la trim (coborare!)"
    assert info['throttle_mid'] == 1500

    # alta cursa, alt mijloc
    link2 = FakeLink(dict(PARAMS, RC3_MIN=1000.0, RC3_MAX=2000.0))
    ch2, _ = sh.neutral_channels(link2, printer=lambda *_a: None)
    assert ch2[2] == 1500, ch2[2]

    link3 = FakeLink(dict(PARAMS, RC3_MIN=1200.0, RC3_MAX=1800.0))
    ch3, _ = sh.neutral_channels(link3, printer=lambda *_a: None)
    assert ch3[2] == 1500, ch3[2]
    return "mijlocul RC3_MIN..RC3_MAX, nu RC3_TRIM (1100 = coborare in LOITER)"


def test_roll_pitch_yaw_la_trim_citit_de_pe_FC():
    link = FakeLink(dict(PARAMS, RC1_TRIM=1487.0, RC4_TRIM=1512.0))
    ch, _ = sh.neutral_channels(link, printer=lambda *_a: None)
    assert ch[0] == 1487 and ch[3] == 1512, ch
    assert ch[1] == 1500
    # §5.12: neutrul nu e 1500 prin definitie, se citeste de pe FC
    fara = FakeLink({'MAV_GCS_SYSID': 255.0})
    ch2, _ = sh.neutral_channels(fara, printer=lambda *_a: None)
    assert ch2[:4] == [1500, 1500, 1500, 1500], ch2
    return "trim citit de pe FC (1487/1512); fara raspuns -> 1500 cu avertisment"


def test_canalul_de_mod_nu_se_atinge():
    """`mode guided` / `takeoff` din MAVProxy trebuie sa ramana functionale."""
    link = FakeLink()
    ruleaza(link)
    idx = sh.MODE_CHANNEL - 1
    valori = {c[idx] for c in link.trimise}
    assert valori == {sh.IGNORE}, (
        f"canalul de mod a primit {valori}; IGNORE={sh.IGNORE} inseamna "
        f"'nu schimba', 0 ar ELIBERA override-ul")

    link2 = FakeLink()
    ruleaza(link2, mode_pwm=1900)
    assert {c[idx] for c in link2.trimise} == {1900}
    return "implicit IGNORE; --mode-pwm forteaza doar daca ceri"


# --- forma semnalului -------------------------------------------------------

def test_frontul_crescator_exista():
    """Poarta cere FRONTUL crescator (§8), nu starea. Deci AUX trebuie sa fi
    fost vazut JOS inainte de ridicare."""
    link = FakeLink()
    ruleaza(link, after=1.0, hold=2.0, settle_low=0.5)
    aux = [c[sh.AUX_CHANNEL - 1] for c in link.trimise]
    assert aux[0] == sh.AUX_LOW_PWM, aux[:3]
    assert sh.AUX_HIGH_PWM in aux, "AUX nu a fost ridicat niciodata"
    # exact o tranzitie jos->sus
    treceri = sum(1 for a, b in zip(aux, aux[1:])
                  if a < sh.AUX_HIGH_PWM <= b)
    assert treceri == 1, f"{treceri} fronturi crescatoare"
    n_jos = aux.index(sh.AUX_HIGH_PWM)
    assert n_jos >= sh.RATE_HZ * 0.5, (
        f"doar {n_jos} cadre cu AUX jos inainte de ridicare")
    return f"{n_jos} cadre jos, apoi exact 1 front crescator"


def test_manetele_stau_nemiscate():
    """Amplitudine ZERO pe roll/pitch/throttle/yaw, tot timpul.

    Orice tremur ar fi fie un refuz al portii, fie un override fals imediat
    dupa ACCEPT - adica incercarea anulata dintr-un artefact al uneltei."""
    link = FakeLink()
    ruleaza(link, after=1.0, hold=4.0)
    assert len(link.trimise) > 100, len(link.trimise)
    for i, nume in enumerate(('roll', 'pitch', 'throttle', 'yaw')):
        valori = {c[i] for c in link.trimise}
        assert len(valori) == 1, f"{nume} a variat: {sorted(valori)}"
    return f"{len(link.trimise)} cadre, amplitudine 0 pe toate patru manetele"


def test_then_low_coboara_AUX():
    """REJECT ramane afisat pana cand AUX coboara (§8)."""
    link = FakeLink()
    ruleaza(link, after=0.5, hold=4.0, then_low=True, then_low_after=1.0)
    aux = [c[sh.AUX_CHANNEL - 1] for c in link.trimise]
    assert sh.AUX_HIGH_PWM in aux
    assert aux[-1] == sh.AUX_LOW_PWM, "AUX nu a coborat la final"
    return "AUX sus apoi jos, ca REJECT-ul sa se poata sterge"


def test_release_elibereaza_toate_canalele():
    link = FakeLink()
    sh.release(link, n=3)
    assert len(link.trimise) == 3
    for c in link.trimise:
        assert c == (0,) * 8, c
    return "0 pe toate 8 canalele = eliberare (nu IGNORE)"


# --- prin poarta reala ------------------------------------------------------

class GateVehicle:
    """Minimul cerut de HandoverGate + OverrideMonitor. `rc` vine din ce
    trimite sim_handover, nu din valori inventate de test."""

    def __init__(self, params):
        self.params = dict(params)
        self.rc = None
        self.rc_t = 0.0
        self.alt = 8.0
        self.x = self.y = self.z = 0.0
        self.have_pos = True
        self.time_boot_ms = 1000

    def request_param(self, name):
        return True


def test_poarta_reala_ACCEPTA_semnalul_injectat():
    """Testul central: canalele injectate trec prin poarta reala.

    Nu verificam ca "am trimis ceva rezonabil" - verificam ca poarta, cu
    logica ei adevarata (fereastra de asezare, manete in neutru fata de
    RCx_TRIM, throttle nemiscat), ACCEPTA."""
    link = FakeLink()
    _rc, _c = ruleaza(link, after=0.5, hold=5.0)
    cadre = link.trimise
    assert cadre, "nimic injectat"

    v = GateVehicle(PARAMS)
    ov = OverrideMonitor(v)
    gate = HandoverGate(v, ov, autonomy_enabled=True)

    dt = 1.0 / sh.RATE_HZ
    t = 0.0
    cerut = False
    verdict = None
    motiv = ''
    for ch in cadre:
        v.rc = tuple(1500 if x == sh.IGNORE else x for x in ch)
        v.rc_t = t
        aux_sus = v.rc[sh.AUX_CHANNEL - 1] >= 1700
        if aux_sus and not cerut:
            cerut = True
            gate.on_aux_requested(t)
        if cerut and verdict is None:
            ok, why = gate.check(t, 1.8, 0.05)      # 1.8 m de marker, det 50 ms
            if ok is not None:
                verdict, motiv = ok, why
                t_verdict = t
        t += dt

    assert verdict is True, f"poarta a REFUZAT semnalul injectat: {motiv}"
    asezare = t_verdict - next(
        tt for tt, ch in zip([i * dt for i in range(len(cadre))], cadre)
        if ch[sh.AUX_CHANNEL - 1] >= 1700)
    assert asezare >= HANDOVER_SETTLE_S - dt, (
        f"a acceptat dupa {asezare:.2f} s, sub fereastra de "
        f"{HANDOVER_SETTLE_S} s")
    return (f"ACCEPT dupa {asezare:.2f} s de asezare "
            f"(fereastra {HANDOVER_SETTLE_S} s)")


def test_semnalul_injectat_NU_declanseaza_override_fals():
    """Dupa ACCEPT, neutrul se memoreaza din ce raporteaza emitatorul atunci.
    Daca unealta ar tremura, override-ul s-ar declansa imediat si ar anula
    incercarea."""
    link = FakeLink()
    ruleaza(link, after=0.5, hold=8.0)
    v = GateVehicle(PARAMS)
    ov = OverrideMonitor(v)
    dt = 1.0 / sh.RATE_HZ
    t = 0.0
    for ch in link.trimise:
        v.rc = tuple(1500 if x == sh.IGNORE else x for x in ch)
        v.rc_t = t
        t += dt
    ov.capture_neutral(t)
    assert ov.neutral is not None

    # inca 5 s de semnal identic: niciun override
    declansat = False
    for _ in range(int(sh.RATE_HZ * 5)):
        v.rc_t = t
        if ov.update(t):
            declansat = True
            break
        t += dt
    assert not declansat, f"override fals pe semnal constant: {ov.status()}"
    return "5 s dupa capture_neutral pe semnal constant: niciun override"


# --- E0: separarea din runda 3 ----------------------------------------------

def test_E0_separarea_intre_simulare_si_bord():
    """I0: `autonomy_enabled` fals in config, ocolit EXPLICIT doar in
    simulare. Separarea e in APLICATII, nu intr-un al doilea fisier de
    config - un config de simulare ar putea ajunge pe vehicul."""
    import json
    radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = json.load(open(os.path.join(radacina, 'config', 'nova.json')))
    assert cfg['autonomy_enabled'] is False, (
        "config/nova.json are autonomy_enabled=true; E0 e sursa de adevar "
        "pentru zbor si ramane false pana la E2")

    sim = open(os.path.join(radacina, 'tools', 'fake_detector.py')).read()
    assert 'autonomy_enabled=True' in sim, (
        "simularea nu mai ocoleste garda; testele de secventa nu pot rula")
    assert 'OCOLITA in simulare' in sim, "ocolirea nu se anunta la pornire"

    bord = open(os.path.join(radacina, 'tools', 'nova_pi.py')).read()
    assert 'autonomy_enabled=True' not in bord, (
        "aplicatia de BORD ocoleste garda E0 - exact ce nu are voie")
    # Bordul CITESTE cheia (o afiseaza in banner) - asta e corect si util.
    # Ce nu are voie e sa o PASEZE portii, adica sa ocoleasca fisierul.
    import re
    apel = re.search(r'HandoverGate\((.*?)\)', bord, re.S)
    assert apel, "nova_pi.py nu mai construieste HandoverGate"
    assert 'autonomy_enabled' not in apel.group(1), (
        f"nova_pi.py paseaza autonomy_enabled portii: {apel.group(1)!r}")

    # si nu exista un al doilea fisier de config care sa o ridice
    for nume in os.listdir(os.path.join(radacina, 'config')):
        if not nume.endswith('.json'):
            continue
        d = json.load(open(os.path.join(radacina, 'config', nume)))
        if isinstance(d, dict) and d.get('autonomy_enabled') is True:
            raise AssertionError(f"config/{nume} ridica garda E0")
    return ("config false; simularea ocoleste explicit si anunta; bordul nu "
            "are cale; niciun alt config nu ridica garda")


TESTS = [
    ('NEGATIV: sysid nepotrivit e refuzat',
     test_NEGATIV_sysid_nepotrivit_e_refuzat),
    ('throttle e mijlocul cursei, nu trim',
     test_throttle_e_mijlocul_cursei_nu_trim),
    ('roll/pitch/yaw la trim citit de pe FC',
     test_roll_pitch_yaw_la_trim_citit_de_pe_FC),
    ('canalul de mod nu se atinge', test_canalul_de_mod_nu_se_atinge),
    ('frontul crescator exista', test_frontul_crescator_exista),
    ('manetele stau nemiscate', test_manetele_stau_nemiscate),
    ('--then-low coboara AUX', test_then_low_coboara_AUX),
    ('release elibereaza toate canalele',
     test_release_elibereaza_toate_canalele),
    ('POARTA REALA accepta semnalul injectat',
     test_poarta_reala_ACCEPTA_semnalul_injectat),
    ('semnalul injectat NU declanseaza override fals',
     test_semnalul_injectat_NU_declanseaza_override_fals),
    ('E0: separarea intre simulare si bord',
     test_E0_separarea_intre_simulare_si_bord),
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
