#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru nova/handover.py (§8, 15.3.1 B3.1), fara SITL.

    python3 tools/test_handover.py

Poarta de handover decide daca o incercare autonoma incepe. Un refuz costa
cateva secunde; o acceptare gresita costa 10 puncte. Fiecare criteriu are
aici si un caz pozitiv, si unul negativ.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.handover import (HANDOVER_ALT_MAX_M, HANDOVER_ALT_MIN_M,  # noqa: E402
                           HandoverGate, Reject)
from nova.rc import OverrideMonitor  # noqa: E402


class StubVehicle:
    def __init__(self, alt=8.0):
        self.z = -alt
        self.params = {'RC1_TRIM': 1500, 'RC2_TRIM': 1500,
                       'RC3_TRIM': 1100, 'RC4_TRIM': 1500}
        self.rc = (1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000)
        self.rc_t = 100.0

    @property
    def alt(self):
        return -self.z

    def request_param(self, name):
        pass

    def set_rc(self, ch, value):
        rc = list(self.rc)
        rc[ch - 1] = value
        self.rc = tuple(rc)


def build(alt=8.0, autonomy=True):
    v = StubVehicle(alt)
    ov = OverrideMonitor(v)
    rejects = []
    gate = HandoverGate(v, ov, on_reject=rejects.append,
                        autonomy_enabled=autonomy)
    gate.on_aux_requested(100.0)
    return v, ov, gate, rejects


SETTLED = 101.5      # dupa fereastra de 1.0 s
GOOD = dict(dist_to_marker_m=2.0, detection_age_s=0.05)


#: Se opreste INAINTE ca fereastra sa expire: altfel ultimul apel ar face
#: validarea completa, cu argumentele de aici, si ar decide in locul testului.
SETTLE_END = 100.95


def settle(gate, start=100.0, until=SETTLE_END, step=0.05):
    """Ruleaza fereastra de asezare, ca in bucla reala. Esantionarea din ea e
    criteriul pentru throttle, deci nu poate fi sarita."""
    t = start
    while t < until:
        gate.check(t, **GOOD)
        t += step


# --- teste -----------------------------------------------------------------

def test_accept_in_conditii_bune():
    v, ov, gate, rej = build()
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is True, f"refuzat: {why}"
    assert not rej
    assert ov.neutral == (1500, 1500, 1100, 1500), ov.neutral
    return f"acceptat, neutru memorat {ov.neutral}"


def test_nu_decide_in_fereastra_de_asezare():
    """B3.1: cat timp arcurile se asaza, nu e nici accept, nici refuz."""
    v, ov, gate, rej = build()
    v.set_rc(1, 1800)                       # tranzitoriu violent de arc
    ok, why = gate.check(100.5, **GOOD)
    assert ok is None, f"a decis {ok} in fereastra de asezare"
    assert why == Reject.SETTLING
    assert not rej, "a semnalizat un refuz in timpul asezarii"
    return "nicio decizie in primele 1.0 s"


def test_refuz_mansa_in_afara_neutrului():
    v, ov, gate, rej = build()
    v.set_rc(2, 1500 + 150)                 # pitch tinut deoparte
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is False, f"a acceptat cu mansa deviata ({why})"
    assert Reject.STICKS in why, why
    assert rej and rej[0] == why, "refuzul nu a fost semnalizat pilotului"
    return f"refuzat: {why}"


def test_roll_pitch_yaw_fata_de_trim_nu_de_1500():
    """Axele care se auto-centreaza se compara cu RCx_TRIM, nu cu 1500."""
    v, ov, gate, rej = build()
    v.params.update({'RC1_TRIM': 1512, 'RC2_TRIM': 1488, 'RC4_TRIM': 1505})
    v.set_rc(1, 1512)
    v.set_rc(2, 1488)
    v.set_rc(4, 1505)
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is True, f"a confundat offset-ul de trim cu o deviatie: {why}"
    return "offset-uri de trim de pana la 12 PWM acceptate"


def test_throttle_departe_de_trim_dar_nemiscat():
    """Pe un emitator real throttle-ul sta la mijlocul cursei pentru hover,
    iar RC3_TRIM e adesea la capatul de jos. Nu e o mansa deviata.

    Varianta care compara si throttle-ul cu trim-ul a refuzat primul handover
    din SITL, cu throttle la 500 PWM de RC3_TRIM."""
    v, ov, gate, rej = build()
    v.params['RC3_TRIM'] = 1100
    v.set_rc(3, 1600)                       # 500 PWM de trim, dar nemiscat
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is True, f"a refuzat un throttle stationar: {why}"
    assert ov.neutral[2] == 1600, ov.neutral
    return "throttle la 500 PWM de trim, dar stationar, acceptat"


def test_refuz_throttle_care_se_misca():
    """CAZ NEGATIV al testului de mai sus: throttle-ul care se misca in
    fereastra de asezare inseamna ca pilotul inca zboara."""
    v, ov, gate, rej = build()
    t = 100.0
    val = 1400
    while t < SETTLE_END:
        val += 20                           # pilotul urca lent throttle-ul
        v.set_rc(3, val)
        gate.check(t, **GOOD)
        t += 0.05
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is False, "a acceptat cu throttle-ul in miscare"
    assert 'throttle' in why, why
    return f"refuzat: {why}"


def test_refuz_altitudine_prea_mica():
    v, ov, gate, rej = build(alt=HANDOVER_ALT_MIN_M - 0.5)
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is False and Reject.ALTITUDE in why, why
    return f"refuzat la {v.alt:.1f} m"


def test_refuz_altitudine_prea_mare():
    v, ov, gate, rej = build(alt=HANDOVER_ALT_MAX_M + 0.5)
    settle(gate)
    ok, why = gate.check(SETTLED, **GOOD)
    assert ok is False and Reject.ALTITUDE in why, why
    return f"refuzat la {v.alt:.1f} m (plafon {HANDOVER_ALT_MAX_M:.0f} m)"


def test_accept_la_marginile_ferestrei():
    """Cazul negativ al testelor de mai sus: exact pe limita se accepta."""
    for alt in (HANDOVER_ALT_MIN_M, HANDOVER_ALT_MAX_M):
        v, ov, gate, rej = build(alt=alt)
        settle(gate)
        ok, why = gate.check(SETTLED, **GOOD)
        assert ok is True, f"la {alt} m a refuzat: {why}"
    return f"{HANDOVER_ALT_MIN_M:.0f} m si {HANDOVER_ALT_MAX_M:.0f} m acceptate"


def test_refuz_prea_departe():
    v, ov, gate, rej = build()
    settle(gate)
    ok, why = gate.check(SETTLED, dist_to_marker_m=7.0, detection_age_s=0.05)
    assert ok is False and Reject.DISTANCE in why, why
    return f"refuzat: {why}"


def test_refuz_marker_nedetectat():
    v, ov, gate, rej = build()
    settle(gate)
    ok, why = gate.check(SETTLED, dist_to_marker_m=2.0, detection_age_s=None)
    assert ok is False and Reject.NO_MARKER in why, why
    v2, ov2, gate2, rej2 = build()
    settle(gate2)
    ok2, why2 = gate2.check(SETTLED, dist_to_marker_m=2.0,
                            detection_age_s=1.0)
    assert ok2 is False and Reject.NO_MARKER in why2, why2
    return "refuzat si fara detectie, si cu detectie veche de 1 s"


def test_neutrul_nu_se_memoreaza_la_refuz():
    """Daca handover-ul e refuzat, nu ramanem cu o referinta de neutru
    luata dintr-un moment prost."""
    v, ov, gate, rej = build()
    v.set_rc(1, 1500 + 200)
    settle(gate)
    gate.check(SETTLED, **GOOD)
    assert ov.neutral is None, f"a memorat neutru la refuz: {ov.neutral}"
    return "fara referinta de neutru dupa refuz"


def test_decizia_e_stabila():
    """Odata decis, nu se razgandeste la urmatorul ciclu."""
    v, ov, gate, rej = build()
    settle(gate)
    ok, _ = gate.check(SETTLED, **GOOD)
    assert ok is True
    v.set_rc(1, 1800)                       # pilotul misca dupa acceptare
    ok2, _ = gate.check(SETTLED + 0.1, **GOOD)
    assert ok2 is True, "s-a razgandit; miscarea de dupa e treaba override-ului"
    return "decizia ramane; ce urmeaza tine de monitorul de override"


# --- E0: garda de autonomie -------------------------------------------------

def test_E0_autonomie_dezactivata_refuza_imediat():
    """E0. Cu autonomy_enabled=false, refuz explicit, cu motiv, INAINTE de
    fereastra de asezare - pilotul afla imediat. Nu se memoreaza neutru."""
    v, ov, gate, rej = build(autonomy=False)
    ok, why = gate.check(100.1, **GOOD)          # inca in fereastra
    assert ok is False, f"a intors {ok}"
    assert why == Reject.AUTONOMY_DISABLED, why
    assert rej == [Reject.AUTONOMY_DISABLED], "refuzul nu a ajuns la pilot"
    assert ov.neutral is None
    settle(gate)
    ok2, _ = gate.check(SETTLED, **GOOD)
    assert ok2 is False, "s-a razgandit dupa asezare"
    return "REJECT imediat, motiv explicit, fara neutru memorat"


def test_E0_config_implicit_este_dezactivat():
    """E0. Fara fisier de config, si fara valoare explicita, garda e INCHISA.
    Nu exista cale de activare din mediu sau din linia de comanda."""
    import os
    import tempfile
    from nova import config as nova_config
    missing = os.path.join(tempfile.mkdtemp(), 'nu_exista.json')
    assert nova_config.autonomy_enabled(missing) is False
    with open(missing, 'w') as f:
        f.write('{"autonomy_enabled": "true"}')      # string, nu bool
    assert nova_config.autonomy_enabled(missing) is False, \
        'un string "true" a trecut drept activare'
    with open(missing, 'w') as f:
        f.write('{"autonomy_enabled": true}')
    assert nova_config.autonomy_enabled(missing) is True
    # Fisierul din repo: E0 e DESCHIS din 24.09.2026, decizia echipei pentru
    # proba de coborare pe vehiculul de test, fara raport E2 (commit-ul o
    # spune). Ce ramane de verificat e ca valoarea e un literal JSON, nu un
    # text care ar parea deschis si n-ar fi.
    import json
    brut = json.load(open(nova_config.DEFAULT_PATH))['autonomy_enabled']
    assert isinstance(brut, bool), (
        f"config/nova.json: autonomy_enabled={brut!r} - trebuie true/false")
    return (f"lipsa fisier -> fals; \"true\" string -> fals; "
            f"repo -> {str(brut).lower()} (literal)")


def test_E0_gate_citeste_config_cand_nu_e_explicit():
    """Aplicatia de bord nu paseaza valoarea: poarta citeste fisierul."""
    # Fisierul se INJECTEAZA, nu se ia din repo: acolo E0 e o decizie a
    # echipei care se schimba, iar testul nu trebuie sa depinda de ea (§5.40).
    import json
    import os
    import tempfile
    from nova import config as nova_config
    cale = os.path.join(tempfile.mkdtemp(), 'nova.json')
    vechi = nova_config.DEFAULT_PATH
    try:
        nova_config.DEFAULT_PATH = cale
        rezultate = {}
        for valoare in (False, True):
            json.dump({'autonomy_enabled': valoare}, open(cale, 'w'))
            v = StubVehicle(8.0)
            gate = HandoverGate(v, OverrideMonitor(v))   # autonomy_enabled=None
            gate.on_aux_requested(100.0)
            rezultate[valoare] = gate.check(100.1, **GOOD)
    finally:
        nova_config.DEFAULT_PATH = vechi
    ok, why = rezultate[False]
    assert ok is False and why == Reject.AUTONOMY_DISABLED, (ok, why)
    ok, why = rezultate[True]
    assert why != Reject.AUTONOMY_DISABLED, (
        "cu fisierul pe true, poarta refuza tot pe E0 - nu citeste fisierul")
    return "fara valoare explicita -> fisierul decide, in ambele sensuri"



def test_monitorul_fortat_refuza_cu_motivul_lui():
    """`nova_pi.py --monitor` (pornirea automata) inchide poarta pentru
    rularea asta, oricare ar fi config/nova.json.

    Motivul trebuie sa fie ALTUL decat cel de E0: dupa ce E0 se deschide, un
    pilot care citeste "autonomy_enabled=false" ar gasi fisierul pe `true` si
    n-ar mai intelege de ce e refuzat. Iar poarta nu are voie sa accepte -
    altfel pornirea automata ar fi o a doua cale spre autonomie, cu alte
    setari decat proba de coborare."""
    # E0 DESCHIS - cazul care conteaza: monitorul trebuie sa castige
    v = StubVehicle(8.0)
    ov = OverrideMonitor(v)
    rejects = []
    gate = HandoverGate(v, ov, on_reject=rejects.append,
                        autonomy_enabled=True, monitor=True)
    gate.on_aux_requested(100.0)
    ok, motiv = gate.check(100.1, **GOOD)
    assert ok is False, "monitorul a acceptat cu E0 deschis"
    assert motiv == Reject.MONITOR, motiv
    assert 'autostart' in motiv, "motivul nu spune ce setare il tine inchis"
    assert 'autonomy_enabled=false' not in motiv
    settle(gate)
    ok2, _ = gate.check(SETTLED, **GOOD)
    assert ok2 is False, "monitorul s-a razgandit dupa asezare"

    # si fara monitor, aceeasi poarta cu E0 deschis accepta: flagul e cel
    # care inchide, nu altceva din configuratia de test
    v2, ov2, gate2, _ = build(autonomy=True)
    settle(gate2)
    ok3, why3 = gate2.check(SETTLED, **GOOD)
    assert ok3 is True, f"fara monitor, poarta ar trebui sa accepte: {why3}"
    return "cu E0 deschis: monitor -> refuz MONITOR; fara monitor -> accept"


def test_monitorul_poarta_motivul_pornirii_automate():
    """Pornirea automata cade in monitor din mai multe cauze: autostart pe
    monitor, E0 inchis, verificari picate la boot. Pe teren, fara laptop,
    pilotul afla care dintre ele doar din motivul refuzului - deci textul dat
    ajunge neschimbat in refuz, iar poarta ramane la fel de inchisa."""
    v = StubVehicle(8.0)
    ov = OverrideMonitor(v)
    rejects = []
    motiv_dat = 'ZBOR refuzat: verificari picate'
    gate = HandoverGate(v, ov, on_reject=rejects.append,
                        autonomy_enabled=True, monitor=motiv_dat)
    gate.on_aux_requested(100.0)
    ok, motiv = gate.check(100.1, **GOOD)
    assert ok is False, "monitorul cu motiv a acceptat cu E0 deschis"
    assert motiv == motiv_dat and rejects == [motiv_dat], (motiv, rejects)
    settle(gate)
    assert gate.check(SETTLED, **GOOD)[0] is False
    # un text gol nu deschide nimic si nu lasa refuzul fara motiv
    gate2 = HandoverGate(v, OverrideMonitor(v), autonomy_enabled=True,
                         monitor='')
    assert gate2.monitor is False, "monitor='' trebuie sa insemne fara monitor"
    return f"refuz cu motivul dat: {motiv_dat!r}"

TESTS = [
    ('monitorul fortat refuza cu motivul lui',
     test_monitorul_fortat_refuza_cu_motivul_lui),
    ('monitorul poarta motivul pornirii automate',
     test_monitorul_poarta_motivul_pornirii_automate),
    ('accept in conditii bune', test_accept_in_conditii_bune),
    ('fara decizie in fereastra de asezare', test_nu_decide_in_fereastra_de_asezare),
    ('NEGATIV: mansa deviata', test_refuz_mansa_in_afara_neutrului),
    ('roll/pitch/yaw fata de trim', test_roll_pitch_yaw_fata_de_trim_nu_de_1500),
    ('throttle departe de trim dar nemiscat', test_throttle_departe_de_trim_dar_nemiscat),
    ('NEGATIV: throttle in miscare', test_refuz_throttle_care_se_misca),
    ('NEGATIV: altitudine prea mica', test_refuz_altitudine_prea_mica),
    ('NEGATIV: altitudine prea mare', test_refuz_altitudine_prea_mare),
    ('accept pe limitele ferestrei', test_accept_la_marginile_ferestrei),
    ('NEGATIV: prea departe de marker', test_refuz_prea_departe),
    ('NEGATIV: marker nedetectat', test_refuz_marker_nedetectat),
    ('fara neutru dupa refuz', test_neutrul_nu_se_memoreaza_la_refuz),
    ('decizia e stabila', test_decizia_e_stabila),
    ('E0: autonomie dezactivata -> REJECT imediat',
     test_E0_autonomie_dezactivata_refuza_imediat),
    ('E0: implicit inchis, fara cale laterala',
     test_E0_config_implicit_este_dezactivat),
    ('E0: poarta citeste config-ul', test_E0_gate_citeste_config_cand_nu_e_explicit),
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
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
