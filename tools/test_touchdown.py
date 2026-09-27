#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Touchdown fara dezarmare, urcarea la alt_riseup (PROMPT_CLAUDE_CODE_TOUCHDOWN).

    python3 tools/test_touchdown.py

Configul secventei si parametrii FC de care depinde (faza A, masurata in
SITL Copter-4.5.7 pe 27.09.2026): in GUIDED dezarmarea automata cere
throttle-ul pilotului la zero, pe sol, DISARM_DELAY secunde.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import config as nova_config                          # noqa: E402


def test_configul_secventei_si_validarea():
    """Implicitul din prompt; touchdown_hold_s < 1 s refuzat (15.1.3);
    alt_riseup < 5 m doar avertisment (15.1.2); nova.json = implicitul."""
    v, w = nova_config.touchdown_settings(nova_config.load())
    assert v == {'alt_riseup': 5.5, 'touchdown_speed': 0.4, 'touchdown_hold_s': 1.5,
                 'riseup_timeout_s': 15.0, 'hover_confirm_s': 1.0}, v
    assert w == [], w
    try:
        nova_config.touchdown_settings({'touchdown_hold_s': 0.8})
        assert False, "0.8 s acceptat"
    except ValueError as e:
        assert '15.1.3' in str(e)
    v2, w2 = nova_config.touchdown_settings({'alt_riseup': 4.5})
    assert v2['alt_riseup'] == 4.5 and len(w2) == 1 and '15.1.2' in w2[0], w2
    for rau in ({'alt_riseup': 'mult'}, {'touchdown_speed': 0}, {'riseup_timeout_s': -1}):
        try:
            nova_config.touchdown_settings(rau)
            assert False, rau
        except ValueError:
            pass
    return "implicit 5.5 / 0.4 / 1.5 / 15 / 1.0; hold 0.8 refuzat; alt 4.5 -> avertisment"


def test_parametrii_FC_ai_secventei_in_fisierele_verificate():
    """check_params.py verifica fisierul de parametri PE NUME: DISARM_DELAY
    20 (dezarmarea automata dupa 20 s cu throttle la zero pe sol, masurat
    21.1 s in SITL 4.5.7) si PILOT_THR_BHV 0 (cu throttle arcuit, "jos" ar
    insemna sub mijloc si drona s-ar dezarma pe marker)."""
    out = {}
    for f in ('nova_flight.parm', 'nova_flight_4.5.parm'):
        vals = {}
        for line in open(os.path.join(REPO, 'config', f)):
            line = line.split('#')[0].strip()
            if ',' in line:
                k, val = line.split(',', 1)
                vals[k.strip()] = float(val)
        assert vals.get('DISARM_DELAY') == 20 and vals.get('PILOT_THR_BHV') == 0, (f, vals.get('DISARM_DELAY'), vals.get('PILOT_THR_BHV'))
        assert vals['DISARM_DELAY'] > nova_config.DEFAULTS['touchdown_hold_s'] + 5
        out[f] = len(vals)
    return f"DISARM_DELAY 20, PILOT_THR_BHV 0 in {', '.join(out)}"


def test_supervizorul_vegheaza_fazele_noi():
    """Punctul 5: fazele de dupa contact sunt in segmentul supravegheat.
    TOUCHDOWN_DESCENT: monitoarele de coborare; GROUND_HOLD: EKF fara
    pozitie -> EXIT (VPE pe sol o tine valida); RISEUP: urcare prea rapida,
    inclinare -> EXIT. Supervizorul ExtNav nu trimite comenzi de mod."""
    import test_safety as ts
    from nova.safety import (Action, CLIMB_PHASES, EXTNAV_ENGAGED_PHASES,
                             EXTNAV_PHASES, MAX_CLIMB_RATE_MS)
    noi = ('TOUCHDOWN_DESCENT', 'CONTACT', 'GROUND_HOLD', 'RISEUP', 'HOVER_CONFIRM')
    assert all(p in EXTNAV_PHASES and p in EXTNAV_ENGAGED_PHASES for p in noi)
    assert 'COMPLETE' not in EXTNAV_PHASES, "COMPLETE preda pilotului"
    assert CLIMB_PHASES == ('RISEUP', 'HOVER_CONFIRM')
    rez = []
    # urcare prea rapida in RISEUP
    v, sup, exits = ts.build_extnav()
    t = ts.ruleaza_extnav(v, sup, ['MOVE'] * 3 + ['GROUND_HOLD'] * 3, 100.0)
    assert sup.armed and not exits
    v.vz = -(MAX_CLIMB_RATE_MS + 0.5)
    t = ts.ruleaza_extnav(v, sup, ['RISEUP'] * 3, t)
    assert not exits, "declansat inainte de CLIMB_RATE_HOLD_S"
    ts.ruleaza_extnav(v, sup, ['RISEUP'] * 4, t)
    assert exits and 'climb_rate' in exits[0], exits
    assert sup.latched == Action.EXIT and v.mode_reqs == []
    rez.append('urcare 3.5 m/s')
    # urcare normala (2.5 m/s, WPNAV_SPEED_UP implicit): nimic
    v, sup, exits = ts.build_extnav()
    t = ts.ruleaza_extnav(v, sup, ['MOVE'] * 3, 100.0)
    v.vz = -2.5
    ts.ruleaza_extnav(v, sup, ['RISEUP'] * 20, t)
    assert not exits, exits
    # EKF fara pozitie pe sol -> EXIT
    v, sup, exits = ts.build_extnav()
    t = ts.ruleaza_extnav(v, sup, ['MOVE'] * 3 + ['GROUND_HOLD'], 100.0)
    v.ekf_ok = False
    ts.ruleaza_extnav(v, sup, ['GROUND_HOLD'] * 3, t)
    assert exits and 'ekf' in exits[0], exits
    rez.append('EKF pe sol')
    # inclinare in RISEUP -> EXIT
    v, sup, exits = ts.build_extnav()
    t = ts.ruleaza_extnav(v, sup, ['MOVE'] * 3, 100.0)
    v.roll = 0.7
    ts.ruleaza_extnav(v, sup, ['RISEUP'] * 6, t)
    assert exits and 'tilt' in exits[0], exits
    rez.append('inclinare')
    # coborare prea rapida in TOUCHDOWN_DESCENT -> EXIT
    v, sup, exits = ts.build_extnav()
    t = ts.ruleaza_extnav(v, sup, ['MOVE'] * 3, 100.0)
    v.vz = 2.5
    ts.ruleaza_extnav(v, sup, ['TOUCHDOWN_DESCENT'] * 8, t)
    assert exits and 'descent_rate' in exits[0], exits
    rez.append('coborare')
    return "EXIT la: " + ', '.join(rez) + "; urcare 2.5 m/s tolerata; fara comenzi de mod"


TESTS = [
    ('supervizorul vegheaza fazele noi', test_supervizorul_vegheaza_fazele_noi),
    ('configul secventei si validarea', test_configul_secventei_si_validarea),
    ('parametrii FC ai secventei in fisierele verificate',
     test_parametrii_FC_ai_secventei_in_fisierele_verificate),
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
