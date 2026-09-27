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


TESTS = [
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
