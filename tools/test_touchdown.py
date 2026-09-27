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


# --- punctul 1: tranzitiile noi, pe vehiculul simulat din test_extnav_landing

def _app(**kw):
    import test_extnav_landing as tel
    app = tel.App(h=kw.pop('h', 6.0), p=kw.pop('p', (1.0, -0.5)), rate=kw.pop('rate', 0.3),
                  **kw)
    app.handover()
    return app


def _pana_la(app, faza, secs=120.0):
    from nova.extnav_landing import Phase
    st = app.run(secs, stop=(faza, Phase.ABORT, Phase.DONE, Phase.GATE_FAIL))
    assert st == faza, (st, app.states, app.sm.exit_reason)


def test_contact_fara_detectie_recenta():
    """Sub 0.61 m markerul nu incape: contactul vine oricum, cu varsta
    ultimei detectii in eveniment (pentru meta.json); secventa continua,
    iar sus detectiile revin si confirma."""
    from nova.extnav_landing import Phase
    app = _app()
    _pana_la(app, Phase.TOUCHDOWN_DESCENT)
    app.det.enabled = False
    _pana_la(app, Phase.GROUND_HOLD, 30.0)
    c = app.ev('contact')[0]
    assert c['last_det_age_s'] is not None and c['last_det_age_s'] > 1.0, c
    assert c['fc_time_boot_ms'] is not None and 'z_contact' in c
    app.det.enabled = True
    st = app.run(60.0, stop=(Phase.DONE, Phase.ABORT))
    assert st == Phase.DONE, (st, app.states, app.sm.exit_reason)
    return f"contact cu ultima detectie veche de {c['last_det_age_s']:.1f} s -> DONE"


def test_dezarmare_pe_sol_fara_comenzi():
    """FC-ul dezarmeaza pe marker (throttle la zero DISARM_DELAY, failsafe):
    eveniment, NICIO comanda de mod; setul EKF inapoi pe 1 (pentru zborul
    urmator), masina in IDLE."""
    from nova.extnav_landing import Phase
    app = _app()
    _pana_la(app, Phase.GROUND_HOLD)
    n_mod = len(app.v.mode_reqs)
    app.v._live.armed = False
    app.run(2.0)
    assert app.sm.state == Phase.IDLE, app.sm.state
    assert app.v.mode_reqs[n_mod:] == [], app.v.mode_reqs[n_mod:]
    d = app.ev('ground_disarmed')
    assert len(d) == 1 and d[0]['phase'] == Phase.GROUND_HOLD, d
    assert app.v.src_cmds[-1] == 1
    return "dezarmat in GROUND_HOLD: eveniment, 0 comenzi de mod, SRC1, IDLE"


def test_timeout_de_urcare_si_decolare_care_nu_porneste():
    from nova.extnav_landing import Phase
    # urcare prea lenta -> riseup_timeout_s -> EXIT
    app = _app()
    _pana_la(app, Phase.RISEUP)
    app.v.climb_ms = 0.2
    st = app.run(30.0, stop=(Phase.ABORT, Phase.DONE))
    assert st == Phase.ABORT and 'urcarea nu a ajuns' in app.sm.exit_reason, (st, app.sm.exit_reason)
    assert app.v.mode_reqs[-1] == 5 and app.v.src_cmds[-1] == 1        # LOITER, SRC1
    # FC-ul ignora NAV_TAKEOFF -> o reincercare, apoi EXIT
    app2 = _app()
    _pana_la(app2, Phase.GROUND_HOLD)
    app2.v._live.mode = 2                 # accepts nothing: ALT_HOLD-like refusal
    app2.v._live.mode = 4
    import types
    real = app2.v.m.mav.command_long_send
    app2.v.m.mav.command_long_send = lambda *a: (app2.v.takeoffs.append(a[-1])
                                                 if a[2] == 22 else real(*a))
    st = app2.run(20.0, stop=(Phase.ABORT, Phase.DONE))
    assert st == Phase.ABORT and 'decolarea nu a pornit' in app2.sm.exit_reason, (st, app2.sm.exit_reason)
    assert len(app2.v.takeoffs) == 2, app2.v.takeoffs
    return "urcare 0.2 m/s -> EXIT la 15 s; NAV_TAKEOFF ignorat x2 -> EXIT"


def test_hover_confirm_offset_mare_corectat_o_data():
    """Sus, detectia proaspata arata vehiculul departe de marker: o
    corectie orizontala la aceeasi altitudine, reconfirmare, DONE. A doua
    oara -> EXIT."""
    from nova.extnav_landing import Phase
    app = _app()
    _pana_la(app, Phase.HOVER_CONFIRM)
    app.v.p[0] += 1.2                    # deriva reala de 1.2 m
    st = app.run(60.0, stop=(Phase.DONE, Phase.ABORT))
    assert st == Phase.DONE, (st, app.states, app.sm.exit_reason)
    cor = app.ev('hover_correction')
    assert len(cor) == 1 and cor[0]['lateral_m'] > cor[0]['tol'], cor
    import math
    assert math.hypot(*app.v.p) < app.sm.tol_now()
    # a doua abatere dupa corectie -> EXIT
    app2 = _app()
    _pana_la(app2, Phase.HOVER_CONFIRM)
    app2.v.p[0] += 1.2
    def muta(a):
        if a.sm._corrected and not a.sm._correcting and not getattr(a, '_mutat', False):
            a._mutat = True
            a.v.p[0] += 1.2
    st = app2.run(60.0, stop=(Phase.DONE, Phase.ABORT), on_step=muta)
    assert st == Phase.ABORT and 'dupa corectie' in app2.sm.exit_reason, (st, app2.sm.exit_reason)
    return f"1.2 m -> o corectie ({cor[0]['lateral_m']:.2f} m) -> DONE; a doua -> EXIT"


def test_abort_in_fiecare_faza_noua():
    """AUX jos in fiecare faza noua: EXIT ordonat (SRC1, apoi LOITER), fara
    RTL; pe sol la fel (LOITER pe sol = pilotul preia)."""
    from nova.extnav_landing import Phase
    rez = []
    for faza in (Phase.TOUCHDOWN_DESCENT, Phase.GROUND_HOLD, Phase.RISEUP,
                 Phase.HOVER_CONFIRM):
        app = _app()
        _pana_la(app, faza)
        n_src = len(app.v.src_cmds)
        app.v.set_aux(1000)
        st = app.run(10.0, stop=(Phase.ABORT,))
        assert st == Phase.ABORT, (faza, st, app.states)
        assert app.v.src_cmds[n_src:] == [1], (faza, app.v.src_cmds)
        assert app.v.mode_reqs[-1] == 5 and 6 not in app.v.mode_reqs, (faza, app.v.mode_reqs)
        assert 'AUX jos' in app.ev('exit')[-1]['reason']
        rez.append(faza)
    return "EXIT ordonat in " + ', '.join(rez)


def test_VPE_pe_sol_tine_EKF_si_fara_el_pica():
    """Decizia din faza A: pe sol, fara vedere, EKF-ul pierde pozitia dupa
    7 s si supervizorul iese. Cu VPE din pozitia de la contact, un sol
    lung (8 s) trece; fara el, acelasi sol duce la EXIT pe EKF."""
    from nova.extnav_landing import ExtNavConfig, Phase
    import test_extnav_landing as tel
    cfg = ExtNavConfig(aux_channel=tel.AUX, touchdown_hold_s=8.0)
    app = _app(cfg=cfg)
    st = app.run(180.0, stop=(Phase.DONE, Phase.ABORT))
    assert st == Phase.DONE, (st, app.states, app.sm.exit_reason)
    n = app.sm.n_ground_vpe
    app2 = _app(cfg=ExtNavConfig(aux_channel=tel.AUX, touchdown_hold_s=8.0))
    app2.sm._ground_vpe = lambda now: None
    st = app2.run(180.0, stop=(Phase.DONE, Phase.ABORT))
    assert st == Phase.ABORT and 'ekf' in app2.sm.exit_reason, (st, app2.sm.exit_reason)
    return f"8 s pe sol: cu VPE ({n} mesaje) -> DONE; fara -> EXIT ({app2.sm.exit_reason[:40]})"


def test_throttle_la_minim_pe_sol_avertizeaza():
    from nova.extnav_landing import Phase
    app = _app()
    _pana_la(app, Phase.GROUND_HOLD)
    rc = list(app.v._live.rc)
    rc[2] = 1000
    app.v._live.rc = tuple(rc)
    app.run(0.5)
    w = app.ev('throttle_low_ground')
    assert len(w) == 1 and w[0]['pwm'] == 1000, w
    assert any('risc dezarmare' in t for t in app.v.statustexts)
    return "throttle 1000 pe sol -> avertisment OSD + STATUSTEXT, o data"


TESTS = [
    ('contact fara detectie recenta', test_contact_fara_detectie_recenta),
    ('dezarmare pe sol fara comenzi', test_dezarmare_pe_sol_fara_comenzi),
    ('timeout de urcare si decolare care nu porneste',
     test_timeout_de_urcare_si_decolare_care_nu_porneste),
    ('HOVER_CONFIRM: offset mare corectat o data',
     test_hover_confirm_offset_mare_corectat_o_data),
    ('abort in fiecare faza noua', test_abort_in_fiecare_faza_noua),
    ('VPE pe sol tine EKF-ul; fara el pica', test_VPE_pe_sol_tine_EKF_si_fara_el_pica),
    ('throttle la minim pe sol avertizeaza', test_throttle_la_minim_pe_sol_avertizeaza),
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
