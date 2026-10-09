#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Altitudinile secventei (B2, 02.10.2026).

    python3 tools/test_altitudes.py

  - urcarea dupa touchdown (15.1.2, 15.2.7): la h_ref + alt_riseup deasupra
    markerului, NAV_TAKEOFF deasupra HOME, fara GNSS pana la COMPLETE;
    alt_riseup implicit 5.5, sub 5.0 avertisment (pornire si preflight);
  - fereastra de handover din config (handover_alt_min 4.0 / 0 = fara
    minim cu avertisment, handover_alt_max 20.0; cheia veche
    handover_alt_min_m inca citita, cu avertisment), validata, ajunsa in
    poarta prin cablajul real (cablaj_extnav), pe ambele ghidari;
  - altitudinea de activare pe OSD; scara de coborare 4 -> 2 -> 1 m.
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import board_window as bw                         # noqa: E402
from nova import config as nova_config                      # noqa: E402
from nova.extnav_landing import ExtNavConfig, Phase         # noqa: E402
from nova.handover import HandoverGate                      # noqa: E402
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.vehicle import Vehicle                            # noqa: E402
import test_extnav_landing as tel                           # noqa: E402
import test_handover as th                                  # noqa: E402


def gate_la(alt, lo, hi=20.0):
    v = th.StubVehicle(alt)
    ov = OverrideMonitor(v)
    gate = HandoverGate(v, ov, autonomy_enabled=True, alt_min_m=lo, alt_max_m=hi)
    gate.on_aux_requested(100.0)
    ov.observe(100.0, True)
    th.settle(gate)
    return gate.check(th.SETTLED, **th.GOOD)


def cablaj(cfg, gate_kw=None):
    import nova_pi
    v = Vehicle('udpin:127.0.0.1:1')
    v.m = types.SimpleNamespace(mav=types.SimpleNamespace(), target_system=1,
                                target_component=1)
    a = types.SimpleNamespace(monitor=False, monitor_motiv=None, no_ascent=False,
                              no_authority=True, conv=2, fast_descent=False)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        gate, sup, sm, _ = nova_pi.cablaj_extnav(
            a, cfg, v, OverrideMonitor(v), gate_kw or {}, 8, 1500,
            lambda r: None, lambda n, i: None)
    return gate, out.getvalue()


def porneste_main(cfg_dict):
    """tools/nova_pi.py main() on a temporary config, up to the serial-port
    check (stubbed as busy: exit 3) - the startup path, for real."""
    import nova_pi
    from nova import serial_guard
    d = tempfile.mkdtemp(dir=os.environ.get('TMPDIR'))
    p = os.path.join(d, 'nova.json')
    base = json.load(open(os.path.join(REPO, 'config', 'nova.json')))
    base.update(cfg_dict)
    base = {k: v for k, v in base.items() if v is not None}
    with open(p, 'w') as f:
        json.dump(base, f)

    def ocupat(*a, **k):
        raise serial_guard.PortBusy('test: port ocupat')
    vechi_port, vechi_argv = serial_guard.ensure_port_free, sys.argv
    serial_guard.ensure_port_free = ocupat
    sys.argv = ['nova_pi.py', '--config', p, '--conn', '/dev/fake0']
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            rc = nova_pi.main()
    finally:
        serial_guard.ensure_port_free, sys.argv = vechi_port, vechi_argv
        try:
            os.remove(p)
            os.rmdir(d)
        except OSError:
            pass
    return rc, out.getvalue()


# --- B2.1: the climb after touchdown --------------------------------------------

def test_urcarea_h_ref_plus_alt_riseup_fara_GNSS_pana_la_COMPLETE():
    """Implicit 5.5. Secventa completa cu home cu 2 m SUB solul markerului
    (h_ref = 2.0): NAV_TAKEOFF la h_ref + 5.5 = 7.5 m deasupra HOME, adica
    5.5 m deasupra markerului; xy tinut, SRC1 abia la COMPLETE (niciun set
    1 cerut inainte, deci fara GNSS in urcare). Negativ: cu h_ref ignorat
    (home = sol) tinta ar fi 5.5 - verificat ca NU e ce s-a trimis."""
    assert ExtNavConfig().alt_riseup == 5.5
    assert nova_config.DEFAULTS['alt_riseup'] == 5.5
    v, w = nova_config.touchdown_settings(nova_config.DEFAULTS)
    assert v['alt_riseup'] == 5.5 and w == []
    app = tel.App(h=8.0, rate=0.2)
    app.v.rel_alt_offset = 2.0
    app.handover()
    t_complete = {}

    def hook(a):
        if a.sm.state == Phase.COMPLETE and 't' not in t_complete:
            t_complete['t'] = a.v.now
    st = app.run(200.0, stop=(Phase.DONE, Phase.ABORT, Phase.GATE_FAIL), on_step=hook)
    assert st == Phase.DONE, (app.states, app.sm.exit_reason)
    c = app.ev('contact')[0]
    assert abs(c['h_ref'] - 2.0) < 0.05, c
    assert app.v.takeoffs and abs(app.v.takeoffs[0] - 7.5) < 0.05, app.v.takeoffs
    assert abs(app.v.takeoffs[0] - 5.5) > 1.0
    assert abs(app.v.h - 5.5) < 0.35, app.v.h
    src1 = [e for e in app.v.wire if e[:2] == ('src', 1)]
    assert len(src1) == 1 and src1[0][2] >= t_complete['t'], (src1, t_complete)
    return (f"NAV_TAKEOFF {app.v.takeoffs[0]:.2f} m deasupra home = h_ref "
            f"{c['h_ref']:.2f} + 5.5; {app.v.h:.2f} m deasupra markerului; SRC1 la COMPLETE")


def test_alt_riseup_sub_5_avertisment_la_pornire_si_in_preflight():
    """alt_riseup 4.9 -> avertisment cu textul regulii; spus de calea de
    pornire din tools/nova_pi.py (main, pana la portul serial) si de
    preflight (check_touchdown). Negativ: 5.0 nu avertizeaza. nova.json
    pastreaza 2.5 (valoarea de test a echipei, deliberat)."""
    v, w = nova_config.touchdown_settings({'alt_riseup': 4.9})
    assert len(w) == 1 and 'alt_riseup = 4.9 m < 5.0 m' in w[0] and '15.1.2' in w[0], w
    assert nova_config.touchdown_settings({'alt_riseup': 5.0})[1] == []
    assert nova_config.load().get('alt_riseup') == 5.5   # team, d38f23c
    rc, out = porneste_main({'alt_riseup': 4.9})
    assert rc == 3, (rc, out[-400:])
    assert '[bord] ATENTIE: alt_riseup = 4.9 m < 5.0 m' in out, out[-600:]
    import preflight_check as pf
    r = pf.check_touchdown({'alt_riseup': 4.9})
    assert r.status == pf.OK and 'ATENTIE' in r.detail and '4.9' in r.detail, r.detail
    r5 = pf.check_touchdown({'alt_riseup': 5.5})
    assert 'ATENTIE' not in r5.detail
    return "4.9 m: avertisment la pornire si in preflight; 5.0 m: nimic"


# --- B2.2: the handover window ------------------------------------------------------

def test_fereastra_handover_din_config_validata():
    """Implicit 4.0-20.0, si nova.json la fel, fara avertisment. 0 = fara
    minim, cu avertisment. Cheia veche singura: folosita + avertisment de
    redenumire; amandoua: cea noua castiga (si se spune). Refuz: nenumeric,
    negativ, max <= min."""
    w, warn = nova_config.handover_settings({})
    assert (w['alt_min_m'], w['alt_max_m']) == (4.0, 20.0) and warn == []
    w, warn = nova_config.handover_settings(nova_config.load())
    assert (w['alt_min_m'], w['alt_max_m']) == (4.0, 20.0) and warn == [], (w, warn)
    w, warn = nova_config.handover_settings({'handover_alt_min': 0})
    assert w['alt_min_m'] == 0.0 and len(warn) == 1 and 'FARA minim' in warn[0], warn
    w, warn = nova_config.handover_settings({'handover_alt_min_m': 3.0})
    assert w['alt_min_m'] == 3.0 and w['source'] == 'handover_alt_min_m'
    assert len(warn) == 1 and 'redenumita' in warn[0], warn
    w, warn = nova_config.handover_settings({'handover_alt_min_m': 3.0,
                                             'handover_alt_min': 4.5})
    assert w['alt_min_m'] == 4.5 and 'ignorata' in warn[0], warn
    for rau in ({'handover_alt_min': 'jos'}, {'handover_alt_min': -1},
                {'handover_alt_max': -2}, {'handover_alt_min': 8, 'handover_alt_max': 8},
                {'handover_alt_min': 10, 'handover_alt_max': 6},
                {'handover_alt_min_m': 'x'}, {'handover_alt_max': True}):
        try:
            nova_config.handover_settings(rau)
        except ValueError:
            continue
        raise AssertionError(f"acceptat: {rau}")
    return "4-20 implicit; 0 avertizat; cheia veche citita + avertisment; 6 refuzuri"


def test_poarta_3_9_refuzat_4_0_acceptat_si_minim_0():
    """Cu minimul 4.0: 3.9 m refuzat, 4.0 m acceptat. Cu minimul 0: 1.0 m
    acceptat (si avertismentul de pornire e spus de main)."""
    ok, why = gate_la(3.9, 4.0)
    assert ok is False and 'altitudine' in why, why
    ok, why = gate_la(4.0, 4.0)
    assert ok is True, why
    ok, why = gate_la(1.0, 0.0)
    assert ok is True, why
    rc, out = porneste_main({'handover_alt_min': 0})
    assert rc == 3 and 'ATENTIE: handover_alt_min = 0: FARA minim' in out, out[-600:]
    rc, out = porneste_main({'handover_alt_min': 10, 'handover_alt_max': 6})
    assert rc == 2 and 'NU PORNESC' in out, out[-400:]
    return "3.9 refuzat, 4.0 acceptat; minim 0: 1 m acceptat + avertisment; max<=min: refuz"


def test_fereastra_ajunge_in_poarta_prin_cablajul_real():
    """cablaj_extnav: fereastra din config ajunge in poarta (si in linia de
    pornire); gate_kw explicit (main) are prioritate. cablaj_plnd la fel.
    Cheia veche singura ajunge si ea. Plafonul supervizorului (30 m) nu
    depinde de handover_alt_max."""
    import nova_pi
    cfg = nova_config.load()
    gate, out = cablaj(cfg)
    assert (gate.alt_min_m, gate.alt_max_m) == (4.0, 20.0)
    assert 'poarta: 4.0-20 m' in out, out
    cfg2 = dict(cfg, handover_alt_min=6.0, handover_alt_max=15.0)
    gate, out = cablaj(cfg2)
    assert (gate.alt_min_m, gate.alt_max_m) == (6.0, 15.0) and 'poarta: 6.0-15 m' in out
    cfg3 = dict(cfg, handover_alt_min=None, handover_alt_min_m=3.5)
    gate, _ = cablaj(cfg3)
    assert gate.alt_min_m == 3.5
    gate, _ = cablaj(cfg2, {'alt_min_m': 5.0, 'alt_max_m': 12.0})
    assert (gate.alt_min_m, gate.alt_max_m) == (5.0, 12.0)
    v = Vehicle('udpin:127.0.0.1:1')
    v.m = types.SimpleNamespace(mav=types.SimpleNamespace(), target_system=1,
                                target_component=1)
    a = types.SimpleNamespace(monitor=False, monitor_motiv=None, no_ascent=False,
                              no_authority=True, conv=2, fast_descent=False)
    with contextlib.redirect_stdout(io.StringIO()):
        gp, sup_p, _, _ = nova_pi.cablaj_plnd(a, cfg2, v, OverrideMonitor(v), {}, 8,
                                              1500, lambda r: None, lambda n, i: None)
    assert (gp.alt_min_m, gp.alt_max_m) == (6.0, 15.0)
    _g, out = cablaj(dict(cfg, handover_alt_max=12.0))
    import re
    assert re.search(r'plafon 15\.2\.4: 30 m', out), out
    src = open(os.path.join(HERE, 'nova_pi.py')).read()
    m = src[src.index('def main():'):]
    assert m.index('handover_settings(cfg)') < m.index('ensure_port_free')
    assert "gate_kw = {'alt_min_m': _hw['alt_min_m'], 'alt_max_m': _hw['alt_max_m']}" in m
    return "config -> cablaj_extnav / cablaj_plnd -> poarta; linia de pornire cu fereastra reala"


# --- B2.3 / B2.4 -----------------------------------------------------------------------

def test_OSD_altitudinea_de_activare():
    """Linia ENGAGE de pe OSD spune altitudinea la care s-a acceptat
    handover-ul (evenimentul 'engage' o poarta)."""
    t, c = bw.OsdMesaje.format('engage', {'n': 1, 'lateral_m': 0.83, 'alt': 4.04})
    assert t == 'ENGAGE la 4.0 m: o detectie, lateral 0.83 m' and c == bw.VERDE, t
    assert bw.OsdMesaje.format('engage', {'n': 1})[0] == 'ENGAGE: o detectie'
    app = tel.App(h=6.0, rate=0.3)
    app.handover()
    app.run(30.0, stop=(Phase.ENGAGE, Phase.ABORT))
    e = app.ev('engage')[0]
    assert abs(e['alt'] - 6.0) < 0.05, e
    m = bw.OsdMesaje()
    m.note('engage', e, 1.0)
    assert m.linii(2.0)[0][0].startswith('ENGAGE la 6.0 m'), m.linii(2.0)
    return t


def test_scara_de_la_4_m_e_4_2_1():
    """Scara porneste de la altitudinea REALA de handover: de la 4 m,
    DESCEND la 2 m, apoi la 1 m (FINAL_H_M), FINAL_ALIGN - fara treapta
    reziduala. Negativ de control: de la 8 m, 8 -> 4 -> 2 -> 1."""
    out = []
    for h, scara in ((4.0, [2.0, 1.0]), (8.0, [4.0, 2.0, 1.0])):
        app = tel.App(h=h, p=(0.6, -0.3), rate=0.2)
        app.handover()
        st = app.run(150.0, stop=(Phase.TOUCHDOWN_DESCENT, Phase.ABORT, Phase.GATE_FAIL))
        assert st == Phase.TOUCHDOWN_DESCENT, (h, app.states, app.sm.exit_reason)
        assert app.h_targets == scara, (h, app.h_targets)
        assert app.states.count(Phase.FINAL_ALIGN) == 1
        out.append(f"{h:g} -> " + ' -> '.join(f"{x:g}" for x in app.h_targets))
    return '; '.join(out)


TESTS = [
    ('urcarea: h_ref + alt_riseup, fara GNSS pana la COMPLETE',
     test_urcarea_h_ref_plus_alt_riseup_fara_GNSS_pana_la_COMPLETE),
    ('alt_riseup < 5: avertisment la pornire si in preflight',
     test_alt_riseup_sub_5_avertisment_la_pornire_si_in_preflight),
    ('fereastra de handover din config, validata', test_fereastra_handover_din_config_validata),
    ('poarta: 3.9 refuzat, 4.0 acceptat, minim 0', test_poarta_3_9_refuzat_4_0_acceptat_si_minim_0),
    ('fereastra ajunge in poarta prin cablajul real',
     test_fereastra_ajunge_in_poarta_prin_cablajul_real),
    ('OSD: altitudinea de activare', test_OSD_altitudinea_de_activare),
    ('scara de la 4 m: 4 -> 2 -> 1', test_scara_de_la_4_m_e_4_2_1),
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
