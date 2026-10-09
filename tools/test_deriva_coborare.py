#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Deriva in DESCEND (10.10.2026): coborarea se opreste si se intoarce in MOVE.

    python3 tools/test_deriva_coborare.py

On the simulated vehicle of tools/test_extnav_landing.py: a side gust pushes
the vehicle while it goes straight down. Two fresh detections in a row
above descend_drift_factor x tol(h) -> DESCEND stops at the current height
and goes back to MOVE; the sequence then completes. Negatives: no gust ->
no stop; one outlier detection -> no stop; factor 0 -> off.
"""

import collections
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import test_extnav_landing as tel                               # noqa: E402
from nova.extnav_landing import ExtNavConfig, Phase             # noqa: E402

DONE = (Phase.ABORT, Phase.DONE)


def _app(gust_ms=0.0, gust_s=0.0, factor=2.0):
    cfg = ExtNavConfig(aux_channel=tel.AUX, descend_drift_factor=factor)
    app = tel.App(h=6.0, p=(0.3, -0.2), rate=0.3, cfg=cfg)
    st = {'t0': None}

    def gust(a):
        # the first DESCEND only: wind on +x for gust_s seconds
        if a.sm.state == Phase.DESCEND and gust_ms > 0:
            if st['t0'] is None:
                st['t0'] = a.t
            if a.t - st['t0'] <= gust_s:
                a.v.p[0] += gust_ms * 0.02
    app.handover()
    final = app.run(300.0, stop=DONE, on_step=gust)
    return app, final


def _after(states, a, b):
    return any(x == a and y == b for x, y in zip(states, states[1:]))


def test_rafala_in_coborare_opreste_si_reia_MOVE():
    app, final = _app(gust_ms=1.6, gust_s=2.5)
    ev = app.ev('descend_drift')
    assert ev, f"fara oprire pe deriva; stari {app.states}"
    assert _after(app.states, Phase.DESCEND, Phase.MOVE), app.states
    e = ev[0]
    assert e['lateral_m'] > e['limit_m'] and e['h'] >= 1.0, e
    assert final == Phase.DONE, (final, app.sm.exit_reason, app.states[-8:])
    return (f"oprit la {e['h']:.2f} m cu {e['lateral_m']:.2f} m > "
            f"{e['limit_m']:.2f}; secventa DONE")


def test_NEGATIV_fara_rafala_nu_se_opreste():
    app, final = _app()
    assert not app.ev('descend_drift'), app.ev('descend_drift')
    assert final == Phase.DONE, (final, app.sm.exit_reason)
    return "fara deriva: nicio oprire, DONE"


def test_NEGATIV_o_singura_detectie_aberanta_nu_opreste():
    """One wild estimate during DESCEND, then normal ones: no stop."""
    cfg = ExtNavConfig(aux_channel=tel.AUX)
    app = tel.App(h=6.0, p=(0.3, -0.2), rate=0.3, cfg=cfg)
    Fake = collections.namedtuple('Fake', 't x y lateral_m')
    done = {'n': 0}

    def outlier(a):
        if a.sm.state == Phase.DESCEND and not done['n']:
            done['n'] = 1
            now = 1000.0 + a.t
            a.sm.last_est = Fake(now, 5.0, 0.0, 5.0)
    app.handover()
    final = app.run(300.0, stop=DONE, on_step=outlier)
    assert done['n'] == 1, "testul nu a ajuns in DESCEND"
    assert not app.ev('descend_drift'), app.ev('descend_drift')
    assert final == Phase.DONE, (final, app.sm.exit_reason)
    return "o estimare de 5 m, singura: ignorata"


def test_NEGATIV_factor_0_dezactiveaza():
    app, final = _app(gust_ms=1.6, gust_s=2.5, factor=0.0)
    assert not app.ev('descend_drift'), app.ev('descend_drift')
    return f"factor 0: fara oprire (stare finala {final})"


TESTS = [
    ('rafala in coborare: oprire si MOVE', test_rafala_in_coborare_opreste_si_reia_MOVE),
    ('NEGATIV: fara rafala nu se opreste', test_NEGATIV_fara_rafala_nu_se_opreste),
    ('NEGATIV: o singura detectie aberanta', test_NEGATIV_o_singura_detectie_aberanta_nu_opreste),
    ('NEGATIV: factor 0 dezactiveaza', test_NEGATIV_factor_0_dezactiveaza),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            print(f"  OK    {name}   ({fn()})")
        except Exception as e:                          # noqa: BLE001
            fails += 1
            print(f"  ESEC  {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
