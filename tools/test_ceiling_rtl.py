#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Plafonul de 30 m deasupra home -> abort -> RTL (regula 15.2.4, B1 02.10.2026).

    python3 tools/test_ceiling_rtl.py

Fara FC: vehiculul de mucava din tools/test_extnav_landing.py (FC-ul raspunde
la SET_EKF_SOURCE_SET cu ACK si reset de pozitie, adopta modurile cu
intarziere) pe cablajul sincron si pe cel cu fire (test_threads_integration),
plus un FC de mucava pe "fir" peste Vehicle-ul REAL (cozile URGENT / TX,
_handle pentru COMMAND_ACK / HEARTBEAT / GLOBAL_POSITION_INT / GPS_RAW_INT).

Ce fixeaza, fiecare cu cazul negativ (§5.11):
  - depasirea in ORICE faza activa duce la SRC1 -> (ack) -> RTL; RTL nu
    pleaca niciodata inaintea unui ACCEPTED atribuit cererii de set 1;
  - zgomot: sub ceiling_confirm_n esantioane distincte nu declanseaza,
    acelasi esantion vazut de 200 de ori nu declanseaza;
  - fara ack -> LAND, zero RTL; RTL refuzat -> LAND; LAND refuzat -> raportat;
  - poarta de cadru (GPI ~ GPS), timeout 1.0 s, fara STATUSTEXT;
  - inactiv in afara procedurii; clasa §5.66; URGENT; firele reale;
  - plafonul vechi (fata de handover) nu mai declanseaza; fara fence nicaieri;
  - ack-ul vechi (stale) nu mai trece drept ack nou: EXIT normal, ENGAGE.
"""

import contextlib
import io
import math
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from pymavlink import mavutil                               # noqa: E402

from nova import config as nova_config                      # noqa: E402
from nova.concurrency import Heartbeat, Latest              # noqa: E402
from nova.ekf_source import EkfSourceManager                # noqa: E402
from nova import extnav_landing as enl                      # noqa: E402
from nova.extnav_landing import (ExtNavConfig, ExtNavLanding,  # noqa: E402
                                 Phase, SRC2_PHASES)
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.safety import (Action, CEILING_PHASES,            # noqa: E402
                         EXTNAV_PHASES, ExtNavSupervisor)
from nova.supervisor_thread import SupervisorThread         # noqa: E402
from nova.vehicle import (MODE_ALT_HOLD, MODE_GUIDED,       # noqa: E402
                          MODE_LAND, MODE_LOITER, MODE_RTL,
                          MODE_STABILIZE, Vehicle)
import test_extnav_landing as tel                           # noqa: E402
import test_safety as ts                                    # noqa: E402
import test_threads_integration as tti                      # noqa: E402

ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED
END = (Phase.ABORT, Phase.DONE, Phase.GATE_FAIL, Phase.REJECT, Phase.IDLE)


# --- helpers -----------------------------------------------------------------

def app_sync(**veh):
    """Cablajul sincron (on_exit), cu FC-ul de mucava reglat ca in SITL:
    ACK la 10 ms, resetul pozitiei EKF la 0.18 s dupa SRC1."""
    app = tel.App(rate=0.2)
    app.v.src_ack_delay_s = veh.pop('ack', 0.01)
    app.v.src_reset_delay_s = veh.pop('reset', 0.18)
    for k, val in veh.items():
        setattr(app.v, k, val)
    return app


def app_fire(**veh):
    app = tti.AppFire(rate=0.2)
    app.v.src_ack_delay_s = veh.pop('ack', 0.01)
    app.v.src_reset_delay_s = veh.pop('reset', 0.18)
    for k, val in veh.items():
        setattr(app.v, k, val)
    return app


def step(app):
    """Exactly one 20 ms step, on either wiring."""
    app.run(0.01)
    return app.sm.state


def reach(app, phase, pred=None, secs=200.0):
    app.handover()
    end = app.t + secs
    while app.t < end:
        st = step(app)
        if st == phase and (pred is None or pred(app)):
            return st
        if st in END or st in (Phase.EXIT, Phase.EXIT_RTL):
            break
    raise AssertionError(f"{phase} neatins: {app.states} ({app.sm.exit_reason})")


def breach(app, h_above=40.0):
    """relative_alt = h + offset: over the ceiling whatever the phase does."""
    app.v.rel_alt_offset = h_above
    return len(app.v.wire), app.t


def until(app, states=END, secs=15.0):
    end = app.t + secs
    while app.t < end:
        if step(app) in states:
            break
    return app.sm.state


def modes(w):
    return [e[1] for e in w if e[0] == 'mode']


def first(w, kind, val, res=None):
    for i, e in enumerate(w):
        if e[0] == kind and e[1] == val and (res is None or e[3] == res):
            return i
    return None


def assert_src1_ack_rtl(w, what=''):
    """The order on the wire: SRC1, its ACCEPTED, then RTL - and RTL once,
    never before the ack."""
    assert w and w[0][:2] == ('src', 1), (what, w[:3])
    i_ack = first(w, 'ack', 1, ACCEPTED)
    i_rtl = first(w, 'mode', MODE_RTL)
    assert i_ack is not None and i_rtl is not None, (what, w)
    assert i_ack < i_rtl, (what, w)
    assert modes(w) == [MODE_RTL], (what, modes(w))
    assert all(e[1] == 1 for e in w if e[0] == 'src'), (what, w)
    return w[i_rtl][2] - w[0][2]


class CeilStub(ts.ExtNavStub):
    """The supervisor's vehicle with relative_alt and the GPI counter."""

    def __init__(self):
        super().__init__()
        self.rel_alt = 10.0
        self.n_gpi = 0
        self.src_cmds = []

    def sample(self, h):
        self.rel_alt = h
        self.n_gpi += 1

    def send_ekf_source_set(self, n, urgent=False):
        self.src_cmds.append(n)
        return True


def sup_stub(confirm_n=3, ceiling=30.0):
    v = CeilStub()
    v.rc_t = 100.0
    exits = []

    def on_exit(reason, rtl=False):
        exits.append((reason, rtl))
    sup = ExtNavSupervisor(v, verbose=False, on_exit=on_exit,
                           ceiling_agl_m=ceiling, ceiling_confirm_n=confirm_n)
    return v, sup, exits


def tick(v, sup, phase, t, h=None):
    if h is not None:
        v.sample(h)
    v.rc_t = v.ekf_t = v.hb_t = t
    return sup.update(t, None, phase)


# --- B1.3: the RTL path from every active phase ------------------------------

def test_depasire_in_fiecare_faza_activa_SRC1_ack_RTL():
    """Depasirea in fiecare faza ACTIVA atinsa de secventa (GATE_SEARCH,
    ENGAGE - si cu GUIDED inca in zbor -, MOVE, CENTER_CHECK, DESCEND,
    FINAL_ALIGN, TOUCHDOWN_DESCENT, GROUND_HOLD, RISEUP, HOVER_CONFIRM):
    EXIT_RTL, pe fir SRC1 -> ACK -> RTL, RTL confirmat, ABORT. Negativ: nicio
    comanda RTL inainte de ACK si nicio comanda de mod in afara de RTL."""
    cazuri = [(Phase.GATE_SEARCH, None), (Phase.ENGAGE, None),
              (Phase.ENGAGE, lambda a: a.sm._sub == 'mode'),
              (Phase.MOVE, None), (Phase.CENTER_CHECK, None),
              (Phase.DESCEND, None), (Phase.FINAL_ALIGN, None),
              (Phase.TOUCHDOWN_DESCENT, None), (Phase.GROUND_HOLD, None),
              (Phase.RISEUP, None), (Phase.HOVER_CONFIRM, None)]
    out = []
    for phase, pred in cazuri:
        app = app_sync()
        reach(app, phase, pred)
        i0, t0 = breach(app)
        st = until(app)
        w = app.v.wire[i0:]
        # an SRC2 switch already under way in ENGAGE may precede the breach
        i1 = first(w, 'src', 1)
        assert i1 is not None and all(e[:2] != ('src', 2) for e in w[i1:]), w
        w = w[i1:]
        assert st == Phase.ABORT, (phase, app.states, app.sm.exit_reason)
        assert Phase.EXIT_RTL in app.states and Phase.EXIT not in app.states, app.states
        dt = assert_src1_ack_rtl(w, phase)
        assert app.v.mode == MODE_RTL, (phase, app.v.mode)
        d = app.ev('exit_done')[-1]
        assert d['kind'] == 'rtl' and d['result'] == 'rtl' and not d['passive'], d
        assert app.sup.latched == Action.EXIT_RTL and 'ceiling_agl' in app.sm.exit_reason
        assert any('PLAFON' in s for s in app.v.statustexts), app.v.statustexts
        out.append(f"{phase}{'(GUIDED in zbor)' if pred else ''} {dt:.2f}s")
    return '; '.join(out)


def test_plafonul_activ_exact_in_lista_pozitiva():
    """Supervizorul, pe faze: activ in TOATE fazele din CEILING_PHASES
    (inclusiv CONTACT, LAND, TOUCHDOWN, pe care secventa normala le
    traverseaza intr-un pas sau deloc), inactiv in IDLE, REJECT, GATE_FAIL,
    EXIT, EXIT_RTL, COMPLETE, ABORT, DONE si intr-o faza necunoscuta - cu
    vehiculul la 80 m deasupra home. Masina de stari porneste calea RTL din
    CONTACT / LAND / TOUCHDOWN la fel."""
    assert CEILING_PHASES == EXTNAV_PHASES, CEILING_PHASES
    for ph in ('GATE_SEARCH', 'ENGAGE', 'GROUND_HOLD', 'RISEUP', 'CONTACT',
               'LAND', 'TOUCHDOWN', 'HOVER_CONFIRM'):
        assert ph in CEILING_PHASES, ph
    for phase in CEILING_PHASES:
        v, sup, exits = sup_stub()
        t = 100.0
        acts = [tick(v, sup, phase, t + 0.05 * i, h=31.0) for i in range(3)]
        assert acts[-1] == Action.EXIT_RTL and exits and exits[0][1], (phase, acts)
        assert sup.abort_rtl and sup.abort.is_set() and v.mode_reqs == []
    inactive = ('IDLE', 'REJECT', 'GATE_FAIL', 'EXIT', 'EXIT_RTL', 'COMPLETE',
                'ABORT', 'DONE', 'NECUNOSCUT')
    for phase in inactive:
        v, sup, exits = sup_stub()
        for i in range(50):
            act = tick(v, sup, phase, 100.0 + 0.05 * i, h=80.0)
        assert act == Action.NONE and not exits and not sup.abort.is_set(), phase
        assert v.mode_reqs == [] and v.src_cmds == []
    # the state machine runs the RTL path from the one-step phases too
    for phase in (Phase.CONTACT, Phase.LAND, Phase.TOUCHDOWN):
        app = app_sync()
        app.sm.state = phase
        app.sm.request_exit('ceiling_agl: test', rtl=True)
        assert app.sm.state == Phase.EXIT_RTL and app.v.src_cmds[-1] == 1, phase
    return (f"{len(CEILING_PHASES)} faze active; {len(inactive)} inactive la "
            f"80 m: zero comenzi")


def test_zgomot_esantioane_distincte_nu_iteratii():
    """3 esantioane DISTINCTE la rand peste prag. Negativ: 2 peste, apoi
    unul sub -> nimic; ACELASI esantion (acelasi n_gpi) vazut de 200 de ori
    -> nimic; un esantion sub intre ele reseteaza; relative_alt None ->
    nimic (cazul monitorului de legatura); 30.0 exact nu e peste."""
    v, sup, exits = sup_stub()
    t = 100.0
    tick(v, sup, 'DESCEND', t, h=31.0)
    for i in range(200):                         # same sample, 200 iterations
        assert tick(v, sup, 'DESCEND', t + 0.005 * i) == Action.NONE
    assert not exits and sup._ceil_count == 1
    tick(v, sup, 'DESCEND', t + 1.1, h=31.0)
    assert tick(v, sup, 'DESCEND', t + 1.2, h=30.0) == Action.NONE   # at, not over
    assert sup._ceil_count == 0 and not exits
    tick(v, sup, 'DESCEND', t + 1.3, h=31.0)
    tick(v, sup, 'DESCEND', t + 1.4, h=31.0)
    assert not exits, "a declansat dupa 2 esantioane"
    assert tick(v, sup, 'DESCEND', t + 1.5, h=31.0) == Action.EXIT_RTL
    assert exits and exits[0][1] is True and 'ceiling_agl' in exits[0][0]
    # relative_alt None / no counter: no verdict from this monitor
    v, sup, exits = sup_stub()
    v.rel_alt = None
    for i in range(20):
        v.n_gpi += 1
        assert tick(v, sup, 'DESCEND', 100.0 + 0.05 * i) == Action.NONE
    v2, sup2, exits2 = sup_stub()
    del v2.n_gpi
    for i in range(20):
        v2.rel_alt = 50.0
        assert tick(v2, sup2, 'DESCEND', 100.0 + 0.05 * i) == Action.NONE
    assert not exits and not exits2
    # ceiling_confirm_n from the config is the one counted
    v, sup, exits = sup_stub(confirm_n=5)
    acts = [tick(v, sup, 'MOVE', 100.0 + 0.05 * i, h=35.0) for i in range(5)]
    assert acts[:4] == [Action.NONE] * 4 and acts[4] == Action.EXIT_RTL, acts
    return "200 iteratii pe acelasi esantion: 0; 2+sub+2: 0; al 3-lea distinct: RTL"


# --- B1.2 b / e: no ack, refusals ---------------------------------------------

def test_fara_ack_pentru_SRC1_LAND_si_zero_RTL():
    """(c) SRC1 pierdut (niciun ACK, niciodata) cu un ACCEPTED vechi prezent
    (comutarea pe SRC2 din ENGAGE): reincercare la 0.5 s, la 2.0 s LAND,
    confirmat. ZERO RTL pe fir. Cazul pozitiv e testul de mai sus."""
    app = app_sync(src_lost={1})
    reach(app, Phase.DESCEND)
    assert app.v.ekf_src_ack == ACCEPTED, "nu exista ack-ul vechi de SRC2"
    i0, t0 = breach(app)
    st = until(app)
    w = app.v.wire[i0:]
    assert st == Phase.ABORT, app.states
    assert MODE_RTL not in app.v.mode_reqs, app.v.mode_reqs
    assert modes(w) == [MODE_LAND] and app.v.mode == MODE_LAND, modes(w)
    srcs = [e for e in w if e[0] == 'src']
    assert [e[1] for e in srcs] == [1, 1, 1, 1], srcs
    gaps = [round(b[2] - a[2], 2) for a, b in zip(srcs, srcs[1:])]
    # (the first gap is one 20 ms tick short on the callback wiring: the
    # request comes from the supervisor's step, before the machine's clock)
    assert all(0.46 <= g <= 0.53 for g in gaps), gaps
    t_land = w[first(w, 'mode', MODE_LAND)][2] - srcs[0][2]
    assert 1.96 <= t_land <= 2.05, t_land
    assert app.ev('exit_rtl_no_ack') and app.ev('exit_done')[-1]['result'] == 'land'
    assert any('neconfirmat' in s for s in app.v.statustexts), app.v.statustexts
    return f"4 x SRC1 la 0.5 s, LAND la {t_land:.2f} s, 0 RTL"


def test_RTL_refuzat_LAND_iar_LAND_refuzat_raportat():
    """RTL refuzat de FC (5 incercari) -> LAND urgent, confirmat. Si LAND
    refuzat -> predare raportata ('fail'), apoi NIMIC trimis 10 s."""
    app = app_sync(refuse_modes=(MODE_RTL,))
    reach(app, Phase.MOVE)
    i0, _ = breach(app)
    assert until(app) == Phase.ABORT, app.states
    w = app.v.wire[i0:]
    assert modes(w) == [MODE_RTL] * 5 + [MODE_LAND], modes(w)
    assert app.v.mode == MODE_LAND and app.ev('exit_rtl_land')[0]['reason'] == 'rtl_refuzat'
    assert app.ev('exit_done')[-1]['result'] == 'land'
    assert any('RTL refuzat' in s for s in app.v.statustexts)
    # both refused
    app = app_sync(refuse_modes=(MODE_RTL, MODE_LAND))
    reach(app, Phase.MOVE)
    i0, _ = breach(app)
    assert until(app) == Phase.ABORT, app.states
    w = app.v.wire[i0:]
    assert modes(w) == [MODE_RTL] * 5 + [MODE_LAND] * 5, modes(w)
    assert app.ev('exit_done')[-1]['result'] == 'fail'
    n = len(app.v.wire)
    app.run(10.0)
    assert len(app.v.wire) == n, app.v.wire[n:]
    return "RTL x5 refuzat -> LAND; LAND x5 refuzat -> raportat, 0 comenzi dupa"


# --- B1.2 c: the frame gate --------------------------------------------------

def test_poarta_de_cadru_GPI_GPS_si_timeout():
    """Dupa ACK, RTL abia cand GLOBAL_POSITION_INT ajunge la < 3 m de
    GPS_RAW_INT (resetul EKF, 0.18 s): RTL nu pleaca inaintea resetului
    (negativ). Fara reset: RTL oricum, la 1.0 s dupa ACK, cu evenimentul de
    timeout. Nimic nu asteapta un STATUSTEXT (nici in sursa)."""
    app = app_sync(ack=0.01, reset=0.18)
    reach(app, Phase.DESCEND)
    i0, _ = breach(app)
    assert until(app) == Phase.ABORT
    w = app.v.wire[i0:]
    t_src, t_ack = w[0][2], w[first(w, 'ack', 1)][2]
    t_rtl = w[first(w, 'mode', MODE_RTL)][2]
    assert t_rtl - t_src >= 0.18, f"RTL inainte de reset: {t_rtl - t_src:.3f} s"
    assert t_rtl - t_src <= 0.30, t_rtl - t_src
    fr = app.ev('exit_rtl_frame')[0]
    assert fr['ok'] and fr['dist_m'] < 1.0, fr
    # the reset never comes: RTL at 1.0 s after the ack, said
    app = app_sync(ack=0.01, reset=0.18)
    reach(app, Phase.DESCEND)
    app.v.src_reset_delay_s = 60.0          # SRC1 acknowledged, frame never back
    i0, _ = breach(app)
    assert until(app) == Phase.ABORT
    w = app.v.wire[i0:]
    t_ack2 = w[first(w, 'ack', 1)][2]
    t_rtl2 = w[first(w, 'mode', MODE_RTL)][2]
    assert 0.99 <= t_rtl2 - t_ack2 <= 1.06, t_rtl2 - t_ack2
    fr = app.ev('exit_rtl_frame')[0]
    assert not fr['ok'] and fr['dist_m'] > 10.0, fr
    assert app.v.mode == MODE_RTL
    src = open(os.path.join(REPO, 'nova', 'extnav_landing.py')).read()
    assert 'using GPS' not in src and 'statustexts' not in src
    assert not any('using GPS' in s for s in app.v.statustexts)
    return (f"cu reset: RTL la {t_rtl - t_src:.2f} s (ack {t_ack - t_src:.2f} s); "
            f"fara reset: RTL la {t_rtl2 - t_ack2:.2f} s dupa ack")


# --- inactive outside the procedure -------------------------------------------

def test_inactiv_in_afara_procedurii_si_fara_escaladare():
    """La 80 m deasupra home: in IDLE (AUX jos), in REJECT, in GATE_FAIL,
    dupa o secventa COMPLETA (DONE) - zero comenzi. Si un EXIT deja in curs
    (AUX jos in DESCEND, SRC1 cu ACK intarziat) nu devine RTL la depasire:
    se termina in LOITER. Nici un EXIT zavorat de supervizor (EKF invalid)."""
    out = []
    # IDLE
    app = app_sync()
    app.v.rel_alt_offset = 80.0
    app.run(5.0)
    assert app.v.wire == [] and app.sm.state == Phase.IDLE, app.v.wire
    out.append('IDLE')
    # REJECT (above the gate's 12 m) then 80 m
    app = tel.App(h=13.0, rate=0.5)
    app.handover()
    assert app.run(5.0, stop=(Phase.REJECT,)) == Phase.REJECT
    app.v.rel_alt_offset = 80.0
    app.run(3.0)
    assert app.v.wire == [] and app.sm.state == Phase.REJECT
    out.append('REJECT')
    # GATE_FAIL
    app = tel.App(rate=0.0)
    app.handover()
    assert app.run(20.0, stop=(Phase.GATE_FAIL,)) == Phase.GATE_FAIL
    app.v.rel_alt_offset = 80.0
    app.run(3.0)
    assert app.v.wire == [], app.v.wire
    out.append('GATE_FAIL')
    # after a completed sequence
    app = tel.App(rate=0.2)
    app.handover()
    assert app.run(200.0, stop=(Phase.DONE, Phase.ABORT)) == Phase.DONE, app.states
    n = len(app.v.wire)
    app.v.rel_alt_offset = 80.0
    app.run(5.0)
    assert len(app.v.wire) == n and not app.sup.abort.is_set(), app.v.wire[n:]
    out.append('DONE')
    # a normal EXIT under way: no escalation
    app = app_sync(ack=0.15)
    reach(app, Phase.DESCEND)
    app.v.set_aux(1000)
    step(app)
    assert app.sm.state == Phase.EXIT
    i0, _ = breach(app)
    assert until(app) == Phase.ABORT
    w = app.v.wire[i0:]
    assert modes(w) == [MODE_LOITER] and MODE_RTL not in app.v.mode_reqs, w
    assert app.sup.latched != Action.EXIT_RTL
    out.append('EXIT in curs')
    # EXIT latched by the supervisor, then the breach
    app = app_sync(ack=0.15)
    reach(app, Phase.MOVE)
    app.v.ekf_valid_after_switch = False
    until(app, states=(Phase.EXIT,), secs=5.0)
    assert app.sup.latched == Action.EXIT
    breach(app)
    assert until(app) == Phase.ABORT
    assert MODE_RTL not in app.v.mode_reqs and app.sup.latched == Action.EXIT
    out.append('EXIT zavorat')
    return ', '.join(out) + ': fara RTL'


# --- §5.66 class ---------------------------------------------------------------

def test_clasa_5_66_nimic_dupa_predare_pilotul_castiga():
    """Dupa RTL confirmat, FC-ul paraseste RTL (pilotul: LOITER, apoi
    STABILIZE) -> zero comenzi 10 s. Al treilea mod in fereastra de
    livrare a RTL (refuzat, pilotul pune STABILIZE) -> pasiv, nici RTL, nici
    LAND dupa. Al treilea mod inainte de RTL (in asteptarea ACK-ului) ->
    pasiv, ZERO comenzi de mod. AUX jos in EXIT_RTL -> nimic (fara LOITER)."""
    app = app_sync()
    reach(app, Phase.DESCEND)
    breach(app)
    assert until(app) == Phase.ABORT and app.v.mode == MODE_RTL, (app.states, app.v.mode)
    n = len(app.v.wire)
    app.v.mode = MODE_LOITER
    app.run(2.0)
    app.v.mode = MODE_STABILIZE
    app.v.set_aux(1000)
    app.run(8.0)
    assert len(app.v.wire) == n, app.v.wire[n:]
    # third mode while RTL is being delivered
    app = app_sync(refuse_modes=(MODE_RTL,))
    reach(app, Phase.DESCEND)
    i0, _ = breach(app)
    end = app.t + 5.0
    while app.t < end and MODE_RTL not in modes(app.v.wire[i0:]):
        step(app)
    step(app)
    app.v.mode = MODE_STABILIZE
    app.v.mode_pending = []
    assert until(app) == Phase.ABORT, app.states
    w = app.v.wire[i0:]
    assert MODE_LAND not in modes(w) and len(modes(w)) <= 2, modes(w)
    n = len(app.v.wire)
    app.run(5.0)
    assert len(app.v.wire) == n and app.ev('exit_done')[-1]['passive']
    # third mode before any mode command (waiting for the ack)
    app = app_sync(ack=0.4)
    reach(app, Phase.MOVE)
    i0, _ = breach(app)
    until(app, states=(Phase.EXIT_RTL,), secs=1.0)
    step(app)
    assert app.sm.state == Phase.EXIT_RTL
    app.v.mode = MODE_ALT_HOLD
    assert until(app) == Phase.ABORT, app.states
    assert modes(app.v.wire[i0:]) == [], app.v.wire[i0:]
    assert app.ev('exit_rtl_taken') and app.ev('exit_done')[-1]['passive']
    # AUX down during EXIT_RTL changes nothing
    app = app_sync(ack=0.3)
    reach(app, Phase.DESCEND)
    i0, _ = breach(app)
    until(app, states=(Phase.EXIT_RTL,), secs=1.0)
    step(app)
    assert app.sm.state == Phase.EXIT_RTL
    app.v.set_aux(1000)
    assert until(app) == Phase.ABORT, app.states
    assert modes(app.v.wire[i0:]) == [MODE_RTL] and app.v.mode == MODE_RTL, app.v.wire[i0:]
    return "RTL confirmat apoi pilot: 0; al 3-lea mod: pasiv; AUX jos: ignorat"


def test_cerere_pasiva_ramane_pasiva():
    """Modul tinut de noi (GUIDED) deja schimbat de altcineva cand vine
    cererea EXIT_RTL: SRC1 se cere, nicio comanda de mod."""
    app = app_sync()
    reach(app, Phase.MOVE)
    app.v.mode = MODE_STABILIZE
    app.v.mode_pending = []
    i0 = len(app.v.wire)
    app.sm.request_exit('ceiling_agl: test', rtl=True)
    assert app.sm.state == Phase.EXIT_RTL and app.sm.exit_passive
    assert until(app) == Phase.ABORT
    w = app.v.wire[i0:]
    assert modes(w) == [] and w[0][:2] == ('src', 1), w
    return "pasiv: SRC1, 0 moduri"


# --- the real Vehicle: URGENT queue, purge, attribution on the wire ----------

class Msg:
    def __init__(self, kind, **kw):
        self._kind = kind
        self.__dict__.update(kw)

    def get_type(self):
        return self._kind

    def get_srcComponent(self):
        return mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


class WireMav:
    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        if not name.endswith('_send'):
            raise AttributeError(name)

        def f(*a, **k):
            self.sent.append((name, a))
        return f


class WireFC:
    """The FC at the other end of the port: reads what the I/O thread
    writes, answers through Vehicle._handle (COMMAND_ACK, HEARTBEAT,
    GLOBAL_POSITION_INT, GPS_RAW_INT) - the production parsing."""

    def __init__(self, v, mav, lose=(), ack_s=0.01, reset_s=0.18,
                 mode_s=0.1, refuse=()):
        self.v, self.mav = v, mav
        self.lose, self.ack_s, self.reset_s = set(lose), ack_s, reset_s
        self.mode_s, self.refuse = mode_s, set(refuse)
        self.mode = MODE_GUIDED
        self.src = 2
        self.frame_off = (14.0, 8.0)       # marker frame vs GNSS frame, m
        self.pending = []
        self.seen = 0
        self.wire = []                     # ('src', n, t) ('mode', m, t) ('ack', t)

    def tick(self, now):
        for name, a in self.mav.sent[self.seen:]:
            if name != 'command_long_send':
                continue
            cmd = a[2]
            if cmd == mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
                n = int(a[4])
                self.wire.append(('src', n, now))
                if n in self.lose:
                    continue
                self.pending.append((now + self.ack_s, 'ack', n))
                self.pending.append((now + self.reset_s, 'src', n))
            elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                m = int(a[5])
                self.wire.append(('mode', m, now))
                if m not in self.refuse:
                    self.pending.append((now + self.mode_s, 'mode', m))
        self.seen = len(self.mav.sent)
        due = [p for p in self.pending if p[0] <= now]
        self.pending = [p for p in self.pending if p[0] > now]
        for _t, what, val in due:
            if what == 'ack':
                self.wire.append(('ack', val, now))
                self.v._handle(Msg('COMMAND_ACK', result=ACCEPTED,
                                   command=mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET))
            elif what == 'src':
                self.src = val
            else:
                self.mode = val
        v = self.v
        v._handle(Msg('HEARTBEAT', base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                      custom_mode=self.mode))
        v.hb_t = now
        k = enl.math.radians(1.0) * 6371000.0
        lat0, lon0 = 44.4, 26.1
        ox, oy = self.frame_off if self.src == 2 else (0.0, 0.0)
        gx, gy = 3.0, -2.0                   # truth, GNSS frame
        v._handle(Msg('GLOBAL_POSITION_INT', time_boot_ms=int(now * 1000),
                      relative_alt=int(self.rel_alt * 1000),
                      lat=int((lat0 + (gx - ox) / k) * 1e7),
                      lon=int((lon0 + (gy - oy) / (k * math.cos(math.radians(lat0)))) * 1e7)))
        v._handle(Msg('GPS_RAW_INT', fix_type=3,
                      lat=int((lat0 + gx / k) * 1e7),
                      lon=int((lon0 + gy / (k * math.cos(math.radians(lat0)))) * 1e7)))
        live = v._live
        live.x, live.y, live.z = gx - ox, gy - oy, -6.0
        live.have_pos = True
        live.ekf_flags = mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_POS_HORIZ_REL
        live.ekf_t = now
        live.rc = (1500, 1500, 1500, 1500, 1000, 1000, 1000, 2000)
        live.rc_t = now

    rel_alt = 6.0


class WireRig:
    """Real Vehicle (I/O thread configured, stepped by hand), the
    supervisor over its VehicleView in SupervisorThread.step, the state
    machine reading the abort - the nova_pi wiring, forced into `phase`."""

    def __init__(self, phase=Phase.DESCEND, **fc):
        self.mav = WireMav()
        v = Vehicle('/dev/fake0', baud=921600, threaded=True)
        v.m = types.SimpleNamespace(mav=self.mav, target_system=1,
                                    target_component=1,
                                    recv_match=lambda **k: None,
                                    close=lambda: None)
        v.link_verbose = False
        v.params.update({'RC1_TRIM': 1500, 'RC2_TRIM': 1500, 'RC3_TRIM': 1500,
                         'RC4_TRIM': 1500})
        self.v = v
        self.now = 1000.0
        v._note_heartbeat(self.now)
        self.fc = WireFC(v, self.mav, **fc)
        if phase == Phase.GATE_SEARCH:
            self.fc.mode = MODE_LOITER      # the pilot's, during the search
        self.ekf = EkfSourceManager(v, verbose=False, faze=SRC2_PHASES)
        self.events = []
        self.sm = ExtNavLanding(v, None, self.ekf, None,
                                ExtNavConfig(aux_channel=8),
                                on_event=lambda n, i: self.events.append((n, i)),
                                verbose=False)
        self.sup = ExtNavSupervisor(v.view(), verbose=False,
                                    override=OverrideMonitor(v.view()))
        self.sm.attach_supervisor(self.sup)
        v.set_abort_event(self.sup.abort)
        self.faza = Latest(phase)
        self.st = SupervisorThread(self.sup, v, Latest(), self.faza,
                                   clock=lambda: self.now)
        self.queues = []                   # per tick: (urgent items, tx items)
        # the SRC2 switch of ENGAGE, acknowledged: the stale ACCEPTED
        if phase != Phase.GATE_SEARCH:
            v.send_ekf_source_set(2)
            self.ekf.state, self.ekf._comutat = self.ekf.ACTIV, True
        for _ in range(5):
            self.tick()
        assert v.ekf_src_ack == ACCEPTED or phase == Phase.GATE_SEARCH
        self.sm.state = phase
        self.sm.h_target = 6.0
        self.sm.state_since = self.now
        if phase != Phase.GATE_SEARCH:
            self.sm._mode_expected = MODE_GUIDED
        else:
            self.sm._search_mode = MODE_LOITER   # as _start_search records it
        self.sm.was_armed = True

    def tick(self, dt=0.01, sm=True):
        self.now += dt
        self.fc.tick(self.now)
        self.v._io_step(self.now)
        self.v.pump()
        self.faza.set(self.sm.state, self.now)
        self.st.step(self.now)
        if sm:
            self.sm.now = self.now
            self.sm.update(self.now)
        self.queues.append((list(self.v._urgent.queue), list(self.v._tx.queue)))
        self.v._io_step(self.now)

    def run(self, secs, until=None):
        end = self.now + secs
        while self.now < end:
            self.tick()
            if until is not None and self.sm.state in until:
                break
        return self.sm.state

    def ev(self, name):
        return [i for n, i in self.events if n == name]


def _kind(item):
    if item[0] == 'ekf_src':
        return f"ekf{item[1]}"
    if item[0] == 'send' and item[1] == 'command_long_send':
        return f"mode{item[2][5]}"
    if item[0] == 'send':
        return item[1]
    return item[0]


def test_vehiculul_real_URGENT_purja_si_ordinea_pe_fir():
    """Pe Vehicle-ul real: supervizorul (VehicleView, SupervisorThread)
    decide, masina de stari executa; tot ce pleaca pe calea EXIT_RTL e in
    coada URGENT (SET_EKF_SOURCE_SET, DO_SET_MODE RTL, STATUSTEXT), TX-ul
    e golit cat abort-ul e aprins (un STATUSTEXT obisnuit pus in TX nu
    pleaca). Pe fir: SRC1 -> ACK -> RTL. Si din GATE_SEARCH (setul 1 deja
    activ): SRC1 se trimite oricum, ACK nou asteptat."""
    out = []
    for phase in (Phase.DESCEND, Phase.GATE_SEARCH):
        r = WireRig(phase)
        r.fc.rel_alt = 40.0
        while not r.sup.abort.is_set():
            r.tick(sm=False)                # the supervisor decides first
        r.v.send_statustext(4, 'NOVA test TX')          # sits in TX
        st = r.run(5.0, until=(Phase.ABORT,))
        assert st == Phase.ABORT, (phase, r.sm.state, r.sm.exit_reason)
        assert r.v.n_tx_purged >= 1, "TX-ul nu a fost golit cu abort aprins"
        assert not any(n == 'statustext_send' and b'TX' in a[1]
                       for n, a in r.mav.sent), "TX a trecut cu abort aprins"
        urgent = [_kind(i) for u, _ in r.queues for i in u]
        tx = [_kind(i) for _, x in r.queues for i in x]
        assert 'ekf1' in urgent and f"mode{MODE_RTL}" in urgent, urgent
        assert 'ekf1' not in tx and f"mode{MODE_RTL}" not in tx, tx
        assert 'statustext_send' in urgent
        w = [e for e in r.fc.wire if e[1] != 2]
        assert [e[:2] for e in w] == [('src', 1), ('ack', 1), ('mode', MODE_RTL)], w
        assert r.sup.abort_rtl and r.fc.mode == MODE_RTL
        assert r.ev('exit_done')[-1]['result'] == 'rtl'
        out.append(f"{phase}: SRC1 {w[1][2] - w[0][2] + 0.0:.2f}s ack, RTL "
                   f"{w[2][2] - w[0][2]:.2f}s")
    return '; '.join(out) + '; totul URGENT, TX golit'


def test_vehiculul_real_SRC1_pierdut_ack_vechi_LAND_zero_RTL():
    """(c) pe fir: ACCEPTED-ul vechi al comutarii pe SRC2 e in ekf_src_ack;
    SRC1 se pierde (FC-ul nu raspunde niciodata). Nicio comanda RTL pe fir,
    LAND la 2 s. Negativ de control: acelasi lucru cu SRC1 primit -> RTL."""
    r = WireRig(Phase.DESCEND, lose={1})
    assert r.v.ekf_src_ack == ACCEPTED
    r.fc.rel_alt = 40.0
    assert r.run(6.0, until=(Phase.ABORT,)) == Phase.ABORT
    mod = [e[1] for e in r.fc.wire if e[0] == 'mode']
    assert MODE_RTL not in mod and mod == [MODE_LAND], r.fc.wire
    assert len([e for e in r.fc.wire if e[:2] == ('src', 1)]) == 4, r.fc.wire
    r2 = WireRig(Phase.DESCEND)
    r2.fc.rel_alt = 40.0
    r2.run(6.0, until=(Phase.ABORT,))
    assert [e[1] for e in r2.fc.wire if e[0] == 'mode'] == [MODE_RTL]
    return "SRC1 pierdut + ACCEPTED vechi: LAND, 0 RTL; cu ACK: RTL"


# --- real threads wiring ---------------------------------------------------------

def test_cablajul_pe_fire_supervizorul_decide_principalul_executa():
    """Cablajul din nova_pi (Vehicle cu fir I/O pasit, supervizorul peste
    VehicleView in SupervisorThread.step, verdictul prin abort + abort_rtl,
    fara on_exit) vs cablajul sincron: aceleasi stari, aceleasi mesaje
    discrete in aceeasi ordine (SRC2 ... SRC1, RTL)."""
    a, b = app_sync(), app_fire()
    # the callback wiring also reads the supervisor (for the ceiling value
    # in the STATUSTEXT); its request then arrives twice and is ignored
    a.sm.attach_supervisor(a.sup)
    for app in (a, b):
        reach(app, Phase.DESCEND)
        breach(app)
        assert until(app) == Phase.ABORT, (type(app).__name__, app.states)
    assert a.states == b.states, (a.states, b.states)
    assert tti._mesaje(a.v) == tti._mesaje(b.v), (tti._mesaje(a.v), tti._mesaje(b.v))
    assert b.sup.on_exit is None and b.sup.abort_rtl and b.sm._last_abort_rtl
    assert b.v.src_cmds[-1] == 1 and b.v.mode_reqs[-1] == MODE_RTL
    assert_src1_ack_rtl([e for e in b.v.wire if e[2] >= b.v.wire[-1][2] - 3.0
                         and not (e[0] in ('src', 'ack') and e[1] == 2)][-3:])
    return f"stari identice ({len(a.states)}); moduri {b.v.mode_reqs[-2:]}"


# --- the old ceiling, the double fault, no fence ---------------------------------

def test_plafonul_vechi_fata_de_handover_nu_mai_declanseaza():
    """35 m deasupra punctului de handover dar 25 m deasupra home: NIMIC
    (plafonul vechi ar fi declansat). Handover la 4 m, 31 m deasupra home
    (27 m deasupra handover-ului): DA (cel vechi n-ar fi declansat)."""
    v, sup, exits = sup_stub()
    t = 100.0
    v.z = 10.0                               # handover point 10 m below origin
    for ph in ('GATE_SEARCH', 'ENGAGE', 'MOVE'):
        t = ts.ruleaza_extnav(v, sup, ph, t, n=3)
    v.z = 10.0 - 35.0                        # 35 m above the handover point
    for i in range(10):
        r = tick(v, sup, 'DESCEND', t + 0.05 * i, h=25.0)
    assert r == Action.NONE and not exits, exits
    v, sup, exits = sup_stub()
    v.z = -4.0
    t = 100.0
    for ph in ('GATE_SEARCH', 'ENGAGE', 'MOVE'):
        t = ts.ruleaza_extnav(v, sup, ph, t, n=3)
    assert abs(sup.origin_alt - 4.0) < 1e-9
    v.z = -31.0
    acts = [tick(v, sup, 'DESCEND', t + 0.05 * i, h=31.0) for i in range(3)]
    assert acts[-1] == Action.EXIT_RTL and exits[0][1], acts
    return "35 m peste handover / 25 m peste home: nimic; 31 m peste home: RTL"


def test_dubla_defectiune_plafon_si_fir_principal_mort():
    """Plafon depasit SI firul principal mort: tratat ca defectiunea firului
    - SRC1 + LOITER direct, URGENT, o data; abort normal (abort_rtl fals),
    zero RTL. Cazul de control: firul principal viu -> EXIT_RTL."""
    v, sup, exits = sup_stub()
    hb = Heartbeat('principal', clock=lambda: 0.0)
    hb.beat(99.0)
    sup.set_thread_heartbeats(principal=hb)
    t = 99.0
    while t < 100.0:                              # alive: beating
        hb.beat(t)
        tick(v, sup, 'MOVE', t)
        t += 0.05
    while t < 100.88:                             # last beat 100.0
        tick(v, sup, 'MOVE', t)
        t += 0.05
    assert sup.latched == Action.NONE
    # two samples over while the thread is alive, the third when it is dead
    for tt in (100.90, 100.95, 101.05):
        r = tick(v, sup, 'MOVE', tt, h=40.0)
    assert r == Action.EXIT and sup.latched_monitor == 'thread_principal', r
    assert not sup.abort_rtl and v.src_cmds == [1] and v.mode_reqs == [MODE_LOITER]
    assert MODE_RTL not in v.mode_reqs
    v, sup, exits = sup_stub()
    hb = Heartbeat('principal', clock=lambda: 0.0)
    sup.set_thread_heartbeats(principal=hb)
    for i in range(3):
        hb.beat(101.0 + 0.05 * i)
        r = tick(v, sup, 'MOVE', 101.0 + 0.05 * i, h=40.0)
    assert r == Action.EXIT_RTL and v.mode_reqs == [] and v.src_cmds == []
    return "fir mort + plafon: SRC1 + LOITER direct; fir viu: EXIT_RTL"


def test_fara_fence_nicaieri_pe_vehicul():
    """Decizia utilizatorului, 02.10.2026: niciun fence pe FC. Nicio sursa de
    bord (nova/, tools/nova_pi.py) nu trimite MAV_CMD_DO_FENCE_ENABLE, nu
    scrie un parametru FENCE_* si nu urca o misiune de tip FENCE; niciun
    fisier .parm nu contine FENCE_*. Negativ: tiparul prinde o linie
    injectata."""
    tipar = re.compile(r'(?<![A-Za-z0-9])FENCE_[A-Z_]+|DO_FENCE_ENABLE|'
                       r'MISSION_TYPE_FENCE')
    fisiere = [os.path.join(REPO, 'tools', 'nova_pi.py')]
    for f in sorted(os.listdir(os.path.join(REPO, 'nova'))):
        if f.endswith('.py'):
            fisiere.append(os.path.join(REPO, 'nova', f))
    for f in sorted(os.listdir(os.path.join(REPO, 'config'))):
        if f.endswith('.parm'):
            fisiere.append(os.path.join(REPO, 'config', f))
    rele = []
    for f in fisiere:
        for i, linie in enumerate(open(f), 1):
            if tipar.search(linie):
                rele.append(f"{os.path.relpath(f, REPO)}:{i}: {linie.strip()}")
    assert not rele, rele
    assert len([f for f in fisiere if f.endswith('.parm')]) == 2
    for injectat in ('FENCE_ENABLE,1', "v.set_param('FENCE_ALT_MAX', 30)",
                     'MAV_CMD_DO_FENCE_ENABLE', 'mavlink.MAV_MISSION_TYPE_FENCE'):
        assert tipar.search(injectat), injectat
    assert not tipar.search('GEOFENCE_RADIUS_M = 10.0')
    return f"{len(fisiere)} fisiere de bord, 0 FENCE"


# --- stale ack: the normal EXIT, ENGAGE (coordinator's addition to B1) ----------

def test_EXIT_normal_asteapta_ACK_real_pentru_SRC1():
    """(b) Pana acum _run_exit citea ACCEPTED-ul ramas de la SRC2 si cerea
    LOITER in acelasi pas. Acum: LOITER abia dupa ACK-ul atribuit cererii de
    SRC1 (aici 80 ms). Negativ: SRC1 pierdut -> LOITER la 2.0 s (timeout
    existent), vehiculul tot e predat."""
    app = app_sync(ack=0.08, reset=0.0)
    reach(app, Phase.DESCEND)
    assert app.v.ekf_src_ack == ACCEPTED
    i0 = len(app.v.wire)
    app.v.set_aux(1000)
    assert until(app) == Phase.ABORT
    w = app.v.wire[i0:]
    i_ack, i_loi = first(w, 'ack', 1), first(w, 'mode', MODE_LOITER)
    assert i_ack is not None and i_ack < i_loi, w
    dt = w[i_loi][2] - w[0][2]
    assert 0.08 <= dt <= 0.15, dt
    app = app_sync(ack=0.08, src_lost={1})
    reach(app, Phase.DESCEND)
    i0 = len(app.v.wire)
    app.v.set_aux(1000)
    assert until(app) == Phase.ABORT and app.v.mode == MODE_LOITER
    w = app.v.wire[i0:]
    dt2 = w[first(w, 'mode', MODE_LOITER)][2] - w[0][2]
    assert 1.99 <= dt2 <= 2.05, dt2
    return f"LOITER la {dt * 1000:.0f} ms (dupa ACK); SRC1 pierdut: {dt2:.2f} s"


def test_ENGAGE_nu_trece_pe_ack_vechi_de_SRC1():
    """(d) A doua incercare din zbor: ACCEPTED-ul SRC1 de la iesirea
    anterioara e ultimul ack. ENGAGE nu trece de pasul src2 pana la ACK-ul
    SRC2 (intarziat 1.0 s aici). Negativ: cu ACK-ul la 10 ms trece imediat."""
    def doua_incercari(ack2):
        app = app_sync(ack=0.01, reset=0.0)
        reach(app, Phase.DESCEND)
        app.v.set_aux(1000)
        assert until(app) == Phase.ABORT
        assert app.v.ekf_src_ack == ACCEPTED and app.v.ekf_src_acked(1)
        app.v.src_ack_delay_s = ack2
        app.run(0.3)
        app.v.set_aux(2000)
        until(app, states=(Phase.ENGAGE,), secs=30.0)
        t_eng = app.t
        while app.sm.state == Phase.ENGAGE and app.sm._sub == 'src2' and app.t < t_eng + 5:
            step(app)
        return app, app.t - t_eng
    app, dt = doua_incercari(1.0)
    i2 = [e for e in app.v.wire if e[:2] == ('src', 2)][-1][2]
    a2 = [e for e in app.v.wire if e[:2] == ('ack', 2)][-1][2]
    assert a2 - i2 >= 0.99 and dt >= 1.0, (a2 - i2, dt)
    _app, dt0 = doua_incercari(0.01)
    assert dt0 < 0.2, dt0
    return f"src2 tinut {dt:.2f} s pana la ACK-ul SRC2; cu ACK prompt {dt0:.2f} s"


# --- config wiring -----------------------------------------------------------------

def test_config_plafon_validare_si_cablaj():
    """ceiling_agl_m 30 / ceiling_confirm_n 3 implicit si in nova.json;
    nenumeric / <= 0 / confirm < 1 -> pornire refuzata; altceva decat 30 ->
    AVERTISMENT 'plafon de test'. Valoarea ajunge prin cablaj_extnav in
    supervizor, si linia de pornire o spune."""
    v, w = nova_config.ceiling_settings(nova_config.DEFAULTS)
    assert v == {'ceiling_agl_m': 30.0, 'ceiling_confirm_n': 3} and w == []
    cfg = nova_config.load()
    v, w = nova_config.ceiling_settings(cfg)
    assert v['ceiling_agl_m'] == 30.0 and w == [], (v, w)
    assert cfg.get('ceiling_agl_m') == 30.0 and cfg.get('ceiling_confirm_n') == 3
    v, w = nova_config.ceiling_settings({'ceiling_agl_m': 8, 'ceiling_confirm_n': 2})
    assert v == {'ceiling_agl_m': 8.0, 'ceiling_confirm_n': 2}
    assert len(w) == 1 and 'plafon de test' in w[0], w
    for rau in ({'ceiling_agl_m': 'sus'}, {'ceiling_agl_m': 0},
                {'ceiling_agl_m': -5}, {'ceiling_confirm_n': 0},
                {'ceiling_confirm_n': 1.5}, {'ceiling_confirm_n': 'trei'},
                {'ceiling_agl_m': True}):
        try:
            nova_config.ceiling_settings(rau)
        except ValueError:
            continue
        raise AssertionError(f"acceptat: {rau}")
    import nova_pi
    vv = Vehicle('udpin:127.0.0.1:1')
    vv.m = types.SimpleNamespace(mav=types.SimpleNamespace(), target_system=1,
                                 target_component=1)
    a = types.SimpleNamespace(monitor=False, monitor_motiv=None, no_ascent=False,
                              no_authority=True, conv=2, fast_descent=False)
    cfg8 = dict(cfg, ceiling_agl_m=8.0, ceiling_confirm_n=4)
    with contextlib.redirect_stdout(io.StringIO()) as out:
        _g, sup, _sm, _a = nova_pi.cablaj_extnav(
            a, cfg8, vv, OverrideMonitor(vv), {}, 8, 1500, lambda r: None,
            lambda n, i: None)
    assert sup.ceiling_agl_m == 8.0 and sup.ceiling_confirm_n == 4
    assert 'plafon 15.2.4: 8 m deasupra home' in out.getvalue(), out.getvalue()
    src = open(os.path.join(HERE, 'nova_pi.py')).read()
    m = src[src.index('def main():'):]
    assert m.index('ceiling_settings(cfg)') < m.index('ensure_port_free')
    return "30 / 3; refuzuri; 8 m -> avertisment, ajunge in supervizor"


TESTS = [
    ('depasire in fiecare faza activa: SRC1 -> ACK -> RTL',
     test_depasire_in_fiecare_faza_activa_SRC1_ack_RTL),
    ('plafonul activ exact in lista pozitiva',
     test_plafonul_activ_exact_in_lista_pozitiva),
    ('zgomot: esantioane distincte, nu iteratii',
     test_zgomot_esantioane_distincte_nu_iteratii),
    ('fara ACK pentru SRC1: LAND, zero RTL',
     test_fara_ack_pentru_SRC1_LAND_si_zero_RTL),
    ('RTL refuzat -> LAND; LAND refuzat -> raportat',
     test_RTL_refuzat_LAND_iar_LAND_refuzat_raportat),
    ('poarta de cadru GPI ~ GPS, timeout 1 s', test_poarta_de_cadru_GPI_GPS_si_timeout),
    ('inactiv in afara procedurii, fara escaladare',
     test_inactiv_in_afara_procedurii_si_fara_escaladare),
    ('clasa 5.66: nimic dupa predare, pilotul castiga',
     test_clasa_5_66_nimic_dupa_predare_pilotul_castiga),
    ('cerere pasiva ramane pasiva', test_cerere_pasiva_ramane_pasiva),
    ('Vehicle real: URGENT, purja TX, ordinea pe fir',
     test_vehiculul_real_URGENT_purja_si_ordinea_pe_fir),
    ('Vehicle real: SRC1 pierdut + ack vechi -> LAND, 0 RTL',
     test_vehiculul_real_SRC1_pierdut_ack_vechi_LAND_zero_RTL),
    ('cablajul pe fire: supervizorul decide, principalul executa',
     test_cablajul_pe_fire_supervizorul_decide_principalul_executa),
    ('plafonul vechi (fata de handover) nu mai declanseaza',
     test_plafonul_vechi_fata_de_handover_nu_mai_declanseaza),
    ('dubla defectiune: plafon + fir principal mort',
     test_dubla_defectiune_plafon_si_fir_principal_mort),
    ('fara fence nicaieri pe vehicul', test_fara_fence_nicaieri_pe_vehicul),
    ('EXIT normal asteapta ACK-ul real al SRC1',
     test_EXIT_normal_asteapta_ACK_real_pentru_SRC1),
    ('ENGAGE nu trece pe ack vechi de SRC1', test_ENGAGE_nu_trece_pe_ack_vechi_de_SRC1),
    ('config: plafon, validare, cablaj', test_config_plafon_validare_si_cablaj),
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
