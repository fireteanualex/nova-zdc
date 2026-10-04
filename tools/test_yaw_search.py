#!/usr/bin/env python3
"""
NOVA - ZDC 2026
B3 (04.10.2026): cautarea markerului prin rotire (YAW_SEARCH).

    python3 tools/test_yaw_search.py

Cifrele masurate in SITL Copter-4.5.7 (fazele A2 / A2b) pe care se sprijina:
pe setul EKF 2 fara nicio proba de viziune pozitia ramane valida 7.0 s de
la comutare (constanta de firmware), apoi failsafe-ul EKF aterizeaza; primul
VPE din timpul rotirii reseteaza EKF-ul curat pe cadrul markerului; un
consemn de pozitie dupa reset opreste rotirea in 0.3 s.

Vehiculul: SimVeh din test_extnav_landing (fizica de mucava, cu
CONDITION_YAW, cele doua cadre EKF si regula celor 7 s), pe cablajul
sincron (App) si pe cel cu fire pasite de test (test_threads_integration.
AppFire). Detectiile vin din adevar, intr-un camp de 73 x 45 grade (axa
lunga inainte, ca detectorul sintetic din SITL): markerul la 3 m in
DREAPTA la 6 m (26.6 grd > 22.6) nu se vede pana nu il aduce rotirea.
"""

import contextlib
import io
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from pymavlink import mavutil                               # noqa: E402

from nova import config as nova_config                      # noqa: E402
from nova import extnav as ex                               # noqa: E402
from nova import extnav_landing as xl                       # noqa: E402
from nova.board_window import OsdMesaje                     # noqa: E402
from nova.detection import Detection                        # noqa: E402
from nova.extnav_landing import (ExtNavConfig, Phase,       # noqa: E402
                                 EKF_NO_VISION_VALID_S, SEARCH_BUDGET_S)
from nova.safety import (CEILING_PHASES, EXTNAV_ENGAGED_PHASES,  # noqa: E402
                         EXTNAV_PHASES, ExtNavSupervisor)
from nova.vehicle import (MODE_ALT_HOLD, MODE_GUIDED,       # noqa: E402
                          MODE_LAND, MODE_LOITER, MODE_RTL, Vehicle)
import test_extnav_landing as tel                           # noqa: E402

AUX = tel.AUX
#: the synthetic camera's half field (forward = long axis, right = short)
FOV = (36.5, 22.6)
H_COVER = 15.7
#: the brief's scenario: 6 m, the marker 3 m to the RIGHT of a vehicle
#: facing north - the vehicle is 3 m WEST of the marker
P_RIGHT = (0.0, -3.0)


def cfg_search(**kw):
    base = dict(aux_channel=AUX, yaw_search=True, h_cover_m=H_COVER,
                yaw_search_max_deg=90.0, yaw_search_rate_dps=25.0)
    base.update(kw)
    return ExtNavConfig(**base)


def timed(app):
    """Events with the loop time (`_t`), on top of App.events."""
    def on_event(n, i):
        app.events.append((n, dict(i, _t=app.v.now)))
    app.sm.on_event = on_event


def app_search(h=6.0, p=P_RIGHT, yaw=0.0, rate=1.0, fov=FOV, seed=1,
               gps_offset=None, **cfg):
    app = tel.App(h=h, p=p, rate=rate, seed=seed, yaw=yaw, cfg=cfg_search(**cfg))
    app.det.fov_half_deg = fov
    app.det.latency = 0.0
    if gps_offset is not None:
        app.v.gps_offset = list(gps_offset)
    timed(app)
    return app


def to_search(app, secs=20.0):
    app.handover()
    st = app.run(secs, stop=(Phase.YAW_SEARCH, Phase.GATE_FAIL, Phase.ENGAGE,
                             Phase.REJECT, Phase.ABORT))
    assert st == Phase.YAW_SEARCH, (st, app.states)
    return app


def ev_t(app, name):
    return [i['_t'] for n, i in app.events if n == name]


def run_until(app, pred, secs):
    end = app.t + secs
    while app.t < end:
        app.run(0.02)
        if pred(app):
            return True
    return False


# --- config ------------------------------------------------------------------

def test_config_tabel_validare():
    """Bugetul 0.5 + grade/rata + 0.3 <= 5.0 s, la pornire. Tabelul din
    brief, plus limitele 180 grd / 60 grd/s si avertismentul peste 30."""
    ok = {(90, 25): (4.4, 0), (180, 45): (4.8, 1), (90, 30): (3.8, 0)}
    for (d, r), (dur, nw) in ok.items():
        v, w = nova_config.yaw_search_settings({'yaw_search_max_deg': d,
                                                'yaw_search_rate_dps': r})
        assert abs(v['duration_s'] - dur) < 1e-9 and len(w) == nw, (d, r, v, w)
    for d, r, dur in ((90, 20, '5.30'), (180, 40, '5.30')):
        try:
            nova_config.yaw_search_settings({'yaw_search_max_deg': d,
                                             'yaw_search_rate_dps': r})
        except ValueError as e:
            m = str(e)
            assert dur in m and '> 5 s' in m and '7 s' in m, m
            need = d / 4.2
            assert f"{need:.1f} grd/s" in m, m
        else:
            raise AssertionError(f"{d}/{r} acceptat")
    for kw, frag in (({'yaw_search_max_deg': 181, 'yaw_search_rate_dps': 60}, '180'),
                     ({'yaw_search_rate_dps': 61}, '60'),
                     ({'yaw_search_rate_dps': 0}, 'pozitiv'),
                     ({'yaw_search_max_deg': 'mult'}, 'numar'),
                     ({'yaw_search_max_deg': True}, 'numar'),
                     ({'search_radius_m': -1}, 'pozitiv'),
                     ({'yaw_search': 'da'}, "'auto'"),
                     ({'yaw_search': 1}, "'auto'")):
        try:
            nova_config.yaw_search_settings(kw)
        except ValueError as e:
            assert frag in str(e), (kw, e)
        else:
            raise AssertionError(f"acceptat: {kw}")
    v, w = nova_config.yaw_search_settings({'yaw_search_rate_dps': 35})
    assert len(w) == 1 and 'nu e testata' in w[0], w
    # oprita: bugetul nu se verifica (rotirea nu ruleaza niciodata)
    v, w = nova_config.yaw_search_settings({'yaw_search': False,
                                            'yaw_search_rate_dps': 20})
    assert v['yaw_search'] is False
    # implicitele si fisierul versionat
    d = nova_config.DEFAULTS
    assert (d['yaw_search'], d['yaw_search_max_deg'], d['yaw_search_rate_dps'],
            d['search_radius_m']) == ('auto', 90.0, 25.0, 6.5)
    v, w = nova_config.yaw_search_settings(nova_config.load())
    assert (v['yaw_search'], v['yaw_search_max_deg'], v['yaw_search_rate_dps']) \
        == ('auto', 90.0, 25.0) and w == [], v
    assert SEARCH_BUDGET_S < EKF_NO_VISION_VALID_S - 1.0
    # preflight, check 'secventa': the same validation, appended
    import preflight_check as pf
    r = pf.check_touchdown(nova_config.load())
    assert r.status == pf.OK and r.detail.startswith('contact ') \
        and 'rotire auto, 90 grd la 25 grd/s, 4.4/5.0 s' in r.detail, r.detail
    r = pf.check_touchdown(dict(nova_config.load(), yaw_search_rate_dps=20))
    assert r.status == pf.ESEC and '5.30 s' in r.detail, r.detail
    r = pf.check_touchdown(dict(nova_config.load(), yaw_search_rate_dps=35))
    assert r.status == pf.OK and 'nu e testata' in r.detail, r.detail
    return "90/25 4.4 s, 180/45 4.8 s ok; 90/20, 180/40 (5.3 s), >180, >60 refuzate; 35 avertisment"


def test_pornirea_refuza_90_20():
    """tools/nova_pi.py main() pe un config cu 90 grd / 20 grd/s: NU PORNESC
    inainte de port si camera, cu motivul (5.30 s > 5 s)."""
    import test_altitudes as ta
    rc, out = ta.porneste_main({'yaw_search_rate_dps': 20})
    assert rc == 2 and 'NU PORNESC' in out and '5.30 s' in out, (rc, out[-600:])
    rc, out = ta.porneste_main({'yaw_search_rate_dps': 35})
    assert rc == 3 and 'nu e testata' in out, (rc, out[-600:])
    return "90/20: rc 2 cu motivul; 35 grd/s: avertisment, pornire continua"


def test_h_cover_din_calibrarea_efectiva():
    """h_cover = raza / tan(FOV_scurt / 2), din calibrarea EFECTIVA a
    presetului (crop1280 ~ 73 x 45 grd -> ~15.7 m); min(HFOV, VFOV), deci
    rotirea camerei nu conteaza. Si cablajul real o duce in ExtNavConfig."""
    import nova_pi
    from nova.detector_pi import CameraCalibration
    fx = 865.0
    K = [[fx, 0, 640], [0, fx, 360], [0, 0, 1]]
    cal = CameraCalibration(K, [0, 0, 0, 0, 0], 1280, 720, rms=0.2, n_images=20)
    det = types.SimpleNamespace(det=types.SimpleNamespace(calib=cal))
    fov = nova_pi.fov_efectiv(det)
    assert 72.0 < fov[0] < 74.0 and 44.0 < fov[1] < 46.0, fov
    hc = xl.yaw_search_h_cover_m(6.5, *fov)
    assert 15.4 < hc < 16.0, hc
    assert xl.yaw_search_h_cover_m(6.5, fov[1], fov[0]) == hc   # rotated mount
    assert xl.yaw_search_h_cover_m(6.5, None, 45.0) is None
    assert nova_pi.fov_efectiv(types.SimpleNamespace()) is None
    # through the real wiring
    v = Vehicle('udpin:127.0.0.1:1')
    v.m = types.SimpleNamespace(mav=types.SimpleNamespace(), target_system=1,
                                target_component=1)
    a = types.SimpleNamespace(monitor=False, monitor_motiv=None, no_ascent=False,
                              no_authority=True, conv=2, fast_descent=False)
    out = io.StringIO()
    from nova.rc import OverrideMonitor
    with contextlib.redirect_stdout(out):
        _g, _s, sm, _ = nova_pi.cablaj_extnav(
            a, nova_config.load(), v, OverrideMonitor(v), {}, 8, 1500,
            lambda r: None, lambda n, i: None, fov_deg=fov)
    c = sm.cfg
    assert c.yaw_search == 'auto' and abs(c.h_cover_m - hc) < 1e-9, c
    assert (c.yaw_search_max_deg, c.yaw_search_rate_dps, c.search_radius_m) == (90, 25, 6.5)
    line = [x for x in out.getvalue().splitlines() if 'cautare prin rotire' in x]
    assert line and '90 grd la 25 grd/s' in line[0] and f"{hc:.1f} m" in line[0] \
        and '4.4/5.0 s' in line[0], line
    # fara FOV (unelte fara detector): 'auto' nu roteste
    with contextlib.redirect_stdout(io.StringIO()):
        _g, _s, sm2, _ = nova_pi.cablaj_extnav(
            a, nova_config.load(), v, OverrideMonitor(v), {}, 8, 1500,
            lambda r: None, lambda n, i: None)
    assert sm2.cfg.h_cover_m is None
    return f"FOV {fov[0]:.1f} x {fov[1]:.1f} grd -> h_cover {hc:.2f} m; linia: {line[0][7:90]}..."


# --- when it starts -----------------------------------------------------------

def test_auto_sub_si_peste_h_cover_true_false():
    """'auto' sub h_cover -> YAW_SEARCH; 'auto' peste -> GATE_FAIL cu
    "fara rotire, h >= h_cover" (OSD); true -> cauta si peste; false ->
    GATE_FAIL, zero comenzi; 'auto' fara h_cover -> GATE_FAIL."""
    out = {}
    for name, kw, want in (('auto<', dict(yaw_search='auto', h_cover_m=15.7), Phase.YAW_SEARCH),
                           ('auto>', dict(yaw_search='auto', h_cover_m=5.0), Phase.GATE_FAIL),
                           ('true>', dict(yaw_search=True, h_cover_m=5.0), Phase.YAW_SEARCH),
                           ('false', dict(yaw_search=False), Phase.GATE_FAIL),
                           ('auto?', dict(yaw_search='auto', h_cover_m=None), Phase.GATE_FAIL)):
        app = app_search(rate=0.0, **kw)
        app.handover()
        st = app.run(20.0, stop=(Phase.YAW_SEARCH, Phase.GATE_FAIL))
        assert st == want, (name, app.states)
        out[name] = round(app.t, 1)
        if st == Phase.GATE_FAIL:
            assert app.v.src_cmds == [] and app.v.mode_reqs == [] and not app.v.cond_yaws
            gf = app.ev('gate_fail')[-1]
            text = OsdMesaje.format('gate_fail', gf)[0]
            if name == 'auto>':
                assert gf['no_rotation'] == 'h_cover', gf
                assert text == "MARKER NEGASIT: fara rotire, h >= h_cover", text
            else:
                assert 'fara rotire' not in text, (name, text)
        else:
            assert 9.0 < app.t < 12.0, app.t      # after BOTH stationary windows
    return f"timpi: {out}"


def test_poarta_refuzata_nu_cauta():
    """Poarta trebuie sa fi acceptat (ok True): manse deviate / altitudine
    in afara ferestrei -> REJECT, nicio cautare; poarta neasezata (ok None)
    sau modul schimbat de pilot in ferestre -> GATE_FAIL, nicio cautare."""
    app = app_search(rate=0.0)
    rc = list(app.v._live.rc)
    rc[0] = 1900                                    # roll deviat
    app.v._live.rc = tuple(rc)
    app.handover()
    st = app.run(15.0, stop=(Phase.REJECT, Phase.YAW_SEARCH, Phase.GATE_FAIL))
    assert st == Phase.REJECT and Phase.YAW_SEARCH not in app.states, app.states
    app = app_search(h=13.0, rate=0.0)              # peste 12 m (fereastra App)
    app.handover()
    st = app.run(15.0, stop=(Phase.REJECT, Phase.YAW_SEARCH, Phase.GATE_FAIL))
    assert st == Phase.REJECT, app.states
    assert app.v.src_cmds == [] and app.v.mode_reqs == []
    # ok None / False -> nu
    app = app_search(rate=0.0)
    assert app.sm._yaw_search_wanted(None) == (False, None)
    assert app.sm._yaw_search_wanted(False) == (False, None)
    # pilotul schimba modul in timpul ferestrelor: decizia lui
    app = app_search(rate=0.0)
    app.handover()
    app.run(3.0)
    assert app.sm.state == Phase.GATE_SEARCH
    app.v.mode = MODE_ALT_HOLD
    st = app.run(15.0, stop=(Phase.YAW_SEARCH, Phase.GATE_FAIL))
    assert st == Phase.GATE_FAIL and app.ev('gate_fail')[-1]['no_rotation'] == 'mod_schimbat'
    assert app.v.src_cmds == [] and app.v.mode_reqs == []
    return "manse / altitudine -> REJECT; ok None, mod schimbat -> GATE_FAIL; zero comenzi"


# --- the sequence on the wire -------------------------------------------------

def test_ordinea_pe_fir_si_oprirea():
    """SRC2 (ack ATRIBUIT) -> GUIDED confirmat -> CONDITION_YAW cu
    param3 = -1, relativ, 90 grd la 25 grd/s; NICIUN consemn de pozitie
    inainte de resetul observat; la prima detectie UN consemn, la pozitia
    EKF in cadrul NOU si yaw-ul curent, care opreste rotirea; ENGAGE fara
    a repeta comutarea / modul -> MOVE."""
    app = to_search(app_search())
    marks = {}

    def hook(a):
        if a.v.mode == MODE_GUIDED and 'guided' not in marks:
            marks['guided'] = a.v.now
        if a.sm._ys_reset_t is not None and 'reset' not in marks:
            marks['reset'] = a.sm._ys_reset_t
            marks['true_p'] = tuple(a.v.p)
            marks['yaw'] = a.v.yaw_true
            marks['ekf'] = (a.v.x, a.v.y, a.v.z)
    st = app.run(15.0, stop=(Phase.MOVE, Phase.ABORT), on_step=hook)
    assert st == Phase.MOVE, (app.states, app.sm.exit_reason)
    w = [e for e in app.v.wire if e[0] in ('src', 'ack', 'mode', 'cyaw')]
    kinds = [(e[0], e[1]) for e in w]
    assert kinds[:4] == [('src', 2), ('ack', 2), ('mode', MODE_GUIDED),
                         ('cyaw', 90.0)], kinds
    assert kinds.count(('src', 2)) == 1 and kinds.count(('mode', MODE_GUIDED)) == 1, kinds
    cy = app.v.cond_yaws
    assert len(cy) == 1 and cy[0][1:] == (90.0, 25.0, -1.0, 1.0), cy
    assert cy[0][0] >= marks['guided'], (cy, marks)
    t_sw = app.sm._ys_t_switch
    ack_t = [e[2] for e in app.v.wire if e[0] == 'ack' and e[1] == 2][0]
    assert ack_t <= marks['guided'] and app.ev('yaw_search_ack')
    # no position target before the reset was observed; the first one is
    # the stop: EKF position (marker frame = truth here) and current yaw
    tl = app.v.target_log
    assert tl and all(t >= marks['reset'] for t, *_ in tl), (tl[:2], marks['reset'])
    t0, x, y, z, yaw = tl[0]
    assert abs(t0 - marks['reset']) < 1e-9
    assert math.hypot(x - marks['ekf'][0], y - marks['ekf'][1]) < 1e-9 and z == marks['ekf'][2]
    assert math.hypot(x - marks['true_p'][0], y - marks['true_p'][1]) < 0.3, (x, y, marks)
    assert abs(ex.wrap_pi(yaw - marks['yaw'])) < 0.05, (yaw, marks['yaw'])
    assert app.sm.n_stop_targets == 1 and app.sm.n_condition_yaw == 1
    # the rotation stopped at the target; ENGAGE went straight on
    assert app.v.cond_yaw is None
    found = app.ev('yaw_search_found')[0]
    assert 33.0 < found['deg'] < 45.0, found          # 33.7 deg = the field's edge
    eng = app.ev('engage')[-1]
    assert eng['from_search'] and eng['n'] == 1, eng
    assert app.states[-3:] == [Phase.YAW_SEARCH, Phase.ENGAGE, Phase.MOVE], app.states
    s_found = found['_t'] - t_sw
    return (f"SRC2 -> ack {1000 * (ack_t - t_sw):.0f} ms -> GUIDED -> CONDITION_YAW(-1, rel); "
            f"gasit la {found['deg']:.0f} grd, {s_found:.2f} s de la SRC2; oprire la "
            f"{marks['reset'] - found['_t']:.2f} s dupa detectie")


def test_un_singur_SRC2_pana_la_DONE():
    """Cautare reusita -> ENGAGE -> ... -> DONE: exact O comanda
    SET_EKF_SOURCE_SET(2), un singur GUIDED cerut, SRC1 la COMPLETE."""
    app = to_search(app_search(rate=0.6))
    st = app.run(240.0, stop=(Phase.DONE, Phase.ABORT))
    assert st == Phase.DONE, (app.states, app.sm.exit_reason)
    assert app.v.src_cmds.count(2) == 1 and app.v.src_cmds[0] == 2, app.v.src_cmds
    assert app.v.src_cmds[-1] == 1
    assert app.v.mode_reqs.count(MODE_GUIDED) == 1, app.v.mode_reqs
    assert app.v.mode_reqs[-1] == MODE_LOITER and MODE_RTL not in app.v.mode_reqs
    assert len(app.ev('sequence_complete')) == 1
    return (f"src {app.v.src_cmds}, moduri {app.v.mode_reqs}, "
            f"{len(app.states)} stari pana la DONE")


def test_consistenta_in_rotire():
    """Doua detectii la yaw-uri diferite, acelasi marker -> consistente
    (offset-ul e in NED cu atitudinea la captura); o detectie intr-o alta
    directie -> nu. Si prin masina de stari, cu gate_detections = 2."""
    v = tel.SimVeh(h=6.0, p=P_RIGHT, yaw=0.0)
    est = ex.ExtNavEstimator(v)

    def det_at(t, yaw, extra_deg=0.0):
        v.yaw_true = yaw
        v.step(t, 0.0)
        d = (-v.p[0], -v.p[1], v.h)
        b = ex.ned_to_body(d, 0.0, 0.0, yaw + math.radians(extra_deg))
        return Detection(t=t, angle_x=math.atan2(b[0], b[2]),
                         angle_y=math.atan2(b[1], b[2]), distance_m=7.0,
                         marker_px=100.0, range_m=6.0, fill=None,
                         marker_yaw_deg=0.0)
    e1 = est.estimate(det_at(10.0, math.radians(-40)))
    e2 = est.estimate(det_at(10.5, math.radians(-70)))
    assert est.consistent(e1, e2), (e1, e2)
    assert abs(e1.x - e2.x) < 1e-6 and abs(e1.y - e2.y) < 1e-6
    e3 = est.estimate(det_at(11.0, math.radians(-70), extra_deg=30.0))
    assert not est.consistent(e2, e3), (e2, e3)
    # the state machine, gate_detections = 2
    app = to_search(app_search(gate_detections=2, rate=1.0))
    st = app.run(15.0, stop=(Phase.MOVE, Phase.ABORT))
    assert st == Phase.MOVE, (app.states, app.sm.exit_reason)
    eng = app.ev('engage')[-1]
    assert eng['from_search'] and eng['n'] == 2, eng
    # the pair was collected after the reset (not straddling it)
    assert eng['_t'] - app.sm._ys_reset_t > 0.05
    return (f"acelasi marker la -40 / -70 grd: {math.hypot(e1.x - e2.x, e1.y - e2.y):.3f} m; "
            f"alta directie: {math.hypot(e2.x - e3.x, e2.y - e3.y):.2f} m; pereche dupa reset")


# --- deadline ----------------------------------------------------------------

def slow_fc(app):
    """GUIDED confirmed only 1.25 s after the request and the FC yawing at
    half the commanded rate: the rotation cannot finish before the 5.0 s
    deadline (it would at 25 deg/s with a fast GUIDED)."""
    app.v.mode_delay_s = 1.25
    real = app.v.m.mav.command_long_send

    def cl(*a):
        if a[2] == mavutil.mavlink.MAV_CMD_CONDITION_YAW:
            a = a[:5] + (a[5] * 0.5,) + a[6:]
        return real(*a)
    app.v.m.mav.command_long_send = cl


def test_termen_5s_de_la_SRC2_inainte_de_7s():
    """Fara detectie: EXIT la t_switch + 5.0 s (pasul buclei), SRC1 trimis
    inainte de 7.0 s, pozitia EKF valida tot timpul, FC in LOITER, linia
    OSD. Negativ (§5.11): fara termen (bugetul ridicat), simulatorul pierde
    pozitia la 7.0 s - testul chiar masoara ceva."""
    app = to_search(app_search(rate=0.0))
    slow_fc(app)
    invalid = []

    def hook(a):
        if not a.v.ekf_pos_horiz_ok():
            invalid.append(a.v.now)
    st = app.run(15.0, stop=(Phase.ABORT,), on_step=hook)
    assert st == Phase.ABORT, app.states
    t_sw = app.sm._ys_t_switch
    t_exit = ev_t(app, 'exit')[0]
    end = app.ev('yaw_search_end')[-1]
    assert end['result'] == 'negasit' and 'termen 5.0 s' in end['reason'], end
    assert SEARCH_BUDGET_S <= t_exit - t_sw < SEARCH_BUDGET_S + 0.03, t_exit - t_sw
    t_src1 = [e[2] for e in app.v.wire if e[0] == 'src' and e[1] == 1][0]
    assert t_src1 - t_sw < EKF_NO_VISION_VALID_S - 1.5, t_src1 - t_sw
    assert not invalid and not [e for e in app.sup.log if e.monitor == 'ekf_position'], \
        invalid[:3]
    assert app.v.mode == MODE_LOITER and app.v.src_cmds == [2, 1]
    text = OsdMesaje.format('yaw_search_end', end)[0]
    assert text.startswith('MARKER NEGASIT dupa rotire') and 0 < end['deg'] < 80, (text, end)
    # negative: no deadline, no rotation end -> the EKF drops the position
    old = xl.SEARCH_BUDGET_S
    try:
        xl.SEARCH_BUDGET_S = 99.0
        bad = to_search(app_search(rate=0.0, yaw_search_rate_dps=8.0))
        slow_fc(bad)
        bad.run(15.0, stop=(Phase.ABORT,))
    finally:
        xl.SEARCH_BUDGET_S = old
    lost_ev = [e for e in bad.sup.log if e.monitor == 'ekf_position']
    assert lost_ev and 'ekf_position' in (bad.sm.exit_reason or ''), bad.sm.exit_reason
    lost = [ev_t(bad, 'exit')[0] - bad.ev('yaw_search_src2')[0]['_t']]
    assert abs(lost[0] - EKF_NO_VISION_VALID_S) < 0.1, lost
    return (f"EXIT la {t_exit - t_sw:.2f} s, SRC1 la {t_src1 - t_sw:.2f} s de la SRC2, "
            f"{end['deg']:.0f} grd; negativ: pozitie pierduta la {lost[0]:.2f} s")


def test_detectie_la_4_9s_nu_iese():
    """Negativul termenului: prima detectie la ~4.9 s (dupa ce VPE-ul ei a
    plecat, ceasul EKF porneste de la el) NU iese la 5.0 s: resetul se
    observa, rotirea se opreste, ENGAGE."""
    app = app_search(p=(0.4, 0.3), fov=None, rate=1.0)
    app.det.enabled = False
    to_search(app)
    slow_fc(app)
    while app.sm._ys_t_switch is None:
        app.run(0.02)
    t_sw = app.sm._ys_t_switch
    run_until(app, lambda a: a.v.now - t_sw >= 4.82, 6.0)
    assert app.sm.state == Phase.YAW_SEARCH and app.sm._ys_sub == 'rotate', app.sm._ys_sub
    app.det.enabled = True
    st = app.run(6.0, stop=(Phase.MOVE, Phase.ABORT))
    assert st == Phase.MOVE, (app.states, app.sm.exit_reason)
    f = app.ev('yaw_search_found')[0]
    assert 4.8 < f['s'] < SEARCH_BUDGET_S, f
    t_eng = ev_t(app, 'engage')[-1]
    assert t_eng - t_sw > SEARCH_BUDGET_S and not app.ev('exit'), t_eng - t_sw
    return f"detectie la {f['s']:.2f} s, ENGAGE la {t_eng - t_sw:.2f} s de la SRC2, fara EXIT"


def test_rotire_completa_fara_detectie():
    """Markerul ascuns: rotirea se termina (>= 85 grd) -> EXIT normal (SRC1
    confirmat, apoi LOITER), OSD 'MARKER NEGASIT dupa rotire <grade>',
    STATUSTEXT cu prefixul NOVA, inainte de 5.0 s."""
    app = to_search(app_search(rate=0.0))
    st = app.run(12.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT, app.states
    end = app.ev('yaw_search_end')[-1]
    assert end['result'] == 'negasit' and 85.0 <= end['deg'] <= 92.0, end
    assert end['s'] < SEARCH_BUDGET_S and 'termen' not in end['reason'], end
    text = OsdMesaje.format('yaw_search_end', end)[0]
    assert text == f"MARKER NEGASIT dupa rotire {end['deg']:.0f} grd", text
    st_txt = [s for s in app.v.statustexts if 'MARKER NEGASIT dupa rotire' in s]
    assert st_txt and st_txt[0].startswith('NOVA') and len(st_txt[0]) <= 50, app.v.statustexts
    rot = [s for s in app.v.statustexts if 'ROTIRE CAUTARE' in s]
    assert rot == ['NOVA ROTIRE CAUTARE 90 grd la 25 grd/s'], app.v.statustexts
    assert app.v.src_cmds == [2, 1] and app.v.mode_reqs == [MODE_GUIDED, MODE_LOITER]
    assert app.v.mode == MODE_LOITER and app.v.cond_yaw is None
    assert OsdMesaje.format('yaw_search_rotate', app.ev('yaw_search_rotate')[0])[0] \
        == 'ROTIRE CAUTARE 90 grd la 25 grd/s'
    return f"EXIT la {end['s']:.2f} s de la SRC2 dupa {end['deg']:.0f} grd; '{text}'"


# --- aborts --------------------------------------------------------------------

def in_rotation(app, deg=20.0):
    run_until(app, lambda a: a.sm._ys_sub == 'rotate' and a.sm._ys_progress_deg() >= deg, 8.0)
    assert app.sm.state == Phase.YAW_SEARCH and app.sm._ys_sub == 'rotate', app.sm._ys_sub
    return len(app.v.mode_reqs), len(app.v.src_cmds)


def test_abort_in_rotire():
    """In rotire: (a) supervizorul (EKF invalid) -> EXIT: SRC1, LOITER, rotirea
    oprita; (b) AUX jos -> la fel; (c) pilotul pune ALT_HOLD -> PASIV: doar
    SRC1, zero moduri; (d) manse -> EXIT; (e) inainte de comutare: IDLE,
    zero comenzi."""
    out = []
    # (a)
    app = to_search(app_search(rate=0.0))
    nm, ns = in_rotation(app)
    app.v.ekf_valid_after_switch = False
    st = app.run(5.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT and 'ekf_position' in app.sm.exit_reason, app.sm.exit_reason
    assert app.v.src_cmds[ns:] == [1] and app.v.mode_reqs[nm:] == [MODE_LOITER]
    assert app.v.cond_yaw is None and app.v.mode == MODE_LOITER
    out.append('supervizor')
    # (b)
    app = to_search(app_search(rate=0.0))
    nm, ns = in_rotation(app)
    app.v.set_aux(1000)
    st = app.run(5.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT and 'AUX jos' in app.sm.exit_reason
    assert app.v.src_cmds[ns:] == [1] and app.v.mode_reqs[nm:] == [MODE_LOITER]
    out.append('AUX')
    # (c)
    app = to_search(app_search(rate=0.0))
    nm, ns = in_rotation(app)
    app.v.mode = MODE_ALT_HOLD
    app.v.mode_pending = []
    st = app.run(5.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT and app.ev('exit')[-1]['passive'], app.ev('exit')
    assert app.v.src_cmds[ns:] == [1] and app.v.mode_reqs[nm:] == [], app.v.mode_reqs
    assert app.v.mode == MODE_ALT_HOLD
    out.append('pilot pasiv')
    # (d) sticks
    app = to_search(app_search(rate=0.0))
    nm, ns = in_rotation(app)
    rc = list(app.v._live.rc)
    rc[1] = 1900
    app.v._live.rc = tuple(rc)
    st = app.run(5.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT and 'pilot_override' in app.sm.exit_reason, app.sm.exit_reason
    assert app.v.src_cmds[ns:] == [1]
    out.append('manse')
    # (e) before the switch: the set still being read back
    app = app_search(rate=0.0)
    app.v.request_param = lambda name: True
    saved = dict(app.v.params)
    app.v.params.pop('EK3_SRC2_POSXY')            # the read-back does not finish
    to_search(app)
    app.run(0.2)
    assert app.sm.state == Phase.YAW_SEARCH and app.sm._ys_t_switch is None
    app.v.set_aux(1000)
    app.run(0.5)
    assert app.sm.state == Phase.IDLE, app.states
    assert app.v.src_cmds == [] and app.v.mode_reqs == [] and app.sm.ekf.state == 'IDLE'
    app.v.params.update(saved)
    app.run(1.0)
    assert app.v.src_cmds == [], "a comutat dupa abort"
    out.append('inainte de SRC2: IDLE, 0 comenzi')
    return ', '.join(out)


def test_plafon_din_cautare_EXIT_RTL():
    """Plafonul (15.2.4) se aplica in YAW_SEARCH: peste el in rotire ->
    EXIT_RTL: SRC1 explicit cu ACK atribuit, poarta de cadru, RTL."""
    app = to_search(app_search(rate=0.0))
    in_rotation(app)
    i0 = len(app.v.wire)
    app.v.rel_alt_offset = 30.0                   # 36 m above home
    st = app.run(5.0, stop=(Phase.ABORT,))
    assert st == Phase.ABORT, app.states
    assert Phase.EXIT_RTL in app.states and app.states.index(Phase.EXIT_RTL) == \
        app.states.index(Phase.YAW_SEARCH) + 1, app.states
    w = app.v.wire[i0:]
    seq = [(e[0], e[1]) for e in w if e[0] in ('src', 'ack', 'mode')]
    assert seq[:3] == [('src', 1), ('ack', 1), ('mode', MODE_RTL)], seq
    assert app.v.mode == MODE_RTL and app.ev('exit_rtl')[0]['phase'] == Phase.YAW_SEARCH
    assert 'YAW_SEARCH' in CEILING_PHASES
    return f"YAW_SEARCH -> EXIT_RTL: {seq}"


# --- supervisor ----------------------------------------------------------------

class OldAnchor(ExtNavSupervisor):
    """The pre-B3 rule, for the negative: the radius origin anchored when
    the FIRST EKF phase is seen (YAW_SEARCH now), and again only after a
    phase outside them (ENGAGE) was seen; the B3 anchor disabled."""

    def update(self, now=None, detection_age_s=None, phase='IDLE', miss_streak=None):
        if phase in self.ekf_phases:
            if getattr(self, '_old', None) is None:
                self._old = now
                if self.armed and self.v.have_pos:
                    self.origin_n, self.origin_e = self.v.x, self.v.y
        else:
            self._old = None
        if phase in self.geometry_phases and self._geometry_since is None:
            self._geometry_since = now                 # no B3 anchor
        return super().update(now, detection_age_s, phase, miss_streak)


def run_sup_misses_engage(app, secs, stop):
    """App.run, except that the supervisor never SEES ENGAGE: from YAW_SEARCH
    ENGAGE lasts one main-loop iteration, and the supervisor thread reads
    the phase PasPrincipal published (Latest) at its own 200 Hz - it may
    well go YAW_SEARCH -> MOVE."""
    end = app.t + secs
    seen = app.sm.state
    while app.t < end:
        app.t += 0.02
        now = 1000.0 + app.t
        app.v.step(now, 0.02)
        if app.sm.state != Phase.ENGAGE:
            seen = app.sm.state
        app.sup.update(now, None, seen)
        for d in app.det.poll(now):
            app.sm.on_detection(d, now)
        app.sm.update(now)
        if app.states[-1] != app.sm.state:
            app.states.append(app.sm.state)
        if app.sm.state in stop:
            break
    return app.sm.state


def test_reancorare_dupa_resetul_de_37m():
    """Cu un salt de cadru de ~37 m la prima detectie din cautare, MOVE nu
    declanseaza raza: originea se reancoreaza la intrarea in prima faza
    GEOMETRICA - si cand supervizorul nu vede deloc ENGAGE-ul de un pas.
    Negativ: cu regula veche (prima faza EKF, adica YAW_SEARCH acum) raza
    declanseaza in MOVE cu zeci de metri."""
    off = (35.0, -12.0)
    jump = math.hypot(*off)
    out = []
    for masked in (False, True):
        app = to_search(app_search(rate=0.6, gps_offset=off))
        run = run_sup_misses_engage if masked else app.run
        st = run(app, 60.0, (Phase.DESCEND, Phase.ABORT)) if masked \
            else app.run(60.0, stop=(Phase.DESCEND, Phase.ABORT))
        assert st == Phase.DESCEND, (masked, app.states, app.sm.exit_reason)
        assert not [e for e in app.sup.log if e.monitor == 'geofence_radius']
        anchors = [e for e in app.sup.log if e.monitor == 'anchor']
        assert len(anchors) == 1 and anchors[0].phase == 'MOVE', [str(e) for e in anchors]
        out.append(anchors[0].detail[-30:])
    # negative
    bad = app_search(rate=0.6, gps_offset=off)
    bad.sup = OldAnchor(bad.v, verbose=False, override=bad.ov,
                        on_exit=bad.sm.request_exit)
    to_search(bad)
    st = run_sup_misses_engage(bad, 60.0, (Phase.DESCEND, Phase.ABORT))
    trips = [e for e in bad.sup.log if e.monitor == 'geofence_radius']
    assert st == Phase.ABORT and trips and trips[0].phase == 'MOVE', \
        (bad.states, [str(e) for e in bad.sup.log][-3:])
    return (f"salt {jump:.1f} m: o ancora, in MOVE, fara raza (si cu ENGAGE nevazut); "
            f"regula veche: {trips[0].detail}")


def test_listele_de_faze():
    """YAW_SEARCH: armat (EXTNAV_PHASES, plafon), legatura / EKF / fire, dar
    NU geometrie; SRC2, GUIDED, ENGAGED, VPE, EXIT_RTL_FROM."""
    sup = ExtNavSupervisor(tel.SimVeh(), verbose=False)
    assert 'YAW_SEARCH' in EXTNAV_PHASES and 'YAW_SEARCH' in CEILING_PHASES
    assert 'YAW_SEARCH' in sup.link_phases and 'YAW_SEARCH' in sup.ekf_phases
    assert 'YAW_SEARCH' not in sup.geometry_phases
    assert 'YAW_SEARCH' not in EXTNAV_ENGAGED_PHASES
    for t in (xl.SRC2_PHASES, xl.GUIDED_PHASES, xl.ENGAGED_PHASES, xl.VPE_PHASES,
              xl.EXIT_RTL_FROM):
        assert Phase.YAW_SEARCH in t, t
    assert Phase.YAW_SEARCH not in xl.TOUCHDOWN_PHASES
    return "pozitiv, ca listele albe (§5.25)"


def test_send_condition_yaw_vehicul_real():
    """Vehicle.send_condition_yaw: COMMAND_LONG(CONDITION_YAW) cu param1
    unghiul, param2 rata, param3 -1 (stanga), param4 1 (relativ), pe coada
    TX NORMALA - deci aruncata cat abort-ul e aprins (fara rotire dupa un
    abort). Directie / rata gresite -> ValueError."""
    import threading
    v = Vehicle('udpin:127.0.0.1:1', threaded=True)
    v.link_healthy = True
    sent = []
    v.m = types.SimpleNamespace(mav=types.SimpleNamespace(
        command_long_send=lambda *a: sent.append(a)), target_system=1,
        target_component=1)
    assert v.send_condition_yaw(90, 25) is True
    assert v._tx.qsize() == 1 and v._urgent.qsize() == 0
    v._drain_queues(0.0)
    a = sent[0]
    assert a[2] == mavutil.mavlink.MAV_CMD_CONDITION_YAW and a[4:8] == (90.0, 25.0, -1.0, 1.0), a
    ev = threading.Event()
    v.set_abort_event(ev)
    ev.set()
    v.send_condition_yaw(90, 25)
    v._drain_queues(1.0)
    assert len(sent) == 1 and v.n_tx_purged == 1, (sent, v.n_tx_purged)
    for bad in (dict(direction=0), dict(rate_dps=0)):
        kw = dict(angle_deg=90, rate_dps=25)
        kw.update(bad)
        try:
            v.send_condition_yaw(**kw)
        except ValueError:
            pass
        else:
            raise AssertionError(bad)
    return "COMMAND_LONG (90, 25, -1, 1) pe TX; aruncat cu abort aprins"


# --- threads -------------------------------------------------------------------

def fire_search(**cfg):
    import test_threads_integration as tti
    b = tti.AppFire(h=6.0, p=P_RIGHT, yaw=0.0, rate=1.0, cfg=cfg_search(**cfg))
    b.det.fov_half_deg = FOV
    b.det.latency = 0.0
    return b


def test_fire_cautare_gasita_pana_la_DONE():
    """Cablajul pe fire (supervizorul in SupervisorThread peste
    VehicleView, abort-ul citit de masina de stari): o cautare care gaseste
    markerul -> ENGAGE -> ... -> DONE; un SRC2, abort-ul niciodata aprins."""
    b = fire_search()
    b.handover()
    st = b.run(240.0, stop=(Phase.DONE, Phase.ABORT, Phase.GATE_FAIL))
    assert st == Phase.DONE, (b.states, b.sm.exit_reason)
    assert Phase.YAW_SEARCH in b.states and b.ev('yaw_search_found')
    assert b.v.src_cmds.count(2) == 1 and b.v.src_cmds[-1] == 1, b.v.src_cmds
    assert b.sup.abort_n == 0 and b.v.n_tx_purged == 0
    assert b.sm.n_condition_yaw == 1 and b.sm.n_stop_targets == 1
    anchors = [e for e in b.sup.log if e.monitor == 'anchor']
    assert len(anchors) == 1 and anchors[0].phase == 'MOVE'
    return f"{len(b.states)} stari, gasit la {b.ev('yaw_search_found')[0]['deg']:.0f} grd"


def test_garda_blocajului_in_rotire():
    """G2 in YAW_SEARCH: bucla principala blocata > 1 s in rotire -> EXIT
    PASIV: doar SRC1 (URGENT), zero comenzi de mod pana la o incercare
    noua - si FC-ul ramane in GUIDED, al pilotului de luat."""
    import test_handback_guard as thg
    b = fire_search()
    b.det.enabled = False
    b.handover()
    st = b.run(20.0, stop=(Phase.YAW_SEARCH, Phase.GATE_FAIL))
    assert st == Phase.YAW_SEARCH, b.states
    for _ in range(400):
        thg.tick(b)
        if b.sm._ys_sub == 'rotate' and b.sm._ys_progress_deg() > 15:
            break
    assert b.sm._ys_sub == 'rotate'
    nm, ns = len(b.v.mode_reqs), len(b.v.src_cmds)
    thg.jump(b, 1.5)
    for _ in range(150):
        thg.tick(b, sup=False)
    assert b.sm.state == Phase.ABORT and b.sm._stalled, (b.states, b.sm.exit_reason)
    assert b.v.mode_reqs[nm:] == [] and b.v.src_cmds[ns:] == [1], (b.v.mode_reqs[nm:],
                                                                   b.v.src_cmds[ns:])
    assert b.ev('exit')[-1]['passive']
    # negative: the same exit without the stall sends LOITER
    b2 = fire_search()
    b2.det.enabled = False
    b2.handover()
    b2.run(20.0, stop=(Phase.YAW_SEARCH,))
    for _ in range(400):
        thg.tick(b2)
        if b2.sm._ys_sub == 'rotate' and b2.sm._ys_progress_deg() > 15:
            break
    nm = len(b2.v.mode_reqs)
    b2.v.set_aux(1000)
    for _ in range(100):
        thg.tick(b2, sup=False)
    assert MODE_LOITER in b2.v.mode_reqs[nm:], b2.v.mode_reqs[nm:]
    return "blocaj 1.5 s in rotire: SRC1, 0 moduri; negativ fara blocaj: LOITER"


def test_osd_si_statustext():
    """Liniile OSD ale cautarii si lungimea STATUSTEXT-urilor (<= 50, NOVA)."""
    f = OsdMesaje.format
    assert f('yaw_search_rotate', {'deg': 90, 'rate': 25})[0] == 'ROTIRE CAUTARE 90 grd la 25 grd/s'
    assert f('yaw_search_found', {'deg': 34.4})[0] == 'MARKER GASIT la 34 grd'
    assert f('yaw_search_end', {'result': 'negasit', 'deg': 88})[0] == \
        'MARKER NEGASIT dupa rotire 88 grd'
    assert f('yaw_search_end', {'result': 'gasit', 'deg': 30})[0] is None
    assert 'dupa rotire 34 grd' in f('engage', {'n': 1, 'alt': 6.0, 'from_search': True,
                                                'deg': 34.0})[0]
    app = to_search(app_search(rate=1.0))
    app.run(15.0, stop=(Phase.MOVE, Phase.ABORT))
    texts = [s for s in app.v.statustexts if 'ROTIRE' in s or 'MARKER' in s]
    assert texts == ['NOVA ROTIRE CAUTARE 90 grd la 25 grd/s',
                     f"NOVA MARKER GASIT la {app.ev('yaw_search_found')[0]['deg']:.0f} grd"], texts
    assert all(len(s) <= 50 for s in texts)
    return f"{texts}"


# --- the §5.66-class fix: no GUIDED over a mode the pilot / a failsafe set ----

def _to_engage(app, secs=20.0):
    app.handover()
    st = app.run(secs, stop=(Phase.ENGAGE, Phase.ABORT, Phase.GATE_FAIL))
    assert st == Phase.ENGAGE, app.states
    return app


def _end(app, secs=6.0):
    return app.run(secs, stop=(Phase.ABORT, Phase.IDLE, Phase.MOVE))


def test_mod_schimbat_inainte_de_GUIDED_in_ENGAGE():
    """ENGAGE inainte de GUIDED (pasii src2 / ekf): pilotul pune ALT_HOLD
    (manse neutre, deci fara override) -> EXIT PASIV, ZERO GUIDED pe fir;
    SRC1 inapoi daca SRC2 plecase. Negativ: FC-ul ramas in LOITER-ul
    pilotului -> GUIDED cerut si secventa merge mai departe."""
    out = []
    # 'src2': the set still being read back -> nothing sent at all
    for change in (True, False):
        app = tel.App(rate=0.3)
        saved = app.v.params.pop('EK3_SRC2_POSXY')
        _to_engage(app)
        app.run(0.2)
        assert app.sm.state == Phase.ENGAGE and app.sm._sub == 'src2'
        if change:
            app.v.mode = MODE_ALT_HOLD
            app.v.mode_pending = []
            # the read-back would now finish: without the fix, SRC2 and
            # then GUIDED over the pilot's ALT_HOLD
            app.v.params['EK3_SRC2_POSXY'] = saved
            st = _end(app)
            assert st == Phase.ABORT and app.ev('exit')[-1]['passive'], app.states
            assert 'inainte de GUIDED' in app.sm.exit_reason, app.sm.exit_reason
            assert MODE_GUIDED not in app.v.mode_reqs and app.v.src_cmds == [], \
                (app.v.mode_reqs, app.v.src_cmds)
            assert app.v.mode == MODE_ALT_HOLD
            out.append('src2: 0 comenzi')
        else:
            app.v.params['EK3_SRC2_POSXY'] = saved
            st = _end(app)
            assert st == Phase.MOVE and app.v.mode_reqs == [MODE_GUIDED], app.states
            out.append('negativ: GUIDED, MOVE')
    # 'ekf': SRC2 already sent -> SRC1 back, still no GUIDED
    app = tel.App(rate=0.3)
    mark = {}

    def hook(a):
        if a.sm.state == Phase.ENGAGE and a.sm._sub == 'ekf' and 'n' not in mark:
            mark['n'] = len(a.v.mode_reqs)
            a.v.mode = MODE_ALT_HOLD
            a.v.mode_pending = []
    app.handover()
    st = app.run(30.0, stop=(Phase.ABORT, Phase.MOVE), on_step=hook)
    assert 'n' in mark and st == Phase.ABORT, (mark, app.states)
    assert MODE_GUIDED not in app.v.mode_reqs and app.v.src_cmds == [2, 1], \
        (app.v.mode_reqs, app.v.src_cmds)
    assert app.ev('exit')[-1]['passive'] and app.v.mode == MODE_ALT_HOLD
    out.append('ekf: SRC1, 0 GUIDED')
    return '; '.join(out)


def test_failsafe_LAND_in_cautare_inainte_de_GUIDED():
    """YAW_SEARCH dupa comutare, in asteptarea ACK-ului: failsafe-ul pune
    LAND -> EXIT PASIV, SRC1 inapoi, zero GUIDED, zero CONDITION_YAW.
    Negativ: acelasi ACK intarziat, FC-ul in LOITER -> GUIDED si rotirea."""
    out = []
    for change in (True, False):
        app = app_search(rate=0.0)
        app.v.src_ack_delay_s = 0.6
        to_search(app)
        run_until(app, lambda a: a.sm._ys_t_switch is not None, 2.0)
        assert app.sm._ys_sub == 'src2' and app.v.src_cmds == [2], app.v.src_cmds
        if change:
            app.v.mode = MODE_LAND
            app.v.mode_pending = []
            st = app.run(5.0, stop=(Phase.ABORT,))
            assert st == Phase.ABORT and app.ev('exit')[-1]['passive'], app.states
            assert MODE_GUIDED not in app.v.mode_reqs and not app.v.cond_yaws
            assert app.v.src_cmds == [2, 1] and app.v.mode == MODE_LAND
            out.append('LAND: SRC1, 0 GUIDED')
        else:
            run_until(app, lambda a: a.sm._ys_sub == 'rotate', 3.0)
            assert app.v.mode_reqs == [MODE_GUIDED] and len(app.v.cond_yaws) == 1
            out.append('negativ: GUIDED + rotire')
    return '; '.join(out)


def test_mod_schimbat_intre_doua_reincercari_GUIDED():
    """FC-ul nu adopta GUIDED (pierdut pe serial); intre doua reincercari
    pilotul pune ALT_HOLD -> nicio reincercare in plus, EXIT pasiv.
    Negativ: FC-ul ramane in LOITER, a treia cerere e adoptata -> MOVE.
    Si: niciun mod al pilotului memorat (_search_mode None) = nimic al nostru."""
    app = tel.App(rate=0.3)
    app.v.refuse_modes = (MODE_GUIDED,)
    mark = {}

    def hook(a):
        if (a.sm.state == Phase.ENGAGE and a.sm._sub == 'mode'
                and a.v.mode_reqs.count(MODE_GUIDED) == 2 and 'n' not in mark):
            mark['n'] = 2
            a.v.mode = MODE_ALT_HOLD
    app.handover()
    st = app.run(30.0, stop=(Phase.ABORT, Phase.MOVE), on_step=hook)
    assert 'n' in mark and st == Phase.ABORT, (mark, app.states)
    assert app.v.mode_reqs.count(MODE_GUIDED) == 2, app.v.mode_reqs
    assert app.ev('exit')[-1]['passive'] and MODE_LOITER not in app.v.mode_reqs
    # negative
    app = tel.App(rate=0.3)
    app.v.refuse_modes = (MODE_GUIDED,)

    def hook2(a):
        if a.v.mode_reqs.count(MODE_GUIDED) == 2:
            a.v.refuse_modes = ()
    app.handover()
    st = app.run(30.0, stop=(Phase.ABORT, Phase.MOVE), on_step=hook2)
    assert st == Phase.MOVE and app.v.mode_reqs.count(MODE_GUIDED) == 3, app.v.mode_reqs
    # no pilot mode recorded: not owned
    app = app_search(rate=0.0)
    app.sm._search_mode = None
    assert app.sm._pilot_mode_lost() and app.sm._yaw_search_wanted(True) == (False, 'mod_schimbat')
    return "pilot intre reincercari: 2 GUIDED, pasiv; negativ: 3 GUIDED, MOVE"


TESTS = [
    ('config: tabelul de validare', test_config_tabel_validare),
    ('pornirea refuza 90/20 cu motivul', test_pornirea_refuza_90_20),
    ('h_cover din calibrarea efectiva, prin cablaj', test_h_cover_din_calibrarea_efectiva),
    ("'auto' sub / peste h_cover, true, false", test_auto_sub_si_peste_h_cover_true_false),
    ('poarta refuzata -> fara cautare', test_poarta_refuzata_nu_cauta),
    ('ordinea pe fir: SRC2 -> GUIDED -> CONDITION_YAW, oprirea', test_ordinea_pe_fir_si_oprirea),
    ('un singur SRC2 pana la DONE', test_un_singur_SRC2_pana_la_DONE),
    ('consistenta in rotire', test_consistenta_in_rotire),
    ('termen 5 s de la SRC2, inainte de 7 s (+negativ)', test_termen_5s_de_la_SRC2_inainte_de_7s),
    ('detectie la 4.9 s nu iese (negativ)', test_detectie_la_4_9s_nu_iese),
    ('rotire completa fara detectie -> EXIT + OSD', test_rotire_completa_fara_detectie),
    ('abort in rotire: supervizor, AUX, pilot, manse, pre-SRC2', test_abort_in_rotire),
    ('plafon din cautare -> EXIT_RTL', test_plafon_din_cautare_EXIT_RTL),
    ('reancorare dupa resetul de 37 m (+negativ)', test_reancorare_dupa_resetul_de_37m),
    ('listele de faze', test_listele_de_faze),
    ('send_condition_yaw pe vehiculul real', test_send_condition_yaw_vehicul_real),
    ('fire: cautare gasita pana la DONE', test_fire_cautare_gasita_pana_la_DONE),
    ('G2: blocaj in rotire -> pasiv (+negativ)', test_garda_blocajului_in_rotire),
    ('OSD si STATUSTEXT', test_osd_si_statustext),
    ('mod schimbat inainte de GUIDED in ENGAGE (+negativ)',
     test_mod_schimbat_inainte_de_GUIDED_in_ENGAGE),
    ('failsafe LAND in cautare inainte de GUIDED (+negativ)',
     test_failsafe_LAND_in_cautare_inainte_de_GUIDED),
    ('mod schimbat intre reincercari GUIDED (+negativ)',
     test_mod_schimbat_intre_doua_reincercari_GUIDED),
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
