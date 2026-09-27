#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru supervizorul in firul lui (faza 3, refactor/threads).

    python3 tools/test_supervisor_thread.py

Fara FC: Vehicle real, configurat cu fir I/O dar NEPORNIT, pasit de mana
(_io_step) - asa se vede exact ce intra in coada URGENT si ce in TX.
Supervizorul e construit peste vehicle.view(); monitoarele sunt cele de
dinainte, verificate aici pe drumul nou: instantaneu -> monitor -> URGENT +
abort. Plus firele moarte si supervizorul mort (watchdog).
"""

import os
import sys
import threading
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import concurrency as cc                          # noqa: E402
from nova import vehicle as vehicle_mod                     # noqa: E402
from nova.concurrency import Heartbeat, Latest              # noqa: E402
from nova.detection import Detection                        # noqa: E402
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.safety import (Action, ExtNavSupervisor,          # noqa: E402
                         SafetySupervisor)
from nova.supervisor_thread import SupervisorThread         # noqa: E402
from nova.vehicle import (MODE_BRAKE, MODE_LAND, MODE_LOITER,  # noqa: E402
                          MODE_RTL, Vehicle, VehicleState)


class FakeMav:
    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        if not name.endswith('_send'):
            raise AttributeError(name)

        def f(*a, **k):
            self.sent.append((name, a))
        return f


class NoLink:
    def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
        return None

    def close(self):
        pass


def vehicul():
    v = Vehicle('/dev/fake0', baud=921600, threaded=True)
    v.m = types.SimpleNamespace(mav=FakeMav(), target_system=1,
                                target_component=1,
                                recv_match=NoLink().recv_match, close=lambda: None)
    v.link_verbose = False
    v._note_heartbeat(100.0)
    return v


def snapshot(**kw):
    st = VehicleState()
    st.have_pos = True
    st.mode = MODE_LAND
    st.z = -6.0
    st.rc = (1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000)
    st.rc_t = 100.0
    st.time_boot_ms = 12345
    st.ekf_flags = 1 | 8
    st.ekf_t = 100.0
    for k, val in kw.items():
        setattr(st, k, val)
    return st


def build(extnav=False, phase='DESCEND_TRACK', clock=lambda: 100.0):
    v = vehicul()
    v.state.set(snapshot(), 100.0)
    view = v.view()
    ov = OverrideMonitor(view)
    if extnav:
        exits = []
        sup = ExtNavSupervisor(view, verbose=False, override=ov,
                               on_exit=exits.append)
    else:
        exits = None
        sup = SafetySupervisor(view, verbose=False, override=ov)
    det = Latest()
    phase_l = Latest(phase)
    st = SupervisorThread(sup, v, det, phase_l, clock=clock)
    return v, sup, st, det, phase_l, exits


def moduri(v, coada):
    out = []
    q = v._urgent if coada == 'urgent' else v._tx
    for item in list(q.queue):
        if item[0] == 'send' and item[1] == 'command_long_send':
            out.append(item[2][5])
        elif item[0] == 'ekf_src':
            out.append(f"ekf{item[1]}")
    return out


def proaspata(det, t):
    det.set(Detection(t=t, angle_x=0, angle_y=0, distance_m=6, marker_px=100,
                      range_m=6, fill=None), t)


# --- teste -----------------------------------------------------------------

def test_supervizorul_citeste_din_instantaneu_si_scrie_in_URGENT():
    """PLND: detectie veche -> BRAKE, dar in coada URGENT, nu in TX; abort
    aprins. Instantaneul e cel din Latest, nu fatada firului principal."""
    v, sup, st, det, phase_l, _ = build()
    proaspata(det, 100.0)
    assert st.step(100.2) == Action.NONE and not sup.abort.is_set()
    assert sup.armed and sup.v.x == 0.0 and sup.v.mode == MODE_LAND
    # firul I/O publica un instantaneu nou; fatada (v.mode) ramane veche
    v.state.set(snapshot(mode=MODE_LAND, x=1.5), 100.3)
    assert v.x == 0.0, "fatada firului principal s-a schimbat fara pump()"
    act = st.step(100.9)                        # detectie de 0.9 s
    assert act == Action.BRAKE, act
    assert sup.v.x == 1.5, "vederea nu s-a improspatat din Latest"
    assert moduri(v, 'urgent') == [MODE_BRAKE] and moduri(v, 'tx') == []
    assert sup.abort.is_set() and 'detection_age' in sup.abort_reason
    assert not sup.abort_passive
    return "BRAKE in URGENT, TX gol, abort aprins; vederea din Latest"


def test_fiecare_monitor_pe_calea_noua():
    """Manse -> LOITER, raza -> RTL, plafon -> RTL, inclinare -> BRAKE,
    legatura -> BRAKE: toate prin URGENT, cu aceleasi praguri."""
    rezultate = []
    # manse (deadband 100 PWM, 100 ms)
    v, sup, st, det, phase_l, _ = build()
    proaspata(det, 100.0)
    st.step(100.05)
    sup.override.capture_neutral(100.05)
    v.state.set(snapshot(rc=(1500, 1900, 1100, 1500, 1000, 1000, 1000, 1000), rc_t=100.1), 100.1)
    proaspata(det, 100.1)
    st.step(100.1)
    v.state.set(snapshot(rc=(1500, 1900, 1100, 1500, 1000, 1000, 1000, 1000), rc_t=100.25), 100.25)
    proaspata(det, 100.25)
    assert st.step(100.25) == Action.OVERRIDE
    assert moduri(v, 'urgent') == [MODE_LOITER] and sup.passive
    rezultate.append('manse->LOITER')
    # raza si plafon
    for camp, val, act in (('x', 12.0, Action.RTL), ('z', -40.0, Action.RTL)):
        v, sup, st, det, phase_l, _ = build()
        proaspata(det, 100.0)
        st.step(100.0)
        v.state.set(snapshot(**{camp: val}), 100.1)
        proaspata(det, 100.1)
        assert st.step(100.1) == act, camp
        assert moduri(v, 'urgent') == [MODE_RTL]
        rezultate.append(f"{camp}->RTL")
    # inclinare 35 grade, 0.3 s
    v, sup, st, det, phase_l, _ = build()
    proaspata(det, 100.0)
    st.step(100.0)
    for t in (100.1, 100.2, 100.3, 100.45):
        v.state.set(snapshot(roll=35 * 3.14159 / 180), t)
        proaspata(det, t)
        act = st.step(t)
    assert act == Action.BRAKE and sup.latched_monitor == 'tilt'
    rezultate.append('inclinare->BRAKE')
    # legatura: hb_t vechi pe vehicul
    v, sup, st, det, phase_l, _ = build()
    proaspata(det, 100.0)
    st.step(100.0)
    v.hb_t = 98.0
    proaspata(det, 100.1)
    assert st.step(100.1) == Action.BRAKE and sup.latched_monitor == 'link_age'
    rezultate.append('legatura->BRAKE')
    return ', '.join(rezultate)


def test_ExtNav_EXIT_prin_abort_si_callback():
    """ExtNav: EKF invalid in MOVE -> EXIT: abort aprins cu motivul,
    callback-ul (inca) chemat, NICIO comanda de mod de la supervizor."""
    v, sup, st, det, phase_l, exits = build(extnav=True, phase='MOVE')
    proaspata(det, 100.0)
    assert st.step(100.0) == Action.NONE and sup.armed
    v.state.set(snapshot(ekf_flags=1), 100.1)   # fara POS_HORIZ_REL
    assert st.step(100.1) == Action.EXIT
    assert sup.abort.is_set() and sup.abort_reason.startswith('ekf_position')
    assert exits and exits[0].startswith('ekf_position')
    assert moduri(v, 'urgent') == [] and moduri(v, 'tx') == []
    return "EXIT: abort + callback, 0 comenzi de mod"


def test_un_fir_mort():
    """Heartbeat stagnat = sursa pierduta. PLND: detectie moarta -> BRAKE
    (in fazele cu detectie), I/O mort -> BRAKE, principal mort -> BRAKE.
    ExtNav: orice fir mort dupa ENGAGE -> EXIT; principal mort -> si
    EXIT-ul direct (SRC1 + LOITER in URGENT), pentru ca masina de stari nu
    mai poate. Un fir care nu a batut niciodata nu e mort."""
    out = []
    for nume in ('detectie', 'mav_io', 'principal'):
        v, sup, st, det, phase_l, _ = build()
        hb = Heartbeat(nume, clock=lambda: 100.0)
        sup.set_thread_heartbeats(**{nume: hb})
        proaspata(det, 100.0)
        assert st.step(100.0) == Action.NONE, "fir nepornit tratat ca mort"
        hb.beat(100.0)
        proaspata(det, 100.5)
        v.hb_t = 100.5                              # legatura vie
        assert st.step(100.5) == Action.NONE
        proaspata(det, 101.2)
        v.hb_t = 101.2
        assert st.step(101.2) == Action.BRAKE, nume
        assert sup.latched_monitor == f"thread_{nume}", sup.latched_monitor
        assert moduri(v, 'urgent') == [MODE_BRAKE]
        out.append(f"PLND {nume}")
    # PLND: detectia moarta NU conteaza in FINAL_DESCENT (ca detectia pierduta)
    v, sup, st, det, phase_l, _ = build(phase='FINAL_DESCENT')
    hb = Heartbeat('detectie', clock=lambda: 100.0)
    sup.set_thread_heartbeats(detectie=hb)
    hb.beat(90.0)
    assert st.step(100.0) == Action.NONE
    # ExtNav
    for nume in ('detectie', 'mav_io', 'principal'):
        v, sup, st, det, phase_l, exits = build(extnav=True, phase='MOVE')
        hb = Heartbeat(nume, clock=lambda: 100.0)
        sup.set_thread_heartbeats(**{nume: hb})
        hb.beat(100.0)
        v.hb_t = 100.5
        assert st.step(100.5) == Action.NONE
        v.hb_t = 101.2
        assert st.step(101.2) == Action.EXIT, nume
        assert exits[0].startswith(f"thread_{nume}")
        if nume == 'principal':
            assert moduri(v, 'urgent') == ['ekf1', MODE_LOITER], moduri(v, 'urgent')
            assert any(e.monitor == 'exit_direct' for e in sup.log)
        else:
            assert moduri(v, 'urgent') == [], nume
        out.append(f"ExtNav {nume}")
    # in GATE_SEARCH firele nu sunt supravegheate pe ExtNav
    v, sup, st, det, phase_l, exits = build(extnav=True, phase='GATE_SEARCH')
    hb = Heartbeat('mav_io', clock=lambda: 100.0)
    sup.set_thread_heartbeats(mav_io=hb)
    hb.beat(90.0)
    assert st.step(100.0) == Action.NONE
    return ', '.join(out) + '; principal mort pe ExtNav -> SRC1 + LOITER direct'


def test_supervizorul_e_proprietarul_monitorului_de_override():
    """Faza 5 (2b): fereastra de asezare a portii o conduce supervizorul,
    din faza (observe): la intrarea in faza portii se deschide (settle_n
    creste, trim-urile se cer), in fereastra se esantioneaza, dupa ea
    neutrul se memoreaza o data - nu la arm, cu mansele in miscare. PLND:
    HANDOVER_CHECK; ExtNav: GATE_SEARCH (unde arm() vine chiar la intrare)."""
    out = []
    for extnav, faza_poarta, faza_segment in ((False, 'HANDOVER_CHECK', 'ACQUIRE'),
                                              (True, 'GATE_SEARCH', 'ENGAGE')):
        v, sup, st, det, phase_l, _ = build(extnav=extnav, phase='IDLE')
        ov = sup.override
        st.step(100.0)
        assert ov.settle_n == 0 and ov.neutral is None and not ov.in_gate
        phase_l.set(faza_poarta)
        st.step(100.1)
        assert ov.settle_n == 1 and ov.in_gate and ov.neutral is None, (
            extnav, ov.settle_n, ov.neutral)
        assert abs(ov.settle_until - (100.1 + ov.settle_s)) < 1e-9
        assert any(i[0] == 'send' and 'param' in i[1] for i in list(v._tx.queue)), (
            "trim-urile nu au fost cerute la deschiderea ferestrei")
        st.step(100.5)
        assert ov.settle_span(0) is not None and ov.neutral is None
        st.step(101.3)                            # fereastra trecuta: neutrul
        assert ov.neutral == (1500, 1500, 1100, 1500), (extnav, ov.neutral)
        phase_l.set(faza_segment)
        st.step(101.4)
        assert sup.armed and ov.neutral == (1500, 1500, 1100, 1500) and not ov.in_gate
        out.append(f"{faza_poarta}: fereastra {ov.settle_n}, neutru dupa {ov.settle_s:.0f} s")
    return '; '.join(out)


def test_supervizorul_mort_watchdog():
    """Firul principal verifica heartbeat-ul supervizorului. Stagnat peste
    0.5 s: abort aprins si actiunea de siguranta in URGENT - BRAKE (PLND)
    sau SRC1 + LOITER (ExtNav). O singura data."""
    for extnav, astept in ((False, [MODE_BRAKE]), (True, ['ekf1', MODE_LOITER])):
        v, sup, st, det, phase_l, _ = build(extnav=extnav, phase='MOVE' if extnav else 'DESCEND_TRACK')
        assert st.watchdog(100.0) is False, "fara fir pornit nu e nimic de pazit"
        st.thread = object()                     # "pornit"
        st.heartbeat.beat(100.0)
        assert st.watchdog(100.4) is False and not sup.abort.is_set()
        assert st.watchdog(100.6) is True
        assert sup.abort.is_set() and 'watchdog' in sup.abort_reason
        assert moduri(v, 'urgent') == astept, moduri(v, 'urgent')
        assert st.watchdog(101.0) is False, "a intervenit de doua ori"
        assert moduri(v, 'urgent') == astept
        assert any(e.monitor == 'watchdog' for e in sup.log)
        st.thread = None
    return "PLND: BRAKE; ExtNav: SRC1 + LOITER; o singura data"


def test_firul_chiar_ruleaza_si_se_opreste():
    v, sup, st, det, phase_l, _ = build(clock=time.monotonic)
    sup.heartbeat_max_s = 10.0
    v.hb_t = time.monotonic()
    v.state.set(snapshot(), time.monotonic())
    proaspata(det, time.monotonic())
    st.start()
    time.sleep(0.15)
    n = st.heartbeat.count
    assert n >= 10, n
    assert st.n_steps >= 10 and st.watchdog() is False
    ramase = st.stop()
    assert ramase == [] and st.thread is None
    n2 = st.heartbeat.count
    time.sleep(0.05)
    assert st.heartbeat.count == n2, "a mai batut dupa stop"
    return f"{n} pasi in 0.15 s la 200 Hz; oprit curat"


def test_override_monitor_sub_lock():
    """Poarta (firul principal) si supervizorul (firul lui) impart acelasi
    OverrideMonitor: begin_settle/capture_neutral contra update, sub stres,
    fara exceptii si fara stare inconsistenta."""
    v = vehicul()
    v.state.set(snapshot(), 100.0)
    view = v.view()
    ov = OverrideMonitor(view)
    stop = threading.Event()
    erori = []

    def poarta():
        t = 100.0
        while not stop.is_set():
            try:
                ov.begin_settle(t)
                ov.sample_settle(t)
                ov.capture_neutral(t)
                ov.deviation()
            except Exception as e:                          # noqa: BLE001
                erori.append(e)
            t += 0.001

    def supervizor():
        t = 100.0
        while not stop.is_set():
            try:
                ov.update(t)
                ov.status()
            except Exception as e:                          # noqa: BLE001
                erori.append(e)
            t += 0.001

    fire = [threading.Thread(target=poarta), threading.Thread(target=supervizor)]
    for f in fire:
        f.start()
    time.sleep(0.3)
    stop.set()
    for f in fire:
        f.join(2.0)
    assert not erori, erori[:2]
    return "0 exceptii in 0.3 s de begin_settle/capture_neutral contra update"


TESTS = [
    ('supervizorul e proprietarul monitorului de override',
     test_supervizorul_e_proprietarul_monitorului_de_override),
    ('supervizorul citeste din instantaneu si scrie in URGENT',
     test_supervizorul_citeste_din_instantaneu_si_scrie_in_URGENT),
    ('fiecare monitor pe calea noua', test_fiecare_monitor_pe_calea_noua),
    ('ExtNav: EXIT prin abort si callback', test_ExtNav_EXIT_prin_abort_si_callback),
    ('un fir mort', test_un_fir_mort),
    ('supervizorul mort: watchdog', test_supervizorul_mort_watchdog),
    ('firul chiar ruleaza si se opreste', test_firul_chiar_ruleaza_si_se_opreste),
    ('OverrideMonitor sub lock', test_override_monitor_sub_lock),
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
