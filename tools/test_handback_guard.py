#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Garda predarii (B1.9, 02.10.2026): nicio comanda de mod peste pilot sau
peste un failsafe, si niciuna dupa un blocaj.

    python3 tools/test_handback_guard.py

Masurat in SITL (campania S, randurile F6c, F2, F2c): bucla principala
blocata 20 s in MOVE, pilotul ia ALT_HOLD la +8 s, la trezire companionul
consuma abort-ul supervizorului ca EXIT NE-pasiv si trimite SRC1 + LOITER
peste pilot. Dupa SIGSTOP 15 s + SIGCONT, de doua ori SRC1 + LOITER peste
LAND-ul failsafe-ului / ALT_HOLD-ul pilotului.

  G1 PROPRIETATE: o comanda de mod pleaca doar daca FC-ul e inca in modul
     pe care il tine companionul; altfel PASIV (SRC1 cerut, URGENT, zero
     moduri) - si pe calea abort-ului, si in _exit_direct, si in watchdog.
  G2 BLOCAJ: fiecare bucla isi masoara pauza fata de iteratia ei
     anterioara; peste THREAD_HEARTBEAT_MAX_S (1.0 s) imaginea ei e veche:
     zero comenzi de mod pana la o incercare noua prin poarta.
  G3 calea rapida ramane: principalul blocat + supervizorul viu -> SRC1 +
     LOITER la ~1 s daca FC-ul e in GUIDED; supervizorul blocat + principalul
     viu -> watchdog la ~0.5 s, aceeasi conditie.

Vehiculul: SimVeh (test_extnav_landing) pe cablajul cu fire pasite de test
(test_threads_integration.AppFire): Vehicle real cu instantanee, supervizorul
peste VehicleView in SupervisorThread.step, abort-ul citit de masina de stari.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from nova.extnav_landing import Phase, STALL_MAX_S          # noqa: E402
from nova.safety import (Action, HANDBACK_VIEW_MAX_AGE_S,   # noqa: E402
                         THREAD_HEARTBEAT_MAX_S)
from nova.vehicle import (MODE_ALT_HOLD, MODE_GUIDED,       # noqa: E402
                          MODE_LAND, MODE_LOITER, MODE_RTL,
                          MODE_STABILIZE)
import test_threads_integration as tti                      # noqa: E402

DT = 0.02


def fire(**veh):
    """The threaded wiring, with the main thread's heartbeat watched by the
    supervisor (as in nova_pi) and the supervisor's beaten by its step."""
    b = tti.AppFire(rate=0.2)
    b.sup.set_thread_heartbeats(principal=b.hb)
    for k, val in veh.items():
        setattr(b.v, k, val)
    return b


def fc_mode(b, mode):
    """The FC changes mode by itself (the pilot's switch, a failsafe):
    written FC-side (`_live`), seen by the companion through the snapshot."""
    b.v._live.mode = mode
    b.v.mode_pending = []


def reach(b, phase, secs=120.0):
    b.handover()
    st = b.run(secs, stop=(phase, Phase.ABORT, Phase.DONE, Phase.GATE_FAIL))
    assert st == phase, (phase, b.states, b.sm.exit_reason)
    tick(b)                     # the phase published, seen by the supervisor
    assert b.sm.state == phase and b.sup._last_phase == phase, b.sup._last_phase


def tick(b, main=True, sup=True, io=True, dt=DT):
    """One step of the wiring with each thread on or off: FC -> I/O ->
    main (PasPrincipal: phase, heartbeat, watchdog) -> supervisor -> state
    machine -> I/O. A thread that is off does not run at all."""
    b.t += dt
    now = 1000.0 + b.t
    b.now = now
    b.v.step(now, dt)
    if io:
        b.v._io_step(now)
    if main:
        b.v.pump()
        dets = b.det.poll(now)
        for d in dets:
            b.det_latest.set(d, d.t)
        b.pas(now)
    if sup:
        b.st.step(now)
        b.st.heartbeat.beat(now)
    if main:
        for d in dets:
            b.sm.on_detection(d, now)
        b.sm.update(now)
    if io:
        b.v._io_step(now)
    if b.states[-1] != b.sm.state:
        b.states.append(b.sm.state)
    return b.sm.state


def jump(b, secs):
    """Nothing runs for `secs` (the whole process frozen); the FC lives."""
    n = int(round(secs / DT))
    for _ in range(n):
        b.t += DT
        b.now = 1000.0 + b.t
        b.v.step(b.now, DT)


def ticks(b, secs, **kw):
    for _ in range(int(round(secs / DT))):
        tick(b, **kw)


# --- G1 ------------------------------------------------------------------------

def test_abort_in_asteptare_peste_modul_altcuiva_e_pasiv():
    """Abort aprins de supervizor, iar FC-ul e deja in ALT_HOLD (pilotul),
    LAND (failsafe-ul de EKF / baterie) sau RTL (failsafe-ul RC): SRC1 cerut,
    ZERO comenzi de mod, EXIT pasiv. Negativ: acelasi abort cu FC-ul in
    GUIDED -> LOITER."""
    out = []
    for mode in (MODE_ALT_HOLD, MODE_LAND, MODE_RTL, MODE_GUIDED):
        b = fire()
        reach(b, Phase.MOVE)
        n_mode, n_src = len(b.v.mode_reqs), len(b.v.src_cmds)
        fc_mode(b, mode)                              # FC, before the abort is read
        b.sup._set_abort('ekf_position: test', passive=False)
        ticks(b, 3.0, sup=False)
        assert b.sm.state == Phase.ABORT, (mode, b.states)
        assert b.v.src_cmds[n_src:] == [1], b.v.src_cmds[n_src:]
        if mode == MODE_GUIDED:
            assert b.v.mode_reqs[n_mode:] == [MODE_LOITER], b.v.mode_reqs[n_mode:]
            assert not b.ev('exit')[-1]['passive']
        else:
            assert b.v.mode_reqs[n_mode:] == [], (mode, b.v.mode_reqs[n_mode:])
            assert b.ev('exit')[-1]['passive'] and b.v.mode == mode
        out.append(f"{b.v.mode_name()}: {len(b.v.mode_reqs) - n_mode} moduri")
    return '; '.join(out)


def test_pilotul_schimba_modul_in_asteptarea_ACK_ului():
    """EXIT normal (AUX jos), ACK-ul SRC1 intarziat 0.3 s; pilotul pune
    STABILIZE la 0.1 s: LOITER NU mai pleaca. Negativ: fara schimbare ->
    LOITER dupa ACK."""
    for pilot in (True, False):
        b = fire(src_ack_delay_s=0.3)
        reach(b, Phase.DESCEND)
        n_mode = len(b.v.mode_reqs)
        b.v.set_aux(1000)
        ticks(b, 0.1)
        assert b.sm.state == Phase.EXIT
        if pilot:
            fc_mode(b, MODE_STABILIZE)
        ticks(b, 3.0)
        assert b.sm.state == Phase.ABORT, b.states
        want = [] if pilot else [MODE_LOITER]
        assert b.v.mode_reqs[n_mode:] == want, (pilot, b.v.mode_reqs[n_mode:])
    return "STABILIZE in asteptarea ACK-ului: 0 moduri; altfel LOITER"


# --- G2 + G3: the main loop hung (F6c) -----------------------------------------------

def test_F6c_bucla_principala_blocata_20s_pilotul_preia():
    """F6c: bucla principala blocata 20 s in MOVE. Supervizorul VIU: la ~1 s
    EXIT direct, SRC1 + LOITER (G3: vederea lui e proaspata si FC-ul in
    GUIDED). Pilotul pune ALT_HOLD la +8 s. La trezire (+20 s) masina de
    stari gaseste abort-ul aprins: il consuma PASIV - zero comenzi de mod,
    FC-ul ramane in ALT_HOLD-ul pilotului."""
    b = fire()
    reach(b, Phase.MOVE)
    n_mode = len(b.v.mode_reqs)
    t0 = b.t
    while b.t - t0 < 20.0:
        tick(b, main=False)
        if abs(b.t - t0 - 8.0) < DT / 2:
            fc_mode(b, MODE_ALT_HOLD)                  # the pilot
    direct = [e for e in b.sup.log if e.monitor == 'exit_direct']
    assert direct and 'LOITER' in direct[0].detail and 'FARA' not in direct[0].detail
    t_dir = direct[0].t - (1000.0 + t0)
    assert 0.9 <= t_dir <= 1.2, t_dir
    assert b.v.mode_reqs[n_mode:] == [MODE_LOITER], b.v.mode_reqs[n_mode:]
    n_wake = len(b.v.mode_reqs)
    ticks(b, 5.0)                                     # the main loop wakes up
    assert b.sm.state == Phase.ABORT, b.states
    assert b.v.mode_reqs[n_wake:] == [], b.v.mode_reqs[n_wake:]
    assert b.v.mode == MODE_ALT_HOLD and b.v.src_cmds[-1] == 1
    assert b.sm._stalled and b.ev('exit')[-1]['passive']
    return (f"EXIT direct la {t_dir:.2f} s cu LOITER; la trezire 0 moduri, "
            f"FC in ALT_HOLD")


def test_F2_tot_procesul_inghetat_15s():
    """F2/F2c: tot procesul inghetat 15 s (SIGSTOP); la SIGCONT toate
    buclele vad o pauza de 15 s, heartbeat-uri vechi, iar instantaneul inca
    spune GUIDED (octeti vechi). Zero comenzi de mod de la masina de stari,
    din _exit_direct si din watchdog - in ambele ordini de trezire, cu FC-ul
    in GUIDED (nimeni n-a atins nimic) sau in LAND (failsafe-ul EKF). Si cu
    supervizorul mort dupa trezire: watchdog-ul cere SRC1, fara LOITER."""
    out = []
    for mod_fc, sup_first in ((MODE_GUIDED, True), (MODE_GUIDED, False),
                               (MODE_LAND, True), (MODE_LAND, False)):
        b = fire()
        b.st.thread = object()                         # "started": watchdog on
        reach(b, Phase.MOVE)
        tick(b)
        n_mode = len(b.v.mode_reqs)
        jump(b, 15.0)
        if mod_fc != MODE_GUIDED:
            fc_mode(b, mod_fc)                        # arrives AFTER the old bytes
        live = b.v._live
        old = live.mode
        live.mode = MODE_GUIDED                       # the old bytes first
        if sup_first:
            tick(b, main=False)
        else:
            tick(b, sup=False)
        live.mode = old if mod_fc == MODE_GUIDED else mod_fc
        ticks(b, 4.0)
        assert b.v.mode_reqs[n_mode:] == [], (mod_fc, sup_first,
                                              b.v.mode_reqs[n_mode:])
        assert 1 in b.v.src_cmds[-3:], b.v.src_cmds
        assert b.sm.state == Phase.ABORT, b.states
        assert not b.st.watchdog_tripped
        assert not any(e.monitor == 'exit_direct' for e in b.sup.log)
        out.append(f"{'LAND' if mod_fc == MODE_LAND else 'GUIDED'}/"
                   f"{'sup' if sup_first else 'principal'} intai: 0")
    # the supervisor never comes back: the watchdog trips, SRC1 only
    b = fire()
    b.st.thread = object()
    reach(b, Phase.MOVE)
    tick(b)
    n_mode = len(b.v.mode_reqs)
    jump(b, 15.0)
    ticks(b, 2.0, sup=False)
    assert b.st.watchdog_tripped and b.st.wd_stalled
    assert b.v.mode_reqs[n_mode:] == [], b.v.mode_reqs[n_mode:]
    out.append('supervizor mort dupa trezire: watchdog SRC1, 0 moduri')
    return '; '.join(out)


# --- negatives: the guard does not make the hand-back inert -------------------------

def test_negativ_vedere_proaspata_in_GUIDED_comenzile_pleaca():
    """Cu vederea proaspata si FC-ul in GUIDED: EXIT normal -> LOITER, EXIT
    direct (principal mort, supervizor viu) -> LOITER, watchdog (supervizor
    mort, principal viu) -> LOITER, iar o pauza de 0.3 s nu e blocaj (EXIT
    dupa ea -> LOITER). EXIT_RTL: tools/test_ceiling_rtl.py."""
    assert STALL_MAX_S == THREAD_HEARTBEAT_MAX_S == 1.0
    out = []
    # 0.3 s gap, then AUX down
    b = fire()
    reach(b, Phase.DESCEND)
    n_mode = len(b.v.mode_reqs)
    jump(b, 0.3)
    b.v.set_aux(1000)
    ticks(b, 3.0)
    assert not b.sm._stalled and b.v.mode_reqs[n_mode:] == [MODE_LOITER]
    assert not b.sup._stalled
    out.append('pauza 0.3 s + AUX jos: LOITER')
    # main thread dead, supervisor alive (G3)
    b = fire()
    reach(b, Phase.DESCEND)
    n_mode = len(b.v.mode_reqs)
    ticks(b, 1.5, main=False)
    assert b.v.mode_reqs[n_mode:] == [MODE_LOITER], b.v.mode_reqs[n_mode:]
    out.append('principal mort: LOITER direct')
    # supervisor dead, main alive (G3)
    b = fire()
    b.st.thread = object()
    reach(b, Phase.DESCEND)
    tick(b)
    n_mode = len(b.v.mode_reqs)
    ticks(b, 1.0, sup=False)
    assert b.st.watchdog_tripped and not b.st.wd_stalled
    assert b.v.mode_reqs[n_mode:] == [MODE_LOITER], b.v.mode_reqs[n_mode:]
    out.append('supervizor mort: watchdog LOITER')
    return '; '.join(out)


def test_watchdog_si_exit_direct_peste_modul_altcuiva():
    """G1 pe vederile lor: watchdog-ul cu FC-ul in ALT_HOLD (pilotul) ->
    SRC1, fara LOITER; _exit_direct cu FC-ul in ALT_HOLD -> SRC1, fara
    LOITER; _exit_direct cu instantaneul vechi (I/O oprit) -> fara LOITER."""
    b = fire()
    b.st.thread = object()
    reach(b, Phase.DESCEND)
    tick(b)
    fc_mode(b, MODE_ALT_HOLD)
    n_mode = len(b.v.mode_reqs)
    ticks(b, 1.0, sup=False)
    assert b.st.watchdog_tripped and b.v.mode_reqs[n_mode:] == [], b.v.mode_reqs[n_mode:]
    b = fire()
    reach(b, Phase.DESCEND)
    fc_mode(b, MODE_ALT_HOLD)
    n_mode = len(b.v.mode_reqs)
    ticks(b, 1.5, main=False)
    d = [e for e in b.sup.log if e.monitor == 'exit_direct']
    assert d and 'FARA LOITER' in d[0].detail and b.v.mode_reqs[n_mode:] == []
    # stale snapshot: the I/O thread stops publishing, FC still GUIDED
    b = fire()
    reach(b, Phase.DESCEND)
    n_mode = len(b.v.mode_reqs)
    b.v.hb_t = None
    for _ in range(int(1.5 / DT)):
        tick(b, main=False, io=False)
        b.v.hb_t = b.now                               # link "alive"
    d = [e for e in b.sup.log if e.monitor == 'exit_direct']
    assert d and 'instantaneu vechi' in d[0].detail, [str(e) for e in b.sup.log[-3:]]
    assert b.v.mode_reqs[n_mode:] == []
    assert HANDBACK_VIEW_MAX_AGE_S == 0.5
    return "watchdog / exit direct peste ALT_HOLD: SRC1 doar; instantaneu vechi: fara LOITER"


def test_blocajul_supervizorului_zavoreste_pasiv_pana_la_incercare_noua():
    """Supervizorul nu a rulat 3 s (doar el): la trezire heartbeat-ul
    principalului PARE vechi doar pentru ca el a dormit - nu 'fir mort ->
    LOITER direct', ci EXIT pasiv zavorat (abort_passive). Masina de stari
    il executa pasiv. O incercare noua prin poarta sterge blocajul: EXIT-ul
    ei (AUX jos) trimite iar LOITER."""
    b = fire()
    reach(b, Phase.DESCEND)
    n_mode = len(b.v.mode_reqs)
    b.hb.beat(b.now - 2.0)          # what the supervisor will see on waking
    jump(b, 3.0)
    b.hb.beat(b.now - 3.0)
    tick(b, main=False)
    assert b.sup.latched == Action.EXIT and b.sup.latched_monitor == 'stall_supervizor'
    assert b.sup.abort_passive and not any(e.monitor == 'exit_direct' for e in b.sup.log)
    ticks(b, 3.0)
    assert b.sm.state == Phase.ABORT and b.v.mode_reqs[n_mode:] == []
    # a new attempt clears it
    b.v.set_aux(1000)
    ticks(b, 0.3)
    b.v.set_aux(2000)
    st = None
    for _ in range(int(60 / DT)):
        st = tick(b)
        if st in (Phase.DESCEND, Phase.ABORT, Phase.GATE_FAIL) and len(b.states) > 3:
            break
    assert st == Phase.DESCEND, b.states
    assert not b.sup._stalled and not b.sm._stalled
    n_mode = len(b.v.mode_reqs)
    b.v.set_aux(1000)
    ticks(b, 3.0)
    assert b.v.mode_reqs[n_mode:] == [MODE_LOITER], b.v.mode_reqs[n_mode:]
    return "blocaj al supervizorului: EXIT pasiv; incercare noua: LOITER din nou"


def test_blocaj_in_EXIT_RTL_opreste_comenzile():
    """Blocaj al buclei principale in mijlocul caii EXIT_RTL (RTL refuzat,
    in reincercare): dupa trezire nicio reincercare RTL si niciun LAND."""
    b = fire(refuse_modes=(MODE_RTL,))
    reach(b, Phase.DESCEND)
    b.v.rel_alt_offset = 40.0
    for _ in range(int(3.0 / DT)):
        tick(b)
        if MODE_RTL in b.v.mode_reqs:
            break
    assert b.sm.state == Phase.EXIT_RTL and MODE_RTL in b.v.mode_reqs
    n_mode = len(b.v.mode_reqs)
    ticks(b, 2.0, main=False)
    ticks(b, 3.0)
    assert b.v.mode_reqs[n_mode:] == [], b.v.mode_reqs[n_mode:]
    assert b.sm.state == Phase.ABORT and b.ev('exit_done')[-1]['passive']
    return "blocaj in EXIT_RTL: 0 comenzi dupa"


TESTS = [
    ('G1: abort in asteptare peste ALT_HOLD / LAND / RTL -> pasiv',
     test_abort_in_asteptare_peste_modul_altcuiva_e_pasiv),
    ('G1: pilotul schimba modul in asteptarea ACK-ului',
     test_pilotul_schimba_modul_in_asteptarea_ACK_ului),
    ('F6c: principal blocat 20 s, pilotul preia', test_F6c_bucla_principala_blocata_20s_pilotul_preia),
    ('F2/F2c: tot procesul inghetat 15 s', test_F2_tot_procesul_inghetat_15s),
    ('negativ: vedere proaspata in GUIDED, comenzile pleaca',
     test_negativ_vedere_proaspata_in_GUIDED_comenzile_pleaca),
    ('G1: watchdog si exit direct peste modul altcuiva',
     test_watchdog_si_exit_direct_peste_modul_altcuiva),
    ('G2: blocajul supervizorului, pana la incercare noua',
     test_blocajul_supervizorului_zavoreste_pasiv_pana_la_incercare_noua),
    ('G2: blocaj in EXIT_RTL opreste comenzile', test_blocaj_in_EXIT_RTL_opreste_comenzile),
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
