#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste de regresie pentru nova/state_machine.py, fara SITL si fara Gazebo.

Masina de stari foloseste ceas injectat (update(now) / on_detection(det, now)),
deci se poate rula in timp accelerat peste un vehicul de mucava. O secventa
completa dureaza ~1 s de perete in loc de ~2 minute.

    python3 tools/test_state_machine.py

Ce acopera, si de ce:
  - secventa completa ajunge la HANDBACK
  - SCORING_CAPTURE se declanseaza EXACT O DATA pe secventa. Bug-ul prins in
    SITL la testul A1: captura pornea din on_detection indiferent de stare,
    iar reset_sequence stergea scoring_shot, deci in coborarea unui RTL se
    intra intr-un ciclu IDLE <-> SCORING_CAPTURE la fiecare cadru. Pe Pi 4
    ar fi insemnat capturi full-res repetate, care satureaza I/O-ul si
    golesc ring buffer-ul exact cand e nevoie de el (8.3.3).
  - captura NU se declanseaza in afara coborarii autonome (cazul RTL)
  - DISTANCE_SENSOR vine numai din detectii, niciodata dintr-o valoare de
    rezerva inventata (A2, vezi 5.9)
  - PLND_ENABLED e armat la intrarea in secventa si stins la iesire (A1)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova.handover import HandoverGate  # noqa: E402
from nova.rc import OverrideMonitor  # noqa: E402
from nova.safety import Action, SafetySupervisor  # noqa: E402
from nova.state_machine import (AUX_HIGH_PWM,  # noqa: E402
                                LandingStateMachine, SequenceConfig, State)
from nova.vehicle import MODE_LOITER, MODE_LAND, MODE_RTL  # noqa: E402
import fake_detector as fd  # noqa: E402

LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2


class StubVehicle:
    """Aceeasi interfata ca nova.vehicle.Vehicle, fara MAVLink."""

    def __init__(self, alt=6.0):
        self.x, self.y, self.z = 0.05, -0.04, -alt
        self.vz = 0.5
        self.rel_alt = alt
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = True
        self.armed = True
        # Pornim in LOITER: intrarea in secventa o face poarta de handover,
        # nu pilotul comutand pe LAND.
        self.mode = MODE_LOITER
        self.landed_state = LANDED_IN_AIR
        self.time_boot_ms = 1000
        self.params = {'RC1_TRIM': 1500, 'RC2_TRIM': 1500,
                       'RC3_TRIM': 1100, 'RC4_TRIM': 1500}
        # canalul 7 (AUX) sus = cerere de handover
        self.rc = (1500, 1500, 1100, 1500, 1000, 1000, AUX_HIGH_PWM, 1000)
        self.rc_t = 0.0
        self.n_lt = self.n_ds = 0
        self.param_sets = []
        self.sent_ranges = []
        self.takeoffs = []
        self.mode_reqs = []
        self.target_alt = None

    @property
    def alt(self):
        return -self.z

    def on_ground(self):
        return self.landed_state == LANDED_ON_GROUND

    def send_landing_target(self, ax, ay, d):
        self.n_lt += 1

    def send_distance(self, r):
        self.n_ds += 1
        self.sent_ranges.append(r)

    def request_param(self, name):
        pass

    def set_aux(self, pwm):
        self.set_rc(7, pwm)

    def set_rc(self, ch, value):
        """ch e 1-indexat, ca in RC_CHANNELS."""
        rc = list(self.rc)
        rc[ch - 1] = value
        self.rc = tuple(rc)

    def set_param(self, name, value, now=None):
        self.params[name] = float(value)
        self.param_sets.append((name, float(value)))

    def update_params(self, now):
        pass

    def request_mode(self, m):
        self.mode_reqs.append(m)
        self.mode = m

    def send_takeoff(self, alt):
        self.takeoffs.append(alt)
        self.target_alt = alt

    def mode_name(self):
        return str(self.mode)

    def step(self, dt, marker_n, marker_e):
        """Fizica de mucava: coboara in LAND, urca dupa NAV_TAKEOFF."""
        if self.target_alt is not None:
            self.landed_state = LANDED_IN_AIR
            self.vz = -1.5
            self.z -= 1.5 * dt
            self.rel_alt = -self.z
            return
        if self.mode == MODE_LAND:
            if self.alt > 0.06:
                self.z += 0.5 * dt
                self.vz = 0.5
                self.x += (marker_n - self.x) * 0.9 * dt
                self.y += (marker_e - self.y) * 0.9 * dt
            else:
                self.z = -0.045
                self.vz = 0.0
                self.landed_state = LANDED_ON_GROUND
            self.rel_alt = -self.z


class Args:
    north, east = 2.0, 1.5
    noise_px = 0.5
    dropout = 0.0
    latency_ms = 0.0
    focal_px = 933.0


def build_app(alt=6.0):
    """Cablajul complet, ca in tools/fake_detector.py: un singur
    OverrideMonitor impartit de poarta si de supervizor, supervizorul legat
    in bucla inaintea masinii de stari."""
    args = Args()
    v = StubVehicle(alt)
    v.mode = MODE_LOITER
    v.set_aux(AUX_HIGH_PWM)
    v.x, v.y = args.north - 0.6, args.east + 0.4
    det = fd.FakeDetector(v, args)
    ov = OverrideMonitor(v)
    gate = HandoverGate(v, ov, autonomy_enabled=True)   # testam secventa, nu E0
    sup = SafetySupervisor(v, verbose=False, override=ov)
    events = []
    sm = LandingStateMachine(v, SequenceConfig(conv=2), verbose=False,
                             on_event=lambda n, i: events.append((n, i)),
                             gate=gate)
    return v, det, sm, sup, events, args


def run_app(v, det, sm, sup, args, seconds=40.0, dt=0.002, stop_states=(),
            on_step=None):
    """Aceeasi ordine ca nova.state_machine.run_loop."""
    t = 0.0
    last_det_t = None
    states = []
    while t < seconds:
        now = 1000.0 + t
        v.rc_t = now
        dets = det.poll(now)
        for d in dets:
            last_det_t = d.t if last_det_t is None else max(last_det_t, d.t)
        age = None if last_det_t is None else (now - last_det_t)
        sup.update(now, age, sm.state)
        for d in dets:
            sm.on_detection(d, now)
        sm.update(now)
        if not states or states[-1] != sm.state:
            states.append(sm.state)
        if on_step:
            on_step(t, v, sm, sup)
        v.step(dt, args.north, args.east)
        t += dt
        if sm.state in stop_states:
            break
    return states


def build(mode=MODE_LOITER, alt=6.0, aux=AUX_HIGH_PWM, **kw):
    args = Args()
    for k, v in kw.items():
        setattr(args, k, v)
    v = StubVehicle(alt)
    v.mode = mode
    v.set_aux(aux)
    # pornim aproape de marker, ca sa fie in cadru
    v.x, v.y = args.north - 0.6, args.east + 0.4
    det = fd.FakeDetector(v, args)
    events = []
    ov = OverrideMonitor(v)
    gate = HandoverGate(v, ov, autonomy_enabled=True)   # testam secventa, nu E0
    sm = LandingStateMachine(v, SequenceConfig(conv=2), verbose=False,
                             on_event=lambda n, i: events.append((n, i)),
                             gate=gate)
    return v, det, sm, events, args


def run(v, det, sm, args, seconds=40.0, dt=0.002, stop_states=()):
    t = 0.0
    states = []
    while t < seconds:
        now = 1000.0 + t          # ceas injectat, timp accelerat
        v.rc_t = now              # fluxul RC e proaspat
        for d in det.poll(now):
            sm.on_detection(d, now)
        sm.update(now)
        if not states or states[-1] != sm.state:
            states.append(sm.state)
        v.step(dt, args.north, args.east)
        t += dt
        if sm.state in stop_states:
            break
    return states


# --- teste -----------------------------------------------------------------

def test_secventa_completa():
    v, det, sm, events, args = build()
    states = run(v, det, sm, args,
                 stop_states=(State.HANDBACK, State.ABORT))
    assert sm.state == State.HANDBACK, f"stare finala {sm.state}, stari {states}"
    assert State.TOUCHDOWN_CONFIRM in states
    assert State.ASCENT in states
    assert len(v.takeoffs) >= 1, "nu s-a comandat NAV_TAKEOFF"
    return f"stari: {' -> '.join(states)}"


def test_scoring_capture_exact_o_data():
    """Regresia pentru bug-ul prins in SITL la A1."""
    v, det, sm, events, args = build()
    run(v, det, sm, args, stop_states=(State.HANDBACK, State.ABORT))
    n = sum(1 for name, _ in events if name == 'scoring_capture')
    assert n == 1, f"scoring_capture s-a declansat de {n} ori, nu o data"
    info = next(i for name, i in events if name == 'scoring_capture')
    assert info['marker_px'] > 980, info
    return f"o singura captura, la {info['alt']:.3f} m / {info['marker_px']:.0f} px"


def test_fara_captura_in_afara_coborarii():
    """Coborare in RTL peste marker, fara cerere de handover: nicio captura,
    nicio oscilatie de stare."""
    v, det, sm, events, args = build(mode=MODE_RTL, alt=3.0, aux=1000)
    states = run(v, det, sm, args, seconds=12.0)
    # vehiculul coboara doar in LAND in stub, deci il coboram manual
    for _ in range(3000):
        now = 2000.0 + _ * 0.002
        v.z += 0.5 * 0.002
        if v.alt < 0.2:
            break
        for d in det.poll(now):
            sm.on_detection(d, now)
        sm.update(now)
        if not states or states[-1] != sm.state:
            states.append(sm.state)
    n = sum(1 for name, _ in events if name == 'scoring_capture')
    assert n == 0, f"captura declansata de {n} ori in afara coborarii autonome"
    assert set(states) == {State.IDLE}, f"stari neasteptate: {states}"
    return "nicio captura, starea a ramas IDLE"


def test_fara_telemetru_de_rezerva():
    """A2: fiecare DISTANCE_SENSOR vine dintr-o detectie reala."""
    v, det, sm, events, args = build()
    run(v, det, sm, args, stop_states=(State.HANDBACK, State.ABORT))
    assert v.n_ds == v.n_lt, (f"{v.n_ds} DISTANCE_SENSOR vs {v.n_lt} "
                              f"LANDING_TARGET: exista o sursa inventata")
    assert v.sent_ranges, "nu s-a trimis niciun range"
    assert max(v.sent_ranges) < 15.0, (
        f"range maxim {max(v.sent_ranges):.1f} m - miroase a valoare de rezerva")
    return (f"{v.n_ds} range-uri, toate din detectii, "
            f"{min(v.sent_ranges):.2f}-{max(v.sent_ranges):.2f} m")


def test_precland_armat_si_stins():
    """A1: PLND_ENABLED 1 la intrarea in secventa, 0 la iesire."""
    v, det, sm, events, args = build()
    run(v, det, sm, args, stop_states=(State.HANDBACK, State.ABORT))
    sets = [(n, val) for n, val in v.param_sets if n == 'PLND_ENABLED']
    assert sets and sets[0][1] == 1.0, f"PLND nu a fost armat: {sets}"
    assert sets[-1][1] == 0.0, f"PLND nu a fost stins la final: {sets}"
    return f"PLND_ENABLED: {' -> '.join(f'{val:g}' for _, val in sets)}"


def test_abort_to_rtl_stinge_precland_intai():
    v, det, sm, events, args = build()
    sm.arm_precland(1000.0)
    v.param_sets.clear()
    sm.abort_to_rtl('test', 1001.0)
    assert v.param_sets and v.param_sets[0] == ('PLND_ENABLED', 0.0), \
        f"primul lucru comandat trebuia sa fie PLND=0, nu {v.param_sets}"
    assert v.mode_reqs[-1] == MODE_RTL
    assert sm.state == State.ABORT
    return "PLND stins inainte de comanda RTL"


# --- intrarea prin poarta de handover (§8, 15.2.3) -------------------------

def test_fara_aux_nu_porneste_nimic():
    """CAZ NEGATIV central: nu exista cale de intrare care ocoleste poarta.
    Chiar daca pilotul pune vehiculul in LAND cu mana, secventa NU porneste."""
    v, det, sm, events, args = build(mode=MODE_LAND, aux=1000)
    states = run(v, det, sm, args, seconds=8.0)
    assert set(states) == {State.IDLE}, f"stari {states}"
    assert not v.param_sets, f"a armat PLND fara handover: {v.param_sets}"
    return "LAND comandat manual nu porneste segmentul autonom"


def test_refuz_altitudine_ajunge_in_REJECT():
    v, det, sm, events, args = build(alt=14.0)
    run(v, det, sm, args, seconds=6.0)
    assert sm.state == State.REJECT, f"stare {sm.state}"
    rej = [i for n, i in events if n == 'handover_reject']
    assert rej and 'altitudine' in rej[0]['reason'], rej
    assert not v.param_sets, "a armat PLND desi a refuzat"
    return f"refuzat: {rej[0]['reason']}"


def test_reject_se_elibereaza_doar_cu_AUX_jos():
    """Refuzul ramane afisat pana cand pilotul lasa comutatorul jos."""
    v, det, sm, events, args = build(alt=14.0)
    run(v, det, sm, args, seconds=6.0)
    assert sm.state == State.REJECT
    run(v, det, sm, args, seconds=3.0)
    assert sm.state == State.REJECT, "a iesit din REJECT cu AUX inca sus"
    v.set_aux(1000)
    run(v, det, sm, args, seconds=0.5)
    assert sm.state == State.IDLE, f"stare {sm.state}"
    return "REJECT -> IDLE doar dupa eliberarea AUX"


def test_frontul_crescator_nu_starea():
    """Un comutator lasat sus nu trebuie sa reporneasca secventa la nesfarsit."""
    v, det, sm, events, args = build(alt=14.0)
    run(v, det, sm, args, seconds=6.0)
    n1 = len([1 for n, _ in events if n == 'handover_reject'])
    run(v, det, sm, args, seconds=6.0)
    n2 = len([1 for n, _ in events if n == 'handover_reject'])
    assert n1 == n2 == 1, f"{n2} refuzuri; ar trebui unul singur"
    return "o singura cerere pe front, nu una pe ciclu"


def test_neutrul_se_memoreaza_la_accept():
    v, det, sm, events, args = build()
    run(v, det, sm, args, stop_states=(State.DESCEND_TRACK,))
    acc = [i for n, i in events if n == 'handover_accept']
    assert acc, "niciun ACCEPT"
    assert acc[0]['neutral'] == (1500, 1500, 1100, 1500), acc[0]
    assert sm.gate.ov.neutral is not None
    return f"neutru memorat la ACCEPT: {acc[0]['neutral']}"


# --- cablajul aplicatiei (regresia raportata din zbor) ---------------------

def test_supervizorul_se_armeaza_in_cablajul_real():
    """REGRESIE. Supervizorul a fost inert in aplicatie pentru ca se arma pe
    tranzitia IDLE -> DESCEND_TRACK, care a disparut cand intrarea a devenit
    IDLE -> HANDOVER_CHECK -> ACQUIRE -> DESCEND_TRACK. Nicio suita nu
    acoperea cablajul aplicatiei, doar piesele separate."""
    v, det, sm, sup, events, args = build_app()
    run_app(v, det, sm, sup, args, stop_states=(State.DESCEND_TRACK,))
    assert sm.state == State.DESCEND_TRACK, sm.state
    assert sup.armed, "supervizorul NU s-a armat in coborarea autonoma"
    return "armat automat cand secventa ajunge in DESCEND_TRACK"


def test_mansa_opreste_coborarea_autonoma():
    """Cazul raportat din zbor: mansa la maxim in timpul aterizarii autonome
    trebuie sa opreasca secventa si sa comute pe un mod pilotabil."""
    v, det, sm, sup, events, args = build_app()
    moved = {'done': False}

    def on_step(t, v, sm, sup):
        # la ~3 m, pilotul pune mana pe manse
        if not moved['done'] and sm.state == State.DESCEND_TRACK and v.alt < 3.0:
            moved['done'] = True
        if moved['done']:
            v.set_rc(1, 1500 + 400)

    states = run_app(v, det, sm, sup, args, on_step=on_step,
                     stop_states=(State.HANDBACK,), seconds=30.0)
    assert moved['done'], "nu s-a ajuns la momentul miscarii"
    assert sup.latched == Action.OVERRIDE, (
        f"supervizorul nu a detectat override-ul (latched={sup.latched})")
    assert sup.passive, "nu a intrat in pasiv definitiv"
    assert v.mode_reqs[-1] == MODE_LOITER, v.mode_reqs
    assert sm.state != State.HANDBACK, "secventa a continuat pana la capat"
    assert State.TOUCHDOWN_CONFIRM not in states, (
        f"a aterizat desi pilotul a preluat: {states}")
    return f"OVERRIDE -> LOITER, secventa oprita in {sm.state}"


TESTS = [
    ('secventa completa ajunge la HANDBACK', test_secventa_completa),
    ('SCORING_CAPTURE exact o data', test_scoring_capture_exact_o_data),
    ('fara captura in afara coborarii', test_fara_captura_in_afara_coborarii),
    ('fara telemetru de rezerva', test_fara_telemetru_de_rezerva),
    ('PLND armat si stins', test_precland_armat_si_stins),
    ('abort_to_rtl stinge PLND intai', test_abort_to_rtl_stinge_precland_intai),
    ('NEGATIV: fara AUX nu porneste nimic', test_fara_aux_nu_porneste_nimic),
    ('refuz de altitudine -> REJECT', test_refuz_altitudine_ajunge_in_REJECT),
    ('REJECT se elibereaza cu AUX jos', test_reject_se_elibereaza_doar_cu_AUX_jos),
    ('cerere pe front, nu pe stare', test_frontul_crescator_nu_starea),
    ('neutru memorat la ACCEPT', test_neutrul_se_memoreaza_la_accept),
    ('REGRESIE: supervizor armat in cablajul real',
     test_supervizorul_se_armeaza_in_cablajul_real),
    ('mansa opreste coborarea autonoma', test_mansa_opreste_coborarea_autonoma),
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
