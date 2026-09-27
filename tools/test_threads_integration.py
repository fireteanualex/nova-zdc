#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Testul de integrare al refactorului pe fire (faza 5, refactor/threads).

    python3 tools/test_threads_integration.py

Stub MAVLink: vehiculul simulat din test_extnav_landing (fizica de mucava,
FC-ul adopta modurile cu intarziere si confirma sursele EKF), rulat pe DOUA
cablaje, cu acelasi ceas injectat si aceleasi detectii (acelasi seed):

  - SINCRON: cablajul de dinaintea refactorului - supervizorul in bucla
    principala, verdictul lui prin callback (on_exit), vehiculul citit si
    scris direct.
  - PE FIRE: cablajul din tools/nova_pi.py dupa faza 5 - Vehicle cu fir
    I/O (instantanee prin _io_step, comenzi prin cozi), supervizorul peste
    VehicleView si pasit prin SupervisorThread.step, verdictul prin `abort`
    (Event + contor), firul principal prin PasPrincipal. Firele sunt PASITE
    de test, determinist, in ordinea in care ar rula: FC -> I/O -> principal
    -> detectie -> supervizor -> masina de stari -> I/O.

Ce conteaza: aceeasi secventa de mesaje discrete (moduri, surse EKF,
STATUSTEXT), in aceeasi ordine; aceleasi stari si trepte; fluxurile
periodice (VPE, consemne) echivalente. Plus proprietatile noi ale caii pe
fire: abort-ul vechi nu omoara incercarea noua, abort-ul nou intre doi pasi
nu se pierde, PLND nu emite cu abort aprins, oprirea e ordonata.
"""

import contextlib
import io
import math
import os
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from nova import extnav as ex                               # noqa: E402
from nova.concurrency import Heartbeat, Latest              # noqa: E402
from nova.detection import Detection                        # noqa: E402
from nova.ekf_source import EkfSourceManager                # noqa: E402
from nova.extnav_landing import (ExtNavConfig, ExtNavLanding,  # noqa: E402
                                 Phase, SRC2_PHASES)
from nova.handover import HandoverGate                      # noqa: E402
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.safety import ExtNavSupervisor, SafetySupervisor  # noqa: E402
from nova.state_machine import (LandingStateMachine,        # noqa: E402
                                SequenceConfig, State)
from nova.supervisor_thread import SupervisorThread         # noqa: E402
from nova.vehicle import MODE_LAND, MODE_LOITER             # noqa: E402
import nova_pi                                              # noqa: E402
import test_extnav_landing as tel                           # noqa: E402
import test_state_machine as tsm                            # noqa: E402

AUX = tel.AUX


class AppFire:
    """Cablajul de bord de dupa faza 5, pe ceas injectat, firele pasite
    de test (vezi docstring-ul modulului)."""

    def __init__(self, h=8.0, p=(2.0, -1.0), rate=0.05, seed=1, yaw=0.3,
                 cfg=None):
        self.v = tel.SimVeh(h=h, p=p, yaw=yaw, threaded=True)
        self.det = tel.TruthDetector(self.v, rate=rate, seed=seed)
        self.est = ex.ExtNavEstimator(self.v)
        self.ekf = EkfSourceManager(self.v, verbose=False, faze=SRC2_PHASES)
        self.ov = OverrideMonitor(self.v)
        self.gate = HandoverGate(self.v, self.ov, autonomy_enabled=True,
                                 alt_min_m=1.0, alt_max_m=12.0,
                                 dist_max_m=None, detection_max_age_s=None)
        self.events = []
        self.sm = ExtNavLanding(self.v, self.est, self.ekf, self.gate,
                                cfg or ExtNavConfig(aux_channel=AUX),
                                on_event=lambda n, i: self.events.append((n, i)),
                                verbose=False)
        # exact ca in nova_pi.cablaj_extnav: vedere, fara on_exit, abort
        self.sup = ExtNavSupervisor(nova_pi.vedere(self.v), verbose=False,
                                    override=self.ov)
        self.sm.attach_supervisor(self.sup)
        self.now = 1000.0
        self.faza = Latest(self.sm.state)
        self.hb = Heartbeat('principal', clock=lambda: self.now)
        self.det_latest = Latest()
        self.st = SupervisorThread(self.sup, self.v, self.det_latest, self.faza,
                                   clock=lambda: self.now)
        self.pas = nova_pi.PasPrincipal(self.sm, self.faza, self.hb,
                                        supervisor_thread=self.st)
        self.t = 0.0
        self.states = [self.sm.state]
        self.h_targets = []
        self.v.step(0.0, 0.0)
        self.v._io_step(0.0)
        self.v.pump()

    def tick(self, now, dt, sup=True):
        self.now = now
        self.v.step(now, dt)              # "FC-ul": fizica in _live
        self.v._io_step(now)              # firul I/O: publica instantaneul
        self.v.pump()                     # firul principal: fatada <- instantaneu
        dets = self.det.poll(now)         # firul de detectie
        for d in dets:
            self.det_latest.set(d, d.t)
        self.pas(now)                     # on_poll: faza, heartbeat, watchdog
        if sup:
            self.st.step(now)             # firul supervizorului
        for d in dets:
            self.sm.on_detection(d, now)
        self.sm.update(now)
        self.v._io_step(now)              # firul I/O: goleste cozile

    def run(self, seconds, dt=0.02, stop=(), sup=True):
        end = self.t + seconds
        while self.t < end:
            self.t += dt
            self.tick(1000.0 + self.t, dt, sup=sup)
            if self.states[-1] != self.sm.state:
                self.states.append(self.sm.state)
                if self.sm.state == Phase.DESCEND:
                    self.h_targets.append(round(self.sm.h_target, 2))
            if self.sm.state in stop:
                break
        return self.sm.state

    def ev(self, name):
        return [i for n, i in self.events if n == name]

    def handover(self):
        self.v.set_aux(1000)
        self.run(0.3)
        self.v.set_aux(2000)


def _mesaje(v):
    """Secventa discreta de mesaje vazuta de FC-ul de mucava, in ordine."""
    return {'moduri': list(v.mode_reqs), 'surse': list(v.src_cmds),
            'statustext': list(v.statustexts)}


# --- teste -----------------------------------------------------------------

def test_secventa_completa_aceleasi_mesaje_aceeasi_ordine():
    """Secventa completa de la 8 m, detectii la 5 %, pe ambele cablaje:
    aceleasi stari, trepte, moduri, surse EKF si STATUSTEXT, in aceeasi
    ordine; fluxurile periodice echivalente; acelasi contact."""
    a = tel.App(h=8.0, p=(2.0, -1.0), rate=0.05)
    b = AppFire(h=8.0, p=(2.0, -1.0), rate=0.05)
    for app in (a, b):
        app.handover()
        st = app.run(120.0, stop=(Phase.TOUCHDOWN, Phase.ABORT, Phase.GATE_FAIL))
        assert st == Phase.TOUCHDOWN, (type(app).__name__, st, app.states,
                                       app.sm.exit_reason)
        app.run(1.0)                  # dezarmarea de pe sol: SRC1, IDLE
        assert app.sm.state == Phase.IDLE
    assert a.states == b.states, (a.states, b.states)
    assert a.h_targets == b.h_targets == [4.0, 2.0, 1.0], (a.h_targets, b.h_targets)
    ma, mb = _mesaje(a.v), _mesaje(b.v)
    assert ma == mb, (ma, mb)
    assert mb['surse'] == [2, 1] and MODE_LAND in mb['moduri'], mb
    # fluxurile periodice: acelasi numar de VPE si de consemne (+-2 %) si
    # aceleasi trepte in consemne
    for camp in ('vpe', 'targets'):
        na, nb = len(getattr(a.v, camp)), len(getattr(b.v, camp))
        assert na and abs(na - nb) <= max(2, 0.02 * na), (camp, na, nb)
    za = sorted({round(t[7], 1) for t in a.v.targets})
    zb = sorted({round(t[7], 1) for t in b.v.targets})
    assert za == zb, (za, zb)
    ea, eb = math.hypot(*a.v.p), math.hypot(*b.v.p)
    assert ea < ex.tol_m(1.0) and eb < ex.tol_m(1.0), (ea, eb)
    assert b.v.n_lt == 0 and b.v.n_ds == 0
    # calea pe fire chiar a fost pe fire: nimic n-a ocolit instantaneul
    assert b.v.threaded and b.v.state.count > 1000 and b.st.n_steps > 1000
    assert b.hb.count == b.st.n_steps and b.faza.get()[0] == Phase.IDLE
    assert b.sup.on_exit is None
    return (f"{len(a.states)} stari identice, moduri {ma['moduri']}, surse "
            f"{ma['surse']}, VPE {len(a.v.vpe)}/{len(b.v.vpe)}, contact "
            f"{ea * 100:.1f}/{eb * 100:.1f} cm")


def test_EXIT_supervizor_prin_abort_fara_callback():
    """EKF invalid dupa comutare: supervizorul cere EXIT. Sincron, prin
    on_exit; pe fire, prin `abort` citit de masina de stari in pasul ei.
    Aceleasi mesaje: SRC1 apoi LOITER, niciun RTL, ABORT la final."""
    a = tel.App(rate=0.2)
    b = AppFire(rate=0.2)
    for app in (a, b):
        app.handover()
        assert app.run(60.0, stop=(Phase.MOVE, Phase.ABORT)) == Phase.MOVE, app.states
        app.v.ekf_valid_after_switch = False
        st = app.run(5.0, stop=(Phase.ABORT, Phase.TOUCHDOWN, Phase.GATE_FAIL))
        assert st == Phase.ABORT, (type(app).__name__, st, app.states)
        assert app.sup.latched_monitor == 'ekf_position', app.sup.latched_monitor
        assert 'ekf_position' in app.sm.exit_reason, app.sm.exit_reason
    assert a.states == b.states, (a.states, b.states)
    assert _mesaje(a.v) == _mesaje(b.v), (_mesaje(a.v), _mesaje(b.v))
    assert b.v.src_cmds[-1] == 1 and b.v.mode_reqs[-1] == MODE_LOITER
    assert 6 not in b.v.mode_reqs
    assert b.sup.abort.is_set() and b.sup.abort_n == 1
    assert b.sm._abort_seen_n == 1
    ex_a, ex_b = a.ev('exit')[0], b.ev('exit')[0]
    assert ex_a['reason'] == ex_b['reason'] and not ex_b['passive'], (ex_a, ex_b)
    return f"EXIT '{ex_b['reason']}' pe ambele cai; moduri {b.v.mode_reqs}"


def test_abort_vechi_nu_omoara_incercarea_noua():
    """Cursa reala a firelor: dupa EXIT -> ABORT, abort-ul ramane aprins
    pana cand FIRUL supervizorului vede incercarea noua. Masina de stari
    poate intra in GATE_SEARCH intre timp (AUX sus): abort-ul vechi
    (acelasi numar) nu o trimite inapoi in IDLE. Cand supervizorul isi
    face pasul, elibereaza zavorul, iar secventa continua pana la ENGAGE."""
    b = AppFire(rate=0.2)
    b.handover()
    assert b.run(60.0, stop=(Phase.MOVE, Phase.ABORT)) == Phase.MOVE, b.states
    b.v.ekf_valid_after_switch = False
    assert b.run(5.0, stop=(Phase.ABORT,)) == Phase.ABORT, b.states
    assert b.sup.abort.is_set() and b.sup.latched_monitor == 'ekf_position'
    b.v.ekf_valid_after_switch = True
    # incercare noua, cu supervizorul "in urma": nu face niciun pas
    b.v.set_aux(1000)
    b.run(0.3, sup=False)
    b.v.set_aux(2000)
    b.run(0.5, sup=False)
    assert b.sup.abort.is_set() and b.sm.state == Phase.GATE_SEARCH, (
        b.sm.state, b.sup.abort_reason)
    # supervizorul isi reia pasii: zavor eliberat, re-armat, secventa
    # continua in ENGAGE
    st = b.run(30.0, stop=(Phase.ENGAGE, Phase.ABORT, Phase.GATE_FAIL))
    assert not b.sup.abort.is_set() and st == Phase.ENGAGE, (st, b.states)
    assert b.sup.armed and b.sup.latched == 0, (b.sup.armed, b.sup.latched)
    assert any('latch_release' in str(e) for e in b.sup.log[-3:]), b.sup.log[-3:]
    # ... si de aici incolo la fel ca pe cablajul sincron, pana la capat
    a = tel.App(rate=0.2)
    a.handover()
    a.run(60.0, stop=(Phase.MOVE, Phase.ABORT))
    a.v.ekf_valid_after_switch = False
    a.run(5.0, stop=(Phase.ABORT,))
    a.v.ekf_valid_after_switch = True
    a.v.set_aux(1000)
    a.run(0.3)
    a.v.set_aux(2000)
    a.run(30.0, stop=(Phase.ENGAGE, Phase.ABORT, Phase.GATE_FAIL))
    for app in (a, b):
        app.run(30.0, stop=(Phase.TOUCHDOWN, Phase.ABORT, Phase.GATE_FAIL))
    assert a.states == b.states, (a.states, b.states)
    assert _mesaje(a.v) == _mesaje(b.v), (_mesaje(a.v), _mesaje(b.v))
    return ("GATE_SEARCH supravietuieste abort-ului vechi; zavor eliberat; "
            f"ENGAGE; apoi identic cu sincronul ({b.states[-1]})")


def test_abort_nou_intre_doi_pasi_nu_se_pierde():
    """Stins si reaprins intre doi pasi ai masinii de stari (alt monitor,
    aceeasi incercare): Event-ul arata la fel ca inainte, contorul nu."""
    b = AppFire(rate=0.2)
    b.handover()
    assert b.run(60.0, stop=(Phase.DESCEND, Phase.ABORT)) == Phase.DESCEND
    n = b.sup.abort_n
    b.sup._clear_abort()
    b.sup._set_abort('thread_detectie: firul de detectie fara heartbeat de 1.2 s')
    assert b.sup.abort_n == n + 1
    n_src = len(b.v.src_cmds)
    b.run(3.0, sup=False, stop=(Phase.ABORT,))
    assert b.sm.state == Phase.ABORT and 'thread_detectie' in b.sm.exit_reason
    assert b.v.src_cmds[n_src:] == [1] and b.v.mode_reqs[-1] == MODE_LOITER
    # si, unitar: acelasi numar = nimic de facut; numar nou = (motiv, pasiv)
    sm = types.SimpleNamespace(sup=None, _abort_seen_n=0)
    stub = types.SimpleNamespace(abort=threading.Event(), abort_n=3,
                                 abort_reason='x', abort_passive=True)
    ExtNavLanding.attach_supervisor(sm, stub)
    assert sm._abort_seen_n == 3 and ExtNavLanding._abort_asked(sm) is None
    stub.abort.set()
    assert ExtNavLanding._abort_asked(sm) is None, "acelasi numar tratat"
    stub.abort_n = 4
    assert ExtNavLanding._abort_asked(sm) == ('x', True)
    assert ExtNavLanding._abort_asked(sm) is None
    return "abort reaprins intre pasi -> EXIT; contorul decide, nu Event-ul"


def test_PLND_nu_emite_cu_abort_aprins():
    """Calea veche: supervizorul comanda BRAKE singur, prin URGENT; masina
    de stari, cu abort-ul aprins, nu mai trimite LANDING_TARGET. Stins
    (incercare noua), emite din nou. Fara supervizor atasat: neschimbat."""
    v = tsm.StubVehicle(4.0)
    v.mode = MODE_LAND
    sm = LandingStateMachine(v, SequenceConfig(conv=2), verbose=False)
    sm.state = State.DESCEND_TRACK
    det = Detection(t=1000.0, angle_x=0.01, angle_y=-0.02, distance_m=4.0,
                    marker_px=120.0, range_m=4.0, fill=None)
    sm.on_detection(det, 1000.0)
    assert v.n_lt == 1
    sup = SafetySupervisor(v, verbose=False, override=OverrideMonitor(v))
    sm.attach_supervisor(sup)
    sm.on_detection(det, 1000.1)
    assert v.n_lt == 2, "abort stins: emisia neschimbata"
    sup._set_abort('link: legatura pierduta')
    sm.on_detection(det, 1000.2)
    sm.on_detection(det, 1000.3)
    assert v.n_lt == 2, "a emis LANDING_TARGET cu abort aprins"
    assert sm.last_det is det, "detectia tot se inregistreaza (poarta, varsta)"
    sup._clear_abort()
    sm.on_detection(det, 1000.4)
    assert v.n_lt == 3
    return "2 LT inainte, 0 cu abort, 1 dupa stingere"


def test_PasPrincipal_si_oprirea_ordonata():
    """Carligul firului principal publica faza, bate heartbeat-ul, cheama
    watchdog-ul si recorder-ul; oprirea merge supervizor -> detectie ->
    I/O si nu sare un pas cand cel dinainte pica."""
    sm = types.SimpleNamespace(state='DESCEND')
    faza = Latest('IDLE')
    hb = Heartbeat('principal', clock=lambda: 5.0)
    apeluri = []
    st = types.SimpleNamespace(watchdog=lambda now: apeluri.append(('wd', now)))
    rec = types.SimpleNamespace(update=lambda now: apeluri.append(('rec', now)))
    pas = nova_pi.PasPrincipal(sm, faza, hb, supervisor_thread=st, recorder=rec)
    pas(7.0)
    assert faza.get() == ('DESCEND', 7.0) and hb.count == 1 and hb.last == 7.0
    assert apeluri == [('wd', 7.0), ('rec', 7.0)] and pas.n == 1
    # LastDetection il cheama la fiecare poll, dupa detector
    inner = types.SimpleNamespace(poll=lambda now: ['d'])
    ld = nova_pi.LastDetection(inner)
    ld.on_poll = pas
    assert ld.poll(8.0) == ['d'] and faza.get()[1] == 8.0 and hb.count == 2

    ordine = []

    def opreste(nume, ret=None, pica=False):
        def f():
            ordine.append(nume)
            if pica:
                raise RuntimeError('port disparut')
            return ret
        return f

    sup_t = types.SimpleNamespace(stop=opreste('supervizor', ret=[]))
    det = types.SimpleNamespace(stop=opreste('detectie', pica=True))
    veh = types.SimpleNamespace(close=opreste('mav_io'))
    with contextlib.redirect_stdout(io.StringIO()) as out:
        ramase = nova_pi.opreste_firele(sup_t, det, veh)
    assert ordine == ['supervizor', 'detectie', 'mav_io'], ordine
    assert ramase == [] and 'port disparut' in out.getvalue()
    sup_t2 = types.SimpleNamespace(stop=opreste('supervizor', ret=['supervizor']))
    with contextlib.redirect_stdout(io.StringIO()) as out:
        ramase = nova_pi.opreste_firele(sup_t2, det, veh)
    assert ramase == ['supervizor'] and 'inca in viata' in out.getvalue()
    return "faza+heartbeat+watchdog+recorder per poll; oprire in ordine, fara sarituri"


def test_nova_pi_cablajul_pe_fire():
    """Pe SURSA: bucla principala nu mai ruleaza supervizorul, firul lui
    porneste inaintea buclei, heartbeat-urile celor trei fire ajung la
    supervizor, oprirea trece prin opreste_firele, excepthook-ul e instalat."""
    src = open(os.path.join(HERE, 'nova_pi.py')).read()
    i = src.index('def main():')
    m = src[i:]
    assert 'supervisor=None' in m and 'supervisor=sup' not in m
    assert m.index('st.start()') < m.index('run_loop(vehicle, detector, sm')
    assert m.index('Vehicle(a.conn') < m.index('st.start()')
    assert 'opreste_firele(st, detector, vehicle)' in m
    assert 'detector.stop()' not in m and 'vehicle.close()' not in m
    for cerut in ('sup.set_thread_heartbeats(detectie=', 'mav_io=',
                  'principal=hb_principal', 'install_excepthook()',
                  'PasPrincipal(sm, faza, hb_principal',
                  'SupervisorThread(sup, vehicle'):
        assert cerut in m, f"{cerut} lipseste din main()"
    assert 'sm.attach_supervisor(sup)' in src[:i].split('def cablaj_plnd(')[1]
    return "run_loop fara supervizor; st.start() inaintea buclei; oprire ordonata"


TESTS = [
    ('secventa completa: aceleasi mesaje, aceeasi ordine',
     test_secventa_completa_aceleasi_mesaje_aceeasi_ordine),
    ('EXIT cerut de supervizor prin abort, fara callback',
     test_EXIT_supervizor_prin_abort_fara_callback),
    ('abort vechi nu omoara incercarea noua',
     test_abort_vechi_nu_omoara_incercarea_noua),
    ('abort nou intre doi pasi nu se pierde',
     test_abort_nou_intre_doi_pasi_nu_se_pierde),
    ('PLND nu emite cu abort aprins', test_PLND_nu_emite_cu_abort_aprins),
    ('PasPrincipal si oprirea ordonata', test_PasPrincipal_si_oprirea_ordonata),
    ('nova_pi: cablajul pe fire', test_nova_pi_cablajul_pe_fire),
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
