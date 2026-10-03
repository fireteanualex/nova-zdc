#!/usr/bin/env python3
"""
Supervizorul in firul lui (faza 3, refactor/threads).

    st = SupervisorThread(sup, vehicle, detection_latest, phase_latest,
                          miss_streak_fn=lambda: detector.miss_streak)
    st.start()
    ...  # firul principal, la fiecare pas:
    st.watchdog(now)
    ...
    st.stop()

Firul citeste DOAR din instantanee (VehicleView peste Latest[VehicleState],
Latest[Detection], Latest[faza]) si heartbeat-uri; scrie DOAR in coada
URGENT a vehiculului (prin metodele lui `SafetySupervisor`, care cer modul
cu urgent=True) si in `sup.abort`. Monitoarele, pragurile si actiunile sunt
cele de dinainte - se schimba doar calea.

WATCHDOG. Supervizorul e cel care apara vehiculul; daca EL moare, nu mai
apara nimeni. Firul principal il verifica: heartbeat-ul supervizorului
stagnat peste WATCHDOG_MAX_S -> `sup.abort` aprins si actiunea de siguranta
pe care ar fi luat-o supervizorul pusa in URGENT: BRAKE (PLND) sau setul
EKF 1 + LOITER (ExtNav, unde RTL/BRAKE pe setul 2 nu au voie).
"""

import time

from .concurrency import Heartbeat, Latest, run_thread, stop_threads
from .safety import EXTNAV_PHASES, THREAD_HEARTBEAT_MAX_S, ExtNavSupervisor
from .vehicle import MODE_BRAKE, MODE_GUIDED, MODE_LOITER

#: Perioada firului supervizorului. Pana acum rula la fiecare iteratie a
#: buclei principale (sleep 2 ms + lucrul buclei, adica ~100-300 Hz);
#: 200 Hz e in acelasi regim si e o cifra fixa.
SUPERVISOR_PERIOD_S = 0.005
#: Watchdog-ul din firul principal: peste atat fara heartbeat de la
#: supervizor, firul principal ia actiunea de siguranta in locul lui.
WATCHDOG_MAX_S = 0.5


class SupervisorThread:

    def __init__(self, sup, vehicle, detection_latest=None, phase_latest=None,
                 miss_streak_fn=None, period_s=SUPERVISOR_PERIOD_S,
                 clock=time.monotonic):
        self.sup = sup
        self.vehicle = vehicle
        #: Vederea firului asupra vehiculului. Supervizorul TREBUIE construit
        #: peste ea (sup.v is view), altfel ar citi fatada firului principal.
        self.view = sup.v
        self.detection = detection_latest if detection_latest is not None else Latest()
        self.phase = phase_latest if phase_latest is not None else Latest('IDLE')
        self.miss_streak_fn = miss_streak_fn or (lambda: None)
        self.period_s = period_s
        self.clock = clock
        self.heartbeat = Heartbeat('supervizor', clock=clock)
        self.stop_event = None
        self.thread = None
        self.n_steps = 0
        self.watchdog_tripped = False
        self.last_action = None
        #: B1.9 (G2): the watchdog's own previous call (the main thread's
        #: loop), when it last woke from a stall, and the sticky flag (a
        #: new attempt - GATE_SEARCH entered from outside the procedure -
        #: clears it)
        self._wd_last_t = None
        self._wd_wake_t = None
        self.wd_stalled = False
        self._wd_last_phase = 'IDLE'

    # -- firul ---------------------------------------------------------------
    def step(self, now=None):
        """Un pas: instantaneele, apoi update() al supervizorului."""
        now = self.clock() if now is None else now
        if hasattr(self.view, 'refresh'):
            self.view.refresh()
        det, _t = self.detection.get()
        age = None if det is None else max(0.0, now - det.t)
        phase, _ = self.phase.get()
        phase = phase if phase is not None else 'IDLE'
        try:
            streak = self.miss_streak_fn()
        except Exception:                                   # noqa: BLE001
            streak = None
        self.last_action = self.sup.update(now, age, phase, miss_streak=streak)
        self.n_steps += 1
        return self.last_action

    def start(self):
        import threading
        if self.thread is None:
            self.stop_event = threading.Event()
            self.thread = run_thread('supervizor', self.step, self.period_s,
                                     heartbeat=self.heartbeat,
                                     stop_event=self.stop_event)
        return self.thread

    def stop(self, timeout_s=2.0):
        if self.thread is None:
            return []
        ramase = stop_threads([self.thread], self.stop_event, timeout_s)
        self.thread = None
        return ramase

    # -- watchdog, in firul principal ----------------------------------------
    def watchdog(self, now=None, max_s=WATCHDOG_MAX_S):
        """De apelat de firul principal la fiecare pas. True daca a
        intervenit (o singura data): supervizorul nu a mai batut de peste
        `max_s` -> abort aprins si actiunea de siguranta in URGENT."""
        if self.watchdog_tripped or self.thread is None:
            return False
        now = self.clock() if now is None else now
        # B1.9 (G2): the main thread's own gap. If WE were stalled, the
        # supervisor's heartbeat looks old because nothing ran - not
        # because it died. No trip on that: its age counts again only from
        # our waking up, and whatever trips later sends no mode command
        # for this attempt.
        faza, _t = self.phase.get()
        if faza == 'GATE_SEARCH' and self._wd_last_phase not in EXTNAV_PHASES:
            self.wd_stalled = False                  # a new attempt
        self._wd_last_phase = faza
        gap = None if self._wd_last_t is None else now - self._wd_last_t
        self._wd_last_t = now
        if gap is not None and gap > THREAD_HEARTBEAT_MAX_S:
            self.wd_stalled = True
            self._wd_wake_t = now
            return False
        age = self.heartbeat.age(now)
        if age is not None and self._wd_wake_t is not None:
            age = min(age, now - self._wd_wake_t)
        if age is None or age <= max_s:
            return False
        self.watchdog_tripped = True
        # B1 (02.10.2026), deliberate: a ceiling breach while the
        # supervisor thread is dead is a DOUBLE fault (nobody measures the
        # ceiling any more) and is handled as the thread fault - the same
        # SRC1 + LOITER as always, a normal abort (abort_rtl stays False).
        # RTL is not asked from here: it needs the ordered EXIT_RTL path
        # (SRC1 acknowledged, EKF back in the GNSS frame) and nothing here
        # waits for either. The pilot gets the vehicle in LOITER.
        motiv = f"watchdog: supervizorul fara heartbeat de {age:.2f} s"
        # B1.9: ExtNav - the abort is PASSIVE for the state machine: the
        # mode of the hand-back is decided here (LOITER, or none), so the
        # machine only requests SRC1 again and sends no second mode command
        # (it would otherwise follow with its own LOITER, or worse, over a
        # pilot who reacted in between). PLND unchanged.
        self.sup._set_abort(motiv,
                            passive=isinstance(self.sup, ExtNavSupervisor))
        v = self.vehicle
        if isinstance(self.sup, ExtNavSupervisor):
            ok1 = v.send_ekf_source_set(1, urgent=True)
            # B1.9 (G1): LOITER only over the GUIDED we hold, on the main
            # thread's own view (refreshed by pump() every iteration), and
            # never after our own stall. Otherwise SRC1 only - passive.
            mode = getattr(v, 'mode', None)
            if not self.wd_stalled and mode == MODE_GUIDED:
                ok2 = v.request_mode(MODE_LOITER, urgent=True)
                ce = f"SRC1 ({ok1}) + LOITER ({ok2})"
            else:
                de_ce = ('firul principal a fost blocat' if self.wd_stalled
                         else f"FC in modul {mode}, nu GUIDED")
                ce = f"SRC1 ({ok1}), FARA LOITER: {de_ce}"
        else:
            # PLND (not flown): unchanged, BRAKE
            ok = v.request_mode(MODE_BRAKE, urgent=True)
            ce = f"BRAKE ({ok})"
        print(f"!! {motiv}: {ce} in URGENT, din firul principal")
        try:
            self.sup._emit(now, 'watchdog', self.sup.latched, f"{motiv}: {ce}",
                           'WATCHDOG')
        except Exception:                                   # noqa: BLE001
            pass
        return True

    def status(self):
        age = self.heartbeat.age()
        a = '-' if age is None else f"{age * 1000:.0f} ms"
        return f"supervizor: {self.n_steps} pasi, ultima bataie acum {a}"
