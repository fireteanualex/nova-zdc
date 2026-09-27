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
from .safety import ExtNavSupervisor
from .vehicle import MODE_BRAKE, MODE_LOITER

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
        age = self.heartbeat.age(now)
        if age is None or age <= max_s:
            return False
        self.watchdog_tripped = True
        motiv = f"watchdog: supervizorul fara heartbeat de {age:.2f} s"
        self.sup._set_abort(motiv, passive=False)
        v = self.vehicle
        if isinstance(self.sup, ExtNavSupervisor):
            ok1 = v.send_ekf_source_set(1, urgent=True)
            ok2 = v.request_mode(MODE_LOITER, urgent=True)
            ce = f"SRC1 ({ok1}) + LOITER ({ok2})"
        else:
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
