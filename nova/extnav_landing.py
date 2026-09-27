#!/usr/bin/env python3
"""
ExtNav landing state machine (redesign of 27.09.2026,
claude-markdown/REPROIECTARE_EXTNAV.md §3). Flight configuration only; the
PLND machine in nova/state_machine.py stays for the simulator.

    IDLE
     |-AUX8 up-> GATE_SEARCH   pilot in LOITER; we command NOTHING. The gate
     |                         (E0, sticks, 1-12 m) decides once settled;
     |                         ONE estimate in a window of <= 5 s opens the
     |                         segment (team decision 27.09.2026; two
     |                         consistent ones with gate_detections=2);
     |                         none -> one retry -> GATE_FAIL (stays in
     |                         LOITER, STATUSTEXT).
     v
    ENGAGE        1. EKF on SRC2 (no GNSS), read back before switching
                  2. first VISION_POSITION_ESTIMATE right away
                  3. EKF position valid and reset onto our frame
                  4. DO_SET_MODE GUIDED, confirmed from HEARTBEAT
     v
    MOVE          setpoint (0, 0, z_now) + yaw aligned with the marker;
                  out when the EKF says < tol(h) from (0, 0) and still
     v
    CENTER_CHECK  hover window; needs a FRESH detection, not the EKF:
                  offset < tol(h) -> DESCEND (or FINAL_ALIGN at ~1 m);
                  >= tol(h) -> MOVE; none in the window -> one retry -> EXIT
     v
    DESCEND       setpoint (0, 0, -h_new), h_new = max(h/2, 1.0) -> CENTER_CHECK
     v
    FINAL_ALIGN   hover at 1 m, yaw aligned, centred on a fresh detection
                  for FINAL_HOLD_S
     v
    TOUCHDOWN_DESCENT  GUIDED, xy on (0, 0), z ramped down at
                  touchdown_speed to 0.5 m below the ground (LAND is not
                  used any more: LAND disarms on contact, and the rules
                  want the climb back - 15.1.2, 15.2.7)
     v
    CONTACT       ON_GROUND from the FC -> 'contact' event (the scoring
                  capture, 8.3.3), h_ref = baro height now (marker plane)
     v
    GROUND_HOLD   armed, ground idle, >= touchdown_hold_s (15.1.3). The
                  marker does not fit the frame here: the EKF gets its own
                  position frozen at contact as vision (the vehicle does not
                  move on the ground) - measured in SITL 4.5.7, without it the
                  EKF loses the position after 7.0 s, the EKF failsafe lands
                  and disarms, and the real pause was 6.4 s (27.09.2026)
     v
    RISEUP        NAV_TAKEOFF to h_ref + alt_riseup (above HOME, measured),
                  xy held by the EKF; in riseup_timeout_s
     v
    HOVER_CONFIRM hover_confirm_s stable at the target, a FRESH detection
                  with offset < tol(h); one horizontal correction allowed
     v
    COMPLETE      SRC1 restored and acknowledged -> LOITER ->
                  'sequence_complete' -> DONE (the pilot's again)

    EXIT (from any phase after ENGAGE, and on AUX down): SRC1 restored and
    acknowledged FIRST, then LOITER, then ALT_HOLD if LOITER is refused.
    On the ground, a vehicle the FC disarmed gets no command: the event is
    reported (the EKF set still goes back to SRC1 for the next flight).
    Never RTL while on SRC2: home is in another frame after the reset.

THE MODE RULE, in every phase (brief §6, closes the ACQUIRE bug and the
incident class of 26.09): the confirmed mode is remembered; a mode we did
not ask for is the pilot's or a failsafe's -> passive EXIT (SRC1 restored,
no mode command from us); a command is re-sent only while the FC is still
in the mode it had before it.

tol(h) = max(0.15 m, 0.10 h). "Maximum X pixels from the image centre"
became a lateral offset in metres, computed with the attitude at capture
(nova/extnav.py): in hover, 3 degrees at 10 m move the image centre by
~0.5 m on the ground - a pixel threshold would mistake tilt for error.
"""

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil

from . import extnav
from .vehicle import (MODE_ALT_HOLD, MODE_GUIDED, MODE_LAND, MODE_LOITER)

# --- Thresholds (brief §3), named for the Safety Case ------------------------
GATE_WINDOW_S = 5.0          # search window for the gate detection(s)
#: How many detections open the segment. Team decision 27.09.2026, after
#: the flight in which four requests at 3.5 m found no pair: ONE. The
#: consistency check of two (brief D7) was a guard against an outlier;
#: what remains against one is the ID filter (26 only) and CENTER_CHECK,
#: which asks for a FRESH detection before any descent. 2 = the pair rule.
GATE_DETECTIONS = 1
GATE_RETRIES = 1             # extra windows before GATE_FAIL
CENTER_WINDOW_S = 5.0        # hover window for a fresh detection
CENTER_RETRIES = 1
ENGAGE_TIMEOUT_S = 8.0       # SRC2 + EKF valid + reset + GUIDED confirmed
MOVE_TIMEOUT_S = 20.0
DESCEND_TIMEOUT_S = 30.0
FINAL_H_M = 1.0              # the floor of the stepped descent
FINAL_H_TOL_M = 0.30         # "h is about 1 m": same as a reached step
FINAL_HOLD_S = 2.0           # centred and aligned this long before LAND
STILL_SPEED_MS = 0.30        # horizontal speed under which the vehicle "holds"
STILL_VZ_MS = 0.25
ALT_TOL_M = 0.30             # step reached
SETPOINT_PERIOD_S = 0.5      # GUIDED targets re-sent at 2 Hz
MODE_RETRY_S = 0.3
MODE_TRIES = 5
EKF_RESET_TOL_M = 1.0        # EKF position within this of our estimate
EKF_SETTLE_S = 0.5           # after the switch, before judging the reset
EXIT_ACK_TIMEOUT_S = 2.0     # SRC1 acknowledged, else go on regardless
# touchdown and the climb back (27.09.2026)
TOUCHDOWN_SETPOINT_S = 0.1   # the z ramp is re-sent at 10 Hz
TOUCHDOWN_BELOW_M = 0.5      # the ramp ends this far below the ground
GROUND_VPE_S = 0.1           # vision on the ground: 10 Hz, like detections
TAKEOFF_RETRY_S = 3.0        # still on the ground this long -> takeoff again
TAKEOFF_TRIES = 2
HOVER_CONFIRM_TIMEOUT_S = 20.0
THROTTLE_LOW_PWM = 1100      # pilot's throttle at the bottom: disarm risk
YAW_ALIGN_TOL_DEG = 5.0

AUX_CHANNEL = 8
AUX_HIGH_PWM = 1500


class Phase:
    IDLE = 'IDLE'
    GATE_SEARCH = 'GATE_SEARCH'
    REJECT = 'REJECT'
    GATE_FAIL = 'GATE_FAIL'
    ENGAGE = 'ENGAGE'
    MOVE = 'MOVE'
    CENTER_CHECK = 'CENTER_CHECK'
    DESCEND = 'DESCEND'
    FINAL_ALIGN = 'FINAL_ALIGN'
    LAND = 'LAND'
    TOUCHDOWN = 'TOUCHDOWN'
    EXIT = 'EXIT'
    ABORT = 'ABORT'
    DONE = 'DONE'
    TOUCHDOWN_DESCENT = 'TOUCHDOWN_DESCENT'
    CONTACT = 'CONTACT'
    GROUND_HOLD = 'GROUND_HOLD'
    RISEUP = 'RISEUP'
    HOVER_CONFIRM = 'HOVER_CONFIRM'
    COMPLETE = 'COMPLETE'


#: After FINAL_ALIGN: descent to contact, the ground and the climb back.
#: SRC2 until COMPLETE included (15.2.5: no GNSS on the climb either).
TOUCHDOWN_PHASES = (Phase.TOUCHDOWN_DESCENT, Phase.CONTACT, Phase.GROUND_HOLD,
                    Phase.RISEUP, Phase.HOVER_CONFIRM)
#: The part of it on the ground (a disarm there gets no command).
GROUND_PHASES = (Phase.CONTACT, Phase.GROUND_HOLD, Phase.RISEUP)


#: Phases in which the EKF must be on SRC2 (given to EkfSourceManager).
SRC2_PHASES = (Phase.ENGAGE, Phase.MOVE, Phase.CENTER_CHECK, Phase.DESCEND,
               Phase.FINAL_ALIGN, Phase.LAND, Phase.TOUCHDOWN, Phase.EXIT) \
    + TOUCHDOWN_PHASES + (Phase.COMPLETE,)
#: Phases from which AUX down or a supervisor request means EXIT.
ENGAGED_PHASES = (Phase.ENGAGE, Phase.MOVE, Phase.CENTER_CHECK, Phase.DESCEND,
                  Phase.FINAL_ALIGN, Phase.LAND, Phase.TOUCHDOWN) + TOUCHDOWN_PHASES
#: Phases in which estimates are sent to the EKF (D5: from ENGAGE on).
VPE_PHASES = ENGAGED_PHASES + (Phase.EXIT, Phase.COMPLETE)
#: Phases in which we hold a mode of our own and watch it.
GUIDED_PHASES = (Phase.MOVE, Phase.CENTER_CHECK, Phase.DESCEND,
                 Phase.FINAL_ALIGN) + TOUCHDOWN_PHASES


@dataclass
class ExtNavConfig:
    aux_channel: int = AUX_CHANNEL
    aux_high_pwm: int = AUX_HIGH_PWM
    gate_window_s: float = GATE_WINDOW_S
    gate_detections: int = GATE_DETECTIONS
    gate_retries: int = GATE_RETRIES
    center_window_s: float = CENTER_WINDOW_S
    center_retries: int = CENTER_RETRIES
    engage_timeout_s: float = ENGAGE_TIMEOUT_S
    move_timeout_s: float = MOVE_TIMEOUT_S
    descend_timeout_s: float = DESCEND_TIMEOUT_S
    final_h_m: float = FINAL_H_M
    final_hold_s: float = FINAL_HOLD_S
    align_yaw: bool = True
    #: Lateral tolerance tol(h) = max(tol_min_m, tol_frac * h) - MOVE exit,
    #: CENTER_CHECK, FINAL_ALIGN. Defaults = the brief (0.15 m, 0.10 h).
    #: config/nova.json sets 0.10 / 0.0667 (27.09.2026): the flights that
    #: landed well had angles 1.5x too large (calibration scaled for the
    #: wrong sensor mode), so their REAL tolerance was the brief's / 1.5;
    #: with the geometry fixed (62808e9) these values reproduce it.
    tol_min_m: float = extnav.TOL_MIN_M
    tol_frac: float = extnav.TOL_FRAC
    #: touchdown and the climb back (nova.config.touchdown_settings)
    alt_riseup: float = 5.5
    touchdown_speed: float = 0.4
    touchdown_hold_s: float = 1.5
    riseup_timeout_s: float = 15.0
    hover_confirm_s: float = 1.0


class ExtNavLanding:
    """
        sm = ExtNavLanding(vehicle, estimator, ekf, gate, cfg, on_event)
        sm.on_detection(det, now)     # every detection, any phase
        sm.update(now)                # at loop rate
        sm.request_exit(reason)       # the supervisor's only lever

    Events (on_event(name, info)): 'state', 'handover_reject', 'gate_fail',
    'engage', 'scoring_capture' (t = capture time of the detection),
    'touchdown' (t = now), 'exit', 'exit_done', 'aux_ignored_disarmed'.
    """

    def __init__(self, vehicle, estimator, ekf, gate, cfg=None,
                 on_event=None, verbose=True):
        self.v = vehicle
        self.est = estimator
        self.ekf = ekf
        self.gate = gate
        self.cfg = cfg or ExtNavConfig()
        self.on_event = on_event
        self.verbose = verbose

        self.now = time.monotonic()
        self.state = Phase.IDLE
        self.state_since = self.now
        self.was_armed = False
        self._aux_was_high = None       # B7: first reading = reference
        #: Phase 5 (refactor/threads): the supervisor whose `abort` Event
        #: we read every step (attach_supervisor). None = the callback
        #: wiring (`on_exit`), which the simulator and the tests keep.
        self.sup = None
        self._abort_seen_n = 0
        #: Phase 5 (2a): mode requests of the EXIT path go URGENT - the TX
        #: queue is dropped while the supervisor's abort is up.
        self._mode_urgent = False

        self.window = extnav.DetectionWindow(estimator)
        self.window_since = None
        self.window_tries = 0
        self.last_est = None            # newest estimate, any phase
        self.last_marker_yaw = None
        self.last_det_t = None
        self.h_target = None
        self.yaw_target = None
        self.scoring_done = False
        self._final_since = None
        self._setpoint_t = float('-inf')
        self._sub = None                # ENGAGE sub-step
        self._engage_switch_t = None

        # mode handling (the rule of §6)
        self._mode_expected = None      # mode we hold and watch
        self._mode_want = None          # mode being requested
        self._mode_before = None        # FC mode when the request started
        self._mode_req_t = float('-inf')
        self._mode_req_n = 0
        self._mode_stage = 0            # EXIT: 0 LOITER, 1 ALT_HOLD

        self.exit_reason = None
        self.exit_passive = False
        self._exit_step = None          # 'src1', 'mode'
        self._exit_t = None
        self.n_vpe_sent = 0
        self._clear_touchdown()

    # -- events ------------------------------------------------------------
    def _emit(self, name, **info):
        if self.on_event:
            self.on_event(name, info)

    def _say(self, text):
        if self.verbose:
            print(text)

    def set_state(self, new, note=''):
        if new == self.state:
            return
        self._say(f"  >> {self.state} -> {new}" + (f"   ({note})" if note else ''))
        self._emit('state', old=self.state, new=new, note=note)
        self.state = new
        self.state_since = self.now

    def reset(self, note=''):
        self.set_state(Phase.IDLE, note)
        if self.gate is not None:
            self.gate.reset()
        self.window.clear()
        self.window_since = None
        self.window_tries = 0
        self.h_target = None
        self.yaw_target = None
        self.scoring_done = False
        self._final_since = None
        self._sub = None
        self._mode_expected = self._mode_want = self._mode_before = None
        self._mode_req_n = 0
        self._mode_stage = 0
        self.exit_reason = None
        self.exit_passive = False
        self._exit_step = None
        self._clear_touchdown()

    def _clear_touchdown(self):
        self.contact = None             # what CONTACT recorded (dict)
        self._td_z0 = self._td_t0 = self._td_h0 = None
        self._td_setpoint_t = float('-inf')
        self._ground_xy = None
        self._ground_vpe_t = float('-inf')
        self._takeoff_t = None
        self._takeoff_n = 0
        self._riseup_t0 = None
        self._z_riseup = None
        self._confirm_since = None
        self._corrected = False
        self._correcting = False
        self._throttle_warned = False
        self.n_ground_vpe = 0

    # -- detections --------------------------------------------------------
    def on_detection(self, det, now=None):
        now = now if now is not None else time.monotonic()
        self.now = now
        self.last_det_t = det.t
        e = self.est.estimate(det)
        if e is None:
            return
        self.last_est = e
        self.last_marker_yaw = getattr(det, 'marker_yaw_deg', None)
        if self.window_since is not None and e.t >= self.window_since:
            self.window.add(e)
        if self.state in VPE_PHASES:
            if self.est.send(e):
                self.n_vpe_sent += 1

    # -- helpers -----------------------------------------------------------
    def aux_state(self):
        ch = self.cfg.aux_channel
        rc = getattr(self.v, 'rc', None)
        if rc is None or len(rc) < ch:
            return None
        return rc[ch - 1] >= self.cfg.aux_high_pwm

    def aux_high(self):
        return self.aux_state() is True

    def _aux_edges(self):
        aux = self.aux_state()
        if aux is None:
            return False, False, False
        if self._aux_was_high is None:
            self._aux_was_high = aux
            return aux, False, False
        rising = aux and not self._aux_was_high
        falling = self._aux_was_high and not aux
        self._aux_was_high = aux
        return aux, rising, falling

    def h_now(self):
        return -self.v.z

    def tol_now(self):
        return extnav.tol_m(max(self.h_now(), 0.0), self.cfg.tol_min_m,
                            self.cfg.tol_frac)

    def _fresh_est(self, since):
        e = self.last_est
        if e is None or e.t < since:
            return None
        return e

    def _open_window(self, now):
        self.window.clear()
        self.window_since = now
        self.window_tries += 1

    def _send_setpoint(self, now, z, force=False):
        if not force and now - self._setpoint_t < SETPOINT_PERIOD_S:
            return
        self._setpoint_t = now
        yaw = self.yaw_target if self.yaw_target is not None else self.v.yaw
        self.v.send_position_target(0.0, 0.0, z, yaw)

    def _pick_yaw_target(self):
        if not self.cfg.align_yaw:
            self.yaw_target = None
            return
        my = getattr(self, 'last_marker_yaw', None)
        self.yaw_target = extnav.yaw_setpoint(self.v.yaw, my)

    def _statustext(self, text, warn=True, urgent=False):
        sev = (mavutil.mavlink.MAV_SEVERITY_WARNING if warn
               else mavutil.mavlink.MAV_SEVERITY_NOTICE)
        # Prin Vehicle, nu direct pe `mav`: portul e al firului I/O (faza 2).
        fn = getattr(self.v, 'send_statustext', None)
        if fn is None:
            return
        try:
            if urgent:
                try:
                    fn(sev, text[:50], urgent=True)
                    return
                except TypeError:
                    pass
            fn(sev, text[:50])
        except Exception:                                   # noqa: BLE001
            pass

    # -- mode requests (the rule of §6) ------------------------------------
    def _request_mode(self, mode):
        """Through the vehicle; URGENT on the EXIT path (phase 5, 2a)."""
        if self._mode_urgent:
            try:
                return self.v.request_mode(mode, urgent=True)
            except TypeError:                  # test vehicles without `urgent`
                pass
        return self.v.request_mode(mode)

    def _mode_begin(self, now, mode, urgent=False):
        self._mode_want = mode
        self._mode_before = self.v.mode
        self._mode_req_t = now
        self._mode_req_n = 1
        self._mode_urgent = urgent
        self._request_mode(mode)

    def _mode_drive(self, now):
        """'ok' when the FC reports the wanted mode; 'taken' when it is in
        a third mode (pilot / failsafe); 'fail' after MODE_TRIES with the
        FC still in the old mode; None while waiting."""
        if self._mode_want is None:
            return None
        if self.v.mode == self._mode_want:
            self._mode_expected = self._mode_want
            self._mode_want = None
            return 'ok'
        if self.v.mode != self._mode_before:
            self._mode_want = None
            return 'taken'
        if now - self._mode_req_t < MODE_RETRY_S:
            return None
        if self._mode_req_n >= MODE_TRIES:
            self._mode_want = None
            return 'fail'
        self._mode_req_t = now
        self._mode_req_n += 1
        self._request_mode(self._mode_want)
        return None

    def _mode_watch(self):
        """A held mode left without our request: someone else's decision."""
        if self._mode_expected is None or self._mode_want is not None:
            return False
        return self.v.mode != self._mode_expected

    # -- EXIT --------------------------------------------------------------
    def attach_supervisor(self, sup):
        """Phase 5: the supervisor's verdict reaches us through its `abort`
        Event, read HERE, in our own thread, at every step - not through a
        callback that would run the EXIT inside the supervisor's thread."""
        self.sup = sup
        self._abort_seen_n = getattr(sup, 'abort_n', 0)

    def _abort_asked(self):
        """(reason, passive) for an abort not yet acted on, else None.
        Counted, not level-triggered: an abort left over from the previous
        attempt (the supervisor releases it only when ITS thread sees the
        new phase) must not end the attempt the pilot just started."""
        sup = self.sup
        if sup is None or not sup.abort.is_set():
            return None
        n = sup.abort_n
        if n == self._abort_seen_n:
            return None
        self._abort_seen_n = n
        return (sup.abort_reason or 'abort supervizor'), bool(sup.abort_passive)

    def request_exit(self, reason, passive=False):
        """The supervisor's lever, and ours. Idempotent inside EXIT."""
        if self.state in (Phase.EXIT, Phase.ABORT, Phase.IDLE, Phase.DONE,
                          Phase.COMPLETE):
            # COMPLETE is already the ordered hand-back (SRC1, LOITER)
            return
        if self.state not in ENGAGED_PHASES:
            # nothing engaged: back to IDLE, nothing to send
            self.reset(f"iesire inainte de ENGAGE: {reason}")
            return
        self.exit_reason = reason
        self.exit_passive = passive
        self._emit('exit', reason=reason, passive=passive)
        self._say(f"\n!! EXIT: {reason}" + (" (pasiv: fara comenzi de mod)"
                                           if passive else '') + "\n")
        # Everything on the EXIT path rides the URGENT queue (phase 5, 2a):
        # the TX queue is dropped while the supervisor's abort is up, and
        # a supervisor-asked EXIT is exactly that case.
        self._statustext(f"NOVA EXIT: {reason}", urgent=True)
        # 1. SRC1 first, whatever else: home is in another frame on SRC2
        self._exit_step = 'src1'
        self._exit_t = self.now
        self.ekf.release(self.now, reason, urgent=True)
        self._mode_want = None
        self._mode_expected = None
        self._mode_stage = 0
        self.set_state(Phase.EXIT, reason)

    def _handback_done(self, text, mode, passive):
        """End of the hand-back: EXIT -> ABORT ('exit_done'), COMPLETE ->
        DONE ('sequence_complete', said on the OSD and to the FC)."""
        if self.state == Phase.COMPLETE:
            self.set_state(Phase.DONE, text)
            self._emit('sequence_complete', mode=mode, passive=passive,
                       h=self.h_now())
            self._statustext("NOVA SECVENTA COMPLETA", warn=False, urgent=True)
            self._say("\n== SECVENTA COMPLETA: SRC1, predat pilotului ==\n")
        else:
            self.set_state(Phase.ABORT, text)
            self._emit('exit_done', mode=mode, passive=passive)

    def _run_exit(self, now):
        """The ordered hand-back, for EXIT and for COMPLETE alike: SRC1
        acknowledged first, then LOITER, ALT_HOLD if LOITER is refused."""
        if self._exit_step == 'src1':
            ack = getattr(self.v, 'ekf_src_ack', None)
            acked = ack == mavutil.mavlink.MAV_RESULT_ACCEPTED
            if not acked and now - self._exit_t < EXIT_ACK_TIMEOUT_S:
                return
            if not acked:
                self._say("  !! SRC1 fara ACK in timp; continui iesirea")
            if self.exit_passive:
                self._handback_done('EXIT pasiv: SRC1 cerut, modul e al pilotului',
                                    self.v.mode, True)
                return
            self._exit_step = 'mode'
            self._mode_stage = 0
            self._mode_begin(now, MODE_LOITER, urgent=True)
            if self.state == Phase.EXIT:
                self._statustext("NOVA EXIT: LOITER, throttle la mijloc", urgent=True)
            return
        r = self._mode_drive(now)
        if r == 'ok' or r == 'taken':
            what = 'EXIT' if self.state == Phase.EXIT else 'COMPLETE'
            self._handback_done(f"{what}: FC in {self.v.mode_name()}", self.v.mode,
                                r == 'taken')
        elif r == 'fail':
            if self._mode_stage == 0:
                self._mode_stage = 1
                self._say("  !! FC-ul refuza LOITER (fara pozitie?); cer ALT_HOLD")
                self._emit('exit_fallback', mode=MODE_ALT_HOLD)
                self._mode_begin(now, MODE_ALT_HOLD, urgent=True)
            else:
                self._handback_done('nici LOITER, nici ALT_HOLD confirmat',
                                    self.v.mode, False)

    # -- loop --------------------------------------------------------------
    def update(self, now=None):
        now = now if now is not None else time.monotonic()
        self.now = now
        if self.was_armed and not self.v.armed and self.v.have_pos:
            if self.state in GROUND_PHASES:
                # the FC disarmed on the marker (throttle at zero for
                # DISARM_DELAY, a failsafe): no mode command - reported
                self._emit('ground_disarmed', phase=self.state,
                           held_s=None if self.contact is None
                           else now - self.contact['t_on_ground_rx'])
                self._say(f"\n!! DEZARMAT PE SOL in {self.state}: fara comenzi\n")
            if self.state in ENGAGED_PHASES or self.state == Phase.EXIT:
                # on the ground, or the pilot disarmed: the EKF set must
                # not stay on SRC2 for the next flight
                self.ekf.release(now, 'dezarmat')
            self.reset('dezarmat')
        self.was_armed = self.v.armed
        if not self.v.have_pos:
            return
        aux_high, aux_rising, aux_falling = self._aux_edges()

        if not self.v.armed:
            if self.state != Phase.IDLE:
                self.reset('dezarmat')
            if aux_rising:
                self._say("  !! AUX sus IGNORAT: vehiculul e dezarmat.")
                self._emit('aux_ignored_disarmed')
            return

        # AUX down = the pilot asks out, from anywhere after the gate
        if aux_falling and self.state in ENGAGED_PHASES:
            self.request_exit('AUX jos: iesire ceruta de pilot')
        elif aux_falling and self.state in (Phase.GATE_SEARCH,):
            self.reset('AUX eliberat in cautare')
            return

        # Phase 5: the supervisor's abort, read every step. Same lever as
        # the old on_exit callback (request_exit), same order (SRC1, LOITER,
        # ALT_HOLD) - only the thread that pulls it changes.
        ab = self._abort_asked()
        if ab is not None:
            self.request_exit(ab[0], passive=ab[1])

        st = self.state
        if st in (Phase.EXIT, Phase.COMPLETE):
            self._run_exit(now)
            return
        if st in (Phase.ABORT, Phase.GATE_FAIL, Phase.DONE):
            if aux_rising:
                self.reset('cerere noua')
                self._start_search(now)
            return
        if st == Phase.REJECT:
            if not aux_high:
                self.reset('AUX eliberat dupa refuz')
            return
        if st == Phase.IDLE:
            if aux_rising:
                self._start_search(now)
            return

        # the mode rule, wherever we hold a mode
        if st in GUIDED_PHASES + (Phase.LAND, Phase.TOUCHDOWN) and self._mode_watch():
            self.request_exit(f"mod schimbat de altcineva ({self.v.mode_name()})",
                              passive=True)
            return

        if st == Phase.GATE_SEARCH:
            self._run_gate_search(now)
        elif st == Phase.ENGAGE:
            self._run_engage(now)
        elif st == Phase.MOVE:
            self._run_move(now)
        elif st == Phase.CENTER_CHECK:
            self._run_center_check(now)
        elif st == Phase.DESCEND:
            self._run_descend(now)
        elif st == Phase.FINAL_ALIGN:
            self._run_final_align(now)
        elif st == Phase.LAND:
            self._run_land(now)
        elif st == Phase.TOUCHDOWN:
            pass
        elif st == Phase.TOUCHDOWN_DESCENT:
            self._run_touchdown_descent(now)
        elif st == Phase.CONTACT:
            self.set_state(Phase.GROUND_HOLD, 'armat pe sol')
        elif st == Phase.GROUND_HOLD:
            self._run_ground_hold(now)
        elif st == Phase.RISEUP:
            self._run_riseup(now)
        elif st == Phase.HOVER_CONFIRM:
            self._run_hover_confirm(now)
        # The EKF set manager arms and releases from the phase (§5.14); the
        # machine owns it, so the shared run_loop needs no new hook.
        self.ekf.update(now, self.state)

    # -- phases ------------------------------------------------------------
    def _start_search(self, now):
        if self.gate is None:
            self.reset('fara poarta')
            return
        self.gate.on_aux_requested(now)
        self.window_tries = 0
        self._open_window(now)
        self.set_state(Phase.GATE_SEARCH, f"AUX sus, alt {self.h_now():.2f} m")

    def _run_gate_search(self, now):
        ok, why = self.gate.check(now, None, None)
        if ok is False:
            self._emit('handover_reject', reason=why, alt=self.h_now())
            self.set_state(Phase.REJECT, why)
            return
        found = self._gate_found()
        if ok is True and found is not None:
            e = found[-1]
            self._emit('engage', alt=self.h_now(), n=len(found),
                       pair_dt=(found[-1].t - found[0].t) if len(found) > 1 else None,
                       lateral_m=e.lateral_m)
            ce = ('o detectie' if len(found) == 1
                  else f"{len(found)} detectii consistente")
            self._say(f"  == {ce} ({e.lateral_m:.2f} m lateral): ENGAGE")
            self._sub = 'src2'
            self.set_state(Phase.ENGAGE, f"alt {self.h_now():.2f} m")
            return
        if now - self.window_since >= self.cfg.gate_window_s:
            if self.window_tries <= self.cfg.gate_retries:
                self._say(f"  .. fereastra {self.window_tries}: {len(self.window)} "
                          f"estimari, {self._gate_need()}; reincerc")
                self._open_window(now)
                return
            self._emit('gate_fail', n=len(self.window), alt=self.h_now())
            self._statustext("NOVA: marker negasit, raman in LOITER")
            self.set_state(Phase.GATE_FAIL, self._gate_need())

    def _gate_found(self):
        """The estimates that open the segment, oldest first, or None.
        One (the newest) by default; with gate_detections >= 2, the newest
        and an earlier one it agrees with (DetectionWindow.pair)."""
        if self.cfg.gate_detections <= 1:
            return (self.window.items[-1],) if len(self.window) else None
        return self.window.pair()

    def _gate_need(self):
        return ('nicio detectie' if self.cfg.gate_detections <= 1
                else 'fara pereche consistenta')

    def _run_engage(self, now):
        if now - self.state_since > self.cfg.engage_timeout_s:
            self.request_exit(f"ENGAGE fara succes in {self.cfg.engage_timeout_s:.0f} s "
                              f"(pas {self._sub})")
            return
        if self._sub == 'src2':
            # EkfSourceManager reads the set back and switches from phase
            self.ekf.update(now, self.state)
            if self.ekf.state == self.ekf.REFUZAT:
                self.request_exit(f"setul EKF refuzat: {'; '.join(self.ekf.motive_refuz)}")
                return
            if self.ekf.state == self.ekf.ACTIV and self.ekf.ack_ok:
                self._engage_switch_t = now
                # 2. first VPE right away, from the newest estimate
                if self.last_est is not None:
                    if self.est.send(self.last_est):
                        self.n_vpe_sent += 1
                self._sub = 'ekf'
            return
        if self._sub == 'ekf':
            # 3. valid position, reset observed onto our frame
            if now - self._engage_switch_t < EKF_SETTLE_S:
                return
            ok = getattr(self.v, 'ekf_pos_horiz_ok', lambda: None)()
            e = self.last_est
            if ok and e is not None:
                d = math.hypot(self.v.x - e.x, self.v.y - e.y)
                if d <= EKF_RESET_TOL_M:
                    self._sub = 'mode'
                    self._mode_begin(now, MODE_GUIDED)
            return
        if self._sub == 'mode':
            r = self._mode_drive(now)
            if r == 'ok':
                self._pick_yaw_target()
                self.h_target = self.h_now()
                self._send_setpoint(now, -self.h_target, force=True)
                self.set_state(Phase.MOVE, f"GUIDED confirmat, h {self.h_target:.2f} m")
            elif r == 'taken':
                self.request_exit(f"mod pus de altcineva in ENGAGE ({self.v.mode_name()})",
                                  passive=True)
            elif r == 'fail':
                self.request_exit('FC-ul nu a confirmat GUIDED')

    def _run_move(self, now):
        self._send_setpoint(now, -self.h_target)
        if now - self.state_since > self.cfg.move_timeout_s:
            self.request_exit(f"MOVE fara convergenta in {self.cfg.move_timeout_s:.0f} s")
            return
        off = math.hypot(self.v.x, self.v.y)
        speed = math.hypot(getattr(self.v, 'vx', 0.0), getattr(self.v, 'vy', 0.0))
        if off < self.tol_now() and speed < STILL_SPEED_MS:
            self.window_tries = 0
            self._open_window(now)
            self.set_state(Phase.CENTER_CHECK, f"EKF la {off:.2f} m de (0,0)")

    def _run_center_check(self, now):
        self._send_setpoint(now, -self.h_target)
        e = self._fresh_est(self.window_since)
        if e is not None:
            tol = self.tol_now()
            if e.lateral_m < tol:
                if abs(self.h_now() - self.cfg.final_h_m) <= FINAL_H_TOL_M:
                    self._pick_yaw_target()
                    self._final_since = None
                    self.set_state(Phase.FINAL_ALIGN,
                                   f"centrat ({e.lateral_m:.2f} m) la {self.h_now():.2f} m")
                else:
                    # the ladder halves the TARGET, not the measured h: from
                    # 12 m exactly 12 -> 6 -> 3 -> 1.5 -> 1 (brief §3), no
                    # residual-driven extra step at the bottom
                    base = self.h_target if self.h_target is not None else self.h_now()
                    self.h_target = max(base / 2.0, self.cfg.final_h_m)
                    self._send_setpoint(now, -self.h_target, force=True)
                    self.set_state(Phase.DESCEND,
                                   f"centrat ({e.lateral_m:.2f} m < {tol:.2f}); "
                                   f"cobor la {self.h_target:.2f} m")
            else:
                self._pick_yaw_target()
                self.set_state(Phase.MOVE, f"offset masurat {e.lateral_m:.2f} m >= {tol:.2f}")
            return
        if now - self.window_since >= self.cfg.center_window_s:
            if self.window_tries <= self.cfg.center_retries:
                self._say(f"  .. fereastra de centrare {self.window_tries} fara detectie; reincerc")
                self._open_window(now)
                return
            self.request_exit('nicio detectie in fereastra de centrare')

    def _run_descend(self, now):
        self._send_setpoint(now, -self.h_target)
        if now - self.state_since > self.cfg.descend_timeout_s:
            self.request_exit(f"DESCEND fara a atinge {self.h_target:.2f} m")
            return
        if (abs(self.h_now() - self.h_target) <= ALT_TOL_M
                and abs(self.v.vz) < STILL_VZ_MS):
            self.window_tries = 0
            self._open_window(now)
            self.set_state(Phase.CENTER_CHECK, f"la {self.h_now():.2f} m")

    def _run_final_align(self, now):
        self.h_target = self.cfg.final_h_m
        self._send_setpoint(now, -self.h_target)
        if now - self.state_since > self.cfg.move_timeout_s:
            self.request_exit('FINAL_ALIGN fara stabilizare')
            return
        e = self._fresh_est(self.state_since)
        centred = e is not None and e.lateral_m < self.tol_now()
        aligned = True
        if self.cfg.align_yaw and self.yaw_target is not None:
            aligned = abs(math.degrees(extnav.wrap_pi(self.v.yaw - self.yaw_target))) \
                <= YAW_ALIGN_TOL_DEG
        if not (centred and aligned):
            self._final_since = None
            return
        if self._final_since is None:
            self._final_since = now
            return
        if now - self._final_since < self.cfg.final_hold_s:
            return
        # 27.09.2026: no LAND (it disarms on contact); the capture for
        # 8.3.3 is taken at CONTACT, on the ground, not here at 1 m
        self._td_z0 = self.v.z
        self._td_h0 = self.h_now()
        self._td_t0 = now
        self._td_setpoint_t = float('-inf')
        self.set_state(Phase.TOUCHDOWN_DESCENT,
                       f"centrat {e.lateral_m:.2f} m, {self.cfg.final_hold_s:.0f} s; "
                       f"cobor cu {self.cfg.touchdown_speed:.2f} m/s")

    def _run_land(self, now):
        r = self._mode_drive(now)
        if r == 'taken':
            self.request_exit(f"mod pus de altcineva la LAND ({self.v.mode_name()})",
                              passive=True)
            return
        if r == 'fail':
            self.request_exit('FC-ul nu a confirmat LAND')
            return
        if self.v.on_ground():
            self._emit('touchdown', alt=self.h_now(), t=now)
            self.set_state(Phase.TOUCHDOWN, 'ON_GROUND; FC-ul dezarmeaza')
            # the EKF set goes back at disarm (update: reset) - here the
            # vehicle is on the marker, in the marker frame; nothing to do

    # -- touchdown and the climb back (27.09.2026) -------------------------
    def touchdown_timeout_s(self):
        """Descent from FINAL_ALIGN to contact: twice the time at
        touchdown_speed, plus the FC's land detection (~1.5 s, SITL)."""
        h0 = self._td_h0 if self._td_h0 is not None else self.cfg.final_h_m
        return 2.0 * (h0 + TOUCHDOWN_BELOW_M) / self.cfg.touchdown_speed + 5.0

    def _run_touchdown_descent(self, now):
        if self.v.on_ground():
            self._contact(now)
            return
        if now - self.state_since > self.touchdown_timeout_s():
            self.request_exit(f"fara contact in {self.touchdown_timeout_s():.0f} s")
            return
        if now - self._td_setpoint_t >= TOUCHDOWN_SETPOINT_S:
            self._td_setpoint_t = now
            # a z ramp at touchdown_speed, down to 0.5 m below the ground:
            # the position controller keeps xy on (0, 0) and the land
            # detector needs the throttle at its lower limit to see contact
            floor = self._td_z0 + self._td_h0 + TOUCHDOWN_BELOW_M
            z = min(self._td_z0 + self.cfg.touchdown_speed * (now - self._td_t0), floor)
            yaw = self.yaw_target if self.yaw_target is not None else self.v.yaw
            self.v.send_position_target(0.0, 0.0, z, yaw)

    def _contact(self, now):
        """ON_GROUND from the FC: what the capture and the climb need,
        recorded once, then the 'contact' event (the 8.3.3 capture)."""
        e = self.last_est
        last_t = self.last_det_t
        self.contact = {
            't_on_ground_rx': now,
            'fc_time_boot_ms': getattr(self.v, 'time_boot_ms', None),
            'fc_time_unix_usec': (self.v.fc_unix_usec_now()
                                  if hasattr(self.v, 'fc_unix_usec_now') else None),
            'h_ref': float(getattr(self.v, 'rel_alt', None) or 0.0),
            'z_contact': self.v.z,
            'last_det_age_s': None if last_t is None else now - last_t,
            'last_lateral_m': None if e is None else e.lateral_m,
        }
        self._ground_xy = (self.v.x, self.v.y)
        self._ground_vpe_t = float('-inf')
        self._emit('contact', **self.contact)
        self.set_state(Phase.CONTACT, f"ON_GROUND, h_ref {self.contact['h_ref']:.2f} m")

    def _ground_vpe(self, now):
        """Vision on the ground (team decision 27.09.2026, after phase A):
        the marker does not fit the frame below ~0.6 m, and without vision
        the EKF drops the position after 7.0 s (SITL 4.5.7) - the failsafe
        then lands and disarms on the marker. While the FC says ON_GROUND
        the vehicle does not move, so its position frozen at contact IS
        the truth: sent as VISION_POSITION_ESTIMATE, only while on the
        ground, stopped the moment it leaves it."""
        if self._ground_xy is None or not self.v.on_ground():
            return
        if now - self._ground_vpe_t < GROUND_VPE_S:
            return
        self._ground_vpe_t = now
        x, y = self._ground_xy
        if self.v.send_vision_position_estimate(
                int(round(now * 1e6)), x, y, self.v.z, self.v.roll,
                self.v.pitch, self.v.yaw):
            self.n_ground_vpe += 1

    def _run_ground_hold(self, now):
        self._ground_vpe(now)
        rc = getattr(self.v, 'rc', None)
        if (not self._throttle_warned and rc is not None and len(rc) >= 3
                and 0 < rc[2] < THROTTLE_LOW_PWM):
            self._throttle_warned = True
            self._emit('throttle_low_ground', pwm=rc[2])
            self._statustext("NOVA: throttle la minim pe sol - risc dezarmare")
        held = now - self.contact['t_on_ground_rx']
        if held < self.cfg.touchdown_hold_s:
            return
        self._takeoff(now)
        self._riseup_t0 = now
        self.set_state(Phase.RISEUP, f"{held:.1f} s pe sol; urc la h_ref + "
                                     f"{self.cfg.alt_riseup:.1f} m")

    def _takeoff(self, now):
        target = self.contact['h_ref'] + self.cfg.alt_riseup
        # NAV_TAKEOFF altitude is ABOVE HOME on 4.5.7 (measured: home 2 m
        # below the ground, takeoff 5 m -> 2.9 m above the ground), and
        # h_ref is the baro height above home at contact
        self.v.send_takeoff(target)
        self._takeoff_t = now
        self._takeoff_n += 1
        self._z_riseup = self.contact['z_contact'] - self.cfg.alt_riseup
        self._emit('riseup', target_above_home=target, n=self._takeoff_n)

    def _run_riseup(self, now):
        self._ground_vpe(now)          # until the vehicle leaves the ground
        if now - self._riseup_t0 > self.cfg.riseup_timeout_s:
            self.request_exit(f"urcarea nu a ajuns in {self.cfg.riseup_timeout_s:.0f} s")
            return
        if self.v.on_ground():
            if now - self._takeoff_t >= TAKEOFF_RETRY_S:
                if self._takeoff_n >= TAKEOFF_TRIES:
                    self.request_exit('decolarea nu a pornit')
                    return
                self._takeoff(now)
            return
        h_t = -self._z_riseup
        if (self.h_now() >= h_t - ALT_TOL_M):
            self.h_target = h_t
            self._send_setpoint(now, self._z_riseup, force=True)
            self._confirm_since = None
            self._corrected = self._correcting = False
            self.window_tries = 0
            self._open_window(now)
            self.set_state(Phase.HOVER_CONFIRM, f"la {self.h_now():.2f} m")

    def _run_hover_confirm(self, now):
        self._send_setpoint(now, self._z_riseup)
        if now - self.state_since > HOVER_CONFIRM_TIMEOUT_S:
            self.request_exit('HOVER_CONFIRM fara confirmare')
            return
        stable = (abs(self.h_now() - self.h_target) <= ALT_TOL_M
                  and abs(self.v.vz) < STILL_VZ_MS
                  and math.hypot(getattr(self.v, 'vx', 0.0),
                                 getattr(self.v, 'vy', 0.0)) < STILL_SPEED_MS)
        tol = self.tol_now()
        if self._correcting:
            # one correction: the fresh detection moved the EKF onto the
            # marker frame again; holding (0, 0) brings the vehicle over it
            if stable and math.hypot(self.v.x, self.v.y) < tol:
                self._correcting = False
                self._confirm_since = None
                self._open_window(now)
            return
        e = self._fresh_est(self.window_since)
        if e is not None and e.lateral_m >= tol:
            if self._corrected:
                self.request_exit(f"nu e deasupra markerului dupa corectie "
                                  f"({e.lateral_m:.2f} m >= {tol:.2f})")
                return
            self._corrected = self._correcting = True
            self._emit('hover_correction', lateral_m=e.lateral_m, tol=tol)
            return
        if e is not None and stable:
            if self._confirm_since is None:
                self._confirm_since = now
            if now - self._confirm_since >= self.cfg.hover_confirm_s:
                self._begin_complete(now, e)
            return
        self._confirm_since = None
        if e is None and now - self.window_since >= self.cfg.center_window_s:
            if self.window_tries <= self.cfg.center_retries:
                self._open_window(now)
                return
            self.request_exit('nicio detectie la altitudinea de predare')

    def _begin_complete(self, now, e):
        self._emit('hover_confirmed', lateral_m=e.lateral_m, h=self.h_now())
        self._exit_step = 'src1'
        self._exit_t = now
        self.exit_passive = False
        self.ekf.release(now, 'secventa completa', urgent=True)
        self._mode_want = None
        self._mode_expected = None
        self._mode_stage = 0
        self.set_state(Phase.COMPLETE, f"deasupra markerului ({e.lateral_m:.2f} m) "
                                       f"la {self.h_now():.2f} m")

    # -- reporting ---------------------------------------------------------
    def status_line(self):
        e = self.last_est
        fresh = e is not None and (self.now - e.t) < 1.0
        vis = (f"lateral {e.lateral_m:5.2f} m (t-{self.now - e.t:.1f}s)"
               if fresh else 'FARA ESTIMARE RECENTA')
        ch = self.cfg.aux_channel
        rc = getattr(self.v, 'rc', None)
        aux = (f"AUX{ch} {rc[ch - 1]:4d} {'SUS' if self.aux_high() else 'jos'}"
               if rc is not None and len(rc) >= ch else f"AUX{ch} -")
        ht = '-' if self.h_target is None else f"{self.h_target:.2f}"
        return (f"[{self.state:<12}] h {self.h_now():5.2f} m -> {ht} | {vis} | "
                f"EKF ({self.v.x:+.2f},{self.v.y:+.2f}) | {aux} | "
                f"VPE {self.n_vpe_sent} SP {getattr(self.v, 'n_pos_target', 0)}")
