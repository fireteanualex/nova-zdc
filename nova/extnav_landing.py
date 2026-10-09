#!/usr/bin/env python3
"""
ExtNav landing state machine (redesign of 27.09.2026,
claude-markdown/REPROIECTARE_EXTNAV.md §3). Flight configuration only; the
PLND machine in nova/state_machine.py stays for the simulator.

    IDLE
     |-AUX8 up-> GATE_SEARCH   pilot in LOITER; we command NOTHING. The gate
     |                         (E0, sticks, the configured window - 4-20 m
     |                         by default) decides once settled;
     |                         ONE estimate in a window of <= 5 s opens the
     |                         segment (team decision 27.09.2026; two
     |                         consistent ones with gate_detections=2);
     |                         none -> one retry -> GATE_FAIL (stays in
     |                         LOITER, STATUSTEXT) - or YAW_SEARCH (B3).
     |
     |-(none found, gate ok, yaw_search on / 'auto' and h < h_cover)->
    YAW_SEARCH    B3 (04.10.2026), GNSS-free from its first command:
                  1. 'src2'   EKF on SRC2 (read back, ATTRIBUTED ack), the
                              time of the command = t_switch
                  2. 'mode'   GUIDED, confirmed from HEARTBEAT
                  3. 'rotate' ONE CONDITION_YAW, relative, counter-
                              clockwise, yaw_search_max_deg at
                              yaw_search_rate_dps; NO position target
                  4. 'found'  first detection -> VPE at once (the EKF
                              resets onto the marker frame); the reset
                              observed -> ONE position target at the EKF
                              position (new frame), current yaw: stops it
                  5. 'confirm' the gate criterion (gate_detections) within
                              YAW_SEARCH_CONFIRM_S of the first detection
                  -> ENGAGE (switch and mode already done: verified only)
                  Nothing found by t_switch + SEARCH_BUDGET_S (5.0 s), or
                  the rotation done without a detection -> EXIT, long
                  before the EKF drops the position (7.0 s, measured).
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

    EXIT_RTL (B1, 02.10.2026, rule 15.2.4: the supervisor's 30 m ceiling
    above home, from GATE_SEARCH to the hand-back), every step URGENT:
      1. SET_EKF_SOURCE_SET(1) sent explicitly from EVERY phase (also from
         GATE_SEARCH, where set 1 is already active: idempotent, one path),
         the EkfSourceManager let go; re-sent every 0.5 s until an ACCEPTED
         attributed to THIS request (no stale ack). None in 2.0 s -> LAND,
         never RTL: RTL on set 2 flies to the marker frame's "home".
      2. frame gate: GLOBAL_POSITION_INT within 3 m of GPS_RAW_INT, both
         received after the ack (the EKF position reset is the only proof
         the EKF is back in the GNSS frame; no STATUSTEXT is ever waited
         for). 1.0 s at most, then RTL anyway (measured: still reaches
         home, without the RTL_ALT_M climb).
      3. RTL, confirmed from HEARTBEAT; refused -> LAND; LAND refused ->
         reported. A third mode at any point -> passive, nothing more.
    After the hand-back (ABORT) nothing is re-sent, whatever the FC does.

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
from .safety import THREAD_HEARTBEAT_MAX_S
from .vehicle import (MODE_ALT_HOLD, MODE_GUIDED, MODE_LAND, MODE_LOITER,
                      MODE_RTL)

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
# EXIT_RTL (B1, 02.10.2026; measured in SITL 4.5.7, fazaA1/RESULTS.md):
# ack 2-12 ms after SET_EKF_SOURCE_SET(1), EKF position reset 0.17-0.19 s
EXIT_RTL_SRC1_RESEND_S = 0.5  # SRC1 re-sent this often until acknowledged
#: Horizontal distance GLOBAL_POSITION_INT (EKF) - GPS_RAW_INT (receiver)
#: under which the EKF is back in the GNSS frame. Real GPS noise; in SITL
#: the two agree within ~1 m right after the reset (14.5 m before it).
FRAME_TOL_M = 3.0
#: The gate is SHORT on purpose: between the reset and RTL, GUIDED chases
#: its old target (0, 0), which after the reset is the EKF origin (run1:
#: 3.35 m drift in 3 s). Measured reset at 0.18 s; on timeout RTL anyway.
FRAME_TIMEOUT_S = 1.0
#: B1.9 (G2, 02.10.2026, SITL campaign S rows F6c / F2 / F2c): a gap between
#: two update() calls above this means the main loop (or the whole
#: process) was stalled - its picture of the vehicle is stale, and so may
#: be the first bytes read after it (old messages in the serial buffer).
#: From then on NO mode command for this attempt: every exit is passive
#: (SRC1 still requested, urgent) until a new attempt through the gate.
#: The same limit as the thread heartbeats the supervisor watches.
STALL_MAX_S = THREAD_HEARTBEAT_MAX_S
# touchdown and the climb back (27.09.2026)
TOUCHDOWN_SETPOINT_S = 0.1   # the z ramp is re-sent at 10 Hz
#: The ramp ends this far below the estimated ground. Was 0.5 m (enough in
#: SITL, perfect baro); flight 09.10.2026 b121: baro read -0.4..-0.5 m on
#: the ground, the target was reached, the throttle never went to its lower
#: limit and the FC never declared ON_GROUND. Deeper = the vehicle keeps
#: descending at touchdown_speed until the FC sees the contact.
TOUCHDOWN_BELOW_M = 1.5
#: In TOUCHDOWN_DESCENT, no detection for this long = the camera is blind
#: (the marker overflows the frame below ~0.6 m): from then on the EKF gets
#: the position frozen at that moment, at GROUND_VPE_S, like on the ground.
#: Without it the EKF drops the position 7 s after the last detection
#: (flight 09.10.2026 b121: EXIT just above the marker, then LOITER on GPS).
TD_BLIND_S = 0.3
GROUND_VPE_S = 0.1           # vision on the ground: 10 Hz, like detections
TAKEOFF_RETRY_S = 3.0        # still on the ground this long -> takeoff again
TAKEOFF_TRIES = 2
HOVER_CONFIRM_TIMEOUT_S = 30.0
#: The one correction in HOVER_CONFIRM ends when the EKF is on (0, 0),
#: stable AND agrees with a fresh detection within half the tolerance - the
#: EKF absorbs vision slowly (VISO_POS_M_NSE 0.5 m, ~3 detections/s): in
#: SITL 4.5.7 (27.09.2026) it said "there" while the camera still saw
#: 0.37 m. Not converged in this long -> EXIT.
HOVER_CORRECTION_TIMEOUT_S = 10.0
THROTTLE_LOW_PWM = 1100      # pilot's throttle at the bottom: disarm risk
YAW_ALIGN_TOL_DEG = 5.0

# --- B3: the marker search by rotation (04.10.2026) ---------------------------
# Measured in SITL Copter-4.5.7, phases A2 / A2b, in the real scenario (no
# VISION_POSITION_ESTIMATE before the switch, a frame jump of ~38 m at the
# first detection):
#: On EKF source set 2 (EK3_SRC2_POSXY 6, no GNSS) with no vision sample,
#: the EKF keeps a valid horizontal position for 7.0 s counted from the LAST
#: FUSED POSITION - the last GPS fusion, i.e. the SRC2 switch - then loses
#: it (posRetryTimeNoVel_ms = 7000, AP_NavEKF3.h:466: a constant, not a
#: parameter); ~0.9 s later the EKF failsafe lands. The variance gives NO
#: early warning. Every deadline of YAW_SEARCH is derived from this number.
EKF_NO_VISION_VALID_S = 7.0
#: The ack of SET_EKF_SOURCE_SET(2) + GUIDED confirmed from HEARTBEAT,
#: budgeted for the serial link (measured 0.04 s on TCP: ack 0.01 s,
#: GUIDED accepted on set 2 without any ExtNav sample in 0.03 s).
SWITCH_OVERHEAD_S = 0.5
#: Stopping the rotation with a position target: 0.30 s measured.
STOP_MARGIN_S = 0.3
#: HARD deadline, from the SRC2 command, for the first accepted detection.
#: SWITCH_OVERHEAD_S + max_deg / rate + STOP_MARGIN_S must fit in it (a
#: startup check, nova.config.yaw_search_settings). 2.0 s below the EKF's
#: 7.0 s: the first VPE must reach the FC before ~6.8 s (a VPE after the
#: position loss resets dirty: a 1.5 m lurch, measured).
SEARCH_BUDGET_S = 5.0
#: After the first detection, the gate criterion (gate_detections) must be
#: met within this. The 7 s rule now counts from the LAST fused VPE, and
#: one goes out with every detection, so 4 s stays inside it.
YAW_SEARCH_CONFIRM_S = 4.0
#: The rotation counts as done this close to yaw_search_max_deg (the FC
#: overshoots the target by ~2.4 deg, it never undershoots by 5).
YAW_SEARCH_DONE_TOL_DEG = 5.0
#: First detection -> the reset must be seen in LOCAL_POSITION_NED within
#: EKF_SETTLE_S + this (measured 14-54 ms) ...
YAW_SEARCH_RESET_WAIT_S = 1.0
#: ... and in any case before this, from the switch: if the first VPE was
#: not fused, the EKF still counts from the switch and drops the position at
#: EKF_NO_VISION_VALID_S - the exit (SRC1, ms) must come well before.
YAW_SEARCH_RESET_CAP_S = EKF_NO_VISION_VALID_S - 1.0
#: The team's limits (startup refused above): 180 deg (more gains nothing -
#: 90 already sweeps the long axis through every direction), 60 deg/s (the
#: FC's ATC_RATE_WPY_MAX 60 deg/s on 4.7.1). Above 30 deg/s the detection is untested.
YAW_SEARCH_MAX_DEG_LIMIT = 180.0
YAW_SEARCH_RATE_LIMIT_DPS = 60.0
YAW_SEARCH_RATE_TESTED_DPS = 30.0
#: Rule 15.2.3's zone: the marker lies within this of the handover point.
SEARCH_RADIUS_M = 6.5
#: NOT TESTED ON THE REAL CAMERA: detection while yawing at these rates.
#: The exposure is locked at <= 2000 us (crop1280), so at 25 deg/s the
#: motion blur is under one pixel - fine on paper, unmeasured in flight.


def yaw_search_h_cover_m(search_radius_m, hfov_deg, vfov_deg):
    """The height from which the camera's SHORT half-axis alone covers the
    search radius: r / tan(min(HFOV, VFOV) / 2). Below it a marker inside
    the radius may be outside the frame - the rotation is worth doing; at
    or above it, it is not. min() so the camera's mounting rotation does
    not matter. From the EFFECTIVE calibration of the running preset
    (crop1280: ~73 x 45 deg -> ~15.7 m for 6.5 m). None if unknown."""
    if hfov_deg is None or vfov_deg is None or search_radius_m is None:
        return None
    short = min(float(hfov_deg), float(vfov_deg))
    if not 0.0 < short < 180.0:
        return None
    return float(search_radius_m) / math.tan(math.radians(short) / 2.0)


def yaw_search_duration_s(max_deg, rate_dps):
    """The startup budget's left side, s, counted from the SRC2 command:
    SWITCH_OVERHEAD_S + max_deg / rate + STOP_MARGIN_S."""
    return SWITCH_OVERHEAD_S + float(max_deg) / float(rate_dps) + STOP_MARGIN_S

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
    #: B1: the ceiling's exit - SRC1 (acknowledged), frame gate, RTL
    EXIT_RTL = 'EXIT_RTL'
    #: B3: the marker search by rotation, GNSS-free (SRC2, GUIDED)
    YAW_SEARCH = 'YAW_SEARCH'


#: After FINAL_ALIGN: descent to contact, the ground and the climb back.
#: SRC2 until COMPLETE included (15.2.5: no GNSS on the climb either).
TOUCHDOWN_PHASES = (Phase.TOUCHDOWN_DESCENT, Phase.CONTACT, Phase.GROUND_HOLD,
                    Phase.RISEUP, Phase.HOVER_CONFIRM)
#: The part of it on the ground (a disarm there gets no command).
GROUND_PHASES = (Phase.CONTACT, Phase.GROUND_HOLD, Phase.RISEUP)


#: Phases in which the EKF must be on SRC2 (given to EkfSourceManager).
#: B3: YAW_SEARCH too - the manager is armed from the phase and switches
#: there; going on to ENGAGE (same set, already ACTIV) it does not switch
#: again: one SET_EKF_SOURCE_SET(2) per attempt.
SRC2_PHASES = (Phase.YAW_SEARCH, Phase.ENGAGE, Phase.MOVE, Phase.CENTER_CHECK,
               Phase.DESCEND, Phase.FINAL_ALIGN, Phase.LAND, Phase.TOUCHDOWN,
               Phase.EXIT) + TOUCHDOWN_PHASES + (Phase.COMPLETE,)
#: Phases from which AUX down or a supervisor request means EXIT.
#: B3: YAW_SEARCH is engaged from its first command (SRC2, GUIDED); before
#: the switch is sent, a request goes back to IDLE (request_exit), like one
#: in GATE_SEARCH - nothing was commanded yet.
ENGAGED_PHASES = (Phase.YAW_SEARCH, Phase.ENGAGE, Phase.MOVE,
                  Phase.CENTER_CHECK, Phase.DESCEND, Phase.FINAL_ALIGN,
                  Phase.LAND, Phase.TOUCHDOWN) + TOUCHDOWN_PHASES
#: Phases in which estimates are sent to the EKF (D5: from ENGAGE on).
#: EXIT_RTL like EXIT: while set 2 may still be active the EKF keeps its
#: vision (with the supervisor's abort up the I/O thread drops it anyway).
VPE_PHASES = ENGAGED_PHASES + (Phase.EXIT, Phase.COMPLETE, Phase.EXIT_RTL)
#: Phases from which an EXIT_RTL request runs the RTL path (B1): the
#: procedure from AUX up. GATE_SEARCH too - nothing engaged yet, but the
#: ceiling holds there as well (rule 15.2.4).
EXIT_RTL_FROM = (Phase.GATE_SEARCH,) + ENGAGED_PHASES
#: Phases in which we hold a mode of our own and watch it.
#: B3: YAW_SEARCH holds GUIDED from its 'mode' sub-step on (the watch is
#: armed once GUIDED is confirmed; before that the sub-steps check the
#: pilot's mode themselves).
GUIDED_PHASES = (Phase.YAW_SEARCH, Phase.MOVE, Phase.CENTER_CHECK,
                 Phase.DESCEND, Phase.FINAL_ALIGN) + TOUCHDOWN_PHASES


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
    #: B3, the marker search by rotation (nova.config.yaw_search_settings):
    #: 'auto' | True | False. False HERE (the dataclass default, i.e. a
    #: caller that does not configure it): the previous behaviour. The app
    #: passes the config's value, 'auto' by default.
    yaw_search: object = False
    yaw_search_max_deg: float = 90.0
    yaw_search_rate_dps: float = 25.0
    search_radius_m: float = SEARCH_RADIUS_M
    #: yaw_search_h_cover_m() from the effective calibration; None =
    #: unknown, and then 'auto' does not search (only True does).
    h_cover_m: object = None


class ExtNavLanding:
    """
        sm = ExtNavLanding(vehicle, estimator, ekf, gate, cfg, on_event)
        sm.on_detection(det, now)     # every detection, any phase
        sm.update(now)                # at loop rate
        sm.request_exit(reason)       # the supervisor's only lever

    Events (on_event(name, info)): 'state', 'handover_reject', 'gate_fail',
    'engage', 'scoring_capture' (t = capture time of the detection),
    'touchdown' (t = now), 'exit', 'exit_done', 'aux_ignored_disarmed';
    B3: 'yaw_search' (begin), 'yaw_search_src2' / '_ack' / '_rotate' /
    '_found' / '_stop' (s = seconds from the SRC2 command) and
    'yaw_search_end' (result 'gasit' / 'negasit' / 'neconfirmat' / 'abort').
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
        self._exit_owned = set()        # G1: modes owned when the exit began
        #: G1: the pilot's mode when the search started (GATE_SEARCH owns it)
        self._search_mode = None
        #: G2: this loop's previous update() time, and the sticky stall flag
        self._last_update_t = None
        self._stalled = False
        self._last_abort_rtl = False    # kind of the abort last read
        self._clear_rtl()
        self.n_vpe_sent = 0
        self._clear_touchdown()
        self._clear_yaw_search()
        #: B3 counters, for the logs and the tests (never reset)
        self.n_condition_yaw = 0
        self.n_stop_targets = 0

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
        self._clear_rtl()
        self._clear_touchdown()
        self._clear_yaw_search()

    def _clear_yaw_search(self):
        """State of YAW_SEARCH (B3). Times on the loop clock."""
        self._ys_sub = None             # 'src2', 'mode', 'rotate', 'found', 'confirm'
        self._ys_t0 = None              # YAW_SEARCH entered
        self._ys_t_switch = None        # SET_EKF_SOURCE_SET(2) sent: THE clock
        self._ys_ack_t = None
        self._ys_guided_t = None
        self._ys_rot_t = None           # CONDITION_YAW sent
        self._ys_yaw0 = None
        self._ys_yaw_last = None
        self._ys_turn = 0.0             # accumulated rotation, rad (signed)
        self._ys_det_t = None           # first detection after the switch
        self._ys_det_deg = None         # degrees rotated at that moment
        self._ys_first_est = None       # its estimate
        self._ys_vpe_t = None           # first VPE of the search sent
        self._ys_reset_t = None         # the reset seen in LOCAL_POSITION_NED
        self._ys_stop_t = None          # the stop target sent
        self._ys_stop = None            # (x, y, z, yaw) of that target

    def _clear_rtl(self):
        """State of the EXIT_RTL path (B1)."""
        self._rtl_step = None           # 'src1', 'frame', 'rtl', 'land'
        self._rtl_t0 = None             # path start (the decision, for us)
        self._rtl_src_t = None          # last SRC1 sent
        self._rtl_src_n = 0
        self._rtl_ack_t = None
        self._rtl_gpi0 = self._rtl_gps0 = None
        self._rtl_mode0 = set()         # modes not yet "someone else's"
        self._rtl_land_why = None

    def _clear_touchdown(self):
        self.contact = None             # what CONTACT recorded (dict)
        self._td_z0 = self._td_t0 = self._td_h0 = None
        self._td_setpoint_t = float('-inf')
        self._ground_xy = None
        self._ground_vpe_t = float('-inf')
        self._td_blind_xy = None
        self._takeoff_t = None
        self._takeoff_n = 0
        self._riseup_t0 = None
        self._z_riseup = None
        self._confirm_since = None
        self._corrected = False
        self._correcting = False
        self._correct_since = None
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
        ys = self.state == Phase.YAW_SEARCH
        if ys and self._ys_t_switch is None:
            # B3: before SET_EKF_SOURCE_SET(2) is out, no vision to the EKF
            # (the measured scenario: the first VPE after the switch resets
            # it onto the marker frame; on SRC1 it would only sit in the
            # buffer). The estimate stays in the window.
            return
        if self.state in VPE_PHASES:
            # B3: in YAW_SEARCH every estimate goes out at once - the frame
            # is built with the attitude AT CAPTURE (extnav.estimate), so
            # it is valid while the vehicle yaws
            if self.est.send(e):
                self.n_vpe_sent += 1
                if ys and self._ys_vpe_t is None:
                    self._ys_vpe_t = now
        if ys and self._ys_det_t is None:
            self._ys_first_detection(now, e)

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
        if (self._mode_want == MODE_GUIDED and self._guided_pending_phase()
                and self._pilot_mode_lost()):
            # B3 fix (§5.66 class): a GUIDED re-send only over the pilot's
            # own mode, recorded at AUX up - never over one set between two
            # retries (normally caught as 'taken' above; this holds even if
            # _mode_before were ever not the pilot's mode)
            self._mode_want = None
            return 'taken'
        self._mode_req_t = now
        self._mode_req_n += 1
        self._request_mode(self._mode_want)
        return None

    def _guided_pending_phase(self):
        """ENGAGE / YAW_SEARCH before GUIDED is confirmed: the FC is in
        the pilot's mode and nothing of ours watches it (no _mode_expected)
        - the window of the §5.66 class found in B3."""
        return (self.state in (Phase.ENGAGE, Phase.YAW_SEARCH)
                and self._mode_expected != MODE_GUIDED)

    def _pilot_mode_lost(self):
        """True when the FC is no longer in the pilot's mode recorded at
        AUX up (_search_mode): the pilot's switch (sticks neutral, so no
        override) or a failsafe (LAND, RTL) - theirs, not ours to override.
        No recorded mode at all (should not happen) = not owned either."""
        return self._search_mode is None or self.v.mode != self._search_mode

    def _exit_pilot_mode(self, where):
        self.request_exit(f"mod schimbat de pilot/failsafe inainte de GUIDED "
                          f"({self.v.mode_name()}, {where})", passive=True)

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
        # B1: which exit - read with the reason, after the new number
        self._last_abort_rtl = bool(getattr(sup, 'abort_rtl', False))
        return (sup.abort_reason or 'abort supervizor'), bool(sup.abort_passive)

    def request_exit(self, reason, passive=False, rtl=False):
        """The supervisor's lever, and ours. Idempotent inside EXIT.

        `rtl=True` (B1): the ceiling's exit, EXIT_RTL - from GATE_SEARCH
        and every engaged phase. While EXIT_RTL runs, any request (a normal
        one too: AUX down, a timeout) is ignored; while a normal EXIT runs,
        an EXIT_RTL request is ignored (the hand-back under way is not
        overruled - same choice as in the supervisor's latch)."""
        if self.state == Phase.EXIT_RTL:
            return
        # B1.9 (G1, G2): the mode rule INSIDE the request, so that no path -
        # the supervisor's abort included, which is read before the mode
        # watch - can bypass it: a mode we do not own, or a stall of this
        # loop, makes the exit passive whatever the caller said.
        if not passive and self._mode_taken_by_other():
            passive = True
            reason = f"{reason} [mod {self.v.mode_name()} nepus de noi: pasiv]"
        if not passive and self._stalled:
            passive = True
            reason = f"{reason} [dupa blocaj: pasiv]"
        if rtl and self.state in EXIT_RTL_FROM:
            self._begin_exit_rtl(reason, passive)
            return
        if self.state in (Phase.EXIT, Phase.ABORT, Phase.IDLE, Phase.DONE,
                          Phase.COMPLETE):
            # COMPLETE is already the ordered hand-back (SRC1, LOITER)
            return
        if (self.state == Phase.YAW_SEARCH and self._ys_t_switch is None
                and self.ekf.state != self.ekf.ACTIV):
            # B3: the search had not sent its first command yet (the set
            # was still being read back): nothing to undo, nothing to send -
            # like a request in GATE_SEARCH. The manager lets go so it does
            # not switch after the fact.
            self.ekf.release(self.now, reason)
            self._emit('yaw_search_end', result='abort', reason=reason,
                       deg=0.0, s=None)
            self.reset(f"iesire inainte de comutare, in cautare: {reason}")
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
        self._exit_owned = self._owned_modes()
        self.ekf.release(self.now, reason, urgent=True)
        self._mode_want = None
        self._mode_expected = None
        self._mode_stage = 0
        self.set_state(Phase.EXIT, reason)

    def _handback_done(self, text, mode, passive, kind='exit', result=None):
        """End of the hand-back: EXIT / EXIT_RTL -> ABORT ('exit_done'),
        COMPLETE -> DONE ('sequence_complete', said on the OSD and to the
        FC). From ABORT nothing is sent again, whatever the FC does next."""
        if self.state == Phase.COMPLETE:
            self.set_state(Phase.DONE, text)
            self._emit('sequence_complete', mode=mode, passive=passive,
                       h=self.h_now())
            self._statustext("NOVA SECVENTA COMPLETA", warn=False, urgent=True)
            self._say("\n== SECVENTA COMPLETA: SRC1, predat pilotului ==\n")
        else:
            self.set_state(Phase.ABORT, text)
            self._emit('exit_done', mode=mode, passive=passive, kind=kind,
                       result=result)

    def _src1_acked(self):
        """True only for an ACCEPTED attributed to the LATEST source-set
        request, and that request is set 1 (Vehicle.ekf_src_ack_for). The
        raw `ekf_src_ack` is not read: right after a request it still holds
        the previous command's ACCEPTED (the SRC2 switch) until the I/O
        thread dispatches - the stale ack found in SITL (02.10.2026). A
        vehicle without the accessor never counts as acknowledged."""
        fn = getattr(self.v, 'ekf_src_ack_for', None)
        if fn is None:
            return False
        return fn(1) == mavutil.mavlink.MAV_RESULT_ACCEPTED

    def _run_exit(self, now):
        """The ordered hand-back, for EXIT and for COMPLETE alike: SRC1
        acknowledged first, then LOITER, ALT_HOLD if LOITER is refused.
        B1: the ack is the one attributed to OUR request (_src1_acked);
        the wait is real now - 2-12 ms measured in SITL on TCP, a few tens
        of ms on the serial link - and still capped at EXIT_ACK_TIMEOUT_S,
        after which the hand-back goes on regardless (the pilot must get
        the vehicle back)."""
        if self._exit_step == 'src1':
            acked = self._src1_acked()
            if not acked and now - self._exit_t < EXIT_ACK_TIMEOUT_S:
                return
            if not acked:
                self._say("  !! SRC1 fara ACK in timp; continui iesirea")
            if self.exit_passive:
                self._handback_done('EXIT pasiv: SRC1 cerut, modul e al pilotului',
                                    self.v.mode, True)
                return
            if self.v.mode not in self._exit_owned:
                # B1.9 (G1): the pilot / a failsafe changed the mode while
                # we waited for the ack - the LOITER would override them
                self._emit('exit_taken', mode=self.v.mode)
                self._handback_done(f"pasiv: {self.v.mode_name()} pus de altcineva "
                                    f"inainte de LOITER", self.v.mode, True)
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

    # -- EXIT_RTL (B1, 02.10.2026, rule 15.2.4) -----------------------------
    def _owned_modes(self):
        """B1.9 (G1): the modes the companion may command OVER - the one it
        holds (GUIDED in the guided phases), a request of its own still in
        flight and the mode that request started from, and in GATE_SEARCH
        / ENGAGE the pilot's mode recorded when the search started. Any
        other FC mode is the pilot's or a failsafe's (the mode rule of §6):
        an exit decided then is PASSIVE. Empty set = nothing is owned."""
        owned = set()
        if self._mode_expected is not None:
            owned.add(self._mode_expected)
        if self._mode_want is not None:
            owned.add(self._mode_want)
            if self._mode_before is not None:
                owned.add(self._mode_before)
        # B3: in YAW_SEARCH (and the ENGAGE that follows it) the pilot's
        # mode is owned only until GUIDED is confirmed: from then on the
        # companion holds GUIDED, and a return to the pilot's mode is the
        # pilot's switch, not ours to override.
        if (self.state in (Phase.GATE_SEARCH, Phase.ENGAGE, Phase.YAW_SEARCH)
                and self._search_mode is not None
                and self._mode_expected != MODE_GUIDED):
            owned.add(self._search_mode)
        return owned

    def _mode_taken_by_other(self):
        """True when the FC is in a mode we do not own (_owned_modes):
        the exit then stays PASSIVE - SRC1 goes back, no mode command."""
        return self.v.mode not in self._owned_modes()

    def _on_stall(self, now, gap):
        """B1.9 (G2): this loop did not run for `gap` s. Sticky for the
        attempt: every exit from here is passive, and one under way stops
        sending mode commands. In the engaged phases the attempt ends here
        (passive EXIT); in GATE_SEARCH the search is dropped (nothing was
        engaged - a new attempt needs AUX down and up)."""
        first = not self._stalled
        self._stalled = True
        self._emit('stall', gap_s=gap, phase=self.state)
        if first:
            self._say(f"\n!! bucla principala blocata {gap:.1f} s in {self.state}: "
                      f"fara comenzi de mod pana la o incercare noua\n")
        st = self.state
        if st in ENGAGED_PHASES:
            self.request_exit(f"bucla principala blocata {gap:.1f} s", passive=True)
        elif st == Phase.GATE_SEARCH:
            self.reset(f"bucla blocata {gap:.1f} s in cautare")
        elif st in (Phase.EXIT, Phase.COMPLETE):
            self.exit_passive = True
            if self._exit_step == 'mode':
                self._mode_want = None
                self._handback_done(f"{st} pasiv dupa blocaj: nicio comanda de mod",
                                    self.v.mode, True)
        elif st == Phase.EXIT_RTL:
            self.exit_passive = True
            if self._rtl_step != 'src1':
                self._mode_want = None
                self._rtl_passive_done(f"EXIT_RTL pasiv dupa blocaj de {gap:.1f} s")

    def _send_src1(self, now, resend):
        """SET_EKF_SOURCE_SET(1), URGENT. The first one is a NEW request:
        it invalidates any earlier ack at once (Vehicle); resends keep it."""
        fn = self.v.send_ekf_source_set
        try:
            ok = fn(1, urgent=True, resend=resend)
        except TypeError:                       # test vehicles without kwargs
            ok = fn(1)
        self._rtl_src_t = now
        self._rtl_src_n += 1
        return ok

    def _begin_exit_rtl(self, reason, passive):
        now = self.now
        self.exit_reason = reason
        self.exit_passive = passive
        self._clear_rtl()
        self._rtl_t0 = now
        # G1: the modes owned when the exit was decided (_owned_modes; a
        # request of ours in flight - ENGAGE asking GUIDED - included: the
        # FC adopting it is not the pilot's doing). Before RTL is sent, the
        # FC must still be in one of them.
        self._rtl_mode0 = self._owned_modes()
        ceiling = getattr(self.sup, 'ceiling_agl_m', None)
        # a. the abort is up (the supervisor raised it; TX is being purged);
        #    the OSD event and the STATUSTEXT, urgent
        self._emit('exit_rtl', reason=reason, passive=passive,
                   ceiling_m=ceiling, phase=self.state)
        self._say(f"\n!! EXIT_RTL: {reason}" + (" (pasiv: fara comenzi de mod)"
                                               if passive else '') + "\n")
        n = f"{ceiling:.0f}m " if ceiling is not None else ''
        self._statustext(f"NOVA PLAFON {n}DEPASIT: SRC1 -> RTL", urgent=True)
        # b. SRC1, explicitly, from every phase; the manager let go so it
        #    does not switch back (it sends nothing itself here)
        self.ekf.release(now, reason, urgent=True, send=False)
        self._send_src1(now, resend=False)
        self._rtl_step = 'src1'
        self._mode_want = None
        self._mode_expected = None
        self._mode_stage = 0
        self.set_state(Phase.EXIT_RTL, reason)

    def _frame_dist_m(self):
        """Horizontal distance GLOBAL_POSITION_INT - GPS_RAW_INT, metres,
        from samples received AFTER the ack was seen; None until both have
        a new one with a position (and the GPS a 3D fix). Abort path only:
        this GNSS reading never feeds guidance (15.2.5; rule 15.2.4 lets
        the monitor use the FC's GNSS / altitude data)."""
        v = self.v
        n_gpi, n_gps = getattr(v, 'n_gpi', None), getattr(v, 'n_gps', None)
        if (n_gpi is None or n_gps is None or n_gpi <= self._rtl_gpi0
                or n_gps <= self._rtl_gps0):
            return None
        lat, lon = getattr(v, 'lat', None), getattr(v, 'lon', None)
        glat, glon = getattr(v, 'gps_lat', None), getattr(v, 'gps_lon', None)
        if None in (lat, lon, glat, glon):
            return None
        fix = getattr(v, 'gps_fix', None)
        if fix is not None and fix < 3:
            return None
        k = math.radians(1.0) * 6371000.0
        dn = (lat - glat) * k
        de = (lon - glon) * k * math.cos(math.radians(glat))
        return math.hypot(dn, de)

    def _rtl_passive_done(self, why):
        self._handback_done(why, self.v.mode, True, kind='rtl', result='passive')

    def _rtl_land(self, now, why):
        """Step e: LAND at the current position (rule 16.2.3), urgent."""
        self._rtl_step = 'land'
        self._rtl_land_why = why
        self._emit('exit_rtl_land', reason=why)
        self._say(f"  !! EXIT_RTL: {why}; cer LAND")
        self._statustext("NOVA RTL refuzat -> LAND" if why == 'rtl_refuzat'
                         else "NOVA SRC1 neconfirmat -> LAND (fara RTL)",
                         urgent=True)
        self._mode_begin(now, MODE_LAND, urgent=True)

    def _run_exit_rtl(self, now):
        step = self._rtl_step
        if step in ('src1', 'frame') and self.v.mode not in self._rtl_mode0:
            # No mode command of ours is out yet: a mode change now is the
            # pilot's switch or a failsafe (§5.66) - it outranks us.
            self._emit('exit_rtl_taken', mode=self.v.mode, step=step)
            self._rtl_passive_done(f"EXIT_RTL: mod schimbat de altcineva "
                                   f"({self.v.mode_name()}) inainte de RTL")
            return
        if step == 'src1':
            if not self._src1_acked():
                if now - self._rtl_t0 >= EXIT_ACK_TIMEOUT_S:
                    self._emit('exit_rtl_no_ack', s=now - self._rtl_t0,
                               n=self._rtl_src_n)
                    if self.exit_passive:
                        self._rtl_passive_done('EXIT_RTL pasiv: SRC1 neconfirmat, '
                                               'modul e al pilotului')
                        return
                    # never RTL here: on set 2 it lands on the marker
                    self._rtl_land(now, 'src1_neconfirmat')
                    return
                if now - self._rtl_src_t >= EXIT_RTL_SRC1_RESEND_S:
                    self._send_src1(now, resend=True)
                return
            self._rtl_ack_t = now
            self._rtl_gpi0 = getattr(self.v, 'n_gpi', 0) or 0
            self._rtl_gps0 = getattr(self.v, 'n_gps', 0) or 0
            self._emit('exit_rtl_src1', s=now - self._rtl_t0, n=self._rtl_src_n)
            if self.exit_passive:
                self._rtl_passive_done('EXIT_RTL pasiv: SRC1 confirmat, modul '
                                       'e al pilotului')
                return
            self._rtl_step = 'frame'
            step = 'frame'
        if step == 'frame':
            d = self._frame_dist_m()
            waited = now - self._rtl_ack_t
            if d is not None and d < FRAME_TOL_M:
                self._emit('exit_rtl_frame', ok=True, dist_m=d, s=waited)
            elif waited >= FRAME_TIMEOUT_S:
                self._emit('exit_rtl_frame', ok=False, dist_m=d, s=waited)
                self._say(f"  !! EXIT_RTL: cadrul GNSS neconfirmat in "
                          f"{FRAME_TIMEOUT_S:.1f} s (dist "
                          f"{'-' if d is None else f'{d:.1f} m'}); cer RTL oricum")
            else:
                return
            # d. RTL, urgent, confirmed from HEARTBEAT
            self._rtl_step = 'rtl'
            self._emit('exit_rtl_mode', mode=MODE_RTL)
            self._mode_begin(now, MODE_RTL, urgent=True)
            return
        r = self._mode_drive(now)
        if r is None:
            return
        if r == 'taken':
            self._rtl_passive_done(f"EXIT_RTL: {self.v.mode_name()} pus de "
                                   f"altcineva; nu mai trimit nimic")
            return
        if step == 'rtl':
            if r == 'ok':
                self._handback_done('EXIT_RTL: RTL confirmat', self.v.mode,
                                    False, kind='rtl', result='rtl')
            else:
                self._rtl_land(now, 'rtl_refuzat')
            return
        if r == 'ok':
            self._handback_done('EXIT_RTL: LAND confirmat', self.v.mode, False,
                                kind='rtl', result='land')
        else:
            self._handback_done('EXIT_RTL: nici RTL, nici LAND confirmat',
                                self.v.mode, False, kind='rtl', result='fail')

    # -- loop --------------------------------------------------------------
    def update(self, now=None):
        now = now if now is not None else time.monotonic()
        self.now = now
        # B1.9 (G2): our own gap, first thing - before anything acts on a
        # picture of the vehicle that may be STALL_MAX_S or more old
        gap = None if self._last_update_t is None else now - self._last_update_t
        self._last_update_t = now
        if gap is not None and gap > STALL_MAX_S:
            self._on_stall(now, gap)
        if self.was_armed and not self.v.armed and self.v.have_pos:
            if self.state in GROUND_PHASES:
                # the FC disarmed on the marker (throttle at zero for
                # DISARM_DELAY, a failsafe): no mode command - reported
                self._emit('ground_disarmed', phase=self.state,
                           held_s=None if self.contact is None
                           else now - self.contact['t_on_ground_rx'])
                self._say(f"\n!! DEZARMAT PE SOL in {self.state}: fara comenzi\n")
            if self.state in ENGAGED_PHASES or self.state in (Phase.EXIT,
                                                                Phase.EXIT_RTL):
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
            self.request_exit(ab[0], passive=ab[1], rtl=self._last_abort_rtl)

        st = self.state
        if st == Phase.EXIT_RTL:
            self._run_exit_rtl(now)
            return
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
        elif st == Phase.YAW_SEARCH:
            self._run_yaw_search(now)
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
        self._ekf_step(now)

    def _ekf_step(self, now):
        """EkfSourceManager.update from the phase, and (B3) the time the
        SRC2 command left, if it left in this very call while searching:
        YAW_SEARCH's deadlines all count from it."""
        self.ekf.update(now, self.state)
        if (self.state == Phase.YAW_SEARCH and self._ys_t_switch is None
                and self.ekf.state == self.ekf.ACTIV):
            self._ys_t_switch = now
            # estimates from before the switch do not open the segment
            self.window.clear()
            self.window_since = now
            self._emit('yaw_search_src2', s=now - self._ys_t0)

    # -- phases ------------------------------------------------------------
    def _start_search(self, now):
        if self.gate is None:
            self.reset('fara poarta')
            return
        # a new attempt through the gate: the stall of a previous one is
        # history (G2); the pilot's mode now is the one GATE_SEARCH owns (G1)
        self._stalled = False
        self._search_mode = self.v.mode
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
            # B3: no marker in the stationary windows. The rotation only
            # when the gate itself accepted (E0, sticks, altitude: ok is
            # True) and the configuration asks for it at this height.
            go, why = self._yaw_search_wanted(ok)
            if go:
                self._begin_yaw_search(now, why)
                return
            self._emit('gate_fail', n=len(self.window), alt=self.h_now(),
                       no_rotation=why, h_cover=self.cfg.h_cover_m)
            if why == 'h_cover':
                self._say(f"  .. MARKER NEGASIT: fara rotire, h {self.h_now():.1f} m "
                          f">= h_cover {self.cfg.h_cover_m:.1f} m")
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

    # -- YAW_SEARCH (B3, 04.10.2026) ----------------------------------------
    def _yaw_search_wanted(self, gate_ok):
        """(go, why) at the end of the gate's windows without a detection.
        why: 'fortat' (yaw_search true), 'auto' (below h_cover), or why NOT:
        None (the gate did not accept / yaw_search false - GATE_FAIL as
        before B3), 'h_cover' (auto, at or above h_cover: the short axis
        already covers the search radius), 'h_cover_necunoscut' (auto
        without a calibration to compute it from)."""
        if gate_ok is not True:
            return False, None
        if self._pilot_mode_lost():
            # the pilot / a failsafe changed the mode during the gate's
            # windows (or none was recorded): their decision - no command
            # of ours over it
            return False, 'mod_schimbat'
        mode = self.cfg.yaw_search
        if mode is True:
            return True, 'fortat'
        if mode != 'auto':
            return False, None
        hc = self.cfg.h_cover_m
        if hc is None:
            return False, 'h_cover_necunoscut'
        if self.h_now() >= hc:
            return False, 'h_cover'
        return True, 'auto'

    def _ys_progress_deg(self):
        return abs(math.degrees(self._ys_turn))

    def _ys_track_yaw(self):
        """Accumulated rotation since CONDITION_YAW, from every new yaw
        (wrapped steps summed, so 180 deg does not fold back to 0)."""
        if self._ys_yaw_last is None:
            return
        y = self.v.yaw
        self._ys_turn += extnav.wrap_pi(y - self._ys_yaw_last)
        self._ys_yaw_last = y

    def _begin_yaw_search(self, now, why):
        self._clear_yaw_search()
        self._ys_t0 = now
        self._ys_sub = 'src2'
        self.window.clear()
        self.window_since = now
        c = self.cfg
        self._emit('yaw_search', alt=self.h_now(), deg=c.yaw_search_max_deg,
                   rate=c.yaw_search_rate_dps, h_cover=c.h_cover_m, why=why)
        self._say(f"  == marker negasit in {self.window_tries} ferestre: CAUTARE "
                  f"prin rotire ({why}), {c.yaw_search_max_deg:.0f} grd la "
                  f"{c.yaw_search_rate_dps:.0f} grd/s")
        self.set_state(Phase.YAW_SEARCH, f"{why}, h {self.h_now():.2f} m")
        # the manager is armed from the phase - this very step
        self._ekf_step(now)

    def _ys_end(self, now, result, reason):
        """The search did not reach ENGAGE: OSD / STATUSTEXT, normal EXIT
        (SRC1 acknowledged, then LOITER)."""
        deg = self._ys_progress_deg()
        s = None if self._ys_t_switch is None else now - self._ys_t_switch
        self._emit('yaw_search_end', result=result, reason=reason, deg=deg, s=s)
        if result == 'negasit':
            self._statustext(f"NOVA MARKER NEGASIT dupa rotire {deg:.0f} grd",
                             warn=False)
        self.request_exit(reason)

    def _ys_first_detection(self, now, e):
        """First estimate after the SRC2 command (called from on_detection,
        after its VPE went out): the EKF resets onto the marker frame."""
        self._ys_det_t = now
        self._ys_first_est = e
        self._ys_det_deg = self._ys_progress_deg()
        self._emit('yaw_search_found', deg=self._ys_det_deg,
                   s=now - self._ys_t_switch, lateral_m=e.lateral_m,
                   sub=self._ys_sub)
        self._say(f"  == MARKER GASIT la {self._ys_det_deg:.0f} grd "
                  f"({now - self._ys_t_switch:.2f} s de la SRC2, "
                  f"{e.lateral_m:.2f} m lateral)")
        self._statustext(f"NOVA MARKER GASIT la {self._ys_det_deg:.0f} grd",
                         warn=False)

    def _ys_deadline(self):
        """(time, why) of the exit that applies NOW, or (None, '') before
        the switch (bounded by engage_timeout_s then).

        Before the first detection: t_switch + SEARCH_BUDGET_S - the HARD
        deadline, 2.0 s before the EKF drops the position (7.0 s without
        vision, counted from the switch).
        First detection, reset not yet seen: EKF_SETTLE_S +
        YAW_SEARCH_RESET_WAIT_S after it, capped at t_switch +
        YAW_SEARCH_RESET_CAP_S (6.0 s): if that VPE was not fused the EKF
        still counts from the switch.
        Reset seen (the VPE was fused: the EKF clock restarted from it):
        YAW_SEARCH_CONFIRM_S after the first detection.

        DELIBERATE DEVIATION from the simplest rule ("exit at t_switch +
        5.0 s unless ENGAGE was reached"): with it a detection at 4.9 s
        could never be confirmed (the reset alone needs EKF_SETTLE_S) and
        the 4.0 s confirmation window would always be cut at 5.0 s. Every
        bound above stays under the 7.0 s of the vision sample it relies
        on."""
        ts = self._ys_t_switch
        if ts is None:
            return None, ''
        if self._ys_det_t is None:
            return ts + SEARCH_BUDGET_S, (f"termen {SEARCH_BUDGET_S:.1f} s de la "
                                          f"SRC2 fara detectie")
        if self._ys_reset_t is None:
            return (min(self._ys_det_t + EKF_SETTLE_S + YAW_SEARCH_RESET_WAIT_S,
                        ts + YAW_SEARCH_RESET_CAP_S),
                    'resetul EKF pe cadrul markerului neobservat')
        return (self._ys_det_t + YAW_SEARCH_CONFIRM_S,
                f"marker neconfirmat in {YAW_SEARCH_CONFIRM_S:.0f} s de la gasire")

    def _run_yaw_search(self, now):
        c = self.cfg
        self._ys_track_yaw()
        sub = self._ys_sub
        # 1. the deadline first, whatever the sub-step
        dl, why = self._ys_deadline()
        if dl is not None and now >= dl:
            deg = self._ys_progress_deg()
            if self._ys_det_t is None:
                self._ys_end(now, 'negasit',
                             f"marker negasit dupa rotire {deg:.0f} grd ({why})")
            else:
                self._ys_end(now, 'neconfirmat', f"cautare: {why}")
            return
        if sub == 'src2':
            if self._pilot_mode_lost():
                # nothing of ours holds a mode yet: the pilot's switch / a
                # failsafe (the mode rule of §6) - passive
                self._exit_pilot_mode('cautare, pas src2')
                return
            if self._ys_t_switch is None:
                self._ekf_step(now)
                if self.ekf.state == self.ekf.REFUZAT:
                    self.request_exit(f"setul EKF refuzat: "
                                      f"{'; '.join(self.ekf.motive_refuz)}")
                    return
                if self._ys_t_switch is None:
                    if now - self.state_since > c.engage_timeout_s:
                        self.request_exit(f"cautare: SRC2 necomutat in "
                                          f"{c.engage_timeout_s:.0f} s")
                    return
            ack = self.ekf.ack                   # ATTRIBUTED (ekf_source)
            if ack is not None and not self.ekf.ack_ok:
                self.request_exit(f"cautare: SET_EKF_SOURCE_SET(2) respins "
                                  f"(result={ack})")
                return
            if not self.ekf.ack_ok:
                return                           # the deadline bounds the wait
            self._ys_ack_t = now
            if self._pilot_mode_lost():            # re-checked right before
                self._exit_pilot_mode('cautare, inainte de GUIDED')
                return
            self._ys_sub = 'mode'
            self._emit('yaw_search_ack', s=now - self._ys_t_switch)
            self._mode_begin(now, MODE_GUIDED)
            return
        if sub == 'mode':
            r = self._mode_drive(now)
            if r is None:
                return
            if r == 'taken':
                self.request_exit(f"mod pus de altcineva in cautare "
                                  f"({self.v.mode_name()})", passive=True)
                return
            if r == 'fail':
                self.request_exit('cautare: FC-ul nu a confirmat GUIDED')
                return
            self._ys_guided_t = now
            if self._ys_det_t is not None:
                # found already (between the switch and GUIDED): no rotation
                self._ys_sub = 'found'
                return
            self._ys_start_rotation(now)
            return
        if sub == 'rotate':
            if self._ys_det_t is not None:
                self._ys_sub = 'found'
                sub = 'found'
            else:
                deg = self._ys_progress_deg()
                dur = c.yaw_search_max_deg / c.yaw_search_rate_dps + STOP_MARGIN_S
                if (deg >= c.yaw_search_max_deg - YAW_SEARCH_DONE_TOL_DEG
                        or now - self._ys_rot_t >= dur):
                    self._ys_end(now, 'negasit',
                                 f"marker negasit dupa rotire {deg:.0f} grd")
                return
        if sub == 'found':
            # the reset onto the marker frame, observed the way ENGAGE does:
            # EKF valid and within EKF_RESET_TOL_M of our newest estimate,
            # EKF_SETTLE_S after the first VPE
            if self._ys_vpe_t is None or now - self._ys_vpe_t < EKF_SETTLE_S:
                return
            ok = getattr(self.v, 'ekf_pos_horiz_ok', lambda: None)()
            e = self.last_est
            if not ok or e is None:
                return
            if math.hypot(self.v.x - e.x, self.v.y - e.y) > EKF_RESET_TOL_M:
                return
            self._ys_reset_t = now
            # ONE position target at the EKF position in the NEW frame and
            # the current yaw: stops the rotation (0.30 s measured), no lurch
            stop = (self.v.x, self.v.y, self.v.z, self.v.yaw)
            self.v.send_position_target(*stop)
            self._ys_stop = stop
            self._ys_stop_t = now
            self.n_stop_targets += 1
            self._emit('yaw_search_stop', x=stop[0], y=stop[1], z=stop[2],
                       yaw=stop[3], deg=self._ys_progress_deg(),
                       s_reset=now - (self._ys_vpe_t or now))
            if c.gate_detections >= 2:
                # pairs must not straddle the frame reset: the marker of an
                # estimate is placed with LOCAL_POSITION_NED at its capture
                # (position_at), i.e. in two different frames
                self.window.clear()
                self.window_since = now
            self._ys_sub = 'confirm'
            sub = 'confirm'
        if sub == 'confirm':
            if c.gate_detections <= 1:
                found = (self._ys_first_est,)
            else:
                found = self._gate_found()
            if found is None:
                return
            self._ys_to_engage(now, found)

    def _ys_start_rotation(self, now):
        c = self.cfg
        self._ys_sub = 'rotate'
        self._ys_yaw0 = self._ys_yaw_last = self.v.yaw
        self._ys_turn = 0.0
        self._ys_rot_t = now
        # ONE command, relative, counter-clockwise (param3 = -1: left seen
        # from above, yaw decreasing). No position target during the
        # rotation: GUIDED holds the position from the EKF
        self.v.send_condition_yaw(c.yaw_search_max_deg, c.yaw_search_rate_dps,
                                  direction=-1, relative=True)
        self.n_condition_yaw += 1
        self._emit('yaw_search_rotate', deg=c.yaw_search_max_deg,
                   rate=c.yaw_search_rate_dps, s=now - self._ys_t_switch)
        self._statustext(f"NOVA ROTIRE CAUTARE {c.yaw_search_max_deg:.0f} grd la "
                         f"{c.yaw_search_rate_dps:.0f} grd/s", warn=False)

    def _ys_to_engage(self, now, found):
        e = found[-1]
        self._emit('engage', alt=self.h_now(), n=len(found),
                   pair_dt=(found[-1].t - found[0].t) if len(found) > 1 else None,
                   lateral_m=e.lateral_m, from_search=True,
                   deg=self._ys_det_deg, s=now - self._ys_t_switch)
        self._emit('yaw_search_end', result='gasit', reason='',
                   deg=self._ys_progress_deg(), s=now - self._ys_t_switch)
        self._say(f"  == cautare: marker confirmat ({len(found)} detectii, "
                  f"{now - self._ys_t_switch:.2f} s de la SRC2): ENGAGE")
        # ENGAGE's 'verify' sub-step: the switch and the mode are done
        self._sub = 'verify'
        self.set_state(Phase.ENGAGE, f"din cautare, alt {self.h_now():.2f} m")

    def _run_engage(self, now):
        if now - self.state_since > self.cfg.engage_timeout_s:
            self.request_exit(f"ENGAGE fara succes in {self.cfg.engage_timeout_s:.0f} s "
                              f"(pas {self._sub})")
            return
        if self._sub == 'verify':
            # B3: from YAW_SEARCH the switch and the mode are DONE - only
            # verified here, never repeated (one SRC2 command per attempt)
            if self.v.mode != MODE_GUIDED or self._mode_expected != MODE_GUIDED:
                self.request_exit(f"ENGAGE dupa cautare: FC in {self.v.mode_name()}, "
                                  f"nu GUIDED", passive=True)
                return
            if not (self.ekf.state == self.ekf.ACTIV and self.ekf.ack_ok):
                self.request_exit(f"ENGAGE dupa cautare: setul EKF 2 neconfirmat "
                                  f"({self.ekf.state})")
                return
            self._pick_yaw_target()
            self.h_target = self.h_now()
            self._send_setpoint(now, -self.h_target, force=True)
            self.set_state(Phase.MOVE, f"din cautare: SRC2 + GUIDED verificate, "
                                       f"h {self.h_target:.2f} m")
            return
        if self._sub in ('src2', 'ekf') and self._pilot_mode_lost():
            # B3 fix (§5.66 class): no mode watch before GUIDED is asked -
            # a mode the pilot / a failsafe set here would otherwise be
            # recorded as "before" by _mode_begin and overridden by GUIDED.
            # Passive: SRC1 back if SRC2 was sent, no mode command.
            self._exit_pilot_mode(f"ENGAGE, pas {self._sub}")
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
        self._blind_descent_vpe(now)
        if now - self._td_setpoint_t >= TOUCHDOWN_SETPOINT_S:
            self._td_setpoint_t = now
            # a z ramp at touchdown_speed, down to TOUCHDOWN_BELOW_M below the ground:
            # the position controller keeps xy on (0, 0) and the land
            # detector needs the throttle at its lower limit to see contact
            floor = self._td_z0 + self._td_h0 + TOUCHDOWN_BELOW_M
            z = min(self._td_z0 + self.cfg.touchdown_speed * (now - self._td_t0), floor)
            yaw = self.yaw_target if self.yaw_target is not None else self.v.yaw
            self.v.send_position_target(0.0, 0.0, z, yaw)

    def _blind_descent_vpe(self, now):
        """The last part of the touchdown descent, camera blind: the EKF
        position frozen at the moment the detections stopped, sent as
        VISION_POSITION_ESTIMATE at GROUND_VPE_S, so the EKF keeps a valid
        position until contact. A new detection ends it."""
        if self.last_det_t is not None and now - self.last_det_t <= TD_BLIND_S:
            self._td_blind_xy = None
            return
        if self._td_blind_xy is None:
            self._td_blind_xy = (self.v.x, self.v.y)
        if now - self._ground_vpe_t < GROUND_VPE_S:
            return
        self._ground_vpe_t = now
        x, y = self._td_blind_xy
        if self.v.send_vision_position_estimate(
                int(round(now * 1e6)), x, y, self.v.z, self.v.roll,
                self.v.pitch, self.v.yaw):
            self.n_ground_vpe += 1

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
            # start of the touchdown descent: the capture's burst reaches
            # back to it, to include the frames at ~1 m
            'td_start_t': self._td_t0,
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
            # one correction: the fresh detections pull the EKF onto the
            # marker frame; holding (0, 0) brings the vehicle over it. Done
            # only when the EKF also AGREES with a fresh detection - "EKF
            # on (0, 0)" alone may just be the EKF not having caught up
            ec = self._fresh_est(self._correct_since + 0.5)
            agree = (ec is not None
                     and math.hypot(self.v.x - ec.x, self.v.y - ec.y) < 0.5 * tol)
            if stable and math.hypot(self.v.x, self.v.y) < tol and agree:
                self._correcting = False
                self._confirm_since = None
                self._open_window(now)
            elif now - self._correct_since > HOVER_CORRECTION_TIMEOUT_S:
                self.request_exit(f"corectia nu a convers in "
                                  f"{HOVER_CORRECTION_TIMEOUT_S:.0f} s")
            return
        e = self._fresh_est(self.window_since)
        if e is not None and e.lateral_m >= tol:
            if self._corrected:
                self.request_exit(f"nu e deasupra markerului dupa corectie "
                                  f"({e.lateral_m:.2f} m >= {tol:.2f})")
                return
            self._corrected = self._correcting = True
            self._correct_since = now
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
        self.exit_passive = self._stalled
        self._exit_owned = self._owned_modes()
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
