#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Safety campaign: what can the Raspberry Pi do to leave the drone STUCK IN THE
AIR (dead, frozen, or fighting the pilot)? SITL ArduCopter 4.5.7, OUR code.

    python3 tools/sitl_pi_failure.py --list
    python3 tools/sitl_pi_failure.py --scenario F1 --phase MOVE --instance 8 --workdir /tmp/x
    python3 tools/sitl_pi_failure.py --campaign --instances 8,9,10 --outdir /tmp/camp

Architecture. The COMPANION is a separate OS process (--role companion), so
the harness can SIGKILL / SIGSTOP it. It runs the onboard stack the way
tools/nova_pi.py wires it (nova_pi.cablaj_extnav, PasPrincipal, LastDetection,
the real nova.state_machine.run_loop, SupervisorThread, thread heartbeats,
watchdog), on serial1 (port + 2). Its synthetic camera takes the simulated
truth from its own read-only MAVLink link on serial2 (port + 3). The parent
(harness) runs SITL, the pilot (RC override + truth, serial0), injects the
fault when the companion reports the requested phase (line "PHASE <name>" in
phase.txt) and observes the FC from the pilot link and from the DataFlash log.

Invariants (pass/fail per run, see check_invariants):
  I1 not stuck      60 s after the injection the vehicle is landed, or in a mode
                    the pilot commanded, or in a failsafe mode (LAND/RTL) that
                    makes progress, or handed back (LOITER/ALT_HOLD + EKF set 1);
                    NOT hovering in GUIDED with nobody commanding it.
  I2 pilot wins     after a pilot mode change, no mode change the pilot did not
                    ask for in the next 20 s (FC failsafes are listed, not
                    counted: they are the FC, not the Pi).
  I3 no stale cmds  after SIGCONT / after a hang ends, the companion does not
                    put the FC back in GUIDED.

Declared plainly: a desktop run, idealised IMU, no wind, synthetic camera at
~30% of 10 fps from perfect truth. It proves the FC/companion protocol, not
the detector, and not the real sensors.
"""

import argparse
import collections
import glob
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from pymavlink import mavutil                                  # noqa: E402

import sitl_touchdown as std                                    # noqa: E402

SITL_DEFAULT = os.path.expanduser('~/ardupilot-4.5.7/build/sitl/bin/arducopter')

COPTER_MODES = {0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
                5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
                13: 'SPORT', 16: 'POSHOLD', 17: 'BRAKE', 21: 'SMART_RTL'}
MODE_NUM = {v: k for k, v in COPTER_MODES.items()}
# libraries/AP_Vehicle/ModeReason.h (4.5.7), the ones that matter here
REASONS = {0: 'UNKNOWN', 1: 'RC_COMMAND', 2: 'GCS_COMMAND', 3: 'RADIO_FAILSAFE',
           4: 'BATTERY_FAILSAFE', 5: 'GCS_FAILSAFE', 6: 'EKF_FAILSAFE', 7: 'GPS_GLITCH',
           8: 'MISSION_END', 9: 'THROTTLE_LAND_ESCAPE', 10: 'FENCE_BREACHED',
           11: 'TERRAIN_FAILSAFE', 12: 'BRAKE_TIMEOUT', 17: 'TERMINATE',
           19: 'CRASH_FAILSAFE', 25: 'FAILSAFE', 26: 'INITIALISED', 31: 'STARTUP'}
FAILSAFE_REASONS = {'RADIO_FAILSAFE', 'BATTERY_FAILSAFE', 'GCS_FAILSAFE', 'EKF_FAILSAFE',
                    'GPS_GLITCH', 'FENCE_BREACHED', 'TERRAIN_FAILSAFE', 'CRASH_FAILSAFE',
                    'FAILSAFE', 'THROTTLE_LAND_ESCAPE', 'BRAKE_TIMEOUT', 'MISSION_END'}
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
MAV_CMD_SET_EKF_SOURCE_SET = 42007

# ---------------------------------------------------------------------------
# Scenario table. `phases`: phases to inject at (first = default).
# ---------------------------------------------------------------------------
ALL_PHASES = ['MOVE', 'DESCEND', 'TOUCHDOWN_DESCENT', 'GROUND_HOLD', 'RISEUP']
SCENARIOS = collections.OrderedDict([
    ('F0', dict(desc='baseline: no fault, the sequence must complete',
                phases=[None])),
    ('F1', dict(desc='kill (SIGKILL) the companion', phases=ALL_PHASES)),
    ('F2', dict(desc='freeze (SIGSTOP) 15 s, then SIGCONT, 30 s more',
                phases=['MOVE', 'DESCEND'])),
    ('F2c', dict(desc='F2 + the pilot takes --pilot-mode at +--at-s s while the companion '
                      'is frozen (after the EKF failsafe LAND); what does it do on SIGCONT',
                 phases=['MOVE'])),
    ('F3', dict(desc='pilot takeover (--pilot-mode ALT_HOLD|LOITER|STABILIZE), '
                     'companion ALIVE', phases=['MOVE', 'DESCEND', 'RISEUP'])),
    ('F4', dict(desc='companion killed in MOVE, pilot asks --pilot-mode '
                     '(ALT_HOLD|LOITER|RTL|NONE) at +--at-s s, then EKF set 1 '
                     'from the pilot link and LOITER (recovery without the Pi)',
                phases=['MOVE'])),
    ('F5a', dict(desc='RC failsafe (SIM_RC_FAIL 1, FS_THR_ENABLE 1), companion alive',
                 phases=['MOVE'])),
    ('F5b', dict(desc='RC failsafe, companion killed 1 s before',
                 phases=['MOVE'])),
    ('F6a', dict(desc='main loop of the LIVING companion hangs --hang-s s',
                 phases=['MOVE'])),
    ('F6c', dict(desc='F6a + the pilot takes --pilot-mode at +--at-s s DURING the hang; '
                      'what does the companion do when it wakes (I2: pilot wins)',
                 phases=['MOVE'])),
    ('F7', dict(desc='companion killed in GATE_SEARCH (AUX up, pilot still in LOITER on '
                     'set 1): the FC must just stay in the pilot LOITER', phases=['GATE_SEARCH'])),
    ('F6b', dict(desc='supervisor thread of the LIVING companion hangs --hang-s s',
                 phases=['MOVE'])),
])


def campaign_matrix():
    """[(run_id, argv-list)] - the whole campaign."""
    runs = []
    for ph in ALL_PHASES:
        runs.append((f"F1_{ph}", ['--scenario', 'F1', '--phase', ph, '--observe', '60']))
    for ph in ('MOVE', 'DESCEND'):
        runs.append((f"F2_{ph}", ['--scenario', 'F2', '--phase', ph, '--observe', '45']))
    for ph in ('MOVE', 'DESCEND', 'RISEUP'):
        for m in ('ALT_HOLD', 'LOITER', 'STABILIZE'):
            runs.append((f"F3_{ph}_{m}", ['--scenario', 'F3', '--phase', ph,
                                          '--pilot-mode', m, '--observe', '30']))
    for at in (3, 12):
        for m in ('ALT_HOLD', 'LOITER', 'RTL', 'NONE'):
            runs.append((f"F4_at{at}_{m}", ['--scenario', 'F4', '--phase', 'MOVE',
                                            '--pilot-mode', m, '--at-s', str(at),
                                            '--observe', '70']))
    for d in (1.0, 3.0):
        runs.append((f"F1_RISEUP_d{int(d)}", ['--scenario', 'F1', '--phase', 'RISEUP',
                                              '--delay-s', str(d), '--observe', '60']))
    for m in ('ALT_HOLD', 'LOITER', 'STABILIZE'):
        runs.append((f"F3_RISEUP_d2_{m}", ['--scenario', 'F3', '--phase', 'RISEUP',
                                           '--delay-s', '2.0', '--pilot-mode', m,
                                           '--observe', '30']))
    runs.append(('F2_MOVE_f4', ['--scenario', 'F2', '--phase', 'MOVE', '--freeze-s', '4',
                                '--observe', '45']))
    for m in ('STABILIZE', 'ALT_HOLD'):
        runs.append((f"F6c_{m}", ['--scenario', 'F6c', '--phase', 'MOVE', '--pilot-mode', m,
                                  '--at-s', '8', '--hang-s', '20', '--observe', '45']))
    for tag in ('', '_r2'):
        runs.append((f"F2c_ALT_HOLD{tag}", ['--scenario', 'F2c', '--phase', 'MOVE',
                                            '--pilot-mode', 'ALT_HOLD', '--at-s', '10',
                                            '--observe', '45']))
    runs.append(('F5a_MOVE_r2', ['--scenario', 'F5a', '--phase', 'MOVE', '--observe', '60']))
    runs.append(('F5b_MOVE_r2', ['--scenario', 'F5b', '--phase', 'MOVE', '--observe', '60']))
    runs.append(('F6c_STABILIZE_r2', ['--scenario', 'F6c', '--phase', 'MOVE', '--pilot-mode',
                                      'STABILIZE', '--at-s', '8', '--hang-s', '20',
                                      '--observe', '45']))
    runs.append(('F7_GATE_SEARCH_r2', ['--scenario', 'F7', '--phase', 'GATE_SEARCH',
                                       '--observe', '30']))
    runs.append(('F5a_MOVE', ['--scenario', 'F5a', '--phase', 'MOVE', '--observe', '60']))
    runs.append(('F5b_MOVE', ['--scenario', 'F5b', '--phase', 'MOVE', '--observe', '60']))
    runs.append(('F6a_MOVE', ['--scenario', 'F6a', '--phase', 'MOVE', '--observe', '45']))
    runs.append(('F6b_MOVE', ['--scenario', 'F6b', '--phase', 'MOVE', '--observe', '45']))
    runs.append(('F7_GATE_SEARCH', ['--scenario', 'F7', '--phase', 'GATE_SEARCH', '--observe', '30']))
    runs.append(('F0_baseline', ['--scenario', 'F0', '--observe', '10']))
    return runs


# ===========================================================================
# COMPANION ROLE
# ===========================================================================
class TruthReader(std.Pilot):
    """Read-only truth for the synthetic camera, on serial2: the same
    truth()/att/origin interface as the pilot (SyntheticCamera needs it),
    but it sends nothing except message-interval requests for ITS channel."""

    def __init__(self, conn):
        threading.Thread.__init__(self, name='truth', daemon=True)
        self.m = mavutil.mavlink_connection(conn, source_system=254, source_component=191)
        self.m.wait_heartbeat(timeout=60)
        self.gps = self.origin = None
        self.att = (0.0, 0.0, 0.0)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 20),
                        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30)):
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)
        self.cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                 mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)

    def run(self):
        while not self.stop.is_set():
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'GPS_RAW_INT':
                with self.lock:
                    self.gps = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
            elif t == 'ATTITUDE':
                self.att = (msg.roll, msg.pitch, msg.yaw)
            elif t == 'GPS_GLOBAL_ORIGIN':
                self.origin = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)


class SiteLog:
    """Append-only line file read by the parent; one os.write per line, so a
    SIGKILL never leaves half a line behind for the lines already written."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        self.lock = threading.Lock()

    def put(self, kind, text=''):
        line = f"{time.time():.3f} {kind} {text}\n".encode()
        with self.lock:
            os.write(self.fd, line)


def companion_main(a):
    """The onboard stack, wired like tools/nova_pi.py (not like
    sitl_touchdown.py, which has no heartbeats / watchdog)."""
    import nova_pi
    from nova import config as nova_config
    from nova.concurrency import Heartbeat, Latest
    from nova.rc import OverrideMonitor
    from nova.state_machine import run_loop
    from nova.supervisor_thread import SupervisorThread
    from nova.vehicle import Vehicle

    phase_log = SiteLog(a.phase_file)
    tx_log = SiteLog(a.tx_file)
    port = a.port
    truth = TruthReader(f"tcp:127.0.0.1:{port + 3}")
    truth.start()
    vehicle = Vehicle(f"tcp:127.0.0.1:{port + 2}", threaded=True).connect()

    # --- TX tap: every message that leaves the companion towards the FC ----
    # (the I/O thread is the only writer: Vehicle._write). Mode commands,
    # EKF set commands, takeoff and position targets are logged one by one,
    # the rest is counted. This is the "what does it send after waking" proof.
    orig_write = vehicle._write
    counts = collections.Counter()

    def tx_tap(name, args, kwargs):
        ok = orig_write(name, args, kwargs)
        if name == 'command_long_send':
            cmd = args[2]
            if cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                tx_log.put('MODE', f"{int(args[5])} ok={ok}")
            elif cmd == MAV_CMD_SET_EKF_SOURCE_SET:
                tx_log.put('SRC', f"{int(args[4])} ok={ok}")
            else:
                tx_log.put('CMD', f"{cmd} p1={args[4]} p7={args[10]} ok={ok}")
        elif name == 'set_position_target_local_ned_send':
            tx_log.put('POSTGT', f"z={args[7]:.2f} ok={ok}")
        elif name == 'vision_position_estimate_send':
            counts['vpe'] += 1
            tx_log.put('VPE', f"n={counts['vpe']}")
        elif name == 'param_set_send':
            tx_log.put('PARAM', f"{args[2]!r} ok={ok}")
        else:
            counts[name] += 1
        return ok
    vehicle._write = tx_tap

    cfg = nova_config.load()
    ov = OverrideMonitor(vehicle)
    sm_ref = {}

    def on_event(n, i):
        if n == 'state':
            phase_log.put('PHASE', i['new'])
        print(f"[ev] {n}: {i}", flush=True)
    a_ns = types.SimpleNamespace(monitor_motiv=None, monitor=False, no_ascent=False,
                                 no_authority=True)
    canal, prag = 8, 1500
    gate, sup, sm, _aut = nova_pi.cablaj_extnav(
        a_ns, cfg, vehicle, ov, {}, canal, prag,
        lambda why: print(f"!! HANDOVER REFUZAT: {why}", flush=True), on_event)
    sm_ref['sm'] = sm

    # --- the supervisor thread, heartbeats, watchdog: exactly nova_pi -----
    cam = std.SyntheticCamera(truth, seed=a.seed)
    det_latest = Latest()
    faza = Latest(sm.state)
    hb_principal = Heartbeat('principal')
    st = SupervisorThread(sup, vehicle, det_latest, faza, miss_streak_fn=lambda: None)
    sup.set_thread_heartbeats(mav_io=getattr(vehicle, 'io_heartbeat', None),
                              principal=hb_principal)
    vehicle.set_abort_event(sup.abort)

    # F6b: the supervisor's step hangs (wrapped here, nova/ is untouched).
    # run_thread() takes self.step at start(), so the instance attribute wins.
    hung_sup = {'done': False}
    if a.hang_supervisor_at:
        real_step = st.step

        def step_hang(now=None):
            ph = st.phase.get()[0]
            if not hung_sup['done'] and ph == a.hang_supervisor_at:
                hung_sup['done'] = True
                phase_log.put('HANG', f"sup start {a.hang_s}")
                print(f"[hang] supervisor hangs {a.hang_s} s", flush=True)
                time.sleep(a.hang_s)
                phase_log.put('HANG', 'sup end')
                print("[hang] supervisor wakes", flush=True)
            return real_step(now)
        st.step = step_hang

    # F6a: the main loop hangs. The hook rides LastDetection.on_poll like
    # PasPrincipal does in nova_pi (once per main-loop iteration).
    hung_main = {'done': False}

    class PasHang(nova_pi.PasPrincipal):
        def __call__(self, now):
            super().__call__(now)
            if (a.hang_main_at and not hung_main['done']
                    and self.sm.state == a.hang_main_at):
                hung_main['done'] = True
                phase_log.put('HANG', f"main start {a.hang_s}")
                print(f"[hang] main loop hangs {a.hang_s} s", flush=True)
                time.sleep(a.hang_s)
                phase_log.put('HANG', 'main end')
                print("[hang] main loop wakes", flush=True)

    class CamDetector:
        """The synthetic camera as a detector for run_loop."""
        latest = det_latest
        heartbeat = None
        miss_streak = None

        def poll(self, now):
            dets = cam.poll(now)
            for d in dets:
                det_latest.set(d, d.t)
            return dets

    detector = nova_pi.LastDetection(CamDetector())
    detector.on_poll = PasHang(sm, faza, hb_principal, supervisor_thread=st)

    def status(now):
        print(f"{sm.status_line()} | {sup.status()} | {st.status()}", flush=True)

    st.start()
    phase_log.put('READY', '')
    print("[companion] ready", flush=True)
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    run_loop(vehicle, detector, sm, supervisor=None, on_status=status)
    return 0


# ===========================================================================
# PARENT: the pilot with observation taps
# ===========================================================================
class Observer(std.Pilot):
    """sitl_touchdown.Pilot + everything the campaign measures. The base
    class keeps its run loop (RC override at 10 Hz, truth, VPE feed before
    ENGAGE); recv_match and the override send are wrapped to tap/gate."""

    def __init__(self, conn):
        super().__init__(conn)
        self.rc_on = True
        self.ev = []                 # (wall, kind, data)
        self.truth_log = []          # (wall, n, e, agl)
        self.ekf_log = []            # (wall, flags)
        self.hb_log = []             # (wall, mode, armed)
        self.ack_log = []            # (wall, command, result)
        self.boot = collections.deque(maxlen=400)   # (wall, boot_s)
        self.req_log = []            # (wall, what)
        self.text_log = []           # (wall, text)
        for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 10),
                        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 10)):
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)
        orig = self.m.recv_match

        def tap(*a, **k):
            msg = orig(*a, **k)
            if msg is not None:
                try:
                    self._tap(msg)
                except Exception:                           # noqa: BLE001
                    pass
            return msg
        self.m.recv_match = tap
        orig_rc = self.m.mav.rc_channels_override_send

        def rc_gate(*a, **k):
            if self.rc_on:
                return orig_rc(*a, **k)
        self.m.mav.rc_channels_override_send = rc_gate

    def _tap(self, msg):
        t = msg.get_type()
        now = time.time()
        if t == 'GPS_RAW_INT':
            tr = self.truth()
            if tr is not None:
                self.truth_log.append((now, tr[0], tr[1], tr[2]))
        elif t == 'ATTITUDE':
            self.boot.append((now, msg.time_boot_ms / 1000.0))
        elif t == 'STATUSTEXT':
            self.text_log.append((now, msg.text))
        elif t == 'EKF_STATUS_REPORT':
            self.ekf_log.append((now, msg.flags))
        elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
            self.hb_log.append((now, mavutil.mode_string_v10(msg),
                                bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)))
        elif t == 'COMMAND_ACK':
            self.ack_log.append((now, msg.command, msg.result))

    def offset(self):
        """wall - fc_boot_s, median over the last samples."""
        xs = sorted(w - b for w, b in list(self.boot))
        return xs[len(xs) // 2] if xs else None

    def request_mode(self, name):
        self.req_log.append((time.time(), f"MODE {name}"))
        self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_NUM[name])

    def request_src(self, n):
        self.req_log.append((time.time(), f"SRC {n}"))
        self.cmd(MAV_CMD_SET_EKF_SOURCE_SET, n)

    def set_param(self, name, value):
        self.req_log.append((time.time(), f"PARAM {name}={value}"))
        self.m.mav.param_set_send(1, 1, name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def cur_mode(self):
        return self.hb_log[-1][1] if self.hb_log else None


# ===========================================================================
# DataFlash
# ===========================================================================
DF_EV = {10: 'ARMED', 11: 'DISARMED', 17: 'LAND_COMPLETE_MAYBE', 18: 'LAND_COMPLETE',
         28: 'NOT_LANDED', 85: 'SRC_PRIMARY', 86: 'SRC_SECONDARY'}


def read_df(path):
    """MODE / EV / MSG / ERR(changes only) / CTUN(alt) of a .BIN, times in
    FC boot seconds."""
    from pymavlink import DFReader
    log = DFReader.DFReader_binary(path, zero_time_base=True)
    out = {'mode': [], 'ev': [], 'msg': [], 'err': [], 'ctun': []}
    last_err = None
    while True:
        m = log.recv_match(type=['MODE', 'EV', 'MSG', 'ERR', 'CTUN'])
        if m is None:
            break
        t = m.TimeUS / 1e6
        ty = m.get_type()
        if ty == 'MODE':
            out['mode'].append((t, COPTER_MODES.get(m.ModeNum, str(m.ModeNum)),
                                REASONS.get(m.Rsn, str(m.Rsn))))
        elif ty == 'EV':
            out['ev'].append((t, DF_EV.get(m.Id, str(m.Id))))
        elif ty == 'MSG':
            out['msg'].append((t, m.Message))
        elif ty == 'ERR':
            if (m.Subsys, m.ECode) != last_err:
                last_err = (m.Subsys, m.ECode)
                out['err'].append((t, m.Subsys, m.ECode))
        else:
            out['ctun'].append((t, m.Alt))
    return out


# ===========================================================================
# Parent run
# ===========================================================================
def tail_phase(path, since_idx):
    """New lines of phase.txt after line index `since_idx`: [(wall, kind, text)]."""
    try:
        lines = open(path).read().splitlines()
    except OSError:
        return [], since_idx
    out = []
    for ln in lines[since_idx:]:
        p = ln.split(' ', 2)
        if len(p) >= 2:
            try:
                out.append((float(p[0]), p[1], p[2] if len(p) > 2 else ''))
            except ValueError:
                pass
    return out, len(lines)


def run_parent(a):
    wd = os.path.abspath(a.workdir)
    os.makedirs(wd, exist_ok=True)
    spec = SCENARIOS[a.scenario]
    phase = a.phase if a.phase else spec['phases'][0]
    res = {'scenario': a.scenario, 'phase': phase, 'instance': a.instance,
           'pilot_mode': a.pilot_mode, 'at_s': a.at_s, 'observe_s': a.observe,
           'status': 'started', 'notes': [], 'invariants': {}}
    port = 5760 + 10 * a.instance
    ap = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(a.sitl))))
    base = os.path.join(ap, 'Tools', 'autotest', 'default_params', 'copter.parm')
    extra = os.path.join(wd, 'nova_extra.parm')
    open(extra, 'w').write(std.EXTRA_PARAMS)
    home = f"{std.HOME[0]},{std.HOME[1]},{std.HOME[2]},0"
    sitl_wd = os.path.join(wd, 'sitl')
    os.makedirs(sitl_wd, exist_ok=True)
    phase_file = os.path.join(wd, 'phase.txt')
    tx_file = os.path.join(wd, 'tx.txt')
    for f in (phase_file, tx_file):
        open(f, 'w').close()
    sitl = subprocess.Popen([a.sitl, '--model', '+', '--speedup', '1', '-I', str(a.instance),
                             '--home', home, '--defaults', f"{base},{extra}", '-w'],
                            cwd=sitl_wd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    comp = None
    comp_out = open(os.path.join(wd, 'companion.log'), 'w')
    pilot = None
    inj = {}                      # injection facts (wall times)
    facts = {}
    try:
        time.sleep(3.0)
        pilot = Observer(f"tcp:127.0.0.1:{port}")
        pilot.start()
        cargs = [sys.executable, '-u', os.path.abspath(__file__), '--role', 'companion',
                 '--port', str(port), '--phase-file', phase_file, '--tx-file', tx_file,
                 '--hang-s', str(a.hang_s)]
        if a.scenario in ('F6a', 'F6c'):
            cargs += ['--hang-main-at', phase]
        if a.scenario == 'F6b':
            cargs += ['--hang-supervisor-at', phase]
        comp = subprocess.Popen(cargs, stdout=comp_out, stderr=subprocess.STDOUT, cwd=REPO)

        # --- the pilot: GPS, arm in GUIDED, takeoff to 6 m, LOITER, AUX up ---
        t0 = time.monotonic()
        while pilot.origin is None or pilot.gps is None:
            if time.monotonic() - t0 > 120:
                raise RuntimeError("SITL without GPS / origin")
            pilot.cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                      mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
            time.sleep(1.0)
        time.sleep(15.0)
        pilot.m.set_mode(pilot.m.mode_mapping()['GUIDED'])
        pilot.rc[2] = 1000
        for _ in range(30):
            pilot.m.arducopter_arm()
            time.sleep(1.0)
            if pilot.armed:
                break
        if not pilot.armed:
            raise RuntimeError(f"did not arm: {pilot.texts[-3:]}")
        pilot.rc[2] = 1500
        pilot.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 6.0)
        t0 = time.monotonic()
        while (pilot.truth() or (0, 0, 0))[2] < 5.7 and time.monotonic() - t0 < 40:
            time.sleep(0.2)
        pilot.m.set_mode(pilot.m.mode_mapping()['LOITER'])
        time.sleep(4.0)
        pilot.rc[7] = 1000
        aux_up_at = time.monotonic() + 1.5
        t_start = time.monotonic()
        idx = 0
        phases_seen = []
        done = False
        while not done:
            now = time.monotonic()
            if aux_up_at is not None and now >= aux_up_at:
                pilot.rc[7] = 2000
                aux_up_at = None
            if comp.poll() is not None:
                raise RuntimeError(f"the companion died by itself (rc {comp.returncode})")
            lines, idx = tail_phase(phase_file, idx)
            for w, kind, text in lines:
                if kind == 'PHASE':
                    phases_seen.append((w, text))
                    if text == 'ENGAGE':
                        pilot.feed_vpe = False      # from here, OUR estimates only
                    if phase is not None and text == phase and 'wall' not in inj:
                        inj['wall'] = time.time()   # as close to the phase line as we get
                        inj['phase_line_wall'] = w
                        done = True
                        break
            if not done and now - t_start > a.phase_timeout:
                res['status'] = 'phase_not_reached'
                res['phases_seen'] = phases_seen
                raise RuntimeError(f"phase {phase} not reached in {a.phase_timeout:.0f} s "
                                   f"(seen: {[p for _, p in phases_seen]})")
            if phase is None and phases_seen and phases_seen[-1][1] in ('DONE', 'ABORT'):
                inj['wall'] = time.time()
                done = True
            if not done:
                time.sleep(0.01)
            if phase is None and now - t_start > a.phase_timeout:
                break
        if 'wall' not in inj:
            inj['wall'] = time.time()
        if a.delay_s > 0:
            time.sleep(a.delay_s)
            inj['wall'] = time.time()
            res['delay_after_phase_s'] = a.delay_s

        # ---- the fault, then the scripted actions -------------------------
        sc = a.scenario
        script = []                       # (t_rel_s, label, fn)
        inj['truth'] = pilot.truth()
        inj['mode'] = pilot.cur_mode()
        inj['armed'] = pilot.armed
        if sc in ('F1', 'F4', 'F7'):
            os.kill(comp.pid, signal.SIGKILL)
            facts['killed_at'] = time.time()
        elif sc in ('F2', 'F2c'):
            os.kill(comp.pid, signal.SIGSTOP)
            facts['stopped_at'] = time.time()

            def cont():
                os.kill(comp.pid, signal.SIGCONT)
                facts['cont_at'] = time.time()
            script.append((a.freeze_s, 'SIGCONT', cont))
        elif sc == 'F3':
            pilot.request_mode(a.pilot_mode)
            facts['pilot_req_at'] = time.time()
        elif sc == 'F5a':
            pilot.rc_on = False
            pilot.set_param('SIM_RC_FAIL', 1)
            facts['rcfail_at'] = time.time()
        elif sc == 'F5b':
            os.kill(comp.pid, signal.SIGKILL)
            facts['killed_at'] = time.time()

            def rcfail():
                pilot.rc_on = False
                pilot.set_param('SIM_RC_FAIL', 1)
                facts['rcfail_at'] = time.time()
            script.append((1.0, 'RCFAIL', rcfail))
        elif sc in ('F6a', 'F6b', 'F6c'):
            # the injection is the hang itself: wait for the companion's line
            t_w = time.time()
            while time.time() - t_w < 15:
                lines, idx = tail_phase(phase_file, idx)
                hit = [w for w, k, t in lines if k == 'HANG' and 'start' in t]
                if hit:
                    inj['wall'] = hit[0]
                    facts['hang_start'] = hit[0]
                    break
                time.sleep(0.02)
            else:
                res['notes'].append('the hang never started')
            inj['truth'] = pilot.truth()
            inj['mode'] = pilot.cur_mode()
        if sc in ('F6c', 'F2c'):
            script.append((a.at_s, f"PILOT {a.pilot_mode}",
                           lambda: pilot.request_mode(a.pilot_mode)))
        if sc == 'F4':
            if a.pilot_mode != 'NONE':
                script.append((a.at_s, f"PILOT {a.pilot_mode}",
                               lambda: pilot.request_mode(a.pilot_mode)))
            t_rec = a.at_s + 10.0

            def rec_src():
                pilot.request_src(1)
                facts['src1_at'] = time.time()
            script.append((t_rec, 'SRC1', rec_src))

            def rec_loiter():
                if pilot.cur_mode() != 'LOITER' and 'loiter_ok_at' not in facts:
                    pilot.request_mode('LOITER')
                    facts.setdefault('loiter_tries', []).append(time.time())
                elif pilot.cur_mode() == 'LOITER' and 'loiter_ok_at' not in facts:
                    facts['loiter_ok_at'] = time.time()
            for k in range(1, 26):
                script.append((t_rec + k, 'LOITER?', rec_loiter))
        script.sort(key=lambda x: x[0])
        t_inj = inj['wall']
        end = t_inj + a.observe
        si = 0
        while time.time() < end:
            while si < len(script) and time.time() >= t_inj + script[si][0]:
                try:
                    script[si][2]()
                except Exception as e:                      # noqa: BLE001
                    res['notes'].append(f"script {script[si][1]}: {e}")
                si += 1
            time.sleep(0.05)
        res['status'] = 'observed'
    except Exception as e:                                  # noqa: BLE001
        if res['status'] not in ('phase_not_reached',):
            res['status'] = 'setup_failed'
        res['error'] = f"{type(e).__name__}: {e}"
    finally:
        comp_alive = comp is not None and comp.poll() is None
        res['companion_alive_at_end'] = comp_alive
        if comp is not None and comp_alive:
            try:
                os.kill(comp.pid, signal.SIGCONT)
            except OSError:
                pass
            comp.kill()
        comp_out.close()
        if pilot is not None:
            pilot.stop.set()
        time.sleep(0.3)
        sitl.terminate()
        try:
            sitl.wait(10)
        except subprocess.TimeoutExpired:
            sitl.kill()

    bins = sorted(glob.glob(os.path.join(sitl_wd, 'logs', '*.BIN')), key=os.path.getmtime)
    df = None
    if bins:
        try:
            df = read_df(bins[-1])
            res['bin'] = os.path.relpath(bins[-1], wd)
        except Exception as e:                              # noqa: BLE001
            res['notes'].append(f"DataFlash unreadable: {e}")
    if pilot is not None and 'wall' in inj and df is not None:
        analyse(a, res, pilot, df, inj, facts, tx_file, phase_file, comp_alive)
    # dump the raw pilot-link record
    if pilot is not None:
        with open(os.path.join(wd, 'truth.csv'), 'w') as f:
            f.write('wall,north,east,agl\n')
            for i, (w, n, e, g) in enumerate(pilot.truth_log):
                if i % 10 == 0:
                    f.write(f"{w:.2f},{n:.2f},{e:.2f},{g:.2f}\n")
        with open(os.path.join(wd, 'pilot_link.json'), 'w') as f:
            json.dump({'hb': [(round(w, 2), m, ar) for w, m, ar in pilot.hb_log],
                       'texts': [(round(w, 2), t) for w, t in pilot.text_log],
                       'acks': pilot.ack_log, 'reqs': pilot.req_log,
                       'ekf': [(round(w, 2), fl) for w, fl in pilot.ekf_log][::5]},
                      f)
    ok = res.get('status') == 'observed' and all(
        v['pass'] for v in res['invariants'].values()) and not res.get('error')
    res['ok'] = bool(ok)
    json.dump(res, open(os.path.join(wd, 'result.json'), 'w'), indent=1, default=str)
    print(summary_line(res), flush=True)
    return 0 if ok else 1


# ===========================================================================
# Analysis
# ===========================================================================
def hdist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def parse_tx(path):
    out = []
    try:
        for ln in open(path).read().splitlines():
            p = ln.split(' ', 2)
            if len(p) >= 2:
                out.append((float(p[0]), p[1], p[2] if len(p) > 2 else ''))
    except (OSError, ValueError):
        pass
    return out


def analyse(a, res, pilot, df, inj, facts, tx_file, phase_file, comp_alive):
    off = pilot.offset()
    if off is None:
        res['notes'].append('no FC clock offset (no ATTITUDE)')
        return
    t_inj = inj['wall']
    wall = lambda t: t + off                                  # noqa: E731
    rel = lambda w: round(w - t_inj, 2)                       # noqa: E731
    obs_end = t_inj + a.observe
    truth = [x for x in pilot.truth_log if x[0] <= obs_end + 0.5]
    after = [x for x in truth if x[0] >= t_inj]
    m = res.setdefault('m', {})

    # --- FC mode timeline (DataFlash, with the reason) --------------------
    tl = [(round(wall(t) - t_inj, 2), n, r) for t, n, r in df['mode']
          if t_inj - 8 <= wall(t) <= obs_end + 2]
    res['mode_timeline_rel_s'] = tl
    m['mode_at_injection'] = inj.get('mode')
    ev = [(round(wall(t) - t_inj, 2), n) for t, n in df['ev'] if t_inj - 3 <= wall(t) <= obs_end + 2]
    res['events_rel_s'] = ev
    keys = ('EKF', 'ailsafe', 'Land', 'LAND', 'isarm', 'rmed', 'Crash', 'crash', 'RTL', 'Lost',
            'lost', 'GPS', 'extern', 'Radio', 'RC', 'Mode', 'ode')
    res['msgs_rel_s'] = [(round(wall(t) - t_inj, 2), s) for t, s in df['msg']
                         if t_inj - 3 <= wall(t) <= obs_end + 2 and any(k in s for k in keys)]
    res['err_rel_s'] = [(round(wall(t) - t_inj, 2), s, c) for t, s, c in df['err']
                        if t_inj - 3 <= wall(t) <= obs_end + 2]

    # --- EKF position loss (pilot link, EKF_STATUS_REPORT) ----------------
    ek = [(w, f) for w, f in pilot.ekf_log if t_inj <= w <= obs_end]
    loss = next((w for w, f in ek if not (f & EKF_POS_HORIZ_REL)), None)
    m['t_ekf_pos_loss_s'] = None if loss is None else rel(loss)
    regain = None
    if loss is not None:
        regain = next((w for w, f in ek if w > loss and (f & EKF_POS_HORIZ_REL)), None)
    m['t_ekf_pos_regain_s'] = None if regain is None else rel(regain)
    # --- failsafe LAND (EKF) ----------------------------------------------
    fs = next(((wall(t) - t_inj, n, r) for t, n, r in df['mode']
               if wall(t) >= t_inj and r == 'EKF_FAILSAFE'), None)
    m['t_ekf_failsafe_mode_s'] = None if fs is None else round(fs[0], 2)
    m['ekf_failsafe_mode'] = None if fs is None else fs[1]
    # --- touchdown / disarm -----------------------------------------------
    land = next((wall(t) - t_inj for t, n in df['ev'] if n == 'LAND_COMPLETE' and wall(t) >= t_inj), None)
    dis = next((wall(t) - t_inj for t, n in df['ev'] if n == 'DISARMED' and wall(t) >= t_inj), None)
    m['t_land_complete_s'] = None if land is None else round(land, 2)
    m['t_disarm_s'] = None if dis is None else round(dis, 2)
    m['armed_at_end'] = bool(pilot.hb_log[-1][2]) if pilot.hb_log else None
    # --- geometry (truth) ---------------------------------------------------
    p0 = inj.get('truth')
    if p0 and after:
        m['agl_at_injection_m'] = round(p0[2], 2)
        m['max_agl_after_m'] = round(max(x[3] for x in after), 2)
        m['max_horiz_excursion_m'] = round(max(hdist(p0, (x[1], x[2])) for x in after), 2)
        t_end_ref = (t_inj + land) if land is not None else after[-1][0]
        at_land = min(after, key=lambda x: abs(x[0] - t_end_ref))
        m['drift_to_touchdown_m'] = round(hdist(p0, (at_land[1], at_land[2])), 2)
        m['drift_to_touchdown_is_landing'] = land is not None
        m['final_agl_m'] = round(after[-1][3], 2)
        m['final_dist_home_m'] = round(math.hypot(after[-1][1], after[-1][2]), 2)
        m['dist_home_at_injection_m'] = round(math.hypot(p0[0], p0[1]), 2)
        m['min_dist_home_m'] = round(min(math.hypot(x[1], x[2]) for x in after), 2)
        m['final_offset_from_marker_m'] = round(hdist(after[-1][1:3], std.MARKER_NE), 2)
    ctun = [(wall(t) - t_inj, al) for t, al in df['ctun'] if t_inj <= wall(t) <= obs_end]
    m['max_ctun_alt_after_m'] = round(max((al for _, al in ctun), default=float('nan')), 2) if ctun else None
    # --- mode timeline from the pilot link ---------------------------------
    hb = [(w, md, ar) for w, md, ar in pilot.hb_log if t_inj - 1 <= w <= obs_end]
    chg = []
    for w, md, ar in hb:
        if not chg or chg[-1][1] != md:
            chg.append((rel(w), md))
    res['hb_mode_changes_rel_s'] = chg
    # --- the companion's own TX -------------------------------------------
    tx = parse_tx(tx_file)
    phase_lines = parse_tx(phase_file)
    res['companion_phases_rel_s'] = [(rel(w), t) for w, k, t in phase_lines if k == 'PHASE' and w >= t_inj - 30]
    res['companion_hang_lines'] = [(rel(w), t) for w, k, t in phase_lines if k == 'HANG']
    W = facts.get('cont_at')
    hang_end = next((w for w, k, t in phase_lines if k == 'HANG' and 'end' in t), None)
    if hang_end is not None:
        W = hang_end
    res['wake_at_rel_s'] = None if W is None else rel(W)
    kind = collections.Counter(k for w, k, t in tx if w >= t_inj)
    res['companion_tx_after_injection'] = dict(kind)
    res['companion_tx_cmds_rel_s'] = [(rel(w), k, t) for w, k, t in tx
                                      if w >= t_inj - 1 and k in ('MODE', 'SRC', 'CMD', 'PARAM')]
    if W is not None:
        wk = [(rel(w), k, t) for w, k, t in tx if w >= W and k in ('MODE', 'SRC', 'CMD', 'POSTGT')]
        res['companion_tx_after_wake'] = {
            'n_vpe': sum(1 for w, k, t in tx if w >= W and k == 'VPE'),
            'n_postgt': sum(1 for w, k, t in tx if w >= W and k == 'POSTGT'),
            'cmds': [x for x in wk if x[1] != 'POSTGT'],
        }
        post = [x for x in after if x[0] >= W]
        if post:
            m['moved_after_wake_m'] = round(max(hdist(post[0][1:3], x[1:3]) for x in post), 2)
    # --- scenario specifics -------------------------------------------------
    sc = a.scenario
    if sc in ('F3', 'F6c'):
        t_req = facts.get('pilot_req_at', t_inj + (a.at_s if sc == 'F6c' else 0))
        want = a.pilot_mode
        t_set = next((wall(t) for t, n, r in df['mode'] if n == want and wall(t) >= t_req - 0.2), None)
        m['t_pilot_mode_taken_s'] = None if t_set is None else round(t_set - t_req, 2)
        t_src1 = next((wall(t) for t, n in df['ev'] if n == 'SRC_PRIMARY' and wall(t) >= t_req), None)
        m['t_src1_restored_s'] = None if t_src1 is None else round(t_src1 - t_req, 2)
        win_end = (t_set if t_set else t_req) + (a.observe if sc == 'F6c' else 20.0)
        after_set = [(round(wall(t) - t_req, 2), n, r) for t, n, r in df['mode']
                     if t_set and wall(t) > t_set + 0.01 and wall(t) <= win_end]
        res['mode_changes_after_pilot_20s'] = after_set
        m['n_mode_changes_after_pilot'] = len(after_set)
    if sc == 'F4':
        t_src1 = facts.get('src1_at')
        if t_src1:
            m['src1_sent_rel_s'] = rel(t_src1)
            ack = next((r for w, c, r in pilot.ack_log if c == MAV_CMD_SET_EKF_SOURCE_SET and w >= t_src1), None)
            m['src1_ack'] = ack
            gps_ok = next((w for w, f in pilot.ekf_log if w >= t_src1 and f & EKF_POS_HORIZ_ABS), None)
            m['t_gps_pos_after_src1_s'] = None if gps_ok is None else round(gps_ok - t_src1, 2)
            lo = facts.get('loiter_ok_at')
            m['t_loiter_after_src1_s'] = None if lo is None else round(lo - t_src1, 2)
            m['loiter_tries'] = len(facts.get('loiter_tries', []))
        # which pilot modes were accepted (ACK + FC mode)
        acc = []
        for w, what in pilot.req_log:
            if what.startswith('MODE'):
                nm = what.split()[1]
                ack = next((r for ww, c, r in pilot.ack_log
                            if c == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ww >= w), None)
                took = any(n == nm and w - 0.2 <= wall(t) <= w + 3 for t, n, r in df['mode'])
                acc.append((rel(w), nm, ack, took))
        res['pilot_mode_requests'] = acc
    if sc in ('F5a', 'F5b'):
        t_rc = facts.get('rcfail_at', t_inj)
        rtl = next(((wall(t) - t_rc, n, r) for t, n, r in df['mode']
                    if wall(t) >= t_rc and r == 'RADIO_FAILSAFE'), None)
        m['t_rc_failsafe_mode_s'] = None if rtl is None else round(rtl[0], 2)
        m['rc_failsafe_mode'] = None if rtl is None else rtl[1]
        src1 = next((wall(t) - t_rc for t, n in df['ev'] if n == 'SRC_PRIMARY' and wall(t) >= t_rc), None)
        m['t_src1_restored_s'] = None if src1 is None else round(src1, 2)
        if facts.get('rcfail_at'):
            m['rcfail_rel_s'] = rel(facts['rcfail_at'])
    if sc in ('F6a', 'F6b', 'F6c'):
        src1 = next((wall(t) - t_inj for t, n in df['ev'] if n == 'SRC_PRIMARY' and wall(t) >= t_inj), None)
        m['t_src1_after_hang_s'] = None if src1 is None else round(src1, 2)
        lo = next((wall(t) - t_inj for t, n, r in df['mode'] if n in ('LOITER', 'ALT_HOLD') and wall(t) >= t_inj), None)
        m['t_loiter_after_hang_s'] = None if lo is None else round(lo, 2)
        first_safety = next((w for w, k, t in tx if w >= t_inj and k in ('SRC', 'MODE')), None)
        m['t_first_companion_safety_cmd_s'] = None if first_safety is None else round(first_safety - t_inj, 2)
    check_invariants(a, res, pilot, df, inj, facts, tx, comp_alive, off, W)


def check_invariants(a, res, pilot, df, inj, facts, tx, comp_alive, off, W):
    sc = a.scenario
    t_inj = inj['wall']
    wall = lambda t: t + off                                  # noqa: E731
    horizon = min(60.0, a.observe)
    t_h = t_inj + horizon
    hb = [x for x in pilot.hb_log if x[0] <= t_h]
    tr = [x for x in pilot.truth_log if t_inj <= x[0] <= t_h]
    inv = res['invariants']

    # ---------------- I1 ----------------
    if hb and tr:
        _, mode_end, armed_end = hb[-1]
        agl_end = tr[-1][3]
        landed = (not armed_end) or agl_end < 0.4
        last = [x for x in tr if x[0] >= tr[-1][0] - 6.0]
        d_agl = last[0][3] - last[-1][3]
        d_home = math.hypot(last[0][1], last[0][2]) - math.hypot(last[-1][1], last[-1][2])
        src_last = next((n for t, n in reversed(df['ev']) if n in ('SRC_PRIMARY', 'SRC_SECONDARY')
                         and wall(t) <= t_h), None)
        pilot_modes = {w.split()[1] for _, w in pilot.req_log if w.startswith('MODE')}
        # the companion is "commanding" only if alive, not stopped / hung at the horizon
        hung_now = False
        if sc in ('F6a', 'F6b', 'F6c'):
            hl = [(w, t) for w, k, t in parse_tx(os.path.join(a.workdir, 'phase.txt'))
                  if k == 'HANG']
            hung_now = bool(hl) and not any('end' in t for _, t in hl)
        commanded = comp_alive and sc not in ('F1', 'F4', 'F5b') and not hung_now
        if landed:
            cls, ok = 'LANDED', True
        elif mode_end in ('LAND', 'RTL', 'SMART_RTL'):
            prog = d_agl >= 0.3 or d_home >= 1.0
            cls, ok = ('FAILSAFE_PROGRESS' if prog else 'FAILSAFE_NO_PROGRESS'), prog
        elif mode_end == 'GUIDED':
            cls, ok = ('GUIDED_COMMANDED' if commanded else 'STUCK_GUIDED'), commanded
        elif mode_end in pilot_modes:
            cls, ok = 'PILOT_MODE', True
        elif mode_end in ('LOITER', 'ALT_HOLD', 'STABILIZE', 'POSHOLD') and src_last != 'SRC_SECONDARY':
            cls, ok = 'HANDBACK', True
        else:
            cls, ok = f'UNEXPLAINED_{mode_end}', False
        inv['I1'] = {'pass': ok, 'class': cls, 'mode_end': mode_end, 'armed_end': armed_end,
                     'agl_end_m': round(agl_end, 2), 'last6s_d_agl_m': round(d_agl, 2),
                     'last6s_d_home_m': round(d_home, 2), 'last_src_event': src_last,
                     'horizon_s': horizon}
    else:
        inv['I1'] = {'pass': False, 'class': 'NO_DATA'}

    # ---------------- I2 ----------------
    if sc == 'F7':
        chg = [(round(wall(t) - t_inj, 2), n, r) for t, n, r in df['mode']
               if t_inj < wall(t) <= t_inj + 20.0]
        src = [(round(wall(t) - t_inj, 2), n) for t, n in df['ev']
               if n in ('SRC_PRIMARY', 'SRC_SECONDARY') and t_inj < wall(t) <= t_inj + 20.0]
        inv['I2'] = {'pass': not chg and not any(n == 'SRC_SECONDARY' for _, n in src),
                     'mode_changes_20s': chg, 'src_events_20s': src}
    elif sc in ('F3', 'F4', 'F6c', 'F2c'):
        reqs = [(w, x.split()[1]) for w, x in pilot.req_log if x.startswith('MODE')]
        req_times = [w for w, _ in reqs]
        bad, fsl = [], []
        for w_req, nm in reqs[:1] if sc in ('F3', 'F6c', 'F2c') else reqs:
            t_set = next((wall(t) for t, n, r in df['mode'] if n == nm and wall(t) >= w_req - 0.2), None)
            if t_set is None:
                continue
            for t, n, r in df['mode']:
                wt = wall(t)
                if t_set + 0.01 < wt <= t_set + (a.observe if sc in ('F6c', 'F2c') else 20.0):
                    if any(abs(wt - rt) < 1.5 for rt in req_times):
                        continue                              # the pilot's own request
                    (fsl if r in FAILSAFE_REASONS else bad).append((round(wt - t_inj, 2), n, r))
        companion_mode_cmds = [(round(w - t_inj, 2), t) for w, k, t in tx if k == 'MODE' and w >= t_inj]
        inv['I2'] = {'pass': not bad, 'unrequested_other': bad, 'failsafe_changes': fsl,
                     'companion_mode_cmds_after_injection': companion_mode_cmds}
    else:
        inv['I2'] = {'pass': True, 'note': 'not applicable'}

    # ---------------- I3 ----------------
    if W is not None and sc in ('F2', 'F2c', 'F6a', 'F6b', 'F6c'):
        guided_after = [(round(wall(t) - t_inj, 2), r) for t, n, r in df['mode']
                        if n == 'GUIDED' and wall(t) > W]
        set_guided = [(round(w - t_inj, 2)) for w, k, t in tx if k == 'MODE' and w >= W
                      and t.startswith('4 ')]
        n_pt = sum(1 for w, k, t in tx if w >= W and k == 'POSTGT')
        inv['I3'] = {'pass': not guided_after and not set_guided,
                     'guided_entries_after_wake': guided_after,
                     'companion_set_guided_after_wake': set_guided,
                     'position_targets_sent_after_wake': n_pt,
                     'wake_rel_s': round(W - t_inj, 2)}
    else:
        inv['I3'] = {'pass': True, 'note': 'not applicable'}


def summary_line(res):
    i = res.get('invariants', {})
    iv = ' '.join(f"{k}={'ok' if v['pass'] else 'FAIL'}" for k, v in sorted(i.items()))
    m = res.get('m', {})
    keys = ('t_ekf_pos_loss_s', 't_ekf_failsafe_mode_s', 't_land_complete_s', 't_disarm_s',
            'drift_to_touchdown_m', 'max_agl_after_m')
    ms = ' '.join(f"{k}={m.get(k)}" for k in keys if k in m)
    cls = i.get('I1', {}).get('class')
    return (f"[{res['scenario']}/{res.get('phase')}"
            f"{'/' + res['pilot_mode'] if res.get('scenario') in ('F3', 'F4') else ''}] "
            f"{res.get('status')} {iv} I1class={cls} {ms}"
            + (f" ERROR={res['error']}" if res.get('error') else ''))


# ===========================================================================
# Campaign driver (resumable across Bash calls)
# ===========================================================================
def run_campaign(a):
    outdir = os.path.abspath(a.outdir)
    os.makedirs(outdir, exist_ok=True)
    only = set(a.only.split(',')) if a.only else None
    runs = [(rid, argv) for rid, argv in campaign_matrix()
            if (only is None or any(rid.startswith(o) for o in only))
            and not os.path.exists(os.path.join(outdir, rid, 'result.json'))]
    # longest first: late phases need the longest setup
    order = {'GATE_SEARCH': 4, 'RISEUP': 0, 'GROUND_HOLD': 1, 'TOUCHDOWN_DESCENT': 2, 'DESCEND': 3, 'MOVE': 4}
    runs.sort(key=lambda r: order.get(r[1][r[1].index('--phase') + 1] if '--phase' in r[1] else 'MOVE', 4))
    insts = [int(x) for x in a.instances.split(',')]
    q = list(runs)
    lock = threading.Lock()
    deadline = time.time() + a.budget_s
    results = []

    def worker(inst):
        while True:
            with lock:
                if not q or time.time() + a.est_s > deadline:
                    return
                rid, argv = q.pop(0)
            wd = os.path.join(outdir, rid)
            os.makedirs(wd, exist_ok=True)
            cmd = [sys.executable, os.path.abspath(__file__)] + argv + [
                '--instance', str(inst), '--workdir', wd]
            t0 = time.time()
            try:
                with open(os.path.join(wd, 'stdout.txt'), 'w') as so:
                    p = subprocess.run(cmd, stdout=so, stderr=subprocess.STDOUT,
                                       timeout=a.run_timeout_s)
                rc = p.returncode
            except subprocess.TimeoutExpired:
                rc = 'timeout'
                subprocess.run(['pkill', '-f', '--', f"-I {inst} --home"])
            with lock:
                results.append((rid, rc, round(time.time() - t0)))
                print(f"[campaign] {rid} inst {inst}: rc={rc} {round(time.time() - t0)} s",
                      flush=True)
    ths = [threading.Thread(target=worker, args=(i,)) for i in insts]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    print(f"[campaign] this call: {len(results)} runs; pending: {len(q)}", flush=True)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--list', action='store_true')
    p.add_argument('--scenario', choices=list(SCENARIOS))
    p.add_argument('--phase')
    p.add_argument('--instance', type=int, default=8)
    p.add_argument('--workdir')
    p.add_argument('--observe', type=float, default=45.0)
    p.add_argument('--sitl', default=SITL_DEFAULT)
    p.add_argument('--pilot-mode', default='ALT_HOLD')
    p.add_argument('--at-s', type=float, default=3.0)
    p.add_argument('--hang-s', type=float, default=30.0)
    p.add_argument('--delay-s', type=float, default=0.0,
                   help='inject this long AFTER the companion reports the phase')
    p.add_argument('--freeze-s', type=float, default=15.0, help='F2: SIGSTOP duration')
    p.add_argument('--phase-timeout', type=float, default=240.0)
    p.add_argument('--role', choices=('parent', 'companion'), default='parent')
    # companion-only
    p.add_argument('--port', type=int)
    p.add_argument('--phase-file')
    p.add_argument('--tx-file')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--hang-main-at')
    p.add_argument('--hang-supervisor-at')
    # campaign
    p.add_argument('--campaign', action='store_true')
    p.add_argument('--instances', default='8,9,10,11,12')
    p.add_argument('--outdir')
    p.add_argument('--only')
    p.add_argument('--budget-s', type=float, default=520.0)
    p.add_argument('--est-s', type=float, default=250.0)
    p.add_argument('--run-timeout-s', type=float, default=420.0)
    a = p.parse_args(argv)
    if a.list:
        for k, v in SCENARIOS.items():
            print(f"{k:4s} phases={v['phases']}  {v['desc']}")
        print('\ncampaign matrix:')
        for rid, args in campaign_matrix():
            print(f"  {rid:28s} {' '.join(args)}")
        return 0
    if a.role == 'companion':
        return companion_main(a)
    if a.campaign:
        return run_campaign(a)
    if not a.scenario or not a.workdir:
        p.error('--scenario and --workdir are required')
    if a.pilot_mode not in MODE_NUM and a.pilot_mode != 'NONE':
        p.error(f"unknown --pilot-mode {a.pilot_mode}")
    return run_parent(a)


if __name__ == '__main__':
    sys.exit(main())
