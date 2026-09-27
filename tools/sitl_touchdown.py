#!/usr/bin/env python3
"""
NOVA - ZDC 2026
The complete touchdown sequence in SITL ArduCopter 4.5.7, with OUR code,
checked on the DataFlash log (PROMPT_CLAUDE_CODE_TOUCHDOWN, point 6).

    python3 tools/sitl_touchdown.py --sitl ~/ardupilot-4.5.7/build/sitl/bin/arducopter

What runs:
- SITL Copter-4.5.7 (quad '+'), our ExtNav parameters (SRC2 = vision
  position, baro height, compass yaw; VISO_TYPE 1; DISARM_DELAY 20;
  PILOT_THR_BHV 0).
- The onboard code as on the vehicle: Vehicle(threaded=True) (the MAVLink
  I/O thread), ExtNavLanding, ExtNavSupervisor in its SupervisorThread over
  a VehicleView, EkfSourceManager, HandoverGate, config/nova.json
  tolerances and touchdown settings. Only the camera is replaced: a
  synthetic detector turns the SIMULATED truth (GPS of SITL + attitude)
  into Detection angles, at ~30% of 10 fps, with the 73x45 degree field of
  the crop1280 preset and the blind zone under 0.61 m (the 0.48 m marker
  overflows the frame there).
- A "pilot" on a second link: GUIDED takeoff to 6 m, LOITER, sticks and
  throttle centred, AUX8 low then high (the rising edge asks for handover).

What is checked on the DataFlash log (logs/*.BIN of the SITL run):
  contact           EV LAND_COMPLETE (18) after the descent began
  >= 1 s stable     LAND_COMPLETE -> NOT_LANDED (28) at least 1.0 s
  no disarm         no EV DISARMED (11) between the arming and the end
  climb >= 5 m      max CTUN.Alt after NOT_LANDED - CTUN.Alt at contact >= 5.0
  SRC2 only         EV SOURCES_SET_TO_SECONDARY (86) at ENGAGE,
                    TO_PRIMARY (85) at COMPLETE, and no "EKF3 ... is using
                    GPS" message between the two

Declared plainly: a desktop run, no camera, no wind, perfect truth. It
proves the sequence and the FC's side of it, not the detector.
"""

import argparse
import glob
import math
import os
import random
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from pymavlink import mavutil                                  # noqa: E402

from nova import config as nova_config                         # noqa: E402
from nova import extnav as ex                                   # noqa: E402
from nova.concurrency import Latest                             # noqa: E402
from nova.detection import Detection                            # noqa: E402
from nova.ekf_source import EkfSourceManager                    # noqa: E402
from nova.extnav_landing import ExtNavLanding, Phase, SRC2_PHASES  # noqa: E402
from nova.handover import HandoverGate                          # noqa: E402
from nova.rc import OverrideMonitor                             # noqa: E402
from nova.safety import ExtNavSupervisor                        # noqa: E402
from nova.supervisor_thread import SupervisorThread             # noqa: E402
from nova.vehicle import Vehicle                                # noqa: E402

HOME = (-35.363261, 149.165230, 584.0)
MARKER_NE = (2.0, 1.0)            # marker, metres north / east of home
EXTRA_PARAMS = """
DISARM_DELAY 20
PILOT_THR_BHV 0
FS_THR_ENABLE 1
WPNAV_ACCEL 150
WPNAV_SPEED 100
WPNAV_SPEED_DN 50
LAND_SPEED 50
EK3_SRC2_POSXY 6
EK3_SRC2_VELXY 0
EK3_SRC2_POSZ 1
EK3_SRC2_VELZ 0
EK3_SRC2_YAW 1
EK3_SRC_OPTIONS 0
VISO_TYPE 1
VISO_DELAY_MS 100
VISO_POS_M_NSE 0.5
"""
EV = {10: 'ARMED', 11: 'DISARMED', 17: 'LAND_COMPLETE_MAYBE', 18: 'LAND_COMPLETE',
      28: 'NOT_LANDED', 85: 'SRC_PRIMARY', 86: 'SRC_SECONDARY'}


class Pilot(threading.Thread):
    """The second link: the pilot's sticks (RC override at 10 Hz) and the
    simulated truth (GPS, attitude, EKF origin) for the synthetic camera."""

    def __init__(self, conn):
        super().__init__(name='pilot', daemon=True)
        self.m = mavutil.mavlink_connection(conn, source_system=255, source_component=190)
        self.m.wait_heartbeat(timeout=60)
        self.rc = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000]
        self.gps = self.origin = None
        self.att = (0.0, 0.0, 0.0)
        self.armed = False
        self.mode = None
        self.texts = []
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.feed_vpe = True          # before ENGAGE only: arming needs a healthy VisOdom
        for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 20),
                        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30)):
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)
        self.cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                 mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)

    def cmd(self, c, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(1, 1, c, 0, *p)

    def truth(self):
        """(north, east, agl) of the vehicle in the HOME frame, or None."""
        with self.lock:
            g = self.gps
        if g is None:
            return None
        lat, lon, alt = g
        R = 6378137.0
        n = math.radians(lat - HOME[0]) * R
        e = math.radians(lon - HOME[1]) * R * math.cos(math.radians(HOME[0]))
        return n, e, alt - HOME[2]

    def run(self):
        last_rc = 0.0
        while not self.stop.is_set():
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                t = msg.get_type()
                if t == 'GPS_RAW_INT':
                    with self.lock:
                        self.gps = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
                elif t == 'ATTITUDE':
                    self.att = (msg.roll, msg.pitch, msg.yaw)
                elif t == 'GPS_GLOBAL_ORIGIN':
                    self.origin = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
                elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = mavutil.mode_string_v10(msg)
                elif t == 'STATUSTEXT':
                    self.texts.append((time.monotonic(), msg.text))
            now = time.monotonic()
            if now - last_rc > 0.1:
                last_rc = now
                self.m.mav.rc_channels_override_send(1, 1, *self.rc)
                tr = self.truth()
                if self.feed_vpe and tr is not None and self.origin is not None:
                    # EKF-origin frame, before ENGAGE only (arming check)
                    R = 6378137.0
                    n0 = math.radians(HOME[0] - self.origin[0]) * R
                    e0 = math.radians(HOME[1] - self.origin[1]) * R * math.cos(math.radians(HOME[0]))
                    r, p, y = self.att
                    self.m.mav.vision_position_estimate_send(
                        int(now * 1e6), tr[0] + n0, tr[1] + e0, -tr[2], r, p, y)


class SyntheticCamera:
    """Detections from the truth: the marker's direction in the body frame
    (attitude of the same instant), at `rate` of 10 fps, blind below
    `blind_below_m`, inside a 73 x 45 degree field."""

    def __init__(self, pilot, rate=0.3, seed=1, blind_below_m=0.61,
                 half_fov_fwd_deg=36.5, half_fov_right_deg=22.6):
        self.p = pilot
        self.rate = rate
        self.rng = random.Random(seed)
        self.blind = blind_below_m
        self.hf = math.radians(half_fov_fwd_deg)
        self.hr = math.radians(half_fov_right_deg)
        self.next_t = None
        self.n = self.n_det = 0

    def poll(self, now):
        if self.next_t is None:
            self.next_t = now
        out = []
        while now >= self.next_t:
            self.next_t += 0.1
            self.n += 1
            tr = self.p.truth()
            if tr is None or self.rng.random() >= self.rate or tr[2] < self.blind:
                continue
            d = (MARKER_NE[0] - tr[0], MARKER_NE[1] - tr[1], tr[2])
            roll, pitch, yaw = self.p.att
            fwd, right, down = ex.ned_to_body(d, roll, pitch, yaw)
            if down <= 0.05:
                continue
            ax, ay = math.atan2(fwd, down), math.atan2(right, down)
            if abs(ax) > self.hf or abs(ay) > self.hr:
                continue
            self.n_det += 1
            out.append(Detection(t=now, angle_x=ax, angle_y=ay,
                                 distance_m=math.sqrt(sum(c * c for c in d)),
                                 marker_px=100.0, range_m=tr[2], fill=None,
                                 marker_yaw_deg=-math.degrees(yaw)))
        return out


def read_bin(path):
    """(events [(t, name)], msgs [(t, text)], ctun [(t, alt)]) from a .BIN."""
    from pymavlink import DFReader
    log = DFReader.DFReader_binary(path, zero_time_base=True)
    ev, msgs, ctun = [], [], []
    while True:
        m = log.recv_match(type=['EV', 'MSG', 'CTUN'])
        if m is None:
            break
        t = m.TimeUS / 1e6
        ty = m.get_type()
        if ty == 'EV':
            ev.append((t, EV.get(m.Id, str(m.Id))))
        elif ty == 'MSG':
            msgs.append((t, m.Message))
        else:
            ctun.append((t, m.Alt))
    return ev, msgs, ctun


def check_log(path, verbose=True):
    ev, msgs, ctun = read_bin(path)
    res = {'bin': os.path.basename(path)}

    def first(name, after=-1.0):
        return next((t for t, n in ev if n == name and t > after), None)
    t_arm = first('ARMED')
    t_src2 = first('SRC_SECONDARY', t_arm or -1)
    t_land = first('LAND_COMPLETE', t_src2 or -1)
    t_up = first('NOT_LANDED', t_land or -1)
    t_src1 = first('SRC_PRIMARY', t_up or -1)
    res['contact'] = t_land is not None
    res['ground_s'] = None if t_land is None or t_up is None else round(t_up - t_land, 2)
    res['stable_1s'] = res['ground_s'] is not None and res['ground_s'] >= 1.0
    dis = [t for t, n in ev if n == 'DISARMED' and t_arm is not None and t > t_arm]
    res['no_disarm'] = not dis
    alt_c = next((a for t, a in ctun if t_land is not None and t >= t_land), None)
    up = [a for t, a in ctun if t_up is not None and t >= t_up]
    res['climb_m'] = None if alt_c is None or not up else round(max(up) - alt_c, 2)
    res['climb_ok'] = res['climb_m'] is not None and res['climb_m'] >= 5.0
    gps = [x for t, x in msgs if t_src2 is not None and t_src2 < t < (t_src1 or 1e12)
           and 'is using GPS' in x]
    res['src2_then_src1'] = t_src2 is not None and t_src1 is not None
    res['no_gps_between'] = not gps
    res['extnav_msg'] = any('external nav' in x for t, x in msgs
                            if t_src2 is not None and t > t_src2)
    res['ok'] = all(res[k] for k in ('contact', 'stable_1s', 'no_disarm', 'climb_ok',
                                     'src2_then_src1', 'no_gps_between'))
    if verbose:
        tl = [(round(t, 2), n) for t, n in ev]
        print(f"[sitl] DataFlash {res['bin']}: evenimente {tl}")
    return res


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--sitl', default=os.path.expanduser(
        '~/ardupilot-4.5.7/build/sitl/bin/arducopter'))
    p.add_argument('--instance', type=int, default=5)
    p.add_argument('--workdir', default=None)
    p.add_argument('--timeout', type=float, default=300.0)
    p.add_argument('--seed', type=int, default=1)
    a = p.parse_args(argv)

    ap = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(a.sitl))))
    base = os.path.join(ap, 'Tools', 'autotest', 'default_params', 'copter.parm')
    wd = a.workdir or os.path.join(REPO, 'data', 'sitl_touchdown', time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(wd, exist_ok=True)
    extra = os.path.join(wd, 'nova_extra.parm')
    open(extra, 'w').write(EXTRA_PARAMS)
    home = f"{HOME[0]},{HOME[1]},{HOME[2]},0"
    proc = subprocess.Popen([a.sitl, '--model', '+', '--speedup', '1', '-I', str(a.instance),
                             '--home', home, '--defaults', f"{base},{extra}", '-w'],
                            cwd=wd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    events = []
    res = {}
    try:
        time.sleep(3.0)
        port = 5760 + 10 * a.instance
        pilot = Pilot(f"tcp:127.0.0.1:{port}")
        pilot.start()
        vehicle = Vehicle(f"tcp:127.0.0.1:{port + 2}", threaded=True).connect()
        cfg = nova_config.load()
        td, _w = nova_config.touchdown_settings(cfg)
        import nova_pi
        sm_cfg = nova_pi.extnav_config(cfg, 8, 1500)
        ov = OverrideMonitor(vehicle)
        gate = HandoverGate(vehicle, ov, autonomy_enabled=True, alt_min_m=1.0,
                            dist_max_m=None, detection_max_age_s=None)
        sup = ExtNavSupervisor(vehicle.view(), override=ov, verbose=True)
        est = ex.ExtNavEstimator(vehicle)
        ekf = EkfSourceManager(vehicle, faze=SRC2_PHASES, verbose=True)

        def on_event(n, i):
            events.append((time.monotonic(), n, i))
            if n in ('state', 'contact', 'riseup', 'sequence_complete', 'exit',
                     'hover_correction', 'ground_disarmed'):
                print(f"[sitl] {n}: {i}", flush=True)
        sm = ExtNavLanding(vehicle, est, ekf, gate, sm_cfg, on_event=on_event, verbose=True)
        sm.attach_supervisor(sup)
        vehicle.set_abort_event(sup.abort)
        faza, det_latest = Latest(sm.state), Latest()
        st = SupervisorThread(sup, vehicle, det_latest, faza)
        st.start()
        cam = SyntheticCamera(pilot, seed=a.seed)

        # --- the pilot: wait for GPS, arm in GUIDED, take off to 6 m ---------
        t0 = time.monotonic()
        while pilot.origin is None or pilot.gps is None:
            if time.monotonic() - t0 > 120:
                raise RuntimeError("SITL fara GPS / origine")
            pilot.cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                      mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
            time.sleep(1.0)
        time.sleep(15.0)
        pilot.m.set_mode(pilot.m.mode_mapping()['GUIDED'])
        pilot.rc[2] = 1000                              # arming: throttle at zero
        for _ in range(30):
            pilot.m.arducopter_arm()
            time.sleep(1.0)
            if pilot.armed:
                break
        if not pilot.armed:
            raise RuntimeError(f"nu s-a armat: {pilot.texts[-3:]}")
        pilot.rc[2] = 1500
        pilot.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 6.0)
        t0 = time.monotonic()
        while (pilot.truth() or (0, 0, 0))[2] < 5.7 and time.monotonic() - t0 < 40:
            time.sleep(0.2)
        pilot.m.set_mode(pilot.m.mode_mapping()['LOITER'])
        time.sleep(4.0)
        print(f"[sitl] pilot: LOITER la {pilot.truth()[2]:.1f} m, AUX8 sus", flush=True)
        pilot.rc[7] = 1000
        loop_until = time.monotonic() + a.timeout
        aux_up_at = time.monotonic() + 1.5
        while time.monotonic() < loop_until:
            now = time.monotonic()
            if aux_up_at is not None and now >= aux_up_at:
                pilot.rc[7] = 2000
                aux_up_at = None
            vehicle.pump()
            for d in cam.poll(now):
                det_latest.set(d, d.t)
                sm.on_detection(d, now)
            faza.set(sm.state, now)
            sm.update(now)
            if sm.state == Phase.ENGAGE:
                pilot.feed_vpe = False                 # from here, OUR estimates only
            if sm.state in (Phase.DONE, Phase.ABORT, Phase.GATE_FAIL, Phase.REJECT):
                break
            time.sleep(0.01)
        res['final_state'] = sm.state
        res['exit_reason'] = sm.exit_reason
        res['detections'] = f"{cam.n_det}/{cam.n}"
        res['ground_vpe'] = sm.n_ground_vpe
        c = [i for _, n, i in events if n == 'contact']
        res['h_ref'] = round(c[0]['h_ref'], 2) if c else None
        tr = pilot.truth()
        res['final_agl'] = round(tr[2], 2)
        res['final_offset_m'] = round(math.hypot(tr[0] - MARKER_NE[0], tr[1] - MARKER_NE[1]), 2)
        time.sleep(2.0)
        st.stop()
        vehicle.close()
        pilot.stop.set()
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
    bins = sorted(glob.glob(os.path.join(wd, 'logs', '*.BIN')), key=os.path.getmtime)
    if not bins:
        print("[sitl] ATENTIE: niciun log DataFlash in", wd)
        res['log'] = None
    else:
        res['log'] = check_log(bins[-1])
    print("\n[sitl] REZULTAT:", res, flush=True)
    ok = res.get('final_state') == Phase.DONE and res.get('log') and res['log']['ok']
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
