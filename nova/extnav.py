#!/usr/bin/env python3
"""
ExtNav estimator (redesign of 27.09.2026, claude-markdown/REPROIECTARE_EXTNAV.md
§4): a marker detection becomes the vehicle's position in the MARKER frame,
sent to EKF3 as VISION_POSITION_ESTIMATE. The marker is the origin; the
guidance then asks GUIDED for (0, 0, z).

Flight configuration only. The PLND path (LANDING_TARGET + DISTANCE_SENSOR)
stays untouched for the simulator.

THE CHAIN, PER DETECTION, AT CAPTURE TIME t_c

  1. ray to the marker centre in the BODY frame (FRD). It is already what
     the detectors publish: angle_x / angle_y are the angles of that ray on
     the forward / right axes (nova/detection.py), with the camera mounting
     applied at the source (detector_pi.axe_corp). Normalised to down = 1:
         b = (tan angle_x, tan angle_y, 1)
  2. rotate into NED with the FC attitude AT t_c - roll, pitch, yaw from
     ATTITUDE, interpolated (Vehicle.attitude_at). Not the latest one: at
     20 Hz and 50-100 ms of latency the difference is a few degrees, i.e.
     tens of centimetres at 10 m.
  3. scale the ray to the marker plane: its vertical component becomes h,
     the barometric altitude above home (assumption: the marker plane is
     at home's level).
  4. the vehicle's position in the marker frame is MINUS that vector;
     z = -h. Sent with usec = t_c and the FC's own attitude at t_c.

A scale error on h scales the estimated offset proportionally; the loop
absorbs it because every step re-measures and the residual shrinks
geometrically (brief §4). Direction is what matters, and direction does
not depend on the marker size - which is why solvePnP is no longer in the
guidance path.

WHAT "CONSISTENT" MEANS (D7: two consistent detections open the segment)

Each detection places the marker in the CURRENT EKF frame:
    marker_ned = position_at(t_c) + offset_ned
Two detections agree when those two points are closer than
max(0.3 m, 0.05 h). The vehicle's own motion between the captures is thus
compensated from LOCAL_POSITION_NED. Unknown position => not consistent:
"nu stim" nu inseamna "sunt de acord".
"""

import math
from dataclasses import dataclass

#: Centering tolerance as a function of height (brief §3):
#: tol(h) = max(0.15 m, 0.10 h). At 1 m it is 15 cm, under the ~19 cm for
#: which the camera footprint at contact stays on the marker.
TOL_MIN_M = 0.15
TOL_FRAC = 0.10
#: Two detections agree when their marker positions (in the same EKF
#: frame) are closer than max(0.3 m, 0.05 h).
CONSIST_MIN_M = 0.30
CONSIST_FRAC = 0.05
#: Below this vertical ray component the marker is near the horizon: the
#: scaling to the plane blows up. Refuse rather than send nonsense.
MIN_DOWN_COMPONENT = 0.05
#: Below this height the baro is not a usable reference for the plane
#: scaling (ground effect, noise). The final LAND is vertical anyway.
MIN_H_M = 0.30


def tol_m(h):
    """Centering tolerance at height h, metres."""
    return max(TOL_MIN_M, TOL_FRAC * float(h))


def consistency_tol_m(h):
    return max(CONSIST_MIN_M, CONSIST_FRAC * float(h))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ray_body(angle_x, angle_y):
    """Ray to the marker in the body frame (FRD), normalised to down = 1."""
    return (math.tan(angle_x), math.tan(angle_y), 1.0)


def body_to_ned(vec, roll, pitch, yaw):
    """Rotate a body-frame (FRD) vector into NED with ArduPilot's Euler
    convention: R = Rz(yaw) . Ry(pitch) . Rx(roll)."""
    x, y, z = vec
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rx
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    x1 = x
    # Ry
    x2 = cp * x1 + sp * z1
    z2 = -sp * x1 + cp * z1
    y2 = y1
    # Rz
    xn = cy * x2 - sy * y2
    yn = sy * x2 + cy * y2
    return (xn, yn, z2)


def ned_to_body(vec, roll, pitch, yaw):
    """Inverse of body_to_ned (transpose): R^T = Rx(-roll) Ry(-pitch) Rz(-yaw)."""
    x, y, z = vec
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(-yaw)
    x1 = cy * x + sy * y
    y1 = -sy * x + cy * y
    z1 = z
    # Ry(-pitch)
    x2 = cp * x1 - sp * z1
    z2 = sp * x1 + cp * z1
    y2 = y1
    # Rx(-roll)
    yb = cr * y2 + sr * z2
    zb = -sr * y2 + cr * z2
    return (x2, yb, zb)


def marker_offset_ned(angle_x, angle_y, roll, pitch, yaw, h):
    """(north, east, down) from the vehicle to the marker centre, metres,
    for a marker plane h metres below the vehicle. None if the ray does not
    point down enough to meet the plane."""
    n = body_to_ned(ray_body(angle_x, angle_y), roll, pitch, yaw)
    if n[2] <= MIN_DOWN_COMPONENT:
        return None
    s = float(h) / n[2]
    return (s * n[0], s * n[1], float(h))


def yaw_setpoint(yaw_now, marker_yaw_in_body_deg):
    """Yaw (rad) at which the marker sits unrotated in the frame.

    `marker_yaw_in_body_deg` is the marker's rotation as seen from above
    in the body frame, positive clockwise (the marker's "up" edge rotated
    to the right of the nose). Yawing right by that angle aligns the nose
    with it. None when the detector did not report an orientation."""
    if marker_yaw_in_body_deg is None:
        return None
    return wrap_pi(yaw_now + math.radians(marker_yaw_in_body_deg))


@dataclass(frozen=True)
class Estimate:
    """One detection turned into a pose in the marker frame."""
    t: float             # capture time (loop clock)
    x: float             # vehicle north of the marker, metres
    y: float             # vehicle east of the marker, metres
    z: float             # -h
    roll: float          # FC attitude at t
    pitch: float
    yaw: float
    h: float             # baro height used for the scaling
    off_n: float         # marker relative to the vehicle, NED
    off_e: float

    @property
    def lateral_m(self):
        return math.hypot(self.off_n, self.off_e)


class ExtNavEstimator:
    """Detection -> Estimate -> VISION_POSITION_ESTIMATE.

        est = ExtNavEstimator(vehicle)
        e = est.estimate(det)          # None when it cannot be computed
        est.send(e)                    # False when the link is down
        est.consistent(e1, e2)         # D7

    Height comes from `vehicle.rel_alt` (baro, above home), as the brief
    fixes; `h_source` exists for tests and for a future rangefinder.
    """

    def __init__(self, vehicle, h_source=None, verbose=False):
        self.v = vehicle
        self.h_source = h_source or (lambda: getattr(self.v, 'rel_alt', None))
        self.verbose = verbose
        self.n_estimates = 0
        self.n_refused = 0
        self.last_refusal = ''
        self.last = None

    def _refuse(self, why):
        self.n_refused += 1
        self.last_refusal = why
        if self.verbose:
            print(f"[extnav] fara estimare: {why}")
        return None

    def estimate(self, det):
        att = self.v.attitude_at(det.t)
        if att is None:
            return self._refuse(f"fara atitudine la t={det.t:.3f}")
        h = self.h_source()
        if h is None:
            return self._refuse("fara altitudine barometrica")
        h = float(h)
        if h < MIN_H_M:
            return self._refuse(f"h {h:.2f} m sub {MIN_H_M} m")
        roll, pitch, yaw = att
        off = marker_offset_ned(det.angle_x, det.angle_y, roll, pitch, yaw, h)
        if off is None:
            return self._refuse("raza nu intalneste planul markerului")
        e = Estimate(t=det.t, x=-off[0], y=-off[1], z=-h,
                     roll=roll, pitch=pitch, yaw=yaw, h=h,
                     off_n=off[0], off_e=off[1])
        self.n_estimates += 1
        self.last = e
        return e

    def send(self, e):
        return self.v.send_vision_position_estimate(
            int(round(e.t * 1e6)), e.x, e.y, e.z, e.roll, e.pitch, e.yaw)

    def marker_in_ekf_frame(self, e):
        """Where this estimate puts the marker in the CURRENT EKF frame
        (LOCAL_POSITION_NED at the capture time + offset), or None."""
        p = self.v.position_at(e.t)
        if p is None:
            return None
        return (p[0] + e.off_n, p[1] + e.off_e)

    def consistent(self, e1, e2):
        """D7 / brief §4: the two detections put the marker within
        max(0.3 m, 0.05 h) of each other, the vehicle's own motion between
        the captures compensated. Unknown position => False."""
        m1 = self.marker_in_ekf_frame(e1)
        m2 = self.marker_in_ekf_frame(e2)
        if m1 is None or m2 is None:
            return False
        d = math.hypot(m1[0] - m2[0], m1[1] - m2[1])
        return d <= consistency_tol_m(max(e1.h, e2.h))


class DetectionWindow:
    """Estimates collected inside one hover window, oldest first.

    `pair()` returns the newest estimate and the most recent earlier one it
    agrees with, or None - "two consistent detections" is the entry
    condition of the segment and the pass condition of a centring check."""

    def __init__(self, estimator, max_len=20):
        self.est = estimator
        self.max_len = max_len
        self.items = []

    def clear(self):
        self.items = []

    def add(self, e):
        self.items.append(e)
        del self.items[:-self.max_len]

    def pair(self):
        if len(self.items) < 2:
            return None
        newest = self.items[-1]
        for older in reversed(self.items[:-1]):
            if self.est.consistent(older, newest):
                return older, newest
        return None

    def __len__(self):
        return len(self.items)
