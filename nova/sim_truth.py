#!/usr/bin/env python3
"""
Adevarul din simulare: unde SUNT de fapt vehiculul si markerul (I4).

Gazebo stie pozitiile exacte. Comparate cu ce raporteaza detectorul, dau
singura validare pe care detectorul sintetic nu o putea da: acolo geometria
era corecta prin constructie, aici trece prin randare, distorsiune,
cuantizare si detectie reala.

    truth = SimTruth(world='nova_marker')
    t = truth.error_vs(detection, now)
    # t['range_rel'], t['angle_deg'], t['truth_range_m'], ...

CONVENTIA DE AXE, INCA O DATA

Gazebo e ENU, noi suntem NED (§5.31). Aici se face conversia INVERSA fata de
`make_marker_model.py`:

    north = pose.y      east = pose.x      down = -pose.z

Scris gresit, eroarea unghiulara iese transpusa si arata exact ca un bug in
`solvePnP` - adica te trimite sa cauti in locul gresit, a doua oara.

OFFSETUL DE 1 CM

Planul markerului e la z = 0.01, ca sa nu existe z-fighting cu solul
(§5.31). Adevarul pentru range e distanta pana la PLANUL markerului, deci
`z_vehicul - 0.01`. Ignorat, offsetul apare ca bias sistematic: 0.08% la
12 m, dar 2.0% la 0.5 m - fata de un prag I4 de 3%.
"""

import math
import threading
import time

#: Inaltimea planului markerului deasupra solului (§5.31).
MARKER_PLANE_Z = 0.01

#: Cat de jos e camera fata de originea vehiculului (§2: 74.5 mm deasupra
#: solului la contact). Adevarul se calculeaza pentru CAMERA, nu pentru
#: centrul vehiculului: detectorul masoara de la ea. Ignorat, apare ca bias
#: pe range - 0.7% la 10 m, dar **15% la 0.5 m**, adica exact acolo unde se
#: decide aterizarea.
CAM_MOUNT_M = 0.0745


def _gz_imports():
    """(Node, Pose_V). Acelasi tipar ca in detector_pi._gz_imports."""
    perechi = ((13, 10), (12, 9), (11, 8))
    erori = []
    for tv, mv in perechi:
        try:
            transport = __import__(f'gz.transport{tv}', fromlist=['Node'])
            msgs = __import__(f'gz.msgs{mv}.pose_v_pb2', fromlist=['Pose_V'])
            return transport.Node, msgs.Pose_V
        except ImportError as e:                            # noqa: PERF203
            erori.append(f"transport{tv}/msgs{mv}: {e}")
    raise ImportError(
        "gz-transport pentru Python nu e disponibil.\n"
        "  Creeaza mediul de simulare:  tools/setup_sim_venv.sh\n"
        "  Incercat: " + '; '.join(erori))


def ned_to_body(dn, de, dd, roll, pitch, yaw):
    """Un vector din NED in cadrul corpului, cu secventa 3-2-1.

    `dd` e componenta in JOS (NED), pozitiva sub vehicul."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x = cp * cy * dn + cp * sy * de - sp * dd
    y = ((sr * sp * cy - cr * sy) * dn + (sr * sp * sy + cr * cy) * de
         + sr * cp * dd)
    z = ((cr * sp * cy + sr * sy) * dn + (cr * sp * sy - sr * cy) * de
         + cr * cp * dd)
    return x, y, z


class Pose:
    """Pozitia unui model, in NED, cu timpul de simulare."""

    __slots__ = ('north', 'east', 'down', 'qw', 'qx', 'qy', 'qz', 't')

    def __init__(self, north, east, down, quat=(1.0, 0.0, 0.0, 0.0), t=None):
        self.north, self.east, self.down = north, east, down
        self.qw, self.qx, self.qy, self.qz = quat
        self.t = t

    @property
    def alt(self):
        return -self.down

    @property
    def yaw_deg(self):
        """Capul vehiculului, in grade.

        Obligatoriu pentru comparatie: detectorul raporteaza unghiuri in
        cadrul CORPULUI (inainte / dreapta), iar adevarul le are in NED
        (nord / est). Cele doua coincid doar cand capul e la zero. Fara
        rotatie, eroarea unghiulara raportata e practic chiar yaw-ul."""
        w, x, y, z = self.qw, self.qx, self.qy, self.qz
        return math.degrees(math.atan2(2 * (w * z + x * y),
                                       1 - 2 * (y * y + z * z)))

    @property
    def roll_pitch_deg(self):
        """(roll, pitch) in grade, din cuaternion.

        Conteaza pentru §5.2: verificarea de incadrare presupune camera la
        NADIR. Cu vehiculul inclinat, axa optica bate solul la
        `alt * tan(inclinare)` de punctul de sub el, iar marja pana la
        marginea cadrului scade cu atat."""
        w, x, y, z = self.qw, self.qx, self.qy, self.qz
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(sp)
        return math.degrees(roll), math.degrees(pitch)

    def __repr__(self):
        return (f"<Pose N={self.north:.3f} E={self.east:.3f} "
                f"alt={self.alt:.3f}>")


def pose_from_enu(x, y, z, quat=(1.0, 0.0, 0.0, 0.0), t=None):
    """ENU (Gazebo) -> NED (noi). Vezi §5.31."""
    return Pose(north=y, east=x, down=-z, quat=quat, t=t)


class SimTruth:
    """Pozitiile reale, abonat la topicul de poze al lumii.

    Nu comanda nimic si nu intra in bucla de control: exista doar ca sa
    compare. Un modul care ar putea influenta ce masoara nu ar mai fi
    adevar."""

    def __init__(self, world='nova_marker', vehicle='iris_with_gimbal',
                 marker='aruco_26', topic=None, verbose=True,
                 cam_mount_m=CAM_MOUNT_M):
        self.world = world
        self.vehicle_name = vehicle
        self.marker_name = marker
        self.verbose = verbose
        self.cam_mount_m = cam_mount_m
        self.topic = topic or f"/world/{world}/pose/info"

        self._lock = threading.Lock()
        self._poses = {}
        self.n_msgs = 0
        self.last_sim_t = None

        Node, PoseV = _gz_imports()
        self._node = Node()
        if not self._node.subscribe(PoseV, self.topic, self._on_msg):
            raise RuntimeError(
                f"nu ma pot abona la {self.topic}.\n"
                f"  Verifica: gz topic -l | grep pose")
        if verbose:
            print(f"[truth] abonat la {self.topic}")

    # -- receptie ----------------------------------------------------------
    def _on_msg(self, msg):
        with self._lock:
            self.n_msgs += 1
            for p in msg.pose:
                nume = p.name
                t = None
                if p.header.stamp.sec or p.header.stamp.nsec:
                    t = p.header.stamp.sec + p.header.stamp.nsec * 1e-9
                    self.last_sim_t = t
                self._poses[nume] = pose_from_enu(
                    p.position.x, p.position.y, p.position.z,
                    (p.orientation.w, p.orientation.x,
                     p.orientation.y, p.orientation.z), t)

    def pose(self, name):
        with self._lock:
            return self._poses.get(name)

    def names(self):
        with self._lock:
            return sorted(self._poses)

    def close(self):
        self._node = None

    # -- adevarul de comparat ----------------------------------------------
    def truth(self, roll=0.0, pitch=0.0, yaw=0.0):
        """Adevarul, EXPRIMAT CA SI CUM l-ar masura detectorul.

        `roll`, `pitch`, `yaw` sunt atitudinea vehiculului in **NED**,
        radiani - de luat din `ATTITUDE` raportat de FC, nu din cuaternionul
        Gazebo. Motivul e o capcana de convenție: in ENU, yaw = 0 inseamna
        nasul spre EST, iar in NED spre NORD. Derivat din cuaternion, unghiul
        iese rotit cu 90 de grade, iar eroarea unghiulara raportata devine
        chiar offsetul de convenție (§5.50).

        Ce intoarce, si de ce exact asa:

        - `angle_x` / `angle_y` in cadrul CAMEREI, prin aceleasi formule ca
          `detector_pi.detect()`. Detectorul nu raporteaza nord/est, deci un
          adevar in nord/est nu se poate compara cu el.
        - `range_m` pe AXA OPTICA pana la planul markerului, `(t·n)/n_z`, nu
          distanta verticala. Cu vehiculul inclinat cele doua difera cu
          `1/cos(inclinare)`: 1.2% la 8.7 grade, 3.5% la 15 (§5.50).
        """
        v = self.pose(self.vehicle_name)
        m = self.pose(self.marker_name)
        if v is None:
            return None
        n_m, e_m, z_m = (0.0, 0.0, MARKER_PLANE_Z)
        if m is not None:
            n_m, e_m, z_m = m.north, m.east, m.alt
        # Camera, nu centrul vehiculului: detectorul masoara de la ea.
        dz = v.alt - self.cam_mount_m - z_m
        if dz <= 0:
            return None
        dn = n_m - v.north
        de = e_m - v.east

        # NED -> corpul vehiculului, cu atitudinea completa (3-2-1).
        xb, yb, zb = ned_to_body(dn, de, dz, roll, pitch, yaw)
        # Normala planului markerului: (0,0,1) in NED, adusa in corp.
        nx, ny, nz = ned_to_body(0.0, 0.0, 1.0, roll, pitch, yaw)
        if abs(nz) < 0.2 or zb <= 0:
            return None
        rng = (xb * nx + yb * ny + zb * nz) / nz

        inclinare = math.degrees(math.acos(
            max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        return {
            'range_m': rng,
            'dist3d_m': math.sqrt(dn * dn + de * de + dz * dz),
            'north_off_m': dn,
            'east_off_m': de,
            'fwd_off_m': xb,
            'right_off_m': yb,
            'vert_m': dz,
            'yaw_deg': math.degrees(yaw),
            'tilt_deg': inclinare,
            'angle_x': math.atan2(xb, zb),
            'angle_y': math.atan2(yb, zb),
            'alt_m': v.alt,
            'marker_z_m': z_m,
            't': v.t,
        }

    def error_vs(self, det, marker_px_expect=None,
                 roll=0.0, pitch=0.0, yaw=0.0):
        """Eroarea detectiei fata de adevar. None daca nu se poate compara.

        `det` e un `Detection` din nova/detection.py."""
        tr = self.truth(roll, pitch, yaw)
        if tr is None or det is None:
            return None
        rng = tr['range_m']
        err_rel = (det.range_m - rng) / rng if rng > 0 else None
        # eroarea unghiulara: distanta unghiulara dintre cele doua directii
        da = det.angle_x - tr['angle_x']
        db = det.angle_y - tr['angle_y']
        out = {
            'truth_range_m': rng,
            'det_range_m': det.range_m,
            'range_rel': err_rel,
            'angle_deg': math.degrees(math.sqrt(da * da + db * db)),
            'angle_x_deg': math.degrees(da),
            'angle_y_deg': math.degrees(db),
            'alt_m': tr['alt_m'],
            'marker_px': det.marker_px,
            'truth_north_off_m': tr['north_off_m'],
            'truth_east_off_m': tr['east_off_m'],
            'yaw_deg': tr['yaw_deg'],
            'tilt_deg': tr['tilt_deg'],
        }
        if marker_px_expect is not None:
            out['marker_px_expect'] = marker_px_expect
        return out


class StaticTruth(SimTruth):
    """Adevar din poze date direct, fara Gazebo. Pentru teste."""

    def __init__(self, vehicle=None, marker=None, **kw):
        self.world = kw.get('world', 'test')
        self.vehicle_name = kw.get('vehicle', 'iris_with_gimbal')
        self.marker_name = kw.get('marker', 'aruco_26')
        self.verbose = False
        self.cam_mount_m = kw.get('cam_mount_m', CAM_MOUNT_M)
        self.topic = None
        self._lock = threading.Lock()
        self._poses = {}
        self.n_msgs = 0
        self.last_sim_t = None
        self._node = None
        if vehicle is not None:
            self._poses[self.vehicle_name] = vehicle
        if marker is not None:
            self._poses[self.marker_name] = marker

    def set(self, name, pose):
        with self._lock:
            self._poses[name] = pose
            if pose.t is not None:
                self.last_sim_t = pose.t


def summarize(errors, praguri=None):
    """Statistici peste o lista de dictionare de la `error_vs`.

    Percentile, nu medie: o singura detectie ratata partial trage media cu
    procente intregi, iar noi vrem cifra tipica si coada, nu amestecul lor
    (aceeasi alegere ca in tools/run_e2.py)."""
    praguri = praguri or {}
    err = [e for e in errors if e is not None]
    if not err:
        return {'n': 0}

    def pct(vals, p):
        if not vals:
            return None
        s = sorted(vals)
        k = (len(s) - 1) * p / 100.0
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return s[lo] + (s[hi] - s[lo]) * (k - lo)

    rng = [abs(e['range_rel']) for e in err if e['range_rel'] is not None]
    ang = [e['angle_deg'] for e in err]
    out = {
        'n': len(err),
        'range_p50': pct(rng, 50), 'range_p95': pct(rng, 95),
        'range_max': max(rng) if rng else None,
        'angle_p50': pct(ang, 50), 'angle_p95': pct(ang, 95),
        'angle_max': max(ang) if ang else None,
    }
    prag_r = praguri.get('range_rel', 0.03)
    prag_a = praguri.get('angle_deg', 0.5)
    out['range_ok'] = (out['range_p95'] is not None
                       and out['range_p95'] <= prag_r)
    out['angle_ok'] = out['angle_p95'] is not None and out['angle_p95'] <= prag_a
    return out


def detection_rate(rezultate, alt_min=3.0, alt_max=12.0):
    """Rata de detectie in fereastra de altitudine care conteaza (I4).

    `rezultate` = [(alt_m, detectat_bool)]. In afara ferestrei nu se
    numara: sub 0.38 m markerul nu incape in cadru PRIN CONSTRUCTIE (§5.2),
    iar a numara acele cadre ca ratari ar transforma o proprietate fizica
    intr-o defectiune inventata."""
    in_fereastra = [(a, d) for a, d in rezultate if alt_min <= a <= alt_max]
    if not in_fereastra:
        return None, 0
    n_det = sum(1 for _a, d in in_fereastra if d)
    return n_det / len(in_fereastra), len(in_fereastra)


def latest_sim_t(sources):
    """Cel mai recent timp de simulare vazut de oricare sursa, sau None.

    `None` inseamna "inca nu stiu", si e diferit de orice cifra. Apelantul
    trebuie sa poata face diferenta: pana la primul mesaj nu exista timp de
    simulare, iar o valoare de rezerva (ceasul de perete) e cu patru ordine
    de marime mai mare. Cand sosesc cadrele, ceasul ar SARI INAPOI, si orice
    diferenta calculata peste acel salt e absurda."""
    t = None
    for s in sources:
        if s is None:
            continue
        v = getattr(s, 'last_sim_t', None)
        if v is None and hasattr(s, 'sim_time'):
            v = s.sim_time()
        if v is not None and (t is None or v > t):
            t = v
    return t


def now_sim(sources, fallback=None):
    """Ca `latest_sim_t`, dar cu o valoare de rezerva cand nu stie nimeni.

    `fallback` e de obicei `time.monotonic`. Cine are nevoie sa deosebeasca
    "nu stiu" de o cifra foloseste `latest_sim_t`."""
    t = latest_sim_t(sources)
    if t is None:
        return fallback() if fallback else time.monotonic()
    return t
