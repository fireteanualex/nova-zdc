#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Detector sintetic de marker ArUco: fixtura pentru tools/test_state_machine.py.

Calculeaza din pozitia vehiculului ce ar "vedea" camera si PUBLICA doar
detectii (offset unghiular, distanta, dimensiunea markerului in pixeli,
range, timestamp), cu aceeasi interfata ca detectorul real:

    poll(now) -> iterabil de nova.detection.Detection

si aceeasi conventie de axe, documentata in nova/detection.py. Nu trimite
MAVLink si nu comanda nimic.

Atentie: detectorul sintetic e optimist. Nu modeleaza motion blur, detectii
false, variatii de expunere sau marker ocluzat.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.detection import CameraModel, Detection          # noqa: E402

DETECT_HZ = 20.0


class FakeDetector:
    """Detector sintetic. Publica Detection, nimic altceva.

    Primeste obiectul Vehicle doar ca sa citeasca pozitia si atitudinea -
    e singurul mod de a sti ce "vede" camera intr-o simulare fara imagini.
    Detectorul ArUco real nu are nevoie de asa ceva: el are cadre.
    """

    def __init__(self, vehicle, args, cam=None):
        self.v = vehicle
        self.cam = cam or CameraModel(focal_px=args.focal_px)
        self.marker_n = args.north
        self.marker_e = args.east
        self.noise_px = args.noise_px
        self.dropout = args.dropout
        self.latency = args.latency_ms / 1000.0

        self.period = 1.0 / DETECT_HZ
        # Ceasul se ia din primul poll(now), nu din time.monotonic() aici:
        # altfel detectorul nu poate fi rulat in timp accelerat, cum e rulata
        # masina de stari in tools/test_state_machine.py.
        self.next_tick = None
        self.pending = []        # coada pentru simularea latentei

        self.n_lost_fov = 0
        self.n_dropout = 0

    # -- interfata detectorului -------------------------------------------
    def poll(self, now):
        """Zero sau mai multe detectii noi. Aceeasi semnatura o va avea si
        detectorul ArUco real."""
        if self.next_tick is None:
            self.next_tick = now
        if now >= self.next_tick:
            self.next_tick += self.period
            det = self._observe(now)
            if det is not None:
                self.pending.append((now + self.latency, det))

        out = []
        while self.pending and self.pending[0][0] <= now:
            out.append(self.pending.pop(0)[1])
        return out

    # -- geometrie ---------------------------------------------------------
    def _observe(self, now):
        """Ce ar extrage solvePnP din cadrul de acum, sau None."""
        if not self.v.have_pos:
            return None

        alt = self.v.alt
        if alt < 0.02:
            return None

        d_n = self.marker_n - self.v.x
        d_e = self.marker_e - self.v.y

        c, s = math.cos(self.v.yaw), math.sin(self.v.yaw)
        fwd = d_n * c + d_e * s
        right = -d_n * s + d_e * c

        angle_x = math.atan2(fwd, alt)
        angle_y = math.atan2(right, alt)

        if not self.cam.in_fov(angle_x, angle_y):
            self.n_lost_fov += 1
            return None

        dist_3d = math.sqrt(fwd * fwd + right * right + alt * alt)
        marker_px = self.cam.marker_px_at(dist_3d)
        if not self.cam.fits_in_frame(marker_px):
            self.n_lost_fov += 1
            return None

        # ArduPilot aplica singur corectia de inclinare (inmulteste cu
        # cos(tilt)), deci raportam valoarea NEcorectata: alt / cos(tilt).
        tilt = math.cos(self.v.roll) * math.cos(self.v.pitch)
        raw_range = alt / max(tilt, 0.3)

        if random.random() < self.dropout:
            self.n_dropout += 1
            return None

        angle_x, angle_y, dist_3d, raw_range = self._add_noise(
            angle_x, angle_y, dist_3d, raw_range, marker_px)

        # Rotatia markerului IN CADRU: markerul e aliniat cu axele lumii,
        # deci e chiar capul vehiculului. Detectorul real o citeste din
        # colturi; aici nu exista colturi, deci se calculeaza din geometrie
        # - dar marimea raportata e aceeasi, `fill`.
        yaw_in_frame = math.degrees(self.v.yaw) % 90.0
        fill = self.cam.fill_at(marker_px, yaw_in_frame)

        return Detection(t=now, angle_x=angle_x, angle_y=angle_y,
                         distance_m=dist_3d, marker_px=marker_px,
                         range_m=raw_range, fill=fill)

    def _add_noise(self, angle_x, angle_y, dist, rng, marker_px):
        sigma_ang = self.noise_px / self.cam.focal_px
        angle_x += random.gauss(0.0, sigma_ang)
        angle_y += random.gauss(0.0, sigma_ang)

        rel_err = self.noise_px / max(marker_px, 1.0)
        scale = 1.0 + random.gauss(0.0, rel_err)
        return angle_x, angle_y, dist * scale, rng * scale
