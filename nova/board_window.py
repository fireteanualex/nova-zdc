#!/usr/bin/env python3
"""
Onboard window for the analog OSD (team decision 27.09.2026).

The pilot watches a 720x480 analog OSD, so the window is composed at
exactly that size: the camera frame scaled to 720x405 (the 16:9 of the
1280x720 detection frame) with the detected marker outlined, and a 75 px
band underneath with three text lines: the state, the newest event and the
one before it. Nothing else - no nose arrow, no mounting rotation (the
270-degree rotation is applied to the AXES for guidance, never to pixels
shown here), no coordinates.

The window never decides anything about the flight: it reads, it draws, it
is fed at 10 Hz from the main thread (OpenCV wants imshow there), and
closing it leaves the application running (nova_pi.FereastraBord history).

    mesaje = OsdMesaje()
    ...on_event -> mesaje.note(name, info, now)
    fereastra = FereastraBord(detector, preview, sm=sm, sup=sup,
                              vehicle=vehicle, mesaje=mesaje)
"""

import math
import time

import cv2
import numpy as np

from .vehicle import MODE_NAME

OSD_W, OSD_H = 720, 480
IMG_H = 405                 # 720 x 405 keeps the 16:9 of 1280x720
BAND_H = OSD_H - IMG_H      # 75 px: three text lines
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
FONT_THICK = 1
LINE_Y = (20, 45, 70)
MAX_CHARS = 62              # what fits at this scale on 720 px

ALB = (230, 230, 230)
VERDE = (0, 220, 0)
GALBEN = (0, 220, 255)
ROSU = (0, 0, 255)

#: A detection older than this is drawn yellow, not green.
PROASPAT_S = 1.0
#: How many past messages are kept for the band (newest + previous).
MESAJE = 2


def _scurt(text, n=MAX_CHARS):
    text = str(text).replace('\n', ' ')
    return text if len(text) <= n else text[:n - 1] + '~'


def compune(gray, corners=None, linii=(), culori=(), corners_ok=True):
    """The 720x480 BGR image: frame on top, text band below.

    `gray` is the detection frame (any size, one channel); `corners` are in
    ITS pixel coordinates (4x2) and get scaled with it. `linii` are up to
    three strings, `culori` their BGR colours (white when missing)."""
    if gray is None:
        img = np.zeros((IMG_H, OSD_W, 3), np.uint8)
    else:
        small = cv2.resize(gray, (OSD_W, IMG_H), interpolation=cv2.INTER_AREA)
        img = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR) if small.ndim == 2 else small
        if corners is not None:
            h, w = gray.shape[:2]
            sx, sy = OSD_W / float(w), IMG_H / float(h)
            pts = np.asarray(corners, np.float32).reshape(-1, 2) * (sx, sy)
            col = VERDE if corners_ok else GALBEN
            cv2.polylines(img, [pts.astype(np.int32).reshape(-1, 1, 2)], True,
                          col, 2)
            cx, cy = pts.mean(axis=0)
            cv2.circle(img, (int(cx), int(cy)), 4, col, -1)
    band = np.zeros((BAND_H, OSD_W, 3), np.uint8)
    for i, linie in enumerate(list(linii)[:3]):
        col = culori[i] if i < len(culori) and culori[i] is not None else ALB
        cv2.putText(band, _scurt(linie), (8, LINE_Y[i]), FONT, FONT_SCALE,
                    col, FONT_THICK, cv2.LINE_AA)
    return np.vstack([img, band])


class OsdMesaje:
    """Turns state-machine events into short lines for the band, with
    the time they happened, and keeps the last MESAJE of them."""

    def __init__(self, keep=MESAJE):
        self.keep = keep
        self.items = []          # (t, text, colour), newest last

    def note(self, name, info, now=None):
        now = time.monotonic() if now is None else now
        text, col = self.format(name, info or {})
        if text is None:
            return
        self.items.append((now, text, col))
        del self.items[:-self.keep]

    @staticmethod
    def format(name, info):
        """(text, colour) or (None, None) for events the pilot need not see."""
        g = info.get
        if name == 'state':
            nota = g('note') or ''
            return (f"{g('old')} -> {g('new')}" + (f": {nota}" if nota else ''), ALB)
        if name == 'handover_reject':
            return f"REFUZ: {g('reason')}", ROSU
        if name == 'gate_fail':
            return "MARKER NEGASIT: raman in LOITER, AUX jos apoi sus", ROSU
        if name == 'engage':
            n = g('n')
            lat = g('lateral_m')
            det = 'o detectie' if n in (None, 1) else f"{n} detectii"
            lat_s = '' if lat is None else f", lateral {lat:.2f} m"
            return f"ENGAGE: {det}{lat_s}", VERDE
        if name == 'exit':
            pas = ' PASIV' if g('passive') else ''
            return f"EXIT{pas}: {g('reason')}", ROSU
        if name == 'exit_done':
            m = g('mode')
            return f"EXIT gata: FC in {MODE_NAME.get(m, m)}", GALBEN
        if name == 'exit_fallback':
            return "LOITER refuzat -> cer ALT_HOLD", GALBEN
        if name == 'scoring_capture':
            alt = g('alt')
            return (f"CAPTURA scoring la {alt:.2f} m" if alt is not None
                    else "CAPTURA scoring"), VERDE
        if name == 'touchdown':
            return "CONTACT", VERDE
        if name == 'handover_accept':
            return "ACCEPT: pornesc coborarea", VERDE
        if name == 'abort':
            act = g('action')
            return f"ABORT: {g('reason')}" + (f" -> {act}" if act else ''), ROSU
        if name == 'aux_ignored_disarmed':
            return "AUX sus ignorat: vehicul dezarmat", GALBEN
        return None, None

    def linii(self, now=None):
        """[(text, colour)] newest first, each with its age."""
        now = time.monotonic() if now is None else now
        out = []
        for t, text, col in reversed(self.items):
            age = max(0.0, now - t)
            out.append((f"{text} ({age:.0f}s)", col))
        return out


class FereastraBord:
    """Feeds the OSD window from the main thread, at AFISARE_HZ, without
    touching run_loop: it wraps the detector's poll(), like LastDetection.

    Closing the window does NOT stop the application: an Escape pressed by
    mistake during a descent must not leave the vehicle without its
    supervisor. The display decides nothing about the flight."""

    AFISARE_HZ = 10.0

    def __init__(self, inner, pv, sm=None, sup=None, vehicle=None,
                 mesaje=None):
        self._inner = inner
        self._pv = pv
        self.sm = sm
        self.sup = sup
        self.v = vehicle
        self.mesaje = mesaje if mesaje is not None else OsdMesaje()
        self._ultima = float('-inf')
        self.n_afisate = 0
        self.ultimul_cadru = None       # the last composed image (tests)

    def poll(self, now):
        dets = self._inner.poll(now)
        if self._pv.enabled and now - self._ultima >= 1.0 / self.AFISARE_HZ:
            self._ultima = now
            self._afiseaza(now)
        return dets

    # -- content -----------------------------------------------------------
    def linia_de_stare(self, now):
        sm, v = self.sm, self.v
        parti = []
        if sm is not None:
            parti.append(f"{getattr(sm, 'state', '?'):<12}")
            h = None
            if hasattr(sm, 'h_now'):
                h = sm.h_now()
            elif v is not None:
                h = getattr(v, 'alt', None)
            if h is not None:
                parti.append(f"h {h:4.1f}m")
            e = getattr(sm, 'last_est', None)
            if e is not None and now - e.t < 2.0:
                parti.append(f"lat {e.lateral_m:.2f}m")
            else:
                d = getattr(self._inner, 'last_detection', None)
                if d is not None and now - d.t < 2.0 and getattr(d, 'range_m', None):
                    lat = d.range_m * math.hypot(math.tan(d.angle_x), math.tan(d.angle_y))
                    parti.append(f"lat {lat:.2f}m")
            if hasattr(sm, 'aux_high'):
                parti.append(f"AUX {'SUS' if sm.aux_high() else 'jos'}")
            n = getattr(sm, 'n_vpe_sent', None)
            if n is not None:
                parti.append(f"VPE {n}")
        st = getattr(self._inner, 'stats', None)
        if st is not None:
            try:
                s = st()
                fps = s.get('fps')
                det = s.get('detection_rate')
                parti.append(f"cam {'-' if fps is None else f'{fps:.1f}'}fps")
                parti.append(f"det {'-' if det is None else f'{100 * det:.0f}%'}")
            except Exception:                               # noqa: BLE001
                pass
        return ' '.join(parti)

    def _afiseaza(self, now):
        cadru = getattr(self._inner, 'last_frame', None)
        aruco = getattr(self._inner, 'det', None)
        colturi = getattr(aruco, 'last_corners', None)
        d = getattr(self._inner, 'last_detection', None)
        proaspat = d is not None and (now - d.t) < PROASPAT_S
        linii = [self.linia_de_stare(now)]
        culori = [ALB]
        for text, col in self.mesaje.linii(now):
            linii.append(text)
            culori.append(col)
        try:
            img = compune(cadru, colturi if d is not None else None, linii,
                          culori, corners_ok=proaspat)
        except Exception as e:                               # noqa: BLE001
            print(f"[bord] fereastra: compunerea a picat ({type(e).__name__}: {e})")
            return
        self.ultimul_cadru = img
        if not self._pv.show(img):
            print("[bord] fereastra inchisa de la tastatura. Aplicatia "
                  "CONTINUA - afisarea nu decide nimic despre zbor.")
            self._pv.close()
            self._pv.enabled = False
            return
        self.n_afisate += 1

    def __getattr__(self, name):
        return getattr(self._inner, name)
