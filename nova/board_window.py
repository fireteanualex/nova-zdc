#!/usr/bin/env python3
"""
Onboard window for the analog OSD (team decision 27.09.2026).

The pilot watches an analog OSD, so the window is composed at exactly its
size (config `osd.size`: 720x480 NTSC by default, 720x576 PAL): the camera
frame scaled to fill everything above a 75 px band (720x405 on NTSC,
720x501 on PAL - stretched, no letterbox) with the detected marker
outlined, and the band with three text lines: the state (plus the camera
preset and the calibration kind), the newest event and the one before it.
Nothing else - no nose arrow, no mounting rotation (the 270-degree rotation
is applied to the AXES for guidance, never to pixels shown here), no
coordinates.

The window never decides anything about the flight: it reads, it draws, it
is fed from the main thread (OpenCV wants imshow there) once per camera
frame the detection thread processed, and closing it leaves the
application running (nova_pi.FereastraBord history). What it costs is
measured, not hidden: every WINDOW_LOG_S one log line with the frames
drawn and the compose+imshow time (median, p99), apart from the
detection's own window line.

    mesaje = OsdMesaje()
    ...on_event -> mesaje.note(name, info, now)
    fereastra = FereastraBord(detector, preview, sm=sm, sup=sup,
                              vehicle=vehicle, mesaje=mesaje,
                              size=cfg['osd']['size'])
"""

import math
import time

import cv2
import numpy as np

from .vehicle import MODE_NAME

OSD_W, OSD_H = 720, 480     # NTSC, the default `osd.size`
BAND_H = 75                 # three text lines, whatever the size
IMG_H = OSD_H - BAND_H      # 405 on NTSC (501 on PAL 720x576)
#: The smallest frame area / width `osd.size` may leave: below that the
#: marker outline and the text are no longer readable.
MIN_IMG_H = 120
MIN_W = 320
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
FONT_THICK = 1
LINE_Y = (20, 45, 70)
TEXT_X = 8                  # left margin; the right one is the same
#: Hard cap before the pixel-width check below (bounds the trimming loop).
#: The real limit is the rendered width: 62 characters was the worst case
#: (all digits), typical status text is narrower and fits ~85.
MAX_CHARS = 120

ALB = (230, 230, 230)
VERDE = (0, 220, 0)
GALBEN = (0, 220, 255)
ROSU = (0, 0, 255)

#: A detection older than this is drawn yellow, not green.
PROASPAT_S = 1.0
#: How many past messages are kept for the band (newest + previous).
MESAJE = 2
#: The window's own cost log: one line per this many seconds.
WINDOW_LOG_S = 5.0

#: Calibration kinds (nova.detector_pi CameraCalibration.kind), short form
#: for the state line.
CALIB_SCURT = {'nativa': 'nat', 'scalata': 'scal', 'derivata': 'der',
               'derivata+scalata': 'der+scal'}


def osd_size(size):
    """(w, h) from `osd.size`, or ValueError saying what is wrong."""
    try:
        w, h = size
    except (TypeError, ValueError):
        raise ValueError(f"osd.size trebuie sa fie [latime, inaltime], "
                         f"nu {size!r}") from None
    for v in (w, h):
        if isinstance(v, bool) or not isinstance(v, (int, np.integer)):
            raise ValueError(f"osd.size: {size!r} - latimea si inaltimea "
                             f"sunt numere intregi de pixeli")
    if w < MIN_W or h < BAND_H + MIN_IMG_H:
        raise ValueError(f"osd.size {w}x{h} prea mic: minim {MIN_W} lat, "
                         f"{BAND_H + MIN_IMG_H} inalt (banda de {BAND_H} px "
                         f"+ cel putin {MIN_IMG_H} px de imagine). NTSC = "
                         f"[720, 480], PAL = [720, 576]")
    return int(w), int(h)


def _latime(text):
    return cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICK)[0][0]


def _scurt(text, max_px=OSD_W - 2 * TEXT_X):
    """`text` on one line, cut (with '~') to what renders in `max_px`."""
    text = str(text).replace('\n', ' ')
    if len(text) > MAX_CHARS:
        text = text[:MAX_CHARS - 1] + '~'
    if _latime(text) <= max_px:
        return text
    while len(text) > 1 and _latime(text[:-1] + '~') > max_px:
        text = text[:-1]
    return text[:-1] + '~'


def compune(gray, corners=None, linii=(), culori=(), corners_ok=True,
            size=(OSD_W, OSD_H)):
    """The `size` (w, h) BGR image: frame on top, 75 px text band below.

    `gray` is the detection frame (any size, one channel); `corners` are in
    ITS pixel coordinates (4x2) and get scaled with it - the frame is
    stretched to w x (h - 75), no letterbox. `linii` are up to three
    strings, `culori` their BGR colours (white when missing)."""
    ow, oh = osd_size(size)
    ih = oh - BAND_H
    if gray is None:
        img = np.zeros((ih, ow, 3), np.uint8)
    else:
        small = cv2.resize(gray, (ow, ih), interpolation=cv2.INTER_AREA)
        img = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR) if small.ndim == 2 else small
        if corners is not None:
            h, w = gray.shape[:2]
            sx, sy = ow / float(w), ih / float(h)
            pts = np.asarray(corners, np.float32).reshape(-1, 2) * (sx, sy)
            col = VERDE if corners_ok else GALBEN
            cv2.polylines(img, [pts.astype(np.int32).reshape(-1, 1, 2)], True,
                          col, 2)
            cx, cy = pts.mean(axis=0)
            cv2.circle(img, (int(cx), int(cy)), 4, col, -1)
    band = np.zeros((BAND_H, ow, 3), np.uint8)
    for i, linie in enumerate(list(linii)[:3]):
        col = culori[i] if i < len(culori) and culori[i] is not None else ALB
        cv2.putText(band, _scurt(linie, ow - 2 * TEXT_X), (TEXT_X, LINE_Y[i]),
                    FONT, FONT_SCALE, col, FONT_THICK, cv2.LINE_AA)
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
        if name == 'contact':
            return f"CONTACT: h_ref {g('h_ref', 0.0):.2f} m, armat pe sol", VERDE
        if name == 'riseup':
            return f"URCARE la {g('target_above_home', 0.0):.1f} m (home)", VERDE
        if name == 'hover_correction':
            return f"CORECTIE: {g('lateral_m', 0.0):.2f} m de marker", GALBEN
        if name == 'sequence_complete':
            return "SECVENTA COMPLETA: SRC1, pilotul are drona", VERDE
        if name == 'ground_disarmed':
            return f"DEZARMAT PE SOL in {g('phase')}", ROSU
        if name == 'throttle_low_ground':
            return "THROTTLE LA MINIM pe sol: risc de dezarmare", GALBEN
        if name == 'capture_saved':
            return f"CAPTURA salvata ({g('center', '?')})", VERDE
        if name == 'capture_failed':
            return f"CAPTURA ESUATA: {g('reason')}", ROSU
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


def _print_flush(msg):
    # flushed like the detector's window line: the flight log is a tee
    print(msg, flush=True)


class FereastraBord:
    """Feeds the OSD window from the main thread, once per camera frame,
    without touching run_loop: it wraps the detector's poll(), like
    LastDetection.

    A new frame is `inner.frame_seq` changing (PiDetector counts the frames
    its thread reads); without that attribute (older detectors, tests),
    `inner.last_frame` being another array than the one drawn last. The
    same frame is never drawn twice, and there is no rate cap: the window
    runs at the detection's fps, so the fps it shows is the real one, and
    what drawing costs is timed (compose + imshow) and logged every
    `window_s` seconds; the last window's numbers are in `ultima_fereastra`.

    Closing the window does NOT stop the application: an Escape pressed by
    mistake during a descent must not leave the vehicle without its
    supervisor. The display decides nothing about the flight."""

    _NIMIC = object()           # "no frame drawn yet" (last_frame may be None)
    #: With no new frame for this long, redraw anyway (coordinator,
    #: 27.09.2026): the band carries the state and the events - an EXIT's
    #: reason - and a stalled camera must not freeze them on the pilot's
    #: OSD. Counted apart from the per-frame draws, never faster than this.
    REIMPROSPATARE_S = 1.0

    def __init__(self, inner, pv, sm=None, sup=None, vehicle=None,
                 mesaje=None, size=(OSD_W, OSD_H), clock=None,
                 window_s=WINDOW_LOG_S, log=_print_flush):
        self._inner = inner
        self._pv = pv
        self.sm = sm
        self.sup = sup
        self.v = vehicle
        self.mesaje = mesaje if mesaje is not None else OsdMesaje()
        self.size = osd_size(size)
        self._clock = clock if clock is not None else time.perf_counter
        self.window_s = float(window_s)
        self._log = log                 # None = no cost line (tests)
        self._seq = self._NIMIC         # frame_seq of the last frame drawn
        self._view = None               # the detector's last_view drawn
        self._amanat = False            # the previous call deferred its draw
        self.n_erori = 0                # draws that failed (said, not raised)
        self._t_desen = self._clock()   # last draw of any kind
        self.n_reimprospatari = 0       # draws without a new frame
        self._win_stale = 0
        self._cadru = self._NIMIC       # last_frame drawn (fallback)
        self.n_afisate = 0
        self.ultimul_cadru = None       # the last composed image (tests)
        # cost window: compose+imshow durations (ms) of the frames drawn
        self._win_t0 = None
        self._win_ms = []
        self.ultima_fereastra = None    # {s, drawn, rate, p50_ms, p99_ms}

    def poll(self, now):
        dets = self._inner.poll(now)
        # A detection just came in: the state machine gets it FIRST (run_loop
        # hands `dets` over right after this returns); the draw waits for the
        # next call, ~2 ms later (review 27.09.2026). One call at most: a
        # detector that returned something on every call would otherwise
        # never let the window draw.
        amana = bool(dets) and not self._amanat
        self._amanat = amana
        if self._pv.enabled and not amana:
            if self._cadru_nou():
                self._afiseaza(now)
            elif self._clock() - self._t_desen >= self.REIMPROSPATARE_S:
                self._afiseaza(now, reimprospatare=True)
            if self._pv.enabled:
                self._fereastra_log(self._clock())
        return dets

    def _cadru_nou(self):
        """True once per frame the detection thread read.

        Preferred: `last_view` = (seq, frame, corners) published by the
        detector at the END of a frame, in one assignment - the frame and
        ITS outline. Reading `last_frame` + `det.last_corners` apart races
        the detection, which clears the corners while it works: the outline
        would mostly be missing or one frame late."""
        view = getattr(self._inner, 'last_view', None)
        if isinstance(view, tuple) and len(view) == 3:
            if view[0] == self._seq:
                return False
            self._seq = view[0]
            self._view = view
            return True
        seq = getattr(self._inner, 'frame_seq', None)
        if seq is not None:
            if seq == self._seq:
                return False
            self._seq = seq
            return True
        cadru = getattr(self._inner, 'last_frame', None)
        if cadru is self._cadru:
            return False
        self._cadru = cadru
        return True

    # -- cost ------------------------------------------------------------------
    def _fereastra_log(self, t):
        """One line per `window_s`: frames drawn, their rate, and the
        compose+imshow time per frame (median, p99 nearest-rank)."""
        if self._win_t0 is None:
            self._win_t0 = t
            return
        span = t - self._win_t0
        if span < self.window_s:
            return
        ms = sorted(self._win_ms)
        n = len(ms)
        p50 = None if not n else (ms[n // 2] if n % 2
                                  else 0.5 * (ms[n // 2 - 1] + ms[n // 2]))
        p99 = None if not n else ms[max(0, math.ceil(0.99 * n) - 1)]
        self.ultima_fereastra = {'s': span, 'drawn': n, 'rate': n / span,
                                 'p50_ms': p50, 'p99_ms': p99,
                                 'stale': self._win_stale}
        if self._log is not None:
            f = (lambda v: '-' if v is None else f"{v:.1f}")
            fara = (f" | {self._win_stale} reimprospatari FARA cadru nou"
                    if self._win_stale else '')
            self._log(f"[fereastra {span:.0f}s] cadre afisate {n} "
                      f"({n / span:.1f}/s) | compunere+imshow p50 {f(p50)} ms, "
                      f"p99 {f(p99)} ms{fara}")
        self._win_t0 = t
        self._win_ms = []
        self._win_stale = 0

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
                # Fallback outside the ExtNav estimate: the barometric
                # height times the angles, like the estimator (level
                # vehicle assumed) - NOT the solvePnP range, which depends
                # on marker_size_m (27.09.2026).
                d = getattr(self._inner, 'last_detection', None)
                h_baro = getattr(v, 'alt', None) if v is not None else None
                if d is not None and now - d.t < 2.0 and h_baro and h_baro > 0.3:
                    lat = h_baro * math.hypot(math.tan(d.angle_x), math.tan(d.angle_y))
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
        # Last on purpose: static configuration, so if the line is ever too
        # wide the cut eats this, never the flight state before it.
        parti.append(self.camera_si_calibrare())
        return ' '.join(parti)

    def camera_si_calibrare(self):
        """'<preset> <calibration kind, short>', '-' for what is unknown."""
        settings = getattr(getattr(self._inner, 'source', None), 'settings', None)
        preset = getattr(settings, 'preset', None)
        kind = getattr(getattr(getattr(self._inner, 'det', None), 'calib', None),
                       'kind', None)
        kind = CALIB_SCURT.get(kind, kind) if isinstance(kind, str) else None
        return f"{preset or '-'} {kind or '-'}"

    def _afiseaza(self, now, reimprospatare=False):
        self._t_desen = self._clock()   # also on failure: no retry storm
        # Guarded whole, like status() in nova_pi (B8): the display runs in
        # the main thread, per frame; nothing in it may leave run_loop.
        try:
            self._deseneaza(now, reimprospatare)
        except Exception as e:                               # noqa: BLE001
            self.n_erori += 1
            if self.n_erori == 1 or self.n_erori % 100 == 0:
                print(f"[bord] fereastra: desenul a picat ({type(e).__name__}: "
                      f"{e}; {self.n_erori} ori); zborul continua", flush=True)

    def _deseneaza(self, now, reimprospatare):
        if self._view is not None:
            _seq, cadru, colturi = self._view
            if cadru is None:
                cadru = getattr(self._inner, 'last_frame', None)
        else:
            cadru = getattr(self._inner, 'last_frame', None)
            colturi = getattr(getattr(self._inner, 'det', None), 'last_corners', None)
        d = getattr(self._inner, 'last_detection', None)
        proaspat = d is not None and (now - d.t) < PROASPAT_S
        linii = [self.linia_de_stare(now)]
        culori = [ALB]
        for text, col in self.mesaje.linii(now):
            linii.append(text)
            culori.append(col)
        t0 = self._clock()
        try:
            img = compune(cadru, colturi if d is not None else None, linii,
                          culori, corners_ok=proaspat, size=self.size)
        except Exception as e:                               # noqa: BLE001
            print(f"[bord] fereastra: compunerea a picat ({type(e).__name__}: {e})")
            return
        self.ultimul_cadru = img
        shown = self._pv.show(img)
        t1 = self._clock()
        if not shown:
            print("[bord] fereastra inchisa de la tastatura. Aplicatia "
                  "CONTINUA - afisarea nu decide nimic despre zbor.")
            self._pv.close()
            self._pv.enabled = False
            return
        if reimprospatare:
            self.n_reimprospatari += 1
            self._win_stale += 1
            return
        self.n_afisate += 1
        if self._win_t0 is None:
            self._win_t0 = t0
        self._win_ms.append(1000.0 * (t1 - t0))

    def __getattr__(self, name):
        return getattr(self._inner, name)
