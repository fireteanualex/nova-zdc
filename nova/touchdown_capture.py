#!/usr/bin/env python3
"""
The touchdown capture for rule 8.3.3, written on the Pi (27.09.2026).

    cap = TouchdownCapture(root, ring, camera_info, on_event=osd_note)
    ...on the state machine's 'contact' event:  cap.on_contact(info)
    ...every main-loop iteration:               cap.update(now)
    ...at exit:                                 cap.stop()

The file format is the contract with the PC (tools/fetch_scoring.py):
docs/SCORING_FORMAT.md. In short, per attempt:

    <root>/<YYYYMMDD>/<session>/attempt_<n>/
        touchdown.png            the frame, untouched (stream size, no
                                 rotation, no crop), colour when the camera
                                 kept that frame's chroma, else gray
        touchdown_annotated.png  a copy with a thin cross at the centre
        burst/<seq>_<t_ms>.jpg   the ring buffer around the contact
        meta.json                sync with the FC log, h_ref, camera, centre
        MANIFEST.sha256          LAST: an attempt without it is incomplete

Rules that make it trustworthy:
- The frame is the newest one captured BEFORE the Pi received ON_GROUND
  (t_capture <= t_on_ground_rx): on the ground, the marker filling the
  image. Never a later frame (the vehicle may be climbing by then).
- Nothing blocks detection or the state machine: on_contact() only takes
  references (the ring's arrays are never modified); the burst is taken
  1 s later in update(); all encoding and writing happens in one worker
  thread at low priority (nice +10 for that thread only).
- Every file is written atomically (temp file, fsync, rename) and the
  manifest is written last, so the PC never takes a half-written attempt.
- The capture is kept whatever happens after contact (a failed climb, an
  EXIT): it is scheduled at contact and does not look at the state
  machine again.
"""

import datetime
import hashlib
import json
import os
import queue
import threading
import time

import numpy as np

FORMAT = 'nova-scoring-1'
#: The burst: from the start of the touchdown descent (the frames at ~1 m,
#: marker whole and centred) or contact - BURST_BEFORE_S, whichever is
#: earlier, to contact + BURST_AFTER_S. Limited by what the ring holds.
BURST_BEFORE_S = 2.0
BURST_AFTER_S = 1.0
JPEG_QUALITY = 95
CENTER_PATCH = 5
#: The informative centre classification (docs/SCORING_FORMAT.md).
BLACK_MAX = 80
WHITE_MIN = 170
UNIFORM_SPAN = 60


def classify_center(gray, patch=CENTER_PATCH):
    """{'class', 'mean', 'min', 'max', 'patch_px'} on the 5x5 window at
    (w // 2, h // 2) of the gray plane. Informative only."""
    h, w = gray.shape[:2]
    cx, cy, r = w // 2, h // 2, patch // 2
    win = gray[cy - r:cy + r + 1, cx - r:cx + r + 1].astype(np.float64)
    mean, lo, hi = float(win.mean()), int(win.min()), int(win.max())
    span = hi - lo
    if mean < BLACK_MAX and span < UNIFORM_SPAN:
        cls = 'negru'
    elif mean > WHITE_MIN and span < UNIFORM_SPAN:
        cls = 'alb'
    else:
        cls = 'ambiguu'
    return {'class': cls, 'mean': round(mean, 1), 'min': lo, 'max': hi,
            'patch_px': patch}


def annotate(img):
    """A copy with a thin cross at the image centre; the 5x5 centre window
    itself is left untouched so the pixel that decides stays visible."""
    import cv2
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    arm, gap = max(20, min(w, h) // 20), CENTER_PATCH // 2 + 2
    col = (0, 0, 255)
    cv2.line(out, (cx - arm, cy), (cx - gap, cy), col, 1)
    cv2.line(out, (cx + gap, cy), (cx + arm, cy), col, 1)
    cv2.line(out, (cx, cy - arm), (cx, cy - gap), col, 1)
    cv2.line(out, (cx, cy + gap), (cx, cy + arm), col, 1)
    return out


def _fsync_dir(path):
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def write_atomic(path, data):
    """Temp file in the same directory, fsync, rename. Returns sha256."""
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, 'wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return hashlib.sha256(data).hexdigest()


class TouchdownCapture:

    def __init__(self, root, ring, camera_info=None, on_event=None,
                 clock=time.monotonic, session=None, threaded=True,
                 color_for=None, meta_for=None, nice=10):
        """`ring`: the detector's FrameRing ((t_capture, gray) pairs).
        `camera_info()`: dict preset / sensor_mode / scaler_crop /
        calibration. `meta_for(t)`: the camera metadata of the frame
        captured at t (ExposureTime, AnalogueGain, LensPosition), or None.
        `color_for(t, gray)`: a BGR image of that frame if the camera kept
        its chroma, else None. `on_event(name, info)` is called from
        update(), i.e. from the main thread, never from the worker."""
        self.root = root
        self.ring = ring
        self.camera_info = camera_info or (lambda: {})
        self.on_event = on_event
        self.clock = clock
        self.color_for = color_for
        self.meta_for = meta_for
        self.nice = nice
        now = datetime.datetime.now()
        self.date = now.strftime('%Y%m%d')
        self.session = session or now.strftime('s%Y%m%d-%H%M%S')
        self.n_attempts = 0
        self.pending = []           # contacts waiting for their burst
        self.done = []              # (attempt dir, ok, reason) - for tests
        self._events = []
        self._events_lock = threading.Lock()
        self._jobs = queue.Queue()
        self.threaded = threaded
        self._thread = None
        if threaded:
            self._thread = threading.Thread(target=self._worker,
                                            name='captura', daemon=True)
            self._thread.start()

    # -- main thread ---------------------------------------------------------
    def on_contact(self, info):
        """The state machine's 'contact' event. Picks the frame now (only
        references), the burst 1 s later (update)."""
        t_rx = info['t_on_ground_rx']
        frames = self._frames()
        before = [(t, f) for t, f in frames if t <= t_rx]
        self.n_attempts += 1
        job = {'n': self.n_attempts, 'info': dict(info), 't_rx': t_rx,
               'main': before[-1] if before else None,
               'burst_from': min(t_rx - BURST_BEFORE_S,
                                 info.get('td_start_t') or t_rx),
               'burst_to': t_rx + BURST_AFTER_S,
               'camera': self._safe(self.camera_info) or {},
               'pi_time_utc': datetime.datetime.now(datetime.timezone.utc)
               .isoformat(timespec='milliseconds')}
        if job['main'] is not None:
            t_main = job['main'][0]
            job['frame_meta'] = self._safe(lambda: self.meta_for(t_main)) \
                if self.meta_for else None
            job['color'] = self._safe(lambda: self.color_for(t_main, job['main'][1])) \
                if self.color_for else None
        self.pending.append(job)

    def update(self, now=None):
        """Every main-loop iteration: hand ripe contacts (burst window
        closed) to the worker, and pass the worker's results on."""
        now = self.clock() if now is None else now
        ripe = [j for j in self.pending if now >= j['burst_to']]
        for j in ripe:
            self.pending.remove(j)
            j['burst'] = [(t, f) for t, f in self._frames()
                          if j['burst_from'] <= t <= j['burst_to']]
            if self.threaded:
                self._jobs.put(j)
            else:
                self._run(j)
        with self._events_lock:
            evs, self._events = self._events, []
        for name, info in evs:
            if self.on_event is not None:
                try:
                    self.on_event(name, info)
                except Exception:                           # noqa: BLE001
                    pass

    def stop(self, timeout_s=10.0):
        """Flush what is pending (the burst may be short) and wait for the
        worker: a capture is evidence, it is not dropped at shutdown."""
        for j in list(self.pending):
            j['burst_to'] = min(j['burst_to'], self.clock())
        self.update(float('inf'))
        if self._thread is not None:
            self._jobs.put(None)
            self._thread.join(timeout_s)

    def _frames(self):
        buf = getattr(self.ring, 'buf', None)
        return list(buf) if buf is not None else []

    @staticmethod
    def _safe(fn):
        try:
            return fn()
        except Exception:                                   # noqa: BLE001
            return None

    # -- worker ---------------------------------------------------------------
    def _worker(self):
        try:
            # this thread only (Linux: nice is per task): the encoding must
            # never compete with detection or the state machine
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), self.nice)
        except (AttributeError, OSError):
            pass
        while True:
            j = self._jobs.get()
            if j is None:
                return
            self._run(j)

    def _run(self, j):
        d = os.path.join(self.root, self.date, self.session, f"attempt_{j['n']}")
        try:
            meta = self._write(j, d)
            ok, reason = True, ''
            ev = ('capture_saved', {'dir': d, 'center': meta['center']['class'],
                                    'attempt': j['n']})
        except Exception as e:                              # noqa: BLE001
            ok, reason = False, f"{type(e).__name__}: {e}"
            ev = ('capture_failed', {'dir': d, 'reason': reason, 'attempt': j['n']})
        self.done.append((d, ok, reason))
        with self._events_lock:
            self._events.append(ev)

    def _write(self, j, d):
        import cv2
        if j['main'] is None:
            raise RuntimeError("niciun cadru capturat inainte de ON_GROUND in ring")
        os.makedirs(os.path.join(d, 'burst'), exist_ok=True)
        hashes = {}
        t_main, gray = j['main']
        img = j.get('color')
        color = img is not None
        if not color:
            img = gray
        ok, png = cv2.imencode('.png', img)
        if not ok:
            raise RuntimeError("PNG-ul nu s-a putut codifica")
        hashes['touchdown.png'] = write_atomic(os.path.join(d, 'touchdown.png'), png.tobytes())
        ok, ann = cv2.imencode('.png', annotate(img))
        hashes['touchdown_annotated.png'] = write_atomic(
            os.path.join(d, 'touchdown_annotated.png'), ann.tobytes())
        g = gray if gray.ndim == 2 else cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        center = classify_center(g)
        files = 0
        burst = j.get('burst') or []
        for k, (t, f) in enumerate(burst):
            ok, jpg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                continue
            name = f"burst/{k:04d}_{int(round(t * 1000))}.jpg"
            hashes[name] = write_atomic(os.path.join(d, name), jpg.tobytes())
            files += 1
        info, fm, cam = j['info'], j.get('frame_meta') or {}, j['camera']
        meta = {
            'format': FORMAT, 'date': self.date, 'session': self.session,
            'attempt': j['n'],
            'sync': {'t_capture': t_main,
                     't_on_ground_rx': j['t_rx'],
                     'fc_time_boot_ms_contact': info.get('fc_time_boot_ms'),
                     'fc_time_unix_usec': info.get('fc_time_unix_usec'),
                     'pi_time_utc': j['pi_time_utc']},
            'h_ref_m': info.get('h_ref'),
            'last_detection': {'age_s': info.get('last_det_age_s'),
                               'lateral_m': info.get('last_lateral_m')},
            'camera': {'preset': cam.get('preset'),
                       'sensor_mode': cam.get('sensor_mode'),
                       'scaler_crop': cam.get('scaler_crop'),
                       'calibration': cam.get('calibration'),
                       'ExposureTime': fm.get('ExposureTime'),
                       'AnalogueGain': fm.get('AnalogueGain'),
                       'LensPosition': fm.get('LensPosition')},
            'image': {'file': 'touchdown.png', 'size': [int(img.shape[1]), int(img.shape[0])],
                      'color': color},
            'center': center,
            'burst': {'files': files,
                      't_from': burst[0][0] if burst else None,
                      't_to': burst[-1][0] if burst else None},
        }
        data = json.dumps(meta, indent=1, default=float).encode()
        hashes['meta.json'] = write_atomic(os.path.join(d, 'meta.json'), data)
        _fsync_dir(os.path.join(d, 'burst'))
        manifest = ''.join(f"{h}  {p}\n" for p, h in sorted(hashes.items()))
        write_atomic(os.path.join(d, 'MANIFEST.sha256'), manifest.encode())   # LAST
        _fsync_dir(d)
        return meta
