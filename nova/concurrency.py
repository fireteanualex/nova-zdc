#!/usr/bin/env python3
"""
Concurrency primitives for the onboard application (refactor/threads).

Four threads share the vehicle: detection, MAVLink I/O, the supervisor and
the main loop with the state machine. The rule that makes them safe is
simple and the primitives here enforce it: shared state is "the latest
value" behind a short lock, never a queue a reader could block on; queues
exist only towards the serial port; a thread never waits for another
thread; and no thread dies silently.

    latest = Latest()
    latest.set(state)             # writer
    state, t = latest.get()       # reader: never waits, (None, None) at first

    att = AttitudeBuffer()
    att.push(t, roll, pitch, yaw)
    att.at(t_capture)             # (roll, pitch, yaw) or None outside

    hb = Heartbeat('detectie')
    hb.beat(); hb.age()

    t = run_thread('supervizor', step, period=0.01, heartbeat=hb,
                   stop_event=stop)
"""

import collections
import logging
import math
import sys
import threading
import time
import traceback

log = logging.getLogger('nova')

#: A step that raises is logged and the loop goes on - but not in a tight
#: loop of tracebacks: after a failure the loop sleeps this long.
FAIL_BACKOFF_S = 0.05


class Latest:
    """The newest value of something, with the time it was set.

    `get()` never waits: before the first `set()` it returns (None, None).
    The lock is held only for the assignment / the read, never across
    anything that could block."""

    __slots__ = ('_lock', '_value', '_t', '_n')

    def __init__(self, value=None):
        self._lock = threading.Lock()
        self._value = value
        self._t = None if value is None else time.monotonic()
        self._n = 0

    def set(self, value, t=None):
        t = time.monotonic() if t is None else t
        with self._lock:
            self._value = value
            self._t = t
            self._n += 1

    def get(self):
        """(value, t_monotonic) - the pair set together, never mixed."""
        with self._lock:
            return self._value, self._t

    @property
    def count(self):
        """How many times set() was called (for tests and rate checks)."""
        with self._lock:
            return self._n


class AttitudeBuffer:
    """Bounded history of (t, roll, pitch, yaw), interpolated at any t
    inside it. Yaw goes through the shortest arc across +-pi.

    `at(t)` returns None outside [first, last] beyond `max_gap_s`: an
    answer for a time we did not observe would be a guess presented as a
    measurement (the estimator needs the attitude AT the capture time).
    Pushes come from the I/O thread, reads from the main thread: one
    lock, held only for the copy."""

    def __init__(self, maxlen=400, max_gap_s=0.25, wrap=(0, 1, 2)):
        self._lock = threading.Lock()
        self._buf = collections.deque(maxlen=maxlen)
        self.max_gap_s = float(max_gap_s)
        #: Which value indices are angles (shortest arc). The default is an
        #: attitude; wrap=() makes the same buffer hold positions.
        self.wrap = tuple(wrap)

    def push(self, t, *values):
        with self._lock:
            self._buf.append((float(t),) + tuple(float(v) for v in values))

    def __len__(self):
        with self._lock:
            return len(self._buf)

    def latest(self):
        with self._lock:
            return self._buf[-1] if self._buf else None

    def snapshot(self):
        """A list copy of the history (oldest first)."""
        with self._lock:
            return list(self._buf)

    def at(self, t):
        with self._lock:
            hist = list(self._buf)          # C-level copy, atomic under the GIL
        return interpolate(hist, t, wrap=self.wrap, max_gap_s=self.max_gap_s)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def interpolate(hist, t, wrap=(), max_gap_s=0.25):
    """Linear interpolation of a sorted (t, *values) list at time t.

    Indices in `wrap` (into the values, 0-based) are angles in radians,
    interpolated through the shortest arc and wrapped to (-pi, pi]. Beyond
    the ends by more than `max_gap_s` -> None; within, the end sample."""
    if not hist:
        return None
    if t <= hist[0][0]:
        return (tuple(hist[0][1:]) if hist[0][0] - t <= max_gap_s else None)
    if t >= hist[-1][0]:
        return (tuple(hist[-1][1:]) if t - hist[-1][0] <= max_gap_s else None)
    prev = hist[0]
    cur = hist[-1]
    for cur in hist:
        if cur[0] >= t:
            break
        prev = cur
    span = cur[0] - prev[0]
    f = 0.0 if span <= 0 else (t - prev[0]) / span
    out = []
    for i in range(1, len(cur)):
        a, b = prev[i], cur[i]
        if (i - 1) in wrap:
            d = wrap_pi(b - a)
            out.append(wrap_pi(a + f * d))
        else:
            out.append(a + f * (b - a))
    return tuple(out)


class Heartbeat:
    """A thread says "I am alive" once per iteration; another thread asks
    how long ago that was. Two fields set together under a lock."""

    def __init__(self, name, clock=time.monotonic):
        self.name = name
        self._clock = clock
        self._lock = threading.Lock()
        self._count = 0
        self._t = None

    def beat(self, t=None):
        t = self._clock() if t is None else t
        with self._lock:
            self._count += 1
            self._t = t

    @property
    def count(self):
        with self._lock:
            return self._count

    @property
    def last(self):
        with self._lock:
            return self._t

    def age(self, now=None):
        """Seconds since the last beat; None if it never beat."""
        with self._lock:
            t = self._t
        if t is None:
            return None
        now = self._clock() if now is None else now
        return now - t


def run_loop(name, step, period, heartbeat=None, stop_event=None,
             clock=time.monotonic, sleep=time.sleep, on_error=None):
    """Fixed-period loop: `step(now)` every `period` seconds until
    `stop_event` is set. An exception in `step` is logged with its
    traceback and the loop continues (after FAIL_BACKOFF_S); the thread
    never dies silently. The heartbeat beats once per iteration, AFTER the
    step - so a step that hangs is seen as a stale heartbeat, not as alive.

    `period` <= 0 means "as fast as it can" (the step blocks by itself,
    e.g. on the serial port or the camera)."""
    stop_event = stop_event if stop_event is not None else threading.Event()
    next_t = clock()
    while not stop_event.is_set():
        now = clock()
        try:
            step(now)
        except Exception as e:                              # noqa: BLE001
            log.error("[%s] pasul a picat: %s: %s\n%s", name,
                      type(e).__name__, e, traceback.format_exc())
            if on_error is not None:
                try:
                    on_error(e)
                except Exception:                           # noqa: BLE001
                    pass
            sleep(FAIL_BACKOFF_S)
        if heartbeat is not None:
            heartbeat.beat()
        if period > 0:
            next_t += period
            wait = next_t - clock()
            if wait > 0:
                sleep(wait)
            else:
                # fell behind (a slow step): do not try to catch up with a
                # burst of iterations, just re-anchor
                next_t = clock()


def run_thread(name, step, period, heartbeat=None, stop_event=None,
               on_error=None, daemon=True):
    """Start `run_loop` in a named daemon thread and return it."""
    t = threading.Thread(target=run_loop, name=name, daemon=daemon,
                         args=(name, step, period, heartbeat, stop_event),
                         kwargs={'on_error': on_error})
    t.start()
    return t


def stop_threads(threads, stop_event, timeout_s=2.0):
    """Set the stop event, then join each thread with its own timeout.
    Returns the names of the threads still alive afterwards."""
    stop_event.set()
    inca = []
    for t in threads:
        t.join(timeout=timeout_s)
        if t.is_alive():
            inca.append(t.name)
    return inca


_excepthook_installed = False


def install_excepthook():
    """threading.excepthook: an exception that escapes a thread (outside
    run_loop) is logged with its name and traceback, not lost on stderr
    of a headless service. Idempotent."""
    global _excepthook_installed
    if _excepthook_installed:
        return

    def _hook(args):
        nume = args.thread.name if args.thread is not None else '?'
        log.error("[%s] firul a murit: %s: %s\n%s", nume,
                  args.exc_type.__name__, args.exc_value,
                  ''.join(traceback.format_exception(
                      args.exc_type, args.exc_value, args.exc_traceback)))
        print(f"[fir {nume}] MORT: {args.exc_type.__name__}: {args.exc_value}",
              file=sys.stderr, flush=True)

    threading.excepthook = _hook
    _excepthook_installed = True
