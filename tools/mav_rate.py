#!/usr/bin/env python3
"""
NOVA - ZDC 2026
MAVLink message rates on the flight link, per message type (step 0, §5.65).

    ~/nova-venv/bin/python tools/mav_rate.py --seconds 20
    ~/nova-venv/bin/python tools/mav_rate.py --conn udpin:127.0.0.1:14552 --seconds 20

Connects exactly like the flight app (nova.vehicle.Vehicle, which requests
the same SET_MESSAGE_INTERVAL rates), counts every message that reaches the
companion for N seconds and prints messages/s per type. This is what the
serial link and the Python loop actually carry - the number to compare
against the minimal SR2_* set of step 6.

The port is exclusive: stop the autostart first
(`systemctl --user stop nova-bringup`) or pass --stop-service.
"""

import argparse
import collections
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import serial_guard                                   # noqa: E402
from nova.vehicle import Vehicle                                # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='/dev/serial0')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--stop-service', action='store_true')
    a = p.parse_args()

    if a.stop_service:
        os.system('systemctl --user stop nova-bringup 2>/dev/null')
    if not a.conn.startswith(('udp', 'tcp')):
        cine = serial_guard.describe_conflict(a.conn)
        if cine:
            print(f"[mav_rate] port held by another process:\n  {cine}")
            return 3

    serial = not a.conn.startswith(('udp', 'tcp'))
    v = Vehicle(a.conn, baud=a.baud if serial else None)
    v.connect()

    counts = collections.Counter()
    nbytes = collections.Counter()
    orig = v.m.recv_match

    def counted(*args, **kw):
        msg = orig(*args, **kw)
        if msg is not None:
            t = msg.get_type()
            counts[t] += 1
            try:
                nbytes[t] += len(msg.get_msgbuf())
            except Exception:                               # noqa: BLE001
                pass
        return msg
    v.m.recv_match = counted

    print(f"[mav_rate] counting for {a.seconds:.0f} s on {a.conn} ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        v.pump()
        time.sleep(0.002)
    dur = time.monotonic() - t0

    total = sum(counts.values())
    tot_b = sum(nbytes.values())
    print(f"\n{'message':<24}{'msg/s':>8}{'bytes/s':>10}")
    for t, n in counts.most_common():
        print(f"{t:<24}{n / dur:8.1f}{nbytes[t] / dur:10.0f}")
    print(f"{'TOTAL':<24}{total / dur:8.1f}{tot_b / dur:10.0f}")
    iv = v.heartbeat_interval() if hasattr(v, 'heartbeat_interval') else None
    if iv:
        print(f"\nHEARTBEAT interval measured: {iv * 1000:.0f} ms")
    if serial:
        print(f"link load: {8 * tot_b / dur / a.baud * 100:.1f}% of {a.baud} baud")
    return 0


if __name__ == '__main__':
    sys.exit(main())
