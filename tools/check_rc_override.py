#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verifica precontitia intregului lant de override (15.3.1): raporteaza
ArduPilot inapoi in RC_CHANNELS valorile primite prin RC_CHANNELS_OVERRIDE?

Daca NU, atunci poarta de handover nu vede manetele si monitorul de override
nu functioneaza - deci nici testele cu pilot in bucla nu inseamna nimic.

    python3 tools/check_rc_override.py --conn tcp:127.0.0.1:5760

ATENTIE la cum se masoara. Prima varianta a acestui test trimitea 3 s fara sa
citeasca, apoi citea - si raporta PICAT, pentru ca bufferul TCP se umpluse si
citea mesaje vechi. Trimiterea si citirea trebuie sa fie CONCURENTE.
Vezi 5.11 din CLAUDE.md.

Conditia de acceptare: MAV_GCS_SYSID trebuie sa se potriveasca cu sysid-ul
sursei noastre (GCS_MAVLINK::handle_rc_channels_override respinge tacit orice
alt sysid).
"""

import argparse
import sys
import time

from pymavlink import mavutil

TARGET = (1234, 1345, 1456, 1567, 0, 0, 1890, 0)
PARAMS = ('MAV_GCS_SYSID', 'RC_OPTIONS', 'RC_OVERRIDE_TIME')


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode('ascii'), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg and msg.param_id.rstrip('\x00') == name:
            return msg.param_value
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='tcp:127.0.0.1:5760')
    p.add_argument('--baud', type=int, default=None)
    p.add_argument('--seconds', type=float, default=5.0)
    args = p.parse_args()

    kwargs = {'baud': args.baud} if args.baud else {}
    m = mavutil.mavlink_connection(args.conn, **kwargs)
    m.wait_heartbeat()
    print(f"heartbeat sys={m.target_system}; sursa noastra sysid="
          f"{m.mav.srcSystem}")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                            int(1e6 / 50), 0, 0, 0, 0, 0)

    gcs_sysid = None
    for n in PARAMS:
        val = get_param(m, n)
        print(f"  {n} = {val}")
        if n == 'MAV_GCS_SYSID':
            gcs_sysid = val
    if gcs_sysid is not None and int(gcs_sysid) != m.mav.srcSystem:
        print(f"\n  ATENTIE: MAV_GCS_SYSID={int(gcs_sysid)} dar noi trimitem "
              f"ca {m.mav.srcSystem}. ArduPilot va ignora TACIT override-ul.")

    print(f"\ntrimit continuu {TARGET}, citesc concurent ({args.seconds:.0f} s):")
    t0 = time.time()
    last_send = 0.0
    seen = []
    while time.time() - t0 < args.seconds:
        if time.time() - last_send > 0.05:
            last_send = time.time()
            m.mav.rc_channels_override_send(m.target_system,
                                            m.target_component, *TARGET)
        msg = m.recv_match(type='RC_CHANNELS', blocking=False)
        if msg:
            rc = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                  msg.chan4_raw, msg.chan7_raw)
            if not seen or seen[-1][1] != rc:
                seen.append((time.time() - t0, rc))
                print(f"  t={time.time() - t0:5.2f}s  ch1-4,7 = {rc}")
        time.sleep(0.002)

    want = (TARGET[0], TARGET[1], TARGET[2], TARGET[3], TARGET[6])
    ok = any(s[1] == want for s in seen)
    print(f"\n{'TRECUT' if ok else 'PICAT'}: RC_CHANNELS_OVERRIDE "
          f"{'se reflecta' if ok else 'NU se reflecta'} in RC_CHANNELS")
    if not ok:
        print("  Fara asta, poarta de handover si monitorul de override nu "
              "pot functiona. Verifica MAV_GCS_SYSID.")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
