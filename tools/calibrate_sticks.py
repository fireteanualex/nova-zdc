#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Masoara zgomotul manselor in repaus, ca sa se poata alege STICK_DEADBAND_PWM
pe cifre, nu din burta (15.3.1, B3.4).

Regulamentul cere deadband-ul **documentat**. O valoare masurata pe
emitatorul de concurs tine mult mai bine la scrutineering decat una aleasa.

    # de la FC (emitator real legat prin receptor):
    python3 tools/calibrate_sticks.py --conn tcp:127.0.0.1:5760
    python3 tools/calibrate_sticks.py --conn /dev/serial0 --baud 921600

    # direct de la gamepad, fara FC (util pe desktop):
    python3 tools/calibrate_sticks.py --joystick

Lasa telecomanda in repaus, cu mansele libere, pe toata durata masuratorii.
NU atinge nimic: orice atingere strica statistica.

La final raporteaza min/max/sigma pe canalele 1-4 si recomandarea de
deadband, calculata ca max(3*sigma peste extrema observata) rotunjit in sus.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.rc import (STICK_CHANNELS, STICK_DEADBAND_PWM,  # noqa: E402
                     STICK_NOISE_PWM, TRIM_PARAMS)

NAMES = ('roll (1)', 'pitch (2)', 'throttle (3)', 'yaw (4)')


def stats(vals):
    n = len(vals)
    mean = sum(vals) / n
    var = sum((x - mean) ** 2 for x in vals) / n
    return mean, math.sqrt(var), min(vals), max(vals)


def report(samples, duration, trims=None):
    print(f"\n  {len(samples[0])} esantioane in {duration:.1f} s "
          f"({len(samples[0]) / duration:.0f} Hz)\n")
    print(f"  {'canal':<14} {'medie':>8} {'sigma':>7} {'min':>7} {'max':>7} "
          f"{'amplitudine':>12}")
    print("  " + "-" * 60)
    worst_dev = 0
    worst_sigma = 0.0
    for i, vals in enumerate(samples):
        mean, sigma, lo, hi = stats(vals)
        dev = max(abs(hi - mean), abs(mean - lo))
        worst_dev = max(worst_dev, dev)
        worst_sigma = max(worst_sigma, sigma)
        print(f"  {NAMES[i]:<14} {mean:8.1f} {sigma:7.2f} {lo:7.0f} {hi:7.0f} "
              f"{dev:10.0f} PWM")
        if trims:
            print(f"  {'':<14} trim {trims[i]}, offset fata de medie "
                  f"{mean - trims[i]:+.1f} PWM")

    rec = int(math.ceil((worst_dev + 3 * worst_sigma) / 10.0) * 10)
    print(f"\n  abatere maxima observata : {worst_dev:.0f} PWM")
    print(f"  sigma maxim              : {worst_sigma:.2f} PWM")
    print(f"  RECOMANDARE deadband     : {rec} PWM "
          f"(max observat + 3 sigma, rotunjit)")
    print(f"  in cod acum              : STICK_NOISE_PWM={STICK_NOISE_PWM}, "
          f"STICK_DEADBAND_PWM={STICK_DEADBAND_PWM}")
    if rec > STICK_DEADBAND_PWM:
        print("\n  ATENTIE: recomandarea depaseste valoarea din cod. "
              "Zgomotul acestui emitator ar putea declansa override fals.")
    else:
        print(f"\n  Valoarea din cod acopera zgomotul masurat "
              f"({STICK_DEADBAND_PWM} >= {rec}).")
    print("\n  De transcris in Safety Case: metoda (repaus, "
          f"{duration:.0f} s), cifrele de mai sus si valoarea aleasa.")


def from_vehicle(args):
    from nova.vehicle import Vehicle
    v = Vehicle(args.conn, baud=args.baud).connect()
    for name in TRIM_PARAMS:
        v.request_param(name)

    print(f"  colectez {args.seconds:.0f} s. NU atinge mansele.")
    samples = [[] for _ in STICK_CHANNELS]
    t0 = time.time()
    last_rc_t = None
    while time.time() - t0 < args.seconds:
        v.pump()
        if v.rc is not None and v.rc_t != last_rc_t:
            last_rc_t = v.rc_t
            for i, c in enumerate(STICK_CHANNELS):
                samples[i].append(v.rc[c])
        time.sleep(0.002)
        el = time.time() - t0
        if abs(el - round(el)) < 0.003 and int(el) % 2 == 0:
            print(f"    {el:.0f} s  n={len(samples[0])}", end='\r')

    if not samples[0]:
        print("\n  niciun RC_CHANNELS primit. Receptorul e legat? "
              "Verifica si ca FC-ul vede emitatorul.")
        return 1
    trims = None
    if all(n in v.params for n in TRIM_PARAMS):
        trims = [int(v.params[n]) for n in TRIM_PARAMS]
    report(samples, time.time() - t0, trims)
    return 0


def from_joystick(args):
    """Fara FC: citeste direct gamepad-ul si converteste in PWM la fel ca
    tools/gamepad_rc.py, ca sa masuram acelasi lant."""
    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("  niciun joystick gasit")
        return 1
    j = pygame.joystick.Joystick(0)
    j.init()
    print(f"  {j.get_name()}, {j.get_numaxes()} axe")
    print(f"  colectez {args.seconds:.0f} s. NU atinge mansele.")

    axes = (args.axis_roll, args.axis_pitch, args.axis_throttle, args.axis_yaw)
    samples = [[] for _ in STICK_CHANNELS]
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        pygame.event.pump()
        for i, ax in enumerate(axes):
            if ax < j.get_numaxes():
                samples[i].append(1500 + j.get_axis(ax) * 500.0)
        time.sleep(0.02)
    report(samples, time.time() - t0)
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='tcp:127.0.0.1:5760')
    p.add_argument('--baud', type=int, default=None)
    p.add_argument('--seconds', type=float, default=10.0)
    p.add_argument('--joystick', action='store_true',
                   help='citeste direct gamepad-ul, fara FC')
    # ATENTIE: maparea difera intre dispozitive si intre driverele Linux.
    # Pe DualSense cu hid-playstation, axele 2 si 5 sunt trigger-ele L2/R2
    # (repaus -1.0), iar stick-ul drept e pe 3 si 4. Verifica intotdeauna cu
    # gamepad_rc.py --calibrate inainte de a te baza pe cifre.
    p.add_argument('--axis-roll', type=int, default=3)
    p.add_argument('--axis-pitch', type=int, default=4)
    p.add_argument('--axis-throttle', type=int, default=1)
    p.add_argument('--axis-yaw', type=int, default=0)
    args = p.parse_args()
    return from_joystick(args) if args.joystick else from_vehicle(args)


if __name__ == '__main__':
    sys.exit(main())
