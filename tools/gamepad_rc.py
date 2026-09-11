#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Punte gamepad -> ArduPilot, prin RC_CHANNELS_OVERRIDE.

Scris pentru DualSense (PS5), dar merge cu orice gamepad recunoscut de
pygame. Butoanele de consola sunt momentane, deci comutatoarele de mod
sunt implementate cu memorie (latch) in software.

    python3 gamepad_rc.py --calibrate     # afla maparea axelor/butoanelor
    python3 gamepad_rc.py                 # ruleaza puntea

Parametri ArduPilot necesari:
    FLTMODE_CH,5
    FLTMODE1,0        # STABILIZE
    FLTMODE4,5        # LOITER
    FLTMODE6,4        # GUIDED
    FS_THR_ENABLE,0   # doar SITL: fara failsafe la deconectare

Zboara in LOITER, nu STABILIZE. Stick-urile de consola au cursa scurta
si auto-centrare; in LOITER centrul inseamna "mentine", ceea ce se
potriveste natural. In STABILIZE vehiculul cade cand eliberezi stick-ul.
"""

import argparse
import os
import sys
import time

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')

import pygame
from pymavlink import mavutil

# --- Maparea axelor (verifica cu --calibrate) ----------------------------
# Valori tipice pentru DualSense pe Linux cu hid-playstation:
AXIS_YAW = 0        # stick stanga X
AXIS_THROTTLE = 1   # stick stanga Y
AXIS_ROLL = 2       # stick dreapta X
AXIS_PITCH = 3      # stick dreapta Y

INVERT_YAW = False
INVERT_THROTTLE = True   # stick sus = -1, vrem valoare mare
INVERT_ROLL = False
INVERT_PITCH = True

# --- Maparea butoanelor -------------------------------------------------
BTN_MODE_STABILIZE = 3   # square
BTN_MODE_LOITER = 0      # cross
BTN_MODE_GUIDED = 2      # triangle
BTN_HANDOVER = 5         # R1 - canal AUX citit de companion
BTN_ABORT = 1            # circle - RTL

# --- Reglaje ------------------------------------------------------------
DEADBAND = 0.08          # zona moarta a stick-urilor (0..1)
EXPO = 0.4               # 0 = liniar, 0.6 = mult mai fin la centru
RATE_HZ = 25

PWM_MIN, PWM_MID, PWM_MAX = 1000, 1500, 2000

# Pozitiile PWM pentru canalul de mod (FLTMODE1/4/6)
MODE_PWM = {
    'STABILIZE': 1100,
    'LOITER': 1500,
    'GUIDED': 1900,
}


def apply_expo(v, expo):
    """Curba de raspuns: mai fina la centru, pastreaza capetele."""
    return (1.0 - expo) * v + expo * (v ** 3)


def axis_to_pwm(raw, invert=False, deadband=DEADBAND, expo=EXPO):
    v = -raw if invert else raw
    if abs(v) < deadband:
        return PWM_MID
    # rescaleaza dupa deadband ca sa nu existe salt
    sign = 1.0 if v > 0 else -1.0
    v = sign * (abs(v) - deadband) / (1.0 - deadband)
    v = apply_expo(v, expo)
    return int(PWM_MID + v * (PWM_MAX - PWM_MID))


class GamepadBridge:

    def __init__(self, conn_str):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            sys.exit("Niciun gamepad detectat. Verifica /dev/input/js*")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"[gamepad] {self.js.get_name()}  "
              f"axe={self.js.get_numaxes()} butoane={self.js.get_numbuttons()}")

        self.mode = 'LOITER'
        self.handover = False
        self.abort = False
        self.prev_buttons = {}

        print(f"[gamepad] conectare la {conn_str} ...")
        self.m = mavutil.mavlink_connection(conn_str)
        self.m.wait_heartbeat()
        print(f"[gamepad] heartbeat sys={self.m.target_system}")

    def pressed(self, idx):
        """True o singura data, la apasare (edge detection)."""
        if idx >= self.js.get_numbuttons():
            return False
        now = self.js.get_button(idx)
        was = self.prev_buttons.get(idx, 0)
        self.prev_buttons[idx] = now
        return now and not was

    def read_buttons(self):
        if self.pressed(BTN_MODE_STABILIZE):
            self.mode = 'STABILIZE'
        if self.pressed(BTN_MODE_LOITER):
            self.mode = 'LOITER'
        if self.pressed(BTN_MODE_GUIDED):
            self.mode = 'GUIDED'
        if self.pressed(BTN_HANDOVER):
            self.handover = not self.handover
            print(f"  >> HANDOVER {'CERUT' if self.handover else 'ANULAT'}")
        if self.pressed(BTN_ABORT):
            self.abort = not self.abort
            print(f"  >> ABORT {'ACTIV' if self.abort else 'inactiv'}")

    def send(self):
        ch = [PWM_MID] * 8
        ch[0] = axis_to_pwm(self.js.get_axis(AXIS_ROLL), INVERT_ROLL)
        ch[1] = axis_to_pwm(self.js.get_axis(AXIS_PITCH), INVERT_PITCH)
        ch[2] = axis_to_pwm(self.js.get_axis(AXIS_THROTTLE), INVERT_THROTTLE)
        ch[3] = axis_to_pwm(self.js.get_axis(AXIS_YAW), INVERT_YAW)
        ch[4] = MODE_PWM[self.mode]
        ch[5] = PWM_MID
        ch[6] = PWM_MAX if self.handover else PWM_MIN   # AUX7: handover
        ch[7] = PWM_MAX if self.abort else PWM_MIN      # AUX8: abort

        self.m.mav.rc_channels_override_send(
            self.m.target_system, self.m.target_component, *ch)
        return ch

    def run(self):
        period = 1.0 / RATE_HZ
        last_print = 0.0
        print("[gamepad] rulez. Ctrl-C pentru oprire.\n")
        print("  square=STABILIZE  cross=LOITER  triangle=GUIDED")
        print("  R1=handover  circle=abort\n")

        try:
            while True:
                pygame.event.pump()
                self.read_buttons()
                ch = self.send()

                now = time.monotonic()
                if now - last_print > 0.5:
                    last_print = now
                    print(f"\rR{ch[0]:5d} P{ch[1]:5d} T{ch[2]:5d} Y{ch[3]:5d} "
                          f"| {self.mode:9s} | HO {'X' if self.handover else '-'} "
                          f"| AB {'X' if self.abort else '-'}", end='', flush=True)

                time.sleep(period)
        except KeyboardInterrupt:
            print("\n[gamepad] eliberez override-ul...")
            for _ in range(5):
                self.m.mav.rc_channels_override_send(
                    self.m.target_system, self.m.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0)
                time.sleep(0.05)
            print("[gamepad] oprit.")


def calibrate():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("Niciun gamepad detectat.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"{js.get_name()}: {js.get_numaxes()} axe, "
          f"{js.get_numbuttons()} butoane")
    print("Misca stick-urile si apasa butoanele. Ctrl-C pentru oprire.\n")

    try:
        while True:
            pygame.event.pump()
            axes = " ".join(f"a{i}:{js.get_axis(i):+.2f}"
                            for i in range(js.get_numaxes()))
            btns = " ".join(f"b{i}"
                            for i in range(js.get_numbuttons())
                            if js.get_button(i))
            print(f"\r{axes}  |  {btns:30s}", end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")


def main():
    p = argparse.ArgumentParser(description="Punte gamepad -> ArduPilot")
    p.add_argument('--conn', default='udpin:127.0.0.1:14553')
    p.add_argument('--calibrate', action='store_true')
    a = p.parse_args()

    if a.calibrate:
        calibrate()
    else:
        GamepadBridge(a.conn).run()


if __name__ == '__main__':
    main()
