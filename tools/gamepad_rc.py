#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Punte gamepad -> ArduPilot, prin RC_CHANNELS_OVERRIDE.

**Maparea NU e in cod.** Se citeste din `config/gamepad.json`, generat de
`--calibrate`. Motivul e in 5.12 din CLAUDE.md: valorile "tipice" hardcodate
anterior puneau roll pe axa 2, care pe DualSense cu hid-playstation e
trigger-ul L2 (repaus -1.0, deci roll blocat la 1000 PWM). Enumerarea axelor
depinde de driver si de versiunea de kernel, nu doar de model.

    python3 tools/gamepad_rc.py --calibrate    # asistent, scrie config-ul
    python3 tools/gamepad_rc.py --preset dualsense   # start rapid, DE VERIFICAT
    python3 tools/gamepad_rc.py --check        # arata valorile mapate, fara MAVLink
    python3 tools/gamepad_rc.py                # ruleaza puntea

Parametri ArduPilot necesari (toti in config/nova_sitl.parm):
    FLTMODE_CH,5 / FLTMODE1,0 / FLTMODE4,5 / FLTMODE6,4
    FS_THR_ENABLE,0   # DOAR in SITL - vezi comentariul din nova_sitl.parm

Zboara in LOITER, nu STABILIZE. Stick-urile de consola au cursa scurta si
auto-centrare; in LOITER centrul inseamna "mentine", ceea ce se potriveste
natural. In STABILIZE vehiculul cade cand eliberezi stick-ul.

Canalul 7 (AUX) e cererea de handover, cu memorie: o apasare pe butonul de
handover il pune sus, urmatoarea il pune jos. Poarta din nova/handover.py
detecteaza FRONTUL CRESCATOR, iar REJECT se elibereaza doar cu AUX jos, deci
trebuie sa poti reveni.
"""

import argparse
import json
import os
import sys
import time

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')

import pygame                                              # noqa: E402
from pymavlink import mavutil                              # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'gamepad.json')

# --- Reglaje ------------------------------------------------------------
DEADBAND = 0.08          # zona moarta a stick-urilor (0..1)
EXPO = 0.4               # 0 = liniar, 0.6 = mult mai fin la centru
RATE_HZ = 25

PWM_MIN, PWM_MID, PWM_MAX = 1000, 1500, 2000

#: Canalul de mod si pozitiile care corespund FLTMODE1/4/6 din nova_sitl.parm.
MODE_CHANNEL = 5
MODE_PWM = {
    'STABILIZE': 1100,   # FLTMODE1
    'LOITER': 1500,      # FLTMODE4
    'GUIDED': 1900,      # FLTMODE6
}

#: Canalul de handover, citit de nova/state_machine.py (AUX_CHANNEL).
HANDOVER_CHANNEL = 7
ABORT_CHANNEL = 8

AXES = ('roll', 'pitch', 'throttle', 'yaw')
BUTTONS = ('stabilize', 'loiter', 'guided', 'handover', 'abort')

#: Ce trebuie sa faca pilotul la calibrare, si ce PWM inseamna asta. Calibram
#: dupa INTENTIE, nu dupa conventia driverului: asa semnul iese corect
#: indiferent cum numeroteaza SDL axele.
AXIS_PROMPTS = {
    'roll':     ('impinge manseta de ROLL spre DREAPTA', PWM_MAX),
    'pitch':    ('impinge manseta de PITCH INAINTE (nas in jos)', PWM_MIN),
    'throttle': ('impinge manseta de THROTTLE IN SUS (urcare)', PWM_MAX),
    'yaw':      ('impinge manseta de YAW spre DREAPTA', PWM_MAX),
}
BUTTON_PROMPTS = {
    'stabilize': 'butonul pentru modul STABILIZE',
    'loiter':    'butonul pentru modul LOITER',
    'guided':    'butonul pentru modul GUIDED',
    'handover':  'butonul de HANDOVER (recomandat R1)',
    'abort':     'butonul de ABORT / RTL',
}

#: Start rapid pentru DualSense cu hid-playstation. E un PUNCT DE PLECARE, nu
#: un adevar: verifica cu --check inainte sa zbori. Axele 2 si 5 sunt
#: trigger-ele L2/R2 pe acest driver, de aceea stick-ul drept e pe 3 si 4.
PRESETS = {
    'dualsense': {
        'device': 'DualSense (preset, DE VERIFICAT cu --check)',
        'axes': {
            'roll':     {'index': 3, 'invert': False},
            'pitch':    {'index': 4, 'invert': False},
            'throttle': {'index': 1, 'invert': True},
            'yaw':      {'index': 0, 'invert': False},
        },
        'buttons': {'stabilize': 2, 'loiter': 0, 'guided': 3,
                    'handover': 5, 'abort': 1},
    },
}


# --- config ---------------------------------------------------------------
def load_config(path=CONFIG_PATH):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        cfg = json.load(f)
    for name in AXES:
        if name not in cfg.get('axes', {}):
            raise SystemExit(f"{path}: lipseste axa '{name}'. "
                             f"Ruleaza --calibrate din nou.")
    return cfg


def save_config(cfg, path=CONFIG_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(cfg, f, indent=2, sort_keys=True)
        f.write('\n')
    print(f"\n[gamepad] scris {path}")


def open_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("Niciun gamepad detectat. Verifica /dev/input/js*")
    js = pygame.joystick.Joystick(0)
    js.init()
    return js


# --- conversie ------------------------------------------------------------
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


# --- calibrare ------------------------------------------------------------
def read_axes(js):
    pygame.event.pump()
    return [js.get_axis(i) for i in range(js.get_numaxes())]


def wait_axis(js, baseline, prompt, threshold=0.6, timeout=25.0):
    """Asteapta ca o axa sa se abata clar de la repaus si intoarce (idx, val)."""
    print(f"\n  {prompt}, apoi tine 1 s.")
    t0 = time.time()
    while time.time() - t0 < timeout:
        vals = read_axes(js)
        deltas = [abs(v - b) for v, b in zip(vals, baseline)]
        idx = max(range(len(deltas)), key=lambda i: deltas[i])
        if deltas[idx] >= threshold:
            # confirma ca ramane acolo
            time.sleep(0.4)
            vals = read_axes(js)
            if abs(vals[idx] - baseline[idx]) >= threshold:
                print(f"    -> axa {idx} ({vals[idx]:+.2f})")
                return idx, vals[idx]
        time.sleep(0.02)
    print("    -> nedetectat, sar peste")
    return None, None


def wait_button(js, prompt, timeout=25.0):
    print(f"\n  apasa {prompt}.")
    t0 = time.time()
    while time.time() - t0 < timeout:
        pygame.event.pump()
        for i in range(js.get_numbuttons()):
            if js.get_button(i):
                print(f"    -> butonul {i}")
                while js.get_button(i):      # asteapta eliberarea
                    pygame.event.pump()
                    time.sleep(0.02)
                return i
        time.sleep(0.02)
    print("    -> nedetectat, sar peste")
    return None


def calibrate():
    js = open_joystick()
    print(f"[gamepad] {js.get_name()}: {js.get_numaxes()} axe, "
          f"{js.get_numbuttons()} butoane")
    print("\nLasa toate mansele LIBERE si apasa Enter pentru a inregistra "
          "repausul.")
    input()
    baseline = read_axes(js)
    print(f"  repaus: {[f'{v:+.2f}' for v in baseline]}")

    cfg = {'device': js.get_name(), 'axes': {}, 'buttons': {}}
    for name in AXES:
        prompt, target_pwm = AXIS_PROMPTS[name]
        idx, val = wait_axis(js, baseline, prompt)
        if idx is None:
            sys.exit(f"Nu am detectat axa pentru '{name}'. Reia calibrarea.")
        # Calibram dupa intentie: daca miscarea ceruta trebuie sa dea PWM mare
        # dar axa a mers in negativ, inversam.
        wants_high = target_pwm == PWM_MAX
        went_positive = val > baseline[idx]
        cfg['axes'][name] = {'index': idx,
                             'invert': bool(wants_high != went_positive)}
        print("    revino la centru.")
        time.sleep(1.2)

    for name in BUTTONS:
        idx = wait_button(js, BUTTON_PROMPTS[name])
        if idx is not None:
            cfg['buttons'][name] = idx

    save_config(cfg)
    print("\nVerifica acum cu:  python3 tools/gamepad_rc.py --check")
    return 0


def check(cfg):
    """Arata valorile mapate, fara MAVLink. De rulat inainte de orice zbor."""
    js = open_joystick()
    print(f"[gamepad] {js.get_name()}")
    print(f"[gamepad] config pentru: {cfg.get('device')}")
    if cfg.get('device') != js.get_name():
        print("  ATENTIE: config-ul a fost facut pentru alt dispozitiv.")
    print("\n  Misca fiecare manseta si verifica sensul:")
    print("    ROLL dreapta -> creste | PITCH inainte -> scade")
    print("    THROTTLE sus -> creste | YAW dreapta  -> creste")
    print("  Apasa butoanele si verifica ca se aprind. Ctrl-C pentru oprire.\n")
    try:
        while True:
            pygame.event.pump()
            vals = {n: axis_to_pwm(js.get_axis(cfg['axes'][n]['index']),
                                   cfg['axes'][n]['invert']) for n in AXES}
            pressed = [n for n, i in cfg.get('buttons', {}).items()
                       if i < js.get_numbuttons() and js.get_button(i)]
            print(f"\r  R{vals['roll']:5d} P{vals['pitch']:5d} "
                  f"T{vals['throttle']:5d} Y{vals['yaw']:5d}  |  "
                  f"{', '.join(pressed) if pressed else '-':40s}",
                  end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")
    return 0


# --- puntea ---------------------------------------------------------------
class GamepadBridge:

    def __init__(self, conn_str, cfg):
        self.cfg = cfg
        self.js = open_joystick()
        print(f"[gamepad] {self.js.get_name()}  "
              f"axe={self.js.get_numaxes()} butoane={self.js.get_numbuttons()}")
        if cfg.get('device') != self.js.get_name():
            print(f"[gamepad] ATENTIE: config-ul e pentru "
                  f"'{cfg.get('device')}'. Ruleaza --calibrate.")

        self.mode = 'LOITER'
        self.handover = False
        self.abort = False
        self.prev_buttons = {}

        print(f"[gamepad] conectare la {conn_str} ...")
        self.m = mavutil.mavlink_connection(conn_str)
        self.m.wait_heartbeat()
        print(f"[gamepad] heartbeat sys={self.m.target_system}")

    def btn(self, name):
        return self.cfg.get('buttons', {}).get(name)

    def pressed(self, name):
        """True o singura data, la apasare (edge detection)."""
        idx = self.btn(name)
        if idx is None or idx >= self.js.get_numbuttons():
            return False
        now = self.js.get_button(idx)
        was = self.prev_buttons.get(idx, 0)
        self.prev_buttons[idx] = now
        return bool(now and not was)

    def read_buttons(self):
        if self.pressed('stabilize'):
            self.mode = 'STABILIZE'
        if self.pressed('loiter'):
            self.mode = 'LOITER'
        if self.pressed('guided'):
            self.mode = 'GUIDED'
        if self.pressed('handover'):
            # Memorie (latch): poarta detecteaza frontul crescator, iar REJECT
            # se elibereaza doar cu AUX jos - deci trebuie sa pot reveni.
            self.handover = not self.handover
            print(f"\n  >> AUX{HANDOVER_CHANNEL} "
                  f"{'SUS - handover cerut' if self.handover else 'JOS'}")
        if self.pressed('abort'):
            self.abort = not self.abort
            print(f"\n  >> ABORT {'ACTIV' if self.abort else 'inactiv'}")

    def axis(self, name):
        a = self.cfg['axes'][name]
        return axis_to_pwm(self.js.get_axis(a['index']), a['invert'])

    def send(self):
        ch = [PWM_MID] * 8
        ch[0] = self.axis('roll')
        ch[1] = self.axis('pitch')
        ch[2] = self.axis('throttle')
        ch[3] = self.axis('yaw')
        ch[MODE_CHANNEL - 1] = MODE_PWM[self.mode]
        ch[HANDOVER_CHANNEL - 1] = PWM_MAX if self.handover else PWM_MIN
        ch[ABORT_CHANNEL - 1] = PWM_MAX if self.abort else PWM_MIN

        self.m.mav.rc_channels_override_send(
            self.m.target_system, self.m.target_component, *ch)
        return ch

    def run(self):
        period = 1.0 / RATE_HZ
        last_print = 0.0
        print("\n[gamepad] rulez. Ctrl-C pentru oprire.")
        b = self.cfg.get('buttons', {})
        print(f"  moduri: stabilize=b{b.get('stabilize')} "
              f"loiter=b{b.get('loiter')} guided=b{b.get('guided')}")
        print(f"  handover=b{b.get('handover')} (AUX{HANDOVER_CHANNEL}, cu "
              f"memorie)   abort=b{b.get('abort')}\n")

        try:
            while True:
                pygame.event.pump()
                self.read_buttons()
                ch = self.send()

                now = time.monotonic()
                if now - last_print > 0.5:
                    last_print = now
                    print(f"\rR{ch[0]:5d} P{ch[1]:5d} T{ch[2]:5d} Y{ch[3]:5d} "
                          f"| {self.mode:9s} | HO "
                          f"{'SUS' if self.handover else 'jos'} "
                          f"| AB {'X' if self.abort else '-'}",
                          end='', flush=True)

                time.sleep(period)
        except KeyboardInterrupt:
            print("\n[gamepad] eliberez override-ul...")
            for _ in range(5):
                self.m.mav.rc_channels_override_send(
                    self.m.target_system, self.m.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0)
                time.sleep(0.05)
            print("[gamepad] oprit.")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='udpin:127.0.0.1:14553')
    p.add_argument('--config', default=CONFIG_PATH)
    p.add_argument('--calibrate', action='store_true',
                   help='asistent de mapare; scrie config/gamepad.json')
    p.add_argument('--preset', choices=sorted(PRESETS),
                   help='scrie un config de start rapid (DE VERIFICAT)')
    p.add_argument('--check', action='store_true',
                   help='arata valorile mapate, fara MAVLink')
    a = p.parse_args()

    if a.calibrate:
        return calibrate()
    if a.preset:
        save_config(PRESETS[a.preset], a.config)
        print("  Preset, nu masuratoare. Verifica cu --check inainte de zbor.")
        return 0

    cfg = load_config(a.config)
    if cfg is None:
        sys.exit(f"Nu exista {a.config}.\n"
                 f"  python3 tools/gamepad_rc.py --calibrate        (recomandat)\n"
                 f"  python3 tools/gamepad_rc.py --preset dualsense (rapid)")
    if a.check:
        return check(cfg)
    GamepadBridge(a.conn, cfg).run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
