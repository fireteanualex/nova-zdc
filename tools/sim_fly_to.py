#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Partea "manuala" a turului, facuta de un script (I4).

    python3 tools/sim_fly_to.py --conn udpin:127.0.0.1:14560 --alt 8.0
    python3 tools/sim_fly_to.py --alt 8 --north 2 --east -1

Decoleaza, urca la altitudinea de handover, se duce unde i se cere si lasa
vehiculul in LOITER - adica exact starea in care un pilot ar apasa
comutatorul AUX. De acolo preia `tools/sim_handover.py`.

DE CE E O UNEALTA SEPARATA

Segmentul autonom incepe la poarta (§8). Tot ce e inainte de poarta e zbor
de pilot, si nu are voie sa fie in `nova/` - un al doilea drum catre
secventa autonoma, chiar si unul de test, ar fi exact ce refuza §8. Aici e
o unealta de banc care foloseste pymavlink direct, nu `nova/vehicle.py`:
`Vehicle` nu are `arm()` si nu trebuie sa capete unul.

CE NU FACE

Nu comanda LAND si nu atinge PLND. Daca secventa autonoma nu porneste,
vehiculul ramane in LOITER si cade in grija pilotului (sau a
temporizatorului din `batch_sim.py`), nu a acestui script.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

#: Pozitie-numai in SET_POSITION_TARGET_LOCAL_NED: ignora viteza,
#: acceleratia si yaw-ul. Biti 3-11 aprinsi = "ignora".
TYPE_MASK_POS = 0b0000111111111000

MODE_GUIDED = 4
MODE_LOITER = 5

ARM_TRIES = 30
ARM_RETRY_S = 2.0


def set_mode(m, mode_id):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id,
        0, 0, 0, 0, 0)


def send_arm(m, arm=True):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if arm else 0, 0, 0, 0, 0, 0, 0)


def send_takeoff(m, alt_m):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(alt_m))


def send_goto_ned(m, north_m, east_m, alt_m):
    """Tinta in LOCAL_NED. `alt_m` e deasupra home; in NED se trimite ca -alt.

    Semnul lui z e capcana clasica: trimis pozitiv, vehiculul primeste
    comanda sa coboare SUB home si ArduPilot o accepta."""
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POS,
        float(north_m), float(east_m), -float(alt_m),
        0, 0, 0, 0, 0, 0, 0, 0)


def wait_heartbeat(m, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0):
            return True
    return False


def armed(m):
    hb = m.messages.get('HEARTBEAT')
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def position(m):
    """(north, east, alt) din LOCAL_POSITION_NED, sau None."""
    p = m.messages.get('LOCAL_POSITION_NED')
    if p is None:
        return None
    return p.x, p.y, -p.z


def pump(m, seconds):
    """Citeste mesaje `seconds` secunde, ca sa se actualizeze m.messages."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        m.recv_match(blocking=True, timeout=0.2)


def arm_with_retry(m, tries=ARM_TRIES, retry_s=ARM_RETRY_S, verbose=True):
    """Armeaza, reincercand.

    Nu asteptam separat convergenta EKF: verificarile de prearm SUNT
    conditia, iar un `arm` respins si reincercat spune mai mult decat o
    pauza fixa - stim CAND a devenit gata, nu doar ca am asteptat destul."""
    for i in range(tries):
        set_mode(m, MODE_GUIDED)
        pump(m, 0.5)
        send_arm(m)
        pump(m, 1.0)
        if armed(m):
            if verbose:
                print(f"[fly] armat dupa {i + 1} incercari")
            return True
        if verbose and i % 5 == 4:
            print(f"[fly] inca nu se armeaza ({i + 1}/{tries}); "
                  f"de obicei e EKF-ul")
        pump(m, retry_s)
    return False


def wait_alt(m, alt_m, tol=0.5, timeout=60.0, verbose=True):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(m, 0.2)
        p = position(m)
        if p and abs(p[2] - alt_m) <= tol:
            if verbose:
                print(f"[fly] la {p[2]:.2f} m dupa {time.time() - t0:.1f} s")
            return True
    return False


def wait_position(m, north_m, east_m, alt_m, tol=1.0, timeout=90.0,
                  verbose=True):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(m, 0.2)
        p = position(m)
        if p is None:
            continue
        d = math.sqrt((p[0] - north_m) ** 2 + (p[1] - east_m) ** 2
                      + (p[2] - alt_m) ** 2)
        if d <= tol:
            if verbose:
                print(f"[fly] la tinta ({d:.2f} m) dupa "
                      f"{time.time() - t0:.1f} s")
            return True
    return False


#: Coduri de iesire distincte. Un singur "a esuat" ar trimite pe teren (sau
#: intr-un CSV de campanie) cauza gresita: SITL care inca se compileaza,
#: prearm care nu trece si vant prea puternic arata la fel altfel.
EXIT_OK = 0
EXIT_TINTA = 1          # a decolat, nu a ajuns unde trebuia
EXIT_FARA_LEGATURA = 2  # niciun HEARTBEAT: SITL nu e (inca) acolo
EXIT_FARA_ARMARE = 3    # legatura exista, armarea e respinsa (prearm/EKF)


def fly_to(m, alt_m, north_m=0.0, east_m=0.0, tol=1.0, verbose=True):
    """Decolare -> altitudine -> pozitie -> LOITER. Intoarce un EXIT_*."""
    if not wait_heartbeat(m):
        print("[fly] EROARE: niciun HEARTBEAT")
        print("      SITL nu raspunde. Daca tocmai a fost pornit, poate inca")
        print("      compileaza; vezi fereastra/logul lui.")
        return EXIT_FARA_LEGATURA
    if verbose:
        print(f"[fly] sistem {m.target_system}, pornesc")

    if not arm_with_retry(m, verbose=verbose):
        print("[fly] EROARE: nu s-a armat (vezi prearm in consola SITL)")
        return EXIT_FARA_ARMARE

    send_takeoff(m, alt_m)
    if not wait_alt(m, alt_m, verbose=verbose):
        print(f"[fly] EROARE: nu a ajuns la {alt_m} m")
        return EXIT_TINTA

    if abs(north_m) > 1e-6 or abs(east_m) > 1e-6:
        send_goto_ned(m, north_m, east_m, alt_m)
        # Retrimis: un singur setpoint se poate pierde, iar ArduPilot
        # trateaza comanda ca o tinta care expira, nu ca pe o misiune.
        t0 = time.time()
        while time.time() - t0 < 90.0:
            send_goto_ned(m, north_m, east_m, alt_m)
            if wait_position(m, north_m, east_m, alt_m, tol=tol,
                             timeout=2.0, verbose=False):
                if verbose:
                    print(f"[fly] la tinta N={north_m:.2f} E={east_m:.2f}")
                break
        else:
            print("[fly] EROARE: nu a ajuns la tinta laterala")
            return EXIT_TINTA

    # LOITER: starea in care ar fi vehiculul cand pilotul apasa AUX.
    set_mode(m, MODE_LOITER)
    pump(m, 2.0)
    hb = m.messages.get('HEARTBEAT')
    if hb is not None and hb.custom_mode != MODE_LOITER:
        print(f"[fly] ATENTIE: FC raporteaza modul {hb.custom_mode}, "
              f"nu LOITER ({MODE_LOITER})")
    elif verbose:
        print("[fly] in LOITER, gata de handover")
    return EXIT_OK


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='udpin:127.0.0.1:14560')
    p.add_argument('--alt', type=float, required=True)
    p.add_argument('--north', type=float, default=0.0)
    p.add_argument('--east', type=float, default=0.0)
    p.add_argument('--tol', type=float, default=1.0)
    a = p.parse_args(argv)

    print(f"[fly] ma conectez la {a.conn}")
    m = mavutil.mavlink_connection(a.conn)
    return fly_to(m, a.alt, a.north, a.east, tol=a.tol)


if __name__ == '__main__':
    sys.exit(main())
