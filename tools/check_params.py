#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verifica prin CITIRE INAPOI ca fiecare parametru din config/nova_sitl.parm
exista si are valoarea ceruta.

De ce exista unealta asta (vezi 5.10 din CLAUDE.md): ArduPilot accepta tacut
PARAM_SET pe nume inexistente. Nu da eroare, nu da PARAM_VALUE, iar fisierul
se incarca "cu succes". Am pierdut asa WP_RFND_USE, scris gresit
WPNAV_RFND_USE, in toate cele 13 rulari din Faza 1.

    python3 tools/check_params.py                       # SITL pe TCP
    python3 tools/check_params.py --conn udpin:127.0.0.1:14552
    python3 tools/check_params.py --conn /dev/serial0 --baud 921600

Cod de iesire 0 numai daca TOTI parametrii exista si se potrivesc. De rulat
inainte de orice campanie de teste si inainte de scrutineering: iesirea lui e
dovada pentru Compliance Matrix.

SCRIERE (--write): scrie fisierul PE NUME, cu citire inapoi pe fiecare.

    python3 tools/check_params.py --conn /dev/serial0 --baud 921600 \
        --parm config/nova_flight_4.5.parm --write --reboot

De ce nu prin Mission Planner. PLND_* si RNGFND1_* sunt ascunsi din LISTA
de parametri cat timp PLND_ENABLED / RNGFND1_TYPE sunt 0
(AP_PARAM_FLAG_ENABLE), iar Mission Planner scrie doar ce gaseste in lista
pe care a descarcat-o: "No matching Params". Dar FC-ul accepta scrieri si
citiri PE NUME oricand - AP_Param::find() nu tine cont de flag (verificat
pe Copter-4.5.7, GCS_Param.cpp). Deci pe nume nu mai e nevoie de dansul
activeaza-reporneste-incarca-dezactiveaza.

Dupa scriere FC-ul trebuie REPORNIT: driverul de telemetru si backend-ul de
precision landing se creeaza o singura data, la boot, din RNGFND1_TYPE si
PLND_TYPE (init_precland() pe 4.5.7). Fara repornire, un PLND_TYPE proaspat
scris nu are niciun efect, iar LANDING_TARGET e ignorat fara niciun mesaj.
"""

import argparse
import os
import sys
import time

from pymavlink import mavutil

DEFAULT_PARM = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'nova_sitl.parm')

# Parametrii intregi se compara exact; cei reali au toleranta, pentru ca
# float32 nu reprezinta exact 0.0745.
REL_TOL = 1e-4
ABS_TOL = 1e-6

# Parametrii-masca. Citirea inapoi confirma ce VALOARE are parametrul, nu ce
# EFECT are: FENCE_TYPE=4 ar fi trecut orice audit si ar fi stins tacut
# plafonul de altitudine cerut de 15.2.4. Pentru mastile de biti raportam
# fiecare bit separat, cu ce inseamna - asa se vede ce am aprins si, mai
# important, ce am stins.
# Bitii verificarilor la armare, din AP_Arming.h (enum Check; identic pe
# Copter-4.5.7 si pe 4.8-dev). Prima varianta avea 12-14 decalati: bitul 12
# aparea ca "hardware de siguranta" cand e GPS_CONFIG - adica tocmai bitul
# pe care echipa l-a stins pe vehicul, descris gresit.
BITI_ARMARE = {
    0: 'toate verificarile', 1: 'baro', 2: 'busola', 3: 'GPS',
    4: 'INS', 5: 'parametri', 6: 'RC', 7: 'tensiune',
    8: 'baterie', 9: 'airspeed', 10: 'logging',
    11: 'comutator de siguranta', 12: 'GPS config', 13: 'sistem',
    14: 'misiune', 15: 'rangefinder', 16: 'camera', 17: 'autorizare aux',
    18: 'vizual', 19: 'FFT', 20: 'osd',
}

BITMASKS = {
    'FENCE_TYPE': {
        0: 'plafon de altitudine (FENCE_ALT_MAX)',
        1: 'cerc centrat pe HOME (FENCE_RADIUS)',
        2: 'cercuri/poligoane de incluziune-excluziune',
    },
    'ARMING_SKIPCHK': BITI_ARMARE,
    # 4.5/4.6: aceiasi biti, sens INVERS - aprins = verificarea se FACE
    'ARMING_CHECK': BITI_ARMARE,
}


def parse_parm(path):
    """[(nume, valoare_ceruta, numar_linie)] din fisierul de parametri."""
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            if ',' in line:
                name, _, val = line.partition(',')
            else:
                parts = line.split()
                if len(parts) != 2:
                    print(f"!! linia {n} nu are forma 'NUME,VALOARE': {line}")
                    continue
                name, val = parts
            out.append((name.strip(), float(val.strip()), n))
    return out


def describe_mask(name, value):
    """Descompune o masca de biti: ce e aprins si ce e stins."""
    bits = BITMASKS[name]
    on, off = [], []
    for bit, meaning in sorted(bits.items()):
        (on if value & (1 << bit) else off).append(f"{bit}:{meaning}")
    lines = [f"      = 0b{value:b}"]
    lines.append(f"      APRINS : {', '.join(on) if on else '(niciunul)'}")
    lines.append(f"      STINS  : {', '.join(off) if off else '(niciunul)'}")
    return "\n".join(lines)


def close_enough(want, got):
    if want == got:
        return True
    return abs(want - got) <= max(ABS_TOL, REL_TOL * abs(want))


def read_param(m, name, timeout=2.0, tries=3):
    """Valoarea raportata de FC, sau None daca parametrul nu exista."""
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode('ascii'), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('ascii', 'ignore')
            if got.rstrip('\x00') == name:
                return msg.param_value
    return None


def is_armed(m, timeout=3.0):
    """True/False dupa HEARTBEAT-ul FC-ului, None daca nu vine niciunul."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is None or hb.get_srcSystem() != m.target_system:
            continue
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def write_params(m, wanted, tries=3):
    """Scrie PE NUME si citeste inapoi. [(nume, cerut, citit_sau_None)].

    Se scrie doar ce difera: un PARAM_SET pe o valoare deja corecta nu
    aduce nimic si umple jurnalul FC-ului. Tipul trimis e REAL32; ArduPilot
    il converteste la tipul stocat al parametrului."""
    out = []
    for name, want, _line in wanted:
        got = read_param(m, name)
        if got is not None and close_enough(want, got):
            out.append((name, want, got))
            continue
        for _ in range(tries):
            m.mav.param_set_send(m.target_system, m.target_component,
                                 name.encode('ascii'), float(want),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.05)
            got = read_param(m, name)
            if got is not None and close_enough(want, got):
                break
        out.append((name, want, got))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='tcp:127.0.0.1:5760')
    p.add_argument('--baud', type=int, default=None)
    p.add_argument('--parm', default=DEFAULT_PARM)
    p.add_argument('--write', action='store_true',
                   help='scrie fisierul PE NUME inainte de verificare. Ocoleste '
                        'filtrul din Mission Planner pentru PLND_*/RNGFND1_*')
    p.add_argument('--reboot', action='store_true',
                   help='cu --write: reporneste FC-ul dupa scriere. Necesar: '
                        'telemetrul si precision landing se initializeaza la boot')
    p.add_argument('--yes', action='store_true',
                   help='fara confirmare la --write')
    args = p.parse_args()

    wanted = parse_parm(args.parm)
    print(f"[check_params] {len(wanted)} parametri din {args.parm}")
    print(f"[check_params] conectare la {args.conn} ...")
    kwargs = {'baud': args.baud} if args.baud else {}
    m = mavutil.mavlink_connection(args.conn, **kwargs)
    m.wait_heartbeat()
    print(f"[check_params] heartbeat sys={m.target_system}\n")

    if args.write:
        armat = is_armed(m)
        if armat is None:
            print("[check_params] REFUZ: niciun HEARTBEAT de la FC, nu scriu.")
            return 2
        if armat:
            print("[check_params] REFUZ: vehiculul e ARMAT. Parametrii nu se "
                  "scriu in zbor sau cu motoarele armate.")
            return 2
        print(f"[check_params] scriu {len(wanted)} parametri din {args.parm} "
              f"PE NUME, cu citire inapoi.")
        if not args.yes:
            r = input("  Vehicul dezarmat, elice demontate? Scrie DA: ")
            if r.strip() != 'DA':
                print("  anulat.")
                return 1
        rez = write_params(m, wanted)
        esuate = [(n, w, g) for n, w, g in rez
                  if g is None or not close_enough(w, g)]
        print(f"  {len(rez) - len(esuate)}/{len(rez)} scrisi si confirmati.")
        for n, w, g in esuate:
            print(f"  ESEC  {n}: cerut {w:g}, citit "
                  f"{'-' if g is None else format(g, 'g')}")
        if esuate:
            print("  Numele de mai sus nu exista pe acest firmware: verifica "
                  "ca --parm corespunde versiunii (4.5 vs 4.7+).")
            return 1
        if args.reboot:
            print("\n  Repornesc FC-ul: telemetrul si precision landing se "
                  "initializeaza doar la boot.")
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0)
            print("  Asteapta ~15 s, apoi ruleaza din nou FARA --write, ca "
                  "citirea inapoi sa se faca dupa boot.")
            return 0
        print("\n  REPORNESTE FC-ul acum (sau da --reboot): telemetrul si "
              "precision landing se initializeaza doar la boot. Apoi ruleaza "
              "din nou fara --write.")
        return 0

    missing, mismatch, ok = [], [], []
    for name, want, line_no in wanted:
        got = read_param(m, name)
        if got is None:
            missing.append((name, want, line_no))
            status = "LIPSESTE  <-- numele nu exista pe acest firmware"
            shown = "-"
        elif close_enough(want, got):
            ok.append(name)
            status = "ok"
            shown = f"{got:g}"
        else:
            mismatch.append((name, want, got, line_no))
            status = "NU SE POTRIVESTE"
            shown = f"{got:g}"
        print(f"  {name:<20} cerut {want:<10g} citit {shown:<12} {status}")
        if name in BITMASKS and got is not None:
            print(describe_mask(name, int(got)))

    print(f"\n  {len(ok)} ok | {len(mismatch)} nepotriviri | "
          f"{len(missing)} inexistenti")

    if missing:
        print("\n  PARAMETRI INEXISTENTI (liniile astea nu fac nimic):")
        for name, want, line_no in missing:
            print(f"    {args.parm}:{line_no}  {name},{want:g}")
        print("  Cauta numele real in sursa ArduPilot (AP_GROUPINFO) si "
              "verifica prefixul obiectului din ArduCopter/Parameters.cpp.")
    if any(n in BITMASKS for n, _, _ in wanted):
        print("\n  Pentru parametrii-masca, verifica lista STINS de mai sus:"
              "\n  o valoare corecta poate dezactiva tacit o functie ceruta de"
              "\n  regulament. Vezi 5.10 din CLAUDE.md.")
    if mismatch:
        print("\n  VALORI DIFERITE DE CE AM CERUT:")
        for name, want, got, line_no in mismatch:
            print(f"    {args.parm}:{line_no}  {name}: cerut {want:g}, "
                  f"FC raporteaza {got:g}")

    return 0 if not missing and not mismatch else 1


if __name__ == '__main__':
    sys.exit(main())
