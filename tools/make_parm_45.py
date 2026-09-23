#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Genereaza config/nova_flight_4.5.parm din config/nova_flight.parm.

    python3 tools/make_parm_45.py            # scrie fisierul
    python3 tools/make_parm_45.py --check    # cod 1 daca fisierul e depasit

Nu se editeaza de mana. Fisierul-sursa e scris pentru ArduCopter 4.7.0+;
placa echipei ruleaza 4.5.7. Trei redenumiri au cazut intre 4.6 si 4.7,
iar doua dintre ele au schimbat si UNITATILE (verificat pe tag-urile
ArduPilot, nu din memorie - CLAUDE.md §5.4):

    4.7+ (sursa)          4.5.7 (generat)       conversie
    WP_ACC         m/s/s  WPNAV_ACCEL   cm/s/s  x 100
    WP_RFND_USE           WPNAV_RFND_USE        -
    RNGFND1_MIN    m      RNGFND1_MIN_CM  cm    x 100
    RNGFND1_MAX    m      RNGFND1_MAX_CM  cm    x 100
    RNGFND1_GNDCLR m      RNGFND1_GNDCLEAR cm   x 100, intreg
    ARMING_SKIPCHK        ARMING_CHECK          masca INVERSATA

DE CE GENERAT SI NU SCRIS DE MANA. Masca de armare se inverseaza:
SKIPCHK listeaza ce SARI, CHECK listeaza ce FACI. Un 32768 copiat in
ARMING_CHECK ar insemna "fa DOAR verificarea de telemetru" - fara busola,
GPS, INS, baterie, RC - si ar trece orice audit pe valoare, fiindca
parametrul exista si are numarul cerut (§5.10). La fel un 1.5 copiat in
WPNAV_ACCEL: 1.5 cm/s/s, adica practic zero corectie laterala. Traducerea
e cod, testata, si se regenereaza cand sursa se schimba.
"""

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SURSA = os.path.join(REPO, 'config', 'nova_flight.parm')
TINTA = os.path.join(REPO, 'config', 'nova_flight_4.5.parm')

#: Bitii masca ARMING_CHECK pentru Copter pe 4.5.7 (tag Copter-4.5.7,
#: AP_Arming.cpp, @Bitmask). Bitul 0 = "All"; 9 e Airspeed, doar Plane.
BITI_COPTER_45 = (1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17,
                  18, 19)

#: nume 4.7+ -> (nume 4.5.7, conversie)
REDENUMIRI = {
    'WP_ACC': ('WPNAV_ACCEL', lambda v: _intreg(v * 100.0)),
    'WP_RFND_USE': ('WPNAV_RFND_USE', lambda v: v),
    'RNGFND1_MIN': ('RNGFND1_MIN_CM', lambda v: _intreg(v * 100.0)),
    'RNGFND1_MAX': ('RNGFND1_MAX_CM', lambda v: _intreg(v * 100.0)),
    'RNGFND1_GNDCLR': ('RNGFND1_GNDCLEAR', lambda v: _intreg(v * 100.0)),
    'ARMING_SKIPCHK': ('ARMING_CHECK', lambda v: arming_check_din_skipchk(v)),
}


def _intreg(x):
    return int(round(x))


def arming_check_din_skipchk(skipchk):
    """Masca de verificari FACUTE, din masca de verificari SARITE.

    Bitul 0 ("All") trebuie stins: aprins, ar insemna "fa tot", inclusiv ce
    am vrut sa sarim."""
    skip = int(skipchk)
    return sum(1 << b for b in BITI_COPTER_45 if not skip & (1 << b))


def citeste(cale):
    """[(nume, valoare, linie_originala)]; comentariile se pastreaza separat."""
    out = []
    for linie in open(cale):
        cod = linie.split('#')[0].strip()
        if ',' not in cod:
            continue
        nume, _, val = cod.partition(',')
        out.append((nume.strip(), float(val)))
    return out


def _fmt(v):
    return str(int(v)) if float(v).is_integer() else repr(float(v))


def genereaza():
    linii = [
        "# NOVA - ZDC 2026 - configuratie VEHICUL REAL, ArduCopter 4.5.7",
        "#",
        "# GENERAT de tools/make_parm_45.py din config/nova_flight.parm.",
        "# NU EDITA DE MANA: modifica sursa si regenereaza. Un test pica daca",
        "# fisierul asta nu mai corespunde sursei.",
        "#",
        "# De ce exista: nova_flight.parm e scris pentru 4.7+, iar placa ruleaza",
        "# 4.5.7. Traducerea schimba nume, UNITATI (m -> cm) si inverseaza masca",
        "# de armare - vezi docstring-ul generatorului.",
        "#",
        "# Aceeasi ordine de incarcare ca sursa: RNGFND1_TYPE=10 si",
        "# PLND_ENABLED=1, Write, REBOOT; apoi fisierul; apoi PLND_ENABLED=0.",
        "#",
        "# Verificare prin citire inapoi:",
        "#   python3 tools/check_params.py --conn /dev/serial0 --baud 921600 \\",
        "#       --parm config/nova_flight_4.5.parm",
        "",
    ]
    for nume, val in citeste(SURSA):
        if nume in REDENUMIRI:
            nou, conv = REDENUMIRI[nume]
            v = conv(val)
            linii.append(f"# {nume},{_fmt(val)} pe 4.7+")
            linii.append(f"{nou},{_fmt(v)}")
        else:
            linii.append(f"{nume},{_fmt(val)}")
    return "\n".join(linii) + "\n"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--check', action='store_true',
                   help='nu scrie; cod 1 daca fisierul generat e depasit')
    a = p.parse_args(argv)
    text = genereaza()
    if a.check:
        actual = open(TINTA).read() if os.path.exists(TINTA) else ''
        if actual != text:
            print(f"[parm 4.5] DEPASIT: {TINTA}\n"
                  f"           ruleaza: python3 tools/make_parm_45.py")
            return 1
        print(f"[parm 4.5] la zi: {TINTA}")
        return 0
    with open(TINTA, 'w') as f:
        f.write(text)
    print(f"[parm 4.5] scris: {TINTA} ({len(citeste(SURSA))} parametri)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
