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
    ARMING_SKIPCHK        (NU se genereaza)     decizia echipei, mai jos

DE CE GENERAT SI NU SCRIS DE MANA. Un 1.5 copiat in WPNAV_ACCEL ar fi
1.5 cm/s/s, adica practic zero corectie laterala - si ar trece orice audit
pe valoare, fiindca parametrul exista si are numarul cerut (§5.10).
Traducerea e cod, testata, si se regenereaza cand sursa se schimba.

ARMING_CHECK NU E IN FISIERUL GENERAT. Decizia echipei, 24.09.2026: pe
vehiculul de test verificarile la armare le seteaza echipa, de mana -
mediul de test nu are GPS lock, iar masca impusa de noi bloca decolarea.
Cu el in fisier, preflight-ul pica pe "1 nepotriviri" si `--write` ar fi
suprascris valoarea echipei. Traducerea lui nu s-a pierdut in sursa:
masca e INVERSA (SKIPCHK = ce sari, CHECK = ce faci), iar un 32768 copiat
direct ar insemna "fa DOAR verificarea de telemetru".

Singurul bit de care depinde proba: 15 (rangefinder) trebuie STINS, altfel
armarea pica cu "Rangefinder 1: No Data" (§5.13). pi/descent_test.sh il
citeste si avertizeaza; nu refuza.
"""

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SURSA = os.path.join(REPO, 'config', 'nova_flight.parm')
TINTA = os.path.join(REPO, 'config', 'nova_flight_4.5.parm')

#: Parametri din sursa care NU trec in fisierul pentru 4.5.7 - decizii ale
#: echipei, nu omisiuni. Vezi docstring-ul.
OMISI = {'ARMING_SKIPCHK': 'verificarile la armare le seteaza echipa de '
                           'mana (decizia din 24.09.2026)'}

#: nume 4.7+ -> (nume 4.5.7, conversie)
REDENUMIRI = {
    'WP_ACC': ('WPNAV_ACCEL', lambda v: _intreg(v * 100.0)),
    'WP_RFND_USE': ('WPNAV_RFND_USE', lambda v: v),
    'RNGFND1_MIN': ('RNGFND1_MIN_CM', lambda v: _intreg(v * 100.0)),
    'RNGFND1_MAX': ('RNGFND1_MAX_CM', lambda v: _intreg(v * 100.0)),
    'RNGFND1_GNDCLR': ('RNGFND1_GNDCLEAR', lambda v: _intreg(v * 100.0)),
}


def _intreg(x):
    return int(round(x))


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
        if nume in OMISI:
            linii.append(f"# {nume},{_fmt(val)} pe 4.7+ - OMIS aici: "
                         f"{OMISI[nume]}")
            linii.append("#   pe 4.5.7 bitul 15 din ARMING_CHECK (rangefinder)"
                         " trebuie STINS")
            continue
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
