#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Modul de concurs (H4). O comanda, un ecran, zero decizii.

    python3 tools/race_mode.py

Asta e comanda din ziua cursei. Face, in ordine:

  1. verifica portul serial (H2) si opreste-se cu mesaj daca e ocupat
  2. ruleaza `preflight_check.py` INTEGRAL. Daca ceva pica sau e sarit,
     REFUZA sa porneasca si spune exact ce.
  3. porneste detectorul si poarta de handover
  4. afiseaza un singur ecran, citibil de la un metru

E un lansator subtire peste `tools/nova_pi.py --race`, deliberat: cablajul
(detector, poarta, supervizor, bucla) ramane UNUL SINGUR. §5.14 din CLAUDE.md
a costat o sesiune de zbor tocmai pentru ca aplicatia si testele aveau
cablaje diferite; un al doilea punct de intrare care si-ar construi singur
piesele ar reintroduce exact acea clasa de bug.

Pe banc, fara FC conectat:

    python3 tools/race_mode.py --no-mavlink

dar atunci ecranul o spune: verificarile sarite NU sunt trecute.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import nova_pi                                              # noqa: E402

if __name__ == '__main__':
    argv = sys.argv[1:]
    if '--race' not in argv:
        argv.append('--race')
    sys.argv = [sys.argv[0]] + argv
    sys.exit(nova_pi.main())
