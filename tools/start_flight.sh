#!/usr/bin/env bash
# NOVA - ZDC 2026
# Pornirea completa pentru zbor, pe Raspberry Pi.
#
#   tools/start_flight.sh                 # autonomie ARMATA (cere confirmare)
#   tools/start_flight.sh --monitor       # doar monitorizare, E0 ramane inchis
#   tools/start_flight.sh --yes           # fara confirmare (scripturi)
#   tools/start_flight.sh --dry-run       # arata pasii, nu executa
#
# Face, in ordine:
#
#   1. verifica ca suntem pe Pi si activeaza ~/nova-venv
#   2. elibereaza /dev/serial0 (opreste nova-monitor daca ruleaza)
#   3. ridica autonomy_enabled in config/nova.json          <- E0
#   4. preda controlul lui tools/race_mode.py, care ruleaza preflight-ul
#      INTEGRAL si refuza sa porneasca daca ceva pica sau e sarit
#
# DE CE PASUL 3 EDITEAZA UN FISIER SI NU EXPORTA O VARIABILA
#
#   `nova/config.py` citeste autonomy_enabled DOAR din config/nova.json.
#   Nu exista activare din linia de comanda sau din mediu, si asta e
#   deliberat (§5.16): o garda care se poate ridica dintr-un export se ridica
#   din greseala. Un test verifica explicit ca nu exista cale laterala.
#   Deci scriptul face ce ar face un om: modifica fisierul versionat.
#
#   Consecinta: dupa rulare, `git status` e MURDAR. Nu e un accident - e
#   urma pe care trebuie sa o vezi. `tools/collect_session.py` o va raporta
#   in manifest, iar regulamentul (6.2.1.30) se sprijina pe a sti ce cod a
#   zburat. Fa commit inainte de cursa, cu motivul.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${NOVA_VENV:-$HOME/nova-venv}"
CONFIG="${NOVA_CONFIG:-$REPO/config/nova.json}"
MODEL_FILE="${NOVA_MODEL_FILE:-/proc/device-tree/model}"
SERVICE="nova-monitor"
CONN="${NOVA_CONN:-/dev/serial0}"
BAUD="${NOVA_BAUD:-921600}"

ARM_AUTONOMY=1
ASSUME_YES=0
DRY_RUN=0
EXTRA=()

say()  { printf '\n\033[1m[flight]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[flight] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[flight] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }
run()  { printf '  + %s\n' "$*"; [[ $DRY_RUN -eq 1 ]] || "$@"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --monitor)  ARM_AUTONOMY=0 ;;
    --yes|-y)   ASSUME_YES=1 ;;
    --dry-run)  DRY_RUN=1 ;;
    --conn)     CONN="$2"; shift ;;
    --baud)     BAUD="$2"; shift ;;
    -h|--help)  sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)          EXTRA+=("$1") ;;     # restul merge catre race_mode.py
  esac
  shift
done

# --- 1. platforma + venv ---------------------------------------------------
model='necunoscut'
[[ -r "$MODEL_FILE" ]] && model=$(tr -d '\0' < "$MODEL_FILE")
case "$model" in
  *"Raspberry Pi"*) ;;
  *) warn "modelul nu contine 'Raspberry Pi' ($model).
       Scriptul e pentru vehicul. Pe desktop foloseste SITL:
         ./start_sim.sh" ;;
esac

[[ -x "$VENV/bin/python" ]] || die "nu exista $VENV/bin/python.
       Ruleaza intai:  tools/setup_pi.sh"

say "1/4  mediu"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
printf '  venv    %s\n' "$VIRTUAL_ENV"
printf '  python  %s\n' "$(python3 -c 'import sys;print(sys.version.split()[0])')"
printf '  cv2     %s\n' "$(python3 -c 'import cv2;print(cv2.__version__)' 2>/dev/null || echo LIPSA)"
printf '  numpy   %s din %s\n' \
  "$(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null || echo LIPSA)" \
  "$(python3 - <<'PY' 2>/dev/null || echo '?'
import numpy, os, sys
print('VENV' if os.path.realpath(numpy.__file__).startswith(
    os.path.realpath(sys.prefix) + os.sep) else 'sistem (apt)')
PY
)"

# --- 2. portul serial ------------------------------------------------------
say "2/4  portul serial ($CONN)"
if [[ "$CONN" != udp* && "$CONN" != tcp* ]]; then
  if systemctl is-active --quiet "$SERVICE" 2>/dev/null; then
    # §5.27: doua procese nu pot tine acelasi port. Mai bine il eliberam
    # acum, explicit, decat sa esueze peste 30 s cu "resource busy".
    say "     $SERVICE ruleaza si tine portul - il opresc"
    run sudo systemctl stop "$SERVICE"
  else
    printf '  %s nu ruleaza, portul e liber\n' "$SERVICE"
  fi
  [[ -e "$CONN" ]] || warn "$CONN nu exista inca (cablu? enable_uart=1?)"
fi

# --- 3. E0 -----------------------------------------------------------------
e0_state() { python3 - "$CONFIG" <<'PY'
import json, sys
try:
    print('true' if json.load(open(sys.argv[1])).get('autonomy_enabled') is True
          else 'false')
except Exception:
    print('?')
PY
}

set_e0() {                       # set_e0 true|false
  python3 - "$CONFIG" "$1" <<'PY'
import re, sys
cale, val = sys.argv[1], sys.argv[2]
s = open(cale).read()
# Inlocuire tintita, nu json.dump: fisierul contine cheile "_comment" si
# "_E0" cu explicatia de ce exista garda. Rescris prin json.dump ar pastra
# cheile dar ar pierde formatarea si ordinea, iar acel text e jumatate din
# valoarea fisierului.
nou, n = re.subn(r'("autonomy_enabled"\s*:\s*)(true|false)',
                 lambda m: m.group(1) + val, s, count=1)
if n != 1:
    sys.exit(f"nu am gasit exact o cheie autonomy_enabled in {cale}")
open(cale, 'w').write(nou)
PY
}

say "3/4  E0 - garda de autonomie"
inainte="$(e0_state)"
printf '  acum: autonomy_enabled = %s\n' "$inainte"

if [[ $ARM_AUTONOMY -eq 1 ]]; then
  if [[ "$inainte" == "true" ]]; then
    printf '  deja armata\n'
  else
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────────────┐
  │  RIDIC GARDA DE AUTONOMIE (E0)                                       │
  │                                                                      │
  │  Dupa asta, poarta de handover POATE accepta, iar companion-ul       │
  │  POATE comanda LAND si coborarea pe marker.                          │
  │                                                                      │
  │  Politica scrisa in config/nova.json si in §5.16: se ridica DOAR     │
  │  dupa ce E2 (validarea offline a detectorului) a trecut criteriile,  │
  │  cu un commit separat care citeaza raportul E2.                      │
  │                                                                      │
  │  Daca E2 nu a trecut inca, zboara cu --monitor.                      │
  └──────────────────────────────────────────────────────────────────────┘

EOF
    if [[ $ASSUME_YES -eq 0 && $DRY_RUN -eq 0 ]]; then
      read -r -p "  Scrie ARMEZ ca sa confirmi: " raspuns
      [[ "$raspuns" == "ARMEZ" ]] || die "neconfirmat. Nimic nu s-a schimbat."
    fi
    run set_e0 true
  fi
else
  if [[ "$inainte" == "true" ]]; then
    say "     --monitor: cobor autonomy_enabled la false"
    run set_e0 false
  fi
fi

dupa="$(e0_state)"
# §5.10 aplicat unui fisier JSON: citeste inapoi, nu presupune.
if [[ $DRY_RUN -eq 0 ]]; then
  asteptat=$([[ $ARM_AUTONOMY -eq 1 ]] && echo true || echo false)
  [[ "$dupa" == "$asteptat" ]] || die "am cerut $asteptat, fisierul zice $dupa"
fi
printf '  acum: autonomy_enabled = %s\n' "$dupa"

if [[ -d "$REPO/.git" ]] && ! git -C "$REPO" diff --quiet -- "$CONFIG" 2>/dev/null; then
  warn "config/nova.json e MODIFICAT si necomis.
       collect_session.py va raporta arborele ca murdar, iar atunci nu se mai
       poate spune exact ce cod a zburat (6.2.1.30). Inainte de cursa:
         git -C $REPO commit -am 'E0: autonomie armata dupa E2 <referinta>'"
fi

# --- 4. zbor ---------------------------------------------------------------
say "4/4  pornesc modul de concurs"
printf '  race_mode.py ruleaza preflight-ul INTEGRAL si refuza sa porneasca\n'
printf '  daca ceva pica SAU e sarit. Cod 4 = preflight, 3 = port, 2 = calibrare.\n\n'

if [[ $DRY_RUN -eq 1 ]]; then
  printf '  + python3 %s/tools/race_mode.py --conn %s --baud %s %s\n' \
    "$REPO" "$CONN" "$BAUD" "${EXTRA[*]-}"
  exit 0
fi

cd "$REPO"
exec python3 tools/race_mode.py --conn "$CONN" --baud "$BAUD" ${EXTRA[@]+"${EXTRA[@]}"}
