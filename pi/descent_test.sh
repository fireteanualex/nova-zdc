#!/usr/bin/env bash
# NOVA - ZDC 2026
# Proba de coborare autonoma pe vehiculul de test.
#
#   pi/descent_test.sh --check        # doar preconditiile, nu zboara nimic
#   pi/descent_test.sh                # briefing + confirmare + rulare
#   pi/descent_test.sh --full-sequence  # cu urcarea la 5 m (15.2.7)
#   pi/descent_test.sh --auto   # din pornirea automata (config: autostart=zbor)
#
# --auto e pentru teren FARA RETEA: il cheama pi/bringup.sh la boot, deci
# nu opreste pornirea automata (ESTE pornirea automata) si nu cere ZBOR
# tastat - decizia e `"autostart": "zbor"` in config/nova.json, cu E0
# deschis. Verificarile sunt ACELEASI: daca una pica, iese cu 4 si
# pornirea automata cade in monitor, cu motivul trimis pilotului.
#
# ===========================================================================
# CE FACE VEHICULUL, PAS CU PAS
# ===========================================================================
#
#   pilotul aduce vehiculul la 5-12 m deasupra markerului, in LOITER
#   pilotul ridica AUX (can. 8)   -> poarta valideaza si ACCEPTA sau REFUZA
#   companion-ul cere LAND        -> asteapta confirmarea FC-ului
#   coborare cu PLND              -> LANDING_TARGET la 20 Hz din camera
#   incadrarea atinge final_fill  -> coborare verticala, fara corectii
#   contact                       -> pauza pe sol
#   implicit AICI SE OPRESTE. Cu --full-sequence urmeaza urcarea la 5 m.
#
# ===========================================================================
# CELE TREI CAI DE ABORT, IN ORDINEA INCREDERII
# ===========================================================================
#
#   1. COMUTATORUL DE MOD de pe emitator (FLTMODE_CH).
#      Merge direct in FC. NU trece prin Raspberry Pi, deci functioneaza si
#      daca Pi-ul e mort, blocat sau deconectat. Asta e abortul real.
#      Scriptul REFUZA sa porneasca daca nu e configurat.
#
#   2. MANSELE. Companion-ul le vede in RC_CHANNELS si comanda LOITER.
#      Masurat in SITL: 150 ms de la miscare pana la mod confirmat de FC.
#      Depinde de Pi, deci e al doilea strat, nu primul.
#
#   3. SAFETY SUPERVISOR. Pierderea detectiei, raza, plafon, inclinare,
#      rata de coborare -> BRAKE sau RTL. Automat, si tot prin Pi.
#
# Pilotul trebuie sa aiba mana pe comutatorul de mod tot timpul. Segmentul
# dureaza ~30 s; nu e un moment in care sa te uiti la ecran.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${NOVA_VENV:-$HOME/nova-venv}"
CONN="${NOVA_CONN:-/dev/serial0}"
BAUD="${NOVA_BAUD:-921600}"
LOG_DIR="${NOVA_LOG_DIR:-$HOME/nova-logs}"
CONFIG="$REPO/config/nova.json"

CHECK_ONLY=0
FULL_SEQ=0
ASSUME_YES=0
AUTO=0
# Canalul pe care pilotul CERE segmentul autonom, pe frontul crescator.
# Modul LAND nu declanseaza nimic: intrarea e doar prin canalul asta (§8).
AUX_CH="${NOVA_AUX_CH:-}"       # gol = cel din config/nova.json

say()  { printf '\n\033[1m[coborare]\033[0m %s\n' "$*"; }
ok()   { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
bad()  { printf '  \033[31mLIPSA\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[coborare] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[coborare] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

for arg in "$@"; do
  case "$arg" in
    --check)         CHECK_ONLY=1 ;;
    --aux-channel=*) AUX_CH="${arg#*=}" ;;
    --full-sequence) FULL_SEQ=1 ;;
    --yes)           ASSUME_YES=1 ;;
    --auto)          AUTO=1; ASSUME_YES=1 ;;
    -h|--help)       sed -n '2,8p' "$0"; exit 0 ;;
    *) die "optiune necunoscuta: $arg" ;;
  esac
done

mkdir -p "$LOG_DIR"

# Pornirea automata (pi/install.sh) tine camera si portul. Proba de coborare
# are nevoie de amandoua - inclusiv preflight-ul, care deschide camera
# INAINTEA aplicatiei. Deci se opreste aici, explicit si spus, nu lasat pe
# seama lui --stop-service din nova_pi.py, care vine prea tarziu.
# Cu --auto scriptul RULEAZA in pornirea automata: oprind-o, s-ar opri pe el.
if [[ $AUTO -eq 0 ]] && command -v systemctl >/dev/null \
   && systemctl --user is-active --quiet nova-bringup 2>/dev/null; then
  printf '\n\033[1m[coborare]\033[0m opresc pornirea automata (nova-bringup): '
  printf 'tine camera si portul.\n'
  printf '  o repornesti dupa proba cu: systemctl --user start nova-bringup\n'
  systemctl --user stop nova-bringup || die "nu am putut opri nova-bringup"
fi
[[ -x "$VENV/bin/python" ]] || die "nu gasesc venv-ul la $VENV"
PY="$VENV/bin/python"
# Canalul de handover: cel din config/nova.json, acelasi pe care il asculta
# si pornirea automata. --aux-channel=N il suprascrie doar pentru proba asta.
if [[ -z "$AUX_CH" ]]; then
  AUX_CH="$("$PY" -c "import sys; sys.path.insert(0, '$REPO')
from nova import config; print(config.load()['aux_channel'])")"
fi
PROBLEME=0
nu_e_gata() { bad "$1"; PROBLEME=$((PROBLEME + 1)); }

# --- 1. E0: garda de autonomie ---------------------------------------------
say "E0 - garda de autonomie"
E0="$("$PY" - "$CONFIG" <<'EOF'
import json, sys
try:
    print(str(json.load(open(sys.argv[1])).get('autonomy_enabled')).lower())
except Exception:
    print('false')
EOF
)"
if [[ "$E0" == "true" ]]; then
  ok "autonomy_enabled = true in config/nova.json"
else
  nu_e_gata "autonomy_enabled = false -> poarta REFUZA orice handover"
  cat <<'NOTA'
        Garda se ridica din FISIERUL VERSIONAT, nu din linia de comanda si
        nu dintr-o variabila de mediu (§5.16). Deliberat: o garda care se
        poate ridica dintr-un export se ridica din greseala.

        Regula proiectului: se ridica DUPA ce trece E2 (validarea offline a
        detectorului - marker tiparit, ruleta, trei conditii de lumina),
        cu un commit care citeaza raportul:

            tools/run_e2.py --help
            nano config/nova.json          # autonomy_enabled: true
            git commit -am "E0 ridicat: raport E2 <data>"

        Scriptul asta NU o ridica singur. Nu e birocratie: e singurul loc
        din proiect unde se poate spune, la scrutineering, CINE a decis si
        pe ce dovada.
NOTA
fi

# --- 2. calibrarea ---------------------------------------------------------
say "calibrarea camerei"
NOVA_REPO="$REPO" "$PY" - <<'EOF' || true
import os, sys
sys.path.insert(0, os.environ['NOVA_REPO'])
from nova.detector_pi import CameraCalibration, MAX_REPROJ_ERR_PX
cale = os.path.join(os.environ['NOVA_REPO'], 'config', 'camera_pi.yaml')
try:
    # acelasi prag ca zborul: MAX_REPROJ_ERR_PX (0.85, decizia echipei)
    cal = CameraCalibration.load(cale, require_real=True)
except Exception as e:                                        # noqa: BLE001
    print(f"  LIPSA calibrare utilizabila: {e}")
    raise SystemExit(0)
print(f"  OK    fy={cal.fy:.1f} px  VFOV={cal.vfov_deg():.1f} deg  "
      f"rms={cal.rms:.3f} px")
if cal.rms and cal.rms > 0.5:
    print(f"  NOTA: rms {cal.rms:.3f} px - acceptat (prag {MAX_REPROJ_ERR_PX}),")
    print(f"        dar peste 0.2-0.5 cat da o calibrare buna. Distanta din")
    print(f"        solvePnP comanda coborarea: urmareste-o fata de altimetru.")
EOF

# --- 3. caile de abort -----------------------------------------------------
say "caile de abort"
set +e
NOVA_REPO="$REPO" NOVA_AUX_CH="$AUX_CH" "$PY" - "$CONN" "$BAUD" <<'EOF'
import os, sys, time
sys.path.insert(0, os.environ['NOVA_REPO'])
from nova.vehicle import Vehicle

conn, baud = sys.argv[1], int(sys.argv[2])
try:
    v = Vehicle(conn, baud=None if conn.startswith(('udp', 'tcp')) else baud)
    v.connect()
except Exception as e:                                        # noqa: BLE001
    print(f"  LIPSA legatura cu FC-ul: {e}")
    raise SystemExit(3)

t0 = time.monotonic()
aux_ch = int(os.environ.get('NOVA_AUX_CH', '7'))
for nume in ('FLTMODE_CH', f'RC{aux_ch}_OPTION', 'FENCE_ENABLE'):
    v.request_param(nume)
while time.monotonic() - t0 < 4.0:
    v.pump()
    time.sleep(0.01)

p = v.params
probleme = 0

# 1. Comutatorul de mod: abortul care NU trece prin Pi (16.2.3).
ch = p.get('FLTMODE_CH')
if ch is None:
    print("  ? FLTMODE_CH nu a raspuns (FC vechi sau legatura lenta)")
elif int(ch) == 0:
    print("  LIPSA FLTMODE_CH = 0: NU exista comutator de mod pe emitator.")
    print("        Asta e singura cale de abort care functioneaza si daca")
    print("        Pi-ul e mort. Fara ea nu se zboara autonom.")
    probleme += 1
elif int(ch) == aux_ch:
    print(f"  LIPSA FLTMODE_CH = {aux_ch}, ACELASI canal cu handover-ul.")
    print(f"        Comutatorul ar schimba modul de zbor SI ar cere segmentul")
    print(f"        autonom deodata. Muta unul dintre ele pe alt canal.")
    probleme += 1
else:
    print(f"  OK    FLTMODE_CH = {int(ch)} (abort hardware, ocoleste Pi-ul)")

# 2. Canalul de handover: singura cale de INTRARE in segment (§8).
opt = p.get(f'RC{aux_ch}_OPTION')
if opt is not None and int(opt) != 0:
    print(f"  ATENTIE: RC{aux_ch}_OPTION = {int(opt)}. Canalul are deja o")
    print(f"           functie in ArduPilot; poarta il citeste oricum din")
    print(f"           RC_CHANNELS, dar verifica sa nu faca si altceva.")
else:
    print(f"  OK    RC{aux_ch}_OPTION = 0 (canalul {aux_ch} e liber)")

# 3. Heartbeat: monitorul de legatura al supervizorului depinde de el.
iv = v.heartbeat_interval() if hasattr(v, 'heartbeat_interval') else None
if iv:
    print(f"  OK    HEARTBEAT la {1.0 / iv:.1f} Hz (interval {iv * 1000:.0f} ms)")

raise SystemExit(1 if probleme else 0)
EOF
# `set -e` ar omori scriptul pe codul de iesire al blocului de mai sus
# INAINTE sa apucam sa-l citim - deci verdictul s-ar pierde exact cand e
# negativ. Codul se captureaza, nu se lasa sa propage.
ABORT_COD=$?
set -e
if [[ $ABORT_COD -eq 3 ]]; then
  nu_e_gata "fara legatura cu FC-ul - verifica UART-ul (pi/setup_uart.sh)"
elif [[ $ABORT_COD -ne 0 ]]; then
  nu_e_gata "caile de abort nu sunt complete (vezi mai sus)"
fi

# --- 4. emitatorul ---------------------------------------------------------
# La boot nu citeste nimeni nota; in jurnal ar fi doar zgomot.
[[ $AUTO -eq 1 ]] || { say "emitatorul"; cat <<'NOTA'
  Doua masuratori care NU se pot deduce, si care nu se fac in zbor:

    tools/calibrate_sticks.py --conn /dev/serial0 --baud 921600
        zgomotul manselor in repaus -> STICK_DEADBAND_PWM. Implicitul de 80
        e PROVIZORIU, masurat pe un gamepad cu iesire cuantizata. Prea mic:
        override fals in mijlocul coborarii. Prea mare: pilotul trage si nu
        se intampla nimic.

    tools/check_rc_override.py --conn /dev/serial0 --baud 921600
        FC-ul chiar raporteaza inapoi in RC_CHANNELS ce primeste? Daca nu,
        poarta nu vede comutatorul AUX si monitorul de override nu exista.
NOTA
}

# --- 5. preflight ----------------------------------------------------------
say "preflight"
set +e
"$PY" "$REPO/tools/preflight_check.py" \
  --conn "$CONN" --baud "$BAUD"
[[ $? -eq 0 ]] || nu_e_gata "preflight-ul nu a trecut integral"
set -e

# --- verdict ---------------------------------------------------------------
if [[ $PROBLEME -gt 0 ]]; then
  say "NU E GATA: $PROBLEME lucruri de rezolvat"
  printf '  Nimic nu s-a pornit. Rezolva-le si reia.\n\n'
  # 4 = "verificarile au picat, nimic pornit". pi/bringup.sh il deosebeste
  # de o aplicatie cazuta: pe 4 cade in monitor, cu motivul spus pilotului.
  exit 4
fi
say "toate preconditiile sunt indeplinite"

if [[ $CHECK_ONLY -eq 1 ]]; then
  printf '  --check: ma opresc aici.\n\n'
  exit 0
fi

# --- briefing si confirmare ------------------------------------------------
ARGS=(--conn "$CONN" --baud "$BAUD" --stop-service --yes
      --no-authority --aux-channel "$AUX_CH")
if [[ $FULL_SEQ -eq 0 ]]; then
  ARGS+=(--no-ascent)
fi

cat <<FIN

  =========================================================================
   PROBA DE COBORARE AUTONOMA - vehicul REAL
  =========================================================================

   Ce faci tu, pilotul:
     1. decolezi si aduci vehiculul la 5-12 m DEASUPRA markerului,
        in raza de 6.5 m lateral, in LOITER
     2. manse libere, in neutru, ~1 s (poarta masoara in fereastra asta)
     3. ridici comutatorul de pe canalul RC $AUX_CH
     4. MANA PE COMUTATORUL DE MOD pana se termina

   Ce face vehiculul:
     LAND cu precision landing, coborare, contact, pauza pe sol
$(if [[ $FULL_SEQ -eq 1 ]]; then
    echo "     apoi URCARE AUTOMATA la 5 m deasupra markerului (15.2.7)"
  else
    echo "     si SE OPRESTE. Fara urcare (--no-ascent)."
    echo "     ArduPilot dezarmeaza singur dupa contact."
  fi)

   Daca poarta refuza, spune de ce si nu se intampla nimic. Un refuz
   costa cateva secunde; o incercare gresita costa vehiculul.

   Abort, in ordine: comutatorul de mod > mansele > supervizorul.

  =========================================================================

FIN

if [[ $ASSUME_YES -eq 0 ]]; then
  printf '  Scrie ZBOR ca sa continui (orice altceva anuleaza): '
  read -r raspuns
  [[ "$raspuns" == "ZBOR" ]] || die "anulat"
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOG_DIR/coborare-$STAMP.log"
say "pornesc. log: $LOG"
cd "$REPO"
"$PY" -u tools/nova_pi.py "${ARGS[@]}" 2>&1 | tee "$LOG"
COD="${PIPESTATUS[0]}"

say "gata. evidenta:"
printf '  log:     %s\n' "$LOG"
printf '  imagini: %s\n' "$REPO/data/scoring/"
printf '  .bin de pe FC: descarca-l si tine-l langa log (6.2.1.30)\n'
printf '  tools/collect_session.py aduna tot intr-un manifest\n\n'
exit "$COD"
