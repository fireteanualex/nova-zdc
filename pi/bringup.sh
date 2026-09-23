#!/usr/bin/env bash
# NOVA - ZDC 2026
# Bring-up pe Raspberry Pi 4 + Pixhawk 6C (TELEM2 pe GPIO 14/15).
#
#   pi/bringup.sh              # verificari + monitor cu fereastra pe tot ecranul
#   pi/bringup.sh --check      # doar verificarile, nu porneste nimic
#   pi/bringup.sh --no-window  # fara fereastra (SSH fara X, sau ca serviciu)
#
# Pornit la fiecare boot de `nova-bringup.service` (vezi pi/install.sh).
#
# ===========================================================================
# CE FACE, SI CE NU FACE
# ===========================================================================
#
# Porneste companion-ul in **RACE_MONITOR**: detectorul merge, telemetria se
# citeste, fereastra arata ce vede camera - si NU pleaca nicio comanda catre
# vehicul. E0 (`config/nova.json: autonomy_enabled`) ramane inchis.
#
# Asta nu e o limitare a bring-up-ului, e chiar ce vrei sa masori primul:
#   - UART-ul tine 921600 fara sa piarda caractere?
#   - camera da 30 fps cu detectorul pornit, pe Pi, nu pe desktop?
#   - markerul se detecteaza pe hartie reala, la lumina reala?
#   - latenta captura -> publicare, p99, pe Pi 4 (criteriul E1.4: sub 150 ms)
#
# Niciuna dintre astea nu cere ca vehiculul sa se miste, si toate patru
# trebuie sa fie verzi inainte sa se miste.
#
# Pentru coborarea autonoma propriu-zisa: vezi pi/README.md, sectiunea
# "De la monitor la coborare". Trece prin E2, care se face tot pe masa.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${NOVA_VENV:-$HOME/nova-venv}"
CONN="${NOVA_CONN:-/dev/serial0}"
BAUD="${NOVA_BAUD:-921600}"
LOG_DIR="${NOVA_LOG_DIR:-$HOME/nova-logs}"

CHECK_ONLY=0
WINDOW=1
# Pragul de reproiectie e cel din cod (MAX_REPROJ_ERR_PX, 0.85 - decizia
# echipei). Bancul si zborul folosesc ACELASI prag: un bring-up mai
# permisiv decat zborul ar valida o configuratie care nu zboara.

say()  { printf '\n\033[1m[bringup]\033[0m %s\n' "$*"; }
ok()   { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
bad()  { printf '  \033[31mESEC\033[0m  %s\n' "$*"; }
warn() { printf '\033[33m[bringup] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[bringup] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

for arg in "$@"; do
  case "$arg" in
    --check)     CHECK_ONLY=1 ;;
    --no-window) WINDOW=0 ;;
    -h|--help)   sed -n '2,9p' "$0"; exit 0 ;;
    *) die "optiune necunoscuta: $arg" ;;
  esac
done

mkdir -p "$LOG_DIR"

# --- 0. o singura instanta -------------------------------------------------
# Serviciul de pornire automata (pi/install.sh) ruleaza chiar scriptul asta.
# Pornit si de mana peste el, a doua copie gaseste camera luata ("Pipeline
# handler in use by another process") si portul ocupat - iar doua procese
# pe acelasi UART isi fura octetii unul altuia: HEARTBEAT-ul trece, citirile
# de parametri se pierd. Asa s-a intamplat prima data pe vehicul.
#
# Cand scriptul E serviciul, PID-ul lui e chiar MainPID-ul serviciului.
if command -v systemctl >/dev/null \
   && systemctl --user is-active --quiet nova-bringup 2>/dev/null; then
  MAIN_PID="$(systemctl --user show -p MainPID --value nova-bringup 2>/dev/null)"
  if [[ "$MAIN_PID" != "$$" ]]; then
    die "pornirea automata (nova-bringup) ruleaza deja - tine camera si portul.
    Vezi ce face:        journalctl --user-unit nova-bringup -f
    Opreste-o ca sa rulezi de mana:
                         systemctl --user stop nova-bringup
    Si apoi o repornesti: systemctl --user start nova-bringup"
  fi
fi

# --- 1. mediul -------------------------------------------------------------
say "mediul"
[[ -x "$VENV/bin/python" ]] || die "nu gasesc venv-ul la $VENV (ruleaza tools/setup_pi.sh)"
PY="$VENV/bin/python"
ok "venv: $VENV"
"$PY" - <<'EOF'
import sys
print(f"  python {sys.version.split()[0]}  ({sys.executable})")
for mod in ('cv2', 'numpy', 'pymavlink', 'picamera2'):
    try:
        m = __import__(mod)
        v = getattr(m, '__version__', '?')
        print(f"  {mod:<12} {v:<10} {getattr(m, '__file__', '?')}")
    except Exception as e:                                    # noqa: BLE001
        print(f"  {mod:<12} LIPSA      {type(e).__name__}: {e}")
EOF

# --- 2. UART-ul ------------------------------------------------------------
say "UART (GPIO 14/15 -> Pixhawk TELEM2)"
if [[ -e "$CONN" ]]; then
  TINTA="$(readlink -f "$CONN")"
  case "$TINTA" in
    *ttyAMA*) ok "$CONN -> $TINTA (PL011)" ;;
    *ttyS0)
      bad "$CONN -> $TINTA (miniUART)"
      warn "miniUART-ul isi ia tactul din ceasul miezului VPU, care se"
      warn "scaleaza cu incarcarea. La 921600 legatura merge si apoi incepe"
      warn "sa dea caractere gresite - si arata ca un cablu prost."
      warn "Repara: sudo pi/setup_uart.sh && sudo reboot"
      ;;
    *) warn "$CONN -> $TINTA (neasteptat)" ;;
  esac
else
  bad "$CONN nu exista - ruleaza: sudo pi/setup_uart.sh && sudo reboot"
fi

# Cine tine portul (§5.27): raspunsul e util doar daca e dat INAINTE de a
# incerca sa-l deschizi. `Errno 16` spune CE, nu spune CINE.
# Un port ocupat OPRESTE scriptul. Prima varianta doar avertiza si mergea
# mai departe, in preflight si intr-un al doilea monitor - adica exact in
# conflictul pe care tocmai il detectase.
set +e
NOVA_REPO="$REPO" "$PY" - "$CONN" <<'EOF'
import os, sys
sys.path.insert(0, os.environ['NOVA_REPO'])
from nova import serial_guard
motiv = serial_guard.describe_conflict(sys.argv[1])
cam = serial_guard.describe_camera_conflict()
if motiv or cam:
    for m in (motiv, cam):
        if m:
            for linie in str(m).splitlines():
                print(f"    {linie}")
    raise SystemExit(3)
print(f"  OK    nimeni altcineva nu tine {sys.argv[1]} sau camera")
EOF
OCUPAT=$?
set -e
[[ $OCUPAT -eq 3 ]] && die "portul sau camera sunt luate de alt proces - vezi mai sus"


# --- 3. calibrarea camerei -------------------------------------------------
say "calibrarea camerei"
NOVA_REPO="$REPO" "$PY" - <<'EOF'
import os, sys
sys.path.insert(0, os.environ['NOVA_REPO'])
from nova.detector_pi import CameraCalibration
cale = os.path.join(os.environ['NOVA_REPO'], 'config', 'camera_pi.yaml')
if not os.path.exists(cale):
    print("  LIPSA config/camera_pi.yaml")
    print("  Detectorul REFUZA sa porneasca fara o calibrare reala (E1.2).")
    print("  Ori copiezi fisierul tau aici, ori il faci pe loc:")
    print("      python3 tools/calibrate_camera.py --help")
    raise SystemExit(0)
try:
    from nova.detector_pi import MAX_REPROJ_ERR_PX
    cal = CameraCalibration.load(cale, require_real=True)
    print(f"  OK    {cale}")
    print(f"        fy={cal.fy:.1f} px  HFOV={cal.hfov_deg():.1f} "
          f"VFOV={cal.vfov_deg():.1f} deg  rms={cal.rms:.3f} px  n={cal.n_images}")
    # Acceptata (sub pragul de MAX_REPROJ_ERR_PX), dar peste ce da de obicei
    # o calibrare buna. Nu blocheaza - se spune, ca sa nu se uite.
    if cal.rms and cal.rms > 0.5:
        print(f"  NOTA: rms {cal.rms:.3f} px - acceptat (prag "
              f"{MAX_REPROJ_ERR_PX}, decizia echipei), dar peste 0.2-0.5, cat")
        print(f"        da de obicei o calibrare buna. E2 spune daca ajunge:")
        print(f"        eroarea de distanta fata de ruleta.")
except Exception as e:                                        # noqa: BLE001
    print(f"  ESEC  {cale}: {e}")
EOF

# --- 4. telecomanda: override si comutatorul de handover -------------------
say "telecomanda (doar informativ la bring-up)"
cat <<'NOTA'
  Monitorul de override si poarta de handover sunt cablate in nova_pi.py,
  dar in bring-up nu se declanseaza: supervizorul se armeaza din FAZA, iar
  cu E0 inchis secventa ramane in IDLE, deci nu exista nimic de preluat.
  Asta e corect - companion-ul nu comanda nimic, deci pilotul are oricum
  controlul integral prin FC.

  Inainte de proba de coborare, doua masuratori cu emitatorul REAL:

    tools/calibrate_sticks.py --conn /dev/serial0 --baud 921600
        zgomotul manselor in repaus -> STICK_DEADBAND_PWM (elementul 13).
        Cifra de pe gamepad NU se transfera: 0 PWM masurat acolo e o
        iesire cuantizata, nu un gimbal analogic cu link RC.

    tools/check_rc_override.py --conn /dev/serial0 --baud 921600
        verifica preconditia intregului lant: FC-ul chiar raporteaza inapoi
        in RC_CHANNELS ce primeste de la emitator. Daca nu, poarta nu vede
        comutatorul AUX 7 si monitorul de override nu functioneaza.
NOTA

# --- 5. preflight ----------------------------------------------------------
say "preflight (camera, legatura, parametri)"
# Ruleaza in AMBELE cazuri. Un bring-up care porneste monitorul fara sa
# spuna ce a gasit preflight-ul e un bring-up care ascunde exact ce ai
# venit sa afli. Nu blocheaza pornirea - pe masa, jumatate din verificari
# pica legitim (nu e vehicul armat, nu e GPS) - dar se vede.
  "$PY" "$REPO/tools/preflight_check.py" --conn "$CONN" --baud "$BAUD" || \
  warn "preflight-ul nu a trecut integral - citeste ce e rosu mai sus"

if [[ $CHECK_ONLY -eq 1 ]]; then
  say "--check: ma opresc aici, nu pornesc monitorul"
  exit 0
fi

# --- 6. monitorul ----------------------------------------------------------
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOG_DIR/bringup-$STAMP.log"
say "pornesc monitorul (E0 INCHIS: zero comenzi catre vehicul)"
printf '  log: %s\n' "$LOG"

ARGS=(--conn "$CONN" --baud "$BAUD" --stop-service --yes)
if [[ $WINDOW -eq 1 ]]; then
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    ARGS+=(--fullscreen)
    printf '  fereastra: pe tot ecranul (iesire: q sau Escape)\n'
  else
    warn "fara DISPLAY/WAYLAND_DISPLAY: pornesc fara fereastra."
    warn "Prin SSH foloseste 'ssh -X', sau ruleaza din sesiunea grafica."
  fi
fi

cd "$REPO"
# Fara `exec`: vrem ca `tee` sa primeasca si iesirea, si codul de iesire al
# lui nova_pi.py, nu al lui tee (de aici PIPESTATUS).
"$PY" -u tools/nova_pi.py "${ARGS[@]}" 2>&1 | tee "$LOG"
exit "${PIPESTATUS[0]}"
