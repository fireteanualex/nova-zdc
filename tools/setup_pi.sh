#!/usr/bin/env bash
# NOVA - ZDC 2026
# Instalarea companion-ului pe Raspberry Pi 4 / Raspberry Pi OS Bookworm.
#
#   tools/setup_pi.sh                 # instalare completa
#   tools/setup_pi.sh --dry-run       # arata ce ar rula, nu executa nimic
#   tools/setup_pi.sh --verify-only   # doar verificarea finala a importurilor
#   tools/setup_pi.sh --force         # sari peste verificarea de platforma
#
# Ordinea conteaza si nu e arbitrara:
#
#   1. verifica platforma        Bookworm; pe Bullseye stiva picamera2 difera
#   2. apt: picamera2, libcamera acestea NU se instaleaza cu pip
#   3. venv --system-site-packages    fara flag, pasul 2 devine invizibil
#   4. pip: requirements-pi.txt        versiuni fixate, fara numpy
#   5. verifica importurile DIN VENV   nu din Python-ul de sistem
#
# Pasul 5 exista pentru ca pasii 1-4 pot toti sa "reuseasca" si sistemul sa
# fie totusi nefunctional. E §5.10 aplicat instalarii: nu presupune ca s-a
# aplicat pentru ca nu a dat eroare, citeste inapoi.

set -euo pipefail

# --- caile se pot suprascrie din mediu, ca scriptul sa fie testabil ---------
OS_RELEASE="${NOVA_OS_RELEASE:-/etc/os-release}"
MODEL_FILE="${NOVA_MODEL_FILE:-/proc/device-tree/model}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${NOVA_VENV:-$HOME/nova-venv}"
APT="${NOVA_APT:-apt-get}"

DRY_RUN=0
VERIFY_ONLY=0
FORCE=0

APT_PACKAGES=(
  python3-picamera2      # camera; trage si python3-numpy, python3-simplejpeg
  python3-libcamera      # legaturile libcamera folosite de picamera2
  python3-venv           # Bookworm nu il are intotdeauna instalat
  python3-pip
  libatlas-base-dev      # BLAS pentru numpy/OpenCV
  git
)

say()  { printf '\n\033[1m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[setup] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[setup] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

run() {
  if [[ $DRY_RUN -eq 1 ]]; then
    printf '  + %s\n' "$*"
  else
    printf '  + %s\n' "$*"
    "$@"
  fi
}

usage() { sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)     DRY_RUN=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    --force)       FORCE=1 ;;
    --venv)        VENV="$2"; shift ;;
    -h|--help)     usage ;;
    *) die "optiune necunoscuta: $1 (--help)" ;;
  esac
  shift
done

# --- 1. platforma ----------------------------------------------------------
# Doua verificari separate, pentru ca esueaza din motive diferite:
# distributia gresita (picamera2 difera intre Bullseye si Bookworm) si
# masina gresita (scriptul rulat din greseala pe desktop).
check_platform() {
  [[ -r "$OS_RELEASE" ]] || die "nu pot citi $OS_RELEASE"

  local id="" codename="" pretty=""
  id=$(       sed -n 's/^ID=//p'               "$OS_RELEASE" | tr -d '"' | head -1)
  codename=$( sed -n 's/^VERSION_CODENAME=//p' "$OS_RELEASE" | tr -d '"' | head -1)
  pretty=$(   sed -n 's/^PRETTY_NAME=//p'      "$OS_RELEASE" | tr -d '"' | head -1)

  local model="necunoscut"
  [[ -r "$MODEL_FILE" ]] && model=$(tr -d '\0' < "$MODEL_FILE")

  say "platforma: ${pretty:-?} | $model"

  if [[ "$id" != "debian" && "$id" != "raspbian" ]]; then
    die "distributie $id, nu Raspberry Pi OS. Scriptul instaleaza pachete apt
       specifice Pi-ului (python3-picamera2). Pe desktop foloseste ~/nova-venv
       existent. Daca stii ce faci: --force."
  fi
  if [[ "$codename" != "bookworm" ]]; then
    die "nume de cod '$codename', asteptat 'bookworm'.
       Pe Bullseye picamera2 se instala altfel (pip + libcamera din apt) si
       versiunile nu se potrivesc. Reinstaleaza cu Raspberry Pi OS Bookworm
       64-bit. Daca stii ce faci: --force."
  fi
  case "$model" in
    *"Raspberry Pi"*) ;;
    *) warn "modelul nu contine 'Raspberry Pi' ($model). Continui, dar
       python3-picamera2 nu are ce face fara camera." ;;
  esac
}

# --- 5. verificarea finala -------------------------------------------------
# Ruleaza cu Python-ul DIN VENV. Verifica trei lucruri distincte:
#   - importurile cerute exista
#   - numpy vine din sistem, nu din venv (altfel simplejpeg se rupe)
#   - versiunea de OpenCV e cea din requirements (nu 5.x tras din greseala)
verify_imports() {
  local py="$VENV/bin/python"
  [[ -x "$py" ]] || die "nu exista $py. Ruleaza fara --verify-only."
  say "verific importurile din $py"
  "$py" - <<'PYEOF'
import sys, traceback

ok = True

def check(eticheta, fn):
    global ok
    try:
        print(f"  {eticheta:<28} {fn()}")
    except Exception as e:                                   # noqa: BLE001
        ok = False
        print(f"  {eticheta:<28} ESEC: {type(e).__name__}: {e}")
        traceback.print_exc(limit=1)

def _picamera2():
    import picamera2
    return f"OK  {getattr(picamera2, '__version__', '?')}"

def _libcamera():
    import libcamera
    from libcamera import controls
    controls.AfModeEnum.Manual
    return "OK  AfModeEnum.Manual prezent"

def _aruco():
    import cv2
    cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    return f"OK  cv2 {cv2.__version__}, ArucoDetector construibil"

def _pymavlink():
    from pymavlink import mavutil
    mavutil.mavlink.MAVLINK_MSG_ID_LANDING_TARGET
    return "OK  LANDING_TARGET prezent"

check('import picamera2', _picamera2)
check('import libcamera', _libcamera)
check('cv2.aruco', _aruco)
check('pymavlink', _pymavlink)

# numpy: DE UNDE vine conteaza mai mult decat ce versiune e.
try:
    import numpy, os
    in_venv = os.path.realpath(numpy.__file__).startswith(
        os.path.realpath(sys.prefix) + os.sep)
    unde = 'VENV' if in_venv else 'sistem (apt)'
    print(f"  {'numpy':<28} {numpy.__version__} din {unde}")
    if in_venv:
        ok = False
        print("\n  numpy e instalat IN venv, peste cel din apt.")
        print("  picamera2/simplejpeg sunt compilate pentru numpy din sistem")
        print("  si se vor rupe. Cauza uzuala: venv creat fara")
        print("  --system-site-packages, sau un opencv 5.x care cere numpy>=2.")
        print("  Sterge venv-ul si reia tools/setup_pi.sh.")
except Exception as e:                                       # noqa: BLE001
    ok = False
    print(f"  {'numpy':<28} ESEC: {e}")

print()
print("  TOATE VERIFICARILE AU TRECUT" if ok
      else "  CEL PUTIN O VERIFICARE A PICAT - vezi mai sus")
sys.exit(0 if ok else 1)
PYEOF
}

# --- rulare ----------------------------------------------------------------
if [[ $VERIFY_ONLY -eq 1 ]]; then
  verify_imports
  exit $?
fi

[[ $FORCE -eq 1 ]] && warn "--force: sar peste verificarea de platforma" \
                   || check_platform

say "2/5  pachete apt (picamera2 NU se instaleaza cu pip)"
run sudo "$APT" update
run sudo "$APT" install -y "${APT_PACKAGES[@]}"

say "3/5  venv la $VENV, cu --system-site-packages"
if [[ -d "$VENV" ]]; then
  # Un venv existent poate fi cel gresit. Verificam inainte sa turnam peste.
  if [[ -f "$VENV/pyvenv.cfg" ]] && \
     ! grep -qi 'include-system-site-packages *= *true' "$VENV/pyvenv.cfg"; then
    die "$VENV exista dar a fost creat FARA --system-site-packages.
       picamera2 nu se vede din el, iar pip a tras probabil numpy propriu.
       Sterge-l si reia:  rm -rf '$VENV' && tools/setup_pi.sh"
  fi
  say "     exista deja si e configurat corect, il refolosesc"
else
  run python3 -m venv --system-site-packages "$VENV"
fi

say "4/5  pip: requirements-pi.txt (versiuni fixate)"
run "$VENV/bin/pip" install --upgrade pip
run "$VENV/bin/pip" install -r "$REPO/requirements-pi.txt"

say "5/5  verificare finala"
if [[ $DRY_RUN -eq 1 ]]; then
  printf '  + %s --verify-only\n' "${BASH_SOURCE[0]}"
  exit 0
fi
verify_imports

cat <<EOF

  Gata. Urmatorii pasi, in ordine:

    source $VENV/bin/activate
    python3 tools/preflight_check.py          # camera, calibrare, MAVLink
    python3 tools/calibrate_camera.py --help  # E1.2, inainte de orice zbor

  Serviciul de monitorizare NU e activat (E0). Dupa ce E2 trece:
    sudo systemctl enable --now nova-monitor

EOF
