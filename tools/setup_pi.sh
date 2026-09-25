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

# Comune ambelor distributii. Ce difera - OpenCV - se adauga in
# check_platform(), dupa ce stim numele de cod.
APT_PACKAGES=(
  python3-picamera2      # camera; trage si python3-numpy, python3-simplejpeg
  python3-libcamera      # legaturile libcamera folosite de picamera2
  python3-venv           # nu e intotdeauna instalat
  python3-pip
  git
)

#: Se completeaza in check_platform(): fisierul de requirements si eventualul
#: python3-opencv din apt depind de distributie.
PIP_REQUIREMENTS=""
DISTRO_CODENAME=""

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

  # Doua distributii suportate, cu stive DIFERITE. Diferenta care conteaza e
  # versiunea de OpenCV din apt: pe Trixie e 4.10 si o folosim ca atare, pe
  # Bookworm e 4.6.0, adica inainte de `cv2.aruco.ArucoDetector` (4.7.0), deci
  # acolo OpenCV trebuie luat din pip. Vezi §5.24 din CLAUDE.md.
  DISTRO_CODENAME="$codename"
  case "$codename" in
    trixie)
      PIP_REQUIREMENTS="requirements-pi.txt"
      APT_PACKAGES+=(python3-opencv python3-numpy)
      say "     Trixie: OpenCV si numpy din apt; pip aduce doar pymavlink si PyYAML"
      ;;
    bookworm)
      PIP_REQUIREMENTS="requirements-pi-bookworm.txt"
      say "     Bookworm: python3-opencv din apt e 4.6.0, prea vechi pentru
     API-ul nou de ArUco (ArucoDetector, din 4.7.0). OpenCV vine din pip,
     fixat la 4.10.0.84 - 5.x ar cere numpy>=2 si ar sparge picamera2."
      ;;
    bullseye)
      die "nume de cod 'bullseye'.
       Pe Bullseye stiva picamera2 se instala altfel (pip + libcamera din
       apt), iar python3-opencv e 4.5.x, adica fara ArucoDetector. Nu e
       suportat. Reinstaleaza cu Raspberry Pi OS Bookworm sau Trixie, 64-bit."
      ;;
    *)
      die "nume de cod '$codename', suportate: 'trixie' si 'bookworm'.
       Daca e o distributie mai noua, regula e in requirements-pi.txt: ia
       OpenCV din apt daca e >= 4.7, altfel din pip cu o versiune care nu
       trage numpy peste cel de sistem. Adauga ramura in check_platform() si
       ruleaza suita de teste inainte sa zbori. Daca stii ce faci: --force."
      ;;
  esac
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
    # `import cv2` care merge nu inseamna ca avem ce ne trebuie. Doua
    # lucruri se verifica separat:
    #   - aruco exista (pachetul `opencv-python` simplu NU il are; pe apt,
    #     depinde daca distributia construieste modulele contrib)
    #   - versiunea e >= 4.7, unde a aparut ArucoDetector. Bookworm are 4.6
    #     in apt, si acolo importul reuseste dar clasa lipseste.
    import cv2
    ver = tuple(int(x) for x in cv2.__version__.split('.')[:2])
    if ver < (4, 7):
        raise RuntimeError(
            f"cv2 {cv2.__version__} < 4.7: nu are cv2.aruco.ArucoDetector. "
            f"Pe Bookworm ia OpenCV din pip (requirements-pi-bookworm.txt).")
    cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    return f"OK  cv2 {cv2.__version__}, ArucoDetector construibil"

def _pymavlink():
    from pymavlink import mavutil
    # `mavutil` se importa si FARA pyserial: il cere abia la deschiderea unui
    # port serial. Asa a trecut verificarea asta pe un Pi pe care
    # /dev/serial0 nu se putea deschide deloc. Se verifica explicit.
    import serial
    assert hasattr(serial, 'Serial'), 'pyserial incomplet'
    mavutil.mavlink.MAVLINK_MSG_ID_LANDING_TARGET
    return f"OK  LANDING_TARGET prezent, pyserial {serial.__version__}"

check('import picamera2', _picamera2)
check('import libcamera', _libcamera)
check('cv2.aruco', _aruco)
check('pymavlink', _pymavlink)

# numpy: DE UNDE vine conteaza mai mult decat ce versiune e.
# Pentru cv2, calea e informativa: din apt pe Trixie, din venv pe Bookworm.
import os

def _origine(mod):
    """('VENV'|'sistem', cale) - de unde a fost incarcat modulul."""
    cale = os.path.realpath(getattr(mod, '__file__', '') or '')
    in_venv = cale.startswith(os.path.realpath(sys.prefix) + os.sep)
    return ('VENV' if in_venv else 'sistem'), cale

try:
    import numpy
    unde, cale = _origine(numpy)
    print(f"  {'numpy':<28} {numpy.__version__} din {unde}")
    print(f"  {'':<28} {cale}")
    if unde == 'VENV':
        ok = False
        print("\n  numpy e instalat IN venv, peste cel din apt.")
        print("  picamera2/simplejpeg sunt compilate pentru numpy din sistem")
        print("  si se vor rupe. Cauza uzuala: venv creat fara")
        print("  --system-site-packages, sau un opencv 5.x care cere numpy>=2.")
        print("  Sterge venv-ul si reia tools/setup_pi.sh.")
except Exception as e:                                       # noqa: BLE001
    ok = False
    print(f"  {'numpy':<28} ESEC: {e}")

try:
    import cv2
    unde, cale = _origine(cv2)
    print(f"  {'cv2':<28} {cv2.__version__} din {unde}")
    print(f"  {'':<28} {cale}")
except Exception:                                            # noqa: BLE001
    pass

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

if [[ $FORCE -eq 1 ]]; then
  warn "--force: sar peste verificarea de platforma"
  # Chiar si cu --force trebuie sa stim ce instalam. Presupunem varianta
  # conservatoare (OpenCV din pip): merge si acolo unde apt are deja 4.10,
  # doar ca instaleaza ceva in plus.
  PIP_REQUIREMENTS="${PIP_REQUIREMENTS:-requirements-pi-bookworm.txt}"
else
  check_platform
fi

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

say "4/5  pip: $PIP_REQUIREMENTS (versiuni fixate)"
run "$VENV/bin/pip" install --upgrade pip
run "$VENV/bin/pip" install -r "$REPO/$PIP_REQUIREMENTS"

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

  Pornirea automata la boot se instaleaza separat, fara sudo:
    pi/install.sh

EOF
