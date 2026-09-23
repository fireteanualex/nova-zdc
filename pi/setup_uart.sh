#!/usr/bin/env bash
# NOVA - ZDC 2026
# UART-ul de pe GPIO 14/15, pentru legatura cu Pixhawk 6C / TELEM2.
#
#   sudo pi/setup_uart.sh --check     # ce e acum, fara sa schimbe nimic
#   sudo pi/setup_uart.sh             # configureaza (idempotent)
#   sudo pi/setup_uart.sh --dry-run   # arata ce ar schimba
#
# Dupa configurare trebuie REPORNIT Pi-ul. Nimic din ce e mai jos nu are
# efect pana la reboot.
#
# ===========================================================================
# DE CE E NEVOIE DE SCRIPTUL ASTA, SI DE CE E PRIMUL PAS
# ===========================================================================
#
# Pe Raspberry Pi 4 exista DOUA UART-uri pe pinii GPIO 14/15:
#
#   PL011  (ttyAMA0)  UART adevarat, ceas propriu, stabil la orice baud
#   miniUART (ttyS0)  ceas legat de FRECVENTA MIEZULUI VPU
#
# Implicit, PL011 e luat de Bluetooth, iar `/dev/serial0` arata spre
# **miniUART**. Iar miniUART-ul isi ia tactul din ceasul miezului, care pe
# Pi 4 se scaleaza cu incarcarea si cu temperatura. La 921600 baud asta
# inseamna o legatura care merge cateva minute si apoi incepe sa dea
# caractere gresite - iar simptomul arata ca un cablu prost sau ca un
# Pixhawk defect, nu ca o problema de configurare.
#
# Deci: `dtoverlay=disable-bt` muta PL011 inapoi pe GPIO 14/15, si de acolo
# `/dev/serial0` devine `ttyAMA0`. Asta e ce verifica --check.
#
# Al doilea lucru, independent: Linux pune implicit o CONSOLA SERIALA pe
# acelasi port. Doua programe pe acelasi UART inseamna ca fiecare inghite
# din mesajele celuilalt; pymavlink vede pachete MAVLink taiate si nu se
# plange, doar nu ajunge niciodata la HEARTBEAT.
#
# Nu e acelasi lucru cu §5.27 (`nova-monitor` care tine /dev/serial0). Acolo
# portul e OCUPAT si deschiderea da `Errno 16`, adica un mesaj. Aici portul
# se deschide perfect si datele sunt doar corupte - genul de esec care nu
# lasa nicio urma.

set -euo pipefail

BOOT_DIR="${NOVA_BOOT_DIR:-}"
DRY_RUN=0
CHECK_ONLY=0

say()  { printf '\n\033[1m[uart]\033[0m %s\n' "$*"; }
ok()   { printf '  \033[32mOK\033[0m    %s\n' "$*"; }
bad()  { printf '  \033[31mLIPSA\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[uart] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[uart] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --check)   CHECK_ONLY=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) die "optiune necunoscuta: $arg" ;;
  esac
done

# Bookworm si Trixie tin fisierele de boot in /boot/firmware; pe versiuni
# mai vechi sunt in /boot. Se cauta, nu se presupune: scris in locul gresit,
# fisierul se editeaza "cu succes" si nu are niciun efect.
if [[ -z "$BOOT_DIR" ]]; then
  for d in /boot/firmware /boot; do
    [[ -f "$d/config.txt" ]] && { BOOT_DIR="$d"; break; }
  done
fi
[[ -n "$BOOT_DIR" && -f "$BOOT_DIR/config.txt" ]] || \
  die "nu gasesc config.txt (cautat in /boot/firmware si /boot). Esti pe un Pi?"

CONFIG="$BOOT_DIR/config.txt"
CMDLINE="$BOOT_DIR/cmdline.txt"

# --- ce e acum -------------------------------------------------------------
raport() {
  say "starea de acum ($BOOT_DIR)"

  grep -qE '^\s*enable_uart=1' "$CONFIG" \
    && ok "enable_uart=1" || bad "enable_uart=1 lipseste din config.txt"

  grep -qE '^\s*dtoverlay=disable-bt' "$CONFIG" \
    && ok "dtoverlay=disable-bt" \
    || bad "dtoverlay=disable-bt lipseste -> serial0 ramane pe miniUART"

  if [[ -f "$CMDLINE" ]] && grep -q 'console=serial0' "$CMDLINE"; then
    bad "consola seriala e pornita in cmdline.txt -> doua programe pe acelasi port"
  else
    ok "fara consola seriala pe serial0"
  fi

  if [[ -e /dev/serial0 ]]; then
    local tinta; tinta="$(readlink -f /dev/serial0)"
    case "$tinta" in
      *ttyAMA*) ok "/dev/serial0 -> $tinta (PL011, ceas propriu)" ;;
      *ttyS0)   bad "/dev/serial0 -> $tinta (miniUART: instabil la 921600)" ;;
      *)        warn "/dev/serial0 -> $tinta (neasteptat)" ;;
    esac
  else
    bad "/dev/serial0 nu exista"
  fi

  if id -nG "${SUDO_USER:-$USER}" 2>/dev/null | grep -qw dialout; then
    ok "utilizatorul ${SUDO_USER:-$USER} e in grupul dialout"
  else
    bad "utilizatorul ${SUDO_USER:-$USER} NU e in dialout -> deschiderea portului da Permission denied"
  fi

  if systemctl is-enabled hciuart >/dev/null 2>&1; then
    bad "hciuart e activ (Bluetooth pe UART) -> dezactiveaza-l"
  else
    ok "hciuart dezactivat"
  fi
}

raport
[[ $CHECK_ONLY -eq 1 ]] && exit 0

[[ $EUID -eq 0 ]] || die "trebuie rulat cu sudo (scrie in $BOOT_DIR)"

# --- modificarile ----------------------------------------------------------
adauga_linie() {   # adauga_linie <fisier> <linie>
  local f="$1" linie="$2"
  if grep -qF -- "$linie" "$f"; then
    printf '  = %s (deja)\n' "$linie"; return
  fi
  printf '  + %s -> %s\n' "$linie" "$f"
  [[ $DRY_RUN -eq 1 ]] && return
  printf '\n# NOVA ZDC: legatura cu Pixhawk pe GPIO 14/15\n%s\n' "$linie" >> "$f"
}

say "modific $CONFIG"
adauga_linie "$CONFIG" "enable_uart=1"
adauga_linie "$CONFIG" "dtoverlay=disable-bt"

if [[ -f "$CMDLINE" ]] && grep -q 'console=serial0[^ ]*' "$CMDLINE"; then
  say "scot consola seriala din $CMDLINE"
  printf '  + copie de siguranta: %s.nova-backup\n' "$CMDLINE"
  if [[ $DRY_RUN -eq 0 ]]; then
    cp -n "$CMDLINE" "$CMDLINE.nova-backup"
    sed -i 's/console=serial0,[0-9]*[[:space:]]*//g' "$CMDLINE"
  fi
  printf '  + %s\n' "$(cat "$CMDLINE" 2>/dev/null || echo '(dry-run)')"
fi

say "dezactivez hciuart (Bluetooth pe UART)"
if [[ $DRY_RUN -eq 0 ]]; then
  systemctl disable hciuart >/dev/null 2>&1 || true
fi

UTIL="${SUDO_USER:-$USER}"
if ! id -nG "$UTIL" | grep -qw dialout; then
  say "adaug $UTIL in grupul dialout"
  [[ $DRY_RUN -eq 0 ]] && usermod -aG dialout "$UTIL"
fi

cat <<'FIN'

  ---------------------------------------------------------------
  REPORNESTE Pi-ul, apoi verifica:

      sudo reboot
      # dupa ce revine:
      pi/setup_uart.sh --check

  Trebuie sa vezi /dev/serial0 -> ttyAMA0. Daca arata inca spre
  ttyS0, dtoverlay-ul nu s-a aplicat: verifica in ce config.txt
  ai scris (pe Bookworm/Trixie e /boot/firmware/config.txt).
  ---------------------------------------------------------------
FIN
