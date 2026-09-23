#!/usr/bin/env bash
# NOVA - ZDC 2026
# Instaleaza bring-up-ul ca serviciu de utilizator, pornit la fiecare boot.
#
#   pi/install.sh              # instaleaza si porneste
#   pi/install.sh --dry-run    # arata ce ar face
#   pi/install.sh --uninstall  # scoate serviciul
#
# NU cere sudo: serviciul e de utilizator (vezi pi/nova-bringup.service
# pentru de ce). Singurul pas cu sudo e pi/setup_uart.sh, separat.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_SRC="$REPO/pi/nova-bringup.service"
UNIT_DIR="$HOME/.config/systemd/user"
UNIT_DST="$UNIT_DIR/nova-bringup.service"

DRY_RUN=0
UNINSTALL=0

say()  { printf '\n\033[1m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[install] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[install] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }
run()  { printf '  + %s\n' "$*"; [[ $DRY_RUN -eq 1 ]] || "$@"; }

for arg in "$@"; do
  case "$arg" in
    --dry-run)   DRY_RUN=1 ;;
    --uninstall) UNINSTALL=1 ;;
    -h|--help)   sed -n '2,10p' "$0"; exit 0 ;;
    *) die "optiune necunoscuta: $arg" ;;
  esac
done

command -v systemctl >/dev/null || die "nu exista systemctl"

# NU sub sudo. Serviciul e DE UTILIZATOR, al celui care are sesiunea grafica
# (`nova`). Sub sudo, $HOME devine /root: unitatea ajunge in configul lui
# root, cu ExecStart spre /root/nova-zdc - care nu exista - iar
# `systemctl --user` nu gaseste busul sesiunii ("Failed to connect to user
# scope bus"). Asa s-a si intamplat, prima data, pe vehicul.
if [[ $EUID -eq 0 ]]; then
  die "nu rula cu sudo. Serviciul e al utilizatorului care are ecranul:
    pi/install.sh              (ca nova, fara sudo)
  Daca l-ai rulat deja cu sudo, curata ce a lasat:
    sudo rm -f /root/.config/systemd/user/nova-bringup.service"
fi

if [[ $UNINSTALL -eq 1 ]]; then
  say "scot serviciul"
  run systemctl --user disable --now nova-bringup.service || true
  run rm -f "$UNIT_DST"
  run systemctl --user daemon-reload
  printf '\n  gata. pi/bringup.sh ramane si se poate rula de mana.\n\n'
  exit 0
fi

[[ -f "$UNIT_SRC" ]] || die "nu gasesc $UNIT_SRC"
[[ -x "$REPO/pi/bringup.sh" ]] || die "pi/bringup.sh nu e executabil (chmod +x)"

say "instalez unitatea in $UNIT_DIR"
run mkdir -p "$UNIT_DIR"
# %h din unitate se extinde la $HOME, deci unitatea e portabila intre
# utilizatori - dar numai daca repo-ul e la ~/nova-zdc. Verificam, in loc sa
# presupunem: o cale gresita da un serviciu care porneste si moare imediat.
if [[ "$REPO" != "$HOME/nova-zdc" ]]; then
  warn "repo-ul e la $REPO, iar unitatea presupune \$HOME/nova-zdc."
  warn "Editeaza WorkingDirectory si ExecStart in $UNIT_DST dupa instalare."
fi
run cp "$UNIT_SRC" "$UNIT_DST"

say "verific unitatea inainte sa o pornesc"
# §5.26: systemd IGNORA TACUT cheile puse in sectiunea gresita. Iesire goala
# = unitate valida; orice "Unknown key name" e o cheie care nu face nimic.
if [[ $DRY_RUN -eq 0 ]]; then
  # Se filtreaza pe NUMELE unitatii noastre: `systemd-analyze verify` se
  # plange si de unitati de sistem fara legatura (snapd, netplan), iar un
  # `grep -q .` ar da avertisment mereu - adica un avertisment pe care
  # nimeni nu-l mai citeste dupa a doua oara.
  OBS="$(systemd-analyze verify "$UNIT_DST" 2>&1 \
         | grep -F 'nova-bringup.service' || true)"
  if [[ -n "$OBS" ]]; then
    printf '%s\n' "$OBS"
    warn "unitatea are observatii - citeste-le inainte de a te baza pe ea"
  else
    printf '  unitate valida (nicio observatie pe nova-bringup.service)\n'
  fi
fi

run systemctl --user daemon-reload
run systemctl --user enable --now nova-bringup.service

cat <<FIN

  ---------------------------------------------------------------
  Instalat. Porneste la fiecare boot, IN SESIUNEA GRAFICA.

  Cere autologin pe desktop, altfel nu porneste pana nu te loghezi:
      sudo raspi-config
      -> System Options -> Boot / Auto Login -> Desktop Autologin

  Comenzi:
      systemctl --user status nova-bringup
      systemctl --user restart nova-bringup
      systemctl --user stop nova-bringup
      journalctl --user-unit nova-bringup -f

  Loguri de rulare (si dupa repornire):
      ls -t ~/nova-logs/ | head
  ---------------------------------------------------------------

FIN
