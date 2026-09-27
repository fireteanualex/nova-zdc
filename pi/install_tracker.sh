#!/usr/bin/env bash
# NOVA - ZDC 2026
# Instaleaza ~/trackerV2.py ca serviciu de utilizator, pornit la fiecare
# boot IN LOCUL bring-up-ului (nova-bringup): il opreste si ii scoate
# pornirea automata. Ca sa revii la bring-up: pi/install.sh.
#
#   pi/install_tracker.sh              # instaleaza si porneste
#   pi/install_tracker.sh --dry-run    # arata ce ar face
#   pi/install_tracker.sh --uninstall  # scoate serviciul (NU repune bring-up-ul)
#
# NU cere sudo: serviciul e de utilizator, ca nova-bringup (vezi
# pi/nova-bringup.service pentru de ce).

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRACKER="$HOME/trackerV2.py"
VENV="${NOVA_VENV:-$HOME/nova-venv}"
UNIT_SRC="$REPO/pi/nova-tracker.service"
UNIT_DIR="$HOME/.config/systemd/user"
UNIT_DST="$UNIT_DIR/nova-tracker.service"
# Pornirea la login o face sesiunea grafica, printr-o intrare de autostart -
# vezi pi/nova-bringup.desktop pentru de ce nu `systemctl enable`.
AUTO_SRC="$REPO/pi/nova-tracker.desktop"
AUTO_DIR="$HOME/.config/autostart"
AUTO_DST="$AUTO_DIR/nova-tracker.desktop"
# Intrarea de autostart a bring-up-ului: se scoate, altfel la login ar porni
# amandoua si ar castiga cine porneste ultimul. Unitatea lui ramane, ca sa
# poata fi pornita de mana (systemctl --user start nova-bringup).
BRINGUP_AUTO="$AUTO_DIR/nova-bringup.desktop"

DRY_RUN=0
UNINSTALL=0

say()  { printf '\n\033[1m[install_tracker]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[install_tracker] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[install_tracker] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }
run()  { printf '  + %s\n' "$*"; [[ $DRY_RUN -eq 1 ]] || "$@"; }

for arg in "$@"; do
  case "$arg" in
    --dry-run)   DRY_RUN=1 ;;
    --uninstall) UNINSTALL=1 ;;
    -h|--help)   sed -n '2,12p' "$0"; exit 0 ;;
    *) die "optiune necunoscuta: $arg" ;;
  esac
done

command -v systemctl >/dev/null || die "nu exista systemctl"

# NU sub sudo: serviciul e al utilizatorului care are sesiunea grafica
# (`nova`). Sub sudo, $HOME devine /root - unitatea ar cauta
# /root/trackerV2.py, iar `systemctl --user` nu ar gasi busul sesiunii.
# Exact ce s-a intamplat cu pi/install.sh, prima data pe vehicul.
if [[ $EUID -eq 0 ]]; then
  die "nu rula cu sudo. Serviciul e al utilizatorului care are ecranul:
    pi/install_tracker.sh      (ca nova, fara sudo)
  Daca l-ai rulat deja cu sudo, curata ce a lasat:
    sudo rm -f /root/.config/systemd/user/nova-tracker.service"
fi

if [[ $UNINSTALL -eq 1 ]]; then
  say "scot pornirea automata a trackerului"
  run systemctl --user stop nova-tracker.service || true
  run systemctl --user disable nova-tracker.service 2>/dev/null || true
  run rm -f "$AUTO_DST" "$UNIT_DST"
  run systemctl --user daemon-reload
  cat <<'GATA'

  gata. ~/trackerV2.py ramane si se poate rula de mana.
  Bring-up-ul NU e repus automat - daca il vrei inapoi la boot:
      pi/install.sh

GATA
  exit 0
fi

[[ -f "$UNIT_SRC" ]] || die "nu gasesc $UNIT_SRC"
[[ -f "$AUTO_SRC" ]] || die "nu gasesc $AUTO_SRC"
# Scriptul nu e in repo. Fara el, serviciul ar porni si ar muri imediat, de
# 5 ori, si ar parea o problema de unitate.
[[ -f "$TRACKER" ]] || die "nu gasesc $TRACKER - pune scriptul acolo intai"
if [[ ! -x "$VENV/bin/python" ]]; then
  warn "nu gasesc venv-ul la $VENV: trackerul va rula cu python3 de sistem."
  warn "Daca ii lipsesc pachete, ruleaza tools/setup_pi.sh."
fi

# --- 1. bring-up-ul se opreste si nu mai porneste la boot -------------------
# Amandoua tin camera: nu pot rula in acelasi timp (§5.27). Unitatea
# nova-bringup ramane instalata, pentru pornire de mana.
say "opresc bring-up-ul (nova-bringup) si ii scot pornirea automata"
run systemctl --user stop nova-bringup.service || true
if [[ -f "$BRINGUP_AUTO" ]]; then
  run rm -f "$BRINGUP_AUTO"
else
  printf '  (nu avea intrare de autostart)\n'
fi

# --- 2. unitatea -----------------------------------------------------------
say "instalez unitatea in $UNIT_DIR"
run mkdir -p "$UNIT_DIR"
run cp "$UNIT_SRC" "$UNIT_DST"

say "verific unitatea inainte sa o pornesc"
# §5.26: systemd IGNORA TACUT cheile puse in sectiunea gresita. Se filtreaza
# pe numele unitatii noastre - vezi pi/install.sh pentru de ce.
if [[ $DRY_RUN -eq 0 ]]; then
  OBS="$(systemd-analyze verify "$UNIT_DST" 2>&1 \
         | grep -F 'nova-tracker.service' || true)"
  if [[ -n "$OBS" ]]; then
    printf '%s\n' "$OBS"
    warn "unitatea are observatii - citeste-le inainte de a te baza pe ea"
  else
    printf '  unitate valida (nicio observatie pe nova-tracker.service)\n'
  fi
fi

# --- 3. autostart ----------------------------------------------------------
say "instalez intrarea de autostart in $AUTO_DIR"
run mkdir -p "$AUTO_DIR"
run cp "$AUTO_SRC" "$AUTO_DST"

run systemctl --user daemon-reload
run systemctl --user disable nova-tracker.service 2>/dev/null || true

# Pornire ACUM doar daca suntem in sesiunea grafica - altfel (prin SSH) nu
# exista ecran; daca trackerul deschide o fereastra, ar porni fara ea.
if [[ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]]; then
  say "suntem in sesiunea grafica: pornesc acum"
  run systemctl --user import-environment DISPLAY WAYLAND_DISPLAY XAUTHORITY
  run systemctl --user start nova-tracker.service
  PORNIT=1
else
  PORNIT=0
fi

if [[ $PORNIT -eq 0 && $DRY_RUN -eq 0 ]]; then
  cat <<'SSH'

  NU l-am pornit acum: rulezi prin SSH, fara ecran. Porneste la urmatorul
  login pe desktop - adica la reboot, daca ai autologin:
      sudo reboot
  Sau, daca trackerul nu are nevoie de fereastra, porneste-l de mana:
      systemctl --user start nova-tracker
SSH
fi

cat <<FIN

  ---------------------------------------------------------------
  Instalat. La fiecare login pe desktop porneste ~/trackerV2.py,
  NU bring-up-ul (nova-bringup e oprit si scos din autostart).

  Comenzi:
      systemctl --user status nova-tracker
      systemctl --user restart nova-tracker
      systemctl --user stop nova-tracker
      journalctl --user-unit nova-tracker -f

  Inapoi la bring-up, la boot:
      pi/install.sh
  Bring-up de mana, o singura data (opreste trackerul, prin Conflicts=):
      systemctl --user start nova-bringup
  ---------------------------------------------------------------

FIN
