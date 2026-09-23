#!/usr/bin/env bash
# NOVA - ZDC 2026
# Trimite codul pe Raspberry Pi. Se ruleaza DE PE DESKTOP, nu de pe Pi.
#
#   pi/deploy.sh 10.41.0.193              # trimite
#   pi/deploy.sh 10.41.0.193 --setup      # trimite + instaleaza stiva Python
#   pi/deploy.sh 10.41.0.193 --check      # trimite + verifica, nu porneste
#   pi/deploy.sh --dry-run 10.41.0.193    # arata ce ar trimite
#
# Gazda se poate da si din mediu:  NOVA_PI=pi@10.41.0.193 pi/deploy.sh
#
# ===========================================================================
# CE SE TRIMITE SI CE NU
# ===========================================================================
#
#   SE TRIMITE      codul, config/, pi/, tools/, docs/ si **.git/**
#   NU SE TRIMITE   data/ (cadre E2, sesiuni, campanii - sute de MB)
#                   __pycache__, *.pyc
#                   venv-ul: se construieste PE Pi, cu --system-site-packages
#                   (§5.24). Copiat de pe desktop ar fi legat de alt Python
#                   si de alt numpy, si s-ar rupe la primul import picamera2.
#
# `.git/` se trimite deliberat. Checklistul de teren cere `git status` curat
# pe Pi INAINTE de zbor: fara el nu se poate spune ce cod a zburat, iar
# 6.2.1.30 se sprijina exact pe asta. Costa cativa MB.
#
# ATENTIE la calibrare: daca ai recalibrat PE Pi, `config/camera_pi.yaml` de
# acolo e mai nou decat cel de pe desktop, iar sincronizarea il suprascrie.
# Adu-l intai:  scp <gazda>:~/nova-zdc/config/camera_pi.yaml config/

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${NOVA_PI_DIR:-nova-zdc}"
HOST="${NOVA_PI:-}"
DRY=0
SETUP=0
CHECK=0

say()  { printf '\n\033[1m[deploy]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[deploy] ATENTIE:\033[0m %s\n' "$*" >&2; }
die()  { printf '\n\033[31m[deploy] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY=1 ;;
    --setup)   SETUP=1 ;;
    --check)   CHECK=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    -*)        die "optiune necunoscuta: $arg" ;;
    *)         HOST="$arg" ;;
  esac
done

[[ -n "$HOST" ]] || die "da gazda: pi/deploy.sh <user@ip>  (sau NOVA_PI=...)"
# fara user explicit: utilizatorul de pe Pi-ul echipei (/home/nova)
[[ "$HOST" == *@* ]] || HOST="nova@$HOST"

command -v rsync >/dev/null || die "rsync lipseste pe masina asta"

say "gazda: $HOST  ->  ~/$DEST"

# Arata ce cod pleaca. Daca arborele e murdar, o spune ACUM, nu la pista.
if git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
  printf '  commit: %s\n' "$(git -C "$REPO" log --oneline -1)"
  if [[ -n "$(git -C "$REPO" status --porcelain)" ]]; then
    warn "arborele git e MURDAR - pe Pi ajunge cod necommis."
    git -C "$REPO" status --short | sed 's/^/        /'
  fi
fi

RSYNC=(rsync -az --delete --human-readable
       --exclude 'data/'
       --exclude '__pycache__/'
       --exclude '*.pyc'
       --exclude '.venv/'
       --exclude 'nova-venv/')
[[ $DRY -eq 1 ]] && RSYNC+=(--dry-run --itemize-changes)

say "sincronizez"
"${RSYNC[@]}" "$REPO/" "$HOST:~/$DEST/"

if [[ $DRY -eq 1 ]]; then
  printf '\n  --dry-run: nimic nu s-a schimbat pe Pi.\n\n'
  exit 0
fi

# rsync pastreaza bitul de executie, dar numai daca l-a avut si sursa.
# Verificam, in loc sa presupunem: un script fara +x da "Permission denied"
# la pista, si arata ca o problema de cale.
say "verific drepturile de executie"
ssh "$HOST" "cd ~/$DEST && for f in pi/*.sh tools/*.sh; do \
  [ -x \"\$f\" ] || { echo \"  chmod +x \$f\"; chmod +x \"\$f\"; }; done; echo '  ok'"

if [[ $SETUP -eq 1 ]]; then
  say "instalez stiva Python pe Pi (dureaza cateva minute)"
  ssh -t "$HOST" "cd ~/$DEST && tools/setup_pi.sh"
fi

if [[ $CHECK -eq 1 ]]; then
  say "verific pe Pi"
  ssh -t "$HOST" "cd ~/$DEST && pi/bringup.sh --check"
fi

cat <<FIN

  ---------------------------------------------------------------
  Trimis. Mai departe, pe Pi:

      ssh $HOST
      cd ~/$DEST

      tools/setup_pi.sh              # prima data
      sudo pi/setup_uart.sh && sudo reboot
      pi/bringup.sh --check
      pi/install.sh                  # pornire automata la boot
  ---------------------------------------------------------------

FIN
