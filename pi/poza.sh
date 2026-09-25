#!/usr/bin/env bash
# NOVA - ZDC 2026
# O poza de verificare de pe camera dronei, adusa si deschisa pe laptop.
#
#   pi/poza.sh nova@100.97.236.82        # sau NOVA_PI=... pi/poza.sh
#
# Cum se citeste: pui markerul pe jos, IN FATA NASULUI dronei, la ~1 m.
# Poza se deschide ROTITA exact cum o vede detectorul (config:
# camera_rotation_deg), cu sageata "NASUL DRONEI ^" desenata sus. Daca
# markerul NU apare sus, rotatia din config e gresita - asa s-a gasit pe
# 25.09.2026 ca valoarea corecta e 270, nu 90 (axe inversate cu 180:
# corectiile de aterizare ar fi impins drona exact invers).
#
# Camera e exclusiva (un singur proces), deci pornirea automata se opreste
# pe durata capturii si se reporneste la final.
set -euo pipefail
HOST="${1:-${NOVA_PI:-}}"
[[ -n "$HOST" ]] || { echo "folosire: pi/poza.sh <user@ip-pi>" >&2; exit 1; }
[[ "$HOST" == *@* ]] || HOST="nova@$HOST"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/nova-poza"
mkdir -p "$OUT"

echo "[poza] captura pe $HOST (pornirea automata se opreste cateva secunde)"
ssh "$HOST" 'systemctl --user stop nova-bringup 2>/dev/null || true
rpicam-still --width 2304 --height 1296 -n -t 1500 -o /tmp/nova-poza.jpg
systemctl --user start nova-bringup 2>/dev/null || true'
scp -q "$HOST:/tmp/nova-poza.jpg" "$OUT/bruta.jpg"

python3 - "$REPO" "$OUT" <<'PY'
import json, os, sys
repo, out = sys.argv[1], sys.argv[2]
rot = 0
try:
    cfg = json.load(open(os.path.join(repo, 'config', 'nova.json')))
    rot = int(cfg.get('camera_rotation_deg', 0)) % 360
except Exception as e:                                        # noqa: BLE001
    print(f"[poza] nu pot citi rotatia din config ({e}); folosesc 0")
try:
    import cv2
    img = cv2.imread(os.path.join(out, 'bruta.jpg'))
    k = {0: None, 90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180,
         270: cv2.ROTATE_90_CLOCKWISE}[rot]
    vaz = img if k is None else cv2.rotate(img, k)
    cv2.putText(vaz, f'NASUL DRONEI ^  (rotatie {rot})', (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
    cv2.imwrite(os.path.join(out, 'ca-in-detector.jpg'), vaz)
    print(f"[poza] rotita {rot} spre stanga, ca in detector")
except Exception as e:                                        # noqa: BLE001
    print(f"[poza] fara rotire ({e}); ramane doar poza bruta")
PY
echo "[poza] fisiere: $OUT/ca-in-detector.jpg si $OUT/bruta.jpg"
xdg-open "$OUT/ca-in-detector.jpg" 2>/dev/null \
  || xdg-open "$OUT/bruta.jpg" 2>/dev/null || true
