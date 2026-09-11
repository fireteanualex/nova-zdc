#!/usr/bin/env bash
#
# NOVA - ZDC 2026
# Lansare completa a mediului de simulare: Gazebo + ArduPilot SITL + detector.
#
#   ./start_sim.sh                       # implicit
#   ./start_sim.sh --noise-px 1.5        # orice argument merge la detector
#   ./start_sim.sh --latency-ms 120
#   ./start_sim.sh --dropout 0.15
#   ./start_sim.sh --dropout 1.0         # test 15.2.7
#
# Dupa pornire, in fereastra MAVProxy:
#   mode guided / arm throttle / takeoff 8 / (asteapta) / mode land

set -euo pipefail

# --- Cai (ajusteaza daca repo-ul e in alta parte) ------------------------
ARDUPILOT_DIR="$HOME/ardupilot"
NOVA_DIR="$HOME/nova-zdc"
VENV="$HOME/nova-venv"
PARAM_FILE="$NOVA_DIR/config/nova_sitl.parm"
DETECTOR="$NOVA_DIR/tools/fake_detector.py"
WORLD="iris_runway.sdf"

# --- Pozitia markerului fata de originea EKF, metri ---------------------
MARKER_N="2.0"
MARKER_E="1.5"
CONV="2"          # conventie axe LANDING_TARGET, confirmata prin test
PORT="14552"

# Orice argument dat scriptului e pasat mai departe detectorului
EXTRA_ARGS=("$@")

# --- Verificari ----------------------------------------------------------
die() { echo "EROARE: $*" >&2; exit 1; }

[[ -d "$ARDUPILOT_DIR" ]] || die "nu gasesc $ARDUPILOT_DIR"
[[ -f "$DETECTOR"      ]] || die "nu gasesc $DETECTOR"
[[ -f "$PARAM_FILE"    ]] || die "nu gasesc $PARAM_FILE"
[[ -d "$VENV"          ]] || die "nu gasesc venv-ul $VENV"
command -v gz >/dev/null   || die "gz nu e in PATH"
command -v gnome-terminal >/dev/null || die "gnome-terminal lipseste"

if [[ -z "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" ]]; then
    die "GZ_SIM_SYSTEM_PLUGIN_PATH nesetat. Ruleaza: source ~/.bashrc"
fi

# --- Curatenie -----------------------------------------------------------
echo "[1/4] opresc procesele ramase..."
pkill -f arducopter    2>/dev/null || true
pkill -f "gz sim"      2>/dev/null || true
pkill -f mavproxy      2>/dev/null || true
pkill -f fake_detector 2>/dev/null || true
sleep 2

# --- Gazebo --------------------------------------------------------------
echo "[2/4] pornesc Gazebo ($WORLD)..."
gnome-terminal --title="NOVA: Gazebo" -- \
    bash -c "gz sim -v4 -r $WORLD; exec bash"

# Asteapta ca plugin-ul sa deschida portul FDM
echo "      astept ca Gazebo sa fie gata..."
for i in {1..30}; do
    sleep 1
    if pgrep -f "gz sim" >/dev/null; then
        [[ $i -ge 8 ]] && break
    fi
done
sleep 2

# --- ArduPilot SITL ------------------------------------------------------
echo "[3/4] pornesc ArduPilot SITL..."
gnome-terminal --title="NOVA: SITL" -- bash -c "
    cd '$ARDUPILOT_DIR' && \
    ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
        --model JSON --console --map \
        --add-param-file='$PARAM_FILE' \
        --out=udp:127.0.0.1:$PORT
    exec bash"

echo "      astept convergenta EKF (45 s)..."
sleep 45

# --- Detector ------------------------------------------------------------
echo "[4/4] pornesc detectorul sintetic..."
DET_CMD="source '$VENV/bin/activate' && \
    python3 '$DETECTOR' \
        --conn udpin:127.0.0.1:$PORT \
        --north $MARKER_N --east $MARKER_E --conv $CONV \
        ${EXTRA_ARGS[*]:-}"

gnome-terminal --title="NOVA: Detector" -- bash -c "$DET_CMD; exec bash"

# --- Rezumat -------------------------------------------------------------
cat <<EOF

================================================================
  Mediu pornit.

  Marker la N=$MARKER_N  E=$MARKER_E  (distanta $(python3 -c \
      "import math;print(f'{math.hypot($MARKER_N,$MARKER_E):.2f}')") m)
  Conventie axe: --conv $CONV
  Argumente detector: ${EXTRA_ARGS[*]:-(implicite)}

  In fereastra MAVProxy:
      mode guided
      arm throttle
      takeoff 8
      (asteapta stabilizarea la ~8 m)
      mode land

  Verificari inainte de armare:
    - fereastra Detector arata "eroare $(python3 -c \
      "import math;print(f'{math.hypot($MARKER_N,$MARKER_E)*100:.1f}')") cm"
    - Console arata AGL 5 m la sol (telemetrul trimite valoarea de rezerva)
    - indicatorii EKF si GPS sunt verzi

  Oprire completa:
      pkill -f arducopter; pkill -f "gz sim"; pkill -f fake_detector
================================================================

EOF
