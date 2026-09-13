#!/usr/bin/env bash
#
# NOVA - ZDC 2026
# Lansare completa a mediului de simulare: Gazebo + ArduPilot SITL + detector.
#
#   ./start_sim.sh                       # implicit
#   ./start_sim.sh --gamepad             # + punte DualSense pentru pilotare
#   ./start_sim.sh --wipe                # sterge eeprom-ul SITL (OBLIGATORIU
#                                        # dupa orice modificare in .parm)
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
GAMEPAD_PORT="14553"
AUDIT_PORT="14554"

# --gamepad e consumat aici; restul argumentelor merg la detector
GAMEPAD=0
WIPE=0
EXTRA_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --gamepad) GAMEPAD=1 ;;
        --wipe)    WIPE=1 ;;
        *)         EXTRA_ARGS+=("$arg") ;;
    esac
done

# --- Verificari ----------------------------------------------------------
die() { echo "EROARE: $*" >&2; exit 1; }

# --- Igiena de mediu (vezi sectiunea 3) ----------------------------------
# Gazebo si SITL TREBUIE sa ruleze cu Python-ul de sistem. Waf alege
# "python" din PATH; nova-venv nu are empy, deci build-ul pica dupa doua
# minute cu "you need to install empy". Daca scriptul e lansat dintr-un
# terminal cu venv-ul activat, gnome-terminal mosteneste mediul si duce
# problema mai departe. Il scoatem explicit, ca regula sa nu mai depinda de
# cine tine minte sa dea "deactivate".
CLEAN_PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VENV}/bin$" \
    | paste -sd: -)"
NOVENV="unset VIRTUAL_ENV PYTHONHOME PYTHONPATH; export PATH='$CLEAN_PATH';"
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    echo "NOTA: venv activ ($VIRTUAL_ENV). Il scot din mediul Gazebo/SITL."
fi

[[ -d "$ARDUPILOT_DIR" ]] || die "nu gasesc $ARDUPILOT_DIR"
[[ -f "$DETECTOR"      ]] || die "nu gasesc $DETECTOR"
[[ -f "$PARAM_FILE"    ]] || die "nu gasesc $PARAM_FILE"
[[ -d "$VENV"          ]] || die "nu gasesc venv-ul $VENV"

if (( GAMEPAD )); then
    [[ -f "$NOVA_DIR/tools/gamepad_rc.py" ]] || die "nu gasesc gamepad_rc.py"
    [[ -f "$NOVA_DIR/config/gamepad.json" ]] || die \
        "lipseste config/gamepad.json. Ruleaza intai:
         python3 tools/gamepad_rc.py --calibrate        (recomandat)
         python3 tools/gamepad_rc.py --preset dualsense (rapid)"
    compgen -G "/dev/input/js*" >/dev/null || die \
        "niciun gamepad in /dev/input/js*. Conecteaza-l si reincearca."
fi
command -v gz >/dev/null   || die "gz nu e in PATH"

# Fara empy, waf construieste doua minute si apoi pica. Verificam intai.
env -u VIRTUAL_ENV -u PYTHONHOME -u PYTHONPATH PATH="$CLEAN_PATH" \
    python3 -c 'import em' 2>/dev/null || die \
    "Python-ul de sistem nu are empy. Ruleaza:
         /usr/bin/python3 -m pip install empy==3.3.4
     (NU in nova-venv: acolo nu foloseste la nimic)"
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
pkill -f gamepad_rc     2>/dev/null || true
sleep 2

# --- Gazebo --------------------------------------------------------------
echo "[2/4] pornesc Gazebo ($WORLD)..."
gnome-terminal --title="NOVA: Gazebo" -- \
    bash -c "$NOVENV gz sim -v4 -r $WORLD; exec bash"

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
SITL_OUT="--out=udp:127.0.0.1:$PORT --out=udp:127.0.0.1:$AUDIT_PORT"
if (( GAMEPAD )); then
    SITL_OUT="$SITL_OUT --out=udp:127.0.0.1:$GAMEPAD_PORT"
fi

# -w sterge eeprom.bin. Fara el, --add-param-file doar seteaza VALORI
# IMPLICITE, care NU suprascriu ce e deja salvat in eeprom - deci o valoare
# schimbata in nova_sitl.parm poate sa nu ajunga niciodata pe FC. Dovedit:
# ARMING_SKIPCHK setat pe 0 si salvat, repornire cu fisierul care cere 32768,
# FC-ul raporteaza tot 0. Vezi 5.13 din CLAUDE.md.
WIPE_ARG=""
if (( WIPE )); then
    WIPE_ARG="-w"
    echo "      -w: sterg eeprom-ul SITL, parametrii se reincarca din fisier"
fi

gnome-terminal --title="NOVA: SITL" -- bash -c "
    $NOVENV
    cd '$ARDUPILOT_DIR' && \
    ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
        --model JSON --console --map \
        --add-param-file='$PARAM_FILE' \
        $WIPE_ARG $SITL_OUT
    exec bash"

echo "      astept convergenta EKF (45 s)..."
sleep 45

# --- Verificarea parametrilor (5.10, 5.13) -------------------------------
# Nu presupune ca fisierul a ajuns pe FC. --add-param-file seteaza doar valori
# implicite; ce e deja in eeprom castiga. Citim inapoi, de fiecare data.
if ! pgrep -x arducopter >/dev/null; then
    echo ""
    echo "  ######################################################"
    echo "  #  SITL NU RULEAZA                                   #"
    echo "  ######################################################"
    echo "  Uita-te in fereastra 'NOVA: SITL'. Cea mai frecventa cauza e"
    echo "  build-ul picat cu 'you need to install empy' - adica waf a luat"
    echo "  Python-ul din venv. Scriptul curata mediul, dar daca ai pornit"
    echo "  sim_vehicle.py manual, da intai 'deactivate'."
    echo ""
    exit 1
fi

echo "      verific parametrii prin citire inapoi..."
if "$VENV/bin/python3" "$NOVA_DIR/tools/check_params.py" \
        --conn "udpin:127.0.0.1:$AUDIT_PORT" > /tmp/nova_param_audit.txt 2>&1; then
    echo "      OK: toti parametrii din nova_sitl.parm sunt aplicati"
else
    echo ""
    echo "  ######################################################"
    echo "  #  PARAMETRI NEAPLICATI - NU ZBURA ASA               #"
    echo "  ######################################################"
    grep -E "LIPSESTE|NU SE POTRIVESTE" /tmp/nova_param_audit.txt | sed 's/^/  /'
    echo ""
    echo "  Cel mai probabil eeprom-ul SITL are valori vechi."
    echo "  Reporneste cu:   ./start_sim.sh --wipe ${EXTRA_ARGS[*]:-}"
    echo "  Raport complet:  /tmp/nova_param_audit.txt"
    echo ""
fi

# --- Detector ------------------------------------------------------------
echo "[4/4] pornesc detectorul sintetic..."
DET_CMD="source '$VENV/bin/activate' && \
    python3 '$DETECTOR' \
        --conn udpin:127.0.0.1:$PORT \
        --north $MARKER_N --east $MARKER_E --conv $CONV \
        ${EXTRA_ARGS[*]:-}"

gnome-terminal --title="NOVA: Detector" -- bash -c "$DET_CMD; exec bash"

# --- Punte gamepad -------------------------------------------------------
if (( GAMEPAD )); then
    echo "[5/5] pornesc puntea de gamepad..."
    GP_CMD="source '$VENV/bin/activate' && \
        python3 '$NOVA_DIR/tools/gamepad_rc.py' \
            --conn udpin:127.0.0.1:$GAMEPAD_PORT"
    gnome-terminal --title="NOVA: Gamepad" -- bash -c "$GP_CMD; exec bash"
fi

# --- Rezumat -------------------------------------------------------------
cat <<EOF

================================================================
  Mediu pornit.

  Marker la N=$MARKER_N  E=$MARKER_E  (distanta $(python3 -c \
      "import math;print(f'{math.hypot($MARKER_N,$MARKER_E):.2f}')") m)
  Conventie axe: --conv $CONV
  Argumente detector: ${EXTRA_ARGS[*]:-(implicite)}
  Gamepad: $( (( GAMEPAD )) && echo "DA, punte pe udp:$GAMEPAD_PORT" || echo "nu (--gamepad)" )

  Intrarea in segmentul autonom se face DOAR prin comutatorul de
  handover (AUX canal 7). Nu exista cale alternativa: o comanda
  "mode land" data cu mana NU porneste secventa.

  Cu gamepad (--gamepad):
      butonul de LOITER, apoi decolare din manete
      du-te deasupra markerului, intre 5 si 12 m
      apasa butonul de HANDOVER (memorie: apesi iar ca sa-l lasi jos)
      poarta asteapta 1.0 s (asezarea manetelor), apoi valideaza
      LASA MANETELE LIBERE in acest interval, altfel primesti REJECT

  Fara gamepad, din MAVProxy:
      mode guided / arm throttle / takeoff 8
      rc 7 1900          # ridica AUX 7 = cerere de handover
      rc 7 1000          # coboara-l (necesar dupa un REJECT)

  Dupa ACCEPT nu interveni: secventa coboara, atinge, tine 1.2 s pe sol
  si urca la 5 m deasupra markerului (15.2.7). Orice miscare de maneta
  peste 80 PWM, tinuta 100 ms, o opreste definitiv (15.3.1).
  Cu --no-ascent revii la comportamentul din Faza 1.

  Verificari inainte de armare:
    - fereastra Detector arata "eroare $(python3 -c \
      "import math;print(f'{math.hypot($MARKER_N,$MARKER_E)*100:.1f}')") cm"
    - Console NU arata AGL la sol: telemetrul exista doar cat markerul e
      in cadru (A2, fara valoare de rezerva)
    - indicatorii EKF si GPS sunt verzi

  Oprire completa:
      pkill -f arducopter; pkill -f "gz sim"; pkill -f fake_detector
      pkill -f gamepad_rc
================================================================

EOF
