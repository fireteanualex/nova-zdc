#!/usr/bin/env bash
# NOVA - ZDC 2026
# Mediul Python pentru bucla inchisa in Gazebo (I3).
#
#   tools/setup_sim_venv.sh              # creeaza ~/nova-sim-venv
#   tools/setup_sim_venv.sh --verify     # doar verificarea importurilor
#
# DE CE UN AL TREILEA MEDIU
#
# Sursa de cadre are nevoie de `gz.transport13` + `gz.msgs10`, care vin din
# APT si exista DOAR in python3 de sistem. Detectorul are nevoie de OpenCV
# >= 4.7, pentru `cv2.aruco.ArucoDetector`. Pe Ubuntu 22.04 cele doua nu se
# intalnesc nicaieri:
#
#   python3 de sistem   gz OK        cv2 4.5.4  -> fara ArucoDetector
#   ~/nova-venv         gz LIPSA     cv2 5.0.0
#
# `PYTHONPATH=/usr/lib/python3/dist-packages` NU rezolva: acea cale ajunge
# INAINTEA lui site-packages din venv, deci cv2 de sistem (4.5.4) il umbreste
# pe cel bun. Masurat.
#
# Solutia e un venv cu --system-site-packages, in care instalam OpenCV
# **4.10**, nu 5.x:
#
#   - 4.10 cere numpy>=1.21, deci se multumeste cu numpy 1.21.5 din apt si nu
#     instaleaza unul propriu peste el (aceeasi regula ca §5.24 pe Pi)
#   - 5.x ar cere numpy>=2, l-ar instala in venv peste cel de sistem, iar
#     legaturile gz - compilate impotriva celui de sistem - s-ar rupe
#   - bonus: 4.10 e EXACT versiunea de pe vehicul, deci simularea ruleaza
#     acelasi detector ca zborul. Diferentele dintre 4.10 si 5.0 sunt reale
#     (§5.24), deci asta e fidelitate castigata, nu compromis.
#
# ~/nova-venv ramane neatins: acolo traieste restul uneltelor de desktop.

set -euo pipefail

VENV="${NOVA_SIM_VENV:-$HOME/nova-sim-venv}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENCV="opencv-contrib-python==4.10.0.84"

VERIFY_ONLY=0
[[ "${1:-}" == "--verify" ]] && VERIFY_ONLY=1

say()  { printf '\n\033[1m[sim-venv]\033[0m %s\n' "$*"; }
die()  { printf '\n\033[31m[sim-venv] OPRIT:\033[0m %s\n\n' "$*" >&2; exit 1; }

verify() {
  local py="$VENV/bin/python"
  [[ -x "$py" ]] || die "nu exista $py. Ruleaza fara --verify."
  say "verific $py"
  "$py" - <<'PYEOF'
import os, sys
ok = True

def unde(mod):
    cale = os.path.realpath(getattr(mod, '__file__', '') or '')
    in_venv = cale.startswith(os.path.realpath(sys.prefix) + os.sep)
    return ('VENV' if in_venv else 'sistem'), cale

def check(eticheta, fn):
    global ok
    try:
        print(f"  {eticheta:<26} {fn()}")
    except Exception as e:                                   # noqa: BLE001
        ok = False
        print(f"  {eticheta:<26} ESEC: {type(e).__name__}: {e}")

def _cv2():
    import cv2
    v = tuple(int(x) for x in cv2.__version__.split('.')[:2])
    if v < (4, 7):
        raise RuntimeError(f"cv2 {cv2.__version__} < 4.7: fara ArucoDetector")
    cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    d, c = unde(cv2)
    return f"OK  {cv2.__version__} din {d}"

def _gz():
    from gz.transport13 import Node            # noqa: F401
    from gz.msgs10.image_pb2 import Image      # noqa: F401
    import gz.transport13 as t
    d, c = unde(t)
    return f"OK  transport13 + msgs10 din {d}"

def _numpy():
    import numpy
    d, c = unde(numpy)
    if d == 'VENV':
        raise RuntimeError(
            f"numpy {numpy.__version__} e in VENV, peste cel de sistem: "
            f"legaturile gz sunt compilate impotriva celui din apt si se vor "
            f"rupe. Sterge venv-ul si reia.")
    return f"OK  {numpy.__version__} din {d}"

check('cv2.aruco', _cv2)
check('gz-transport', _gz)
check('numpy', _numpy)
print()
print("  TOATE VERIFICARILE AU TRECUT" if ok
      else "  CEL PUTIN O VERIFICARE A PICAT")
sys.exit(0 if ok else 1)
PYEOF
}

if [[ $VERIFY_ONLY -eq 1 ]]; then
  verify
  exit $?
fi

say "1/3  venv la $VENV, cu --system-site-packages"
if [[ -d "$VENV" ]]; then
  if [[ -f "$VENV/pyvenv.cfg" ]] && \
     ! grep -qi 'include-system-site-packages *= *true' "$VENV/pyvenv.cfg"; then
    die "$VENV exista dar a fost creat FARA --system-site-packages.
       Fara el nu vede gz-transport din apt.
       Sterge-l si reia:  rm -rf '$VENV' && tools/setup_sim_venv.sh"
  fi
  echo "     exista deja si e configurat corect"
else
  python3 -m venv --system-site-packages "$VENV"
fi

say "2/3  OpenCV $OPENCV (NU 5.x: ar trage numpy 2 peste cel de sistem)"
"$VENV/bin/pip" install --quiet --upgrade pip
"$VENV/bin/pip" install --quiet "$OPENCV" pymavlink==2.4.49

say "3/3  verificare"
verify

cat <<EOF

  Gata. Foloseste-l pentru tot ce atinge Gazebo:

    $VENV/bin/python tools/nova_sim.py        # I4
    $VENV/bin/python tools/gz_frames.py --topic /down_cam/image

  Restul uneltelor raman pe ~/nova-venv.

EOF
