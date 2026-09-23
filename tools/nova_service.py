#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Serviciul de bord in RACE_MONITOR (G2).

    python3 tools/nova_service.py                  # rulare in prim-plan
    python3 tools/nova_service.py --install-unit   # scrie unitatea systemd
    python3 tools/nova_service.py --check          # doar verificarile, iese

RACE_MONITOR, din §8: **detector activ, ZERO comenzi, ring buffer.** Serviciul
porneste la boot, detecteaza markerul, tine ultimele cadre intr-un buffer
circular si scrie un log persistent. Nu comanda nimic si nu poate comanda
nimic.

**"Nu poate", nu "nu o face".** Un serviciu care porneste la boot si are
acces la obiectul Vehicle e la o singura linie gresita distanta de a comanda
un mod de zbor. De aceea legatura MAVLink e impachetata in `ReadOnlyVehicle`,
care ridica `PermissionError` la orice metoda de comanda. Nu e o conventie,
e o bariera: un apel gresit pica zgomotos la prima rulare, nu tacut in zbor.

Din acelasi motiv serviciul **nu instantiaza** masina de stari, poarta de
handover sau Safety Supervisor. Cu `autonomy_enabled=false` poarta ar refuza
oricum orice cerere (E0, §5.16), dar atunci singurul lucru care ar sta intre
un serviciu pornit la boot si o coborare autonoma ar fi o valoare dintr-un
fisier JSON. Doua bariere independente sunt mai bune decat una, iar
RACE_MONITOR chiar nu are nevoie de ele.
# DECIZIE DESCHISA: daca vrei ca serviciul sa poata prelua si segmentul
# autonom dupa E2, calea nu e sa scoatem ReadOnlyVehicle, ci sa adaugam un
# al doilea mod explicit (`--mode race`) care construieste cablajul din
# tools/nova_pi.py. Asa ramane evident din linia de comanda ce ruleaza.

Refuza sa porneasca fara calibrare reala (E1.2), si verifica mai mult decat
`CameraCalibration.load`: vezi `check_calibration`.
"""

import argparse
import collections
import getpass
import logging
import logging.handlers
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import config as nova_config                      # noqa: E402
from nova.detector_pi import CameraCalibration              # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Log persistent cu rotatie. 5 fisiere x 4 MB = 20 MB, suficient pentru mai
#: multe zile de sesiuni si destul de mic pentru un card SD.
LOG_DIR = os.path.join(REPO_ROOT, 'logs')
LOG_NAME = 'nova-monitor.log'
LOG_MAX_BYTES = 4 * 1024 * 1024
LOG_BACKUPS = 5

#: Ring buffer (pregatire pentru grupul C / 8.3.3).
#: Un cadru de tracking 2304x1296 gri = 2.99 MB. 30 de cadre = 1.0 s la
#: 30 fps = 90 MB. Pi 4 are 4 GB, deci incape cu mult, dar buffer-ul e in
#: RAM si creste liniar: 2 s ar fi 180 MB.
# DECIZIE DESCHISA: 8.3.3 cere cadrul de la CONTACT, iar §4 arata ca deriva
# intre captura si contact e sub 1.2 cm, deci 1 s de istoric e suficient cu
# marja. Grupul C poate cere mai mult; se schimba din --buffer-frames.
from nova.frame_ring import RING_FRAMES, FrameRing  # noqa: E402,F401

#: Asteptarea perifericelor la boot. Serviciul poate porni inaintea lor;
#: systemd le-ar reporni oricum, dar un restart in bucla polueaza logul.
WAIT_SERIAL_S = 30.0
WAIT_POLL_S = 0.5

#: Cat de des scriem o linie de stare in log (secunde).
STATUS_EVERY_S = 30.0

UNIT_NAME = 'nova-monitor.service'
UNIT_TEMPLATE = """\
# NOVA - ZDC 2026 - serviciu de monitorizare (RACE_MONITOR)
#
# Generat de tools/nova_service.py --install-unit. Nu edita direct: valorile
# de mai jos sunt cele de pe masina pe care s-a rulat generarea.
#
# IMPLICIT DEZACTIVAT (E0). Se activeaza manual, dupa ce E2 trece:
#     sudo systemctl enable --now {unit}
# Oprire:
#     sudo systemctl disable --now {unit}
#
# Serviciul NU comanda vehiculul - vezi ReadOnlyVehicle in nova_service.py.

[Unit]
Description=NOVA ZDC - monitor de bord (RACE_MONITOR, fara comenzi)
Documentation=file://{repo}/CLAUDE.md
# Camera si serialul nu au unitati proprii pe care sa le asteptam: libcamera
# apare odata cu udev, iar /dev/serial0 e un symlink creat de firmware.
# Asteptarea propriu-zisa o face serviciul (--wait-serial), care poate da si
# un mesaj util in log. Aici cerem doar ca sistemul de fisiere si reteaua
# locala sa fie gata.
After=local-fs.target network.target
Wants=network.target
# StartLimit* sunt chei de [Unit], nu de [Service]. Puse in [Service] sunt
# ignorate TACUT - `systemd-analyze verify` le raporteaza ca "Unknown key
# name", dar serviciul porneste si pare configurat. Inca o instanta din §5.10:
# acceptat fara eroare nu inseamna aplicat.
# Daca pica de 5 ori in 5 minute, ceva e rupt structural (calibrare lipsa,
# camera defecta). Ne oprim si lasam logul sa fie citit, in loc sa
# reincercam la infinit.
StartLimitBurst=5
StartLimitIntervalSec=300

[Service]
Type=simple
User={user}
Group={group}
WorkingDirectory={repo}
ExecStart={python} {repo}/tools/nova_service.py --conn {conn} --baud {baud}

# Restart la esec, nu la oprire curata: `systemctl stop` trebuie sa opreasca.
# RestartSec 5 s ca o camera ocupata sau un serial inca inexistent sa aiba
# timp sa apara, fara sa umple logul.
Restart=on-failure
RestartSec=5

# Logul propriu e in {repo}/logs/ (rotativ). Ce apare pe stdout merge si in
# journal, ca `systemctl status` sa arate ceva util.
StandardOutput=journal
StandardError=journal

# Acces la camera si la serial.
SupplementaryGroups=video dialout

# Intarire: serviciul citeste camera si serialul si scrie doar in logs/ si
# data/. Nu are nevoie de nimic altceva.
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=read-only
ReadWritePaths={repo}/logs {repo}/data

[Install]
WantedBy=multi-user.target
"""


# --- verificari de pornire --------------------------------------------------

class StartupRefusal(Exception):
    """Refuz deliberat de pornire, nu eroare de program. Serviciul iese cu
    cod 2, ca systemd si preflight-ul sa il poata deosebi de un crash."""


def check_calibration(path, max_rms=None):
    """Calibrarea, sau `StartupRefusal` cu motivul exact.

    `CameraCalibration.load(require_real=True)` prinde deja fisierul lipsa,
    `n_images = 0` si RMS-ul peste prag. Aici adaugam ce ii scapa: un fisier
    scris de mana care arata ca o calibrare reala si contine de fapt focala
    geometrica din fisa tehnica.

    Nu se poate verifica dupa VALOAREA focalei - o calibrare reala a acestui
    obiectiv da ~933 px, adica exact cat da si formula geometrica; de aia e
    formula un punct de plecare rezonabil. Semnatura care le deosebeste e
    distorsiunea: un obiectiv de 102 grade are k1 de ordinul -0.05, iar
    `CameraCalibration.geometric()` pune coeficienti IDENTIC zero. Un zero
    perfect pe toti coeficientii nu apare niciodata dintr-o calibrare reala.
    """
    # Pragul vine din UN singur loc: MAX_REPROJ_ERR_PX. Aici statea o copie
    # scrisa de mana, `max_rms=0.5`, iar cand pragul s-a ridicat la 0.85
    # (decizia echipei) preflight-ul - care importa functia asta, nu
    # constanta - a continuat sa refuze calibrarea cu "0.829 > 0.5".
    from nova.detector_pi import MAX_REPROJ_ERR_PX
    if max_rms is None:
        max_rms = MAX_REPROJ_ERR_PX
    try:
        cal = CameraCalibration.load(path, require_real=True, max_rms=max_rms)
    except (FileNotFoundError, ValueError) as e:
        raise StartupRefusal(str(e)) from e

    sursa = (cal.source or '').lower()
    if 'geometric' in sursa:
        raise StartupRefusal(
            f"{path}: sursa calibrarii e '{cal.source}'.\n"
            f"  Focala geometrica din fisa tehnica NU e o calibrare. "
            f"Ruleaza tools/calibrate_camera.py.")

    if not cal.dist.any():
        raise StartupRefusal(
            f"{path}: toti coeficientii de distorsiune sunt zero.\n"
            f"  La 102 grade HFOV asta e imposibil pentru o calibrare reala "
            f"(asteptat k1 ~ -0.05). Fisierul pare scris de mana sau copiat "
            f"de la o camera cu alt obiectiv. Ruleaza tools/calibrate_camera.py.")

    return cal


def wait_for_path(path, timeout=WAIT_SERIAL_S, poll=WAIT_POLL_S, log=None):
    """True daca `path` a aparut in `timeout` secunde. La boot, /dev/serial0
    poate lipsi cateva secunde dupa ce systemd porneste serviciul."""
    t0 = time.monotonic()
    warned = False
    while time.monotonic() - t0 < timeout:
        if os.path.exists(path):
            return True
        if log and not warned:
            log.info("astept %s (pana la %.0f s)", path, timeout)
            warned = True
        time.sleep(poll)
    return os.path.exists(path)


# --- bariera de comanda -----------------------------------------------------

class ReadOnlyVehicle:
    """Vehicle fara nicio cale de comanda.

    **Lista e alba, nu neagra, si asta e tot rostul clasei.** Prima varianta
    enumera metodele de comanda si lasa restul sa treaca. Am scris-o asa si
    am ratat imediat `update_params()`, care retrimite `PARAM_SET` pentru
    valorile neconfirmate - nu incepe cu niciun prefix de "comanda" si arata
    ca o metoda de intretinere. Cu lista neagra, o metoda noua in Vehicle e
    permisa pana isi aminteste cineva sa o interzica; cu lista alba e
    interzisa pana decide cineva ca e sigura. Singura directie acceptabila
    pentru o bariera de siguranta e a doua (acelasi rationament ca
    `DETECTION_MONITORED_PHASES` in §8).

    `m` - conexiunea mavutil bruta - e blocat explicit, altfel bariera ar fi
    decorativa: `vehicle.m.mav.command_long_send(...)` ar ocoli-o complet.
    """

    #: Suprafata de CITIRE. Orice altceva ridica PermissionError.
    ALLOWED = frozenset({
        'pump',           # citeste mesaje din socket si actualizeaza starea
        'alt',            # PROPRIETATE, nu metoda: `v.alt`, nu `v.alt()`
        'on_ground',      # EXTENDED_SYS_STATE
        'mode_name',      # HEARTBEAT
        'connect',        # deschide legatura; nu comanda nimic
        'param_pending',  # interogare de stare interna
        'params',         # dictionarul de parametri cititi
        'time_boot_ms',   # ceasul FC-ului (6.2.1.30)
        # H1: sanatatea legaturii.
        'link_healthy',
        'time_since_heartbeat',
        'reconnects',
        'reconnect_attempts',
        'on_link_event',
        # `check_link` e singura intrare din lista care trimite ceva pe fir:
        # dupa o redeschidere reusita cere din nou fluxurile de telemetrie
        # (SET_MESSAGE_INTERVAL). E o cerere de DATE, nu o comanda de zbor -
        # nu poate schimba modul, nu poate arma si nu poate misca vehiculul.
        # Un monitor care nu se poate reconecta ar deveni inutil la prima
        # miscare de cablu, adica exact scenariul pentru care exista H1.
        'check_link',
    })

    def __init__(self, vehicle):
        object.__setattr__(self, '_v', vehicle)

    def __getattr__(self, name):
        if name not in self.ALLOWED:
            raise PermissionError(
                f"RACE_MONITOR: acces blocat la Vehicle.{name}. Serviciul de "
                f"bord nu comanda vehiculul; lista de citire e "
                f"ReadOnlyVehicle.ALLOWED. Vezi nova_service.py.")
        return getattr(self._v, name)

    def __setattr__(self, name, value):
        raise PermissionError(
            f"RACE_MONITOR: nu se scrie in Vehicle ({name}).")

    @classmethod
    def write_paths_allowed(cls, vehicle_cls):
        """Metode permise care totusi trimit octeti catre FC.

        Rulata de teste. Citeste sursa fiecarei metode din ALLOWED si cauta
        apeluri `self.m.mav.*_send(`, mai putin cele de tip cerere
        (`*_request_*`, `request_data_stream`). Daca cineva adauga in ALLOWED
        ceva care scrie, testul pica aici - nu serviciul, in zbor."""
        import inspect
        import re
        scrie = []
        for name in sorted(cls.ALLOWED):
            fn = getattr(vehicle_cls, name, None)
            if not callable(fn):
                continue
            try:
                src = inspect.getsource(fn)
            except (OSError, TypeError):
                continue
            for m in re.finditer(r'\.mav\.(\w+?)_send\(', src):
                trimis = m.group(1)
                if 'request' in trimis or trimis in ('heartbeat',):
                    continue
                scrie.append(f"{name} -> {trimis}_send")
        return scrie


# --- ring buffer ---------------------------------------------------------
# Mutat in `nova/frame_ring.py` la J2: acelasi ring e folosit si de
# aplicatia de bord (8.3.3), iar doua copii ar fi divergat.

class RingTapSource:
    """Deriveaza cadrele intr-un FrameRing, fara sa schimbe sursa.

    Alternativa ar fi fost ca PiDetector sa tina el buffer-ul, dar asta ar
    fi cerut o modificare in nova/detector_pi.py, care e validat. Un
    decorator peste `FrameSource` face acelasi lucru cu zero risc de
    regresie: interfata e doar `read()` / `close()` / `nominal_fps`."""

    def __init__(self, source, ring):
        self.source = source
        self.ring = ring
        self.nominal_fps = getattr(source, 'nominal_fps', None)

    def read(self):
        item = self.source.read()
        if item is not None:
            frame, t = item
            self.ring.push(frame, t)
        return item

    def close(self):
        self.source.close()

    def __getattr__(self, name):
        # capture_scoring_frame, control_problems, size etc.
        return getattr(self.source, name)


# --- log --------------------------------------------------------------------

def setup_logging(log_dir=LOG_DIR, verbose=True):
    """Log rotativ in fisier + stdout (care ajunge in journal)."""
    os.makedirs(log_dir, exist_ok=True)
    log = logging.getLogger('nova.monitor')
    log.setLevel(logging.INFO)
    log.handlers.clear()
    fmt = logging.Formatter('%(asctime)s %(levelname)-7s %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')

    fh = logging.handlers.RotatingFileHandler(
        os.path.join(log_dir, LOG_NAME), maxBytes=LOG_MAX_BYTES,
        backupCount=LOG_BACKUPS)
    fh.setFormatter(fmt)
    log.addHandler(fh)

    if verbose:
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(fmt)
        log.addHandler(sh)
    return log


# --- unitatea systemd -------------------------------------------------------

def render_unit(repo=REPO_ROOT, python=None, user=None, group=None,
                conn='/dev/serial0', baud=921600):
    """Unitatea, cu caile masinii curente completate."""
    user = user or getpass.getuser()
    return UNIT_TEMPLATE.format(
        unit=UNIT_NAME, repo=repo, python=python or sys.executable,
        user=user, group=group or user, conn=conn, baud=baud)


def install_unit(args):
    text = render_unit(python=args.python, conn=args.conn, baud=args.baud)
    dest = args.unit_path or os.path.join(REPO_ROOT, 'systemd', UNIT_NAME)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, 'w') as f:
        f.write(text)
    print(f"  scris: {dest}\n")
    print("  Instalare (serviciul ramane DEZACTIVAT, E0):\n")
    print(f"    sudo cp {dest} /etc/systemd/system/{UNIT_NAME}")
    print("    sudo systemctl daemon-reload")
    print(f"    sudo systemctl disable {UNIT_NAME}   # explicit, implicit")
    print(f"    sudo systemctl start {UNIT_NAME}     # o rulare de proba\n")
    print("  Activarea la boot se face DUPA ce E2 trece criteriile:\n")
    print(f"    sudo systemctl enable --now {UNIT_NAME}\n")
    return 0


# --- bucla ------------------------------------------------------------------

def build_monitor_detector(cfg, cal, ring, source=None):
    """Detectorul de bord, cu ring buffer derivat din sursa.

    Reface ce face `build_pi_detector`, cu doua diferente: sursa trece prin
    `RingTapSource`, si se poate injecta o sursa in teste. Verificarea de
    rezolutie e aceeasi si e la fel de necesara - o calibrare pentru alta
    rezolutie da distante gresite, tacut."""
    from nova.detector_pi import (ArucoMarkerDetector, PiCameraSource,
                                  PiDetector)
    aruco = ArucoMarkerDetector(cal, marker_id=cfg['marker_id'],
                                marker_size_m=cfg['marker_size_m'],
                                roi_below_m=cfg['roi_below_m'],
                                roi_size_px=cfg['roi_size_px'],
                                camera_rotation_deg=cfg['camera_rotation_deg'])
    if source is None:
        source = PiCameraSource(verbose=False)
        if (source.size[0], source.size[1]) != (cal.width, cal.height):
            raise StartupRefusal(
                f"calibrarea e pentru {cal.width}x{cal.height}, camera da "
                f"{source.size[0]}x{source.size[1]}. Recalibreaza la "
                f"rezolutia de tracking.")
    return PiDetector(RingTapSource(source, ring), aruco, threaded=True).start()


def run_monitor(cfg, cal, args, log, detector=None, vehicle=None, ring=None,
                source=None):
    """RACE_MONITOR. `detector`, `vehicle`, `source` se pot injecta (teste)."""
    ring = ring if ring is not None else FrameRing(args.buffer_frames)
    if detector is None:
        detector = build_monitor_detector(cfg, cal, ring, source=source)

    if vehicle is None and args.conn:
        if not wait_for_path(args.conn, args.wait_serial, log=log):
            # Nu e motiv de refuz: monitorul are sens si fara FC (banc,
            # masurare de detectie). Doar il notam si mergem mai departe.
            log.warning("%s nu a aparut in %.0f s - rulez fara telemetrie",
                        args.conn, args.wait_serial)
        else:
            from nova.vehicle import Vehicle
            baud = args.baud if not args.conn.startswith(('udp', 'tcp')) else None
            vehicle = ReadOnlyVehicle(Vehicle(args.conn, baud=baud).connect())
            log.info("telemetrie: %s (numai citire)", args.conn)

    # Dump la cerere, pe SIGUSR1. Serviciul tine ultima secunda in RAM;
    # fara o cale de a o scoate, ea se pierde la fiecare repornire - exact
    # cadrele de care e nevoie dupa un incident.
    #     kill -USR1 $(systemctl show -p MainPID --value nova-monitor)
    def _dump(_signum=None, _frame=None):
        try:
            scrise = ring.dump(args.frames_dir)
            log.info("ring buffer scris: %d cadre in %s", len(scrise),
                     args.frames_dir)
        except Exception as e:                               # noqa: BLE001
            log.error("dump-ul ring bufferului a esuat: %s", e)

    try:
        import signal
        signal.signal(signal.SIGUSR1, _dump)
        log.info("SIGUSR1 -> scrie ring bufferul in %s", args.frames_dir)
    except (ValueError, OSError, AttributeError):
        # Fara fir principal (teste) sau pe platforme fara SIGUSR1.
        pass

    log.info("RACE_MONITOR pornit | calibrare %s | buffer %d cadre",
             cal, args.buffer_frames)
    log.info("autonomy_enabled=%s | serviciul NU comanda vehiculul",
             cfg.get('autonomy_enabled') is True)

    t_status = 0.0
    n_det = 0
    t0 = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if vehicle is not None:
                vehicle.pump()
                vehicle.check_link(now)      # H1, nu blocheaza
            # Buffer-ul se umple in RingTapSource, pe firul de captura, deci
            # tine si cadrele in care markerul NU a fost vazut - exact ce
            # trebuie pentru 8.3.3, unde sub 0.38 m nu mai exista detectie.
            n_det += len(detector.poll(now))
            if now - t_status >= args.status_every:
                t_status = now

                # `alt` e o PROPRIETATE in Vehicle, nu o metoda. Prima
                # varianta scria `vehicle.alt()`, ceea ce arunca TypeError
                # ('float' object is not callable) - prins de except-ul de
                # mai jos, deci altitudinea lipsea tacut din TOATE liniile de
                # stare. Un except larg in jurul unei citiri de telemetrie
                # ascunde exact genul asta de greseala.
                alt = None
                link = ''
                if vehicle is not None:
                    try:
                        alt = vehicle.alt
                    except Exception:                        # noqa: BLE001
                        alt = None
                    try:
                        age = vehicle.time_since_heartbeat()
                        link = (' | link CAZUT' if not vehicle.link_healthy
                                else f" | link {age:.1f}s"
                                if age is not None else '')
                    except Exception:                        # noqa: BLE001
                        link = ''
                log.info("%s | detectii %d | buffer %d cadre / %.1f s / "
                         "%.0f MB%s%s", detector.status_line(), n_det,
                         len(ring), ring.span_s(), ring.nbytes() / 1e6,
                         '' if alt is None else f" | alt {alt:.2f} m", link)
            if getattr(detector, 'exhausted', False) and not detector.poll(now):
                log.info("sursa de cadre s-a terminat, ies")
                break
            if args.max_seconds and now - t0 >= args.max_seconds:
                log.info("--max-seconds atins, ies")
                break
            time.sleep(args.loop_sleep)
    except KeyboardInterrupt:
        log.info("oprire la cerere")
    finally:
        detector.stop()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default=None, help='implicit config/nova.json')
    p.add_argument('--conn', default='/dev/serial0',
                   help="'' dezactiveaza complet telemetria")
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--buffer-frames', type=int, default=RING_FRAMES)
    p.add_argument('--wait-serial', type=float, default=WAIT_SERIAL_S)
    p.add_argument('--status-every', type=float, default=STATUS_EVERY_S)
    p.add_argument('--loop-sleep', type=float, default=0.02)
    p.add_argument('--max-seconds', type=float, default=0.0,
                   help='0 = la nesfarsit (implicit sub systemd)')
    p.add_argument('--log-dir', default=LOG_DIR)
    p.add_argument('--frames-dir',
                   default=os.path.join(REPO_ROOT, 'data', 'frames'),
                   help='unde scrie ring bufferul la SIGUSR1')
    p.add_argument('--check', action='store_true',
                   help='ruleaza verificarile de pornire si iese')
    p.add_argument('--install-unit', action='store_true')
    p.add_argument('--unit-path', default=None)
    p.add_argument('--python', default=None, help='pentru --install-unit')
    a = p.parse_args(argv)

    if a.install_unit:
        return install_unit(a)

    log = setup_logging(a.log_dir)
    cfg = nova_config.load(a.config)
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    try:
        cal = check_calibration(cal_path)
    except StartupRefusal as e:
        # Codul 2 il deosebeste de un crash. systemd il trateaza la fel
        # (Restart=on-failure), dar in log si in `systemctl status` se vede
        # imediat ca e un refuz, nu o exceptie.
        log.error("NU PORNESC: %s", e)
        return 2

    if a.check:
        log.info("verificari trecute: %s", cal)
        return 0
    try:
        return run_monitor(cfg, cal, a, log)
    except ModuleNotFoundError as e:
        log.error("NU PORNESC: %s - picamera2 exista doar pe Pi.", e)
        return 2


if __name__ == '__main__':
    sys.exit(main())
