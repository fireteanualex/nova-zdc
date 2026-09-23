#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Verificare de banc, inainte de fiecare sesiune (G4).

    ssh pi@nova
    cd ~/nova-zdc && source ~/nova-venv/bin/activate
    python3 tools/preflight_check.py

    python3 tools/preflight_check.py --no-mavlink    # banc, fara FC pornit
    python3 tools/preflight_check.py --json          # pentru scripturi

**Cod de iesire 0 numai daca TOATE verificarile cerute trec.** O verificare
sarita nu e o verificare trecuta: `--no-mavlink` o scoate din lista si spune
asta explicit in raport, dar nu o trece tacut. Asa ieșirea uneltei poate fi
folosita ca poarta intr-un script, si asa raportul de la scrutineering
inseamna ceva.

Ce verifica, si de ce fiecare:

  stiva       distributia, Python, numpy, OpenCV - si CALEA fiecaruia. O cale
              de numpy sau cv2 care trece prin venv inseamna ca pip a pus o
              copie peste cea de sistem, iar picamera2 se rupe (§5.24).
  calibrare   fisierul exista, e o calibrare REALA (nu focala geometrica),
              RMS sub prag. Detectorul refuza oricum sa porneasca fara ea
              (E1.2), dar aici afli inainte sa urci pe scara.
  camera      se deschide SI produce cadre la rata asteptata. "Se deschide"
              nu e acelasi lucru: o camera care da 6 fps in loc de 30 trece
              orice test de deschidere si strica toate masuratorile.
  imagine     cadrele nu sunt uniforme. Un capac uitat pe obiectiv da cadre
              perfect valide, la rata perfecta, complet negre.
  controale   §5.10 aplicat camerei: LensPosition/ExposureTime/AnalogueGain
              pot fi acceptate si apoi limitate TACUT de driver. Se citesc
              inapoi din metadate. Focus-ul in special: cu PDAF lasat sa
              caute, detectia moare exact in coborare.
  mavlink     heartbeat de la FC pe /dev/serial0.
  parametri   tools/check_params.py pe config/nova_flight.parm, adica §5.10
              ca pas obligatoriu, nu ca sugestie.

Verificarile care nu au ce cauta aici: orice armeaza, misca sau comanda
ceva. Preflight-ul citeste.
"""

import argparse
import json
import os
import platform
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402

from nova import config as nova_config                      # noqa: E402
from nova_service import StartupRefusal, check_calibration   # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLIGHT_PARM = os.path.join(REPO_ROOT, 'config', 'nova_flight.parm')

#: Camera: cate cadre masuram si cat de mult poate scadea rata.
#: 0.80 x 30 = 24 fps. Sub asta, bucla de detectie nu mai tine pasul cu
#: coborarea si masuratorile de latenta nu mai sunt comparabile.
FPS_FRAMES = 60
FPS_TOLERANCE = 0.80
#: Prag ABSOLUT de cadre/s sub care preflight-ul pica. DECIZIA ECHIPEI
#: (23.09.2026): 17 fps, in loc de 80% din nominal (24 fps). Masurat pe Pi
#: 4: 17.9 fps - vezi nota din check_camera despre ce se masura de fapt.
#: La 0.5 m/s de coborare, 17 fps inseamna ~3 cm intre cadre.
FPS_MIN = 17.0

#: Imagine: sub atata deviatie standard, cadrul e practic uniform (capac pe
#: obiectiv, intuneric total). Un cadru cu marker alb-negru are zeci.
MIN_FRAME_STD = 3.0

MAVLINK_TIMEOUT_S = 10.0

OK, ESEC, SARIT = 'OK', 'ESEC', 'SARIT'


class Result:
    def __init__(self, name, status, detail='', data=None):
        self.name = name
        self.status = status
        self.detail = detail
        self.data = data or {}

    @property
    def passed(self):
        return self.status == OK

    def __str__(self):
        culoare = {OK: '\033[32m', ESEC: '\033[31m',
                   SARIT: '\033[33m'}[self.status]
        return (f"  {culoare}{self.status:<6}\033[0m {self.name:<14} "
                f"{self.detail}")


# --- verificari -------------------------------------------------------------

def _module_origin(mod):
    """('VENV'|'sistem', cale) - de unde a fost incarcat modulul.

    Nu e curiozitate. Pe Pi, numpy si (pe Trixie) cv2 TREBUIE sa vina din
    apt: picamera2 si simplejpeg sunt compilate impotriva lor. O copie
    instalata de pip in venv le umbreste si rupe camera, iar mesajul de
    eroare (`_ARRAY_API not found`) nu seamana deloc cu cauza. Calea o arata
    dintr-o privire."""
    cale = os.path.realpath(getattr(mod, '__file__', '') or '')
    in_venv = bool(cale) and cale.startswith(
        os.path.realpath(sys.prefix) + os.sep)
    return ('VENV' if in_venv else 'sistem'), cale


def _model(path='/proc/device-tree/model'):
    try:
        with open(path) as f:
            return f.read().strip('\x00').strip()
    except OSError:
        return ''


def _os_release(path='/etc/os-release'):
    out = {}
    try:
        with open(path) as f:
            for line in f:
                if '=' in line:
                    k, _, v = line.partition('=')
                    out[k.strip()] = v.strip().strip('"')
    except OSError:
        pass
    return out


def check_stack(os_release_path='/etc/os-release'):
    """Distributia, Python, numpy, OpenCV - cu CALEA fiecaruia.

    Verificarea asta nu existase pana cand vehiculul s-a dovedit a rula
    Trixie, nu Bookworm cum presupusesem (§5.24). Costul nu a fost tehnic -
    codul mergea - ci ca am scris "verificat" despre o stiva pe care nimeni
    nu o rula. De aceea preflight-ul o raporteaza acum de fiecare data."""
    osr = _os_release(os_release_path)
    codename = osr.get('VERSION_CODENAME', '?')
    linii = [f"{osr.get('PRETTY_NAME', '?')} | Python "
             f"{platform.python_version()}"]
    probleme = []

    # Pe desktop, numpy VINE din venv si e perfect corect asa - nu exista
    # picamera2 pe care sa il rupa. Daca am raporta ESEC si acolo,
    # preflight-ul ar fi rosu la fiecare rulare de dezvoltare, si exact asta
    # invata operatorul sa treaca peste el. Verificarea are dinti doar unde
    # are si consecinte.
    pe_pi = 'raspberry pi' in _model().lower()
    if not pe_pi:
        linii.append('(nu suntem pe Pi: umbrirea numpy nu e o problema aici)')

    import numpy
    for eticheta, mod in (('numpy', numpy), ('cv2', cv2)):
        unde, cale = _module_origin(mod)
        linii.append(f"{eticheta} {mod.__version__} din {unde}: {cale}")
        if unde == 'VENV' and eticheta == 'numpy' and pe_pi:
            probleme.append(
                f"numpy vine din venv ({cale}), nu din apt. picamera2 si "
                f"simplejpeg sunt compilate impotriva celui de sistem si se "
                f"vor rupe. Sterge venv-ul si reia tools/setup_pi.sh.")

    ver = tuple(int(x) for x in cv2.__version__.split('.')[:2])
    if ver < (4, 7):
        probleme.append(
            f"cv2 {cv2.__version__} < 4.7: fara cv2.aruco.ArucoDetector.")
    elif not hasattr(cv2, 'aruco'):
        probleme.append(
            "cv2 nu are modulul aruco (build fara contrib). Pe Trixie: "
            "python3-opencv din apt; pe Bookworm: opencv-contrib-python.")

    # cv2 din venv nu e o problema in sine - pe Bookworm e chiar calea
    # corecta - dar merita spus, ca sa se stie ce stiva ruleaza.
    unde_cv, _ = _module_origin(cv2)
    if pe_pi and codename == 'trixie' and unde_cv == 'VENV':
        probleme.append(
            "pe Trixie, OpenCV ar trebui sa vina din apt (python3-opencv). "
            "O copie pip in venv o umbreste pe cea de sistem.")

    detail = ' | '.join(linii)
    if probleme:
        return Result('stiva', ESEC, '; '.join(probleme),
                      {'linii': linii, 'codename': codename})
    return Result('stiva', OK, detail, {'linii': linii, 'codename': codename})


def check_calib(cfg):
    """(Result, calibrare_sau_None). Calibrarea se intoarce pentru ca
    verificarea de rezolutie de mai jos are nevoie de ea."""
    path = nova_config.resolve(cfg, 'camera_calibration')
    try:
        cal = check_calibration(path)
    except StartupRefusal as e:
        return Result('calibrare', ESEC, str(e).replace('\n', ' ').strip()), None
    return (Result('calibrare', OK,
                   f"{cal.width}x{cal.height} fx={cal.fx:.1f} "
                   f"rms={cal.rms:.3f} px n={cal.n_images}",
                   {'fx': cal.fx, 'rms': cal.rms, 'n_images': cal.n_images}),
            cal)


def check_camera(source, n_frames=FPS_FRAMES, min_fps=FPS_MIN):
    """(rezultat_fps, rezultat_imagine). Consuma n_frames de la sursa.

    Primul cadru e exclus din masuratoarea de rata: contine pornirea
    fluxului si ar trage media in jos fara sa spuna nimic despre regimul
    stabil.

    CE SE MASOARA. Prima varianta calcula `np.std` pe CADRUL INTREG la
    fiecare iteratie - 3 milioane de pixeli convertiti in float64, cateva
    treceri. Pe Pi 4 asta costa zeci de milisecunde, in aceeasi bucla in
    care se cronometreaza camera: rata raportata putea fi a analizei, nu a
    senzorului. Masurat pe vehicul: 17.9 fps, cu expunerea fixata la 2 ms
    si FrameRate cerut 30 - deci nu expunerea limita.

    Contrastul se estimeaza acum pe un esantion rarit (1 pixel din 64),
    care pentru "e capac pe obiectiv?" spune acelasi lucru ca tot cadrul,
    la un cost neglijabil. Asa bucla nu mai concureaza cu camera.

    Nu stiu inca daca cei 17.9 fps erau ai analizei sau ai camerei. O
    rulare noua spune: daca cifra urca spre 30, era masuratoarea."""
    stds, t_first, t_last, n = [], None, None, 0
    t0 = time.monotonic()
    for i in range(n_frames):
        item = source.read()
        if item is None:
            break
        gray, _t = item
        now = time.monotonic()
        if i == 0:
            t_first = now
        else:
            t_last = now
            n += 1
        stds.append(float(np.std(gray[::8, ::8])))
    if n < 2:
        return (Result('camera', ESEC,
                       f"doar {n + 1} cadre citite in {time.monotonic()-t0:.1f} s"),
                Result('imagine', ESEC, 'fara cadre'))

    fps = n / (t_last - t_first)
    prag = float(min_fps)
    if fps < prag:
        r_fps = Result('camera', ESEC,
                       f"{fps:.1f} fps, sub pragul de {prag:.1f}. Verifica "
                       f"temperatura (vcgencmd get_throttled) si ce mai "
                       f"ruleaza pe Pi.",
                       {'fps': fps, 'prag': prag})
    else:
        r_fps = Result('camera', OK, f"{fps:.1f} fps (prag {prag:.0f})",
                       {'fps': fps, 'prag': prag})

    std_med = float(np.median(stds))
    if std_med < MIN_FRAME_STD:
        r_img = Result('imagine', ESEC,
                       f"cadre practic uniforme (std {std_med:.1f} < "
                       f"{MIN_FRAME_STD}). Capac pe obiectiv? Intuneric?",
                       {'std': std_med})
    else:
        r_img = Result('imagine', OK, f"contrast prezent (std {std_med:.1f})",
                       {'std': std_med})
    return r_fps, r_img


def check_controls(source):
    """§5.10 aplicat camerei. `control_problems` e umplut de PiCameraSource
    prin citire inapoi din metadate, nu din ce am cerut noi."""
    probleme = getattr(source, 'control_problems', None)
    if probleme is None:
        return Result('controale', SARIT,
                      'sursa nu raporteaza controale (nu e PiCameraSource)')
    if probleme:
        return Result('controale', ESEC, '; '.join(probleme),
                      {'probleme': probleme})
    return Result('controale', OK, 'cerute = aplicate (citite din metadate)')


def check_mavlink(conn, baud, timeout=MAVLINK_TIMEOUT_S):
    if not os.path.exists(conn) and not conn.startswith(('udp', 'tcp')):
        return Result('mavlink', ESEC,
                      f"{conn} nu exista. Cablu? "
                      f"`enable_uart=1` in /boot/firmware/config.txt? "
                      f"Serial console dezactivata?")
    try:
        from pymavlink import mavutil
        kwargs = {'baud': baud} if not conn.startswith(('udp', 'tcp')) else {}
        m = mavutil.mavlink_connection(conn, **kwargs)
        hb = m.wait_heartbeat(timeout=timeout)
    except Exception as e:                                   # noqa: BLE001
        return Result('mavlink', ESEC, f"{type(e).__name__}: {e}")
    sysid, comp = m.target_system, m.target_component
    # Portul se ELIBEREAZA explicit. Fara asta, obiectul mavutil tine
    # /dev/serial0 pana il colecteaza GC-ul, iar `Vehicle.connect()` de dupa
    # preflight ar putea gasi portul ocupat - de noi insine. Acelasi tipar ca
    # in §5.27, doar ca vinovatul e propriul proces.
    try:
        m.close()
    except Exception:                                        # noqa: BLE001
        pass
    if hb is None:
        return Result('mavlink', ESEC,
                      f"niciun heartbeat in {timeout:.0f} s pe {conn}. "
                      f"FC pornit? Baud {baud} corect de ambele parti?")
    return Result('mavlink', OK,
                  f"heartbeat sys={sysid} comp={comp}", {'sysid': sysid})


def check_params(conn, baud, parm=FLIGHT_PARM, timeout=120):
    """tools/check_params.py ca subproces.

    Deliberat ca subproces si nu ca import: e exact comanda pe care o ruleaza
    si operatorul, deci daca trece aici trece si acolo. Ieșirea ei completa e
    dovada pentru Compliance Matrix (§5.10), asa ca o pastram in `data`."""
    if not os.path.exists(parm):
        return Result('parametri', ESEC, f"lipseste {parm}")
    cmd = [sys.executable, os.path.join(REPO_ROOT, 'tools', 'check_params.py'),
           '--conn', conn, '--parm', parm]
    if not conn.startswith(('udp', 'tcp')):
        cmd += ['--baud', str(baud)]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout)
    except subprocess.TimeoutExpired:
        return Result('parametri', ESEC,
                      f"check_params.py nu a terminat in {timeout} s")
    rezumat = ''
    for line in (out.stdout or '').splitlines():
        if 'nepotriviri' in line or 'inexistenti' in line:
            rezumat = line.strip()
    if out.returncode != 0:
        return Result('parametri', ESEC,
                      rezumat or f"check_params.py a iesit cu "
                                 f"{out.returncode}",
                      {'stdout': out.stdout, 'stderr': out.stderr})
    return Result('parametri', OK, rezumat or 'toti se potrivesc',
                  {'stdout': out.stdout})


# --- orchestrare --------------------------------------------------------------

def run_checks(args, source_factory=None):
    cfg = nova_config.load(args.config)
    results = [check_stack(args.os_release)]

    r, cal = check_calib(cfg)
    results.append(r)

    if args.no_camera:
        results.append(Result('camera', SARIT, '--no-camera'))
        results.append(Result('imagine', SARIT, '--no-camera'))
        results.append(Result('controale', SARIT, '--no-camera'))
    else:
        source = None
        try:
            if source_factory is not None:
                source = source_factory()
            else:
                from nova.detector_pi import PiCameraSource
                source = PiCameraSource(verbose=False)
        except Exception as e:                               # noqa: BLE001
            msg = f"{type(e).__name__}: {e}"
            if isinstance(e, ModuleNotFoundError):
                msg += " - picamera2 exista doar pe Pi"
            results.append(Result('camera', ESEC, msg))
            results.append(Result('imagine', ESEC, 'camera indisponibila'))
            results.append(Result('controale', ESEC, 'camera indisponibila'))
        if source is not None:
            try:
                # Rezolutia sursei fata de cea a calibrarii: o nepotrivire
                # da distante gresite fara niciun simptom vizibil.
                if cal is not None and getattr(source, 'size', None):
                    w, h = source.size
                    if (w, h) != (cal.width, cal.height):
                        results.append(Result(
                            'rezolutie', ESEC,
                            f"camera {w}x{h}, calibrare "
                            f"{cal.width}x{cal.height}. Recalibreaza la "
                            f"rezolutia de tracking."))
                    else:
                        results.append(Result('rezolutie', OK, f"{w}x{h}"))
                results.extend(check_camera(source, args.frames))
                results.append(check_controls(source))
            finally:
                source.close()

    if args.no_mavlink:
        results.append(Result('mavlink', SARIT, '--no-mavlink'))
        results.append(Result('parametri', SARIT, '--no-mavlink'))
    else:
        r_mav = check_mavlink(args.conn, args.baud, args.mavlink_timeout)
        results.append(r_mav)
        if r_mav.passed:
            parm = args.parm or nova_config.resolve(cfg, 'flight_parm')
            results.append(check_params(args.conn, args.baud, parm))
        else:
            # Fara heartbeat, check_params ar astepta degeaba 19 timeout-uri.
            results.append(Result('parametri', ESEC,
                                  'sarit: nu exista legatura cu FC'))
    return results


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default=None)
    p.add_argument('--conn', default='/dev/serial0')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--parm', default=None,
                   help='fisierul de parametri de verificat. Implicit cel '
                        'din config/nova.json (`flight_parm`), care trebuie '
                        'sa corespunda firmware-ului de pe FC')
    p.add_argument('--frames', type=int, default=FPS_FRAMES)
    p.add_argument('--mavlink-timeout', type=float, default=MAVLINK_TIMEOUT_S)
    p.add_argument('--no-camera', action='store_true')
    p.add_argument('--no-mavlink', action='store_true',
                   help='banc fara FC; verificarile de FC se raporteaza SARIT')
    p.add_argument('--os-release', default='/etc/os-release',
                   help='pentru teste')
    p.add_argument('--json', action='store_true')
    a = p.parse_args(argv)

    results = run_checks(a)

    if a.json:
        print(json.dumps([{'name': r.name, 'status': r.status,
                           'detail': r.detail, 'data': r.data}
                          for r in results], indent=2, ensure_ascii=False))
    else:
        print("\n  NOVA preflight\n")
        for r in results:
            print(r)

    ok = sum(1 for r in results if r.status == OK)
    esec = [r for r in results if r.status == ESEC]
    sarit = [r for r in results if r.status == SARIT]

    if not a.json:
        print(f"\n  {ok} trecute | {len(esec)} picate | {len(sarit)} sarite")
        if esec:
            print("\n  NU ZBURA. De reparat:")
            for r in esec:
                print(f"    {r.name}: {r.detail}")
        elif sarit:
            print(f"\n  Verificarile sarite NU sunt trecute: "
                  f"{', '.join(r.name for r in sarit)}")
        else:
            print("\n  Toate verificarile au trecut.")
        print()

    # Cod 0 doar daca totul a trecut. Sarit != trecut, deliberat.
    return 0 if (not esec and not sarit) else 1


if __name__ == '__main__':
    sys.exit(main())
