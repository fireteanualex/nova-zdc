#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Colectarea evidentei de sesiune (H5, 6.2.1.30).

    python3 tools/collect_session.py --nota "cursa 2, vant lateral"
    python3 tools/collect_session.py --no-fc-log      # fara descarcare .bin
    python3 tools/collect_session.py --list           # ce loguri are FC-ul

Aduna intr-un singur director, cu timestamp, tot ce trebuie predat sau
folosit ca dovada:

    data/sessions/20260913-154500/
        manifest.json        versiuni, hash de commit, parametri, calibrare
        fc/log_42.bin        logul de la FC (6.2.1.30)
        logs/                logurile companion-ului, copiate
        frames/              cadrele din ring buffer, daca s-au scris
        params.txt           parametrii CITITI de la FC, nu cei ceruti
        camera_pi.yaml       calibrarea folosita, copiata ca atare

**De ce automat.** Regulamentul cere `.bin`/`.tlog` plus imaginea de
touchdown, iar Compliance Matrix se sprijina pe ele. Daca se aduna manual,
dupa cursa, cu bateria pe terminate si urmatoarea echipa asteptand, nu se
aduna. Sau se aduna partial, ceea ce e mai rau: un set incomplet arata ca o
dovada pana cand cineva il deschide.

**Sincronizarea.** Toate sursele se leaga prin `time_boot_ms`, ceasul FC-ului
(§8). Logul supervizorului il poarta pe fiecare eveniment; `.bin` e scris de
FC pe acelasi ceas. Ceasul Pi-ului NU e o referinta buna: nu e acelasi, si nu
apare in `.bin`.

**Ce face si ce NU face.** Copiaza si descarca; nu sterge nimic de pe FC si
nu modifica nimic. O sesiune se poate colecta de mai multe ori fara efecte.
"""

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova import config as nova_config                      # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SESSIONS_ROOT = os.path.join(REPO_ROOT, 'data', 'sessions')
LOGS_DIR = os.path.join(REPO_ROOT, 'logs')
FRAMES_DIR = os.path.join(REPO_ROOT, 'data', 'frames')

#: Parametrii cititi inapoi de la FC si pusi in manifest. Nu e o verificare
#: (aia e tools/check_params.py) - e o INREGISTRARE a ce era pe vehicul in
#: momentul zborului. La o contestatie, "ce era setat" e intrebarea.
PARAMS_OF_RECORD = (
    'PLND_ENABLED', 'PLND_TYPE', 'PLND_EST_TYPE', 'PLND_STRICT',
    'PLND_ALT_MIN', 'PLND_YAW_ALIGN',
    'RNGFND1_TYPE', 'RNGFND1_MIN', 'RNGFND1_MAX', 'RNGFND1_GNDCLR',
    'SURFTRAK_MODE', 'TERRAIN_ENABLE', 'WP_RFND_USE', 'ARMING_SKIPCHK',
    'DISARM_DELAY', 'FS_THR_ENABLE',
    'FENCE_ENABLE', 'FENCE_TYPE', 'FENCE_RADIUS', 'FENCE_ALT_MAX',
    'FENCE_ACTION',
    # Parametrii modulati in zbor de nova/authority.py. Sunt inregistrati
    # DUPA zbor, cand modularea a restaurat deja originalele - deci valorile
    # din manifest trebuie sa fie cele nominale. Daca nu sunt, restaurarea a
    # esuat si se vede aici, negru pe alb.
    'PSC_NE_POS_P', 'PSC_NE_VEL_D', 'WP_ACC', 'WP_SPD_DN', 'LAND_SPD_MS',
    'PLND_LAG',
    # Neatins de noi, deliberat (nova/authority.py: FORBIDDEN). Inregistrat
    # tocmai ca sa se poata dovedi ca nu l-am schimbat.
    'PSC_ANGLE_MAX',
)

#: Descarcarea logului pe serial la 921600 e lenta: un log de 10 MB ia
#: minute. Plafonul exista ca sa nu blocam sesiunea din greseala.
LOG_DOWNLOAD_TIMEOUT_S = 600.0
LOG_CHUNK = 90                       # octeti per LOG_DATA, fix in protocol


# --- provenienta ------------------------------------------------------------

def git_info(repo=REPO_ROOT):
    """Hash-ul commit-ului si daca arborele e murdar.

    `dirty` conteaza mai mult decat hash-ul: un hash curat spune exact ce cod
    a zburat; un arbore murdar spune ca NU stim, si atunci evidenta trebuie
    citita cu rezerva. Ascuns, ar fi o minciuna linistitoare."""
    def g(*args):
        try:
            out = subprocess.run(['git', '-C', repo] + list(args),
                                 capture_output=True, text=True, timeout=10)
            return out.stdout.strip() if out.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None

    status = g('status', '--porcelain')
    return {
        'commit': g('rev-parse', 'HEAD'),
        'branch': g('rev-parse', '--abbrev-ref', 'HEAD'),
        'describe': g('describe', '--always', '--dirty'),
        'dirty': None if status is None else bool(status),
        'modificate': [] if not status else status.splitlines()[:50],
    }


def software_info():
    info = {
        'python': platform.python_version(),
        'platform': platform.platform(),
        'host': platform.node(),
    }
    for mod in ('cv2', 'numpy', 'pymavlink'):
        try:
            m = __import__(mod)
            info[mod] = getattr(m, '__version__', '?')
            info[mod + '_path'] = getattr(m, '__file__', None)
        except Exception:                                    # noqa: BLE001
            info[mod] = None
    try:
        with open('/proc/device-tree/model') as f:
            info['model'] = f.read().strip('\x00').strip()
    except OSError:
        info['model'] = None
    return info


# --- FC ---------------------------------------------------------------------

def read_params(m, names=PARAMS_OF_RECORD, timeout=2.0):
    """{nume: valoare | None}. None inseamna ca FC-ul nu a raspuns."""
    out = {}
    for name in names:
        val = None
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode('ascii'), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('ascii', 'ignore')
            if got.rstrip('\x00') == name:
                val = msg.param_value
                break
        out[name] = val
    return out


def list_logs(m, timeout=10.0):
    """[{id, size, time_utc}] de pe FC, prin LOG_REQUEST_LIST."""
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
    intrari, t0, asteptate = {}, time.time(), None
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1.0)
        if msg is None:
            continue
        asteptate = msg.num_logs
        if msg.num_logs == 0:
            break
        intrari[msg.id] = {'id': msg.id, 'size': msg.size,
                           'time_utc': msg.time_utc}
        t0 = time.time()
        if asteptate is not None and len(intrari) >= asteptate:
            break
    return [intrari[k] for k in sorted(intrari)]


def download_log(m, log_id, size, dest, timeout=LOG_DOWNLOAD_TIMEOUT_S,
                 progress=None):
    """Descarca un log prin LOG_REQUEST_DATA. (octeti, lipsuri).

    Protocolul nu retrimite de la sine: cerem intervale si urmarim ce a
    sosit. Golurile se raporteaza explicit - un .bin cu gauri e inutilizabil
    ca dovada, si e mai bine sa se stie acum decat la scrutineering."""
    primite = bytearray(size)
    vazut = bytearray(size)          # 1 pe fiecare octet sosit
    m.mav.log_request_data_send(m.target_system, m.target_component,
                                log_id, 0, 0xFFFFFFFF)
    t0 = time.time()
    ultim = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=2.0)
        if msg is None:
            if time.time() - ultim > 5.0:
                break                # FC-ul a tacut
            continue
        ultim = time.time()
        if msg.id != log_id:
            continue
        ofs, n = msg.ofs, msg.count
        date = bytes(bytearray(msg.data[:n]))
        primite[ofs:ofs + n] = date
        vazut[ofs:ofs + n] = b'\x01' * n
        if progress and ofs % (LOG_CHUNK * 200) == 0:
            progress(sum(vazut), size)
        if n < LOG_CHUNK and ofs + n >= size:
            break
    lipsa = size - sum(vazut)
    with open(dest, 'wb') as f:
        f.write(bytes(primite))
    return len(primite), lipsa


# --- colectare ---------------------------------------------------------------

def copy_tree(src, dest, eticheta, raport):
    if not os.path.isdir(src):
        raport.append(f"{eticheta}: lipseste ({src})")
        return 0
    fisiere = [f for f in sorted(os.listdir(src))
               if os.path.isfile(os.path.join(src, f))]
    if not fisiere:
        raport.append(f"{eticheta}: director gol")
        return 0
    os.makedirs(dest, exist_ok=True)
    for f in fisiere:
        shutil.copy2(os.path.join(src, f), os.path.join(dest, f))
    raport.append(f"{eticheta}: {len(fisiere)} fisiere")
    return len(fisiere)


def collect(args, connect=None):
    stamp = args.stamp or time.strftime('%Y%m%d-%H%M%S')
    ses = os.path.join(args.out_root, stamp)
    os.makedirs(ses, exist_ok=True)
    raport = []

    cfg = nova_config.load(args.config)
    manifest = {
        'sesiune': stamp,
        'creat': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'nota': args.nota,
        'git': git_info(),
        'software': software_info(),
        'config': {k: v for k, v in cfg.items() if not k.startswith('_')},
        'sincronizare': (
            'Toate sursele se aliniaza prin time_boot_ms (ceasul FC-ului). '
            'Logul supervizorului il poarta pe fiecare eveniment; .bin e '
            'scris de FC pe acelasi ceas. Ceasul Pi-ului nu e o referinta.'),
    }

    # calibrarea, copiata ca atare
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    if os.path.exists(cal_path):
        shutil.copy2(cal_path, os.path.join(ses, os.path.basename(cal_path)))
        try:
            from nova.detector_pi import CameraCalibration
            cal = CameraCalibration.load(cal_path, require_real=False)
            manifest['calibrare'] = {
                'fisier': os.path.basename(cal_path), 'sursa': cal.source,
                'rms_px': cal.rms, 'n_images': cal.n_images,
                'fx': cal.fx, 'fy': cal.fy, 'cx': cal.cx, 'cy': cal.cy,
                'meta': cal.meta,
            }
            raport.append(f"calibrare: {os.path.basename(cal_path)} "
                          f"(rms {cal.rms}, n={cal.n_images})")
        except Exception as e:                               # noqa: BLE001
            manifest['calibrare'] = {'eroare': str(e)}
            raport.append(f"calibrare: copiata, dar necitibila ({e})")
    else:
        manifest['calibrare'] = None
        raport.append(f"calibrare: LIPSESTE ({cal_path})")

    copy_tree(args.logs_dir, os.path.join(ses, 'logs'), 'loguri companion',
              raport)
    copy_tree(args.frames_dir, os.path.join(ses, 'frames'),
              'cadre din ring buffer', raport)

    # --- FC ---
    manifest['fc'] = {'conectat': False}
    if not args.no_fc:
        m = None
        try:
            if connect is not None:
                m = connect()
            else:
                from pymavlink import mavutil
                kwargs = ({'baud': args.baud}
                          if not args.conn.startswith(('udp', 'tcp')) else {})
                m = mavutil.mavlink_connection(args.conn, **kwargs)
                m.wait_heartbeat(timeout=args.timeout)
        except Exception as e:                               # noqa: BLE001
            raport.append(f"FC: neconectat ({type(e).__name__}: {e})")
            manifest['fc']['eroare'] = f"{type(e).__name__}: {e}"
            m = None

        if m is not None and getattr(m, 'target_system', 0):
            manifest['fc'] = {'conectat': True, 'sysid': m.target_system}
            params = read_params(m)
            manifest['fc']['parametri'] = params
            lipsa = [k for k, v in params.items() if v is None]
            with open(os.path.join(ses, 'params.txt'), 'w') as f:
                f.write("# Parametri CITITI de la FC in momentul colectarii.\n"
                        "# '-' = FC-ul nu a raspuns (parametrul nu exista pe "
                        "acest firmware).\n")
                for k, v in params.items():
                    f.write(f"{k},{'-' if v is None else f'{v:g}'}\n")
            raport.append(f"parametri: {len(params) - len(lipsa)}/"
                          f"{len(params)} cititi de la FC")

            logs = list_logs(m)
            manifest['fc']['loguri'] = logs
            if args.list_only:
                return ses, manifest, raport, logs
            if logs and not args.no_fc_log:
                tinta = logs[-1] if args.log_id is None else next(
                    (x for x in logs if x['id'] == args.log_id), None)
                if tinta is None:
                    raport.append(f"log {args.log_id}: inexistent pe FC")
                else:
                    os.makedirs(os.path.join(ses, 'fc'), exist_ok=True)
                    dest = os.path.join(ses, 'fc', f"log_{tinta['id']}.bin")
                    print(f"  descarc log {tinta['id']} "
                          f"({tinta['size'] / 1e6:.1f} MB)...")
                    n, gol = download_log(
                        m, tinta['id'], tinta['size'], dest,
                        progress=lambda a, b: print(
                            f"\r    {a * 100.0 / max(b, 1):.0f}%", end='',
                            flush=True))
                    print()
                    manifest['fc']['log_descarcat'] = {
                        'id': tinta['id'], 'octeti': n, 'lipsa': gol,
                        'complet': gol == 0}
                    raport.append(
                        f"log FC: {n} octeti" +
                        (f", {gol} LIPSA - .bin incomplet!" if gol else
                         ", complet"))
            elif args.no_fc_log:
                raport.append("log FC: sarit (--no-fc-log)")
            else:
                raport.append("log FC: FC-ul nu raporteaza niciun log")

    with open(os.path.join(ses, 'manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    return ses, manifest, raport, manifest['fc'].get('loguri', [])


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='/dev/serial0')
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--config', default=None)
    p.add_argument('--out-root', default=SESSIONS_ROOT)
    p.add_argument('--logs-dir', default=LOGS_DIR)
    p.add_argument('--frames-dir', default=FRAMES_DIR)
    p.add_argument('--nota', default='')
    p.add_argument('--stamp', default=None)
    p.add_argument('--timeout', type=float, default=10.0)
    p.add_argument('--no-fc', action='store_true',
                   help='nu contacta FC-ul deloc')
    p.add_argument('--no-fc-log', action='store_true',
                   help='parametri da, descarcarea .bin nu (e lenta)')
    p.add_argument('--log-id', type=int, default=None,
                   help='implicit: cel mai recent')
    p.add_argument('--list', dest='list_only', action='store_true',
                   help='listeaza logurile de pe FC si iesi')
    a = p.parse_args(argv)

    ses, manifest, raport, logs = collect(a)

    print(f"\n  sesiune: {ses}\n")
    for linie in raport:
        print(f"    {linie}")

    if a.list_only:
        print(f"\n  {len(logs)} loguri pe FC:")
        for x in logs[-10:]:
            print(f"    id {x['id']:>3}  {x['size'] / 1e6:>7.2f} MB")
        return 0

    g = manifest['git']
    if g.get('dirty'):
        print(f"\n    ATENTIE: arborele git e MURDAR ({g.get('describe')}). "
              f"Codul care a zburat nu corespunde exact cu niciun commit; "
              f"manifestul listeaza fisierele modificate.")
    log_info = manifest['fc'].get('log_descarcat')
    if log_info and not log_info['complet']:
        print(f"\n    ATENTIE: .bin-ul are {log_info['lipsa']} octeti lipsa. "
              f"Reia descarcarea cu --log-id {log_info['id']}.")
    print(f"\n  Adu pe desktop:\n    rsync -av "
          f"{manifest['software']['host']}:{ses}/ ~/nova-zdc/{os.path.relpath(ses, REPO_ROOT)}/\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
