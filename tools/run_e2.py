#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Colectarea datelor pentru E2 - validarea offline a detectorului (G3).

    ssh pi@nova
    cd ~/nova-zdc && source ~/nova-venv/bin/activate
    python3 tools/run_e2.py --lumina soare

Unealta se ruleaza prin SSH, de langa drona. La fiecare statie cere distanta
**masurata cu ruleta**, captureaza N cadre, ruleaza detectia si afiseaza pe
loc rata de detectie, `marker_px` si eroarea fata de distanta masurata.

**De ce pe loc.** Criteriul practic nu e „am adunat datele", ci „am adunat
datele bune". O statie cu 30% rata de detectie inseamna ca ceva e gresit -
focus, expunere, marker murdar, ID gresit - si se vede in 5 secunde. Daca
cifrele apar abia la procesarea de pe desktop, greseala se descopera dupa ce
scara a fost stransa si lumina s-a schimbat.

Ce se scrie pe disc, in `data/e2/<timestamp>/`:

    manifest.json              config, calibrare, versiuni, conditii, mediu
    stations.csv               un rand pe statie: adevar, estimat, eroare
    d05.00_soare/
        frames/0000.png ...    cadrele BRUTE, gri, nemodificate
        detections.csv         un rand pe cadru

Cadrele brute se pastreaza intotdeauna, si la statiile care ies prost -
**mai ales** la ele. O statie cu rata mica de detectie e datul cel mai
valoros din tot setul: pe desktop se poate rula orice alt detector pe exact
aceleasi imagini. Structura e plata si autodescriptiva ca sa poata fi adusa
cu un singur rsync:

    rsync -av pi@nova:~/nova-zdc/data/e2/ ~/nova-zdc/data/e2/

Temperatura si latenta se inregistreaza in paralel, la fiecare cadru, pentru
criteriile termice din E2 (§7/15 din CLAUDE.md): pe Pi 4 fara radiator,
throttling-ul la 80 C se manifesta ca scadere de FPS, iar fara masuratoare
arata identic cu un detector lent.
"""

import argparse
import csv
import json
import os
import platform
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402

from nova import config as nova_config                      # noqa: E402
from nova.detector_pi import (ArucoMarkerDetector,          # noqa: E402
                              CameraCalibration)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_ROOT = os.path.join(REPO_ROOT, 'data', 'e2')

#: Cadre pe statie. 30 la 30 fps = 1 s. Destule pentru o rata de detectie cu
#: sens (rezolutie 3.3%) si destul de putine incat o statie sa dureze
#: secunde, nu minute - la 12 statii x 3 conditii de lumina, diferenta dintre
#: 1 s si 10 s pe statie e diferenta dintre o sesiune si o dupa-amiaza.
FRAMES_PER_STATION = 30

#: Protocolul implicit, in metri. Acopera plaja din §2: de la plafonul de
#: handover (12 m) pana sub pragul de incadrare in cadru (0.38 m, §5.2).
#: Ultimele doua sunt sub prag DELIBERAT: acolo detectia TREBUIE sa dispara,
#: iar o statie care detecteaza la 0.30 m inseamna ca verificarea de
#: incadrare nu functioneaza.
DEFAULT_STATIONS = (12.0, 10.0, 8.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.3)

CONDITII_LUMINA = ('soare', 'nori', 'umbra', 'interior', 'amurg')

#: Temperatura: sysfs e calea portabila; vcgencmd da in plus starea de
#: throttling, care e informatia care chiar conteaza.
THERMAL_PATH = '/sys/class/thermal/thermal_zone0/temp'

STATION_HEADER = [
    'statie', 'lumina', 'distanta_masurata_m', 'cadre', 'detectate',
    'rata_detectie', 'distanta_estimata_mediana_m', 'eroare_rel',
    'marker_px_median', 'lat_p50_ms', 'lat_p99_ms',
    'temp_start_c', 'temp_max_c', 'nota',
]

FRAME_HEADER = [
    'cadru', 'fisier', 'detectat', 'distanta_m', 'range_m', 'marker_px',
    'angle_x', 'angle_y', 'latenta_ms', 'temp_c',
]


# --- mediu ------------------------------------------------------------------

def cpu_temp_c(path=THERMAL_PATH):
    """Temperatura CPU in Celsius, sau None daca nu se poate citi (desktop)."""
    try:
        with open(path) as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def throttled_state():
    """`vcgencmd get_throttled`, decodat. None in afara Pi-ului.

    Bitii care conteaza pentru E2: 0 = under-voltage ACUM, 1 = frecventa
    limitata acum, 2 = throttled acum; 16/17/18 = acelasi lucru, s-a
    intamplat de la boot. Un Pi care a fost throttled chiar si o data in
    sesiune produce cifre de FPS care nu se pot compara cu restul."""
    import subprocess
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], timeout=2.0,
                             capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError):
        return None
    txt = (out.stdout or '').strip()
    if '=' not in txt:
        return None
    val = int(txt.split('=')[1], 0)
    biti = {0: 'subtensiune ACUM', 1: 'frecventa limitata ACUM',
            2: 'throttled ACUM', 3: 'limita soft de temperatura ACUM',
            16: 'subtensiune de la boot', 17: 'frecventa limitata de la boot',
            18: 'throttled de la boot', 19: 'limita soft de la boot'}
    return {'raw': val,
            'active': [t for b, t in biti.items() if val & (1 << b)]}


def environment_info():
    """Ce a fost adevarat la momentul masuratorii. Fara asta, un set de date
    de acum trei saptamani nu se poate interpreta."""
    info = {
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'host': platform.node(),
        'platform': platform.platform(),
        'python': platform.python_version(),
        'opencv': cv2.__version__,
        'numpy': np.__version__,
        'temp_c': cpu_temp_c(),
        'throttled': throttled_state(),
    }
    try:
        with open('/proc/device-tree/model') as f:
            info['model'] = f.read().strip('\x00').strip()
    except OSError:
        info['model'] = None
    return info


# --- statistica -------------------------------------------------------------

def _median(vals):
    return float(np.median(vals)) if len(vals) else None


def _percentile(vals, p):
    return float(np.percentile(vals, p)) if len(vals) else None


def summarize_station(rows, truth_m):
    """Rezumatul unei statii din randurile ei de cadru.

    Mediana, nu media: o singura detectie ratata partial (colt prins pe o
    umbra) trage media cu procente intregi, iar noi vrem cifra tipica, nu
    cea influentata de coada. Aceeasi alegere ca la percentilele de latenta
    din §5 (E1.4)."""
    det = [r for r in rows if r['detectat']]
    dist = [r['distanta_m'] for r in det]
    px = [r['marker_px'] for r in det]
    lat = [r['latenta_ms'] for r in rows if r['latenta_ms'] is not None]
    temps = [r['temp_c'] for r in rows if r['temp_c'] is not None]

    med = _median(dist)
    err = None
    if med is not None and truth_m:
        err = med / truth_m - 1.0
    return {
        'cadre': len(rows),
        'detectate': len(det),
        'rata_detectie': (len(det) / len(rows)) if rows else 0.0,
        'distanta_estimata_mediana_m': med,
        'eroare_rel': err,
        'marker_px_median': _median(px),
        'lat_p50_ms': _percentile(lat, 50),
        'lat_p99_ms': _percentile(lat, 99),
        'temp_start_c': temps[0] if temps else None,
        'temp_max_c': max(temps) if temps else None,
    }


def verdict(summary, truth_m, sub_prag):
    """Ce sa ii spunem operatorului, cat e inca pe scara.

    `sub_prag` schimba semnul verificarii: sub ~0.38 m markerul nu incape in
    cadru (§5.2), deci acolo ABSENTA detectiei e rezultatul corect, iar
    prezenta ei e bug."""
    r = summary['rata_detectie']
    if sub_prag:
        if r > 0.05:
            return ('PROBLEMA', f"detectie in {r:.0%} din cadre sub pragul de "
                                f"incadrare - verifica CameraModel.fits_in_frame")
        return ('OK', "fara detectie, cum trebuie sub prag")
    if r < 0.5:
        return ('PROBLEMA', f"rata de detectie {r:.0%}. Verifica focus "
                            f"(LensPosition), expunere, ID-ul markerului si "
                            f"daca markerul e intreg in cadru.")
    if r < 0.95:
        return ('ATENTIE', f"rata de detectie {r:.0%} - utilizabil, dar "
                           f"investigheaza inainte de zbor")
    e = summary['eroare_rel']
    if e is not None and abs(e) > 0.05:
        return ('PROBLEMA', f"eroare de distanta {e:+.1%} (criteriu E2: 5%). "
                            f"Verifica latura markerului si calibrarea.")
    if e is not None and abs(e) > 0.02:
        return ('ATENTIE', f"eroare de distanta {e:+.1%}")
    return ('OK', "")


# --- captura ----------------------------------------------------------------

def capture_station(source, detector, n_frames, out_dir, save_frames=True):
    """N cadre: salvare bruta + detectie + latenta + temperatura."""
    frames_dir = os.path.join(out_dir, 'frames')
    if save_frames:
        os.makedirs(frames_dir, exist_ok=True)
    rows = []
    for i in range(n_frames):
        item = source.read()
        if item is None:
            break
        gray, t_cap = item
        t0 = time.monotonic()
        det = detector.detect(gray, t_cap)
        lat_ms = 1000.0 * (time.monotonic() - t0)

        name = f"{i:04d}.png"
        if save_frames:
            cv2.imwrite(os.path.join(frames_dir, name), gray)
        rows.append({
            'cadru': i,
            'fisier': f"frames/{name}" if save_frames else '',
            'detectat': det is not None,
            'distanta_m': None if det is None else det.distance_m,
            'range_m': None if det is None else det.range_m,
            'marker_px': None if det is None else det.marker_px,
            'angle_x': None if det is None else det.angle_x,
            'angle_y': None if det is None else det.angle_y,
            'latenta_ms': lat_ms,
            'temp_c': cpu_temp_c(),
        })
    return rows


def write_frame_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FRAME_HEADER, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if r.get(k) is None else r.get(k))
                        for k in FRAME_HEADER})


# --- sesiunea -----------------------------------------------------------------

class E2Session:
    """Un director de iesire, un manifest, N statii."""

    def __init__(self, root=DATA_ROOT, stamp=None, lumina='necunoscut',
                 note=''):
        self.stamp = stamp or time.strftime('%Y%m%d-%H%M%S')
        self.dir = os.path.join(root, self.stamp)
        os.makedirs(self.dir, exist_ok=True)
        self.lumina = lumina
        self.stations = []
        self.manifest = {
            'sesiune': self.stamp,
            'lumina': lumina,
            'nota': note,
            'mediu': environment_info(),
        }

    def add_calibration(self, cal, path):
        self.manifest['calibrare'] = {
            'fisier': path, 'sursa': cal.source, 'rms_px': cal.rms,
            'n_images': cal.n_images, 'fx': cal.fx, 'fy': cal.fy,
            'cx': cal.cx, 'cy': cal.cy,
            'dist': [float(v) for v in np.asarray(cal.dist).reshape(-1)],
            'meta': cal.meta,
        }

    def add_config(self, cfg):
        self.manifest['config'] = {k: v for k, v in cfg.items()
                                   if not k.startswith('_')}

    def station_dir(self, truth_m, lumina):
        name = f"d{truth_m:05.2f}_{lumina}"
        d = os.path.join(self.dir, name)
        os.makedirs(d, exist_ok=True)
        return name, d

    def record(self, name, lumina, truth_m, rows, summary, nota=''):
        rec = {'statie': name, 'lumina': lumina,
               'distanta_masurata_m': truth_m, 'nota': nota}
        rec.update(summary)
        self.stations.append(rec)
        return rec

    def flush(self):
        """Manifest + rezumat, rescrise dupa FIECARE statie.

        Deliberat: o sesiune pe teren se intrerupe - bateria Pi-ului, un
        Ctrl-C, un cablu. Datele statiilor deja facute trebuie sa fie
        complete pe disc in orice moment, nu scrise la final."""
        with open(os.path.join(self.dir, 'manifest.json'), 'w') as f:
            json.dump(self.manifest, f, indent=2, ensure_ascii=False)
        with open(os.path.join(self.dir, 'stations.csv'), 'w',
                  newline='') as f:
            w = csv.DictWriter(f, fieldnames=STATION_HEADER,
                               extrasaction='ignore')
            w.writeheader()
            for s in self.stations:
                w.writerow({k: ('' if s.get(k) is None else s.get(k))
                            for k in STATION_HEADER})


# --- interactiv ---------------------------------------------------------------

def prompt_distance(input_fn, sugestie=None):
    """Distanta masurata, in metri. '' accepta sugestia, 'q' termina."""
    eticheta = (f"  Distanta masurata cu ruleta [m]"
                f"{f' (Enter = {sugestie:g})' if sugestie else ''}, "
                f"q = gata: ")
    while True:
        raw = (input_fn(eticheta) or '').strip().lower()
        if raw in ('q', 'quit', 'gata'):
            return None
        if not raw and sugestie:
            return float(sugestie)
        try:
            v = float(raw.replace(',', '.'))
        except ValueError:
            print("    numar, te rog (ex: 5.02)")
            continue
        if not 0.05 <= v <= 50:
            print("    valoare in afara plajei plauzibile (0.05-50 m)")
            continue
        return v


def report_station(rec, verd):
    stare, motiv = verd
    culoare = {'OK': '\033[32m', 'ATENTIE': '\033[33m',
               'PROBLEMA': '\033[31m'}.get(stare, '')
    r = rec['rata_detectie']
    px = rec['marker_px_median']
    est = rec['distanta_estimata_mediana_m']
    err = rec['eroare_rel']
    print(f"    detectie {r:.0%} ({rec['detectate']}/{rec['cadre']})"
          f" | marker {'-' if px is None else f'{px:.0f} px'}"
          f" | est {'-' if est is None else f'{est:.3f} m'}"
          f" | eroare {'-' if err is None else f'{err:+.2%}'}")
    print(f"    latenta p50 {_fmt(rec['lat_p50_ms'])} p99 "
          f"{_fmt(rec['lat_p99_ms'])} ms | temp "
          f"{_fmt(rec['temp_start_c'])} -> {_fmt(rec['temp_max_c'])} C")
    print(f"    {culoare}{stare}\033[0m{(' - ' + motiv) if motiv else ''}")


def _fmt(v, fmt='.0f'):
    return '-' if v is None else format(v, fmt)


def run_session(args, source_factory, input_fn=input):
    cfg = nova_config.load(args.config)
    cal_path = nova_config.resolve(cfg, 'camera_calibration')
    try:
        cal = CameraCalibration.load(cal_path, require_real=True)
    except (FileNotFoundError, ValueError) as e:
        print(f"\n  NU RULEZ: {e}\n")
        return 2

    detector = ArucoMarkerDetector(
        cal, marker_id=cfg['marker_id'], marker_size_m=cfg['marker_size_m'],
        # ROI OPRIT pentru E2: introduce dependenta de ordinea cadrelor si de
        # istoricul detectiilor, iar o statie trebuie sa fie reproductibila
        # independent de cele dinainte. Calea cu ROI se masoara separat.
        roi_below_m=0.0)

    ses = E2Session(root=args.out_root, lumina=args.lumina, note=args.nota)
    ses.add_calibration(cal, cal_path)
    ses.add_config(cfg)
    ses.flush()

    print(f"\n  sesiune: {ses.dir}")
    print(f"  calibrare: {cal}")
    mediu = ses.manifest['mediu']
    print(f"  {mediu['model'] or mediu['platform']} | cv2 {mediu['opencv']} | "
          f"temp {_fmt(mediu['temp_c'], '.1f')} C")
    thr = mediu.get('throttled')
    if thr and thr['active']:
        print(f"  \033[33mATENTIE throttling: {', '.join(thr['active'])}\033[0m")
    print(f"  marker ID {cfg['marker_id']}, {cfg['marker_size_m']} m | "
          f"lumina: {args.lumina}\n")

    sugestii = list(args.stations)
    n = 0
    while True:
        sug = sugestii[n] if n < len(sugestii) else None
        print(f"  --- statia {n + 1} ---")
        truth = prompt_distance(input_fn, sug)
        if truth is None:
            break
        name, sdir = ses.station_dir(truth, args.lumina)
        print(f"    captez {args.frames} cadre...")
        source = source_factory()
        try:
            rows = capture_station(source, detector, args.frames, sdir,
                                   save_frames=not args.no_frames)
        finally:
            source.close()
        if not rows:
            print("    \033[31mniciun cadru citit - camera?\033[0m")
            continue
        write_frame_csv(os.path.join(sdir, 'detections.csv'), rows)
        summary = summarize_station(rows, truth)
        sub_prag = truth < args.prag_incadrare
        verd = verdict(summary, truth, sub_prag)
        rec = ses.record(name, args.lumina, truth, rows, summary,
                         nota=verd[0])
        ses.flush()
        report_station(rec, verd)
        print()
        n += 1

    ses.flush()
    print(f"\n  {len(ses.stations)} statii scrise in {ses.dir}")
    print(f"  Adu-le pe desktop:\n"
          f"    rsync -av {mediu['host']}:{ses.dir}/ "
          f"~/nova-zdc/data/e2/{ses.stamp}/\n")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default=None)
    p.add_argument('--lumina', default='necunoscut', metavar='CONDITIE',
                   help=f"una din: {', '.join(CONDITII_LUMINA)}")
    p.add_argument('--nota', default='', help='text liber in manifest')
    p.add_argument('--frames', type=int, default=FRAMES_PER_STATION)
    p.add_argument('--stations', default=None,
                   help='distante sugerate, separate prin virgula')
    p.add_argument('--out-root', default=DATA_ROOT)
    p.add_argument('--no-frames', action='store_true',
                   help='nu salva imaginile brute (NU se recomanda)')
    p.add_argument('--prag-incadrare', type=float, default=0.38,
                   help='sub aceasta distanta absenta detectiei e corecta')
    p.add_argument('--images', default=None,
                   help='director de imagini in loc de camera (reluare)')
    a = p.parse_args(argv)

    if a.stations:
        a.stations = [float(x) for x in a.stations.replace(' ', '').split(',')
                      if x]
    else:
        a.stations = list(DEFAULT_STATIONS)

    if a.images:
        from nova.detector_pi import ImageDirSource
        def source_factory():
            return ImageDirSource(a.images)
    else:
        def source_factory():
            from nova.detector_pi import PiCameraSource
            return PiCameraSource(verbose=False)

    try:
        return run_session(a, source_factory)
    except KeyboardInterrupt:
        print("\n  intrerupt - statiile facute sunt deja scrise pe disc\n")
        return 1
    except ModuleNotFoundError as e:
        print(f"\n  NU RULEZ: {e} - picamera2 exista doar pe Pi.\n")
        return 2


if __name__ == '__main__':
    sys.exit(main())
