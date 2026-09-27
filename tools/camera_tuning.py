#!/usr/bin/env python3
"""
NOVA - ZDC 2026
The camera tuning file, kept in the repo (27.09.2026).

    # on the Pi, once (and after every libcamera upgrade you decide to take):
    ~/nova-venv/bin/python tools/camera_tuning.py save
    git add config/tuning && git commit -m "Tuning camera din libcamera X.Y"

    # any time (preflight material): is the system file still the one copied?
    ~/nova-venv/bin/python tools/camera_tuning.py check

`exposure: auto_capped` needs the sensor's tuning (the AGC exposure modes)
to add its capped 'custom' mode. Read from /usr/share/libcamera it would
change under us with an `apt upgrade`, silently. So the file is COPIED into
config/tuning/<model>.json, with a sidecar <model>.json.meta.json (source
path, sha256, libcamera package version, date), and nova/detector_pi.py
loads only the repo copy (camera key `tuning_file`). Nothing here writes to
/usr/share.
"""

import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST_DIR = os.path.join(REPO, 'config', 'tuning')
#: Where libcamera installs the Raspberry Pi tuning files (Pi 4 = vc4,
#: Pi 5 = pisp), in the order picamera2 searches them.
SYSTEM_DIRS = ('/usr/local/share/libcamera/ipa/rpi', '/usr/share/libcamera/ipa/rpi')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def system_file(model, platform='vc4', dirs=SYSTEM_DIRS):
    for d in dirs:
        p = os.path.join(d, platform, f"{model}.json")
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        f"{model}.json nu e in {', '.join(os.path.join(d, platform) for d in dirs)}")


def libcamera_version():
    """The installed libcamera package version, or None (not a Debian Pi)."""
    try:
        out = subprocess.run(
            ['dpkg-query', '-W', '-f=${Package} ${Version}\\n'],
            capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    lines = [ln for ln in out.splitlines()
             if ln.startswith(('libcamera0', 'libcamera-ipa'))]
    return '; '.join(lines) or None


def save(model, platform='vc4', dest_dir=DEST_DIR, dirs=SYSTEM_DIRS):
    """Copy the system tuning into the repo, with its sidecar. Returns the
    destination path. The copy is byte for byte: libcamera parses it."""
    src = system_file(model, platform, dirs)
    with open(src) as f:
        json.load(f)                      # refuse a file that is not JSON
    os.makedirs(dest_dir, exist_ok=True)
    dst = os.path.join(dest_dir, f"{model}.json")
    shutil.copyfile(src, dst)
    meta = {'model': model, 'platform': platform, 'source': src,
            'sha256': sha256(dst), 'libcamera': libcamera_version(),
            'saved_at': datetime.datetime.now().isoformat(timespec='seconds')}
    with open(dst + '.meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return dst


def check(model, platform='vc4', dest_dir=DEST_DIR, dirs=SYSTEM_DIRS):
    """(ok, message): the repo copy exists, is intact, and the system file
    is still the one it was copied from."""
    dst = os.path.join(dest_dir, f"{model}.json")
    mp = dst + '.meta.json'
    if not os.path.exists(dst) or not os.path.exists(mp):
        return False, f"{dst} lipseste: tools/camera_tuning.py save (pe Pi)"
    with open(mp) as f:
        meta = json.load(f)
    if sha256(dst) != meta.get('sha256'):
        return False, f"{dst} a fost modificat dupa copiere (sha256 diferit)"
    try:
        src = system_file(model, platform, dirs)
    except FileNotFoundError as e:
        return True, f"copia din repo e intacta; fisierul de sistem nu se vede ({e})"
    if sha256(src) != meta['sha256']:
        return False, (f"{src} s-a schimbat fata de copia din repo (apt upgrade? "
                       f"copiat din: {meta.get('libcamera')}, acum: "
                       f"{libcamera_version()}). Camera foloseste copia din repo; "
                       f"decide daca o reimprospatezi (save + commit).")
    return True, f"copia din repo = fisierul de sistem ({meta['sha256'][:12]})"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', choices=('save', 'check'))
    p.add_argument('--model', default=None,
                   help='senzorul (implicit: cel conectat, din picamera2; '
                        'Camera Module 3 Wide = imx708_wide)')
    p.add_argument('--platform', default='vc4', help='vc4 = Pi 4, pisp = Pi 5')
    a = p.parse_args(argv)
    model = a.model
    if model is None:
        try:
            from picamera2 import Picamera2
            model = (Picamera2.global_camera_info() or [{}])[0].get('Model')
        except Exception:                                   # noqa: BLE001
            model = None
        if not model:
            p.error('nu stiu senzorul: da --model (ex. imx708_wide)')
    if a.command == 'save':
        dst = save(model, a.platform)
        print(f"[tuning] copiat in {dst} (+ .meta.json). Fa commit la config/tuning/.")
        return 0
    ok, msg = check(model, a.platform)
    print(f"[tuning] {'OK' if ok else 'ATENTIE'}: {msg}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
