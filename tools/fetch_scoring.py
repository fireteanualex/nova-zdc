#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Bring the touchdown captures (8.3.3) from the Pi to the PC and build the
handover package (6.2.1.30). Runs on Windows and Linux: Python 3 standard
library + the system OpenSSH client (`ssh`). numpy/OpenCV are optional.

    python3 tools/fetch_scoring.py list
    python3 tools/fetch_scoring.py fetch
    python3 tools/fetch_scoring.py fetch --watch 5 --cursa cursa1
    python3 tools/fetch_scoring.py fetch --session s20260927-140312 --fc-log 00000042.BIN
    python3 tools/fetch_scoring.py verify data/scoring_pc/handover/20260927_cursa1

Contract: docs/SCORING_FORMAT.md. On the Pi each touchdown is one flat
directory, ~/nova-zdc/scoring/Handoff<n>-touchdown/, complete only when it
has MANIFEST.sha256 (written last). Incomplete ones are listed and ignored.

Local layout (root: --local, default <repo>/data/scoring_pc):

    Handoff<n>-touchdown/                 verified copy, byte-identical to the Pi
        Handoff<n>-touchdown.png          the touchdown image
    .partial/Handoff<n>-touchdown/        download in progress (never trusted)
    carantina/Handoff<n>-touchdown/       failed sha256 check + CARANTINA.txt
    handover/<date>_<cursa>/              the package handed to the jury, one
                                          per Pi session (date/session come
                                          from each capture's meta.json)
        README.txt                        race summary
        MANIFEST_HANDOVER.sha256          every file of the package (incl. fc/)
        fc/<log>.bin                      FC DataFlash log(s), via --fc-log
        Handoff<n>-touchdown/             Pi files + README.txt (+ regenerated
                                          annotated copy, if the Pi had none)

A capture counts as received only once Handoff<n>-touchdown/ exists locally,
and that directory appears only by an atomic rename after every file matched
the manifest. So a second fetch downloads nothing, and an interrupted or
corrupt download is never mistaken for a received one. If the Pi restarts
its numbering (scoring/ emptied), a new Handoff<n> would carry the name of
one already received: the remote manifest's sha256 is compared with the
local one and a mismatch is reported, never overwritten.

Nothing is ever written or deleted on the Pi: the only remote commands are
`find`, `tar -c` to stdout and `cat` (all read-only).
"""

import argparse
import collections
import datetime
import hashlib
import io
import json
import os
import re
import shlex
import shutil
import stat
import struct
import subprocess
import sys
import tarfile
import time
import zlib

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_HOST = 'nova@100.97.236.82'
DEFAULT_REMOTE_ROOT = '~/nova-zdc/scoring'
DEFAULT_LOCAL = os.path.join(REPO_ROOT, 'data', 'scoring_pc')

MANIFEST = 'MANIFEST.sha256'
HANDOVER_MANIFEST = 'MANIFEST_HANDOVER.sha256'
README = 'README.txt'
QUARANTINE_NOTE = 'CARANTINA.txt'
INDEX_FILE = '_index_sesiuni.json'
BUILDING_PREFIX = '.building_'
NO_ROOT_MARK = '__NOVA_NO_ROOT__'

NAME_DATE = re.compile(r'^\d{8}$')
NAME_SESSION = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
NAME_HANDOFF = re.compile(r'^Handoff(\d+)-touchdown$')
#: `sha256sum` output for a remote manifest (absolute or relative path).
SUM_LINE = re.compile(r'^([0-9a-fA-F]{64}) [ *](?:.*/)?(Handoff\d+-touchdown)/'
                      + re.escape('MANIFEST.sha256') + r'$')
UNKNOWN_KEY = 'necunoscut/necunoscut'
MANIFEST_LINE = re.compile(r'^([0-9a-fA-F]{64}) [ *](.+)$')

#: How long a verification failure is left alone in --watch before the
#: attempt is downloaded again (a corrupt file on the Pi would otherwise be
#: pulled over the radio link every cycle).
WATCH_RETRY_BAD_S = 60.0

# Optional numpy/OpenCV backend. The pure-Python PNG path is the fallback
# and gives the same numbers (same luminance formula as cv2).
try:                                                    # pragma: no cover
    import cv2 as _cv2
    import numpy as _np
except Exception:                                       # noqa: BLE001
    _cv2 = None
    _np = None
_USE_CV2 = _cv2 is not None


def set_backend(use_cv2):
    """Force the pure-Python image path (False) or cv2 when importable."""
    global _USE_CV2
    _USE_CV2 = bool(use_cv2) and _cv2 is not None
    return _USE_CV2


def backend_name():
    return 'cv2' if _USE_CV2 else 'python'


class TransportError(Exception):
    """Transient: link down, ssh failed, timeout. Retried in --watch."""


class SetupError(Exception):
    """Not transient: ssh missing, bad arguments."""


class PngError(Exception):
    pass


# --- attempts ---------------------------------------------------------------

class AttemptRef(collections.namedtuple('AttemptRef', 'n complete manifest_sha')):
    """One touchdown capture. `manifest_sha`: sha256 of its MANIFEST.sha256
    as seen at the source (None when unknown or incomplete)."""
    __slots__ = ()

    def __new__(cls, n, complete, manifest_sha=None):
        return super().__new__(cls, n, complete, manifest_sha)

    @property
    def name(self):
        return f"Handoff{self.n}-touchdown"

    @property
    def rel(self):
        return self.name


def _sorted_refs(refs):
    return sorted(refs, key=lambda r: r.n)


def handoff_n(name):
    m = NAME_HANDOFF.match(name)
    return int(m.group(1)) if m else None


def parse_find_listing(text):
    """Parse the Pi listing: `find ROOT -mindepth 1 -maxdepth 2 -printf
    '%y %P\\n'` followed by `sha256sum` of every Handoff*/MANIFEST.sha256.

    Returns (refs, ignored): Handoff<n>-touchdown directories at depth 1,
    complete when their MANIFEST.sha256 is listed. Other names at depth 1
    are ignored (and reported), which also keeps every path used later free
    of shell syntax."""
    found, sums, ignored = {}, {}, []
    for line in text.splitlines():
        line = line.rstrip('\r')
        if not line.strip():
            continue
        m = SUM_LINE.match(line)
        if m:
            n = handoff_n(m.group(2))
            if n is not None:
                sums[n] = m.group(1).lower()
            continue
        kind, _, rel = line.partition(' ')
        parts = rel.split('/')
        if len(parts) == 1:
            n = handoff_n(parts[0])
            if n is None:
                ignored.append(rel)
            else:
                found.setdefault(n, False)
        elif len(parts) == 2 and parts[1] == MANIFEST and kind == 'f':
            n = handoff_n(parts[0])
            if n is not None:
                found[n] = True
    refs = [AttemptRef(n, c, sums.get(n) if c else None) for n, c in found.items()]
    return _sorted_refs(refs), ignored


# --- files, hashes, manifests ----------------------------------------------

def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def safe_rel(rel):
    """A relative '/'-path that cannot leave its directory on any OS."""
    if not rel or rel.startswith('/') or '\\' in rel or ':' in rel:
        return False
    parts = rel.split('/')
    return all(p not in ('', '.', '..') for p in parts)


def list_files(root):
    """Every regular file under root, as sorted '/'-relative paths."""
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        for fn in filenames:
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, root).replace(os.sep, '/')
            out.append(rel)
    return sorted(out)


def parse_manifest(text):
    """sha256sum format -> ({rel: hex}, [problems])."""
    entries, problems = {}, []
    for i, line in enumerate(text.splitlines(), 1):
        line = line.rstrip('\r')
        if not line.strip():
            continue
        m = MANIFEST_LINE.match(line)
        if not m:
            problems.append(f"manifest linia {i} nu e in format sha256sum")
            continue
        h, rel = m.group(1).lower(), m.group(2)
        if not safe_rel(rel):
            problems.append(f"manifest linia {i}: cale nesigura '{rel}'")
        elif rel in entries:
            problems.append(f"manifest: '{rel}' apare de doua ori")
        else:
            entries[rel] = h
    if not entries and not problems:
        problems.append("manifest gol")
    return entries, problems


def verify_dir(d, manifest_name=MANIFEST, strict=True, exclude=()):
    """Check every file listed in the manifest.

    Returns (problems, extras, n_checked). `extras` are files present but not
    listed; with strict=True they are problems too (the Pi's manifest covers
    every file of the attempt, so an unlisted file is an unverified one)."""
    path = os.path.join(d, manifest_name)
    if not os.path.isfile(path):
        return [f"lipseste {manifest_name}"], [], 0
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        entries, problems = parse_manifest(f.read())
    for rel, h in sorted(entries.items()):
        p = os.path.join(d, *rel.split('/'))
        if not os.path.isfile(p):
            problems.append(f"lipseste {rel}")
        elif sha256_file(p) != h:
            problems.append(f"sha256 diferit: {rel}")
    skip = {manifest_name} | set(exclude)
    extras = [r for r in list_files(d) if r not in entries and r not in skip]
    if strict:
        problems += [f"fisier neacoperit de manifest: {r}" for r in extras]
    return problems, extras, len(entries)


def manifest_text(root, exclude=()):
    lines = []
    for rel in list_files(root):
        if (rel in exclude or rel.split('/')[0].startswith(BUILDING_PREFIX)
                or rel.endswith(('.part', '.tmp'))):
            continue
        lines.append(f"{sha256_file(os.path.join(root, *rel.split('/')))}  {rel}\n")
    return ''.join(lines)


def write_text_if_changed(path, text):
    """LF line endings on every OS (the package hashes must not depend on
    which PC built it). Returns True when the file was (re)written."""
    data = text.encode('utf-8')
    try:
        with open(path, 'rb') as f:
            if f.read() == data:
                return False
    except OSError:
        pass
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(data)
    os.replace(tmp, path)
    return True


def _rmtree(path):
    """Local cleanup only (our own staging dirs). Handles read-only files on
    Windows."""
    def retry(func, p, _exc):
        try:
            os.chmod(p, stat.S_IWRITE)
            func(p)
        except OSError:
            pass
    if os.path.isdir(path):
        if sys.version_info >= (3, 12):
            shutil.rmtree(path, onexc=retry)
        else:
            shutil.rmtree(path, onerror=retry)


# --- PNG without numpy ------------------------------------------------------

PNG_SIG = b'\x89PNG\r\n\x1a\n'
_PNG_CHANNELS = {0: 1, 2: 3, 4: 2, 6: 4}             # colour type -> channels
_PNG_CTYPE = {v: k for k, v in _PNG_CHANNELS.items()}


class Png(object):
    def __init__(self, w, h, channels, rows):
        self.w, self.h, self.channels, self.rows = w, h, channels, rows


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def _unfilter(ftype, line, prev, bpp):
    n = len(line)
    if ftype == 0:
        pass
    elif ftype == 1:
        for i in range(bpp, n):
            line[i] = (line[i] + line[i - bpp]) & 0xFF
    elif ftype == 2:
        for i in range(n):
            line[i] = (line[i] + prev[i]) & 0xFF
    elif ftype == 3:
        for i in range(n):
            a = line[i - bpp] if i >= bpp else 0
            line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
    elif ftype == 4:
        for i in range(n):
            if i >= bpp:
                line[i] = (line[i] + _paeth(line[i - bpp], prev[i],
                                            prev[i - bpp])) & 0xFF
            else:
                line[i] = (line[i] + prev[i]) & 0xFF
    else:
        raise PngError(f"filtru PNG necunoscut {ftype}")
    return line


def _filter(ftype, row, prev, bpp):
    n = len(row)
    out = bytearray(n)
    for i in range(n):
        a = row[i - bpp] if i >= bpp else 0
        b = prev[i]
        c = prev[i - bpp] if i >= bpp else 0
        pred = (0, a, b, (a + b) >> 1, _paeth(a, b, c))[ftype]
        out[i] = (row[i] - pred) & 0xFF
    return out


def decode_png(path, max_rows=None):
    """8-bit, non-interlaced gray / gray+alpha / RGB / RGBA PNG -> Png.

    Rows are bytearrays of w*channels bytes, RGB order. `max_rows` stops
    early (the centre verdict needs only the top half). Anything else
    (16-bit, palette, interlaced, bad CRC) raises PngError."""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] != PNG_SIG:
        raise PngError("nu e PNG")
    pos, idat, ihdr = 8, [], None
    while pos + 12 <= len(data):
        (length,) = struct.unpack('>I', data[pos:pos + 4])
        ctype = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        (crc,) = struct.unpack('>I', data[pos + 8 + length:pos + 12 + length])
        if zlib.crc32(ctype + body) & 0xFFFFFFFF != crc:
            raise PngError(f"CRC gresit in {ctype!r}")
        pos += 12 + length
        if ctype == b'IHDR':
            ihdr = struct.unpack('>IIBBBBB', body)
        elif ctype == b'IDAT':
            idat.append(body)
        elif ctype == b'IEND':
            break
    if ihdr is None or not idat:
        raise PngError("PNG fara IHDR/IDAT")
    w, h, depth, color, _comp, _filt, interlace = ihdr
    if depth != 8 or color not in _PNG_CHANNELS or interlace != 0:
        raise PngError(f"PNG nesuportat fara OpenCV (adancime {depth}, "
                       f"tip culoare {color}, interlace {interlace})")
    ch = _PNG_CHANNELS[color]
    stride = w * ch
    raw = zlib.decompress(b''.join(idat))
    n_rows = h if max_rows is None else min(h, max_rows)
    if len(raw) < n_rows * (stride + 1):
        raise PngError("date PNG trunchiate")
    rows, prev = [], bytearray(stride)
    for y in range(n_rows):
        i = y * (stride + 1)
        line = _unfilter(raw[i], bytearray(raw[i + 1:i + 1 + stride]), prev, ch)
        rows.append(line)
        prev = line
    return Png(w, h, ch, rows)


def encode_png(path, w, h, channels, rows, filters=None):
    """Write an 8-bit PNG. `filters` (list of filter types, cycled per row)
    exists for tests; default is filter 0."""
    bpp = channels
    raw = bytearray()
    prev = bytes(w * bpp)
    for y in range(h):
        row = bytes(rows[y])
        if len(row) != w * bpp:
            raise ValueError(f"randul {y}: {len(row)} octeti, astept {w * bpp}")
        f = filters[y % len(filters)] if filters else 0
        raw.append(f)
        raw += row if f == 0 else _filter(f, row, prev, bpp)
        prev = row

    def chunk(t, body):
        return (struct.pack('>I', len(body)) + t + body
                + struct.pack('>I', zlib.crc32(t + body) & 0xFFFFFFFF))
    ihdr = struct.pack('>IIBBBBB', w, h, 8, _PNG_CTYPE[channels], 0, 0, 0)
    data = (PNG_SIG + chunk(b'IHDR', ihdr) + chunk(b'IDAT', zlib.compress(bytes(raw), 6))
            + chunk(b'IEND', b''))
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(data)
    os.replace(tmp, path)


def _gray(row, x, ch):
    """Luminance exactly as cv2.cvtColor(..., COLOR_RGB2GRAY) on 8 bit
    (0.299/0.587/0.114 in 15-bit fixed point, checked against OpenCV).
    Other OpenCV builds may round differently by 1 level, far below the
    classification margins."""
    i = x * ch
    if ch <= 2:
        return row[i]
    r, g, b = row[i], row[i + 1], row[i + 2]
    return (r * 9798 + g * 19235 + b * 3735 + 16384) >> 15


# --- centre verdict and annotated copy -------------------------------------

def classify_center(mean, lo, hi):
    """docs/SCORING_FORMAT.md: 5x5 window on gray, informative only."""
    if mean < 80 and hi - lo < 60:
        return 'negru'
    if mean > 170 and hi - lo < 60:
        return 'alb'
    return 'ambiguu'


def _window(w, h):
    cx, cy = w // 2, h // 2
    return cx, cy, max(0, cx - 2), min(w, cx + 3), max(0, cy - 2), min(h, cy + 3)


def _center_values_cv2(path):
    img = _cv2.imread(path, _cv2.IMREAD_UNCHANGED)
    if img is None:
        raise PngError("OpenCV nu poate citi imaginea")
    if img.dtype != _np.uint8:
        raise PngError(f"imagine {img.dtype}, astept 8 biti")
    if img.ndim == 3:
        code = _cv2.COLOR_BGRA2GRAY if img.shape[2] == 4 else _cv2.COLOR_BGR2GRAY
        img = _cv2.cvtColor(img, code)
    h, w = img.shape[:2]
    cx, cy, x0, x1, y0, y1 = _window(w, h)
    return [int(v) for v in img[y0:y1, x0:x1].ravel()], cx, cy, w, h


def _center_values_python(path):
    # Only the rows down to the window's bottom edge are decoded.
    with open(path, 'rb') as f:
        head = f.read(24)
    if head[:8] != PNG_SIG or head[12:16] != b'IHDR':
        raise PngError("nu e PNG")
    w, h = struct.unpack('>II', head[16:24])
    cx, cy, x0, x1, y0, y1 = _window(w, h)
    png = decode_png(path, max_rows=y1)
    vals = [_gray(png.rows[y], x, png.channels)
            for y in range(y0, y1) for x in range(x0, x1)]
    return vals, cx, cy, w, h


def center_verdict(path):
    """Never raises. {'class': 'negru'|'alb'|'ambiguu'|None, ...}; with
    class None, 'error' says why ("verdict indisponibil")."""
    try:
        if not os.path.isfile(path):
            raise PngError(f"nu exista {os.path.basename(path)}")
        fn = _center_values_cv2 if _USE_CV2 else _center_values_python
        return _verdict(*fn(path))
    except Exception as e:                              # noqa: BLE001
        return {'class': None, 'error': f"{type(e).__name__}: {e}",
                'backend': backend_name()}


def _verdict(vals, cx, cy, w, h):
    if not vals:
        return {'class': None, 'error': 'imagine goala', 'backend': backend_name()}
    mean = sum(vals) / float(len(vals))
    lo, hi = min(vals), max(vals)
    return {'class': classify_center(mean, lo, hi), 'mean': round(mean, 1),
            'min': lo, 'max': hi, 'patch_px': len(vals), 'cx': cx, 'cy': cy,
            'size': [w, h], 'backend': backend_name()}


def cross_pixels(w, h):
    """A fine 1-px cross at (w//2, h//2) with the centre left clear, so the
    5x5 window being judged is never covered by the annotation."""
    cx, cy = w // 2, h // 2
    arm = max(8, min(w, h) // 20)
    gap = 4
    pts = []
    for d in range(gap, arm + 1):
        for x, y in ((cx - d, cy), (cx + d, cy), (cx, cy - d), (cx, cy + d)):
            if 0 <= x < w and 0 <= y < h:
                pts.append((x, y))
    return pts


def annotate_center(src, dst):
    """Regenerate touchdown_annotated.png (red cross, RGB). Never raises:
    returns (ok, message)."""
    try:
        if _USE_CV2:
            img = _cv2.imread(src, _cv2.IMREAD_UNCHANGED)
            if img is None or img.dtype != _np.uint8:
                raise PngError("OpenCV nu poate citi imaginea pe 8 biti")
            if img.ndim == 2:
                img = _cv2.cvtColor(img, _cv2.COLOR_GRAY2BGR)
            elif img.shape[2] == 4:
                img = _cv2.cvtColor(img, _cv2.COLOR_BGRA2BGR)
            h, w = img.shape[:2]
            for x, y in cross_pixels(w, h):
                img[y, x] = (0, 0, 255)
            tmp = dst + '.tmp.png'
            if not _cv2.imwrite(tmp, img):
                raise PngError("cv2.imwrite a esuat")
            os.replace(tmp, dst)
        else:
            png = decode_png(src)
            rows = []
            for row in png.rows:
                if png.channels == 3:
                    rows.append(bytearray(row))
                    continue
                out = bytearray(png.w * 3)
                for x in range(png.w):
                    i = x * png.channels
                    if png.channels <= 2:
                        out[3 * x:3 * x + 3] = bytes((row[i],)) * 3
                    else:
                        out[3 * x:3 * x + 3] = row[i:i + 3]
                rows.append(out)
            for x, y in cross_pixels(png.w, png.h):
                rows[y][3 * x:3 * x + 3] = b'\xff\x00\x00'
            encode_png(dst, png.w, png.h, 3, rows)
        return True, f"regenerata ({backend_name()})"
    except Exception as e:                              # noqa: BLE001
        return False, f"nu am putut regenera: {type(e).__name__}: {e}"


# --- sources: local directory (tests) and SSH (the Pi) ----------------------

class LocalSource(object):
    """A directory with the Pi's layout (tests, or a copy on a USB stick)."""

    def __init__(self, root):
        self.root = root
        self.ignored = []

    def describe(self):
        return f"director local {self.root}"

    def list_attempts(self):
        if not os.path.isdir(self.root):
            raise TransportError(f"nu exista directorul sursa {self.root}")
        refs, self.ignored = [], []
        for name in sorted(os.listdir(self.root)):
            ap = os.path.join(self.root, name)
            if not os.path.isdir(ap):
                continue
            n = handoff_n(name)
            if n is None:
                self.ignored.append(name)
                continue
            man = os.path.join(ap, MANIFEST)
            complete = os.path.isfile(man)
            refs.append(AttemptRef(n, complete, sha256_file(man) if complete else None))
        return _sorted_refs(refs)

    def download(self, ref, dest):
        src = os.path.join(self.root, ref.name)
        try:
            shutil.copytree(src, dest)
        except (OSError, shutil.Error) as e:
            raise TransportError(f"copierea din {src} a esuat: {e}")


def remote_path_expr(path):
    """Quote a remote path for the Pi's POSIX shell; '~/' is expanded there
    via "$HOME" (a quoted '~' would not expand)."""
    if not path or any(c in path for c in '\n\r\0'):
        raise SetupError(f"cale remote invalida: {path!r}")
    if path == '~' or path == '~/':
        return '"$HOME"'
    if path.startswith('~/'):
        rest = path[2:].rstrip('/')
        return '"$HOME"/' + shlex.quote(rest) if rest else '"$HOME"'
    if path.startswith('~'):
        raise SetupError("cale remote: foloseste ~/... sau o cale absoluta")
    return shlex.quote(path.rstrip('/') or '/')


def subprocess_runner(argv, timeout):
    """(rc, stdout, stderr) of a local executable. No shell, so no local
    quoting issues on Windows or Linux."""
    try:
        p = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, timeout=timeout)
    except FileNotFoundError:
        raise SetupError(
            f"'{argv[0]}' nu e instalat. Windows 10+: Settings > Apps > Optional "
            "features > OpenSSH Client; Linux: pachetul openssh-client")
    except subprocess.TimeoutExpired:
        raise TransportError(f"timeout dupa {timeout:.0f} s")
    return p.returncode, p.stdout, p.stderr


def extract_tar(data, top, dest):
    """Unpack `tar -cf - attempt_<n>` into dest (= the attempt directory).
    Only regular files and directories under `top/` with safe names; anything
    else is skipped and then shows up as missing in the manifest check."""
    skipped = []
    os.makedirs(dest)
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:') as tf:
        for m in tf:
            name = m.name[2:] if m.name.startswith('./') else m.name
            name = name.rstrip('/')
            if name == top:
                continue
            if not name.startswith(top + '/') or not safe_rel(name):
                skipped.append(m.name)
                continue
            rel = name[len(top) + 1:]
            target = os.path.join(dest, *rel.split('/'))
            if m.isdir():
                os.makedirs(target, exist_ok=True)
            elif m.isfile():
                os.makedirs(os.path.dirname(target), exist_ok=True)
                with tf.extractfile(m) as src, open(target, 'wb') as out:
                    shutil.copyfileobj(src, out)
            else:
                skipped.append(m.name)
    return skipped


class SshSource(object):
    """The Pi over the system OpenSSH client. Read-only remote commands:
    `find` (listing), `tar -c ... -f -` to stdout (download, one connection
    per attempt), `cat` (fallback when the Pi has no tar)."""

    def __init__(self, host=DEFAULT_HOST, remote_root=DEFAULT_REMOTE_ROOT,
                 runner=None, ssh='ssh', ssh_args=(), timeout_list=30.0,
                 timeout_get=600.0):
        if not host or host.startswith('-') or any(c.isspace() for c in host):
            raise SetupError(f"host invalid: {host!r}")
        self.host = host
        self.remote_root = remote_root
        self.root_expr = remote_path_expr(remote_root)
        self.runner = runner or subprocess_runner
        self.ssh = ssh
        self.ssh_args = list(ssh_args)
        self.timeout_list = timeout_list
        self.timeout_get = timeout_get
        self.ignored = []

    def describe(self):
        return f"Pi {self.host}:{self.remote_root}"

    def argv(self, remote_cmd):
        # BatchMode: never prompt for a password (would hang --watch).
        return ([self.ssh, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10']
                + self.ssh_args + ['--', self.host, remote_cmd])

    def _run(self, remote_cmd, timeout):
        rc, out, err = self.runner(self.argv(remote_cmd), timeout)
        if rc == 255:
            msg = err.decode('utf-8', 'replace').strip().splitlines()
            raise TransportError(f"ssh {self.host}: "
                                 + (msg[-1] if msg else 'conexiune esuata'))
        return rc, out, err

    def list_cmd(self):
        r = self.root_expr
        return (f"if [ -d {r} ]; then find {r} -mindepth 1 -maxdepth 2 "
                f"-printf '%y %P\\n'; find {r} -mindepth 2 -maxdepth 2 "
                f"-name {MANIFEST} -exec sha256sum {{}} +; "
                f"else echo {NO_ROOT_MARK}; fi")

    def attempt_expr(self, ref):
        return f"{self.root_expr}/{shlex.quote(ref.name)}"

    def tar_cmd(self, ref):
        return f"tar -C {self.root_expr} -cf - {shlex.quote(ref.name)}"

    def list_attempts(self):
        rc, out, err = self._run(self.list_cmd(), self.timeout_list)
        text = out.decode('utf-8', 'replace')
        if rc != 0:
            raise TransportError(f"listarea pe Pi a esuat (rc {rc}): "
                                 f"{err.decode('utf-8', 'replace').strip()}")
        if text.strip() == NO_ROOT_MARK:
            self.ignored = []
            return []
        refs, self.ignored = parse_find_listing(text)
        return refs

    def download(self, ref, dest):
        rc, out, err = self._run(self.tar_cmd(ref), self.timeout_get)
        if rc == 0:
            try:
                skipped = extract_tar(out, ref.name, dest)
            except tarfile.TarError as e:
                raise TransportError(f"arhiva tar de la Pi e invalida: {e}")
            for name in skipped:
                print(f"  atentie: intrare tar ignorata: {name}")
            return
        if rc == 127:
            return self._download_cat(ref, dest)
        raise TransportError(f"tar pe Pi a esuat (rc {rc}): "
                             f"{err.decode('utf-8', 'replace').strip()}")

    def _download_cat(self, ref, dest):
        a = self.attempt_expr(ref)
        rc, out, err = self._run(f"find {a} -type f -printf '%P\\n'",
                                 self.timeout_list)
        if rc != 0:
            raise TransportError(f"listarea incercarii a esuat (rc {rc})")
        os.makedirs(dest)
        for rel in out.decode('utf-8', 'replace').splitlines():
            rel = rel.rstrip('\r')
            if not rel:
                continue
            if not safe_rel(rel):
                print(f"  atentie: fisier ignorat (nume nesigur): {rel!r}")
                continue
            rc, data, err = self._run(f"cat -- {a}/{shlex.quote(rel)}",
                                      self.timeout_get)
            if rc != 0:
                raise TransportError(f"cat {rel} a esuat (rc {rc})")
            target = os.path.join(dest, *rel.split('/'))
            os.makedirs(os.path.dirname(target), exist_ok=True)
            with open(target, 'wb') as f:
                f.write(data)


# --- local store ------------------------------------------------------------

class Store(object):
    def __init__(self, local_root=DEFAULT_LOCAL, handover_root=None):
        self.root = local_root
        self.mirror_root = local_root
        self.partial_root = os.path.join(local_root, '.partial')
        self.quar_root = os.path.join(local_root, 'carantina')
        self.handover_root = handover_root or os.path.join(local_root, 'handover')

    def mirror(self, ref):
        return os.path.join(self.mirror_root, ref.name)

    def partial(self, ref):
        return os.path.join(self.partial_root, ref.name)

    def quarantine(self, ref):
        return os.path.join(self.quar_root, ref.name)

    def is_received(self, ref):
        return os.path.isfile(os.path.join(self.mirror(ref), MANIFEST))

    def conflict(self, ref):
        """The source's Handoff<n> is not the one received under that name
        (the Pi restarted its numbering)."""
        if not (ref.manifest_sha and self.is_received(ref)):
            return False
        return sha256_file(os.path.join(self.mirror(ref), MANIFEST)) != ref.manifest_sha

    def local_attempts(self):
        if not os.path.isdir(self.mirror_root):
            return []
        refs = LocalSource(self.mirror_root).list_attempts()
        return [r for r in refs if r.complete]

    def session_key(self, ref):
        """'<date>/<session>' of a received capture, from its meta.json."""
        meta, _err = load_meta(self.mirror(ref))
        date, session = str(meta.get('date', '')), str(meta.get('session', ''))
        if NAME_DATE.match(date) and NAME_SESSION.match(session):
            return f"{date}/{session}"
        return UNKNOWN_KEY


def accept_attempt(source, ref, store):
    """Download into .partial/, verify, then atomically move to pi/ (ok) or
    to carantina/ (bad). Returns (ok, problems, n_files). TransportError
    propagates, leaving only a .partial/ directory behind."""
    stage = store.partial(ref)
    _rmtree(stage)
    os.makedirs(os.path.dirname(stage), exist_ok=True)
    source.download(ref, stage)
    problems, _extras, n = verify_dir(stage, MANIFEST, strict=True)
    if problems:
        q = store.quarantine(ref)
        _rmtree(q)
        os.makedirs(os.path.dirname(q), exist_ok=True)
        os.replace(stage, q)
        note = ("NU E PRIMITA. Verificarea sha256 fata de MANIFEST.sha256 a esuat;\n"
                "incercarea nu intra in pachetul de predare. Se descarca din nou la\n"
                "urmatorul fetch; daca eroarea ramane, fisierul e stricat pe Pi.\n\n"
                + ''.join(f"  {p}\n" for p in problems))
        write_text_if_changed(os.path.join(q, QUARANTINE_NOTE), note)
        return False, problems, n
    dst = store.mirror(ref)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    os.replace(stage, dst)
    _rmtree(store.quarantine(ref))
    return True, [], n


# --- handover package -------------------------------------------------------

def load_meta(d):
    try:
        with open(os.path.join(d, 'meta.json'), 'r', encoding='utf-8') as f:
            meta = json.load(f)
        return (meta if isinstance(meta, dict) else {}), None
    except Exception as e:                              # noqa: BLE001
        return {}, f"meta.json necitibil: {type(e).__name__}: {e}"


def fmt_boot_ms(ms):
    if not isinstance(ms, int) or isinstance(ms, bool):
        return "INDISPONIBIL (meta.json fara sync.fc_time_boot_ms_contact)"
    h, rem = divmod(ms, 3600000)
    m, rem = divmod(rem, 60000)
    s, milli = divmod(rem, 1000)
    return f"{ms} ms  (= {h}:{m:02d}:{s:02d}.{milli:03d} de la pornirea FC)"


def fmt_unix_usec(us):
    if not isinstance(us, int) or isinstance(us, bool) or us <= 0:
        return None
    t = (datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
         + datetime.timedelta(microseconds=us))
    return t.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + ' UTC'


def _get(d, *keys):
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def verdict_line(v):
    if v.get('class') is None:
        return f"verdict indisponibil ({v.get('error', '?')})"
    return (f"{v['class']}  (media {v['mean']}, min {v['min']}, max {v['max']}, "
            f"{v['patch_px']} px in jurul ({v['cx']}, {v['cy']}), {v['backend']})")


def verdict_warnings(v, meta):
    """Reasons for the visible banner (empty list = no banner)."""
    pi_cls = _get(meta, 'center', 'class')
    out = []
    if v.get('class') is None:
        out.append("verdict indisponibil pe PC")
    elif v['class'] == 'ambiguu':
        out.append("centrul e AMBIGUU")
    if pi_cls is None:
        out.append("meta.json nu are center.class")
    elif pi_cls == 'ambiguu' and v.get('class') != 'ambiguu':
        out.append("Pi-ul a clasificat centrul AMBIGUU")
    if v.get('class') and pi_cls and pi_cls != v['class']:
        out.append(f"PC ({v['class']}) si Pi ({pi_cls}) nu sunt de acord")
    return out


def attempt_readme(ref, meta, meta_err, verdict, regenerated):
    sync = _get(meta, 'sync') or {}
    size = _get(meta, 'image', 'size')
    color = _get(meta, 'image', 'color')
    burst_n = _get(meta, 'burst', 'files')
    size_txt = (f"{size[0]}x{size[1]}" if isinstance(size, list) and len(size) == 2
                else "rezolutia fluxului")
    color_txt = {True: ', color', False: ', gri'}.get(color, '')
    gps = fmt_unix_usec(sync.get('fc_time_unix_usec'))
    pi_c = _get(meta, 'center') or {}
    L = [
        "NOVA - ZDC 2026 - imaginea de touchdown (regula 8.3.3), predare conform 6.2.1.30",
        "",
        f"{ref.name}   Data: {meta.get('date', '?')}   Sesiune Pi: "
        f"{meta.get('session', '?')}   Incercarea in sesiune: {meta.get('attempt', '?')}",
        "",
        f"{ref.name}.png",
        "  Imaginea de touchdown pentru 8.3.3: cadrul camerei de aterizare facut in",
        "  momentul in care FC-ul (ArduPilot) a raportat ON_GROUND, adica la contactul",
        "  cu solul. Nerotita, nedecupata, nemodificata, la rezolutia fluxului",
        f"  ({size_txt}{color_txt}).",
        f"{ref.name}_annotated.png",
        "  Copie cu o cruce fina in centrul imaginii (x = w // 2, y = h // 2).",
    ]
    if regenerated:
        L += ["  REGENERATA PE PC de tools/fetch_scoring.py (Pi-ul nu a scris-o). Crucea",
              "  lasa libera fereastra 5x5 din centru. Nu e in MANIFEST.sha256 (al Pi-ului);",
              "  e acoperita de ../MANIFEST_HANDOVER.sha256."]
    L += [
        "burst/",
        "  Seria de cadre din jurul contactului (ring buffer), JPEG calitate 95"
        + (f", {burst_n} fisiere." if isinstance(burst_n, int) else "."),
        "meta.json",
        "  Toate metadatele: sincronizare, camera, ultima detectie, clasificarea centrului.",
        "",
        "Momentul contactului",
        f"  ceasul FC (time_boot_ms): {fmt_boot_ms(sync.get('fc_time_boot_ms_contact'))}",
        "    -> se aliniaza direct cu logul DataFlash (.bin) al FC-ului (../fc/).",
        "  ora GPS/UTC de la FC (doar eticheta): "
        + (gps if gps else "indisponibila (FC fara ora GPS la contact)"),
        f"  ora Pi-ului (doar eticheta): {sync.get('pi_time_utc') or 'indisponibila'}",
        f"  h_ref la contact: {meta.get('h_ref_m', '?')} m",
    ]
    if meta_err:
        L.append(f"  ATENTIE: {meta_err}")
    L += [
        "",
        "Verdict automat al centrului (INFORMATIV; fereastra 5x5 px pe gri;",
        "negru: media < 80 si max-min < 60; alb: media > 170 si max-min < 60;",
        "altfel ambiguu)",
        f"  PC: {verdict_line(verdict)}",
        f"  Pi (meta.json): {pi_c.get('class', 'lipseste')}"
        + (f"  (media {pi_c.get('mean')}, min {pi_c.get('min')}, max {pi_c.get('max')})"
           if 'mean' in pi_c else ""),
    ]
    for w in verdict_warnings(verdict, meta):
        L.append(f"  ATENTIE: {w}")
    L += [
        "  Decizia finala apartine omului care priveste imaginea, nu acestui verdict.",
        "",
        "Integritate",
        "  MANIFEST.sha256: manifestul scris de Pi, neschimbat (sha256sum -c MANIFEST.sha256).",
        "  ../MANIFEST_HANDOVER.sha256: tot pachetul cursei, inclusiv acest fisier si ../fc/.",
        "",
    ]
    return '\n'.join(L)


def race_readme(folder, key, race_dir):
    date, session = key.split('/')
    L = [
        "NOVA - ZDC 2026 - pachet de predare (6.2.1.30)",
        "",
        f"Cursa: {folder}",
        f"Sesiune Pi: {key}",
        "",
        "Capturi (imaginea de touchdown 8.3.3 + README in fiecare Handoff<n>-touchdown/):",
    ]
    atts = sorted((d for d in os.listdir(race_dir) if handoff_n(d) is not None),
                  key=handoff_n)
    for a in atts:
        meta, _ = load_meta(os.path.join(race_dir, a))
        ms = _get(meta, 'sync', 'fc_time_boot_ms_contact')
        L.append(f"  {a}: contact FC time_boot_ms {ms if ms is not None else '?'}, "
                 f"centru (Pi) {_get(meta, 'center', 'class') or '?'}")
    L += ["", "Log FC (DataFlash, acelasi ceas time_boot_ms):"]
    fc_dir = os.path.join(race_dir, 'fc')
    logs = sorted(os.listdir(fc_dir)) if os.path.isdir(fc_dir) else []
    logs = [x for x in logs if not x.endswith('.part')]
    if logs:
        L += [f"  fc/{x}" for x in logs]
    else:
        L += ["  niciunul adaugat inca:",
              f"  python tools/fetch_scoring.py fetch --session {session} --fc-log <log.bin>"]
    L += [
        "",
        "Integritate",
        "  MANIFEST_HANDOVER.sha256 acopera fiecare fisier din acest folder:",
        "    Linux:   sha256sum -c MANIFEST_HANDOVER.sha256",
        "    Windows: python tools\\fetch_scoring.py verify <acest folder>",
        "  Handoff<n>-touchdown/MANIFEST.sha256 e manifestul Pi-ului, neschimbat.",
        "",
    ]
    return '\n'.join(L)


def _banner(out, title, reasons):
    bar = '#' * 72
    out(bar)
    out(f"## ATENTIE {title}: " + '; '.join(reasons))
    out("## Verdictul automat e doar informativ - decizia finala e a omului.")
    out(bar)


def build_attempt_package(ref, src_dir, race_dir, out):
    """Copy a verified attempt into the race folder (atomic rename), with a
    regenerated annotated copy if missing and a README. Prints the verdict."""
    final = os.path.join(race_dir, ref.name)
    tmp = os.path.join(race_dir, BUILDING_PREFIX + ref.name)
    _rmtree(tmp)
    os.makedirs(race_dir, exist_ok=True)
    shutil.copytree(src_dir, tmp)
    meta, meta_err = load_meta(tmp)
    img_name, ann_name = f"{ref.name}.png", f"{ref.name}_annotated.png"
    img = os.path.join(tmp, img_name)
    ann = os.path.join(tmp, ann_name)
    regenerated = False
    if not os.path.isfile(img):
        verdict = {'class': None, 'error': f'{img_name} lipseste',
                   'backend': backend_name()}
    else:
        if not os.path.isfile(ann):
            regenerated, msg = annotate_center(img, ann)
            out(f"  {ann_name} lipsea: {msg}")
        verdict = center_verdict(img)
    write_text_if_changed(os.path.join(tmp, README),
                          attempt_readme(ref, meta, meta_err, verdict, regenerated))
    os.replace(tmp, final)
    pi_cls = _get(meta, 'center', 'class')
    out(f"  centru {ref.rel}: PC {verdict_line(verdict)} | Pi {pi_cls or 'lipseste'}")
    reasons = verdict_warnings(verdict, meta)
    if not os.path.isfile(os.path.join(final, img_name)):
        reasons.insert(0, "IMAGINEA DE TOUCHDOWN LIPSESTE")
    if reasons:
        _banner(out, ref.rel, reasons)
    return verdict


def add_fc_log(race_dir, src, out):
    """Copy an FC log into race_dir/fc/. Returns (changed, error)."""
    if not os.path.isfile(src):
        return False, f"--fc-log: nu exista fisierul {src}"
    name = os.path.basename(src)
    if not safe_rel(name) or name.startswith('.'):
        return False, f"--fc-log: nume de fisier nepotrivit {name!r}"
    fc_dir = os.path.join(race_dir, 'fc')
    dst = os.path.join(fc_dir, name)
    h = sha256_file(src)
    if os.path.isfile(dst):
        if sha256_file(dst) == h:
            return False, None
        return False, (f"--fc-log: fc/{name} exista deja in pachet cu alt continut; "
                       "nu suprascriu evidenta (redenumeste fisierul nou)")
    with open(src, 'rb') as f:
        if f.read(2) != b'\xa3\x95':
            out(f"  atentie: {name} nu incepe ca un log DataFlash (0xA3 0x95)")
    os.makedirs(fc_dir, exist_ok=True)
    part = dst + '.part'
    shutil.copyfile(src, part)
    if sha256_file(part) != h:
        os.remove(part)
        return False, f"--fc-log: copia lui {name} nu are acelasi sha256"
    os.replace(part, dst)
    out(f"  log FC adaugat in {os.path.basename(race_dir)}: fc/{name} "
        f"(sha256 {h[:16]}...)")
    return True, None


def safe_name(s):
    return re.sub(r'[^A-Za-z0-9._-]+', '_', s).strip('._') or 'cursa'


def _load_index(store):
    try:
        with open(os.path.join(store.handover_root, INDEX_FILE), encoding='utf-8') as f:
            idx = json.load(f)
        return idx if isinstance(idx, dict) else {}
    except (OSError, ValueError):
        return {}


def _save_index(store, idx):
    os.makedirs(store.handover_root, exist_ok=True)
    write_text_if_changed(os.path.join(store.handover_root, INDEX_FILE),
                          json.dumps(idx, indent=1, sort_keys=True) + '\n')


def _folder_owner(race_dir):
    """The session key a race folder belongs to, from its README."""
    try:
        with open(os.path.join(race_dir, README), encoding='utf-8') as f:
            for line in f:
                if line.startswith('Sesiune Pi: '):
                    return line[len('Sesiune Pi: '):].strip()
    except OSError:
        pass
    return None


def _unique_folder(store, want, key, index):
    session = key.split('/')[1]
    taken = {v for k, v in index.items() if k != key}
    cands = [want, f"{want}_{session}"] + [f"{want}_{session}_{i}" for i in range(2, 100)]
    for c in cands:
        if c in taken:
            continue
        d = os.path.join(store.handover_root, c)
        if os.path.exists(d) and _folder_owner(d) not in (None, key):
            continue
        return c
    raise SetupError(f"nu gasesc un nume liber pentru {want}")


def update_handover(store, target_key, cursa, fc_logs, rename, out):
    """Build/refresh the handover packages from the verified copies.

    --cursa names the target session's package when it is created (or, with
    rename=True i.e. an explicit --session, renames an existing one). Other
    sessions keep their names. --fc-log goes to the target's fc/.
    Returns (changed_any, errors)."""
    by_key = collections.defaultdict(list)
    for ref in store.local_attempts():
        by_key[store.session_key(ref)].append(ref)
    if target_key is None and by_key:
        known = [k for k in by_key if k != UNKNOWN_KEY]
        target_key = max(known) if known else UNKNOWN_KEY
    index = _load_index(store)
    errors, changed_any = [], False
    for key in sorted(by_key):
        date, session = key.split('/')
        is_target = key == target_key
        folder = index.get(key)
        changed = False
        if folder is None:
            base = safe_name(cursa) if (is_target and cursa) else session
            folder = _unique_folder(store, f"{date}_{base}", key, index)
            index[key] = folder
            _save_index(store, index)
            if folder != f"{date}_{base}":
                out(f"  atentie: {date}_{base} e folosit de alta sesiune; pachetul "
                    f"sesiunii {key} e {folder}")
        elif is_target and cursa and rename and folder != f"{date}_{safe_name(cursa)}":
            new = _unique_folder(store, f"{date}_{safe_name(cursa)}", key, index)
            if new != folder:
                old_dir = os.path.join(store.handover_root, folder)
                if os.path.isdir(old_dir):
                    os.replace(old_dir, os.path.join(store.handover_root, new))
                out(f"  pachet redenumit: {folder} -> {new}")
                index[key] = new
                _save_index(store, index)
                folder, changed = new, True
        elif is_target and cursa and not rename and folder != f"{date}_{safe_name(cursa)}":
            out(f"  --cursa: sesiunea {key} are deja pachetul {folder}; pentru "
                f"redenumire: fetch --session {session} --cursa {cursa}")
        race_dir = os.path.join(store.handover_root, folder)
        os.makedirs(race_dir, exist_ok=True)
        for d in os.listdir(race_dir):
            if d.startswith(BUILDING_PREFIX):
                _rmtree(os.path.join(race_dir, d))
        for ref in sorted(by_key[key], key=lambda r: r.n):
            if not os.path.isdir(os.path.join(race_dir, ref.name)):
                build_attempt_package(ref, store.mirror(ref), race_dir, out)
                changed = True
        if is_target:
            for src in fc_logs:
                c, err = add_fc_log(race_dir, src, out)
                changed |= c
                if err:
                    errors.append(err)
        readme_changed = write_text_if_changed(os.path.join(race_dir, README),
                                               race_readme(folder, key, race_dir))
        if changed or readme_changed or not os.path.isfile(
                os.path.join(race_dir, HANDOVER_MANIFEST)):
            write_text_if_changed(os.path.join(race_dir, HANDOVER_MANIFEST),
                                  manifest_text(race_dir, exclude={HANDOVER_MANIFEST}))
            out(f"  pachet de predare: {race_dir}")
            changed_any = True
    if fc_logs and target_key not in by_key:
        errors.append("--fc-log: nicio incercare primita pentru sesiunea tinta; "
                      "logul nu are unde sa fie pus")
    return changed_any, errors


# --- commands ---------------------------------------------------------------

def resolve_session(opt, keys):
    """--session NAME or DATE/NAME -> session key (or None)."""
    if not opt:
        return None
    hits = sorted({k for k in keys if k == opt or k.split('/')[1] == opt})
    if len(hits) > 1:
        raise SetupError(f"--session {opt} e ambiguu: {', '.join(hits)}; foloseste DATA/SESIUNE")
    if not hits:
        raise SetupError(f"--session {opt}: nu exista nici pe Pi, nici local")
    return hits[0]


def cmd_list(source, store, session_opt, out):
    refs = source.list_attempts()
    local = {r.name: store.session_key(r) for r in store.local_attempts()}
    key = resolve_session(session_opt, set(local.values()))
    if key is not None:
        refs = [r for r in refs if local.get(r.name) == key]
    out(f"Sursa: {source.describe()}")
    out(f"Local: {store.root}")
    if not refs:
        out("  nicio captura")
    todo = 0
    for r in refs:
        remote = 'completa  ' if r.complete else 'INCOMPLETA (fara MANIFEST.sha256, ignorata)'
        if r.complete and store.conflict(r):
            state = 'CONFLICT: alt continut decat copia locala (vezi fetch)'
        elif store.is_received(r):
            state = f"primita ({local.get(r.name, UNKNOWN_KEY)})"
        elif os.path.isdir(store.quarantine(r)):
            state = 'CARANTINA (sha256 gresit la ultima descarcare)'
        else:
            state = '-'
        if r.complete and not store.is_received(r):
            todo += 1
        out(f"  {r.name:<24} {remote}  {state if r.complete else ''}".rstrip())
    for x in source.ignored:
        out(f"  ignorat (nume neasteptat): {x}")
    n_c = sum(1 for r in refs if r.complete)
    out(f"{len(refs)} capturi: {n_c} complete, {len(refs) - n_c} incomplete; "
        f"{todo} de adus cu fetch")
    return 0


class WatchState(object):
    def __init__(self):
        self.cycle = 0
        self.seen_incomplete = set()
        self.seen_conflict = set()
        self.failed_at = {}
        self.link_down = None


def fetch_once(source, store, opts, out, state=None, now=time.monotonic):
    """One fetch pass. Returns 0 (ok) or 1 (verification / --fc-log errors).
    TransportError propagates (after packaging what did arrive)."""
    state = state or WatchState()
    refs = source.list_attempts()
    bad, new, link_err = [], [], None
    for ref in refs:
        if not ref.complete:
            if ref.rel not in state.seen_incomplete:
                state.seen_incomplete.add(ref.rel)
                out(f"  incompleta (fara MANIFEST.sha256), ignorata: {ref.rel}")
            continue
        if store.conflict(ref):
            bad.append(ref)
            if ref.rel not in state.seen_conflict:
                state.seen_conflict.add(ref.rel)
                out(f"  EROARE {ref.rel}: pe Pi are ALT continut decat copia locala "
                    f"(numerotarea a reinceput pe Pi?). Nu suprascriu nimic; muta "
                    f"{store.mirror(ref)} in alta parte si reia fetch ca s-o aduc "
                    f"pe cea noua.")
            continue
        if store.is_received(ref):
            continue
        t_bad = state.failed_at.get(ref.rel)
        if t_bad is not None and now() - t_bad < WATCH_RETRY_BAD_S:
            bad.append(ref)
            continue
        out(f"  descarc {ref.rel} ...")
        try:
            ok, problems, n = accept_attempt(source, ref, store)
        except TransportError as e:
            link_err = e
            out(f"  descarcare intrerupta ({e}); partialul ramane in "
                f"{store.partial(ref)} si nu e folosit")
            break
        if ok:
            state.failed_at.pop(ref.rel, None)
            new.append(ref)
            out(f"  primita: {ref.rel} ({n} fisiere, sha256 OK)")
        else:
            state.failed_at[ref.rel] = now()
            bad.append(ref)
            out(f"  EROARE {ref.rel}: verificarea sha256 a esuat, incercarea NU e primita")
            for p in problems[:20]:
                out(f"      {p}")
            out(f"      copia e in carantina: {store.quarantine(ref)}")
    key = resolve_session(opts.session, {store.session_key(r)
                                         for r in store.local_attempts()})
    changed, errors = update_handover(store, key, opts.cursa, opts.fc_log or [],
                                      rename=bool(opts.session), out=out)
    for e in errors:
        out(f"  EROARE {e}")
    if link_err is not None:
        raise link_err
    if not new and not changed and not bad and not errors and state.cycle == 0:
        out("  nimic nou")
    return 1 if (bad or errors) else 0


def run_watch(source, store, opts, out, sleep=time.sleep):
    plain = out

    def out(s=''):
        plain(time.strftime('%H:%M:%S ') + s if s else s)
    state = WatchState()
    interval = opts.watch
    out(f"urmaresc {source.describe()} la fiecare {interval:g} s (Ctrl-C opreste)")
    try:
        while True:
            try:
                fetch_once(source, store, opts, out, state)
                if state.link_down:
                    out("  legatura revenita")
                    state.link_down = None
            except (TransportError, OSError) as e:
                # OSError: local file busy (e.g. a folder open in Explorer).
                what = 'legatura' if isinstance(e, TransportError) else 'fisier local'
                if state.link_down != str(e):
                    out(f"  {what}: {e} - reincerc la fiecare {interval:g} s")
                    state.link_down = str(e)
            state.cycle += 1
            sleep(interval)
    except KeyboardInterrupt:
        out("oprit (Ctrl-C)")
        return 0


def cmd_verify(path, out):
    """Check a race folder (MANIFEST_HANDOVER.sha256 + each attempt's
    MANIFEST.sha256) or a single attempt folder. For PCs without sha256sum."""
    if not os.path.isdir(path):
        out(f"nu exista {path}")
        return 1
    fails = 0
    dirs = []
    if os.path.isfile(os.path.join(path, HANDOVER_MANIFEST)):
        problems, _x, n = verify_dir(path, HANDOVER_MANIFEST, strict=True)
        out(f"{HANDOVER_MANIFEST}: {n} fisiere, " + ('OK' if not problems else 'ESEC'))
        for p in problems:
            out(f"    {p}")
        fails += bool(problems)
        dirs = [os.path.join(path, d) for d in sorted(os.listdir(path), key=str)
                if handoff_n(d) is not None]
    elif os.path.isfile(os.path.join(path, MANIFEST)):
        dirs = [path]
    else:
        out(f"nici {HANDOVER_MANIFEST}, nici {MANIFEST} in {path}")
        return 1
    for d in dirs:
        problems, extras, n = verify_dir(d, MANIFEST, strict=False)
        extra = f"; in plus (ale PC-ului): {', '.join(extras)}" if extras else ''
        out(f"{os.path.basename(d)}/{MANIFEST}: {n} fisiere, "
            + ('OK' if not problems else 'ESEC') + extra)
        for p in problems:
            out(f"    {p}")
        fails += bool(problems)
    return 1 if fails else 0


def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--host', default=DEFAULT_HOST,
                        help=f"user@host al Pi-ului (implicit {DEFAULT_HOST})")
    common.add_argument('--remote-root', default=DEFAULT_REMOTE_ROOT,
                        help=f"radacina pe Pi (implicit {DEFAULT_REMOTE_ROOT})")
    common.add_argument('--source-dir', default=None,
                        help="director local cu structura Pi-ului, in loc de SSH")
    common.add_argument('--local', default=DEFAULT_LOCAL,
                        help="radacina locala (implicit data/scoring_pc din repo)")
    common.add_argument('--handover', default=None,
                        help="radacina pachetelor de predare (implicit <local>/handover)")
    common.add_argument('--session', default=None,
                        help="sesiunea Pi (sNNNNNNNN-NNNNNN sau DATA/SESIUNE) careia "
                             "i se aplica --cursa / --fc-log; list: doar capturile ei")
    common.add_argument('--ssh', default='ssh', help="executabilul ssh")
    common.add_argument('--identity', default=None, metavar='CHEIE',
                        help="cheia privata ssh (ssh -i), daca nu e cea implicita")
    common.add_argument('--ssh-arg', action='append', default=[],
                        help="argument suplimentar pentru ssh (repetabil), "
                             "scris --ssh-arg=-oPort=2222")
    common.add_argument('--no-cv2', action='store_true',
                        help="nu folosi OpenCV chiar daca e instalat")
    p = argparse.ArgumentParser(
        description="Aduce capturile de touchdown de pe Pi si face pachetul de predare.")
    sub = p.add_subparsers(dest='cmd')
    sub.required = True
    sub.add_parser('list', parents=[common], help="capturile de pe Pi")
    f = sub.add_parser('fetch', parents=[common],
                       help="descarca capturile complete lipsa + pachetul de predare")
    f.add_argument('--watch', type=float, default=None, metavar='N',
                   help="repeta la fiecare N secunde (Ctrl-C opreste)")
    f.add_argument('--cursa', default=None,
                   help="numele cursei in handover/<data>_<cursa> (implicit sesiunea)")
    f.add_argument('--fc-log', action='append', default=[], metavar='LOG.bin',
                   help="logul DataFlash al FC-ului, pus in pachetul cursei (repetabil)")
    v = sub.add_parser('verify', help="verifica un pachet (fara sha256sum)")
    v.add_argument('dir')
    return p


def _print(s=''):
    """print() that survives a non-UTF-8 Windows console (paths with
    diacritics in the user name, output redirected to a file)."""
    try:
        print(s, flush=True)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, 'encoding', None) or 'ascii'
        print(s.encode(enc, 'replace').decode(enc, 'replace'), flush=True)


def main(argv=None, runner=None, sleep=time.sleep, out=None):
    out = out or _print
    args = build_parser().parse_args(argv)
    if args.cmd == 'verify':
        return cmd_verify(args.dir, out)
    if args.no_cv2:
        set_backend(False)
    try:
        if args.cmd == 'fetch':
            if args.watch is not None and args.watch <= 0:
                raise SetupError("--watch N cere N > 0")
            if args.watch is not None and args.fc_log:
                raise SetupError("--fc-log se adauga dupa cursa, fara --watch")
        store = Store(args.local, args.handover)
        if args.source_dir:
            source = LocalSource(args.source_dir)
        else:
            extra = (['-i', args.identity] if args.identity else []) + args.ssh_arg
            source = SshSource(args.host, args.remote_root, runner=runner,
                               ssh=args.ssh, ssh_args=extra)
        if args.cmd == 'list':
            return cmd_list(source, store, args.session, out)
        if args.watch is not None:
            return run_watch(source, store, args, out, sleep=sleep)
        out(f"Sursa: {source.describe()}   local: {store.root}")
        return fetch_once(source, store, args, out)
    except TransportError as e:
        out(f"EROARE legatura: {e}")
        return 2
    except SetupError as e:
        out(f"EROARE: {e}")
        return 3
    except OSError as e:
        out(f"EROARE fisier local: {e}")
        return 1
    except KeyboardInterrupt:
        out("oprit (Ctrl-C)")
        return 130


if __name__ == '__main__':
    sys.exit(main())
