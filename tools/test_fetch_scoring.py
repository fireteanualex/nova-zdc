#!/usr/bin/env python3
"""
NOVA - ZDC 2026
tools/fetch_scoring.py: aducerea capturii de touchdown pe PC (8.3.3, 6.2.1.30).

    python3 tools/test_fetch_scoring.py

Fara Pi si fara retea: sursa e un director local cu structura din
docs/SCORING_FORMAT.md, iar transportul SSH e verificat cu un runner fals
care joaca rolul Pi-ului (comenzile construite, nu executate). Imaginile
sunt PNG-uri reale scrise in Python pur; testele de baza trec si fara
OpenCV (calea Python e fortata), iar cand OpenCV exista se compara si el.
"""

import contextlib
import hashlib
import io
import json
import os
import shlex
import shutil
import sys
import tarfile
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fetch_scoring as fs                                   # noqa: E402

DATE = '20260927'
SESS = 's20260927-140312'
FC_MS = 123456
UNIX_US = 1790517792345000          # 2026-09-27 14:03:12.345 UTC
UNIX_TXT = '2026-09-27 14:03:12.345 UTC'
W, H = 48, 36
FAKE_JPEG = b'\xff\xd8\xff\xe0' + b'NOVA' * 16 + b'\xff\xd9'


# --- fixtures ---------------------------------------------------------------

def image_rows(center, channels=3, w=W, h=H, bg=200):
    """Background `bg`, a 9x9 patch in the centre. center='mix' = half black,
    half white (ambiguous)."""
    cx, cy = w // 2, h // 2
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            v = bg
            if abs(x - cx) <= 4 and abs(y - cy) <= 4:
                v = (0 if x < cx else 255) if center == 'mix' else center
            px = [v] * (1 if channels <= 2 else 3)
            if channels in (2, 4):
                px.append(255)
            row += bytes(px)
        rows.append(row)
    return rows


def write_png(path, center=10, channels=3, w=W, h=H):
    fs.encode_png(path, w, h, channels, image_rows(center, channels, w, h),
                  filters=[0, 1, 2, 3, 4])


def write_manifest(d):
    lines = []
    for rel in fs.list_files(d):
        if rel == fs.MANIFEST:
            continue
        h = hashlib.sha256(open(os.path.join(d, *rel.split('/')), 'rb').read()).hexdigest()
        lines.append(f"{h}  {rel}\n")
    with open(os.path.join(d, fs.MANIFEST), 'w', newline='\n') as f:
        f.write(''.join(lines))


def make_attempt(root, n=1, session=SESS, center=10, annotated=True, manifest=True,
                 pi_class='negru', burst=3, png=None):
    name = f"Handoff{n}-touchdown"
    d = os.path.join(root, name)
    os.makedirs(os.path.join(d, 'burst'))
    img = os.path.join(d, f"{name}.png")
    if png is not None:
        with open(img, 'wb') as f:
            f.write(png)
    else:
        write_png(img, center)
    if annotated:
        shutil.copyfile(img, os.path.join(d, f"{name}_annotated.png"))
    for i in range(burst):
        with open(os.path.join(d, 'burst', f"{i:04d}_{1000 + 33 * i}.jpg"), 'wb') as f:
            f.write(FAKE_JPEG + bytes([n, i]))
    meta = {
        'format': 'nova-scoring-2', 'handoff': n, 'name': name,
        'date': DATE, 'session': session, 'attempt': n,
        'sync': {'t_capture': 812.5, 't_on_ground_rx': 812.49,
                 'fc_time_boot_ms_contact': FC_MS + n, 'fc_time_unix_usec': UNIX_US,
                 'pi_time_utc': '2026-09-27T14:03:12.400+00:00'},
        'h_ref_m': 0.08,
        'last_detection': {'age_s': 0.4, 'lateral_m': 0.03},
        'camera': {'preset': 'crop1280', 'sensor_mode': [1536, 864],
                   'scaler_crop': [0, 0, 4608, 2592], 'calibration': 'derivata+scalata',
                   'ExposureTime': 2000, 'AnalogueGain': 1.5, 'LensPosition': 1.63},
        'image': {'file': f"{name}.png", 'annotated': f"{name}_annotated.png",
                  'size': [W, H], 'color': True},
        'center': {'class': pi_class, 'mean': 10.0, 'min': 10, 'max': 10, 'patch_px': 25},
        'burst': {'files': burst, 't_from': 811.0, 't_to': 813.0},
    }
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    if manifest:
        write_manifest(d)
    return d


def tmp():
    return tempfile.mkdtemp(prefix='nova_fetch_')


def run(*argv, runner=None, sleep=None):
    lines = []
    kw = {'runner': runner, 'out': lambda s='': lines.append(s)}
    if sleep is not None:
        kw['sleep'] = sleep
    rc = fs.main(list(argv), **kw)
    return rc, '\n'.join(lines)


def snapshot(root):
    out = {}
    for dp, _dn, fns in os.walk(root):
        for fn in fns:
            p = os.path.join(dp, fn)
            st = os.stat(p)
            out[os.path.relpath(p, root)] = (st.st_size, st.st_mtime_ns)
    return out


def pure_python(fn):
    """Run a test body with OpenCV disabled, then restore."""
    def wrapped():
        prev = fs._USE_CV2
        fs.set_backend(False)
        try:
            return fn()
        finally:
            fs._USE_CV2 = prev
    wrapped.__name__ = fn.__name__
    return wrapped


# --- tests ------------------------------------------------------------------

@pure_python
def test_png_python_pur_toate_filtrele():
    d = tmp()
    for ch in (1, 2, 3, 4):
        rows = image_rows('mix', ch)
        p = os.path.join(d, f"c{ch}.png")
        fs.encode_png(p, W, H, ch, rows, filters=[0, 1, 2, 3, 4, 4, 3])
        png = fs.decode_png(p)
        assert (png.w, png.h, png.channels) == (W, H, ch)
        assert [bytes(r) for r in png.rows] == [bytes(r) for r in rows], ch
    note = "gri, gri+alfa, RGB, RGBA; filtrele 0-4"
    if fs._cv2 is not None:
        np, cv2 = fs._np, fs._cv2
        rng = np.random.RandomState(1)
        img = rng.randint(0, 256, (37, 53, 3)).astype(np.uint8)
        p = os.path.join(d, 'cv.png')
        cv2.imwrite(p, img)
        png = fs.decode_png(p)
        got = np.frombuffer(b''.join(png.rows), np.uint8).reshape(37, 53, 3)
        assert (got[:, :, ::-1] == img).all(), "decodare diferita de OpenCV"
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mine = [fs._gray(png.rows[y], x, 3) for y in range(37) for x in range(53)]
        assert mine == [int(v) for v in g.ravel()], "luminanta diferita de cv2"
        note += "; identic cu OpenCV (pixeli si gri)"
    shutil.rmtree(d)
    return note


def test_clasificarea_centrului_ambele_cai():
    d = tmp()
    cases = {'negru': 10, 'alb': 240, 'ambiguu': 'mix'}
    backends = [False] + ([True] if fs._cv2 is not None else [])
    prev = fs._USE_CV2
    try:
        for use in backends:
            fs.set_backend(use)
            for want, c in cases.items():
                for ch in (1, 3):
                    p = os.path.join(d, f"{want}{ch}.png")
                    write_png(p, c, ch)
                    v = fs.center_verdict(p)
                    assert v['class'] == want, (fs.backend_name(), want, ch, v)
                    assert v['patch_px'] == 25 and (v['cx'], v['cy']) == (W // 2, H // 2)
            # 16-bit PNG: no crash, "verdict indisponibil"
            p16 = os.path.join(d, 'p16.png')
            png16 = png_16bit()
            open(p16, 'wb').write(png16)
            v = fs.center_verdict(p16)
            assert v['class'] is None and v['error'], v
            v = fs.center_verdict(os.path.join(d, 'lipsa.png'))
            assert v['class'] is None
    finally:
        fs._USE_CV2 = prev
    shutil.rmtree(d)
    return f"negru/alb/ambiguu pe gri si RGB, cai: {'python+cv2' if len(backends) > 1 else 'python'}; 16 biti -> indisponibil fara exceptie"


def png_16bit(w=8, h=8):
    import struct
    import zlib
    raw = b''.join(b'\x00' + b'\x80\x00' * w for _ in range(h))

    def chunk(t, b):
        return struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xFFFFFFFF)
    return (fs.PNG_SIG + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 16, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


@pure_python
def test_listare_completa_si_incompleta():
    src, loc = tmp(), tmp()
    make_attempt(src, 1)
    make_attempt(src, 2, manifest=False)
    os.makedirs(os.path.join(src, 'ciudat'))
    rc, out = run('list', '--source-dir', src, '--local', loc)
    assert rc == 0, out
    l1 = [x for x in out.splitlines() if 'Handoff1-touchdown' in x][0]
    l2 = [x for x in out.splitlines() if 'Handoff2-touchdown' in x][0]
    assert 'completa' in l1 and 'INCOMPLETA' not in l1, l1
    assert 'INCOMPLETA' in l2, l2
    assert 'ignorat' in out and 'ciudat' in out, out
    assert '1 de adus cu fetch' in out, out
    shutil.rmtree(src), shutil.rmtree(loc)
    return "Handoff1 completa, Handoff2 INCOMPLETA, nume ciudat raportat"


@pure_python
def test_fetch_ignora_incompleta():
    src, loc = tmp(), tmp()
    make_attempt(src, 1)
    make_attempt(src, 2, manifest=False)
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0, out
    got = sorted(x for x in os.listdir(loc) if x.startswith('Handoff'))
    assert got == ['Handoff1-touchdown'], got
    race = os.path.join(loc, 'handover', f"{DATE}_{SESS}")
    assert os.path.isdir(os.path.join(race, 'Handoff1-touchdown'))
    assert not os.path.exists(os.path.join(race, 'Handoff2-touchdown'))
    assert 'incompleta' in out and 'Handoff2-touchdown' in out, out
    # once the Pi finishes Handoff2 it is picked up
    write_manifest(os.path.join(src, 'Handoff2-touchdown'))
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0 and os.path.isdir(os.path.join(race, 'Handoff2-touchdown')), out
    shutil.rmtree(src), shutil.rmtree(loc)
    return "incompleta ignorata, preluata dupa ce apare manifestul"


@pure_python
def test_sha256_gresit_carantina():
    src, loc = tmp(), tmp()
    d = make_attempt(src, 1)
    jpg = os.path.join(d, 'burst', '0001_1033.jpg')
    good = open(jpg, 'rb').read()
    open(jpg, 'wb').write(good[:-1] + b'\x00')
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 1, (rc, out)
    assert 'sha256 diferit: burst/0001_1033.jpg' in out, out
    assert 'NU e primita' in out
    assert not os.path.exists(os.path.join(loc, 'Handoff1-touchdown'))
    race = os.path.join(loc, 'handover', f"{DATE}_{SESS}")
    assert not os.path.exists(os.path.join(race, 'Handoff1-touchdown'))
    q = os.path.join(loc, 'carantina', 'Handoff1-touchdown')
    assert os.path.isfile(os.path.join(q, fs.QUARANTINE_NOTE))
    assert 'NU E PRIMITA' in open(os.path.join(q, fs.QUARANTINE_NOTE)).read()
    rc, out = run('list', '--source-dir', src, '--local', loc)
    assert 'CARANTINA' in out, out
    # unlisted extra file is refused too
    open(jpg, 'wb').write(good)
    extra = os.path.join(d, 'burst', '9999_0.jpg')
    open(extra, 'wb').write(FAKE_JPEG)
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 1 and 'neacoperit de manifest: burst/9999_0.jpg' in out, out
    # fixed on the source -> accepted, quarantine cleared
    os.remove(extra)
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0, out
    assert os.path.isdir(os.path.join(race, 'Handoff1-touchdown')) and not os.path.exists(q)
    assert not os.listdir(os.path.join(loc, '.partial'))
    shutil.rmtree(src), shutil.rmtree(loc)
    return "hash gresit si fisier in plus -> rc 1, carantina, nepredat; reparat -> primit"


@pure_python
def test_fetch_repetat_fara_duplicate():
    src, loc = tmp(), tmp()
    make_attempt(src, 1)
    make_attempt(src, 2)
    rc, out1 = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0 and out1.count('primita:') == 2, out1
    before = snapshot(loc)
    rc, out2 = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0, out2
    assert 'nimic nou' in out2 and 'primita:' not in out2 and 'descarc' not in out2, out2
    assert snapshot(loc) == before, "al doilea fetch a modificat fisiere"
    shutil.rmtree(src), shutil.rmtree(loc)
    return f"{len(before)} fisiere locale, neschimbate (nume, marime, mtime) la al doilea fetch"


@pure_python
def test_pachet_de_predare_si_readme():
    src, loc = tmp(), tmp()
    d = make_attempt(src, 1)
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--cursa', 'cursa 1')
    assert rc == 0, out
    race = os.path.join(loc, 'handover', f"{DATE}_cursa_1")
    a = os.path.join(race, 'Handoff1-touchdown')
    for f in ('Handoff1-touchdown.png', 'Handoff1-touchdown_annotated.png', 'meta.json',
              fs.MANIFEST,
              'README.txt', 'burst/0000_1000.jpg'):
        assert os.path.isfile(os.path.join(a, *f.split('/'))), f
    assert open(os.path.join(a, fs.MANIFEST), 'rb').read() == \
        open(os.path.join(d, fs.MANIFEST), 'rb').read(), "manifestul Pi-ului s-a schimbat"
    readme = open(os.path.join(a, 'README.txt')).read()
    for s in ('8.3.3', 'ON_GROUND', f"{FC_MS + 1} ms", '0:02:03.457', UNIX_TXT,
              'PC: negru', 'Pi (meta.json): negru'):
        assert s in readme, (s, readme)
    assert 'Decizia finala apartine omului' in readme
    assert readme.isascii(), "README cu caractere non-ASCII"
    race_txt = open(os.path.join(race, 'README.txt')).read()
    assert f"Sesiune Pi: {DATE}/{SESS}" in race_txt and 'Handoff1-touchdown' in race_txt
    man = open(os.path.join(race, fs.HANDOVER_MANIFEST)).read()
    assert 'Handoff1-touchdown/README.txt' in man and 'README.txt' in man
    rc, vout = run('verify', race)
    assert rc == 0, vout
    assert fs.load_meta(a)[0]['sync']['fc_time_boot_ms_contact'] == FC_MS + 1
    shutil.rmtree(src), shutil.rmtree(loc)
    return "handover/20260927_cursa_1/Handoff1-touchdown: fisierele Pi + README (ms FC, UTC, verdict); verify OK"


@pure_python
def test_fc_log_in_pachet_si_manifest():
    src, loc, logs = tmp(), tmp(), tmp()
    make_attempt(src, 1)
    make_attempt(src, 2)
    binp = os.path.join(logs, '00000042.BIN')
    open(binp, 'wb').write(b'\xa3\x95\x80' + os.urandom(4000))
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--cursa', 'c2',
                  '--fc-log', binp)
    assert rc == 0, out
    race = os.path.join(loc, 'handover', f"{DATE}_c2")
    dst = os.path.join(race, 'fc', '00000042.BIN')
    assert open(dst, 'rb').read() == open(binp, 'rb').read()
    h = fs.sha256_file(binp)
    man = open(os.path.join(race, fs.HANDOVER_MANIFEST)).read()
    assert f"{h}  fc/00000042.BIN\n" in man, man
    assert 'Handoff1-touchdown/MANIFEST.sha256' in man
    assert 'Handoff2-touchdown/Handoff2-touchdown.png' in man
    assert 'fc/00000042.BIN' in open(os.path.join(race, 'README.txt')).read()
    assert run('verify', race)[0] == 0
    before = snapshot(loc)
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--fc-log', binp)
    assert rc == 0 and 'nimic nou' in out and snapshot(loc) == before, out
    # same name, other content: refused, evidence untouched
    other = os.path.join(tmp(), '00000042.BIN')
    open(other, 'wb').write(b'\xa3\x95' + b'x' * 10)
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--fc-log', other)
    assert rc == 1 and 'nu suprascriu' in out and fs.sha256_file(dst) == h, out
    # tampering is caught by verify
    open(dst, 'ab').write(b'!')
    rc, vout = run('verify', race)
    assert rc == 1 and 'sha256 diferit: fc/00000042.BIN' in vout, vout
    # --fc-log with --watch is refused
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--watch', '5',
                  '--fc-log', binp)
    assert rc == 3, out
    shutil.rmtree(src), shutil.rmtree(loc), shutil.rmtree(logs)
    return "fc/00000042.BIN la nivel de cursa, in MANIFEST_HANDOVER; idempotent; suprascriere refuzata"


def test_adnotata_regenerata_cand_lipseste():
    notes = []
    backends = [False] + ([True] if fs._cv2 is not None else [])
    prev = fs._USE_CV2
    try:
        for use in backends:
            fs.set_backend(use)
            src, loc = tmp(), tmp()
            make_attempt(src, 1, annotated=False)
            rc, out = run('fetch', '--source-dir', src, '--local', loc)
            assert rc == 0, out
            assert 'touchdown_annotated.png lipsea: regenerata' in out, out
            assert not os.path.exists(os.path.join(loc, 'Handoff1-touchdown',
                                                   'Handoff1-touchdown_annotated.png'))
            a = os.path.join(loc, 'handover', f"{DATE}_{SESS}", 'Handoff1-touchdown')
            fs.set_backend(False)
            ann = fs.decode_png(os.path.join(a, 'Handoff1-touchdown_annotated.png'))
            org = fs.decode_png(os.path.join(a, 'Handoff1-touchdown.png'))
            cx, cy = W // 2, H // 2
            for x, y in fs.cross_pixels(W, H):
                assert ann.rows[y][3 * x:3 * x + 3] == b'\xff\x00\x00', (x, y)
            for y in range(cy - 2, cy + 3):
                assert ann.rows[y][3 * (cx - 2):3 * (cx + 3)] == \
                    org.rows[y][3 * (cx - 2):3 * (cx + 3)], "crucea acopera centrul"
            assert 'REGENERATA PE PC' in open(os.path.join(a, 'README.txt')).read()
            race = os.path.dirname(a)
            assert 'Handoff1-touchdown/Handoff1-touchdown_annotated.png' in open(
                os.path.join(race, fs.HANDOVER_MANIFEST)).read()
            rc, vout = run('verify', race)
            assert rc == 0 and 'in plus (ale PC-ului)' in vout, vout
            notes.append('cv2' if use else 'python')
            fs._USE_CV2 = prev
            shutil.rmtree(src), shutil.rmtree(loc)
    finally:
        fs._USE_CV2 = prev
    return f"regenerata ({'+'.join(notes)}); cruce rosie, centrul 5x5 neatins; in MANIFEST_HANDOVER"


@pure_python
def test_centru_ambiguu_avertisment_vizibil():
    src, loc = tmp(), tmp()
    make_attempt(src, 1, center='mix', pi_class='ambiguu')
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0, out
    assert '#' * 40 in out and 'ATENTIE' in out and 'AMBIGUU' in out, out
    assert 'decizia finala e a omului' in out, out
    readme = open(os.path.join(loc, 'handover', f"{DATE}_{SESS}", 'Handoff1-touchdown',
                               'README.txt')).read()
    assert 'PC: ambiguu' in readme and 'ATENTIE: centrul e AMBIGUU' in readme
    # undecodable image: no crash, "verdict indisponibil" banner
    src2, loc2 = tmp(), tmp()
    make_attempt(src2, 1, png=png_16bit())
    rc, out = run('fetch', '--source-dir', src2, '--local', loc2)
    assert rc == 0 and 'verdict indisponibil' in out and 'ATENTIE' in out, out
    # PC and Pi disagree
    src3, loc3 = tmp(), tmp()
    make_attempt(src3, 1, center=240, pi_class='negru')
    rc, out = run('fetch', '--source-dir', src3, '--local', loc3)
    assert 'nu sunt de acord' in out, out
    for x in (src, loc, src2, loc2, src3, loc3):
        shutil.rmtree(x)
    return "ambiguu, indisponibil (PNG 16 biti) si dezacord PC/Pi -> banner; fara exceptie"


class FakePi(object):
    """Plays the Pi behind `ssh`: checks the argv, interprets the remote
    command (after shell-style splitting) against a local directory."""

    def __init__(self, home, host='nova@100.97.236.82', tar_missing=False, down=0):
        self.home, self.host = home, host
        self.tar_missing, self.down = tar_missing, down
        self.calls = []

    def path(self, tok):
        assert tok.startswith('$HOME/'), tok
        return os.path.join(self.home, *tok[len('$HOME/'):].split('/'))

    def __call__(self, argv, timeout):
        self.calls.append(argv)
        assert argv[:5] == ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10'], argv
        if self.host != 'nova@100.97.236.82':
            assert argv[5:9] == ['-i', 'k', '-oPort=2222', '--'], argv
        i = argv.index('--')
        assert argv[i + 1] == self.host and len(argv) == i + 3, argv
        cmd = argv[-1]
        toks = shlex.split(cmd)
        forbidden = {'rm', 'mv', 'cp', 'mkdir', 'touch', 'dd', 'tee', 'chmod', 'sudo'}
        assert not forbidden & set(toks) and '>' not in cmd, cmd
        if self.down > 0:
            self.down -= 1
            return 255, b'', b'ssh: connect to host 100.97.236.82 port 22: No route to host\r\n'
        if toks[0] == 'if':
            root = self.path(toks[3])
            if not os.path.isdir(root):
                return 0, (fs.NO_ROOT_MARK + '\n').encode(), b''
            assert "-exec sha256sum {} +" in cmd, cmd
            lines, sums = [], []
            for dp, dn, fns in os.walk(root):
                rel = os.path.relpath(dp, root).replace(os.sep, '/')
                depth = 0 if rel == '.' else rel.count('/') + 1
                for n in dn:
                    if depth + 1 in (1, 2):
                        lines.append(f"d {n if rel == '.' else rel + '/' + n}")
                for n in fns:
                    if depth + 1 in (1, 2):
                        lines.append(f"f {n if rel == '.' else rel + '/' + n}")
                    if depth + 1 == 2 and n == fs.MANIFEST:
                        p = os.path.join(dp, n)
                        sums.append(f"{fs.sha256_file(p)}  {p}")
            return 0, ('\n'.join(lines + sums) + '\n').encode(), b''
        if toks[0] == 'tar':
            assert toks[1] == '-C' and toks[3:5] == ['-cf', '-'], toks
            if self.tar_missing:
                return 127, b'', b'sh: 1: tar: not found\n'
            buf = io.BytesIO()
            with tarfile.open(fileobj=buf, mode='w') as tf:
                tf.add(os.path.join(self.path(toks[2]), toks[5]), arcname=toks[5])
            return 0, buf.getvalue(), b''
        if toks[0] == 'find':
            base = self.path(toks[1])
            files = [r for r in fs.list_files(base)]
            return 0, ('\n'.join(files) + '\n').encode(), b''
        if toks[0] == 'cat':
            assert toks[1] == '--'
            return 0, open(self.path(toks[2]), 'rb').read(), b''
        raise AssertionError(f"comanda neasteptata: {cmd}")


@pure_python
def test_ssh_comenzile_si_transferul():
    home, loc = tmp(), tmp()
    root = os.path.join(home, 'nova-zdc', 'scoring')
    make_attempt(root, 1)
    make_attempt(root, 2, manifest=False)
    pi = FakePi(home)
    rc, out = run('list', '--local', loc, runner=pi)
    assert rc == 0 and 'INCOMPLETA' in out, out
    list_cmd = pi.calls[0][-1]
    assert list_cmd.startswith('if [ -d "$HOME"/nova-zdc/scoring ]; then find'), list_cmd
    assert "-printf '%y %P\\n'" in list_cmd, list_cmd
    rc, out = run('fetch', '--local', loc, runner=pi)
    assert rc == 0, out
    tar_cmds = [c[-1] for c in pi.calls if c[-1].startswith('tar')]
    assert tar_cmds == ['tar -C "$HOME"/nova-zdc/scoring -cf - Handoff1-touchdown'], tar_cmds
    assert fs.remote_path_expr('~/my scoring') == '"$HOME"/\'my scoring\''
    a = os.path.join(loc, 'Handoff1-touchdown')
    assert fs.verify_dir(a)[0] == [] and os.path.isfile(os.path.join(a, 'burst', '0002_1066.jpg'))
    # quoting of odd roots, other hosts and extra ssh args
    assert fs.remote_path_expr('/srv/nova data/it\'s') == "'/srv/nova data/it'\"'\"'s'"
    assert shlex.split(fs.remote_path_expr('/srv/nova data/it\'s')) == ["/srv/nova data/it's"]
    assert fs.remote_path_expr('~') == '"$HOME"'
    for bad in ('~alt/x', 'a\nb'):
        try:
            fs.remote_path_expr(bad)
            raise AssertionError(bad)
        except fs.SetupError:
            pass
    s = fs.SshSource('pi@10.0.0.5', '/data/scoring', runner=pi, ssh_args=['-i', 'k'])
    assert s.argv('x') == ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                           '-i', 'k', '--', 'pi@10.0.0.5', 'x']
    try:
        fs.SshSource('-oProxyCommand=x')
        raise AssertionError("host cu '-' acceptat")
    except fs.SetupError:
        pass
    rc, out = run('list', '--local', loc, '--host', 'pi@10.0.0.5', '--identity', 'k',
                  '--ssh-arg=-oPort=2222', runner=FakePi(home, host='pi@10.0.0.5'))
    assert rc == 0, out
    # no ssh executable on the PC: clear message, rc 3, no traceback
    rc, out = run('list', '--local', loc, '--ssh', os.path.join(loc, 'nu-exista-ssh'))
    assert rc == 3 and 'OpenSSH Client' in out, out
    # missing remote root -> empty list, not an error
    rc, out = run('list', '--local', loc, '--remote-root', '~/nimic', runner=pi)
    assert rc == 0 and 'nicio captura' in out, out
    shutil.rmtree(home), shutil.rmtree(loc)
    return f"{len(pi.calls)} apeluri ssh, doar find/tar -c/cat; ~ prin \"$HOME\"; BatchMode; '--' inainte de host"


@pure_python
def test_ssh_fara_tar_foloseste_cat():
    home, loc = tmp(), tmp()
    root = os.path.join(home, 'nova-zdc', 'scoring')
    make_attempt(root, 1)
    pi = FakePi(home, tar_missing=True)
    rc, out = run('fetch', '--local', loc, runner=pi)
    assert rc == 0, out
    cats = [c[-1] for c in pi.calls if c[-1].startswith('cat')]
    assert len(cats) == 7, cats          # 2 png + meta + manifest + 3 burst
    shutil.rmtree(home), shutil.rmtree(loc)
    return f"tar lipsa (rc 127) -> find + {len(cats)} x cat; verificat si primit"


def test_watch_reincearca_si_ctrl_c():
    home, loc = tmp(), tmp()
    root = os.path.join(home, 'nova-zdc', 'scoring')
    make_attempt(root, 1)
    pi = FakePi(home, down=2)            # first two cycles: link down
    sleeps = []

    def fake_sleep(s):
        sleeps.append(s)
        if len(sleeps) == 3:
            make_attempt(root, 2)       # lands while watching
        if len(sleeps) >= 5:
            raise KeyboardInterrupt
    prev = fs._USE_CV2
    fs.set_backend(False)
    try:
        rc, out = run('fetch', '--local', loc, '--watch', '2', runner=pi, sleep=fake_sleep)
    finally:
        fs._USE_CV2 = prev
    assert rc == 0, out
    assert out.count('legatura: ssh') == 1, out          # reported once, not per cycle
    assert 'No route to host' in out and 'legatura revenita' in out, out
    assert 'oprit (Ctrl-C)' in out and sleeps == [2.0] * 5, (sleeps, out)
    race = os.path.join(loc, 'handover', f"{DATE}_{SESS}")
    assert sorted(os.listdir(race)) == sorted(['Handoff1-touchdown', 'Handoff2-touchdown',
                                               'README.txt',
                                               fs.HANDOVER_MANIFEST]), os.listdir(race)
    assert run('verify', race)[0] == 0
    # a single fetch with the link down: rc 2, message, no traceback
    pi2 = FakePi(home, down=1)
    rc, out = run('fetch', '--local', tmp(), runner=pi2)
    assert rc == 2 and 'EROARE legatura' in out, out
    shutil.rmtree(home), shutil.rmtree(loc)
    return "2 cicluri fara legatura raportate o data, apoi primite 2 incercari; Ctrl-C curat"


@pure_python
def test_numele_cursei_si_sesiuni_multiple():
    src, loc = tmp(), tmp()
    make_attempt(src, 1, session='s20260927-100000')
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    hr = os.path.join(loc, 'handover')
    assert os.path.isdir(os.path.join(hr, f"{DATE}_s20260927-100000")), os.listdir(hr)
    # new session arrives while --cursa is given: only the new one takes the name
    make_attempt(src, 2, session='s20260927-140000')
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--cursa', 'cursa2')
    assert rc == 0, out
    got = sorted(x for x in os.listdir(hr) if not x.startswith('_'))
    assert got == [f"{DATE}_cursa2", f"{DATE}_s20260927-100000"], got
    # explicit rename of the old one; the name cursa2 is taken -> suffixed
    rc, out = run('fetch', '--source-dir', src, '--local', loc,
                  '--session', 's20260927-100000', '--cursa', 'cursa1')
    assert rc == 0 and 'redenumit' in out, out
    got = sorted(x for x in os.listdir(hr) if not x.startswith('_'))
    assert got == [f"{DATE}_cursa1", f"{DATE}_cursa2"], got
    assert run('verify', os.path.join(hr, f"{DATE}_cursa1"))[0] == 0
    rc, out = run('fetch', '--source-dir', src, '--local', loc,
                  '--session', 's20260927-100000', '--cursa', 'cursa2')
    got = sorted(x for x in os.listdir(hr) if not x.startswith('_'))
    assert f"{DATE}_cursa2_s20260927-100000" in got and f"{DATE}_cursa2" in got, got
    rc, out = run('fetch', '--source-dir', src, '--local', loc, '--session', 's2099',
                  '--cursa', 'x')
    assert rc == 3 and 'nu exista' in out, out
    shutil.rmtree(src), shutil.rmtree(loc)
    return "cursa noua -> doar sesiunea noua; redenumire doar cu --session; coliziune -> sufix sesiune"


@pure_python
def test_numerotarea_reluata_pe_Pi_e_conflict_nu_suprascriere():
    """Daca scoring/ e golit pe Pi, numerotarea reincepe si un Handoff1 nou
    poarta numele unuia deja primit. PC-ul compara sha256 al manifestului
    de pe Pi cu cel local: raporteaza, rc 1, nu suprascrie; dupa ce copia
    veche e mutata, o aduce pe cea noua. Aceeasi verificare si prin ssh."""
    src, loc = tmp(), tmp()
    make_attempt(src, 1, session='s20260927-100000')
    assert run('fetch', '--source-dir', src, '--local', loc)[0] == 0
    vechi = open(os.path.join(loc, 'Handoff1-touchdown', 'meta.json')).read()
    shutil.rmtree(os.path.join(src, 'Handoff1-touchdown'))
    make_attempt(src, 1, session='s20260928-090000')
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 1 and 'ALT continut' in out and 'Nu suprascriu' in out, out
    assert open(os.path.join(loc, 'Handoff1-touchdown', 'meta.json')).read() == vechi
    rc, out = run('list', '--source-dir', src, '--local', loc)
    assert 'CONFLICT' in out, out
    # prin ssh: sha256 vine din listare
    home = tmp()
    shutil.copytree(src, os.path.join(home, 'nova-zdc', 'scoring'))
    rc, out = run('fetch', '--local', loc, runner=FakePi(home))
    assert rc == 1 and 'ALT continut' in out, out
    os.replace(os.path.join(loc, 'Handoff1-touchdown'), os.path.join(tmp(), 'vechi'))
    rc, out = run('fetch', '--source-dir', src, '--local', loc)
    assert rc == 0 and 'primita: Handoff1-touchdown' in out, out
    assert 's20260928-090000' in open(os.path.join(loc, 'Handoff1-touchdown', 'meta.json')).read()
    return "Handoff1 refolosit pe Pi -> CONFLICT, rc 1, copia locala neatinsa (director si ssh)"


TESTS = [
    ('numerotarea reluata pe Pi -> conflict, nu suprascriere',
     test_numerotarea_reluata_pe_Pi_e_conflict_nu_suprascriere),
    ('PNG in Python pur, toate filtrele', test_png_python_pur_toate_filtrele),
    ('clasificarea centrului pe ambele cai', test_clasificarea_centrului_ambele_cai),
    ('list: completa si incompleta', test_listare_completa_si_incompleta),
    ('fetch ignora incercarea incompleta', test_fetch_ignora_incompleta),
    ('sha256 gresit -> carantina, nepredat', test_sha256_gresit_carantina),
    ('fetch repetat fara duplicate', test_fetch_repetat_fara_duplicate),
    ('pachetul de predare si README', test_pachet_de_predare_si_readme),
    ('--fc-log in pachet si in manifest', test_fc_log_in_pachet_si_manifest),
    ('adnotata regenerata cand lipseste', test_adnotata_regenerata_cand_lipseste),
    ('centru ambiguu -> avertisment vizibil', test_centru_ambiguu_avertisment_vizibil),
    ('ssh: comenzile si transferul', test_ssh_comenzile_si_transferul),
    ('ssh fara tar foloseste cat', test_ssh_fara_tar_foloseste_cat),
    ('--watch reincearca si Ctrl-C', test_watch_reincearca_si_ctrl_c),
    ('numele cursei si sesiuni multiple', test_numele_cursei_si_sesiuni_multiple),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                note = fn()
            print(f"  OK    {name}" + (f"   ({note})" if note else ""))
        except AssertionError as e:
            fails += 1
            print(f"  ESEC  {name}\n        {e}")
        except Exception as e:                      # noqa: BLE001
            fails += 1
            import traceback
            traceback.print_exc()
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
