#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru nova/board_window.py (fereastra OSD de bord, punctul 4), fara
GUI: `pv` e un fals cu enabled / show / close, ceasul e injectat.

    python3 tools/test_board_window.py

Ce verificam: marimea OSD (NTSC 720x480, PAL 720x576, refuz pentru valori
proaste), redesenarea la FIECARE cadru nou (frame_seq, sau identitatea lui
last_frame cand frame_seq lipseste) fara dubluri, costul ferestrei masurat
si scris o data la 5 s, presetul camerei si tipul calibrarii in banda.
"""

import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import board_window as bw  # noqa: E402
from nova import config as nova_config  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Ceas:
    """Ceas injectat: avanseaza doar cand ii spunem."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class PV:
    """Previzualizare falsa. `cost_s` = cat "dureaza" show() pe ceasul
    injectat (se adauga cate un element din lista, ciclic)."""

    def __init__(self, ceas=None, cost_s=(0.0,)):
        self.enabled = True
        self.cadre = []
        self.ceas = ceas
        self.cost_s = list(cost_s)
        self.raspuns = True

    def show(self, img):
        if self.ceas is not None:
            self.ceas.t += self.cost_s[len(self.cadre) % len(self.cost_s)]
        self.cadre.append(img)
        return self.raspuns

    def close(self):
        self.enabled = False


class DetFals:
    """Detector fals: frame_seq optional, last_frame, stats."""

    def __init__(self, cu_seq=True):
        if cu_seq:
            self.frame_seq = 0
        self.last_frame = np.full((72, 128), 50, np.uint8)
        self.last_detection = None

    def poll(self, now):
        return []

    def stats(self):
        return {'fps': 29.5, 'detection_rate': 0.9}


def _banda_si_imagine(img):
    """(inaltimea imaginii, inaltimea benzii) gasite pe pixeli: cadrul e
    gri uniform (50), banda e neagra cu text."""
    randuri_gri = np.where((img[:, 2:6, :] == 50).all(axis=(1, 2)))[0]
    ih = int(randuri_gri.max()) + 1
    return ih, img.shape[0] - ih


def test_marimi_ntsc_pal_si_refuz():
    gray = np.full((720, 1280), 50, np.uint8)
    ntsc = bw.compune(gray, None, ['stare'])
    assert ntsc.shape == (480, 720, 3), ntsc.shape
    assert _banda_si_imagine(ntsc) == (405, 75), _banda_si_imagine(ntsc)
    pal = bw.compune(gray, None, ['stare'], size=(720, 576))
    assert pal.shape == (576, 720, 3), pal.shape
    assert _banda_si_imagine(pal) == (501, 75), _banda_si_imagine(pal)
    # conturul se scaleaza pe inaltimea PAL: coltul (640, 360) al unui
    # cadru 1280x720 ajunge la (360, 250.5)
    c = np.array([[600, 300], [680, 300], [680, 360], [600, 360]], np.float32)
    pal_c = bw.compune(gray, c, ['x'], size=[720, 576])
    assert tuple(pal_c[250, 360]) == bw.VERDE, tuple(pal_c[250, 360])
    # compune(None) pe PAL: tot 576
    assert bw.compune(None, None, ['x'], size=(720, 576)).shape == (576, 720, 3)
    for rau in ([720], [720, 480, 3], [720.0, 480], [720, '480'], [True, 480],
                [720, 75], [720, 150], [0, 480], [-720, 480], None, 'NTSC'):
        try:
            bw.osd_size(rau)
        except ValueError as e:
            assert 'osd.size' in str(e), str(e)
            continue
        raise AssertionError(f"osd.size {rau!r} acceptat")
    try:
        bw.FereastraBord(DetFals(), PV(), size=[720, 60])
    except ValueError:
        pass
    else:
        raise AssertionError("FereastraBord a acceptat 720x60")
    f = bw.FereastraBord(DetFals(), PV(), size=[720, 576])
    f.poll(0.0)
    assert f._pv.cadre[0].shape == (576, 720, 3)
    return "NTSC 405+75, PAL 501+75, 11 marimi proaste refuzate"


def test_config_osd_implicit_si_in_fisier():
    assert nova_config.DEFAULTS['osd'] == {'size': [720, 480]}
    cfg = nova_config.load(os.path.join(REPO, 'config', 'nova.json'))
    assert bw.osd_size(cfg['osd']['size']) in ((720, 480), (720, 576))
    src = open(os.path.join(REPO, 'tools', 'nova_pi.py')).read()
    assert "size=osd_size" in src and "get('size', [720, 480])" in src
    return "DEFAULTS [720, 480]; nova.json valid; nova_pi il paseaza"


def test_redesenare_la_fiecare_cadru_frame_seq():
    det = DetFals(cu_seq=True)
    pv = PV()
    f = bw.FereastraBord(det, pv, log=None)
    # bucla principala la 500 Hz timp de 1 s, camera la 30 cadre/s: fiecare
    # cadru apare la al ~17-lea poll. 30 de valori distincte -> 30 desene.
    vazute = set()
    for i in range(500):
        det.frame_seq = (i * 30) // 500
        vazute.add(det.frame_seq)
        f.poll(1000.0 + i * 0.002)
    assert len(pv.cadre) == len(vazute) == 30, (len(pv.cadre), len(vazute))
    # peste 10 Hz: nu mai exista plafon. 200 de cadre in 1 s -> 200 desene
    n0 = len(pv.cadre)
    for i in range(1000):
        det.frame_seq = 1000 + i // 5
        f.poll(2000.0 + i * 0.001)
    assert len(pv.cadre) - n0 == 200, len(pv.cadre) - n0
    # acelasi frame_seq, oricate poll-uri: niciun desen in plus
    n1 = len(pv.cadre)
    for i in range(50):
        f.poll(3000.0 + i)
    assert len(pv.cadre) == n1, "acelasi cadru desenat de doua ori"
    assert f.n_afisate == len(pv.cadre)
    return "30 cadre -> 30 desene; 200/s -> 200, fara plafon; zero dubluri"


def test_redesenare_fara_frame_seq_pe_identitate():
    det = DetFals(cu_seq=False)
    pv = PV()
    f = bw.FereastraBord(det, pv, log=None)
    for i in range(10):
        f.poll(float(i))
    assert len(pv.cadre) == 1, "acelasi last_frame desenat de mai multe ori"
    cadre = [np.full((72, 128), 50 + k, np.uint8) for k in range(7)]
    for c in cadre:
        det.last_frame = c
        for i in range(4):
            f.poll(100.0 + i)
    assert len(pv.cadre) == 8, len(pv.cadre)
    # acelasi continut, alt tablou: e alt cadru (identitate, nu egalitate)
    det.last_frame = cadre[-1].copy()
    f.poll(200.0)
    assert len(pv.cadre) == 9
    # fara cadru deloc: un singur desen (banda), apoi nimic
    det2 = DetFals(cu_seq=False)
    det2.last_frame = None
    pv2 = PV()
    f2 = bw.FereastraBord(det2, pv2, log=None)
    for i in range(20):
        f2.poll(float(i))
    assert len(pv2.cadre) == 1 and pv2.cadre[0].shape == (480, 720, 3)
    return "7 tablouri noi -> 7 desene; acelasi tablou -> niciunul"


def test_costul_ferestrei_masurat_la_5s():
    ceas = Ceas(1000.0)
    # show() "costa" 2, 4, 6 ... 20 ms, ciclic
    pv = PV(ceas, cost_s=[k * 0.002 for k in range(1, 11)])
    det = DetFals(cu_seq=True)
    linii = []
    f = bw.FereastraBord(det, pv, clock=ceas, log=linii.append)
    # 30 cadre/s timp de 5 s si ceva: cate un poll cu cadru nou la 1/30 s
    # (costul desenului e INCLUS in acest pas, ca pe bord)
    t_pas = 1.0 / 30
    i = 0
    while not linii:
        t_inainte = ceas.t
        det.frame_seq += 1
        f.poll(ceas.t)
        ceas.t = t_inainte + t_pas
        i += 1
        assert i < 1000, "linia de cost nu a aparut"
    assert len(linii) == 1
    w = f.ultima_fereastra
    # Fereastra incepe la primul desen si se inchide la primul desen care o
    # depaseste: 150 de pasi de 1/30 s = 5 s, deci al 151-lea desen o
    # inchide (la 5 s + costul lui, 2 ms). Desenele numarate = cele din pv.
    assert w['drawn'] == 151 == len(pv.cadre), (w, len(pv.cadre))
    assert abs(w['s'] - 5.002) < 1e-6, w
    assert abs(w['rate'] - 151 / 5.002) < 1e-9, w
    # costuri: 2..20 ms ciclic (15 cicluri + inca un 2 ms). Mediana =
    # elementul 76 din 151 sortate = 10 ms; p99 (rang, 150 din 151) = 20 ms
    assert abs(w['p50_ms'] - 10.0) < 1e-6, w
    assert abs(w['p99_ms'] - 20.0) < 1e-6, w
    l = linii[0]
    assert l.startswith('[fereastra 5s] cadre afisate 151 (30.2/s)'), l
    assert 'p50 10.0 ms' in l and 'p99 20.0 ms' in l, l
    # o singura linie pe fereastra: inca ~5.3 s de cadre -> exact inca una
    for _ in range(160):
        det.frame_seq += 1
        t_inainte = ceas.t
        f.poll(ceas.t)
        ceas.t = t_inainte + t_pas
    assert len(linii) == 2, linii
    # fara cadre noi, fereastra se inchide oricum si spune 0: o fereastra
    # "inghetata" (camera oprita) se vede in log
    ceas.t += 5.1
    f.poll(ceas.t)                  # inchide fereastra cu cele ~10 desene
    assert len(linii) == 3
    ceas.t += 5.1
    f.poll(ceas.t)
    assert len(linii) == 4 and f.ultima_fereastra['drawn'] == 0, linii[-1]
    assert 'cadre afisate 0 (0.0/s)' in linii[-1] and 'p50 - ms' in linii[-1]
    return l


def test_banda_preset_si_calibrare():
    det = DetFals()
    det.source = types.SimpleNamespace(settings=types.SimpleNamespace(preset='crop1280'))
    det.det = types.SimpleNamespace(calib=types.SimpleNamespace(kind='derivata+scalata'),
                                    last_corners=None)
    sm = types.SimpleNamespace(state='DESCEND_TRACK', h_now=lambda: 8.5,
                               last_est=types.SimpleNamespace(t=100.0, lateral_m=0.83),
                               aux_high=lambda: True, n_vpe_sent=1234)
    f = bw.FereastraBord(det, PV(), sm=sm, log=None)
    linie = f.linia_de_stare(100.5)
    assert linie.endswith('crop1280 der+scal'), linie
    for parte in ('DESCEND_TRACK', 'h  8.5m', 'lat 0.83m', 'AUX SUS', 'VPE 1234',
                  'cam 29.5fps', 'det 90%'):
        assert parte in linie, (parte, linie)
    # cazul lung obisnuit incape INTREG in banda (taierea e pe pixeli)
    assert bw._scurt(linie) == linie, bw._scurt(linie)
    for kind, scurt in (('nativa', 'nat'), ('scalata', 'scal'), ('derivata', 'der')):
        det.det.calib.kind = kind
        assert f.linia_de_stare(100.5).endswith(f"crop1280 {scurt}")
    # lipsa: '-' pentru fiecare
    gol = DetFals()
    f2 = bw.FereastraBord(gol, PV(), sm=sm, log=None)
    assert f2.linia_de_stare(100.5).endswith(' - -'), f2.linia_de_stare(100.5)
    gol.source = types.SimpleNamespace(settings=types.SimpleNamespace(preset=None))
    gol.det = types.SimpleNamespace(calib=None)
    assert f2.camera_si_calibrare() == '- -'
    gol.source = 'nu are settings'
    gol.det = types.SimpleNamespace(calib=types.SimpleNamespace(kind='nativa'))
    assert f2.camera_si_calibrare() == '- nat'
    # o linie prea lunga se taie pe latime, cu '~', si nu iese din banda
    lung = linie + ' ' + 'W' * 40
    t = bw._scurt(lung)
    assert t.endswith('~') and t.startswith('DESCEND_TRACK')
    assert bw._latime(t) <= bw.OSD_W - 2 * bw.TEXT_X
    return linie


def test_banda_ramane_vie_fara_cadre_noi():
    """Camera blocata (frame_seq nu se mai schimba): banda - starea si
    evenimentele, de ex. motivul unui EXIT - se redeseneaza o data pe
    secunda, nu ingheata. Numarata separat de desenele pe cadru, in linia
    de cost; cu cadre noi nu apare."""
    det = DetFals(cu_seq=True)
    pv = PV()
    ceas = Ceas()
    linii = []
    f = bw.FereastraBord(det, pv, clock=ceas, log=linii.append)
    det.frame_seq = 1
    f.poll(0.0)
    assert len(pv.cadre) == 1
    for i in range(350):                       # 3.5 s, fara cadru nou
        ceas.t += 0.01
        f.poll(0.0)
    assert f.n_reimprospatari == 3 and len(pv.cadre) == 4, (f.n_reimprospatari, len(pv.cadre))
    assert f.n_afisate == 1, "reimprospatarile numarate ca cadre"
    for i in range(200):                       # 2 s cu cadre noi la 50/s
        ceas.t += 0.01
        if i % 2 == 0:
            det.frame_seq += 1
        f.poll(0.0)
    assert f.n_reimprospatari == 3, "reimprospatare cu cadre noi"
    ceas.t += 5.0
    f.poll(0.0)
    assert any('reimprospatari FARA cadru nou' in l for l in linii), linii
    return f"3.5 s fara cadre -> 3 reimprospatari (1/s); cu cadre -> 0; spuse in log"


TESTS = [
    ('banda ramane vie fara cadre noi', test_banda_ramane_vie_fara_cadre_noi),
    ('marimi OSD: NTSC, PAL, refuz', test_marimi_ntsc_pal_si_refuz),
    ('config osd.size: implicit, fisier, nova_pi', test_config_osd_implicit_si_in_fisier),
    ('redesenare la fiecare cadru (frame_seq), fara plafon',
     test_redesenare_la_fiecare_cadru_frame_seq),
    ('redesenare fara frame_seq: identitatea last_frame',
     test_redesenare_fara_frame_seq_pe_identitate),
    ('costul ferestrei: linie la 5 s, p50/p99', test_costul_ferestrei_masurat_la_5s),
    ('banda: presetul camerei si calibrarea', test_banda_preset_si_calibrare),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            note = fn()
            print(f"  OK    {name}" + (f"   ({note})" if note else ""))
        except AssertionError as e:
            fails += 1
            print(f"  ESEC  {name}\n        {e}")
        except Exception as e:                      # noqa: BLE001
            fails += 1
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
