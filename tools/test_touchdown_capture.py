#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Captura de touchdown pe Pi (8.3.3, 6.2.1.30): nova/touchdown_capture.py.

    python3 tools/test_touchdown_capture.py

Ce conteaza: cadrul ales e cel dinaintea momentului in care Pi-ul a primit
ON_GROUND; fisierele sunt atomice; manifestul e scris ULTIMUL (fara el
incercarea e incompleta si PC-ul nu o ia); captura ramane si daca urcarea
esueaza; formatul e cel pe care il citeste tools/fetch_scoring.py (verificat
cap-coada, cu scriptul de PC).
"""

import contextlib
import hashlib
import io
import json
import os
import sys
import tempfile
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from nova import touchdown_capture as tc                        # noqa: E402
from nova.frame_ring import FrameRing                           # noqa: E402

W, H = 320, 180


def cadru(t, center=0):
    """A gray frame whose top-left pixels encode t (ms) - to know which one
    was saved - and whose centre is `center`."""
    f = np.full((H, W), 128, np.uint8)
    ms = int(round(t * 1000))
    f[0, 0:4] = [(ms >> 24) & 255, (ms >> 16) & 255, (ms >> 8) & 255, ms & 255]
    f[H // 2 - 6:H // 2 + 7, W // 2 - 6:W // 2 + 7] = center
    return f


def t_din(img):
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    b = [int(x) for x in g[0, 0:4]]
    return ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]) / 1000.0


def ring_cu(t0, t1, dt=1 / 30.0, center=0):
    r = FrameRing(400)
    t = t0
    while t <= t1 + 1e-9:
        r.push(cadru(t, center), t)
        t += dt
    return r


def contact(t_rx, **kw):
    info = {'t_on_ground_rx': t_rx, 'fc_time_boot_ms': 123456,
            'fc_time_unix_usec': 1790000000000000, 'h_ref': 0.03,
            'z_contact': 0.0, 'last_det_age_s': 1.9, 'last_lateral_m': 0.04,
            'td_start_t': t_rx - 4.0}
    info.update(kw)
    return info


def cam():
    return {'preset': 'crop1280', 'sensor_mode': [1536, 864],
            'scaler_crop': [768, 432, 3072, 1728], 'calibration': 'derivata+scalata'}


def test_cadrul_de_dinaintea_ON_GROUND():
    """Cadrul cu t_capture cel mai apropiat, dar ANTERIOR, momentului in
    care s-a primit ON_GROUND; nemodificat (aceeasi marime, aceiasi
    pixeli). Seria acopera de la inceputul coborarii (~1 m) la contact+1 s."""
    root = tempfile.mkdtemp()
    ring = ring_cu(100.0, 108.0)
    cap = tc.TouchdownCapture(root, ring, camera_info=cam, threaded=False,
                              clock=lambda: 0.0,
                              meta_for=lambda t: {'ExposureTime': 1500, 'AnalogueGain': 2.0,
                                                  'LensPosition': 1.63})
    cap.on_contact(contact(105.05))
    cap.update(105.9)
    assert cap.done == [], "seria nu se ia inainte de contact + 1 s"
    cap.update(106.1)
    d, ok, why = cap.done[0]
    assert ok, why
    img = cv2.imread(os.path.join(d, 'touchdown.png'), cv2.IMREAD_UNCHANGED)
    t = t_din(img)
    assert t <= 105.05 and 105.05 - t < 1 / 30.0 + 1e-3, t
    ref = [f for tt, f in ring.buf if abs(tt - t) < 1e-3][0]       # t in ms, in pixels
    assert img.shape == ref.shape and np.array_equal(img, ref), "imaginea modificata"
    meta = json.load(open(os.path.join(d, 'meta.json')))
    assert meta['sync']['t_capture'] <= meta['sync']['t_on_ground_rx']
    assert meta['burst']['t_from'] <= 101.1 and meta['burst']['t_to'] >= 106.0, meta['burst']
    assert meta['camera']['ExposureTime'] == 1500 and meta['camera']['preset'] == 'crop1280'
    return f"cadrul de la t={t:.3f} (ON_GROUND primit la 105.050); {meta['burst']['files']} in serie"


def test_fisiere_atomice_si_manifestul_ultimul():
    root = tempfile.mkdtemp()
    ordine = []
    vechi = os.replace

    def spion(a, b):
        ordine.append(os.path.basename(b))
        return vechi(a, b)
    tc.os.replace = spion
    try:
        cap = tc.TouchdownCapture(root, ring_cu(10.0, 14.0), camera_info=cam,
                                  threaded=False, clock=lambda: 0.0)
        cap.on_contact(contact(12.0))
        cap.update(20.0)
    finally:
        tc.os.replace = vechi
    d = cap.done[0][0]
    assert ordine[-1] == 'MANIFEST.sha256', ordine[-3:]
    assert ordine.index('meta.json') < ordine.index('MANIFEST.sha256')
    rest = [p for dp, _, fs in os.walk(d) for p in fs if '.tmp' in p]
    assert rest == [], rest
    linii = open(os.path.join(d, 'MANIFEST.sha256')).read().splitlines()
    fisiere = sorted(os.path.relpath(os.path.join(dp, f), d).replace(os.sep, '/')
                     for dp, _, fs in os.walk(d) for f in fs if f != 'MANIFEST.sha256')
    assert [ln.split('  ', 1)[1] for ln in linii] == fisiere, "manifestul nu acopera tot"
    for ln in linii:
        h, p = ln.split('  ', 1)
        assert hashlib.sha256(open(os.path.join(d, p), 'rb').read()).hexdigest() == h, p
    return f"{len(ordine)} redenumiri atomice, manifestul ultimul, {len(linii)} fisiere verificate"


def test_esec_la_scriere_lasa_incercarea_incompleta():
    """O eroare la jumatatea scrierii: fara manifest (PC-ul nu o ia),
    eveniment CAPTURA ESUATA pe OSD, din firul principal."""
    root = tempfile.mkdtemp()
    ev = []
    cap = tc.TouchdownCapture(root, ring_cu(10.0, 14.0), camera_info=cam,
                              threaded=False, clock=lambda: 0.0,
                              on_event=lambda n, i: ev.append((n, i)))
    vechi = tc.cv2.imencode if hasattr(tc, 'cv2') else None
    import cv2 as _cv2
    real = _cv2.imencode

    def rau(ext, img, *a):
        if ext == '.jpg':
            raise OSError(28, 'No space left on device')
        return real(ext, img, *a)
    _cv2.imencode = rau
    try:
        cap.on_contact(contact(12.0))
        cap.update(20.0)
    finally:
        _cv2.imencode = real
    d, ok, why = cap.done[0]
    assert not ok and 'No space' in why
    assert not os.path.exists(os.path.join(d, 'MANIFEST.sha256'))
    cap.update(21.0)
    assert ev and ev[0][0] == 'capture_failed' and 'No space' in ev[0][1]['reason'], ev
    # niciun cadru inainte de ON_GROUND -> esec spus, nu exceptie
    cap2 = tc.TouchdownCapture(tempfile.mkdtemp(), ring_cu(50.0, 51.0), threaded=False,
                               clock=lambda: 0.0, on_event=lambda n, i: ev.append((n, i)))
    cap2.on_contact(contact(40.0))
    cap2.update(60.0)
    cap2.update(60.0)
    assert ev[-1][0] == 'capture_failed' and 'niciun cadru' in ev[-1][1]['reason'], ev[-1]
    return "disc plin la serie -> fara manifest + CAPTURA ESUATA; fara cadru -> motiv"


def test_centrul_culoarea_si_adnotarea():
    root = tempfile.mkdtemp()
    rez = {}
    for center, cls in ((5, 'negru'), (250, 'alb'), (128, 'ambiguu')):
        cap = tc.TouchdownCapture(tempfile.mkdtemp(), ring_cu(1.0, 3.0, center=center),
                                  threaded=False, clock=lambda: 0.0)
        cap.on_contact(contact(2.0))
        cap.update(9.0)
        meta = json.load(open(os.path.join(cap.done[0][0], 'meta.json')))
        assert meta['center']['class'] == cls, (center, meta['center'])
        rez[cls] = meta['center']['mean']
    # culoare: color_for da BGR -> PNG color, adnotarea nu atinge originalul
    ring = ring_cu(1.0, 3.0, center=5)
    cap = tc.TouchdownCapture(root, ring, threaded=False, clock=lambda: 0.0,
                              color_for=lambda t, g: cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    cap.on_contact(contact(2.0))
    cap.update(9.0)
    d = cap.done[0][0]
    img = cv2.imread(os.path.join(d, 'touchdown.png'), cv2.IMREAD_UNCHANGED)
    ann = cv2.imread(os.path.join(d, 'touchdown_annotated.png'), cv2.IMREAD_UNCHANGED)
    meta = json.load(open(os.path.join(d, 'meta.json')))
    assert img.ndim == 3 and meta['image']['color'] is True
    diff = np.any(img != ann, axis=2)
    assert diff.any(), "fara cruce"
    assert not diff[H // 2 - 2:H // 2 + 3, W // 2 - 2:W // 2 + 3].any(), "crucea acopera centrul 5x5"
    return f"negru {rez['negru']} / alb {rez['alb']} / ambiguu; color; crucea lasa centrul"


def test_firul_de_scriere_nu_blocheaza():
    """Firul principal doar preda: update() se intoarce imediat chiar cu o
    serie mare de codificat; scrierea se face in firul 'captura'."""
    root = tempfile.mkdtemp()
    ring = FrameRing(400)
    big = np.random.default_rng(0).integers(0, 255, (720, 1280), dtype=np.uint8)
    for k in range(180):
        ring.push(big, 10.0 + k / 30.0)
    cap = tc.TouchdownCapture(root, ring, camera_info=cam, clock=time.monotonic)
    cap.on_contact(contact(14.0))
    t0 = time.perf_counter()
    cap.update(20.0)
    dt = time.perf_counter() - t0
    assert dt < 0.05, f"update() a durat {dt * 1000:.0f} ms"
    cap.stop(60.0)
    assert cap.done and cap.done[0][1], cap.done
    assert cap._thread.name == 'captura' and not cap._thread.is_alive()
    return f"update() {dt * 1000:.1f} ms cu {len(ring)} cadre 1280x720 de scris"


def test_captura_ramane_cand_urcarea_esueaza():
    """Cap-coada cu masina de stari: contactul programeaza captura; o
    urcare ratata (EXIT) dupa aceea nu o anuleaza."""
    from nova.extnav_landing import Phase
    import test_extnav_landing as tel
    root = tempfile.mkdtemp()
    app = tel.App(h=6.0, p=(1.0, -0.5), rate=0.3)
    ring = FrameRing(400)
    cap = tc.TouchdownCapture(root, ring, camera_info=cam, threaded=False,
                              clock=lambda: 1000.0 + app.t)

    def on_event(n, i):
        app.events.append((n, i))
        if n == 'contact':
            cap.on_contact(i)
    app.sm.on_event = on_event

    def pas(a):
        now = 1000.0 + a.t
        ring.push(cadru(now, center=5 if a.v.h < 0.05 else 128), now)
        if a.sm.state == Phase.RISEUP:
            a.v.climb_ms = 0.2          # urcare prea lenta -> timeout -> EXIT
        cap.update(now)
    app.handover()
    st = app.run(200.0, stop=(Phase.ABORT, Phase.DONE), on_step=pas)
    assert st == Phase.ABORT and 'urcarea' in app.sm.exit_reason, (st, app.sm.exit_reason)
    assert cap.done and cap.done[0][1], cap.done
    d = cap.done[0][0]
    meta = json.load(open(os.path.join(d, 'meta.json')))
    assert os.path.exists(os.path.join(d, 'MANIFEST.sha256'))
    assert meta['center']['class'] == 'negru', meta['center']
    return f"EXIT pe urcare, captura intreaga: {os.path.relpath(d, root)}, centru {meta['center']['class']}"


def test_PC_preia_ce_scrie_Pi():
    """Formatul, verificat din ambele capete: tools/fetch_scoring.py (PC)
    preia, cu sha256, incercarea scrisa de nova/touchdown_capture (Pi) si
    face pachetul de predare cu README."""
    import fetch_scoring as fs
    pi = tempfile.mkdtemp()
    cap = tc.TouchdownCapture(pi, ring_cu(10.0, 14.0, center=5), camera_info=cam,
                              threaded=False, clock=lambda: 0.0)
    cap.on_contact(contact(12.0))
    cap.update(20.0)
    assert cap.done[0][1]
    local = tempfile.mkdtemp()
    lines = []
    rc = fs.main(['fetch', '--source-dir', pi, '--local', local],
                 out=lambda *a, **k: lines.append(' '.join(str(x) for x in a)))
    text = '\n'.join(lines)
    assert rc == 0, text
    pachet = [dp for dp, _, fsn in os.walk(os.path.join(local, 'handover'))
              if 'README.txt' in fsn and os.path.basename(dp).startswith('attempt_')]
    assert pachet, text
    readme = open(os.path.join(pachet[0], 'README.txt')).read()
    assert '123456' in readme and 'negru' in readme, readme[:400]
    assert fs.main(['fetch', '--source-dir', pi, '--local', local],
                   out=lambda *a, **k: None) == 0
    return f"PC: preluat + pachet {os.path.relpath(pachet[0], local)}; a doua rulare fara nimic nou"


def test_piesele_capturii_metadate_culoare_ora_FC():
    """Metadatele cadrului salvat (nu ale celui mai nou), culoarea din YUV-ul
    pastrat de sursa (stub picamera2), ora GPS/UTC a FC-ului la contact
    extrapolata din SYSTEM_TIME cu time_boot_ms (doar etichetare)."""
    import types
    from nova.detector_pi import (ArraySource, ArucoMarkerDetector, PiCameraSource,
                                  PiDetector, camera_settings)
    import test_detector_pi as tdp
    from test_camera_geometry import fake_camera, quiet
    # metadate dupa timpul capturii
    class Sursa(ArraySource):
        def read(self):
            it = super().read()
            if it is not None:
                self.last_metadata = {'ExposureTime': int(it[1] * 1000)}
            return it
    ts = [5.0, 5.1, 5.2]
    pd = PiDetector(Sursa([np.zeros((H, W), np.uint8)] * 3, timestamps=ts),
                    ArucoMarkerDetector(tdp.synthetic_calibration()), threaded=False)
    pd.window_log = False
    for _ in range(3):
        pd.poll(0.0)
    assert pd.metadata_at(5.1) == {'ExposureTime': 5100} and pd.metadata_at(6.0) is None
    # culoare din crominanta pastrata (I420 -> BGR), doar cand e ceruta
    with fake_camera(), quiet():
        src = PiCameraSource(settings=camera_settings({}, 'crop1280'), keep_color=True)
        gray, t = src.read()
        bgr = src.color_for(t)
        src.close()
        src2 = PiCameraSource(settings=camera_settings({}, 'crop1280'))
        g2, t2 = src2.read()
        assert src2.color_for(t2) is None and len(src2.chroma_ring) == 0
        src2.close()
    assert bgr is not None and bgr.shape == (720, 1280, 3), None if bgr is None else bgr.shape
    assert src.color_for(t + 1.0) is None
    # ora FC: SYSTEM_TIME + time_boot_ms
    from nova.vehicle import Vehicle
    v = Vehicle('udpin:127.0.0.1:1')
    assert v.fc_unix_usec_now() is None
    msg = types.SimpleNamespace(time_unix_usec=1790000000000000, time_boot_ms=10000,
                                get_type=lambda: 'SYSTEM_TIME')
    v._handle(msg)
    v.time_boot_ms = 12500
    assert v.fc_unix_usec_now() == 1790000000000000 + 2500 * 1000
    return "metadate dupa t_capture; BGR 1280x720 din I420; ora FC extrapolata 2.5 s"


def test_deploy_nu_sterge_capturile_de_pe_Pi():
    """pi/deploy.sh face rsync --delete: scoring/ (scris pe Pi) trebuie
    exclus, altfel fiecare deploy ar sterge dovezile 8.3.3."""
    d = open(os.path.join(REPO, 'pi', 'deploy.sh')).read()
    assert "--delete" in d and "--exclude '/scoring/'" in d, "scoring/ neexclus din rsync --delete"
    return "rsync --delete cu scoring/ exclus"


TESTS = [
    ('deploy nu sterge capturile de pe Pi', test_deploy_nu_sterge_capturile_de_pe_Pi),
    ('piesele capturii: metadate, culoare, ora FC',
     test_piesele_capturii_metadate_culoare_ora_FC),
    ('cadrul de dinaintea ON_GROUND', test_cadrul_de_dinaintea_ON_GROUND),
    ('fisiere atomice, manifestul ultimul', test_fisiere_atomice_si_manifestul_ultimul),
    ('esec la scriere -> incompleta, spus pe OSD',
     test_esec_la_scriere_lasa_incercarea_incompleta),
    ('centrul, culoarea si adnotarea', test_centrul_culoarea_si_adnotarea),
    ('firul de scriere nu blocheaza', test_firul_de_scriere_nu_blocheaza),
    ('captura ramane cand urcarea esueaza', test_captura_ramane_cand_urcarea_esueaza),
    ('PC-ul preia ce scrie Pi-ul', test_PC_preia_ce_scrie_Pi),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            with contextlib.redirect_stdout(io.StringIO()) as o:
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
