#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru sursa de cadre din Gazebo (I3).

    python3 tools/test_gz_source.py

Fara Gazebo si fara gz-transport: nodul si mesajele sunt false, iar cadrele
se injecteaza direct in callback. Asa suita ruleaza si pe masini unde
gz-transport nu exista - inclusiv in venv-ul de dezvoltare.

Ce conteaza cel mai mult, in ordine:

  - ceasul e cel de SIMULARE, din antetul mesajului. Cu ceasul de perete, o
    rulare accelerata ar face `now - det.t` sa compare doua lumi.
  - `read()` nu intoarce de doua ori acelasi cadru, si nu se blocheaza la
    infinit cand simularea se opreste.
  - cand detectia nu tine pasul, se pierde cadrul VECHI, nu cel nou.
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                          # noqa: E402

from nova import detector_pi as dp                          # noqa: E402


# --- gz fals ----------------------------------------------------------------

class FakeTime:
    def __init__(self, sec=0, nsec=0):
        self.sec, self.nsec = sec, nsec


class FakeHeader:
    def __init__(self, t_sim):
        self.stamp = FakeTime(int(t_sim), int(round((t_sim % 1) * 1e9)))


class FakeImage:
    """Cat din gz.msgs.Image foloseste _image_to_gray."""

    def __init__(self, gray=None, t_sim=0.0, fmt=1, data=None,
                 w=None, h=None):
        if gray is not None:
            h, w = gray.shape[:2]
            data = gray.tobytes()
        self.width, self.height = w, h
        self.data = data
        self.pixel_format_type = fmt
        self.header = FakeHeader(t_sim)


class FakeNode:
    """Retine callback-ul, ca testul sa poata injecta mesaje."""

    ultimul = None

    def __init__(self):
        self.cb = None
        self.topic = None
        self.ok = True
        FakeNode.ultimul = self

    def subscribe(self, _msg_type, topic, cb):
        self.topic, self.cb = topic, cb
        return self.ok


def cu_gz_fals(fn, subscribe_ok=True):
    """Ruleaza `fn` cu _gz_imports inlocuit."""
    vechi = dp._gz_imports

    def fals():
        def face_nod():
            n = FakeNode()
            n.ok = subscribe_ok
            return n
        return face_nod, FakeImage
    try:
        dp._gz_imports = fals
        return fn()
    finally:
        dp._gz_imports = vechi


def sursa(**kw):
    kw.setdefault('verbose', False)
    return dp.GazeboFrameSource(**kw)


def cadru(val=128, w=64, h=48):
    return np.full((h, w), val, np.uint8)


# --- conversie --------------------------------------------------------------

def test_conversia_formatelor():
    g = np.arange(48 * 64, dtype=np.uint8).reshape(48, 64)
    out = dp._image_to_gray(FakeImage(g, fmt=1))
    assert out.shape == (48, 64) and (out == g).all()

    # RGB si BGR se convertesc, nu se refuza
    rgb = np.zeros((10, 12, 3), np.uint8)
    rgb[:, :, 0] = 255                       # rosu
    msg = FakeImage(fmt=3, data=rgb.tobytes(), w=12, h=10)
    gr = dp._image_to_gray(msg)
    assert gr.shape == (10, 12)
    msg_b = FakeImage(fmt=9, data=rgb.tobytes(), w=12, h=10)
    gb = dp._image_to_gray(msg_b)
    # acelasi buffer citit ca RGB vs BGR da luminante diferite
    assert int(gr[0, 0]) != int(gb[0, 0]), (gr[0, 0], gb[0, 0])

    # format necunoscut: mesaj care spune ce sa faci
    try:
        dp._image_to_gray(FakeImage(fmt=99, data=b'\x00' * 120, w=12, h=10))
        raise AssertionError('format necunoscut acceptat')
    except ValueError as e:
        assert 'L8' in str(e), e

    # buffer prea scurt: prins, nu reshape care crapa obscur
    try:
        dp._image_to_gray(FakeImage(fmt=1, data=b'\x00' * 10, w=64, h=48))
        raise AssertionError('buffer scurt acceptat')
    except ValueError as e:
        assert 'octeti' in str(e), e
    return "L8, RGB, BGR; format necunoscut si buffer scurt -> ValueError"


def test_conversia_copiaza_bufferul():
    """Buffer-ul protobuf poate fi reciclat sub noi; cadrul trebuie sa fie
    al nostru."""
    date = bytearray(b'\x7f' * (10 * 12))
    msg = FakeImage(fmt=1, data=bytes(date), w=12, h=10)
    out = dp._image_to_gray(msg)
    assert out.flags['OWNDATA'] or out.base is None or out.flags['C_CONTIGUOUS']
    val = int(out[0, 0])
    date[:] = b'\x00' * len(date)
    assert int(out[0, 0]) == val, "cadrul s-a schimbat odata cu bufferul"
    return "cadrul e o copie, nu o vedere peste protobuf"


# --- ceas -------------------------------------------------------------------

def test_ceasul_e_cel_de_simulare():
    """Cerinta explicita din I3: timpul de simulare, nu cel de perete."""
    def corp():
        s = sursa(clock='sim')
        n = FakeNode.ultimul
        n.cb(FakeImage(cadru(), t_sim=12.5))
        _g, t = s.read()
        assert abs(t - 12.5) < 1e-6, t
        n.cb(FakeImage(cadru(), t_sim=12.5 + 1 / 30))
        _g2, t2 = s.read()
        assert abs((t2 - t) - 1 / 30) < 1e-6, t2 - t
        assert abs(s.sim_time() - t2) < 1e-9
        s.close()
        return t2 - t
    dt = cu_gz_fals(corp)

    def corp_perete():
        s = sursa(clock='wall')
        n = FakeNode.ultimul
        n.cb(FakeImage(cadru(), t_sim=999.0))
        _g, t = s.read()
        s.close()
        # ceasul de perete NU ia valoarea din mesaj
        assert abs(t - 999.0) > 1.0, t
        assert abs(t - time.monotonic()) < 5.0, t
    cu_gz_fals(corp_perete)

    try:
        cu_gz_fals(lambda: sursa(clock='altceva'))
        raise AssertionError('ceas necunoscut acceptat')
    except ValueError:
        pass
    return f"sim: dt {dt * 1000:.1f} ms din antet; wall: ceasul local; alt ceas respins"


def test_fps_dedus_din_timpul_de_simulare():
    def corp():
        s = sursa()
        n = FakeNode.ultimul
        for i in range(5):
            n.cb(FakeImage(cadru(), t_sim=100.0 + i / 30.0))
            s.read()
        st = s.stats()
        s.close()
        return st
    st = cu_gz_fals(corp)
    assert abs(st['fps'] - 30.0) < 0.5, st['fps']
    assert st['received'] == 5
    return f"{st['fps']:.1f} fps dedus din antet, nu presupus"


# --- comportamentul lui read() ----------------------------------------------

def test_read_nu_repeta_acelasi_cadru():
    def corp():
        s = sursa(timeout_s=0.2)
        n = FakeNode.ultimul
        n.cb(FakeImage(cadru(7), t_sim=1.0))
        g1, _t = s.read()
        assert int(g1[0, 0]) == 7
        # niciun cadru nou: read() asteapta si intoarce None
        t0 = time.monotonic()
        assert s.read() is None
        asteptat = time.monotonic() - t0
        assert 0.15 < asteptat < 1.0, asteptat
        s.close()
        return asteptat
    dt = cu_gz_fals(corp)
    return f"al doilea read fara cadru nou: None dupa {dt:.2f} s"


def test_read_se_deblocheaza_la_close():
    """Fara asta, o bucla care se inchide ar astepta timeout-ul intreg."""
    def corp():
        s = sursa(timeout_s=30.0)
        rezultat = {}

        def cititor():
            rezultat['val'] = s.read()
        th = threading.Thread(target=cititor, daemon=True)
        t0 = time.monotonic()
        th.start()
        time.sleep(0.1)
        s.close()
        th.join(timeout=3.0)
        dt = time.monotonic() - t0
        assert not th.is_alive(), "read() a ramas blocat dupa close()"
        assert rezultat['val'] is None
        assert dt < 2.0, dt
        return dt
    dt = cu_gz_fals(corp)
    return f"close() deblocheaza read() in {dt:.2f} s, nu dupa timeout de 30 s"


def test_se_pierde_cadrul_VECHI_nu_cel_nou():
    """Cand detectia nu tine pasul, vrem sa lucreze pe cel mai nou cadru."""
    def corp():
        s = sursa()
        n = FakeNode.ultimul
        for i, val in enumerate((10, 20, 30)):
            n.cb(FakeImage(cadru(val), t_sim=1.0 + i))
        g, t = s.read()
        st = s.stats()
        s.close()
        return int(g[0, 0]), t, st
    val, t, st = cu_gz_fals(corp)
    assert val == 30, f"a intors cadrul cu {val}, nu cel mai nou"
    assert abs(t - 3.0) < 1e-6, t
    assert st['dropped'] == 2, st
    return "3 cadre, 2 pierdute, read() da cel mai nou"


# --- erori ------------------------------------------------------------------

def test_abonarea_esuata_e_raportata():
    try:
        cu_gz_fals(lambda: sursa(topic='/nu/exista'), subscribe_ok=False)
        raise AssertionError('abonarea esuata a trecut')
    except RuntimeError as e:
        assert 'gz topic -l' in str(e), e
        assert '/nu/exista' in str(e)
    return "mesaj cu comanda de diagnostic, nu doar 'False'"


def test_gz_lipsa_da_mesaj_util():
    """In venv-ul de dezvoltare gz NU exista - mesajul trebuie sa spuna de ce
    si unde e mediul bun."""
    try:
        dp._gz_imports()
    except ImportError as e:
        text = str(e)
        assert 'apt' in text and 'setup_sim_venv.sh' in text, text
        assert 'transport13' in text, text
        return "mesaj cu apt, setup_sim_venv.sh si versiunile incercate"
    return "gz e disponibil aici (mediu de simulare)"


def test_cadru_invalid_nu_omoara_sursa():
    """Un mesaj stricat trebuie ignorat, nu sa rupa abonamentul."""
    def corp():
        s = sursa(timeout_s=0.2)
        n = FakeNode.ultimul
        n.cb(FakeImage(fmt=99, data=b'\x00' * 10, w=5, h=2))   # invalid
        assert s.read() is None, "un cadru invalid a fost livrat"
        n.cb(FakeImage(cadru(42), t_sim=5.0))                  # valid
        g, t = s.read()
        assert int(g[0, 0]) == 42 and abs(t - 5.0) < 1e-6
        st = s.stats()
        s.close()
        return st
    st = cu_gz_fals(corp)
    assert st['received'] == 1, st
    return "cadrul invalid ignorat, urmatorul livrat normal"


def test_sursa_e_aditiva():
    """Regula rundei: nova/detector_pi.py doar pentru noul FrameSource."""
    assert issubclass(dp.GazeboFrameSource, dp.FrameSource)
    for nume in ('read', 'close', 'nominal_fps'):
        assert hasattr(dp.GazeboFrameSource, nume) or nume == 'nominal_fps'
    # clasele existente neatinse
    for nume in ('ArraySource', 'ImageDirSource', 'PiCameraSource',
                 'PiDetector', 'ArucoMarkerDetector', 'CameraCalibration'):
        assert hasattr(dp, nume), nume
    return "subclasa de FrameSource; clasele existente intacte"


TESTS = [
    ('conversia formatelor', test_conversia_formatelor),
    ('conversia copiaza bufferul', test_conversia_copiaza_bufferul),
    ('ceasul e cel de simulare', test_ceasul_e_cel_de_simulare),
    ('fps dedus din timpul de simulare',
     test_fps_dedus_din_timpul_de_simulare),
    ('read nu repeta acelasi cadru', test_read_nu_repeta_acelasi_cadru),
    ('read se deblocheaza la close', test_read_se_deblocheaza_la_close),
    ('se pierde cadrul VECHI, nu cel nou',
     test_se_pierde_cadrul_VECHI_nu_cel_nou),
    ('abonarea esuata e raportata', test_abonarea_esuata_e_raportata),
    ('gz lipsa da mesaj util', test_gz_lipsa_da_mesaj_util),
    ('cadru invalid nu omoara sursa', test_cadru_invalid_nu_omoara_sursa),
    ('sursa e aditiva', test_sursa_e_aditiva),
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
        except Exception as e:                              # noqa: BLE001
            fails += 1
            import traceback
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
            traceback.print_exc(limit=3)
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
