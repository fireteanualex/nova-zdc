#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru imaginea predata juriului (8.3.3, J2).

    python3 tools/test_scoring.py

Fara camera si fara vehicul: ringul primeste cadre sintetice cu timestamp-uri
alese, iar evenimentele se injecteaza direct.

Ce conteaza cel mai mult:

  - cadrul se alege dupa timestamp-ul CAPTURII purtat de eveniment, nu dupa
    "ultimul cadru de acum". Intre ele sunt zeci de milisecunde de coborare.
  - daca ringul nu acopera momentul cerut, NU se salveaza alt cadru. O
    imagine gresita predata ca dovada e mai rea decat una lipsa: nimeni nu
    mai poate afla ca e gresita.
  - fiecare imagine are un `.json` cu `time_boot_ms`, altfel nu se poate
    alinia cu `.bin` (6.2.1.30).
"""

import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                          # noqa: E402

from nova.frame_ring import FrameRing                       # noqa: E402
from nova.scoring import ScoringRecorder                    # noqa: E402


class FakeVehicle:
    time_boot_ms = 123456
    alt = 0.65
    roll = pitch = yaw = 0.0


def ring_cu(timpi, valoare_din_t=True):
    """Ring in care fiecare cadru poarta o valoare derivata din timpul lui,
    ca sa se poata verifica CARE cadru a fost salvat, nu doar ca s-a scris
    ceva."""
    r = FrameRing(maxlen=len(timpi) + 4)
    for t in timpi:
        v = int(round(t * 1000)) % 250 if valoare_din_t else 7
        r.push(np.full((8, 8), v, dtype=np.uint8), t)
    return r


def test_cadrul_se_alege_dupa_timestampul_capturii():
    """Nu "ultimul cadru de acum": la 0.5 m/s si 100 ms de intarziere,
    diferenta e 5 cm de altitudine."""
    d = tempfile.mkdtemp()
    try:
        timpi = [10.00, 10.05, 10.10, 10.15, 10.20]
        rec = ScoringRecorder(d, ring_cu(timpi), FakeVehicle(), verbose=False)
        rez = rec.salveaza('scoring_capture', 10.10)
        assert rez is not None
        png, js = rez
        m = json.load(open(js))
        assert abs(m['t_cadru'] - 10.10) < 1e-9, m['t_cadru']
        assert abs(m['decalaj_ms']) < 1e-6, m['decalaj_ms']
        # numele fisierului poarta timpul de CAPTURA, nu un index
        assert '000000010100' in os.path.basename(png), os.path.basename(png)
        import cv2
        img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
        assert int(img[0, 0]) == 100, (
            f"s-a salvat alt cadru: valoare {img[0, 0]}, asteptat 100")
        return "cadrul de la 10.10, nu ultimul din ring (10.20)"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_un_cadru_lipsa_nu_se_inlocuieste():
    """CAZUL CARE CONTEAZA. Daca ringul nu acopera momentul, nu se salveaza
    altul. Evidenta lipsa se poate cere din nou; evidenta gresita nu se mai
    poate detecta."""
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.0, 10.05]), FakeVehicle(),
                              verbose=False)
        # cerut cu mult dupa ce ringul s-a terminat
        assert rec.salveaza('touchdown', 12.0) is None
        assert 'touchdown' in rec.lipsa
        assert not os.path.exists(d) or not os.listdir(d), os.listdir(d)
        # si exact la limita tolerantei
        rec2 = ScoringRecorder(d, ring_cu([10.0, 10.5]), FakeVehicle(),
                               verbose=False)
        assert rec2.salveaza('touchdown', 10.1) is None, (
            "un cadru la 400 ms distanta e alt moment al coborarii")
        return f"peste {ScoringRecorder.TOLERANTA_S:g} s -> lipsa, nu alt cadru"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_nu_se_ia_un_cadru_de_DINAINTE():
    """Se cauta inainte, nu in jur: cadrul cerut e cel care a produs
    detectia. Unul de dinainte arata un alt moment, mai sus."""
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.00, 10.30]), FakeVehicle(),
                              verbose=False)
        # 10.05 e la 50 ms dupa un cadru existent, dar la 250 ms inainte de
        # urmatorul: nu se ia niciunul
        assert rec.salveaza('scoring_capture', 10.05) is None
        return "un cadru anterior nu e o aproximare acceptabila"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_evidenta_poarta_ancora_spre_bin():
    """6.2.1.30: fara `time_boot_ms` imaginea nu se poate pune in relatie cu
    logul de zbor, si atunci nu e evidenta, e o poza."""
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.0]), FakeVehicle(), verbose=False)
        _png, js = rec.salveaza('scoring_capture', 10.0,
                                {'marker_px': 812.0, 'alt': 0.65})
        m = json.load(open(js))
        assert m['time_boot_ms'] == 123456, m
        assert m['marker_px'] == 812.0 and m['eveniment'] == 'scoring_capture'
        for cheie in ('roll', 'pitch', 'yaw', 'alt_m', 't_cerut', 't_cadru'):
            assert cheie in m, cheie
        return f"{len(m)} campuri, inclusiv time_boot_ms si atitudinea"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_ambele_evenimente_produc_imagini():
    """8.3.3 cere imaginea la CONTACT; captura de scoring exista pentru ca
    la 74.5 mm juriul nu poate masura nimic. Se predau amandoua."""
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.0, 10.5, 11.0]), FakeVehicle(),
                              verbose=False)
        rec.on_event('scoring_capture', {'t': 10.0, 'marker_px': 800})
        rec.on_event('touchdown', {'t': 11.0, 'alt': 0.19})
        r = rec.raport()
        assert r['complet_8_3_3'], r
        assert set(r['salvate']) == {'scoring_capture', 'touchdown'}, r
        assert len(os.listdir(d)) == 4, os.listdir(d)   # 2 png + 2 json
        return "doua imagini si doua fisiere de evidenta"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_un_eveniment_fara_timestamp_nu_ghiceste():
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.0]), FakeVehicle(), verbose=False)
        rec.on_event('touchdown', {'alt': 0.19})            # fara 't'
        assert 'touchdown' in rec.lipsa
        assert not rec.raport()['complet_8_3_3']
        return "fara timestamp -> lipsa raportata, nu cadrul curent"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_alte_evenimente_sunt_ignorate():
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, ring_cu([10.0]), FakeVehicle(), verbose=False)
        for nume in ('state', 'abort', 'ascent_done', 'disarm_early'):
            rec.on_event(nume, {'t': 10.0})
        assert not rec.salvate and not rec.lipsa
        return "doar scoring_capture si touchdown scriu fisiere"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_fara_ring_raporteaza_lipsa_nu_arunca():
    """Pe bord ringul e optional (RAM). Fara el, 8.3.3 nu se poate
    indeplini - dar aplicatia nu are voie sa cada in mijlocul unei curse."""
    d = tempfile.mkdtemp()
    try:
        rec = ScoringRecorder(d, None, FakeVehicle(), verbose=False)
        rec.on_event('scoring_capture', {'t': 10.0})
        assert 'scoring_capture' in rec.lipsa
        assert not rec.raport()['complet_8_3_3']
        return "ring absent -> raportat, fara exceptie"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_ringul_e_acelasi_cod_peste_tot():
    """§8: acelasi cod in sim si pe Pi. Doua copii ale ringului ar fi
    divergat la prima modificare (§5.14)."""
    radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    serv = open(os.path.join(radacina, 'tools', 'nova_service.py')).read()
    assert 'from nova.frame_ring import' in serv, (
        "serviciul si-a pastrat propria copie a ringului")
    assert 'class FrameRing' not in serv, "au ramas doua definitii"
    det = open(os.path.join(radacina, 'nova', 'detector_pi.py')).read()
    assert 'self.ring.push(gray, t_cap)' in det, (
        "detectorul nu alimenteaza ringul cu timestamp-ul de captura")
    for app in ('nova_pi.py', 'nova_sim.py'):
        text = open(os.path.join(radacina, 'tools', app)).read()
        assert 'ScoringRecorder(' in text, f"{app} nu preda imaginea"
    return "un singur FrameRing, folosit de serviciu, bord si sim"


def test_ringul_primeste_cadrul_chiar_daca_detectia_pica():
    """Cadrul de contact e tocmai unul pe care markerul nu mai incape in
    cadru (§5.2). Daca ringul s-ar alimenta doar la detectie reusita, exact
    imaginea ceruta de 8.3.3 ar lipsi."""
    radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src = open(os.path.join(radacina, 'nova', 'detector_pi.py')).read()
    i = src.index('self.ring.push(gray, t_cap)')
    j = src.index('det = self.det.detect(gray, t_cap)')
    assert i < j, "ringul se alimenteaza DUPA detectie: cadrele fara marker "\
                  "s-ar pierde"
    return "push inainte de detect"


TESTS = [
    ('cadrul se alege dupa timestampul capturii',
     test_cadrul_se_alege_dupa_timestampul_capturii),
    ('un cadru lipsa nu se inlocuieste',
     test_un_cadru_lipsa_nu_se_inlocuieste),
    ('nu se ia un cadru de DINAINTE', test_nu_se_ia_un_cadru_de_DINAINTE),
    ('evidenta poarta ancora spre .bin',
     test_evidenta_poarta_ancora_spre_bin),
    ('ambele evenimente produc imagini',
     test_ambele_evenimente_produc_imagini),
    ('un eveniment fara timestamp nu ghiceste',
     test_un_eveniment_fara_timestamp_nu_ghiceste),
    ('alte evenimente sunt ignorate', test_alte_evenimente_sunt_ignorate),
    ('fara ring raporteaza lipsa, nu arunca',
     test_fara_ring_raporteaza_lipsa_nu_arunca),
    ('ringul e acelasi cod peste tot', test_ringul_e_acelasi_cod_peste_tot),
    ('ringul primeste cadrul chiar daca detectia pica',
     test_ringul_primeste_cadrul_chiar_daca_detectia_pica),
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
