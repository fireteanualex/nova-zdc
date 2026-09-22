#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru comutarea surselor EKF (15.2.5, J1).

    python3 tools/test_ekf_source.py

Fara FC si fara SITL: vehiculul e fals, iar parametrii se injecteaza direct.
Ce conteaza cel mai mult:

  - predicatul de conformitate e mai STRICT decat cel din firmware. Cazul
    negativ (YAW = GPS) e tot ce apara aici: firmware-ul l-ar raporta drept
    "fara GPS".
  - nu se comuta nimic inainte de a citi ce se comuta. Un set necitit nu e
    un set curat.
  - restaurarea se face si pe calea de ABORT, nu doar la handback.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                               # noqa: E402

from nova import ekf_source as ek                           # noqa: E402


class FakeVehicle:
    def __init__(self, valori=None, raspunde=True):
        self.params = {}
        self.cerute = []
        self.comenzi = []
        self.ekf_src_ack = None
        self.time_boot_ms = 1000
        self._valori = valori or {'POSXY': 0, 'VELXY': 0, 'POSZ': 1,
                                  'VELZ': 0, 'YAW': 1}
        self._raspunde = raspunde
        self.link = True

    def request_param(self, nume):
        self.cerute.append(nume)
        if not self._raspunde:
            return True
        for termen, val in self._valori.items():
            if nume.endswith('_' + termen):
                self.params[nume] = float(val)
        return True

    def send_ekf_source_set(self, n):
        if not self.link:
            return False
        self.comenzi.append(n)
        self.ekf_src_ack = mavutil.mavlink.MAV_RESULT_ACCEPTED
        return True


def ruleaza(s, v, faze, t0=0.0, dt=0.1):
    t = t0
    for faza in faze:
        s.update(t, faza)
        t += dt
    return t


def test_predicatul_e_mai_strict_decat_firmware_ul():
    """CAZUL CARE CONTEAZA. `AP_NavEKF_Source::usingGPS()` verifica doar
    `YAW == GSF` (8). `YAW == GPS` (2) si `GPS_COMPASS_FALLBACK` (3)
    alimenteaza tot estimatorul de cap cu GNSS, si ar trece de el."""
    curat = {'POSXY': 0, 'VELXY': 0, 'POSZ': 1, 'VELZ': 0, 'YAW': 1}
    rau, motive = ek.contine_gnss(curat)
    assert not rau, motive

    for yaw_gnss in (2, 3, 8):
        v = dict(curat, YAW=yaw_gnss)
        rau, motive = ek.contine_gnss(v)
        assert rau, f"YAW={yaw_gnss} trebuie semnalat; firmware-ul nu o face"
    # firmware-ul ar semnala doar 8
    assert 8 in ek.SURSA_YAW_GNSS and 2 in ek.SURSA_YAW_GNSS
    return "YAW 2, 3 si 8 semnalate; firmware-ul semnaleaza doar 8"


def test_fiecare_termen_e_verificat():
    curat = {'POSXY': 0, 'VELXY': 0, 'POSZ': 1, 'VELZ': 0, 'YAW': 1}
    for termen, rele in ek.TERMENI:
        v = dict(curat)
        v[termen] = rele[0]
        rau, motive = ek.contine_gnss(v)
        assert rau, f"{termen}={rele[0]} nu a fost semnalat"
    return f"{len(ek.TERMENI)} termeni, fiecare semnalat separat"


def test_un_set_necitit_nu_e_un_set_curat():
    """Necunoscut nu inseamna conform. §5.10 aplicat unei afirmatii de
    conformitate: lipsa dovezii nu e dovada."""
    partial = {'POSXY': 0, 'VELXY': 0, 'POSZ': 1, 'VELZ': 0}
    rau, motive = ek.contine_gnss(partial)
    assert rau and 'YAW necitit' in motive[0], motive
    return "termen lipsa -> refuz, nu acceptare tacita"


def test_nu_comuta_inainte_de_a_citi():
    v = FakeVehicle()
    s = ek.EkfSourceManager(v, verbose=False)
    s.update(0.0, 'DESCEND_TRACK')
    assert s.state == s.CITESTE, s.state
    assert v.comenzi == [], "a comutat inainte sa citeasca"
    assert len(v.cerute) == len(ek.TERMENI), v.cerute
    s.update(0.1, 'DESCEND_TRACK')
    assert s.state == s.ACTIV, s.state
    assert v.comenzi == [ek.SET_AUTONOM], v.comenzi
    return f"{len(v.cerute)} parametri ceruti inainte de comanda"


def test_refuza_un_set_cu_GNSS():
    """Daca setul tinta contine GNSS, nu se comuta DELOC - nu se comuta si
    apoi se raporteaza. O comutare care incalca regula e mai rea decat
    niciuna: vehiculul ar zbura autonom in afara conformitatii."""
    v = FakeVehicle(valori={'POSXY': 3, 'VELXY': 0, 'POSZ': 1,
                            'VELZ': 0, 'YAW': 1})
    s = ek.EkfSourceManager(v, verbose=False)
    ruleaza(s, v, ['DESCEND_TRACK'] * 3)
    assert s.state == s.REFUZAT, s.state
    assert v.comenzi == [], "a comutat pe un set cu GNSS"
    assert not s.conform
    assert any('POSXY' in m for m in s.motive_refuz), s.motive_refuz
    return f"refuzat: {s.motive_refuz[0]}"


def test_restaureaza_la_handback():
    v = FakeVehicle()
    s = ek.EkfSourceManager(v, verbose=False)
    ruleaza(s, v, ['DESCEND_TRACK'] * 3)
    assert s.conform
    s.update(1.0, 'HANDBACK')
    assert v.comenzi == [ek.SET_AUTONOM, ek.SET_NORMAL], v.comenzi
    assert not s.conform, "dupa restaurare nu mai suntem in regim fara GNSS"
    return "set 2 -> set 1 la iesirea din segment"


def test_restaureaza_si_pe_calea_de_abort():
    """Calea care conteaza: pilotul preia, supervizorul comanda BRAKE, faza
    devine IDLE. Vehiculul trebuie predat cu estimator nominal."""
    v = FakeVehicle()
    s = ek.EkfSourceManager(v, verbose=False)
    ruleaza(s, v, ['DESCEND_TRACK'] * 3)
    s.update(1.0, 'IDLE')
    assert v.comenzi[-1] == ek.SET_NORMAL, v.comenzi
    # si o faza pe care nimeni nu a prevazut-o
    v2 = FakeVehicle()
    s2 = ek.EkfSourceManager(v2, verbose=False)
    ruleaza(s2, v2, ['DESCEND_TRACK'] * 3)
    s2.update(1.0, 'O_FAZA_INVENTATA')
    assert v2.comenzi[-1] == ek.SET_NORMAL, v2.comenzi
    return "abort si faza necunoscuta duc amandoua la restaurare"


def test_fara_raspuns_de_la_FC_nu_comuta():
    v = FakeVehicle(raspunde=False)
    s = ek.EkfSourceManager(v, verbose=False)
    t = 0.0
    for _ in range(80):
        s.update(t, 'DESCEND_TRACK')
        t += 0.1
    assert s.state == s.REFUZAT, s.state
    assert v.comenzi == [], "a comutat fara sa fi citit nimic"
    return "timeout la citire -> refuz, nu comutare oarba"


def test_legatura_cazuta_nu_pierde_comutarea():
    """`send_ekf_source_set` intoarce False cu legatura cazuta. Nu se
    marcheaza ACTIV, deci se reincearca - acelasi tipar ca §5.30."""
    v = FakeVehicle()
    v.link = False
    s = ek.EkfSourceManager(v, verbose=False)
    ruleaza(s, v, ['DESCEND_TRACK'] * 3)
    assert s.state == s.CITESTE, s.state
    v.link = True
    s.update(1.0, 'DESCEND_TRACK')
    assert s.state == s.ACTIV and v.comenzi == [ek.SET_AUTONOM]
    return "reincercat la revenirea legaturii"


def test_conformitatea_e_o_masuratoare_nu_o_intentie():
    v = FakeVehicle()
    s = ek.EkfSourceManager(v, verbose=False)
    assert not s.conform, "inainte de comutare nu suntem conformi"
    ruleaza(s, v, ['DESCEND_TRACK'] * 3)
    assert s.conform and s.ack_ok
    r = s.raport()
    assert r['valori']['YAW'] == 1 and r['ack_ok']
    assert set(r['valori']) == {t for t, _ in ek.TERMENI}
    return "raportul poarta valorile CITITE si raspunsul FC-ului"


def test_parametrii_din_fisiere_sunt_conformi():
    """Fisierele de parametri trebuie sa descrie un set fara GNSS. Daca
    cineva schimba o linie, asta pica - nu campania de peste doua zile."""
    radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for fisier in ('nova_sitl.parm', 'nova_flight.parm'):
        cale = os.path.join(radacina, 'config', fisier)
        valori = {}
        for linie in open(cale):
            linie = linie.split('#')[0].strip()
            if not linie or ',' not in linie:
                continue
            nume, val = linie.split(',', 1)
            for termen, _ in ek.TERMENI:
                if nume.strip() == ek.nume_param(ek.SET_AUTONOM, termen):
                    valori[termen] = float(val)
        rau, motive = ek.contine_gnss(valori)
        assert not rau, f"{fisier}: {motive}"
    return f"ambele fisiere descriu setul {ek.SET_AUTONOM} fara GNSS"


TESTS = [
    ('predicatul e mai strict decat firmware-ul',
     test_predicatul_e_mai_strict_decat_firmware_ul),
    ('fiecare termen e verificat', test_fiecare_termen_e_verificat),
    ('un set necitit nu e un set curat',
     test_un_set_necitit_nu_e_un_set_curat),
    ('nu comuta inainte de a citi', test_nu_comuta_inainte_de_a_citi),
    ('refuza un set cu GNSS', test_refuza_un_set_cu_GNSS),
    ('restaureaza la handback', test_restaureaza_la_handback),
    ('restaureaza si pe calea de abort',
     test_restaureaza_si_pe_calea_de_abort),
    ('fara raspuns de la FC nu comuta', test_fara_raspuns_de_la_FC_nu_comuta),
    ('legatura cazuta nu pierde comutarea',
     test_legatura_cazuta_nu_pierde_comutarea),
    ('conformitatea e o masuratoare, nu o intentie',
     test_conformitatea_e_o_masuratoare_nu_o_intentie),
    ('parametrii din fisiere sunt conformi',
     test_parametrii_din_fisiere_sunt_conformi),
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
