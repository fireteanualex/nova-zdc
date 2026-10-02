#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru modularea autoritatii pe praguri de altitudine.

    python3 tools/test_authority.py

Fara FC: `FakeFC` imita semantica reala a parametrilor din ArduPilot -
PARAM_SET nu are efect pana la PARAM_VALUE, un nume inexistent nu raspunde
DELOC (§5.4), iar confirmarile se pot pierde. Astea sunt starile care conteaza
si care nu se pot produce la comanda pe banc.

Testul central e **restaurarea**: pilotul trebuie sa regaseasca vehiculul cu
autoritate nominala pe ORICE cale de iesire, inclusiv abort, inclusiv cand
legatura a cazut la mijloc.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova import authority as auth                          # noqa: E402
from nova.authority import (MANAGED, AuthorityScheduler,     # noqa: E402
                            PROFIL_IMPLICIT, PROFIL_RAPID)

#: Valori "de pe vehicul" inainte de orice modificare. Copter-4.5.7 names
#: and units: WPNAV_ACCEL 250 cm/s/s = 2.5 m/s/s, speeds 50 cm/s = 0.5 m/s.
ORIGINALE = {
    'PSC_POSXY_P': 1.0,
    'PSC_VELXY_D': 0.25,
    'WPNAV_ACCEL': 250.0,
    'WPNAV_SPEED_DN': 50.0,
    'LAND_SPEED': 50.0,
    'PLND_LAG': 0.02,
    # parametri pe care NU ii gestionam - prezenti ca sa se vada ca raman
    'ANGLE_MAX': 30.0,
    'PSC_ANGLE_MAX': 0.0,
}


class FakeFC:
    """Semantica reala a parametrilor, cat ne trebuie.

    - `set_param` intra in pending si se aplica abia la `pump()`, ca pe FC
    - un nume din `inexistente` nu raspunde niciodata: nici la citire, nici
      la scriere (§5.4 - exact cum se comporta un nume gresit)
    - `drop_confirms` pierde confirmarile, ca sa se poata testa persistenta
    - `link_healthy` False => nu pleaca nimic (H1)
    """

    def __init__(self, params=None, inexistente=(), drop_confirms=0):
        # `params` incepe POPULAT, ca pe un vehicul real unde check_params
        # sau GCS-ul au citit deja parametrii. `params_t` e gol: nimic nu a
        # fost vazut dupa armare, deci nimic nu conteaza ca "proaspat".
        self.params = dict(params or ORIGINALE)
        self.params_t = {}
        self.reale = dict(self.params)
        self.inexistente = set(inexistente)
        for n in self.inexistente:
            self.params.pop(n, None)
            self.reale.pop(n, None)
        self.drop_confirms = drop_confirms
        self.link_healthy = True
        self.time_boot_ms = 4242
        self.scrieri = []            # (nume, valoare) - TOT ce s-a trimis
        self.citiri = []
        self._pending = {}
        self._cereri = []

    # -- interfata folosita de scheduler --
    def request_param(self, name):
        if not self.link_healthy:
            return False
        self.citiri.append(name)
        if name not in self.inexistente:
            self._cereri.append(name)
        return True

    def set_param(self, name, value, now=None):
        if not self.link_healthy:
            return False
        self.scrieri.append((name, float(value)))
        if name in self.inexistente:
            return True              # pleaca, dar nu confirma niciodata
        self._pending[name] = float(value)
        return True

    def param_pending(self, name=None):
        if name is None:
            return bool(self._pending)
        return name in self._pending

    # -- simuleaza FC-ul --
    def pump(self):
        """Aplica ce e in pending si raspunde la cereri."""
        import time as _t
        for name in self._cereri:
            self.params[name] = self.reale[name]
            self.params_t[name] = _t.monotonic()
        self._cereri = []
        for name, val in list(self._pending.items()):
            if self.drop_confirms > 0:
                self.drop_confirms -= 1
                del self._pending[name]
                continue             # FC-ul nu a primit: ramane vechea valoare
            self.reale[name] = val
            self.params[name] = val
            self.params_t[name] = _t.monotonic()
            del self._pending[name]


def rulare(sched, fc, pasi, phase, agl, latency=None, t0=0.0, dt=0.5):
    """N cicluri de bucla: update + pump. Intoarce timpul final."""
    t = t0
    for _ in range(pasi):
        a = agl(t) if callable(agl) else agl
        p = phase(t) if callable(phase) else phase
        sched.update(t, a, p, latency_s=latency)
        fc.pump()
        t += dt
    return t


def make(fc=None, **kw):
    fc = fc or FakeFC()
    return fc, AuthorityScheduler(fc, verbose=False, **kw)


# --- salvare ----------------------------------------------------------------

def test_nu_modifica_nimic_inainte_de_a_salva():
    """Regula 1: un parametru al carui original nu l-am citit nu se atinge.

    Altfel am face o modificare pe care nu o putem anula - exact ce nu are
    voie sa se intample cu autoritatea de control."""
    fc, s = make()
    # un singur ciclu: cererile au plecat, dar raspunsurile inca nu au venit
    s.update(0.0, 10.0, 'DESCEND_TRACK')
    assert s.state == s.SAVING, s.state
    assert not fc.scrieri, f"a scris inainte sa salveze: {fc.scrieri}"
    assert set(fc.citiri) == {sp.name for sp in MANAGED}, fc.citiri

    rulare(s, fc, 3, 'DESCEND_TRACK', 10.0, latency=0.05)
    assert s.state == s.ACTIVE
    assert s.original['WPNAV_ACCEL'] == 250.0
    assert len(s.original) == len(MANAGED)
    return f"{len(fc.citiri)} citiri inainte de prima scriere; 0 scrieri"


def test_NEGATIV_parametru_inexistent_nu_e_gestionat():
    """§5.4: un nume gresit nu da eroare, doar nu raspunde.

    Daca l-am scrie oricum, am avea in `original` o gaura si n-am putea
    restaura - plus ca scrierea nu ar face nimic, deci am crede ca am
    modulat ceva ce de fapt nu s-a schimbat."""
    fc, s = make(FakeFC(inexistente={'WPNAV_ACCEL'}))
    # READ_TRIES cereri la cate READ_TIMEOUT_S, plus cicluri pentru aplicare.
    # Detectia dureaza ~6 s, in care autoritatea ramane NOMINALA - adica
    # partea sigura a compromisului.
    rulare(s, fc, 24, 'DESCEND_TRACK', 10.0, latency=0.05)
    assert s.state == s.ACTIVE, s.state
    assert 'WPNAV_ACCEL' in s.unavailable, s.unavailable
    assert 'WPNAV_ACCEL' not in s.original
    scrise = {n for n, _ in fc.scrieri}
    assert 'WPNAV_ACCEL' not in scrise, "a scris un parametru inexistent"
    assert 'PSC_POSXY_P' in scrise, "restul nu s-au mai aplicat"
    ev = [e for e in s.log if e.kind == 'unavailable']
    assert ev and 'WPNAV_ACCEL' in ev[0].detail
    return ("WPNAV_ACCEL neraspuns -> raportat, negestionat, nescris; "
            "restul merg")


# --- benzi ------------------------------------------------------------------

def test_benzi_si_tinte():
    fc, s = make()
    rulare(s, fc, 3, 'DESCEND_TRACK', 12.0, latency=0.05)
    assert s.band.nume == 'sus', s.band
    # multiplicator: 0.70 * originalul 1.0 (pasul 2 din procedura de
    # diagnostic a oscilatiei)
    assert abs(fc.reale['PSC_POSXY_P'] - 0.70) < 1e-6, fc.reale
    # D: 0.60 * 0.25
    assert abs(fc.reale['PSC_VELXY_D'] - 0.15) < 1e-6, fc.reale

    rulare(s, fc, 6, 'DESCEND_TRACK', 1.0, latency=0.05, t0=10.0)
    assert s.band.nume == 'jos', s.band
    assert abs(fc.reale['PSC_POSXY_P'] - 1.25) < 1e-6
    # 0.7 * 250 cm/s/s = 175 cm/s/s = 1.75 m/s/s, as before the translation
    assert abs(fc.reale['WPNAV_ACCEL'] - 0.7 * 250.0) < 1e-6, (
        fc.reale['WPNAV_ACCEL'])
    assert abs(fc.reale['LAND_SPEED'] - 35.0) < 1e-6      # 0.35 m/s
    return ("sus: P x0.70, D x0.60; jos: P x1.25, WPNAV_ACCEL 175 cm/s/s, "
            "LAND_SPEED 35 cm/s")


def test_histereza_nu_lasa_sa_oscileze():
    """Zgomot de +-0.3 m in jurul pragului: o singura trecere.

    Pragul se ia din profil, nu se scrie de mana: mutat (2.0 -> 3.0 la I5),
    o cifra fixa aici ar face oscilatia sa cada INTEGRAL intr-o banda, iar
    testul ar raporta '0 treceri' - adica ar trece fara sa testeze nimic."""
    fc, s = make()
    rulare(s, fc, 3, 'DESCEND_TRACK', 5.0, latency=0.05)
    n0 = len(fc.scrieri)

    import math
    prag = auth.PROFIL_IMPLICIT[1].min_agl     # granita mijloc / jos
    t = rulare(s, fc, 60, 'DESCEND_TRACK',
               lambda tt: prag + 0.3 * math.sin(tt * 3.0), latency=0.05,
               t0=10.0, dt=0.2)
    treceri = [e for e in s.log if e.kind == 'band']
    assert len(treceri) <= 3, (
        f"{len(treceri)} treceri de banda pe zgomot in jurul pragului: "
        f"{[e.detail[:20] for e in treceri]}")
    scrieri_pe_ciclu = (len(fc.scrieri) - n0) / 60.0
    assert scrieri_pe_ciclu < 1.0, (
        f"{scrieri_pe_ciclu:.1f} PARAM_SET pe ciclu - histereza nu tine")
    del t
    return (f"{len(treceri)} treceri totale, "
            f"{scrieri_pe_ciclu:.2f} scrieri/ciclu pe zgomot de +-0.3 m")


def test_PLND_LAG_din_latenta_masurata():
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.087)
    assert abs(fc.reale['PLND_LAG'] - 0.087) < 1e-6, fc.reale['PLND_LAG']

    # clamp la domeniul din sursa (@Range 0.02 0.250)
    fc2, s2 = make()
    rulare(s2, fc2, 4, 'DESCEND_TRACK', 10.0, latency=0.9)
    assert abs(fc2.reale['PLND_LAG'] - auth.PLND_LAG_MAX_S) < 1e-6

    # fara masuratoare NU inventam o valoare
    fc3, s3 = make()
    rulare(s3, fc3, 6, 'DESCEND_TRACK', 10.0, latency=None)
    assert 'PLND_LAG' not in {n for n, _ in fc3.scrieri}, (
        "a scris PLND_LAG fara sa aiba o latenta masurata")
    return "0.087 s scris; 0.9 s taiat la 0.25; fara masuratoare -> nescris"


def test_viteza_de_coborare_limitata_la_ce_e_validat():
    """§6/15.2.9: franarea e masurata doar la 0.5 m/s. Profilul rapid nu
    trece fara masuratoarea de distanta de franare la fiecare treapta."""
    fc, s = make(bands=PROFIL_RAPID)
    rulare(s, fc, 4, 'DESCEND_TRACK', 12.0, latency=0.05)
    for k in auth.DESCENT_SPEEDS:
        assert abs(fc.reale[k] - auth.DESCENT_VALIDATED_CMS) < 1e-6, (
            f"{k} {fc.reale[k]} - profilul rapid a trecut de limita")
    assert auth.DESCENT_VALIDATED_CMS == 50 and \
        auth.DESCENT_VALIDATED_MS == 0.5

    fc2, s2 = make(bands=PROFIL_RAPID, allow_fast_descent=True)
    rulare(s2, fc2, 4, 'DESCEND_TRACK', 12.0, latency=0.05)
    assert abs(fc2.reale['WPNAV_SPEED_DN'] - 150.0) < 1e-6, (
        fc2.reale['WPNAV_SPEED_DN'])
    return ("profil rapid taiat la 50 cm/s (0.5 m/s) implicit; "
            "cu allow_fast_descent, 150 cm/s")


# --- restaurare -------------------------------------------------------------

def _verifica_restaurat(fc, s, eticheta):
    for name, orig in ORIGINALE.items():
        assert abs(fc.reale[name] - orig) < 1e-6, (
            f"{eticheta}: {name} = {fc.reale[name]:g}, original {orig:g}")
    assert s.restored is True, f"{eticheta}: restored={s.restored}"
    assert s.modified == [], f"{eticheta}: inca modificati: {s.modified}"
    assert s.state == s.DONE, f"{eticheta}: stare {s.state}"


def test_restaurare_pe_handback():
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.05)
    rulare(s, fc, 4, 'FINAL_DESCENT', 0.5, latency=0.05, t0=10.0)
    assert s.modified, "nu a modificat nimic, deci testul nu masoara nimic"
    modificati = list(s.modified)

    rulare(s, fc, 10, 'HANDBACK', 5.0, t0=20.0)
    _verifica_restaurat(fc, s, 'handback')
    return f"{len(modificati)} parametri modificati, toti confirmati inapoi"


def test_restaurare_pe_ABORT():
    """Calea de abort e cea care conteaza: acolo ceva a mers deja prost,
    si tocmai atunci pilotul preia vehiculul."""
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 9.0, latency=0.05)
    rulare(s, fc, 6, 'DESCEND_TRACK', 1.2, latency=0.05, t0=10.0)
    assert s.band.nume == 'jos'
    assert s.modified

    # abort: faza se schimba brusc, fara nicio pregatire
    rulare(s, fc, 12, 'ABORT', 1.2, t0=20.0)
    _verifica_restaurat(fc, s, 'abort')
    ev = [e for e in s.log if e.kind == 'release']
    assert ev and 'ABORT' in ev[0].detail, [e.detail for e in ev]
    return "abort din banda 'jos' -> toti parametrii inapoi la original"


def test_restaurare_pe_faza_necunoscuta():
    """O faza pe care nimeni nu a prevazut-o trebuie sa restaureze, nu sa
    lase vehiculul modulat. Lista MODULATED_PHASES e pozitiva tocmai de-aia."""
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.05)
    rulare(s, fc, 12, 'O_FAZA_NOUA_INVENTATA', 3.0, t0=10.0)
    _verifica_restaurat(fc, s, 'faza necunoscuta')
    return "faza necunoscuta = iesire din segment = restaurare"


def test_restaurare_supravietuieste_caderii_de_legatura():
    """H1: cu legatura cazuta nu pleaca nimic si NU se consuma incercari.
    Cand revine, restaurarea se duce la capat."""
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.05)
    rulare(s, fc, 4, 'DESCEND_TRACK', 1.0, latency=0.05, t0=10.0)
    assert s.modified

    fc.link_healthy = False
    n_inainte = len(fc.scrieri)
    rulare(s, fc, 40, 'ABORT', 1.0, t0=20.0)
    assert len(fc.scrieri) == n_inainte, (
        f"{len(fc.scrieri) - n_inainte} scrieri cu legatura cazuta")
    assert s.restored is False, "s-a declarat restaurat cu legatura cazuta"
    assert s.state == s.RESTORING

    fc.link_healthy = True
    rulare(s, fc, 20, 'ABORT', 1.0, t0=60.0)
    _verifica_restaurat(fc, s, 'dupa revenirea legaturii')
    return ("40 de cicluri cu link cazut: 0 scrieri, 0 incercari consumate; "
            "la revenire, restaurare completa")


def test_restaurare_nu_renunta_unde_Vehicle_ar_renunta():
    """`Vehicle.set_param` abandoneaza dupa PARAM_TRIES. Pentru restaurare
    asta ar lasa pilotul cu autoritate modificata, tacut."""
    fc, s = make(FakeFC(drop_confirms=25))
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.05)
    fc.drop_confirms = 25
    rulare(s, fc, 6, 'DESCEND_TRACK', 1.0, latency=0.05, t0=10.0)

    fc.drop_confirms = 12          # primele 12 confirmari de restaurare se pierd
    rulare(s, fc, 60, 'ABORT', 1.0, t0=20.0)
    _verifica_restaurat(fc, s, 'confirmari pierdute')
    return "12 confirmari pierdute la restaurare -> tot s-a dus la capat"


def test_restored_e_afirmatie_verificata():
    """`restored` nu are voie sa fie True doar pentru ca am trimis comenzile."""
    fc, s = make()
    rulare(s, fc, 4, 'DESCEND_TRACK', 10.0, latency=0.05)
    rulare(s, fc, 4, 'DESCEND_TRACK', 1.0, latency=0.05, t0=10.0)
    assert s.restored is False, "modulat, dar se declara restaurat"

    # release, dar FC-ul nu confirma nimic: NU e restaurat
    fc.drop_confirms = 10_000
    s.release(20.0, 'test')
    for i in range(10):
        s.update(20.0 + i, 1.0, 'ABORT')
        fc.pump()
    assert s.restored is False, (
        "s-a declarat restaurat fara nicio confirmare de la FC")
    assert s.state == s.RESTORING

    fc.drop_confirms = 0
    rulare(s, fc, 30, 'ABORT', 1.0, t0=40.0)
    assert s.restored is True
    return "False cat timp FC-ul nu confirma; True doar dupa citire inapoi"


def test_release_inainte_de_prima_scriere():
    """Abort in timpul salvarii: nimic nu fusese modificat, deci nimic de
    restaurat - si nu trebuie sa ramana blocat in RESTORING."""
    fc, s = make()
    s.update(0.0, 10.0, 'DESCEND_TRACK')      # doar cererile de citire
    assert s.state == s.SAVING and not fc.scrieri
    s.update(0.5, 10.0, 'ABORT')
    assert s.state == s.DONE, s.state
    assert s.restored is True
    assert not fc.scrieri, fc.scrieri
    return "abort in timpul salvarii -> DONE, fara scrieri"


# --- garzi ------------------------------------------------------------------

def test_ANGLE_MAX_nu_e_atins_niciodata():
    """Cerinta explicita. Verificata pe TOT ce s-a trimis, nu pe intentie."""
    assert 'ANGLE_MAX' in auth.FORBIDDEN and 'PSC_ANGLE_MAX' in auth.FORBIDDEN
    for s in MANAGED:
        assert s.name not in auth.FORBIDDEN

    fc, sched = make()
    rulare(sched, fc, 4, 'DESCEND_TRACK', 12.0, latency=0.05)
    rulare(sched, fc, 4, 'DESCEND_TRACK', 5.0, latency=0.05, t0=10.0)
    rulare(sched, fc, 4, 'DESCEND_TRACK', 0.8, latency=0.05, t0=20.0)
    rulare(sched, fc, 12, 'ABORT', 0.8, t0=30.0)

    atinse = {n for n, _ in fc.scrieri} | set(fc.citiri)
    for interzis in auth.FORBIDDEN:
        assert interzis not in atinse, f"{interzis} atins: {fc.scrieri}"
        assert abs(fc.reale[interzis] - ORIGINALE[interzis]) < 1e-9

    # si o banda nu poate numi un parametru interzis
    try:
        auth.Band('rea', 0.0, ANGLE_MAX=0.5)
        raise AssertionError('o banda a acceptat ANGLE_MAX')
    except AssertionError as e:
        if 'a acceptat' in str(e):
            raise
    return (f"{len(atinse)} parametri atinsi in tot ciclul, niciunul din "
            f"FORBIDDEN; banda cu ANGLE_MAX respinsa")


def test_benzile_acopera_toata_plaja():
    """O altitudine fara banda ar lasa autoritatea la ce era inainte."""
    fc, s = make()
    rulare(s, fc, 3, 'DESCEND_TRACK', 10.0, latency=0.05)
    for agl in (100.0, 12.0, 8.0, 7.99, 2.0, 1.99, 0.3, 0.0):
        b = s.band_for(agl)
        assert b is not None, f"nicio banda pentru {agl} m"
    assert s.band_for(None) is s.band, "agl necunoscut nu trebuie sa schimbe banda"

    praguri = [b.min_agl for b in PROFIL_IMPLICIT]
    assert praguri == sorted(praguri, reverse=True), praguri
    assert praguri[-1] == 0.0, "ultima banda trebuie sa inceapa de la 0"
    return f"praguri {praguri}, agl=None pastreaza banda curenta"


def test_numele_sunt_cele_de_pe_4_5_7():
    """The board runs Copter-4.5.7 (CUAV X7+): a 4.7+/4.8 name gets no
    PARAM_VALUE there, so the scheduler would mark it unavailable and the
    modulation would be silently inert (§5.4). The test pins the 4.5.7
    names; the full list check is in tools/test_params_x7.py."""
    nume = {s.name for s in MANAGED}
    for doar_47 in ('WP_ACC', 'PSC_NE_POS_P', 'PSC_NE_VEL_D', 'WP_SPD_DN',
                    'LAND_SPD_MS'):
        assert doar_47 not in nume, f"{doar_47} nu exista pe Copter-4.5.7"
    for corect in ('WPNAV_ACCEL', 'PSC_POSXY_P', 'PSC_VELXY_D',
                   'WPNAV_SPEED_DN', 'LAND_SPEED', 'PLND_LAG'):
        assert corect in nume, f"lipseste {corect}"
    return ', '.join(sorted(nume))


#: The profile as it was written for 4.7+/4.8 (SI units), frozen here as the
#: reference for the translation. 'mul' entries are factors, 'abs' entries
#: m/s. (min_agl, {4.7 name: value})
_PROFIL_47 = {
    'PROFIL_IMPLICIT': [
        (8.0, dict(PSC_NE_POS_P=0.70, PSC_NE_VEL_D=0.60, WP_ACC=1.00,
                   WP_SPD_DN=0.5, LAND_SPD_MS=0.5)),
        (3.0, dict(PSC_NE_POS_P=1.00, PSC_NE_VEL_D=1.00, WP_ACC=1.00,
                   WP_SPD_DN=0.5, LAND_SPD_MS=0.5)),
        (0.5, dict(PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.70,
                   WP_SPD_DN=0.35, LAND_SPD_MS=0.35)),
        (0.0, dict(PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.50,
                   WP_SPD_DN=0.35, LAND_SPD_MS=0.35)),
    ],
    'PROFIL_RAPID': [
        (8.0, dict(PSC_NE_POS_P=0.70, PSC_NE_VEL_D=0.60, WP_ACC=1.00,
                   WP_SPD_DN=1.5, LAND_SPD_MS=1.5)),
        (3.0, dict(PSC_NE_POS_P=1.00, PSC_NE_VEL_D=1.00, WP_ACC=1.00,
                   WP_SPD_DN=0.8, LAND_SPD_MS=0.8)),
        (0.5, dict(PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.70,
                   WP_SPD_DN=0.3, LAND_SPD_MS=0.3)),
        (0.0, dict(PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.50,
                   WP_SPD_DN=0.2, LAND_SPD_MS=0.2)),
    ],
}
#: Spec limits as they were for 4.7+ (SI units): name -> (mode, lo, hi).
_LIMITE_47 = {
    'PSC_NE_POS_P': ('mul', 0.1, 5.0), 'PSC_NE_VEL_D': ('mul', 0.0, 1.0),
    'WP_ACC': ('mul', 0.5, 5.0), 'WP_SPD_DN': ('abs', 0.1, 5.0),
    'LAND_SPD_MS': ('abs', 0.1, 5.0),
}


def test_traducerea_4_5_7_e_cea_din_make_parm_45():
    """Behaviour identical to the 4.7+ version: every 4.5.7 value is the old
    SI value passed through the SAME table that generates
    config/nova_flight_4.5.parm (tools/make_parm_45.py REDENUMIRI). 'mul'
    factors are unitless and stay as they are; absolute values and the
    clamp limits go through the unit conversion. LAND_SPEED is AP_Int16 on
    4.5.7 (ArduCopter/Parameters.h:429): a fractional cm/s value would be
    truncated by the FC and never confirmed, so every target is an integer."""
    import make_parm_45 as mp
    n = 0
    for nume_profil, benzi47 in _PROFIL_47.items():
        benzi = getattr(auth, nume_profil)
        assert [b.min_agl for b in benzi] == [a for a, _ in benzi47]
        for b, (_, valori47) in zip(benzi, benzi47):
            assert len(b.valori) == len(valori47), (b.nume, b.valori)
            for n47, v47 in valori47.items():
                n45, conv = mp.REDENUMIRI[n47]
                mode = _LIMITE_47[n47][0]
                asteptat = v47 if mode == 'mul' else conv(v47)
                assert b.valori[n45] == asteptat, (
                    f"{nume_profil}/{b.nume}: {n45}={b.valori[n45]}, "
                    f"din {n47}={v47} -> {asteptat}")
                n += 1
            assert float(b.valori['LAND_SPEED']).is_integer()
    for n47, (mode, lo, hi) in _LIMITE_47.items():
        n45, conv = mp.REDENUMIRI[n47]
        sp = auth.MANAGED_BY_NAME[n45]
        assert sp.mode == mode, (n45, sp.mode)
        assert (sp.lo, sp.hi) == (conv(lo), conv(hi)), (
            f"{n45}: limite {(sp.lo, sp.hi)} vs {n47} {(lo, hi)}")
    assert auth.DESCENT_VALIDATED_CMS == mp.REDENUMIRI['WP_SPD_DN'][1](0.5)
    assert len(MANAGED) == len(_LIMITE_47) + 1          # + PLND_LAG, same
    return f"{n} valori de banda + {len(_LIMITE_47)} limite = tabela REDENUMIRI"


def test_profilele_difera_doar_pe_viteza():
    """I5: cand se compara doua rulari, singura variabila schimbata trebuie
    sa fie profilul de coborare. Daca si castigurile difera, o oscilatie
    apuruta la 1.5 m/s nu se mai poate atribui vitezei."""
    viteze = set(auth.DESCENT_SPEEDS)
    a, b = auth.PROFIL_IMPLICIT, auth.PROFIL_RAPID
    assert len(a) == len(b), (len(a), len(b))
    for ba, bb in zip(a, b):
        assert ba.min_agl == bb.min_agl, (ba.nume, ba.min_agl, bb.min_agl)
        for k in set(ba.valori) | set(bb.valori):
            if k in viteze:
                continue
            assert ba.valori.get(k) == bb.valori.get(k), (
                f"{ba.nume}.{k}: {ba.valori.get(k)} vs {bb.valori.get(k)} - "
                f"profilele nu au voie sa difere decat pe viteza")
    return f"{len(a)} benzi, identice in afara de {sorted(viteze)}"


def test_profilul_rapid_e_cel_din_tabelul_I5():
    # cm/s on 4.5.7: 1.5 / 0.8 / 0.3 / 0.2 m/s
    asteptat = [(8.0, 150), (3.0, 80), (0.5, 30), (0.0, 20)]
    gasit = [(b.min_agl, b.valori['WPNAV_SPEED_DN'])
             for b in auth.PROFIL_RAPID]
    assert gasit == asteptat, f"{gasit} != {asteptat}"
    for b in auth.PROFIL_RAPID:
        assert b.valori['LAND_SPEED'] == b.valori['WPNAV_SPEED_DN'], (
            f"{b.nume}: LAND_SPEED si WPNAV_SPEED_DN trebuie sa spuna "
            f"acelasi lucru, altfel viteza depinde de care controler e activ")
    return "8m/150, 3m/80, 0.5m/30, contact/20 cm/s"


def test_profilul_implicit_nu_depaseste_ce_s_a_validat():
    for b in auth.PROFIL_IMPLICIT:
        for k in auth.DESCENT_SPEEDS:
            assert b.valori[k] <= auth.DESCENT_VALIDATED_CMS, (
                f"{b.nume}.{k} = {b.valori[k]} peste "
                f"{auth.DESCENT_VALIDATED_CMS} cm/s validati")
    return f"toate <= {auth.DESCENT_VALIDATED_CMS} cm/s"


TESTS = [
    ('profilele difera doar pe viteza', test_profilele_difera_doar_pe_viteza),
    ('profilul rapid e cel din tabelul I5',
     test_profilul_rapid_e_cel_din_tabelul_I5),
    ('profilul implicit nu depaseste ce s-a validat',
     test_profilul_implicit_nu_depaseste_ce_s_a_validat),
    ('nu modifica nimic inainte de a salva',
     test_nu_modifica_nimic_inainte_de_a_salva),
    ('NEGATIV: parametru inexistent nu e gestionat',
     test_NEGATIV_parametru_inexistent_nu_e_gestionat),
    ('benzi si tinte', test_benzi_si_tinte),
    ('histereza nu lasa sa oscileze', test_histereza_nu_lasa_sa_oscileze),
    ('PLND_LAG din latenta masurata', test_PLND_LAG_din_latenta_masurata),
    ('viteza de coborare limitata la ce e validat',
     test_viteza_de_coborare_limitata_la_ce_e_validat),
    ('RESTAURARE pe handback', test_restaurare_pe_handback),
    ('RESTAURARE pe ABORT', test_restaurare_pe_ABORT),
    ('RESTAURARE pe faza necunoscuta', test_restaurare_pe_faza_necunoscuta),
    ('RESTAURARE supravietuieste caderii de legatura',
     test_restaurare_supravietuieste_caderii_de_legatura),
    ('RESTAURARE nu renunta unde Vehicle ar renunta',
     test_restaurare_nu_renunta_unde_Vehicle_ar_renunta),
    ('restored e afirmatie verificata', test_restored_e_afirmatie_verificata),
    ('release inainte de prima scriere',
     test_release_inainte_de_prima_scriere),
    ('ANGLE_MAX nu e atins niciodata', test_ANGLE_MAX_nu_e_atins_niciodata),
    ('benzile acopera toata plaja', test_benzile_acopera_toata_plaja),
    ('numele sunt cele de pe 4.5.7', test_numele_sunt_cele_de_pe_4_5_7),
    ('traducerea 4.5.7 e cea din make_parm_45',
     test_traducerea_4_5_7_e_cea_din_make_parm_45),
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
