#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste de regresie pentru nova/safety.py (15.2.9, 15.2.10), fara SITL.

    python3 tools/test_safety.py

Supervizorul e cod de siguranta, deci fiecare prag si fiecare exceptie are
nevoie de un test care esueaza daca cineva le schimba din greseala. Exceptia
de la FINAL_DESCENT in special: fara test, un refactor care "uniformizeaza"
fazele ar reintroduce abortul in ultimul metru la fiecare incercare.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova.safety import (Action, DETECTION_MONITORED_PHASES,  # noqa: E402
                         SafetySupervisor)
from nova.rc import STICK_NOISE_PWM  # noqa: E402
from nova.vehicle import (MODE_BRAKE, MODE_LAND,  # noqa: E402
                          MODE_LOITER, MODE_RTL)


class StubVehicle:
    def __init__(self):
        self.x = self.y = 0.0
        self.z = -6.0
        self.vz = 0.5
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = True
        self.armed = True
        self.mode = MODE_LAND
        self.time_boot_ms = 12345
        self.mode_reqs = []
        self.accept_mode = True      # simuleaza un FC care ignora comenzile
        self.params = {'RC1_TRIM': 1500, 'RC2_TRIM': 1500,
                       'RC3_TRIM': 1100, 'RC4_TRIM': 1500}
        self.rc = (1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000)
        self.rc_t = 0.0

    def request_param(self, name):
        pass

    def set_rc(self, ch, value, t=None):
        """ch e 1-indexat, ca in RC_CHANNELS."""
        rc = list(self.rc)
        rc[ch - 1] = value
        self.rc = tuple(rc)
        if t is not None:
            self.rc_t = t

    @property
    def alt(self):
        return -self.z

    def request_mode(self, m):
        self.mode_reqs.append(m)
        if self.accept_mode:
            self.mode = m

    def mode_name(self):
        return str(self.mode)


def build():
    v = StubVehicle()
    v.rc_t = 100.0
    sup = SafetySupervisor(v, verbose=False)
    sup.arm(100.0, 0.0, 0.0, 6.0)
    return v, sup


# --- teste -----------------------------------------------------------------

def test_detectie_veche_declanseaza_brake():
    v, sup = build()
    assert sup.update(100.1, 0.1, 'DESCEND_TRACK') == Action.NONE
    act = sup.update(100.7, 0.6, 'DESCEND_TRACK')
    assert act == Action.BRAKE, f"actiune {act}"
    assert v.mode_reqs and v.mode_reqs[0] == MODE_BRAKE
    return f"BRAKE la varsta 0.60 s (prag {sup.detection_max_age_s:.2f})"


def test_abort_abia_dupa_5_ratari_consecutive():
    """Decizia echipei, 25.09.2026: BRAKE dupa 5 cadre CONSECUTIVE fara
    marker, nu la prima pauza de 0.5 s. Caz negativ inclus: sub 5 ratari,
    aceeasi varsta care inainte dadea BRAKE (0.6 s) acum NU mai da."""
    from nova.safety import DETECTION_MAX_MISSES
    assert DETECTION_MAX_MISSES == 5
    v, sup = build()
    assert sup.update(100.1, 0.6, 'DESCEND_TRACK', miss_streak=4) == Action.NONE, (
        "4 ratari consecutive au declansat - regula echipei cere 5")
    assert not v.mode_reqs
    act = sup.update(100.2, 0.7, 'DESCEND_TRACK', miss_streak=5)
    assert act == Action.BRAKE and v.mode_reqs[0] == MODE_BRAKE, act
    ev = [e for e in sup.log if e.monitor == 'detection_age'][-1]
    assert '5 cadre consecutive' in ev.detail, ev.detail
    return "4 ratari la 0.6 s -> nimic; 5 ratari -> BRAKE, cu motivul in log"


def test_detector_blocat_tot_opreste_coborarea():
    """Plasa: un detector blocat nu proceseaza cadre, deci contorul de
    ratari nu creste - fara plafon, coborarea oarba ar continua la infinit.
    NEGATIV pentru regula noua: cu contor mic dar varsta peste plafon,
    BRAKE oricum."""
    from nova.safety import DETECTION_HARD_MAX_AGE_S
    v, sup = build()
    assert sup.update(100.1, 1.4, 'DESCEND_TRACK', miss_streak=1) == Action.NONE
    act = sup.update(100.2, DETECTION_HARD_MAX_AGE_S + 0.1, 'DESCEND_TRACK',
                     miss_streak=1)
    assert act == Action.BRAKE, f"detector blocat, {act}"
    ev = [e for e in sup.log if e.monitor == 'detection_age'][-1]
    assert 'blocat' in ev.detail, ev.detail
    return f"1 ratare dar {DETECTION_HARD_MAX_AGE_S + 0.1:.1f} s -> BRAKE"


def test_fara_contor_ramane_regula_pe_timp():
    """Un detector care nu numara ratari (sim sintetic, teste vechi) pastreaza
    regula veche - altfel introducerea contorului ar fi facut monitorul inert
    exact acolo unde nu e cablat."""
    v, sup = build()
    act = sup.update(100.7, 0.6, 'DESCEND_TRACK', miss_streak=None)
    assert act == Action.BRAKE, act
    return "miss_streak=None -> BRAKE la 0.6 s, ca inainte"


def test_incercarea_noua_elibereaza_zavorul_confirmat():
    """b14 (§5.64): BRAKE -> pilotul preia (OVERRIDE, confirmat) -> poarta
    accepta un handover NOU -> masina de stari comanda LAND -> zavorul inca
    activ re-comanda LOITER in aceeasi secunda. Trei acceptari, zero
    coborari. O incercare noua trebuie sa porneasca cu zavorul eliberat."""
    v, sup = build()
    act = sup.update(100.7, 0.6, 'DESCEND_TRACK')          # BRAKE
    assert act == Action.BRAKE and v.mode == MODE_BRAKE
    sup.update(100.8, 0.7, 'DESCEND_TRACK')                  # confirmare
    assert sup._mode_confirmed is not None
    # masina de stari a vazut modul schimbat -> IDLE; zavorul ramane
    sup.update(101.0, 0.9, 'IDLE')
    assert sup.latched == Action.BRAKE
    n_before = len(v.mode_reqs)
    # incercare NOUA: poarta a acceptat, masina de stari a comandat LAND
    v.mode = MODE_LAND
    sup.update(120.0, 0.05, 'ACQUIRE')
    assert sup.latched == Action.NONE, "zavorul confirmat a supravietuit"
    assert any(e.monitor == 'latch_release' for e in sup.log)
    sup.update(120.1, 0.05, 'DESCEND_TRACK')
    assert v.mode == MODE_LAND and len(v.mode_reqs) == n_before, (
        f"zavorul vechi a re-comandat modul: {v.mode_reqs[n_before:]}")
    assert sup.armed, "supervizorul nu s-a re-armat pe incercarea noua"
    # si supravegheaza din nou: pierderea detectiei da BRAKE, nu tacere
    act = sup.update(121.0, 0.9, 'DESCEND_TRACK')
    assert act == Action.BRAKE, act
    return "BRAKE confirmat -> IDLE -> ACQUIRE nou: eliberat, re-armat, vigilent"


def test_B2_zavorul_pasiv_sau_epuizat_se_elibereaza_la_incercare_noua():
    """26.09.2026 seara (B2), reprodus offline. BRAKE pierdut pe serial,
    pilotul pune STABILIZE in fereastra de reincercare -> PASIV, dar
    NECONFIRMAT. Regula veche elibera doar zavorul confirmat: la
    incercarea noua supervizorul ramanea zavorat, iesea din update()
    inaintea monitoarelor, si coborarea noua rula FARA supraveghere:
    marker pierdut 3 s -> zero comenzi. La fel dupa mode_fail (epuizat)."""
    from nova.safety import MODE_RETRY_MAX
    from nova.vehicle import MODE_STABILIZE

    def incercare_noua_supravegheata(v, sup):
        v.accept_mode = True
        v.mode = MODE_LAND
        sup.update(300.0, 0.05, 'ACQUIRE')
        assert sup.latched == Action.NONE and sup.armed and not sup.passive, (
            Action.NAMES[sup.latched], sup.armed, sup.passive)
        assert any(e.monitor == 'latch_release' for e in sup.log)
        sup.update(300.1, 0.05, 'DESCEND_TRACK')
        n = len(v.mode_reqs)
        act = sup.update(301.0, 0.9, 'DESCEND_TRACK', miss_streak=9)
        assert act == Action.BRAKE and v.mode_reqs[n:] == [MODE_BRAKE], (
            act, v.mode_reqs[n:])

    # 1. pasiv, neconfirmat: al treilea mod in fereastra de reincercare
    v, sup = build()
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')          # BRAKE cerut, pierdut
    v.mode = MODE_STABILIZE                          # pilotul, din comutator
    sup.update(101.0, 1.0, 'DESCEND_TRACK')
    sup.update(101.2, 1.0, 'IDLE')
    assert sup.passive and sup._mode_confirmed is None
    incercare_noua_supravegheata(v, sup)
    # 2. epuizat: mode_fail dupa MODE_RETRY_MAX, FC ramas in LAND
    v, sup = build()
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    for i in range(12):
        sup.update(101.0 + 0.5 * i, 1.0, 'DESCEND_TRACK')
    assert any(e.monitor == 'mode_fail' for e in sup.log)
    sup.update(110.0, 1.0, 'IDLE')
    incercare_noua_supravegheata(v, sup)
    return "pasiv neconfirmat si epuizat: eliberat la incercare noua, BRAKE din nou la marker pierdut"


def test_zavorul_neconfirmat_NU_se_elibereaza():
    """NEGATIV: daca FC-ul nu a adoptat inca modul comandat, o cerere noua
    nu are voie sa lase actiunea de siguranta nelivrata."""
    v, sup = build()
    v.accept_mode = False                    # FC ignora comenzile
    sup.update(100.7, 0.6, 'DESCEND_TRACK')  # BRAKE cerut, neconfirmat
    assert sup.latched == Action.BRAKE and sup._mode_confirmed is None
    sup.update(101.0, 0.9, 'IDLE')
    sup.update(120.0, 0.05, 'ACQUIRE')
    assert sup.latched == Action.BRAKE, "zavor neconfirmat eliberat"
    assert not any(e.monitor == 'latch_release' for e in sup.log)
    return "BRAKE neconfirmat -> ramane zavorat prin incercarea noua"


def test_INCIDENT_zavorul_confirmat_nu_retrimite_peste_pilot_sau_failsafe():
    """26.09.2026 (§5.66), din .bin: BRAKE confirmat 16:14:10; pilotul
    trece in STABILIZE 16:15:26.05 -> noi BRAKE la .07; pilotul LOITER
    16:15:31.66 -> noi BRAKE in aceeasi ms; failsafe-ul de baterie LAND
    16:19:04.01 -> noi BRAKE la .04. Drona a plutit pana la caderea
    bateriei. Un zavor CONFIRMAT nu mai trimite NIMIC: orice schimbare de
    mod de dupa e a pilotului sau a FC-ului si ii apartine."""
    v, sup = build()
    assert sup.update(100.7, 0.6, 'DESCEND_TRACK') == Action.BRAKE
    sup.update(100.8, 0.7, 'DESCEND_TRACK')
    assert v.mode == MODE_BRAKE and sup._mode_confirmed is not None
    n = len(v.mode_reqs)
    # masina de stari vede modul schimbat -> IDLE; zavorul ramane confirmat
    sup.update(101.0, 0.9, 'IDLE')
    # 1. pilotul, din comutator: LOITER
    v.mode = MODE_LOITER
    for t in (161.0, 161.5, 162.0, 165.0):
        sup.update(t, 60.0, 'IDLE')
    assert len(v.mode_reqs) == n, f"a retrimis peste pilot: {v.mode_reqs[n:]}"
    assert sup.passive and sup._want_mode is None
    ev = [e for e in sup.log if e.monitor == 'mode_taken']
    assert ev and 'PASIV' in ev[0].detail, ev
    assert 'PASIV' in sup.status()
    # 2. failsafe-ul FC-ului: LAND - tot nimic
    v.mode = MODE_LAND
    for t in (400.0, 400.5, 401.0):
        sup.update(t, 300.0, 'IDLE')
    assert len(v.mode_reqs) == n, f"a retrimis peste failsafe: {v.mode_reqs[n:]}"
    # 3. si niciun alt monitor nu mai poate declansa ceva cat timp pilotul
    #    zboara (IDLE): zavorul tine, chiar cu plafonul depasit
    v.z = -40.0                                   # peste plafon
    assert sup.update(402.0, 300.0, 'IDLE') == Action.BRAKE
    assert len(v.mode_reqs) == n
    # 4. doar o incercare NOUA acceptata de poarta il readuce la lucru
    v.mode = MODE_LAND
    sup.update(500.0, 0.05, 'ACQUIRE')
    assert sup.latched == Action.NONE and not sup.passive and sup.armed
    return ("BRAKE confirmat; LOITER (pilot), LAND (failsafe): 0 comenzi, "
            "PASIV; vigilent din nou doar la o incercare noua")


def test_B1_supervizorul_pasiv_nu_escaladeaza_la_override():
    """26.09.2026 seara (B1). Dupa mode_taken supervizorul e PASIV: pilotul
    sau un failsafe are modul. O miscare de mansa nu mai are peste ce sa
    escaladeze - LOITER ar pleca peste STABILIZE-ul pilotului sau peste
    LAND-ul failsafe-ului de baterie. Zero comenzi, zero evenimente."""
    from nova.vehicle import MODE_STABILIZE
    for luat_de, mod in (('pilot', MODE_STABILIZE), ('failsafe', MODE_LAND)):
        v, sup = build()
        sup.update(100.7, 0.6, 'DESCEND_TRACK')          # BRAKE
        sup.update(100.8, 0.7, 'DESCEND_TRACK')          # confirmat
        sup.update(101.0, 0.9, 'IDLE')                   # sm -> IDLE
        v.mode = mod                                     # cineva a luat modul
        sup.update(160.0, 60.0, 'IDLE')
        assert sup.passive and sup._want_mode is None
        n = len(v.mode_reqs)
        for t in (161.0, 161.1, 161.2, 161.3, 162.0):    # throttle +300, 1 s
            v.set_rc(3, 1400, t)
            sup.update(t, 60.0, 'IDLE')
        assert len(v.mode_reqs) == n, (
            f"{luat_de}: a comandat peste modul luat: {v.mode_reqs[n:]}")
        assert v.mode == mod and sup.latched == Action.BRAKE
        assert not any(e.monitor == 'pilot_override' for e in sup.log)
    # NEGATIV: zavor confirmat, FC INCA in BRAKE (nimeni nu a luat modul):
    # mansa escaladeaza la OVERRIDE -> LOITER, ca pana acum
    v, sup = build()
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    sup.update(100.8, 0.7, 'DESCEND_TRACK')
    sup.update(101.0, 0.9, 'IDLE')
    assert not sup.passive
    for t in (161.0, 161.1, 161.2):
        v.set_rc(3, 1400, t)
        sup.update(t, 60.0, 'IDLE')
    assert sup.latched == Action.OVERRIDE and v.mode == MODE_LOITER, (
        sup.latched, v.mode)
    return "PASIV (pilot / failsafe) + mansa: 0 comenzi; FC inca in BRAKE + mansa: LOITER"


def test_zavorul_neconfirmat_inca_retrimite_pana_la_plafon():
    """NEGATIV pentru reparatia de mai sus: cat timp FC-ul NU a adoptat
    modul, livrarea se reincearca (o comanda pierduta pe serial e reala),
    cu plafonul MODE_RETRY_MAX. Doar confirmarea o face finala."""
    from nova.safety import MODE_RETRY_MAX
    v, sup = build()
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    for i in range(12):
        sup.update(101.0 + 0.5 * i, 1.0, 'DESCEND_TRACK')
    assert len(v.mode_reqs) == MODE_RETRY_MAX, len(v.mode_reqs)
    assert not sup.passive
    return f"neconfirmat: {MODE_RETRY_MAX} comenzi, apoi mode_fail"


def test_INCIDENT_pilotul_castiga_si_in_fereastra_de_reincercare():
    """Completarea §5.66: cat timp comanda NU e confirmata, se reincearca
    (testul de mai sus) - dar numai cat FC-ul e inca in modul de dinainte.
    Daca FC-ul apare intr-un AL TREILEA mod (nici cel vechi, nici cel
    cerut), l-a pus pilotul sau un failsafe in fereastra noastra de
    reincercare, iar noi nu mai trimitem nimic. Fara asta, un comutator
    apasat in cele ~1.5 s de dupa declansare ar fi anulat de retrimitere."""
    from nova.safety import MODE_RETRY_MAX
    v, sup = build()
    v.mode = 9                                  # LAND: coborarea autonoma
    v.accept_mode = False                       # BRAKE nu ajunge (serial)
    sup.update(100.7, 0.6, 'DESCEND_TRACK')     # declansare -> 1 comanda
    sup.update(101.0, 1.0, 'DESCEND_TRACK')     # a doua, FC inca in LAND
    n = len(v.mode_reqs)
    assert 1 <= n < MODE_RETRY_MAX, n
    v.mode = 0                                  # pilotul: STABILIZE
    for i in range(10):
        sup.update(101.5 + 0.5 * i, 1.0, 'DESCEND_TRACK')
    assert len(v.mode_reqs) == n, f"a retrimis peste pilot: {v.mode_reqs}"
    assert sup.passive
    assert 'PASIV' in sup.status()
    # Cazul negativ: FC-ul ramane in modul VECHI -> e livrare, se reincearca.
    v, sup = build()
    v.mode = 9
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    for i in range(12):
        sup.update(101.0 + 0.5 * i, 1.0, 'DESCEND_TRACK')
    assert len(v.mode_reqs) == MODE_RETRY_MAX
    return "al treilea mod inainte de confirmare: 0 retrimiteri, PASIV"


def test_exceptia_final_descent():
    """Sub 0.38 m markerul iese din cadru prin constructie (5.2). Monitorul
    NU are voie sa se aplice acolo, altfel abortam in ultimul metru mereu."""
    for phase in ('FINAL_DESCENT', 'TOUCHDOWN_CONFIRM', 'ASCENT'):
        v, sup = build()
        v.z = -0.30
        act = sup.update(105.0, 12.0, phase)      # detectie veche de 12 s
        assert act == Action.NONE, f"{phase}: a declansat {act}"
        assert not v.mode_reqs, f"{phase}: a comandat {v.mode_reqs}"
    assert 'FINAL_DESCENT' not in DETECTION_MONITORED_PHASES
    return "fara declansare in FINAL_DESCENT / TOUCHDOWN_CONFIRM / ASCENT"


def test_detectie_lipsa_complet():
    v, sup = build()
    act = sup.update(100.1, None, 'DESCEND_TRACK')
    assert act == Action.BRAKE
    return "age=None tratat ca pierdere, nu ca 'inca nu stim'"


def test_raza_declanseaza_rtl():
    v, sup = build()
    v.x, v.y = 8.0, 0.0
    assert sup.update(100.1, 0.0, 'DESCEND_TRACK') == Action.NONE
    v2, sup2 = build()
    v2.x, v2.y = 11.0, 0.0
    act = sup2.update(100.1, 0.0, 'DESCEND_TRACK')
    assert act == Action.RTL, f"actiune {act}"
    assert v2.mode_reqs[0] == MODE_RTL
    return f"RTL la 11.0 m (prag {sup2.geofence_radius_m:.1f} m)"


def test_plafon_declanseaza_rtl():
    v, sup = build()
    v.z = -(6.0 + 31.0)          # 31 m peste originea de handover
    act = sup.update(100.1, 0.0, 'DESCEND_TRACK')
    assert act == Action.RTL, f"actiune {act}"
    return f"RTL la 31 m AGL (prag {sup.ceiling_agl_m:.0f} m)"


def test_cea_mai_severa_actiune_castiga():
    """Detectie veche (BRAKE) si raza depasita (RTL) simultan -> RTL."""
    v, sup = build()
    v.x = 12.0
    act = sup.update(100.9, 5.0, 'DESCEND_TRACK')
    assert act == Action.RTL, f"actiune {act}"
    assert sup.latched_monitor == 'geofence_radius', sup.latched_monitor
    return "RTL bate BRAKE"


def test_zavor_fara_revenire():
    """Semnalul redevine bun: actiunea NU se anuleaza."""
    v, sup = build()
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    assert sup.latched == Action.BRAKE
    for t in (101.0, 102.0, 103.0):
        act = sup.update(t, 0.01, 'DESCEND_TRACK')
        assert act == Action.BRAKE, f"la t={t} a revenit la {act}"
    return "ramane BRAKE si dupa ce detectia revine"


def test_zavorul_supravietuieste_dezarmarii():
    """Cand supervizorul comanda BRAKE, masina de stari isi incheie secventa
    si aplicatia dezarmeaza supervizorul. Comanda de mod trebuie sa continue
    pana o confirma FC-ul, altfel actiunea de siguranta esueaza tacut."""
    v, sup = build()
    v.accept_mode = False           # FC-ul ignora primele comenzi
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    n_after_trigger = len(v.mode_reqs)
    sup.disarm(100.8, 'masina de stari a iesit din secventa')
    assert not sup.armed
    t = 101.0
    for _ in range(10):
        sup.update(t, 0.01, 'IDLE')
        t += 0.35
    assert len(v.mode_reqs) > n_after_trigger, (
        "dupa dezarmare nu s-a mai reincercat comanda de mod")
    assert all(m == MODE_BRAKE for m in v.mode_reqs)
    return f"{len(v.mode_reqs)} comenzi BRAKE, reincercate si dupa dezarmare"


def test_confirmare_independenta_a_modului():
    """Logul trebuie sa distinga 'am comandat' de 'FC a confirmat'."""
    v, sup = build()
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    t = 101.0
    for _ in range(3):
        sup.update(t, 0.01, 'DESCEND_TRACK')
        t += 0.35
    assert not any(e.monitor == 'mode_confirm' for e in sup.log), \
        "a raportat confirmare desi FC-ul nu a schimbat modul"
    v.accept_mode = True
    v.mode = MODE_BRAKE
    sup.update(t, 0.01, 'DESCEND_TRACK')
    assert any(e.monitor == 'mode_confirm' for e in sup.log), \
        "nu a inregistrat confirmarea cand FC-ul a adoptat modul"
    return "confirmarea vine din starea FC-ului, nu din decizia proprie"


def test_esec_de_mod_raportat():
    v, sup = build()
    v.accept_mode = False
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    t = 101.0
    for _ in range(12):
        sup.update(t, 0.01, 'DESCEND_TRACK')
        t += 0.35
    assert any(e.monitor == 'mode_fail' for e in sup.log), \
        "nu a raportat ca FC-ul refuza modul"
    return "esecul de comutare ajunge in log, nu se pierde"


def test_log_are_ceasul_fc():
    v, sup = build()
    sup.update(100.7, 0.6, 'DESCEND_TRACK')
    ev = [e for e in sup.log if e.monitor == 'detection_age'][0]
    assert ev.time_boot_ms == 12345, ev.time_boot_ms
    assert 'boot_ms=12345' in str(ev)
    return "fiecare eveniment poarta time_boot_ms (6.2.1.30)"


def test_inactiv_inainte_de_handover():
    """In afara segmentului autonom nu comanda nimic, oricat de rea ar fi
    detectia."""
    for phase in ('IDLE', 'HANDOVER_CHECK', 'REJECT', 'HANDBACK'):
        v = StubVehicle()
        sup = SafetySupervisor(v, verbose=False)
        assert sup.update(100.0, 99.0, phase) == Action.NONE, phase
        assert not v.mode_reqs, f"{phase}: a comandat {v.mode_reqs}"
        assert not sup.armed, f"{phase}: s-a armat"
    return "IDLE / HANDOVER_CHECK / REJECT / HANDBACK: inert"


def test_auto_armare_din_faza():
    """REGRESIE. Prima varianta se arma pe tranzitia IDLE -> DESCEND_TRACK.
    Cand intrarea a devenit IDLE -> HANDOVER_CHECK -> ACQUIRE ->
    DESCEND_TRACK, conditia nu s-a mai potrivit niciodata si supervizorul a
    ramas inert, tacut, cu toate monitoarele oprite. Legarea de faza face
    imposibila reaparitia."""
    v = StubVehicle()
    v.rc_t = 100.0
    sup = SafetySupervisor(v, verbose=False)
    assert not sup.armed
    sup.update(100.0, 0.0, 'ACQUIRE')
    assert sup.armed, "nu s-a armat la intrarea in ACQUIRE"
    assert abs(sup.origin_alt - v.alt) < 1e-6, "origine gresita"
    # si prin lantul complet de faze, fara nicio tranzitie speciala
    for phase in ('DESCEND_TRACK', 'SCORING_CAPTURE', 'FINAL_DESCENT',
                  'TOUCHDOWN_CONFIRM', 'ASCENT'):
        sup.update(100.1, 0.0, phase)
        assert sup.armed, f"s-a dezarmat in {phase}"
    sup.update(100.2, 0.0, 'HANDBACK')
    assert not sup.armed, "nu s-a dezarmat la HANDBACK"
    return "armat in ACQUIRE..ASCENT, dezarmat in rest"


def test_override_dupa_auto_armare():
    """Cazul raportat: manseta miscata in coborarea autonoma trebuie sa
    opreasca secventa, fara ca aplicatia sa armeze ceva explicit."""
    v = StubVehicle()
    v.rc_t = 100.0
    sup = SafetySupervisor(v, verbose=False)
    sup.update(100.0, 0.0, 'DESCEND_TRACK')      # se armeaza singur
    t = 100.05
    v.set_rc(1, 1500 + 400, t)                   # mansa la maxim
    sup.update(t, 0.0, 'DESCEND_TRACK')
    t += 0.15
    v.rc_t = t
    act = sup.update(t, 0.0, 'DESCEND_TRACK')
    assert act == Action.OVERRIDE, f"actiune {act}"
    assert v.mode_reqs[-1] == MODE_LOITER, v.mode_reqs
    assert sup.passive
    return "mansa la maxim -> LOITER, fara armare explicita din aplicatie"


# --- 15.3.1 / 15.1.7: override pe manse ------------------------------------

def test_override_declanseaza_pasiv():
    v, sup = build()
    t = 100.0
    v.set_rc(1, 1500 + 200, t)          # mansa impinsa clar
    assert sup.update(t + 0.02, 0.0, 'DESCEND_TRACK') == Action.NONE, \
        "a declansat fara persistenta"
    t += 0.15
    v.rc_t = t
    act = sup.update(t, 0.0, 'DESCEND_TRACK')
    assert act == Action.OVERRIDE, f"actiune {act}"
    assert sup.passive, "nu a intrat in pasiv"
    assert v.mode_reqs[0] == MODE_LOITER
    return "OVERRIDE dupa depasire sustinuta, mod pilotabil comandat"


def test_zgomotul_de_mansa_nu_declanseaza():
    """Cazul negativ: fluctuatie la nivelul zgomotului, sustinuta mult timp."""
    v, sup = build()
    t = 100.0
    for i in range(200):               # 2 s de zgomot
        v.set_rc(1, 1500 + (STICK_NOISE_PWM if i % 2 else -STICK_NOISE_PWM), t)
        assert sup.update(t, 0.0, 'DESCEND_TRACK') == Action.NONE, \
            f"zgomotul de {STICK_NOISE_PWM} PWM a declansat override la i={i}"
        t += 0.01
    assert not sup.passive
    return f"{STICK_NOISE_PWM} PWM timp de 2 s nu declanseaza"


def test_varf_izolat_nu_declanseaza():
    """Cazul negativ: depasire mare, dar mai scurta decat OVERRIDE_HOLD_S."""
    v, sup = build()
    t = 100.0
    v.set_rc(1, 1500 + 300, t)
    sup.update(t, 0.0, 'DESCEND_TRACK')
    t += 0.05                           # sub 0.1 s
    v.set_rc(1, 1500, t)
    act = sup.update(t, 0.0, 'DESCEND_TRACK')
    assert act == Action.NONE, f"un varf de 50 ms a declansat {act}"
    return "varf de 50 ms ignorat (prag 100 ms)"


def test_neutru_nu_e_1500():
    """Throttle-ul nu se auto-centreaza: referinta e ce s-a memorat, nu 1500."""
    v = StubVehicle()
    v.rc = (1500, 1500, 1100, 1500, 1000, 1000, 1000, 1000)
    v.rc_t = 100.0
    sup = SafetySupervisor(v, verbose=False)
    sup.arm(100.0, 0.0, 0.0, 6.0)
    assert sup.override.neutral[2] == 1100, sup.override.neutral
    t = 100.0
    for _ in range(30):                 # throttle stationar la 1100
        t += 0.01
        v.rc_t = t
        assert sup.update(t, 0.0, 'DESCEND_TRACK') == Action.NONE, \
            "a considerat throttle 1100 drept override fata de 1500"
    return "neutru memorat (1100 pe throttle), nu centrul teoretic"


def test_override_escaladeaza_peste_rtl():
    """Pilotul a preluat: nu ii comandam RTL peste mana (15.1.7).

    Raza se depaseste instantaneu, override-ul cere 0.1 s de persistenta,
    deci RTL se zavoraste primul. Escaladarea trebuie sa rupa zavorul."""
    v, sup = build()
    v.x = 12.0                          # si in afara razei
    t = 100.0
    v.set_rc(1, 1500 + 200, t)
    assert sup.update(t, 0.0, 'DESCEND_TRACK') == Action.RTL
    t += 0.15
    v.rc_t = t
    act = sup.update(t, 0.0, 'DESCEND_TRACK')
    assert act == Action.OVERRIDE, f"actiune {act}"
    assert sup.latched_monitor == 'pilot_override', sup.latched_monitor
    assert sup.passive
    assert v.mode_reqs[-1] == MODE_LOITER, v.mode_reqs
    return "RTL zavorat, apoi escaladat la OVERRIDE de pilot"


def test_override_si_in_final_descent():
    """Spre deosebire de monitorul de detectie, override-ul nu are exceptii
    de faza: pilotul poate prelua si in ultimul metru."""
    v, sup = build()
    v.z = -0.30
    t = 100.0
    v.set_rc(2, 1500 + 200, t)
    sup.update(t, 0.0, 'FINAL_DESCENT')
    t += 0.15
    v.rc_t = t
    assert sup.update(t, 0.0, 'FINAL_DESCENT') == Action.OVERRIDE
    return "se aplica in toate fazele"


def test_flux_rc_vechi_nu_declanseaza():
    """Fara RC proaspat nu ne putem pronunta; nu inventam un override."""
    v, sup = build()
    v.set_rc(1, 1500 + 300, 100.0)
    t = 102.0                            # RC vechi de 2 s
    assert sup.update(t, 0.0, 'DESCEND_TRACK') == Action.NONE
    return "RC mai vechi de RC_STALE_S ignorat"


def test_latenta_override_masurata():
    v, sup = build()
    t = 100.0
    v.set_rc(1, 1500 + 200, t)
    sup.update(t, 0.0, 'DESCEND_TRACK')      # prima depasire
    t += 0.12
    v.rc_t = t
    sup.update(t, 0.0, 'DESCEND_TRACK')      # declansare + comanda
    t += 0.01
    sup.update(t, 0.0, 'IDLE')               # FC a confirmat LOITER
    lat = sup.override.latency_s
    assert lat is not None, "latenta nemasurata"
    assert 0.10 <= lat <= 0.20, f"latenta {lat}"
    assert any('latenta override' in e.detail for e in sup.log), \
        "latenta nu a ajuns in log"
    return f"latenta {lat * 1000:.0f} ms, scrisa in log (buget 250 ms)"


def test_fereastra_de_asezare():
    """B3.1: dupa comutarea AUX, tranzitoriul arcurilor nu trebuie sa anuleze
    incercarea. Referinta de neutru se memoreaza DUPA asezare."""
    v = StubVehicle()
    v.rc_t = 100.0
    sup = SafetySupervisor(v, verbose=False)
    ov = sup.override
    ov.begin_settle(100.0)
    v.set_rc(1, 1500 + 300, 100.2)           # tranzitoriu de arc
    assert not ov.settled(100.5), "fereastra s-a inchis prea devreme"
    assert ov.update(100.5) is False, "a declansat in fereastra de asezare"
    assert ov.settled(101.1), "fereastra nu s-a inchis dupa 1.0 s"
    v.set_rc(1, 1500, 101.1)
    ov.capture_neutral(101.1)
    assert ov.neutral[0] == 1500, ov.neutral
    return "1.0 s de asezare, neutru memorat la validare"


# --- ExtNav (27.09.2026) -----------------------------------------------------

class ExtNavStub(StubVehicle):
    """StubVehicle + ce cere supervizorul ExtNav: validitatea EKF si
    varsta legaturii."""

    def __init__(self):
        super().__init__()
        self.mode = 4                    # GUIDED
        self.ekf_ok = True
        self.ekf_t = 100.0
        self.hb_t = 100.0

    def ekf_pos_horiz_ok(self):
        return self.ekf_ok

    def time_since_heartbeat(self, now=None):
        return None if self.hb_t is None else (now - self.hb_t)


def build_extnav():
    from nova.safety import ExtNavSupervisor
    v = ExtNavStub()
    v.rc_t = 100.0
    exits = []
    sup = ExtNavSupervisor(v, verbose=False, on_exit=exits.append)
    return v, sup, exits


def ruleaza_extnav(v, sup, faze, t0, dt=0.1, n=None):
    t = t0
    for faza in faze if n is None else [faze] * n:
        v.rc_t = t
        v.ekf_t = t if v.ekf_t is not None else None
        v.hb_t = t if v.hb_t is not None else None
        sup.update(t, None, faza)
        t += dt
    return t


def test_ExtNav_fara_monitor_de_detectie_si_fara_comenzi_de_mod():
    """Brief §6: intre detectii pozitia o tine EKF-ul; o detectie lipsa nu
    e o defectiune, e logica de fereastra a masinii de stari. Supervizorul
    ExtNav nu trimite NICIO comanda de mod: cere EXIT si atat."""
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'GATE_SEARCH', 100.0, n=5)
    assert sup.armed, "nu s-a armat din faza"
    t = ruleaza_extnav(v, sup, 'MOVE', t, n=30)         # 3 s fara detectii
    assert sup.latched == Action.NONE and not exits
    assert v.mode_reqs == [], v.mode_reqs
    # iar argumentele vechi (varsta detectiei, ratari) sunt ignorate
    assert sup.update(t, 99.0, 'MOVE', miss_streak=50) == Action.NONE
    assert v.mode_reqs == [] and not exits
    return "3 s fara detectii in MOVE: nimic; 0 comenzi de mod"


def test_ExtNav_EKF_invalid_cere_EXIT_o_singura_data():
    """Validitatea pozitiei EKF e monitorul nou. Invalid in MOVE -> EXIT,
    predat masinii de stari o data; apoi nimic pana la o incercare noua."""
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'GATE_SEARCH', 100.0, n=3)
    t = ruleaza_extnav(v, sup, 'ENGAGE', t, n=3)
    t = ruleaza_extnav(v, sup, 'MOVE', t, n=3)
    v.ekf_ok = False
    v.rc_t = v.ekf_t = v.hb_t = t
    assert sup.update(t, None, 'MOVE') == Action.EXIT
    assert exits and exits[0].startswith('ekf_position'), exits
    assert sup.latched == Action.EXIT and 'EXIT' in sup.status()
    n = len(exits)
    ruleaza_extnav(v, sup, 'MOVE', t + 0.1, n=10)
    ruleaza_extnav(v, sup, 'IDLE', t + 1.2, n=5)
    assert len(exits) == n, "EXIT cerut de mai multe ori"
    assert v.mode_reqs == [], "supervizorul ExtNav a comandat un mod"
    # incercare noua: zavorul se elibereaza, re-armare, iar EKF ok -> nimic
    v.ekf_ok = True
    t2 = ruleaza_extnav(v, sup, 'GATE_SEARCH', t + 2.0, n=3)
    assert sup.latched == Action.NONE and sup.armed
    assert any(e.monitor == 'latch_release' for e in sup.log)
    ruleaza_extnav(v, sup, 'MOVE', t2, n=5)
    assert len(exits) == n
    return "EKF invalid in MOVE -> EXIT o data, 0 moduri; eliberat la incercare noua"


def test_ExtNav_fara_raport_EKF_e_necunoscut_nu_valid():
    """Fara EKF_STATUS_REPORT (None) mai mult de 2 s cat vehiculul e
    ghidat pe EKF -> EXIT. In GATE_SEARCH (inainte de comutare) lipsa
    raportului nu conteaza."""
    v, sup, exits = build_extnav()
    v.ekf_ok = None
    v.ekf_t = None
    t = ruleaza_extnav(v, sup, 'GATE_SEARCH', 100.0, n=40)   # 4 s
    assert not exits, exits
    t = ruleaza_extnav(v, sup, 'MOVE', t, n=15)              # 1.5 s
    assert not exits, "a iesit inainte de prag"
    t = ruleaza_extnav(v, sup, 'MOVE', t, n=10)              # 2.5 s
    assert exits and 'EKF_STATUS_REPORT' in exits[0], exits
    # raport vechi (a fost, dar a incetat): la fel
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'MOVE', 100.0, n=3)
    v.ekf_t = t - 3.0
    v.rc_t = v.hb_t = t
    assert sup.update(t, None, 'MOVE') == Action.EXIT and 'vechi' in exits[0]
    return "None > 2 s in MOVE -> EXIT; in GATE_SEARCH nu; raport vechi -> EXIT"


def test_ExtNav_celelalte_monitoare_dau_tot_EXIT():
    """Manse, legatura, inclinare, raza, plafon, rata de coborare: pastrate
    (brief §6), toate cu aceeasi actiune, EXIT, si fara comenzi de mod.
    Legatura si EKF-ul doar dupa ENGAGE; manse si geometrie in toate fazele."""
    from nova.rc import OVERRIDE_HOLD_S
    cazuri = []
    # manse in DESCEND
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'DESCEND', 100.0, n=3)
    for i in range(4):
        v.set_rc(2, 1500 + 300, t)
        v.ekf_t = v.hb_t = t
        act = sup.update(t, None, 'DESCEND')
        t += OVERRIDE_HOLD_S / 2 + 0.01
    assert act == Action.EXIT and exits[0].startswith('pilot_override'), exits
    assert sup.passive
    cazuri.append('manse')
    # legatura in MOVE, dar nu in GATE_SEARCH
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'GATE_SEARCH', 100.0, n=3)
    v.hb_t = t - 5.0
    v.rc_t = v.ekf_t = t
    assert sup.update(t, None, 'GATE_SEARCH') == Action.NONE
    assert sup.update(t + 0.1, None, 'MOVE') == Action.EXIT and exits[0].startswith('link_age')
    cazuri.append('legatura')
    # inclinare 35 grade, 0.3 s, in CENTER_CHECK
    v, sup, exits = build_extnav()
    t = ruleaza_extnav(v, sup, 'CENTER_CHECK', 100.0, n=3)
    v.roll = 35 * 3.14159265 / 180
    t = ruleaza_extnav(v, sup, 'CENTER_CHECK', t, n=5)
    assert exits and exits[0].startswith('tilt'), exits
    cazuri.append('inclinare')
    # raza si plafon in FINAL_ALIGN, rata de coborare in LAND
    for camp, val, mon, faza in (('x', 12.0, 'geofence_radius', 'FINAL_ALIGN'),
                                 ('z', -40.0, 'ceiling', 'FINAL_ALIGN'),
                                 ('vz', 3.0, 'descent_rate', 'LAND')):
        v, sup, exits = build_extnav()
        t = ruleaza_extnav(v, sup, faza, 100.0, n=3)
        setattr(v, camp, val)
        ruleaza_extnav(v, sup, faza, t, n=8)
        assert exits and exits[0].startswith(mon), (mon, exits)
        assert v.mode_reqs == []
        cazuri.append(mon)
    return ', '.join(cazuri) + ' -> EXIT, 0 comenzi de mod'


def test_ExtNav_nu_schimba_supervizorul_PLND():
    """Varianta e o subclasa; constantele si fazele supervizorului PLND
    raman exact cele de dinainte (simulatorul zboara pe ele)."""
    from nova.safety import (AUTONOMOUS_PHASES, DETECTION_MONITORED_PHASES,
                             EXTNAV_PHASES, EXTNAV_ENGAGED_PHASES)
    assert AUTONOMOUS_PHASES == ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE',
                                 'FINAL_DESCENT', 'TOUCHDOWN_CONFIRM', 'ASCENT')
    assert DETECTION_MONITORED_PHASES == ('ACQUIRE', 'DESCEND_TRACK',
                                          'SCORING_CAPTURE')
    v, sup = build()
    assert sup.autonomous_phases == AUTONOMOUS_PHASES
    assert sup.link_phases == DETECTION_MONITORED_PHASES
    assert not set(EXTNAV_PHASES) & set(AUTONOMOUS_PHASES)
    assert set(EXTNAV_ENGAGED_PHASES) < set(EXTNAV_PHASES)
    # supervizorul PLND nu reactioneaza la fazele ExtNav si invers
    sup.update(100.5, 0.0, 'MOVE')
    assert not sup.armed
    return "fazele PLND neschimbate; seturile de faze disjuncte"


TESTS = [
    ('ExtNav: fara monitor de detectie, fara comenzi de mod',
     test_ExtNav_fara_monitor_de_detectie_si_fara_comenzi_de_mod),
    ('ExtNav: EKF invalid -> EXIT o singura data',
     test_ExtNav_EKF_invalid_cere_EXIT_o_singura_data),
    ('ExtNav: fara raport EKF = necunoscut, nu valid',
     test_ExtNav_fara_raport_EKF_e_necunoscut_nu_valid),
    ('ExtNav: celelalte monitoare dau tot EXIT',
     test_ExtNav_celelalte_monitoare_dau_tot_EXIT),
    ('ExtNav: supervizorul PLND neschimbat',
     test_ExtNav_nu_schimba_supervizorul_PLND),
    ('detectie veche -> BRAKE', test_detectie_veche_declanseaza_brake),
    ('abort abia dupa 5 ratari consecutive',
     test_abort_abia_dupa_5_ratari_consecutive),
    ('NEGATIV: detector blocat tot opreste coborarea',
     test_detector_blocat_tot_opreste_coborarea),
    ('fara contor ramane regula pe timp',
     test_fara_contor_ramane_regula_pe_timp),
    ('INCIDENT: zavorul confirmat nu retrimite peste pilot/failsafe',
     test_INCIDENT_zavorul_confirmat_nu_retrimite_peste_pilot_sau_failsafe),
    ('B2: zavorul pasiv/epuizat se elibereaza la incercare noua',
     test_B2_zavorul_pasiv_sau_epuizat_se_elibereaza_la_incercare_noua),
    ('B1: supervizorul PASIV nu escaladeaza la override',
     test_B1_supervizorul_pasiv_nu_escaladeaza_la_override),
    ('NEGATIV: zavorul neconfirmat inca retrimite pana la plafon',
     test_zavorul_neconfirmat_inca_retrimite_pana_la_plafon),
    ('INCIDENT: pilotul castiga si in fereastra de reincercare',
     test_INCIDENT_pilotul_castiga_si_in_fereastra_de_reincercare),
    ('incercarea noua elibereaza zavorul confirmat',
     test_incercarea_noua_elibereaza_zavorul_confirmat),
    ('NEGATIV: zavorul neconfirmat NU se elibereaza',
     test_zavorul_neconfirmat_NU_se_elibereaza),
    ('exceptia FINAL_DESCENT', test_exceptia_final_descent),
    ('detectie complet absenta', test_detectie_lipsa_complet),
    ('raza -> RTL', test_raza_declanseaza_rtl),
    ('plafon -> RTL', test_plafon_declanseaza_rtl),
    ('cea mai severa actiune castiga', test_cea_mai_severa_actiune_castiga),
    ('zavor fara revenire', test_zavor_fara_revenire),
    ('zavorul supravietuieste dezarmarii', test_zavorul_supravietuieste_dezarmarii),
    ('confirmare independenta a modului', test_confirmare_independenta_a_modului),
    ('esec de mod raportat', test_esec_de_mod_raportat),
    ('log cu ceasul FC', test_log_are_ceasul_fc),
    ('inactiv inainte de handover', test_inactiv_inainte_de_handover),
    ('REGRESIE: auto-armare din faza', test_auto_armare_din_faza),
    ('override dupa auto-armare', test_override_dupa_auto_armare),
    ('override -> pasiv definitiv', test_override_declanseaza_pasiv),
    ('NEGATIV: zgomot de mansa', test_zgomotul_de_mansa_nu_declanseaza),
    ('NEGATIV: varf izolat', test_varf_izolat_nu_declanseaza),
    ('NEGATIV: neutru != 1500', test_neutru_nu_e_1500),
    ('override escaladeaza peste RTL', test_override_escaladeaza_peste_rtl),
    ('override si in FINAL_DESCENT', test_override_si_in_final_descent),
    ('NEGATIV: flux RC vechi', test_flux_rc_vechi_nu_declanseaza),
    ('latenta override masurata', test_latenta_override_masurata),
    ('fereastra de asezare la handover', test_fereastra_de_asezare),
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
