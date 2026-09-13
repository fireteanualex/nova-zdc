#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru H1: reconectarea la FC si monitorul de link.

    python3 tools/test_link.py

Fara SITL si fara serial: legatura mavutil e inlocuita cu o falsa, care se
poate pune in orice stare - port inexistent, port deschis fara FC in spate,
scriere care esueaza la mijloc. Astea sunt exact starile greu de produs pe
banc si usor de produs cu un cablu prost pe teren.

Cazul negativ care conteaza: un port care nu raspunde NICIODATA. Reconectarea
trebuie sa reincerce la infinit, cu backoff, si bucla trebuie sa ramana
responsiva - adica `check_link()` sa se intoarca in milisecunde, nu sa
blocheze in `wait_heartbeat()`.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import safety as safety_mod                       # noqa: E402
from nova import vehicle as vehicle_mod                     # noqa: E402
from nova.safety import Action, SafetySupervisor            # noqa: E402
from nova.vehicle import HEARTBEAT_TIMEOUT_S, Vehicle       # noqa: E402


# --- legatura falsa ---------------------------------------------------------

class FakeMav:
    """`m.mav`: numara mesajele si poate fi pus sa arunce."""

    def __init__(self, link):
        self.link = link

    def __getattr__(self, name):
        if not name.endswith('_send'):
            raise AttributeError(name)

        def sender(*a, **kw):
            self.link.sent.append(name)
            if self.link.write_fails:
                raise OSError(5, 'Input/output error')
        return sender


class FakeLink:
    """Inlocuieste conexiunea mavutil.

    heartbeats=False imita un port care se deschide dar nu are FC in spate -
    cazul real cand cablul e in alt conector, sau FC-ul e nealimentat."""

    def __init__(self, heartbeats=True, write_fails=False):
        self.heartbeats = heartbeats
        self.write_fails = write_fails
        self.sent = []
        self.closed = False
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav(self)

    def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
        if not self.heartbeats:
            # Un port real ar bloca pana la timeout. Nu dormim in teste, dar
            # nici nu intoarcem un mesaj.
            return None
        if type in (None, 'HEARTBEAT'):
            return FakeHeartbeat()
        return None

    def close(self):
        self.closed = True


class FakeHeartbeat:
    def get_type(self):
        return 'HEARTBEAT'

    def get_srcComponent(self):
        return 1
    base_mode = 0
    custom_mode = 9


class Harness:
    """Fabrica de legaturi: controleaza ce primeste fiecare `connect`."""

    def __init__(self, *stari):
        # stari: lista de FakeLink de intors, in ordine. Ultima se repeta.
        self.stari = list(stari) or [FakeLink()]
        self.opens = 0
        self.cereri = []

    def __call__(self, conn, **kwargs):
        self.cereri.append((conn, kwargs))
        link = self.stari[min(self.opens, len(self.stari) - 1)]
        self.opens += 1
        if link is None:
            raise OSError(2, 'No such file or directory')
        return link


def make_vehicle(harness, conn='/dev/fake0'):
    vechi = vehicle_mod.mavutil.mavlink_connection
    vehicle_mod.mavutil.mavlink_connection = harness
    try:
        v = Vehicle(conn, baud=921600)
        v.m = harness(conn, baud=921600)
        v.link_verbose = False
        v._note_heartbeat()
        return v
    finally:
        vehicle_mod.mavutil.mavlink_connection = vechi


def with_harness(harness, fn):
    vechi = vehicle_mod.mavutil.mavlink_connection
    vehicle_mod.mavutil.mavlink_connection = harness
    try:
        return fn()
    finally:
        vehicle_mod.mavutil.mavlink_connection = vechi


# --- vehicul simulat pentru supervizor --------------------------------------

class SupVehicle:
    """Minimul de care are nevoie SafetySupervisor, plus starea de link."""

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.vz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.have_pos = True
        self.mode = 9
        self.time_boot_ms = 1234
        self.rc = (1500, 1500, 1500, 1500, 1500, 1500, 1000, 1000)
        self.rc_t = time.monotonic()
        self.link_healthy = True
        self.hb_age = 0.0
        self.moduri = []

    @property
    def alt(self):
        return -self.z

    def mode_name(self):
        return 'LAND'

    def time_since_heartbeat(self, now=None):
        return self.hb_age

    def request_mode(self, mode):
        if not self.link_healthy:
            return False
        self.moduri.append(mode)
        self.mode = mode
        return True


def make_sup(v=None):
    v = v or SupVehicle()
    sup = SafetySupervisor(v, verbose=False)
    sup.auto_arm = False
    sup.arm(0.0, 0.0, 0.0, 0.0)
    return v, sup


# --- teste: reconectare -----------------------------------------------------

def test_link_sanatos_nu_reconecteaza():
    h = Harness(FakeLink())
    v = make_vehicle(h)
    deschideri = h.opens
    t = 100.0
    v.hb_t = t
    for dt in (0.0, 1.0, 2.0, 2.9):
        assert v.check_link(t + dt) is True, f"cazut la {dt} s"
    assert h.opens == deschideri, "a reconectat degeaba"
    assert v.link_healthy and v.reconnect_attempts == 0
    a = v.time_since_heartbeat(t + 2.9)
    assert abs(a - 2.9) < 1e-9, a
    return f"pana la {HEARTBEAT_TIMEOUT_S:.0f} s: nicio reconectare"


def test_reconectare_reusita():
    """Heartbeat vechi -> redeschide, cere fluxurile din nou, reseteaza."""
    h = Harness(FakeLink(), FakeLink())
    v = make_vehicle(h)
    evenimente = []
    v.on_link_event = evenimente.append
    v.time_boot_ms = 4242          # ultima valoare primita de la FC
    vechi = h.opens
    t = 100.0
    v.hb_t = t - (HEARTBEAT_TIMEOUT_S + 0.5)

    ok = with_harness(h, lambda: v.check_link(t))
    assert ok is True, "nu a reconectat"
    assert h.opens == vechi + 1, f"{h.opens - vechi} deschideri"
    assert v.link_healthy and v.reconnects == 1
    assert v.reconnect_attempts == 0, "contorul nu s-a resetat dupa reusita"

    feluri = [e['kind'] for e in evenimente]
    assert feluri == ['link_down', 'reconnect_try', 'link_up'], (
        f"{feluri} - fiecare reconectare trebuie sa produca EXACT un link_up")
    # Fiecare eveniment poarta ceasul FC-ului, ca sa se poata alinia cu .bin
    # (6.2.1.30). E ultima valoare dinainte de cadere, deci o ancora, nu o
    # valoare curenta - dar fara ea reconectarea nu se poate localiza in log.
    fara_ceas = [e['kind'] for e in evenimente if e['time_boot_ms'] is None]
    assert not fara_ceas, f"evenimente fara time_boot_ms: {fara_ceas}"
    # fluxurile se cer din nou: FC-ul poate sa fi repornit si el
    ceruri = [s for s in h.stari[1].sent if 'command_long' in s]
    assert len(ceruri) >= 5, f"fluxuri recerute: {len(ceruri)}"
    return (f"1 redeschidere, {len(ceruri)} fluxuri recerute, contoare "
            f"resetate")


def test_NEGATIV_port_care_nu_raspunde_niciodata():
    """Cazul care conteaza: reincearca la infinit, cu backoff, fara sa blocheze.

    Trei lucruri simultan, si toate trei sunt necesare:
      - nu renunta niciodata
      - intervalele urmeaza 1, 2, 4, 8, 8, ... (plafon)
      - check_link() se intoarce in milisecunde, altfel bucla principala si
        supervizorul odata cu ea ar ingheta exact cand e nevoie de ei
    """
    h = Harness(FakeLink(), FakeLink(heartbeats=False))
    v = make_vehicle(h)
    v.on_link_event = lambda e: None
    t = 100.0
    v.hb_t = t - (HEARTBEAT_TIMEOUT_S + 0.5)

    momente, durate = [], []
    now = t
    for _ in range(400):                      # ~ multe minute de ceas simulat
        n_inainte = h.opens
        t0 = time.monotonic()
        with_harness(h, lambda: v.check_link(now))
        durate.append(time.monotonic() - t0)
        if h.opens > n_inainte:
            momente.append(now)
        now += 0.25

    assert not v.link_healthy, "s-a declarat sanatos fara HEARTBEAT"
    assert len(momente) >= 10, f"doar {len(momente)} incercari in 100 s"
    assert v.reconnect_attempts == len(momente)

    intervale = [round(b - a, 2) for a, b in zip(momente, momente[1:])]
    asteptat = [1.0, 2.0, 4.0, 8.0]
    assert intervale[:4] == asteptat, f"backoff {intervale[:4]}, astept {asteptat}"
    assert all(abs(i - 8.0) < 0.3 for i in intervale[4:]), (
        f"plafonul de 8 s nu se respecta: {intervale[4:]}")

    cel_mai_lung = max(durate)
    assert cel_mai_lung < 0.05, (
        f"check_link a blocat {cel_mai_lung * 1000:.0f} ms - bucla principala "
        f"ar ingheta")
    return (f"{len(momente)} incercari, backoff {intervale[:5]}, "
            f"cel mai lung apel {cel_mai_lung * 1000:.1f} ms")


def test_NEGATIV_portul_dispare_cu_totul():
    """Cablu scos: `mavlink_connection` arunca. Nu e o eroare fatala."""
    h = Harness(FakeLink(), None)
    v = make_vehicle(h)
    evenimente = []
    v.on_link_event = evenimente.append
    t = 100.0
    v.hb_t = t - 10.0
    for i in range(5):
        with_harness(h, lambda: v.check_link(t + i * 10.0))
    assert not v.link_healthy
    esecuri = [e for e in evenimente if e['kind'] == 'reconnect_fail']
    assert len(esecuri) >= 3, f"{len(esecuri)} esecuri raportate"
    assert 'FileNotFoundError' in esecuri[0]['detail'] or \
           'OSError' in esecuri[0]['detail'], esecuri[0]['detail']
    return f"{len(esecuri)} esecuri raportate, procesul e viu"


def test_emisia_suspendata_cat_e_cazuta():
    """Detectorul continua; doar octetii catre FC se opresc."""
    h = Harness(FakeLink())
    v = make_vehicle(h)
    link = h.stari[0]
    assert v.send_landing_target(0.1, 0.2, 5.0) is True
    assert v.send_distance(5.0) is True
    n = v.n_lt
    trimise = len(link.sent)

    v.link_healthy = False
    assert v.send_landing_target(0.1, 0.2, 5.0) is False
    assert v.send_distance(5.0) is False
    assert v.request_mode(5) is False
    assert v.send_takeoff(5.0) is False
    assert len(link.sent) == trimise, "au plecat octeti cu legatura cazuta"
    assert v.n_lt == n, "contorul a crescut fara sa se trimita nimic"

    v.link_healthy = True
    assert v.send_landing_target(0.1, 0.2, 5.0) is True
    assert v.n_lt == n + 1
    return "4 comenzi suprimate cat e cazuta, reluate dupa"


def test_scriere_esuata_marcheaza_legatura():
    """O scriere care arunca nu asteapta expirarea heartbeat-ului."""
    link = FakeLink()
    h = Harness(link)
    v = make_vehicle(h)
    evenimente = []
    v.on_link_event = evenimente.append
    assert v.link_healthy
    link.write_fails = True
    assert v.send_distance(5.0) is False
    assert not v.link_healthy, "scrierea a esuat dar legatura pare sanatoasa"
    assert [e['kind'] for e in evenimente] == ['link_down']
    return "OSError la scriere -> link_down imediat, fara sa arunce"


def test_pump_nu_moare_cu_portul_scos():
    class Rupt(FakeLink):
        def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
            raise OSError(5, 'Input/output error')

    h = Harness(Rupt())
    v = make_vehicle(h)
    v.pump()                    # nu trebuie sa arunce
    v.pump()
    return "recv_match care arunca nu omoara bucla"


# --- teste: monitorul de link din supervizor --------------------------------

def test_monitor_link_franeaza():
    v, sup = make_sup()
    v.hb_age = 0.1
    assert sup.update(1.0, 0.05, 'DESCEND_TRACK') == Action.NONE

    v2, sup2 = make_sup()
    v2.hb_age = safety_mod.LINK_MAX_AGE_S + 0.2
    act = sup2.update(1.0, 0.05, 'DESCEND_TRACK')
    assert act == Action.BRAKE, Action.NAMES[act]
    assert sup2.latched_monitor == 'link_age', sup2.latched_monitor
    ev = [e for e in sup2.log if e.monitor == 'link_age'][0]
    assert 'LANDING_TARGET' in ev.detail, ev.detail
    assert ev.time_boot_ms == 1234, "evenimentul nu poarta ceasul FC-ului"
    return (f"link vechi de {v2.hb_age:.1f} s (prag "
            f"{safety_mod.LINK_MAX_AGE_S:.1f}) -> BRAKE")


def test_monitor_link_aceleasi_faze_ca_detectia():
    """Nu se aplica in FINAL_DESCENT: acolo coborarea e oricum oarba."""
    rezultate = {}
    for faza in ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE',
                 'FINAL_DESCENT', 'TOUCHDOWN_CONFIRM', 'ASCENT'):
        v, sup = make_sup()
        v.hb_age = 99.0
        act, nume, _ = sup._mon_link(1.0, 0.05, faza)
        rezultate[faza] = act
    aplicat = {f for f, a in rezultate.items() if a == Action.BRAKE}
    assert aplicat == set(safety_mod.DETECTION_MONITORED_PHASES), (
        f"monitorul de link se aplica in {sorted(aplicat)}, iar cel de "
        f"detectie in {sorted(safety_mod.DETECTION_MONITORED_PHASES)} - "
        f"trebuie sa fie aceleasi")
    return f"se aplica exact in {sorted(aplicat)}"


def test_monitor_link_fara_heartbeat_deloc():
    class FaraHb(SupVehicle):
        def time_since_heartbeat(self, now=None):
            return None
    v, sup = make_sup(FaraHb())
    act, nume, why = sup._mon_link(1.0, 0.05, 'DESCEND_TRACK')
    assert act == Action.BRAKE and nume == 'link_age', (act, nume)
    assert 'de la pornire' in why
    return "niciun HEARTBEAT vreodata -> BRAKE, nu 'presupunem ca merge'"


def test_vehicul_fara_metoda_nu_e_monitorizat():
    """Compatibilitate: un vehicul care nu expune metoda nu declanseaza."""
    class Vechi(SupVehicle):
        time_since_heartbeat = None
    v = Vechi()
    del Vechi.time_since_heartbeat
    sup = SafetySupervisor(v, verbose=False)
    sup.auto_arm = False
    sup.arm(0.0, 0.0, 0.0, 0.0)
    act, _, _ = sup._mon_link(1.0, 0.05, 'DESCEND_TRACK')
    assert act == Action.NONE
    return "vehicul fara time_since_heartbeat: monitorul tace, nu arunca"


def test_NEGATIV_reincercarile_de_mod_nu_se_consuma_in_gol():
    """Cu legatura cazuta, supervizorul nu isi arde cele 5 incercari.

    Fara asta, o cadere de link de cateva secunde ar lasa supervizorul in
    mode_fail permanent: ar fi 'comandat' BRAKE de cinci ori catre un port
    inchis si ar renunta, exact cand legatura revine."""
    v, sup = make_sup()
    v.hb_age = 99.0
    v.link_healthy = False
    sup.update(1.0, 0.05, 'DESCEND_TRACK')
    assert sup.latched == Action.BRAKE

    t = 1.0
    for _ in range(40):
        t += safety_mod.MODE_CONFIRM_S + 0.01
        sup.update(t, 0.05, 'DESCEND_TRACK')
    assert sup._mode_req_n == 0, (
        f"{sup._mode_req_n} incercari consumate cu legatura cazuta")
    assert not v.moduri, f"comenzi trimise pe un port inchis: {v.moduri}"
    assert not [e for e in sup.log if e.monitor == 'mode_fail'], (
        "a declarat mode_fail fara sa fi trimis niciun octet")

    # legatura revine: comanda pleaca, cu numaratoarea intacta
    v.link_healthy = True
    t += safety_mod.MODE_CONFIRM_S + 0.01
    sup.update(t, 0.05, 'DESCEND_TRACK')
    assert v.moduri == [vehicle_mod.MODE_BRAKE], v.moduri
    assert sup._mode_req_n == 1
    return ("40 de cicluri cu link cazut: 0 incercari consumate; "
            "la revenire, BRAKE trimis din prima")


def test_prioritate_override_peste_link():
    """Pilotul are prioritate si peste o cadere de legatura (15.1.7)."""
    v, sup = make_sup()
    v.hb_age = 99.0
    v.rc = (1500, 1500, 1500, 1900, 1500, 1500, 1000, 1000)
    t = 1.0
    sup.update(t, 0.05, 'DESCEND_TRACK')
    for _ in range(5):
        t += 0.05
        sup.update(t, 0.05, 'DESCEND_TRACK')
    assert sup.latched == Action.OVERRIDE, Action.NAMES[sup.latched]
    assert sup.passive
    return "override escaladeaza peste BRAKE de link"


TESTS = [
    ('link sanatos nu reconecteaza', test_link_sanatos_nu_reconecteaza),
    ('reconectare reusita', test_reconectare_reusita),
    ('NEGATIV: port care nu raspunde niciodata',
     test_NEGATIV_port_care_nu_raspunde_niciodata),
    ('NEGATIV: portul dispare cu totul', test_NEGATIV_portul_dispare_cu_totul),
    ('emisia suspendata cat e cazuta', test_emisia_suspendata_cat_e_cazuta),
    ('scriere esuata marcheaza legatura', test_scriere_esuata_marcheaza_legatura),
    ('pump nu moare cu portul scos', test_pump_nu_moare_cu_portul_scos),
    ('monitor de link franeaza', test_monitor_link_franeaza),
    ('monitor de link: aceleasi faze ca detectia',
     test_monitor_link_aceleasi_faze_ca_detectia),
    ('monitor de link: fara heartbeat deloc',
     test_monitor_link_fara_heartbeat_deloc),
    ('vehicul fara metoda nu e monitorizat',
     test_vehicul_fara_metoda_nu_e_monitorizat),
    ('NEGATIV: reincercarile de mod nu se consuma in gol',
     test_NEGATIV_reincercarile_de_mod_nu_se_consuma_in_gol),
    ('prioritate override peste link', test_prioritate_override_peste_link),
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
