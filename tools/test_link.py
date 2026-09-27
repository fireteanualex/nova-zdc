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
            self.link.sent_args.append((name, a, kw))
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
        self.sent_args = []
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


def test_WP_ACC_lasa_marja_pentru_tranzitoriu():
    """WP_ACC guverneaza inclinarea in LAND: ModeLand::init o preia din
    wp_nav pentru controlerul NE, iar a = g*tan(unghi).

    Implicitul iris (2.5 m/s2) da 14.3 grade in regim, iar bugetul cadrului
    la cazul masurat era 14.0 - marja zero. Valoarea din fisierul de
    parametri trebuie sa lase loc tranzitoriului."""
    import math as _m
    for fisier in ('nova_sitl.parm', 'nova_flight.parm'):
        radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        text = open(os.path.join(radacina, 'config', fisier)).read()
        linii = [l for l in text.splitlines()
                 if l.strip().startswith('WP_ACC,')]
        assert linii, f"WP_ACC lipseste din {fisier}"
        val = float(linii[-1].split(',')[1])
        unghi = _m.degrees(_m.atan(val / 9.81))
        assert unghi <= 10.0, (
            f"{fisier}: WP_ACC {val} -> {unghi:.1f} deg in regim, prea "
            f"aproape de bugetul cadrului")
    return f"WP_ACC {val} m/s2 -> {unghi:.1f} deg in regim, in ambele fisiere"


def test_pragul_de_legatura_are_marja_fata_de_rata_ceruta():
    """H1 avea MARJA ZERO, si a oprit o secventa autonoma reala.

    ArduPilot trimite HEARTBEAT implicit la 1 Hz (verificat in
    GCS_Common.cpp: `set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT,
    1000)`). Cu LINK_MAX_AGE_S = 1.0, monitorul declara legatura cazuta
    exact cand soseste urmatorul heartbeat.

    Masurat in Gazebo: DESCEND_TRACK oprit dupa 1.2 m de coborare, cu
    `fara HEARTBEAT de 1.02 s (prag 1.00 s)`. Pe vehicul raportul e
    identic - nu e artefact de simulare.

    Testul leaga cele doua constante: nici pragul, nici rata nu se pot
    schimba singure fara ca asta sa pice."""
    from nova.vehicle import HEARTBEAT_HZ
    from nova.safety import LINK_MAX_AGE_S
    marja = LINK_MAX_AGE_S * HEARTBEAT_HZ
    assert marja >= 3.0, (
        f"pragul de {LINK_MAX_AGE_S} s la {HEARTBEAT_HZ} Hz inseamna doar "
        f"{marja:.1f} heartbeat-uri pierdute; sub 3 nu e marja, e noroc")
    return (f"{HEARTBEAT_HZ} Hz cerut, prag {LINK_MAX_AGE_S} s "
            f"= {marja:.0f} heartbeat-uri de marja")


def test_rata_de_heartbeat_chiar_se_cere():
    """§5.10 pe stream-uri: daca nu o ceri, primesti implicitul.

    Prima varianta cerea LOCAL_POSITION_NED, ATTITUDE, GLOBAL_POSITION_INT,
    EXTENDED_SYS_STATE si RC_CHANNELS - dar nu HEARTBEAT, tocmai mesajul de
    care atarna un monitor de siguranta."""
    from pymavlink import mavutil
    from nova.vehicle import Vehicle, HEARTBEAT_HZ
    cerute = []

    class _Mav:
        def command_long_send(self, *a):
            # command_long_send(sys, comp, cmd, confirmation, p1..p7):
            # p1 = msg_id, p2 = interval_us. HEARTBEAT are id 0, deci un
            # index gresit aici trece neobservat pana la prima citire.
            cerute.append((a[4], a[5]))

    v = object.__new__(Vehicle)
    v.m = type('M', (), {'mav': _Mav(), 'target_system': 1,
                         'target_component': 1})()
    v.telem_hz, v.landed_hz = 20, 50
    v._request_streams()
    ids = dict(cerute)
    hb = mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT
    assert hb in ids, f"HEARTBEAT nu e cerut deloc; cerute: {sorted(ids)}"
    assert abs(ids[hb] - 1e6 / HEARTBEAT_HZ) < 1.0, ids[hb]
    return f"{len(ids)} rate cerute, HEARTBEAT la {1e6 / ids[hb]:.0f} Hz"


def test_intervalul_observat_e_masurat_nu_presupus():
    """Nu putem citi inapoi un interval de mesaj, dar putem masura ce vine.
    Daca iese ~1 s, cererea nu s-a aplicat si marja nu exista."""
    from nova.vehicle import Vehicle
    v = object.__new__(Vehicle)
    v.hb_t = None
    v.hb_gaps = __import__('collections').deque(maxlen=20)
    v.link_healthy = True
    v.reconnect_attempts = 0
    v.reconnects = 0
    v._reconnect_next_t = None
    v.time_boot_ms = None
    v.link_verbose = False
    v.on_link_event = None
    assert v.heartbeat_interval() is None, "fara date trebuie 'nu stiu'"
    t = 0.0
    for _ in range(6):
        v._note_heartbeat(t)
        t += 0.2
    assert abs(v.heartbeat_interval() - 0.2) < 1e-9, v.heartbeat_interval()
    return "median 0.2 s din 5 intervale; fara date -> None"


def test_ExtNav_istoricul_de_atitudine_si_pozitie_la_momentul_capturii():
    """Brief ExtNav §4: raza spre marker se roteste in NED cu atitudinea de
    la CAPTURA, nu cu ultima primita - la 20 Hz si 50-100 ms latenta,
    diferenta e de cateva grade, adica zeci de cm la 10 m. Interpolare
    liniara, yaw pe arcul scurt, None in afara istoricului."""
    import math
    h = Harness(FakeLink())
    v = make_vehicle(h)
    assert v.attitude_at(1.0) is None and v.position_at(1.0) is None
    v._on_attitude(1.00, 0.00, 0.10, math.radians(350))
    v._on_attitude(1.10, 0.20, 0.30, math.radians(10))
    r, p, y = v.attitude_at(1.05)
    assert abs(r - 0.10) < 1e-9 and abs(p - 0.20) < 1e-9, (r, p)
    assert abs(math.degrees(y)) < 1e-6, f"yaw prin arcul scurt: {math.degrees(y)}"
    assert v.attitude_at(1.10 + 0.20) == (0.20, 0.30, v.att_hist[-1][3])
    assert v.attitude_at(1.10 + 0.30) is None, "in afara istoricului: None"
    assert v.attitude_at(0.80) == (0.00, 0.10, v.att_hist[0][3])
    assert v.attitude_at(0.70) is None
    v._on_position(2.0, 0.0, 0.0, -10.0, 0.5, 0.0, 0.0)
    v._on_position(3.0, 1.0, -2.0, -9.0, 0.5, 0.0, 0.0)
    assert v.position_at(2.5) == (0.5, -1.0, -9.5)
    assert v.have_pos and v.vx == 0.5 and v.alt == 9.0
    return "atitudine si pozitie interpolate la t; yaw 350->10 da 0; None in afara"


def test_ExtNav_mesajele_noi_pleaca_doar_cu_legatura_vie():
    """VISION_POSITION_ESTIMATE si SET_POSITION_TARGET_LOCAL_NED: aceeasi
    regula ca restul comenzilor (H1), plus masca de tip verificata - o
    masca gresita ar face FC-ul sa ignore pozitia sau sa urmareasca viteze
    zero, fara nicio eroare."""
    from pymavlink import mavutil
    from nova.vehicle import POS_TARGET_MASK_POS_YAW
    h = Harness(FakeLink())
    v = make_vehicle(h)
    link = h.stari[0]
    assert v.send_vision_position_estimate(123456, 1.0, -2.0, -5.0,
                                           0.1, 0.2, 0.3) is True
    assert v.n_vpe == 1 and link.sent[-1] == 'vision_position_estimate_send'
    nume, a, _ = link.sent_args[-1]
    assert a == (123456, 1.0, -2.0, -5.0, 0.1, 0.2, 0.3), a
    assert v.send_position_target(0.0, 0.0, -3.0, 1.5) is True
    assert v.n_pos_target == 1
    nume, a, _ = link.sent_args[-1]
    assert nume == 'set_position_target_local_ned_send'
    assert a[3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    masca = a[4]
    m = mavutil.mavlink
    for ignorat in (m.POSITION_TARGET_TYPEMASK_VX_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_VY_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_AX_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_AY_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE):
        assert masca & ignorat, f"bitul {ignorat} nu e in masca"
    for folosit in (m.POSITION_TARGET_TYPEMASK_X_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_Y_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_Z_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
                    m.POSITION_TARGET_TYPEMASK_FORCE_SET):
        assert not (masca & folosit), f"bitul {folosit} e aprins gresit"
    assert masca == POS_TARGET_MASK_POS_YAW
    assert (a[5], a[6], a[7], a[14]) == (0.0, 0.0, -3.0, 1.5), a
    n = len(link.sent)
    v.link_healthy = False
    assert v.send_vision_position_estimate(1, 0, 0, 0, 0, 0, 0) is False
    assert v.send_position_target(0, 0, 0, 0) is False
    assert len(link.sent) == n and v.n_vpe == 1 and v.n_pos_target == 1
    return "VPE si consemnul de pozitie: masca pozitie+yaw, suprimate cu legatura cazuta"


def test_ExtNav_EKF_STATUS_REPORT_se_cere_si_se_citeste():
    """Validitatea pozitiei EKF e un monitor nou (brief §6). Fara
    stream-ul cerut, raportul vine rar sau deloc; fara raport, raspunsul e
    None - necunoscut nu e "ok"."""
    from pymavlink import mavutil
    from nova.vehicle import Vehicle, EKF_HZ
    cerute = []

    class _Mav:
        def command_long_send(self, *a):
            cerute.append((a[4], a[5]))

    v = object.__new__(Vehicle)
    v.m = type('M', (), {'mav': _Mav(), 'target_system': 1,
                         'target_component': 1})()
    v.telem_hz, v.landed_hz = 20, 50
    v._request_streams()
    ids = dict(cerute)
    mid = mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT
    assert mid in ids and abs(ids[mid] - 1e6 / EKF_HZ) < 1.0, ids.get(mid)
    h = Harness(FakeLink())
    v = make_vehicle(h)
    assert v.ekf_pos_horiz_ok() is None
    v.ekf_flags = mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_POS_HORIZ_REL
    assert v.ekf_pos_horiz_ok() is True
    v.ekf_flags = mavutil.mavlink.EKF_ATTITUDE
    assert v.ekf_pos_horiz_ok() is False
    return f"EKF_STATUS_REPORT cerut la {EKF_HZ} Hz; None / True / False din flags"


# --- teste: firul I/O (faza 2, refactor/threads) ------------------------------

def make_threaded(link=None):
    """Vehicle cu fir I/O configurat dar NEPORNIT: testele pasesc firul de
    mana cu _io_step(), ca ordinea sa fie deterministă."""
    link = link or FakeLink()
    h = Harness(link)
    vechi = vehicle_mod.mavutil.mavlink_connection
    vehicle_mod.mavutil.mavlink_connection = h
    try:
        v = Vehicle('/dev/fake0', baud=921600, threaded=True)
        v.m = h('/dev/fake0', baud=921600)
        v.link_verbose = False
        v._note_heartbeat()
    finally:
        vehicle_mod.mavutil.mavlink_connection = vechi
    return v, link


def test_IO_urgent_pleaca_inaintea_lui_TX():
    """Coada URGENT (supervizor) se goleste INAINTEA cozii TX si a mesajelor
    periodice, oricare ar fi ordinea in care au fost puse."""
    from nova.vehicle import MODE_BRAKE, MODE_LAND
    v, link = make_threaded()
    assert v.send_landing_target(0.1, 0.2, 5.0) is True         # periodic
    assert v.request_mode(MODE_LAND) is True                    # TX
    assert v.send_vision_position_estimate(1, 0, 0, -5, 0, 0, 0) is True
    assert v.request_mode(MODE_BRAKE, urgent=True) is True      # URGENT
    assert link.sent == [], "a trimis inainte ca firul I/O sa paseasca"
    v._io_step(100.0)
    nume = [n for n, _a, _k in link.sent_args]
    moduri = [a[5] for n, a, _k in link.sent_args if n == 'command_long_send']
    assert nume[0] == 'command_long_send' and moduri[0] == MODE_BRAKE, nume
    assert moduri == [MODE_BRAKE, MODE_LAND], moduri
    assert nume.index('landing_target_send') > nume.index('command_long_send')
    assert nume[-2:] in (['landing_target_send', 'vision_position_estimate_send'],
                         ['vision_position_estimate_send', 'landing_target_send'])
    assert v.n_lt == 0, "contorul creste in fatada abia la pump()"
    v.pump()
    assert v.n_lt == 1 and v.n_vpe == 1
    return "URGENT -> TX -> periodice; contoarele prin instantaneu"


def test_IO_coada_plina_pentru_comenzi_inlocuire_pentru_periodice():
    """Comenzile nu se pierd tacut: coada plina = False + eroare logata.
    Mesajele periodice au un singur loc pe tip: cel mai nou castiga."""
    import logging
    from nova.vehicle import QUEUE_TX

    class _H(logging.Handler):
        def __init__(self):
            super().__init__()
            self.n = 0

        def emit(self, r):
            self.n += 1

    h = _H()
    vehicle_mod.log.addHandler(h)
    try:
        v, link = make_threaded()
        for i in range(QUEUE_TX):
            assert v.send_takeoff(1.0 + i) is True
        assert v.send_takeoff(99.0) is False, "coada plina acceptata tacut"
        assert v.n_queue_full == 1 and h.n == 1
        # periodice: trei VPE, pleaca doar ultimul
        for k in range(3):
            assert v.send_vision_position_estimate(k, k, 0, -5, 0, 0, 0) is True
        v._io_step(100.0)
        vpe = [a for n, a, _k in link.sent_args if n == 'vision_position_estimate_send']
        assert len(vpe) == 1 and vpe[0][0] == 2, vpe
        dec = [a for n, a, _k in link.sent_args if n == 'command_long_send']
        assert len(dec) == QUEUE_TX
        v.pump()
        assert v.n_vpe == 1
    finally:
        vehicle_mod.log.removeHandler(h)
    return f"comanda {QUEUE_TX + 1} refuzata si logata; 3 VPE -> 1 trimis (ultimul)"


def test_IO_timeout_pe_confirmarea_de_mod():
    """Supervizorul cere BRAKE prin coada; FC-ul (fals) ramane in LAND.
    Reincercarile si mode_fail au aceleasi cifre ca in modul sincron:
    MODE_RETRY_MAX comenzi la MODE_CONFIRM_S, apoi nimic."""
    from nova.safety import MODE_RETRY_MAX, MODE_CONFIRM_S, Action
    v, link = make_threaded()
    v._on_position(100.0, 0.0, 0.0, -6.0, 0.0, 0.0, 0.0)   # firul I/O: pozitie
    v._io_step(100.0)                              # HEARTBEAT fals: LAND (9)
    v.pump()
    assert v.mode == 9 and v.have_pos, (v.mode, v.have_pos)
    sup = SafetySupervisor(v, verbose=False)
    sup.auto_arm = False
    sup.arm(100.0, 0.0, 0.0, 0.0)
    t = 100.0
    for i in range(30):
        sup.update(t, 0.9, 'DESCEND_TRACK')        # detectie veche -> BRAKE
        v._io_step(t)
        v.pump()
        t += MODE_CONFIRM_S + 0.01
    braki = [a for n, a, _k in link.sent_args
             if n == 'command_long_send' and a[5] == 17]
    assert len(braki) == MODE_RETRY_MAX, len(braki)
    assert any(e.monitor == 'mode_fail' for e in sup.log)
    assert sup.latched == Action.BRAKE and v.mode == 9
    return f"{MODE_RETRY_MAX} BRAKE prin coada, apoi mode_fail; FC ramas in LAND"


def test_IO_instantaneul_prin_pump():
    """Firul I/O scrie in starea lui; fatada se schimba DOAR la pump(),
    deci o iteratie a buclei vede o singura telemetrie."""
    class HB:
        base_mode = 128
        custom_mode = 5

        def get_type(self):
            return 'HEARTBEAT'

        def get_srcComponent(self):
            return 1

    class L(FakeLink):
        def __init__(self):
            super().__init__()
            self.msgs = [HB()]

        def recv_match(self, type=None, blocking=False, timeout=None):  # noqa: A002
            return self.msgs.pop(0) if self.msgs else None

    v, link = make_threaded(L())
    assert v.mode is None and not v.armed
    v._io_step(100.0)
    assert v.mode is None, "fatada s-a schimbat fara pump()"
    snap, t = v.state.get()
    assert snap.mode == 5 and snap.armed and t == 100.0
    v.pump()
    assert v.mode == 5 and v.armed
    # istoricul de atitudine e in buffer-ul sigur, citibil imediat
    v._on_attitude(100.0, 0.1, 0.2, 0.3)
    assert v.attitude_at(100.0) == (0.1, 0.2, 0.3)
    assert v.att_hist[-1][3] == 0.3
    return "starea firului -> Latest -> fatada la pump(); atitudinea in buffer"


def test_IO_niciun_apel_pe_mav_in_afara_firului():
    """Regula 1: un singur fir atinge serialul. Verificat pe SURSA (cine
    mai scrie `.m.mav` / `recv_match`) si la RUNTIME (o scriere din alt fir
    decat cel I/O e refuzata cu eroare)."""
    import ast
    import threading
    radacina = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # 1. in vehicle.py, `self.m.mav` doar in cele trei locuri ale firului I/O
    src = open(os.path.join(radacina, 'nova', 'vehicle.py')).read()
    tree = ast.parse(src)
    unde = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            text = ast.get_source_segment(src, node) or ''
            if 'self.m.mav' in text or 'recv_match(' in text:
                unde.add(node.name)
    assert unde == {'_request_streams', '_probe_distance_api', '_write',
                    '_recv_batch', '_try_reopen'}, sorted(unde)
    # 2. in restul codului de zbor, nimeni nu atinge mav / recv_match.
    #    fence.py: necablat in zbor, foloseste protocolul de misiune direct
    #    - de rutat prin Vehicle INAINTE de a fi cablat (raport, in afara
    #    perimetrului). nova_sim.py e simulatorul, sincron, un singur fir.
    exceptii = {'vehicle.py', 'fence.py'}
    rele = []
    for d, fisiere in (('nova', os.listdir(os.path.join(radacina, 'nova'))),
                       ('tools', ['nova_pi.py'])):
        for f in fisiere:
            if not f.endswith('.py') or f in exceptii:
                continue
            text = open(os.path.join(radacina, d, f)).read()
            for tipar in ('.m.mav.', 'recv_match(', '.m.close('):
                if tipar in text:
                    rele.append(f"{d}/{f}: {tipar}")
    assert not rele, rele
    # 3. runtime: cu firul I/O pornit, o scriere din alt fir e refuzata
    v, link = make_threaded()
    v.start_io()
    try:
        time.sleep(0.05)
        assert v.io_heartbeat.count > 0, "firul I/O nu bate"
        try:
            v._write('command_long_send', (1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0), {})
            assert False, "scriere din firul principal acceptata"
        except RuntimeError as e:
            assert 'I/O' in str(e)
        # iar prin coada ajunge: de aici, nu din firul principal
        n = len(link.sent)
        assert v.request_mode(9) is True
        t0 = time.monotonic()
        while len(link.sent) == n and time.monotonic() - t0 < 1.0:
            time.sleep(0.005)
        assert len(link.sent) > n, "comanda din coada nu a plecat"
    finally:
        v.close()
    assert not v._io_thread and not threading.current_thread() is None
    return "sursa: mav doar in 5 metode ale firului; runtime: scriere din alt fir refuzata"


TESTS = [
    ('I/O: URGENT inaintea lui TX', test_IO_urgent_pleaca_inaintea_lui_TX),
    ('I/O: coada plina pentru comenzi, inlocuire pentru periodice',
     test_IO_coada_plina_pentru_comenzi_inlocuire_pentru_periodice),
    ('I/O: timeout pe confirmarea de mod', test_IO_timeout_pe_confirmarea_de_mod),
    ('I/O: instantaneul prin pump()', test_IO_instantaneul_prin_pump),
    ('I/O: niciun apel pe mav in afara firului',
     test_IO_niciun_apel_pe_mav_in_afara_firului),
    ('ExtNav: istoricul de atitudine si pozitie la captura',
     test_ExtNav_istoricul_de_atitudine_si_pozitie_la_momentul_capturii),
    ('ExtNav: VPE si consemnul de pozitie pleaca doar cu legatura vie',
     test_ExtNav_mesajele_noi_pleaca_doar_cu_legatura_vie),
    ('ExtNav: EKF_STATUS_REPORT cerut si citit',
     test_ExtNav_EKF_STATUS_REPORT_se_cere_si_se_citeste),
    ('WP_ACC lasa marja pentru tranzitoriu',
     test_WP_ACC_lasa_marja_pentru_tranzitoriu),
    ('pragul de legatura are marja fata de rata ceruta',
     test_pragul_de_legatura_are_marja_fata_de_rata_ceruta),
    ('rata de heartbeat chiar se cere',
     test_rata_de_heartbeat_chiar_se_cere),
    ('intervalul observat e masurat, nu presupus',
     test_intervalul_observat_e_masurat_nu_presupus),
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
