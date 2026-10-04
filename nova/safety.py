#!/usr/bin/env python3
"""
Safety Supervisor determinist (15.2.9, 15.2.10).

Ruleaza in paralel cu masina de stari si are autoritate peste ea: cand
comanda o schimbare de mod, masina de stari vede ca nu mai e in LAND si isi
incheie singura secventa (nova/state_machine.py: _supervise).

De ce exista: comportamentul cerut de 15.2.9 - sub pragul de incredere al
detectiei aeronava trebuie sa **planeze**, nu sa aterizeze - **nu e garantat
de ArduPilot**. Testat: cu PLND_STRICT 2 si pierdere totala a detectiei,
vehiculul a coborat de la 6.6 m si a aterizat cu 33.4 cm eroare, dupa
`PrecLand: Failsafe Measures` si `Disarming motors`. Deci trebuie implementat
aici.

Determinism (15.2.10):
  - niciun prag nu depinde de date de viziune; monitorul de detectie primeste
    VARSTA ultimei detectii valide, niciodata continutul ei
  - monitoarele se evalueaza in aceeasi ordine la fiecare ciclu, iar actiunea
    retinuta e cea mai severa
  - nimic aleator, nimic dependent de istoric in afara temporizarilor explicite
  - o actiune odata declansata se **zavoraste**: nu se revine automat, nici
    daca semnalul redevine bun

Toate pragurile sunt constante numite, la inceputul fisierului, ca sa se
transcrie direct in Safety Case.
"""

import threading
import time

from .rc import OverrideMonitor
from .vehicle import MODE_BRAKE, MODE_GUIDED, MODE_LAND, MODE_LOITER, MODE_RTL

#: Faza 3 (refactor/threads): heartbeat-ul unui fir (detectie, I/O,
#: principal) stagnat peste atat = firul e mort sau blocat. Aceeasi
#: actiune ca la pierderea sursei respective. Din config, cu implicitul de
#: aici.
THREAD_HEARTBEAT_MAX_S = 1.0

# --- PRAGURI DE SIGURANTA -------------------------------------------------
# Se transcriu ca atare in Safety Case. Fiecare are nevoie de o justificare
# scrisa acolo; cele marcate PROVIZORIU inca nu au masuratoare in spate.

#: 15.2.9 - varsta maxima a ultimei detectii valide inainte de a pluti.
#: 0.5 s = 10 cadre la 20 Hz. PROVIZORIU: de recalibrat pe detectorul ArUco
#: real, unde rata de pierdere a markerului e alta decat in sintetic.
DETECTION_MAX_AGE_S = 0.5

#: DECIZIA ECHIPEI, 25.09.2026: BRAKE abia dupa atatea cadre CONSECUTIVE
#: procesate fara marker, nu la prima pauza de 0.5 s. Cand detectorul
#: raporteaza contorul, regula de mai sus (pe timp) e inlocuita de asta.
#: La ~10 cadre/s (masurat in zbor, §5.63) 5 cadre ~ 0.5 s; la rate mai
#: mici, coborarea oarba se lungeste - de-asta plafonul de mai jos.
DETECTION_MAX_MISSES = 5

#: Plasa pentru un detector BLOCAT: daca nu proceseaza cadre, contorul de
#: ratari nu creste niciodata - si fara plafon supervizorul ar lasa
#: coborarea oarba la infinit. Peste atat, BRAKE oricum. 1.5 s la
#: 0.5 m/s = 0.75 m de coborare oarba, plus frana (1.00 m masurat).
DETECTION_HARD_MAX_AGE_S = 1.5

#: 15.2.4 - raza fata de punctul de handover si plafonul deasupra lui.
#: On ExtNav (B1, 02.10.2026) the ceiling is measured above HOME
#: (GLOBAL_POSITION_INT.relative_alt), not above the handover point - see
#: ExtNavSupervisor._mon_ceiling. The PLND path keeps the old meaning.
GEOFENCE_RADIUS_M = 10.0
CEILING_AGL_M = 30.0
#: ExtNav ceiling: consecutive DISTINCT GLOBAL_POSITION_INT samples above
#: the ceiling before the breach is confirmed (3 at 20 Hz = 0.15 s). One
#: noisy baro sample does not abort an attempt; three in a row do.
CEILING_CONFIRM_N = 3

#: Rata de coborare peste care ceva e in neregula. Coborarea normala e
#: limitata de LAND_SPD_MS (0.5 m/s) si WP_SPD_DN (ArduCopter 4.7.1 names);
#: 2.0 m/s inseamna ca nu mai
#: comanda controlerul de aterizare. PROVIZORIU.
MAX_DESCENT_RATE_MS = 2.0
DESCENT_RATE_HOLD_S = 0.5

#: Inclinare peste care vehiculul nu mai e intr-o coborare controlata.
#:
#: **E plafonul de INTEGRITATE al vehiculului, nu limita camerei.** Cele doua
#: au impartit acelasi numar pana acum, si nu sunt acelasi lucru:
#:
#:   integritate  30 grade, constanta. Peste, coborarea nu mai e controlata,
#:                indiferent ce vede camera.
#:   camera       `CameraModel.tilt_budget_deg(alt, lateral)`, functie de
#:                altitudine SI de eroarea laterala. Masurat: 14.0 grade la
#:                7.17 m cu 2.95 m lateral (§5.48), 19.5 grade la 1 m cu
#:                10 cm - adica sub 30 in tot regimul de sub ~2 m.
#:
#: Deci monitorul asta NU poate proteja detectia: pana ajunge la 30 de grade,
#: markerul a iesit demult din cadru si a declansat monitorul de varsta a
#: detectiei. Nu e o scapare de reglat coborand pragul la 20 - un BRAKE pe
#: bugetul camerei ar transforma un tranzitoriu recuperabil in incercare
#: anulata, si ar lovi tocmai cand controlerul face ce trebuie: se inclina ca
#: sa corecteze lateral.
#:
#: Bugetul camerei se apara in alta parte, preventiv:
#:   - `WP_ACC = 1.5` plafoneaza inclinarea de regim la 8.7 grade (§5.48)
#:   - `SequenceConfig.final_fill` scoate coborarea din fazele supravegheate
#:     inainte ca bugetul sa se prabuseasca sub 1 m
#:   - poarta ar trebui sa refuze geometria nerecuperabila (elementul 28/J4)
#:
#: Masurat si raportat ca marja in campanie, ca relatia sa intre in Safety
#: Case - nu o singura cifra (§6/15.2.9). Elementul deschis 30.
MAX_TILT_DEG = 30.0
TILT_HOLD_S = 0.3

#: H1 - varsta maxima a legaturii cu FC-ul inainte de a pluti.
#:
#: De ce are acelasi tratament ca pierderea detectiei: sunt moduri de esec
#: diferite cu ACEEASI consecinta. Daca LANDING_TARGET nu ajunge la FC,
#: vehiculul continua sa coboare in LAND fara corectie laterala - exact ce
#: se intampla cand markerul nu mai e vazut. Un supervizor care monitorizeaza
#: doar detectia acopera jumatate din cazuri.
#:
#: 1.0 s, adica dublul lui DETECTION_MAX_AGE_S si o treime din
#: HEARTBEAT_TIMEOUT_S al lui Vehicle. Mai lung decat pragul de detectie
#: pentru ca o cadere de link e mai rara si mai grava decat un cadru pierdut,
#: si nu vrem sa declansam pe o intarziere de planificare a Pi-ului.
#: Mai scurt decat pragul de reconectare pentru ca supervizorul trebuie sa
#: reactioneze INAINTE ca Vehicle sa inceapa sa reincerce - franarea nu are
#: voie sa astepte dupa un cablu. PROVIZORIU: de recalibrat pe teren.
LINK_MAX_AGE_S = 1.0

#: Cat asteptam confirmarea modului comandat, inainte de a reincerca.
MODE_CONFIRM_S = 0.3
MODE_RETRY_MAX = 5


class Action:
    """Ordonate dupa severitate; supervizorul retine maximul pe ciclu.

    OVERRIDE e deasupra lui RTL deliberat: 15.1.7 spune ca pilotul poate
    prelua oricand, iar un RTL comandat peste un pilot care zboara activ ar
    lupta cu el. Cand pilotul a pus mana pe manse, autoritatea e a lui."""
    NONE = 0
    BRAKE = 1        # planeaza pe loc (15.2.9)
    LOITER = 2
    RTL = 3          # abort (15.2.4)
    OVERRIDE = 4     # pilotul a preluat (15.3.1, 15.1.7)
    #: ExtNav (27.09.2026): the single action of ExtNavSupervisor. It is
    #: not a mode - the state machine executes the ordered exit (SRC1
    #: restored and read back, then LOITER, then ALT_HOLD; never RTL on
    #: SRC2). The supervisor only asks for it.
    EXIT = 5
    #: B1 (02.10.2026, rule 15.2.4): the ExtNav ceiling's action. Not a
    #: mode either: the state machine runs the EXIT_RTL path (SRC1 sent and
    #: acknowledged, EKF back in the GNSS frame, then RTL; LAND if set 1 is
    #: never acknowledged or RTL is refused). The supervisor only asks.
    EXIT_RTL = 6

    NAMES = {NONE: 'NONE', BRAKE: 'BRAKE', LOITER: 'LOITER', RTL: 'RTL',
             OVERRIDE: 'OVERRIDE', EXIT: 'EXIT', EXIT_RTL: 'EXIT_RTL'}
    MODES = {BRAKE: MODE_BRAKE, LOITER: MODE_LOITER, RTL: MODE_RTL,
             OVERRIDE: MODE_LOITER}


#: Fazele in care monitorul de detectie are sens.
#:
#: EXCEPTIE OBLIGATORIE - FINAL_DESCENT si tot ce urmeaza dupa el:
#: sub 0.38 m markerul nu mai incape in cadru (§5.2 din CLAUDE.md), deci
#: detectorul inceteaza sa mai publice **prin constructie**, nu din cauza unei
#: defectiuni. Un supervizor care ar cere detectie valida si acolo ar aborta
#: in ultimul metru la FIECARE incercare, transformand o proprietate fizica a
#: camerei intr-o defectiune inventata. Coborarea finala e deliberat oarba si
#: verticala (§8): autoritatea de corectie la acea inaltime e oricum 2-4 cm.
#:
#: Formulat pozitiv (fazele in care monitorul SE aplica), nu negativ, ca sa
#: nu devina o gaura de acoperire la adaugarea unei stari noi: o faza
#: necunoscuta nu e supravegheata de monitorul asta si trebuie adaugata
#: explicit aici.
DETECTION_MONITORED_PHASES = ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE')

#: Fazele in care segmentul autonom e activ (pentru monitoarele geometrice).
AUTONOMOUS_PHASES = DETECTION_MONITORED_PHASES + (
    'FINAL_DESCENT', 'TOUCHDOWN_CONFIRM', 'ASCENT')


class SafetyEvent:
    """O intrare de log. Poarta si ceasul FC-ului, ca sa se poata alinia cu
    telemetria din .bin/.tlog (6.2.1.30)."""

    __slots__ = ('t', 'time_boot_ms', 'monitor', 'action', 'detail', 'phase')

    def __init__(self, t, time_boot_ms, monitor, action, detail, phase):
        self.t = t
        self.time_boot_ms = time_boot_ms
        self.monitor = monitor
        self.action = action
        self.detail = detail
        self.phase = phase

    def __str__(self):
        tb = '-' if self.time_boot_ms is None else f"{self.time_boot_ms}"
        return (f"[SAFETY t={self.t:.3f} boot_ms={tb} faza={self.phase}] "
                f"{self.monitor}: {Action.NAMES[self.action]} - {self.detail}")


class SafetySupervisor:
    """
        sup = SafetySupervisor(vehicle)
        sup.arm(now, origin_n, origin_e)      # la handover
        ...
        sup.update(now, detection_age_s, phase)

    Semnatura difera de schita din sarcina (`update(now, telemetry,
    last_detection)`): telemetria se citeste din `vehicle`, iar detectia intra
    **numai** ca varsta in secunde. Asa, niciun pixel nu ajunge in logica de
    decizie, ceea ce e chiar cerinta de determinism din 15.2.10.
    """

    def __init__(self, vehicle, on_event=None, verbose=True, override=None,
                 detection_max_age_s=DETECTION_MAX_AGE_S,
                 geofence_radius_m=GEOFENCE_RADIUS_M,
                 ceiling_agl_m=CEILING_AGL_M,
                 max_descent_rate_ms=MAX_DESCENT_RATE_MS,
                 max_tilt_deg=MAX_TILT_DEG,
                 link_max_age_s=LINK_MAX_AGE_S):
        self.v = vehicle
        self.on_event = on_event
        self.verbose = verbose

        self.detection_max_age_s = detection_max_age_s
        self.detection_max_misses = DETECTION_MAX_MISSES
        self.detection_hard_max_age_s = DETECTION_HARD_MAX_AGE_S
        #: ratari consecutive raportate de detector; None = detectorul nu le
        #: numara, si atunci ramane regula veche pe timp
        self.miss_streak = None
        self.geofence_radius_m = geofence_radius_m
        self.ceiling_agl_m = ceiling_agl_m
        self.max_descent_rate_ms = max_descent_rate_ms
        self.max_tilt_deg = max_tilt_deg
        self.link_max_age_s = link_max_age_s

        #: Phase sets the monitors consult. Instance attributes so a
        #: variant (ExtNavSupervisor) can run the SAME monitors over other
        #: phase names; the module constants keep the PLND behaviour.
        self.autonomous_phases = AUTONOMOUS_PHASES
        self.link_phases = DETECTION_MONITORED_PHASES
        #: Radius / ceiling: phases in which the position frame is the one
        #: the origin was captured in. PLND: all autonomous phases.
        self.geometry_phases = AUTONOMOUS_PHASES

        self.armed = False
        self.auto_arm = True      # se armeaza singur din faza primita
        self.origin_n = self.origin_e = 0.0
        self.origin_alt = 0.0

        # 15.3.1: detectia de override pe manse. Monitor separat, ca sa poata
        # fi testat independent - si ca sa fie ACELASI obiect pe care il
        # foloseste poarta de handover cand memoreaza referinta de neutru.
        self.override = override if override is not None else OverrideMonitor(vehicle)
        #: Faza 5 (2b): supervizorul e SINGURUL proprietar al monitorului de
        #: override - fereastra de asezare o conduce el, din faza portii
        #: (override.observe), neutrul il memoreaza el (arm). Poarta citeste.
        self.gate_phases = ('HANDOVER_CHECK',)
        #: True dupa un override: companion-ul nu mai comanda NIMIC pentru
        #: restul incercarii. Zavor ireversibil, si daca mansa revine la
        #: neutru.
        self.passive = False

        self.latched = Action.NONE
        self.latched_monitor = None
        self.log = []
        self._last_phase = 'IDLE'
        #: Faza 3: aprins cand supervizorul a decis o actiune. Masina de
        #: stari il verifica la fiecare pas si, aprins, nu mai trimite
        #: nimic - comenzile de mod sunt ale supervizorului, prin URGENT.
        #: Se stinge la o incercare noua (latch_release) si la re-armare.
        self.abort = threading.Event()
        self.abort_reason = None
        self.abort_passive = False
        #: B1: WHICH exit the abort asks for. True = EXIT_RTL (the ExtNav
        #: ceiling), False = the normal EXIT. Written with abort_reason,
        #: before abort_n / the Event, and cleared with it - the state
        #: machine reads it after seeing a new abort_n, like the reason.
        self.abort_rtl = False
        #: Faza 5: de cate ori s-a aprins abort-ul. Masina de stari retine
        #: ultimul numar tratat: un abort vechi, inca neconsumat de
        #: supervizor (firul lui nu a vazut inca incercarea noua), nu
        #: omoara incercarea noua; unul nou, aprins intre doi pasi ai ei,
        #: chiar dupa o stingere, nu se pierde.
        self.abort_n = 0
        #: Heartbeat-urile firelor supravegheate: nume -> Heartbeat, cu
        #: pragul fiecaruia. Se dau prin set_thread_heartbeats().
        self.heartbeats = {}
        self.heartbeat_max_s = THREAD_HEARTBEAT_MAX_S

        self._descent_since = None
        self._tilt_since = None
        self._mode_req_t = 0.0
        self._mode_req_n = 0
        self._want_mode = None
        self._mode_confirmed = None
        self._mode_before = None

    # -- ciclu de viata ----------------------------------------------------
    def arm(self, now, origin_n, origin_e, origin_alt=0.0,
            rc_neutral_captured=True):
        """La handover: memoreaza punctul fata de care se masoara raza si
        plafonul (15.2.4) si porneste supravegherea."""
        self.armed = True
        self.origin_n = origin_n
        self.origin_e = origin_e
        self.origin_alt = origin_alt
        self.latched = Action.NONE
        self.latched_monitor = None
        self._clear_abort()
        self._descent_since = None
        self._tilt_since = None
        self._want_mode = None
        self._mode_confirmed = None
        self.passive = False
        if (rc_neutral_captured and self.override.neutral is None
                and not getattr(self.override, 'in_gate', False)):
            # Cine nu trece prin faza portii (teste, intrare directa in
            # LAND) primeste referinta de neutru acum. In cursa, referinta o
            # memoreaza observe(), dupa fereastra de asezare (15.3.1 B3.1) -
            # nu aici, la intrarea in GATE_SEARCH, cu mansele inca in miscare.
            self.override.capture_neutral(now)
        self._emit(now, 'arm', Action.NONE,
                   f"origine N={origin_n:.2f} E={origin_e:.2f} "
                   f"alt={origin_alt:.2f}", 'ARM')

    def disarm(self, now, reason=''):
        """Opreste monitoarele. NU anuleaza o actiune deja zavorata: aceea se
        duce la capat prin update() pana cand FC-ul confirma modul."""
        self.armed = False
        self._emit(now, 'disarm', Action.NONE, reason, 'DISARM')

    # -- log ---------------------------------------------------------------
    def _emit(self, now, monitor, action, detail, phase):
        ev = SafetyEvent(now, self.v.time_boot_ms, monitor, action, detail,
                         phase)
        self.log.append(ev)
        if self.verbose:
            print(str(ev))
        if self.on_event:
            self.on_event(ev)
        return ev

    def log_lines(self):
        return [str(e) for e in self.log]

    # -- bucla -------------------------------------------------------------
    def update(self, now=None, detection_age_s=None, phase='IDLE',
               miss_streak=None):
        """De apelat la viteza buclei, inaintea masinii de stari.

        detection_age_s: secunde de la ultima detectie valida, sau None daca
        nu a existat niciuna. NU primeste detectia in sine (15.2.10).
        miss_streak: cadre consecutive procesate fara marker, daca detectorul
        le numara; None pastreaza regula veche, pe timp.
        """
        now = now if now is not None else time.monotonic()
        self.miss_streak = miss_streak
        self.override.observe(now, phase in self.gate_phases)

        # Zavorul se verifica INAINTEA armarii, deliberat. Cand supervizorul
        # comanda BRAKE, masina de stari vede ca nu mai e in LAND si isi
        # incheie secventa, iar aplicatia dezarmeaza supervizorul. Daca
        # dezarmarea ar opri si reincercarile de mod, o comanda pierduta ar
        # ramane definitiv nelivrata - exact actiunea de siguranta ar esua
        # tacut. Odata declansata, ducem comanda la capat indiferent de rest.
        # Un zavor CONFIRMAT nu supravietuieste unei incercari NOI. Zborul
        # b14 (§5.64): dupa BRAKE si preluarea pilotului, poarta a acceptat
        # inca doua handovere, masina de stari a comandat LAND - iar zavorul
        # de OVERRIDE, inca activ, a re-comandat LOITER in aceeasi secunda.
        # Trei acceptari, zero coborari. O cerere noua trecuta prin poarta
        # (front crescator + validare) e intentia explicita a pilotului, deci
        # zavorul vechi si-a facut treaba. Unul NECONFIRMAT ramane: actiunea
        # de siguranta nu se pierde pentru ca s-a apasat un comutator.
        new_attempt = (phase in AUTONOMOUS_PHASES
                       and self._last_phase not in AUTONOMOUS_PHASES)
        self._last_phase = phase
        # B2 (26.09.2026 seara): "confirmed" was too narrow. A latch that
        # went PASSIVE before confirmation (third mode in the retry window)
        # or that gave up (mode_fail) sends nothing either - keeping it
        # only blinds the next attempt: the latched branch returns before
        # the monitors, so the new descent would run with NO supervision.
        # Reproduced offline: BRAKE lost on serial + pilot STABILIZE, new
        # attempt accepted, marker lost 3 s -> zero commands.
        if (self.latched != Action.NONE and new_attempt
                and self._latch_final()):
            self._emit(now, 'latch_release', self.latched,
                       f"incercare noua acceptata de poarta ({phase}): "
                       f"zavorul {Action.NAMES[self.latched]} "
                       f"({self._latch_state()}) se elibereaza", phase)
            self.latched = Action.NONE
            self.latched_monitor = None
            self._want_mode = None
            self._mode_confirmed = None
            self.passive = False
            self._clear_abort()
            self.armed = False          # re-armare curata din faza, mai jos

        if self.latched != Action.NONE:
            # Singura escaladare permisa peste un zavor: pilotul. 15.1.7 ii da
            # autoritate oricand, inclusiv peste un RTL pe care tocmai l-am
            # comandat noi - altfel ne-am bate cu el pe comenzi. Un geofence
            # breach se declanseaza instantaneu, override-ul are nevoie de
            # OVERRIDE_HOLD_S, deci fara asta RTL ar castiga mereu cursa.
            # Not once PASSIVE (B1, found 26.09.2026 after the crash of
            # §5.66): passive means the pilot's switch or an FC failsafe
            # already owns the mode. Escalating on a stick movement there
            # would send LOITER over the pilot's STABILIZE, or over the
            # battery failsafe's LAND - the incident on another path.
            # Passive stays passive until a new attempt through the gate.
            if (self.latched != Action.OVERRIDE and not self.passive
                    and self.override.update(now)):
                self._trigger(now, Action.OVERRIDE, 'pilot_override',
                              f"pilotul a preluat peste "
                              f"{Action.NAMES[self.latched]}", phase)
                return self.latched
            self._drive_mode(now)
            return self.latched

        if not self.v.have_pos:
            return Action.NONE

        # Armare/dezarmare din FAZA, nu dintr-o tranzitie pe care aplicatia
        # trebuie sa o ghiceasca. Prima varianta arma pe tranzitia
        # IDLE -> DESCEND_TRACK; cand intrarea a devenit
        # IDLE -> HANDOVER_CHECK -> ACQUIRE -> DESCEND_TRACK, conditia nu s-a
        # mai potrivit niciodata si supervizorul a ramas inert, tacut, cu
        # TOATE monitoarele oprite. Legat de faza, nu se mai poate intampla.
        if self.auto_arm:
            in_segment = phase in self.autonomous_phases
            if in_segment and not self.armed:
                self.arm(now, self.v.x, self.v.y, self.v.alt)
            elif not in_segment and self.armed:
                self.disarm(now, f"faza {phase}")

        if not self.armed:
            return Action.NONE

        worst, monitor, detail = Action.NONE, None, ''
        for mon in (self._mon_override, self._mon_detection_age,
                    self._mon_link, self._mon_threads, self._mon_radius,
                    self._mon_ceiling, self._mon_descent_rate, self._mon_tilt):
            act, name, why = mon(now, detection_age_s, phase)
            if act > worst:
                worst, monitor, detail = act, name, why

        if worst != Action.NONE:
            self._trigger(now, worst, monitor, detail, phase)
        return worst

    def _latch_state(self):
        if self._mode_confirmed is not None:
            return 'confirmat'
        if self.passive or self._want_mode is None:
            return 'pasiv'
        if self._mode_req_n > MODE_RETRY_MAX:
            return 'epuizat'
        return 'in livrare'

    def _latch_final(self):
        """True when nothing more will ever be sent for the latched action:
        confirmed by the FC, taken by the pilot / a failsafe (passive), or
        given up after MODE_RETRY_MAX (mode_fail). Only such a latch may be
        released by a new attempt. A latch still being delivered (FC in the
        mode it had, retries left) is kept: a safety action is not lost
        because a switch was flipped."""
        return self._latch_state() != 'in livrare'

    def _set_abort(self, reason, passive=False, rtl=False):
        self.abort_reason = reason
        self.abort_passive = passive
        self.abort_rtl = bool(rtl)
        self.abort_n += 1
        self.abort.set()

    def _clear_abort(self):
        self.abort.clear()
        self.abort_reason = None
        self.abort_passive = False
        self.abort_rtl = False

    def _cere_mod(self, mode):
        """Comanda de mod a supervizorului: prin coada URGENT cand vehiculul
        o are (faza 2); vehiculele simulate din teste nu o au."""
        try:
            return self.v.request_mode(mode, urgent=True)
        except TypeError:
            return self.v.request_mode(mode)

    def set_thread_heartbeats(self, max_s=None, **heartbeats):
        """Heartbeat-urile firelor de supravegheat: detectie=, mav_io=,
        principal=. Un fir fara heartbeat dat nu e supravegheat."""
        if max_s is not None:
            self.heartbeat_max_s = float(max_s)
        for nume, hb in heartbeats.items():
            if hb is None:
                self.heartbeats.pop(nume, None)
            else:
                self.heartbeats[nume] = hb

    def _fire_stagnate(self, now):
        """[(nume, varsta)] pentru firele cu heartbeat-ul stagnat. Un fir
        care nu a batut niciodata nu e inca pornit - nu e mort."""
        rele = []
        for nume, hb in self.heartbeats.items():
            age = hb.age(now)
            if age is not None and age > self.heartbeat_max_s:
                rele.append((nume, age))
        return rele

    def _mon_threads(self, now, age, phase):
        """Faza 3: un fir mort inseamna sursa lui pierduta. Detectia
        moarta = detectie pierduta (BRAKE, in fazele in care detectia
        conteaza); I/O mort = legatura pierduta (BRAKE, aceleasi faze);
        firul principal mort = nimeni nu mai comanda: BRAKE, in tot
        segmentul autonom."""
        for nume, varsta in self._fire_stagnate(now):
            if nume == 'detectie' and phase in DETECTION_MONITORED_PHASES:
                return (Action.BRAKE, 'thread_detectie',
                        f"firul de detectie fara heartbeat de {varsta:.1f} s")
            if nume == 'mav_io' and phase in self.link_phases:
                return (Action.BRAKE, 'thread_mav_io',
                        f"firul I/O fara heartbeat de {varsta:.1f} s")
            if nume == 'principal' and phase in self.autonomous_phases:
                return (Action.BRAKE, 'thread_principal',
                        f"firul principal fara heartbeat de {varsta:.1f} s")
        return Action.NONE, None, ''

    def _trigger(self, now, action, monitor, detail, phase):
        self.latched = action
        self._mode_confirmed = None
        if action == Action.OVERRIDE:
            self.passive = True
        self.latched_monitor = monitor
        self._set_abort(f"{monitor}: {detail}", passive=False)
        self._emit(now, monitor, action, detail, phase)
        self._want_mode = Action.MODES[action]
        # The mode the FC was in when we decided. While unconfirmed we
        # re-send only as long as the FC is STILL in this mode; any third
        # mode means the pilot or a failsafe acted in the meantime (§5.66).
        self._mode_before = self.v.mode
        self._mode_req_t = 0.0
        self._mode_req_n = 0
        self._drive_mode(now)

    def _drive_mode(self, now):
        """Comanda modul si verifica INDEPENDENT ca FC-ul chiar l-a adoptat.

        Ce a decis supervizorul nu e dovada; dovada e ce raporteaza FC-ul in
        HEARTBEAT. Fara verificarea asta, un test de siguranta poate raporta
        succes desi vehiculul a continuat sa coboare."""
        if self._want_mode is None:
            return
        if self.v.mode == self._want_mode:
            if self._mode_confirmed is None:
                self._mode_confirmed = now
                extra = ''
                if self.latched == Action.OVERRIDE:
                    lat = self.override.note_mode_confirmed(now)
                    # 15.3.1 cere sub 250 ms, si cere masuratoare, nu afirmatie
                    extra = (f", latenta override {lat * 1000:.0f} ms "
                             f"(buget 250 ms)") if lat is not None else ''
                self._emit(now, 'mode_confirm', self.latched,
                           f"FC raporteaza {self.v.mode_name()} dupa "
                           f"{self._mode_req_n} comenzi{extra}", 'CONFIRM')
            return
        if self._mode_confirmed is not None:
            # INCIDENT 26.09.2026 (§5.66). The FC had adopted our mode and
            # has since LEFT it. Only two things can do that: the pilot's
            # mode switch or an FC failsafe. Both outrank us (15.1.7). The
            # old code treated it as a lost command and re-sent BRAKE within
            # 20-30 ms - over the pilot's STABILIZE, over the pilot's LOITER,
            # and over the battery failsafe's LAND. The vehicle hovered in
            # BRAKE until the battery collapsed. A confirmed command is
            # FINAL: from here the supervisor is passive and sends nothing.
            self._emit(now, 'mode_taken', self.latched,
                       f"FC a trecut din {Action.NAMES[self.latched]} in "
                       f"{self.v.mode_name()} dupa confirmare: decizia "
                       f"pilotului sau a unui failsafe. Supervizorul devine "
                       f"PASIV, nu retrimite nimic.", 'PASSIVE')
            self._want_mode = None
            self.passive = True
            return
        if (self._mode_before is not None and self.v.mode is not None
                and self.v.mode != self._mode_before):
            # Unconfirmed, but the FC is in a THIRD mode: neither the one it
            # had when we decided nor the one we asked for. Someone else set
            # it (pilot switch or failsafe) inside our retry window. A lost
            # serial command cannot produce a third mode, so this is not a
            # delivery problem to retry - it outranks us. Same rule as the
            # confirmed case above: passive, nothing more is sent.
            self._emit(now, 'mode_taken', self.latched,
                       f"FC a trecut in {self.v.mode_name()}, un mod pe care "
                       f"nu l-am cerut, inainte de confirmare: decizia "
                       f"pilotului sau a unui failsafe. Supervizorul devine "
                       f"PASIV.", 'PASSIVE')
            self._want_mode = None
            self.passive = True
            return
        if now - self._mode_req_t < MODE_CONFIRM_S:
            return
        if self._mode_req_n >= MODE_RETRY_MAX:
            if self._mode_req_n == MODE_RETRY_MAX:
                self._mode_req_n += 1
                self._emit(now, 'mode_fail', self.latched,
                           f"FC ramane in {self.v.mode_name()} dupa "
                           f"{MODE_RETRY_MAX} comenzi", 'FAIL')
            return
        # H1: daca legatura e cazuta, request_mode() intoarce False fara sa
        # trimita nimic. Nu contorizam incercarea - altfel supervizorul si-ar
        # consuma cele MODE_RETRY_MAX incercari vorbind cu un port inchis si
        # ar declara mode_fail fara sa fi emis vreun octet. Cand legatura
        # revine, comanda pleaca si numaratoarea e intacta.
        self._mode_req_t = now
        if self._cere_mod(self._want_mode) is False:
            return
        self._mode_req_n += 1

    # -- monitoare ---------------------------------------------------------
    # Fiecare intoarce (actiune, nume, motiv). Niciunul nu are voie sa
    # foloseasca date de viziune.

    def _mon_override(self, now, age, phase):
        """15.3.1 / 15.1.7: pilotul a miscat o mansa -> pasiv definitiv.

        Nu depinde de faza: pilotul poate prelua in orice moment al
        segmentului autonom, inclusiv in coborarea finala."""
        if not self.override.update(now):
            return Action.NONE, None, ''
        return (Action.OVERRIDE, 'pilot_override',
                f"abatere {self.override.deviation()} PWM sustinuta "
                f"{self.override.hold_s * 1000:.0f} ms "
                f"(prag {self.override.deadband_pwm})")

    def _mon_detection_age(self, now, age, phase):
        """15.2.9: sub pragul de incredere, planeaza - nu ateriza.

        Nu se aplica in FINAL_DESCENT si dupa: vezi
        DETECTION_MONITORED_PHASES pentru motiv."""
        if phase not in DETECTION_MONITORED_PHASES:
            return Action.NONE, None, ''
        if age is None:
            return (Action.BRAKE, 'detection_age',
                    'nicio detectie valida de la intrarea in faza')
        streak = self.miss_streak
        if streak is not None:
            # Regula echipei: cadre CONSECUTIVE ratate, nu o pauza de timp.
            if streak >= self.detection_max_misses:
                return (Action.BRAKE, 'detection_age',
                        f"{streak} cadre consecutive fara marker "
                        f"(prag {self.detection_max_misses}), "
                        f"ultima detectie acum {age:.2f} s")
            if age > self.detection_hard_max_age_s:
                return (Action.BRAKE, 'detection_age',
                        f"ultima detectie acum {age:.2f} s, peste plafonul "
                        f"de {self.detection_hard_max_age_s:.1f} s "
                        f"(detector blocat? doar {streak} ratari numarate)")
            return Action.NONE, None, ''
        if age > self.detection_max_age_s:
            return (Action.BRAKE, 'detection_age',
                    f"ultima detectie acum {age:.2f} s "
                    f"(prag {self.detection_max_age_s:.2f} s)")
        return Action.NONE, None, ''

    def _mon_link(self, now, age, phase):
        """H1: legatura cu FC-ul cazuta -> planeaza, ca la pierderea detectiei.

        ACELEASI faze ca monitorul de detectie, si pentru acelasi motiv
        (DETECTION_MONITORED_PHASES). In FINAL_DESCENT si dupa, coborarea e
        deliberat oarba si verticala: nu mai trimitem corectii laterale, deci
        o legatura cazuta acolo nu schimba traiectoria. A abortat in ultimul
        metru pentru un cablu ar fi exact greseala pe care o evitam la
        detectie.

        Se bazeaza pe `Vehicle.time_since_heartbeat()`. Un vehicul care nu
        expune metoda (teste vechi, obiecte simulate) nu e monitorizat -
        formulat asa deliberat, ca introducerea monitorului sa nu strice ce
        mergea; testele care CHIAR verifica monitorul folosesc un vehicul
        care o expune."""
        if phase not in self.link_phases:
            return Action.NONE, None, ''
        fn = getattr(self.v, 'time_since_heartbeat', None)
        if fn is None:
            return Action.NONE, None, ''
        link_age = fn(now)
        if link_age is None:
            return (Action.BRAKE, 'link_age',
                    'niciun HEARTBEAT de la FC de la pornire')
        if link_age > self.link_max_age_s:
            return (Action.BRAKE, 'link_age',
                    f"fara HEARTBEAT de {link_age:.2f} s "
                    f"(prag {self.link_max_age_s:.2f} s); "
                    f"LANDING_TARGET nu mai ajunge la FC")
        return Action.NONE, None, ''

    def _mon_radius(self, now, age, phase):
        """15.2.4: raza fata de punctul de handover."""
        if phase not in self.geometry_phases:
            return Action.NONE, None, ''
        d = ((self.v.x - self.origin_n) ** 2 +
             (self.v.y - self.origin_e) ** 2) ** 0.5
        if d > self.geofence_radius_m:
            return (Action.RTL, 'geofence_radius',
                    f"{d:.2f} m fata de handover "
                    f"(prag {self.geofence_radius_m:.1f} m)")
        return Action.NONE, None, ''

    def _mon_ceiling(self, now, age, phase):
        """15.2.4: plafon deasupra punctului de handover."""
        if phase not in self.geometry_phases:
            return Action.NONE, None, ''
        agl = self.v.alt - self.origin_alt
        if agl > self.ceiling_agl_m:
            return (Action.RTL, 'ceiling',
                    f"{agl:.1f} m AGL (prag {self.ceiling_agl_m:.1f} m)")
        return Action.NONE, None, ''

    def _mon_descent_rate(self, now, age, phase):
        """Coborare mai rapida decat poate comanda controlerul de aterizare.

        Singura parghie a unui companion e schimbarea de mod; limitarea
        propriu-zisa a vitezei sta in FC (LAND_SPD_MS, WP_SPD_DN on the
        4.7.1 board). Aici doar
        oprim coborarea."""
        if phase not in self.autonomous_phases:
            self._descent_since = None
            return Action.NONE, None, ''
        if self.v.vz <= self.max_descent_rate_ms:
            self._descent_since = None
            return Action.NONE, None, ''
        if self._descent_since is None:
            self._descent_since = now
        if now - self._descent_since >= DESCENT_RATE_HOLD_S:
            return (Action.BRAKE, 'descent_rate',
                    f"{self.v.vz:.2f} m/s de {now - self._descent_since:.2f} s "
                    f"(prag {self.max_descent_rate_ms:.1f} m/s)")
        return Action.NONE, None, ''

    def _mon_tilt(self, now, age, phase):
        if phase not in self.autonomous_phases:
            self._tilt_since = None
            return Action.NONE, None, ''
        tilt = max(abs(self.v.roll), abs(self.v.pitch))
        limit = self.max_tilt_deg * 3.14159265 / 180.0
        if tilt <= limit:
            self._tilt_since = None
            return Action.NONE, None, ''
        if self._tilt_since is None:
            self._tilt_since = now
        if now - self._tilt_since >= TILT_HOLD_S:
            return (Action.BRAKE, 'tilt',
                    f"{tilt * 180 / 3.14159265:.1f} deg de "
                    f"{now - self._tilt_since:.2f} s "
                    f"(prag {self.max_tilt_deg:.0f} deg)")
        return Action.NONE, None, ''

    # -- raportare ---------------------------------------------------------
    def status(self):
        if not self.armed:
            return 'SAFETY dezarmat'
        if self.latched == Action.NONE:
            fn = getattr(self.v, 'time_since_heartbeat', None)
            link = ''
            if fn is not None:
                a = fn()
                link = (' | link -' if a is None
                        else f" | link {a:.1f}s")
            return f"SAFETY activ | {self.override.status()}{link}"
        conf = 'confirmat' if self._mode_confirmed else 'NECONFIRMAT'
        if self.passive and self._want_mode is None:
            conf = 'PASIV - pilotul/FC-ul a preluat modul'
        return (f"SAFETY {Action.NAMES[self.latched]} "
                f"({self.latched_monitor}), mod {conf}")


# --- ExtNav (27.09.2026, claude-markdown/REPROIECTARE_EXTNAV.md §6) --------

#: Phases of nova/extnav_landing.py in which the supervisor is armed.
#: Positive list, like AUTONOMOUS_PHASES.
EXTNAV_PHASES = ('GATE_SEARCH',
                 # B3 (04.10.2026): the marker search by rotation - armed,
                 # so the ceiling, the override and the mode rule hold there
                 'YAW_SEARCH',
                 'ENGAGE', 'MOVE', 'CENTER_CHECK', 'DESCEND',
                 'FINAL_ALIGN', 'LAND', 'TOUCHDOWN',
                 # touchdown without disarming and the climb back (15.1.2):
                 # the segment goes on after contact, so does supervision
                 'TOUCHDOWN_DESCENT', 'CONTACT', 'GROUND_HOLD', 'RISEUP',
                 'HOVER_CONFIRM')
#: Phases after ENGAGE: the EKF runs on SRC2 with the camera as position
#: source, and the vehicle is guided on it. Here a lost link or an invalid
#: EKF position means the guidance has nothing to stand on -> EXIT.
#: GATE_SEARCH is before the switch (pilot in LOITER, we command nothing);
#: ENGAGE is the switch itself, guarded by its own timeout in the state
#: machine (a reset of position is expected there).
EXTNAV_ENGAGED_PHASES = ('MOVE', 'CENTER_CHECK', 'DESCEND', 'FINAL_ALIGN',
                         'LAND', 'TOUCHDOWN', 'TOUCHDOWN_DESCENT', 'CONTACT',
                         'GROUND_HOLD', 'RISEUP', 'HOVER_CONFIRM')
#: B3 (04.10.2026): YAW_SEARCH runs on SRC2 and in GUIDED like the phases
#: above, so the link, the EKF position (valid for 7.0 s after the switch
#: without vision, measured - an invalid one there is a fault -> EXIT) and
#: the threads are watched there. NOT the geometry: the EKF frame jumps by
#: tens of metres at the first detection during the search, so the radius
#: origin may only be anchored after that reset - on entering the first
#: geometry phase (MOVE), see ExtNavSupervisor.update.
EXTNAV_SEARCH_PHASES = ('YAW_SEARCH',)
EXTNAV_EKF_PHASES = EXTNAV_SEARCH_PHASES + EXTNAV_ENGAGED_PHASES
#: The climb back to alt_riseup: GUIDED takeoff climbs at WP_SPD_UP
#: (2.5 m/s by default, not set in our parameter files). Faster than this,
#: sustained, is not the takeoff controller any more -> EXIT.
CLIMB_PHASES = ('RISEUP', 'HOVER_CONFIRM')
#: B1 (02.10.2026, rule 15.2.4): phases in which the 30 m ceiling above
#: HOME is enforced - the autonomous procedure, from AUX up (GATE_SEARCH)
#: to the hand-back. Positive list (§5.25): IDLE, REJECT, GATE_FAIL, EXIT,
#: EXIT_RTL, COMPLETE, ABORT, DONE and any phase not listed here are NOT
#: watched - outside the sequence the vehicle is the pilot's and flies
#: above 30 m in the race (user decision 02.10.2026: no fence on the FC).
CEILING_PHASES = EXTNAV_PHASES
MAX_CLIMB_RATE_MS = 3.0
CLIMB_RATE_HOLD_S = 0.5
#: B1.9 (G1, 02.10.2026): the supervisor's direct exit commands LOITER only
#: on a FRESH view showing the FC in GUIDED (the mode the companion holds).
#: Fresh = the snapshot was published at most this long ago (the I/O
#: thread publishes every <= 10 ms) and this thread was not stalled.
HANDBACK_VIEW_MAX_AGE_S = 0.5
#: EKF_STATUS_REPORT is requested at 5 Hz (vehicle.EKF_HZ). Older than
#: this, or never received while engaged, the position is unknown - and
#: unknown is not valid.
EKF_REPORT_MAX_AGE_S = 2.0


class ExtNavSupervisor(SafetySupervisor):
    """Supervisor for the ExtNav landing (brief §6). Same monitors, minus
    the detection one (between detections the EKF holds the position, and
    a missing detection is the state machine's window logic, not a fault),
    plus the validity of the EKF position. One action, EXIT, executed by
    the state machine through `on_exit(reason)` in a fixed order (SRC1
    restored and read back, LOITER, ALT_HOLD). This class sends NO mode
    command - so the incident class of §5.66 has no path here.

    B1 (02.10.2026, rule 15.2.4): the ceiling is the one monitor with
    another action. Above `ceiling_agl_m` over HOME (relative_alt) for
    `ceiling_confirm_n` distinct samples, in CEILING_PHASES, it asks for
    EXIT_RTL (abort with `abort_rtl`, or on_exit(reason, rtl=True)); the
    state machine restores SRC1, waits for the EKF to be back in the GNSS
    frame and asks RTL. Still no mode command from here.

        sup = ExtNavSupervisor(vehicle, override=ov, on_exit=sm.request_exit)
        sup.update(now, phase)
    """

    def __init__(self, vehicle, on_exit=None,
                 ceiling_confirm_n=CEILING_CONFIRM_N, **kw):
        super().__init__(vehicle, **kw)
        self.on_exit = on_exit
        self.autonomous_phases = EXTNAV_PHASES
        self.link_phases = EXTNAV_EKF_PHASES
        self.ekf_phases = EXTNAV_EKF_PHASES
        self.gate_phases = ('GATE_SEARCH',)
        # Radius only once the frame is the marker's and the origin has
        # been re-anchored (see update): in GATE_SEARCH the vehicle is the
        # pilot's, in ENGAGE the EKF position is being reset, in YAW_SEARCH
        # (B3) it is reset at the first detection, mid-phase. The ceiling
        # does not use this origin any more (B1): it is above HOME.
        self.geometry_phases = EXTNAV_ENGAGED_PHASES
        self.ceiling_phases = CEILING_PHASES
        self.ceiling_confirm_n = int(ceiling_confirm_n)
        self._ceil_reset()
        self.ekf_report_max_age_s = EKF_REPORT_MAX_AGE_S
        self._engaged_since = None
        #: B3: when the current run of geometry phases began (the radius
        #: origin is re-anchored at its start), None outside them
        self._geometry_since = None
        self.max_climb_rate_ms = MAX_CLIMB_RATE_MS
        self._climb_since = None
        #: B1.9 (G2): this thread's previous update() time, and the sticky
        #: stall flag (cleared only by a new attempt through the gate)
        self._last_update_t = None
        self._stalled = False
        self.stall_gap_s = None

    def _ceil_reset(self):
        #: consecutive distinct samples above the ceiling, and the identity
        #: (n_gpi) of the last sample counted
        self._ceil_count = 0
        self._ceil_last_n = None
        self._ceil_last_h = None

    def _mon_ceiling(self, now, age, phase):
        """15.2.4 on ExtNav (B1): height above HOME over the ceiling.

        Deterministic: GLOBAL_POSITION_INT.relative_alt (barometric, the
        FC's own height above home - not the EKF origin, which ENGAGE
        re-anchors on the marker, and not the handover altitude). Counted
        on DISTINCT samples (`n_gpi`, the message counter of the vehicle):
        this thread runs at 200 Hz, the message comes at 20 Hz, and the
        same sample seen 10 times is still one sample. `ceiling_confirm_n`
        consecutive samples above -> EXIT_RTL; one at or below resets.

        No sample yet / relative_alt None / a vehicle without the counter:
        no verdict from THIS monitor, deliberately. A link that stopped
        delivering is the link monitor's case (link_age, EXIT), not a
        ceiling breach - and a breach must not be invented from nothing.

        This replaces SafetySupervisor._mon_ceiling (origin-relative, i.e.
        above the handover point: a handover at 12 m would only have fired
        at ~42 m above home) on this supervisor; the PLND one is unchanged.
        """
        if phase not in self.ceiling_phases:
            self._ceil_reset()
            return Action.NONE, None, ''
        n = getattr(self.v, 'n_gpi', None)
        h = getattr(self.v, 'rel_alt', None)
        if n is None or h is None:
            return Action.NONE, None, ''
        if n != self._ceil_last_n:
            self._ceil_last_n = n
            self._ceil_last_h = h
            if h > self.ceiling_agl_m:
                self._ceil_count += 1
            else:
                self._ceil_count = 0
        if self._ceil_count >= self.ceiling_confirm_n:
            return (Action.EXIT_RTL, 'ceiling_agl',
                    f"{self._ceil_last_h:.1f} m deasupra home, "
                    f"{self._ceil_count} esantioane la rand peste plafonul "
                    f"de {self.ceiling_agl_m:.1f} m")
        return Action.NONE, None, ''

    def _principal_dead(self, now, phase):
        """The main thread's heartbeat is stale where the thread monitor
        watches it. Used by the double-fault rule in update()."""
        if phase not in self.ekf_phases:
            return False
        return any(nume == 'principal' for nume, _ in self._fire_stagnate(now))

    def update(self, now=None, detection_age_s=None, phase='IDLE',
               miss_streak=None):
        """Same positional order as SafetySupervisor.update, because the
        shared run_loop calls it positionally: (now, age, phase). The
        detection arguments are accepted and ignored."""
        now = now if now is not None else time.monotonic()
        self.override.observe(now, phase in self.gate_phases)
        new_attempt = (phase in self.autonomous_phases
                       and self._last_phase not in self.autonomous_phases)
        self._last_phase = phase
        # B1.9 (G2): our own gap. Above heartbeat_max_s (THREAD_HEARTBEAT_
        # MAX_S, 1.0 s) this thread - or the whole process - was stalled:
        # the other threads' heartbeats look old because WE did not run,
        # and the snapshot may hold old bytes read after the freeze. Sticky
        # until a new attempt; a gap measured in this very call still
        # counts for it (it happened before the attempt's first look).
        if new_attempt:
            self._stalled = False
        gap = (None if self._last_update_t is None
               else now - self._last_update_t)
        self._last_update_t = now
        if gap is not None and gap > self.heartbeat_max_s:
            self._stalled = True
            self.stall_gap_s = gap
            self._emit(now, 'stall', Action.NONE,
                       f"firul supervizorului nu a rulat {gap:.1f} s: fara "
                       f"comenzi de mod pana la o incercare noua", phase)
        if phase in self.ekf_phases:
            if self._engaged_since is None:
                self._engaged_since = now
        else:
            self._engaged_since = None
        if phase in self.geometry_phases:
            if self._geometry_since is None:
                self._geometry_since = now
                # The EKF position has just been RESET onto the marker frame
                # (the state machine enters MOVE only after observing it).
                # The radius origin captured at arm time was in
                # the previous frame: measured against it, the reset alone
                # reads as tens of metres and would EXIT at once. Re-anchor
                # here; from now on the frame is stable.
                # B3: on entering the first GEOMETRY phase, no longer the
                # first EKF one - YAW_SEARCH is an EKF phase, and its reset
                # comes mid-phase, at the first detection: anchored on
                # entering YAW_SEARCH, the origin would sit in the GNSS frame
                # and MOVE would trip the radius by the frame jump.
                if self.armed and self.v.have_pos:
                    self.origin_n, self.origin_e = self.v.x, self.v.y
                    self.origin_alt = self.v.alt
                    self._emit(now, 'anchor', Action.NONE,
                               f"origine geometrica reancorata dupa resetul "
                               f"EKF: N={self.v.x:.2f} E={self.v.y:.2f} "
                               f"alt={self.v.alt:.2f}", phase)
        else:
            self._geometry_since = None

        if self.latched != Action.NONE:
            # EXIT was asked once; the state machine owns it from there.
            # A new attempt through the gate starts clean.
            # DELIBERATE (B1): nothing escalates over a latched exit. Once
            # an EXIT is latched for another reason (the pilot is getting
            # the vehicle back in LOITER), a ceiling breach after it does
            # NOT turn it into EXIT_RTL: we return here, before any monitor
            # runs. Same rule as the mode rule of §5.66 - the hand-back
            # under way is not overruled by a second command of ours.
            if new_attempt:
                self._emit(now, 'latch_release', self.latched,
                           f"incercare noua ({phase}): EXIT-ul anterior "
                           f"s-a consumat", phase)
                self.latched = Action.NONE
                self.latched_monitor = None
                self.passive = False
                self._clear_abort()
                self._exit_direct_done = False
                self.armed = False
                self._ceil_reset()
            else:
                return self.latched

        if not self.v.have_pos:
            return Action.NONE
        if self.auto_arm:
            in_segment = phase in self.autonomous_phases
            if in_segment and not self.armed:
                self.arm(now, self.v.x, self.v.y, self.v.alt)
                self._ceil_reset()
            elif not in_segment and self.armed:
                self.disarm(now, f"faza {phase}")
        if not self.armed:
            self._ceil_reset()
            return Action.NONE

        # B1.9 (G2): after our own stall, no verdict from stale knowledge -
        # not "thread dead -> direct exit with LOITER" from heartbeats that
        # look old only because we slept, not RTL either: a PASSIVE exit
        # (SRC1 requested by the state machine, no mode command).
        if self._stalled:
            self._trigger_exit(now, 'stall_supervizor',
                               f"firul supervizorului blocat "
                               f"{self.stall_gap_s or 0.0:.1f} s: iesire pasiva",
                               phase, passive=True)
            return Action.EXIT

        # B1: the ceiling first. In the same step it outranks every other
        # monitor - it is the only one whose action is RTL (rule 15.2.4:
        # abort -> return home). One exception, a DOUBLE FAULT: the main
        # thread is dead too. EXIT_RTL is executed by the state machine, in
        # that very thread, so asking for it would ask nobody; the thread
        # fault is handled instead (_exit_direct: SRC1 + LOITER, urgent,
        # once), exactly as without a breach.
        c_act, c_mon, c_why = self._mon_ceiling(now, None, phase)
        if c_act != Action.NONE:
            if self._principal_dead(now, phase):
                varsta = dict(self._fire_stagnate(now)).get('principal', 0.0)
                self._emit(now, 'ceiling_agl', Action.NONE,
                           f"{c_why} - DAR firul principal e mort: tratat ca "
                           f"defectiunea firului (dubla defectiune)", phase)
                self._trigger_exit(now, 'thread_principal',
                                   f"firul principal fara heartbeat de "
                                   f"{varsta:.1f} s (si plafon depasit)", phase)
                return Action.EXIT
            else:
                self._trigger_exit(now, c_mon, c_why, phase, rtl=True)
                return Action.EXIT_RTL

        worst, monitor, detail = Action.NONE, None, ''
        # The ceiling is NOT in this list (B1): it ran above, with its own
        # action; the old origin-relative one must not fire here as well.
        for mon in (self._mon_override, self._mon_link, self._mon_ekf,
                    self._mon_threads, self._mon_radius,
                    self._mon_descent_rate, self._mon_climb_rate,
                    self._mon_tilt):
            act, name, why = mon(now, None, phase)
            if act > worst:
                worst, monitor, detail = act, name, why
        if worst != Action.NONE:
            self._trigger_exit(now, monitor, detail, phase)
            return Action.EXIT
        return Action.NONE

    def _mon_climb_rate(self, now, age, phase):
        """The climb back (RISEUP, HOVER_CONFIRM): faster than
        max_climb_rate_ms for CLIMB_RATE_HOLD_S -> EXIT. NED: up is -vz."""
        if phase not in CLIMB_PHASES or -self.v.vz <= self.max_climb_rate_ms:
            self._climb_since = None
            return Action.NONE, None, ''
        if self._climb_since is None:
            self._climb_since = now
        if now - self._climb_since >= CLIMB_RATE_HOLD_S:
            return (Action.EXIT, 'climb_rate',
                    f"urcare {-self.v.vz:.2f} m/s de {now - self._climb_since:.2f} s "
                    f"(prag {self.max_climb_rate_ms:.1f} m/s)")
        return Action.NONE, None, ''

    def _mon_threads(self, now, age, phase):
        """Faza 3, pe ExtNav: orice fir mort dupa ENGAGE e EXIT - un fir
        mort e o defectiune, nu o fereastra ratata. Firul principal mort
        nu mai poate executa EXIT-ul: supervizorul il face singur, direct
        (_exit_direct)."""
        if phase not in self.ekf_phases:
            return Action.NONE, None, ''
        for nume, varsta in self._fire_stagnate(now):
            return (Action.EXIT, f"thread_{nume}",
                    f"firul {nume} fara heartbeat de {varsta:.1f} s")
        return Action.NONE, None, ''

    def _trigger_exit(self, now, monitor, detail, phase, rtl=False,
                      passive=False):
        """Latch and ask the state machine for the exit: the normal EXIT,
        or EXIT_RTL (`rtl`, the ceiling only). The kind travels with the
        reason (abort_rtl next to abort_reason; on_exit(reason, rtl=True)
        on the callback wiring of the simulator and the tests)."""
        action = Action.EXIT_RTL if rtl else Action.EXIT
        # B1.9: when this supervisor does the hand-back itself (main thread
        # dead -> _exit_direct), the abort is passive for the state machine:
        # if it ever wakes up it only re-requests SRC1, no second mode.
        passive = passive or monitor == 'thread_principal'
        self.latched = action
        self.latched_monitor = monitor
        self.passive = monitor == 'pilot_override' or passive
        self._set_abort(f"{monitor}: {detail}", passive=passive, rtl=rtl)
        self._emit(now, monitor, action, detail, phase)
        if monitor == 'thread_principal':
            self._exit_direct(now, phase)
        if self.on_exit is not None:
            if rtl:
                try:
                    self.on_exit(f"{monitor}: {detail}", rtl=True)
                    return
                except TypeError:          # a callback without the kind
                    pass
            self.on_exit(f"{monitor}: {detail}")

    def _exit_direct(self, now, phase):
        """EXIT-ul ordonat, facut de supervizor cand masina de stari nu
        mai poate (firul principal mort): setul EKF 1 inapoi, apoi LOITER,
        amandoua prin URGENT, o singura data. Fara ALT_HOLD de rezerva:
        aici nu mai e nimeni sa confirme modurile.

        Unchanged by B1, deliberately: a ceiling breach together with a
        dead main thread is a DOUBLE fault and is handled as the thread
        fault - SRC1 + LOITER, not RTL. RTL needs the ordered path (SRC1
        acknowledged, EKF back in the GNSS frame) that only the state
        machine runs; asked blindly from here it could fly to the marker
        frame's idea of home. The pilot gets the vehicle in LOITER."""
        if getattr(self, '_exit_direct_done', False):
            return
        self._exit_direct_done = True
        ok1 = False
        fn = getattr(self.v, 'send_ekf_source_set', None)
        if fn is not None:
            try:
                ok1 = fn(1, urgent=True)
            except TypeError:
                ok1 = fn(1)
        # B1.9 (G1): LOITER only over the mode we hold, seen on a FRESH view
        # of our own. Anything else (the pilot's ALT_HOLD, a failsafe's LAND
        # or RTL, a stale snapshot, our own stall) -> SRC1 only: passive.
        fresh, why = self.view_fresh(now)
        if fresh and self.v.mode == MODE_GUIDED:
            ok2 = self._cere_mod(MODE_LOITER)
            self._emit(now, 'exit_direct', Action.EXIT,
                       f"masina de stari nu raspunde: SRC1 ({ok1}) si LOITER "
                       f"({ok2}) cerute direct, prin URGENT", phase)
            return
        self.passive = True
        self._emit(now, 'exit_direct', Action.EXIT,
                   f"masina de stari nu raspunde: SRC1 ({ok1}) cerut direct; "
                   f"FARA LOITER - {why or f'FC in {self.v.mode_name()}, nu GUIDED'}",
                   phase)

    def view_fresh(self, now):
        """(True, '') when this thread may trust its view for a mode
        command: not stalled, and the snapshot (VehicleView.last_snapshot_t)
        at most HANDBACK_VIEW_MAX_AGE_S old. A view without a snapshot time
        (the synchronous wiring: the vehicle itself) counts as fresh."""
        if self._stalled:
            return False, 'firul supervizorului a fost blocat'
        t = getattr(self.v, 'last_snapshot_t', None)
        if t is not None and now - t > HANDBACK_VIEW_MAX_AGE_S:
            return False, f"instantaneu vechi de {now - t:.1f} s"
        return True, ''

    def _mon_ekf(self, now, age, phase):
        """Brief §6: the EKF must hold a valid horizontal position while
        the vehicle is guided on it. False -> EXIT; no report for longer
        than EKF_REPORT_MAX_AGE_S while engaged -> EXIT (unknown is not
        valid)."""
        if phase not in self.ekf_phases:
            return Action.NONE, None, ''
        fn = getattr(self.v, 'ekf_pos_horiz_ok', None)
        if fn is None:
            return Action.NONE, None, ''
        ok = fn()
        t_rep = getattr(self.v, 'ekf_t', None)
        if ok is None or t_rep is None:
            since = self._engaged_since if self._engaged_since is not None else now
            if now - since > self.ekf_report_max_age_s:
                return (Action.EXIT, 'ekf_position',
                        f"niciun EKF_STATUS_REPORT de {now - since:.1f} s "
                        f"(prag {self.ekf_report_max_age_s:.1f} s)")
            return Action.NONE, None, ''
        if now - t_rep > self.ekf_report_max_age_s:
            return (Action.EXIT, 'ekf_position',
                    f"EKF_STATUS_REPORT vechi de {now - t_rep:.1f} s")
        if not ok:
            return (Action.EXIT, 'ekf_position',
                    'EKF fara pozitie orizontala valida (EKF_POS_HORIZ_REL)')
        return Action.NONE, None, ''

    def status(self):
        if not self.armed:
            return 'SAFETY dezarmat'
        if self.latched == Action.NONE:
            fn = getattr(self.v, 'ekf_pos_horiz_ok', None)
            ok = fn() if fn is not None else None
            ekf = '-' if ok is None else ('ok' if ok else 'INVALID')
            return f"SAFETY activ | {self.override.status()} | ekf {ekf}"
        return (f"SAFETY {Action.NAMES.get(self.latched, self.latched)} "
                f"({self.latched_monitor}), executat de masina de stari")


# Modurile in care supervizorul considera ca vehiculul nu mai e in segmentul
# autonom. Expus ca sa il poata folosi si aplicatia.
HANDBACK_MODES = (MODE_BRAKE, MODE_RTL, MODE_LAND)
