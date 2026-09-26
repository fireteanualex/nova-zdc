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

import time

from .rc import OverrideMonitor
from .vehicle import MODE_BRAKE, MODE_LAND, MODE_LOITER, MODE_RTL

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
GEOFENCE_RADIUS_M = 10.0
CEILING_AGL_M = 30.0

#: Rata de coborare peste care ceva e in neregula. Coborarea normala e
#: limitata de LAND_SPD_MS (0.5 m/s) si WP_SPD_DN; 2.0 m/s inseamna ca nu mai
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

    NAMES = {NONE: 'NONE', BRAKE: 'BRAKE', LOITER: 'LOITER', RTL: 'RTL',
             OVERRIDE: 'OVERRIDE'}
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

        self.armed = False
        self.auto_arm = True      # se armeaza singur din faza primita
        self.origin_n = self.origin_e = 0.0
        self.origin_alt = 0.0

        # 15.3.1: detectia de override pe manse. Monitor separat, ca sa poata
        # fi testat independent - si ca sa fie ACELASI obiect pe care il
        # foloseste poarta de handover cand memoreaza referinta de neutru.
        self.override = override if override is not None else OverrideMonitor(vehicle)
        #: True dupa un override: companion-ul nu mai comanda NIMIC pentru
        #: restul incercarii. Zavor ireversibil, si daca mansa revine la
        #: neutru.
        self.passive = False

        self.latched = Action.NONE
        self.latched_monitor = None
        self.log = []
        self._last_phase = 'IDLE'

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
        self._descent_since = None
        self._tilt_since = None
        self._want_mode = None
        self._mode_confirmed = None
        self.passive = False
        if rc_neutral_captured and self.override.neutral is None:
            # Cine nu trece prin HANDOVER_CHECK (teste, intrare directa in
            # LAND) primeste referinta de neutru acum. In cursa, referinta se
            # memoreaza la validare, dupa fereastra de asezare (15.3.1 B3.1).
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
        if (self.latched != Action.NONE and new_attempt
                and self._mode_confirmed is not None):
            self._emit(now, 'latch_release', self.latched,
                       f"incercare noua acceptata de poarta ({phase}): "
                       f"zavorul {Action.NAMES[self.latched]}, confirmat, "
                       f"se elibereaza", phase)
            self.latched = Action.NONE
            self.latched_monitor = None
            self._want_mode = None
            self._mode_confirmed = None
            self.passive = False
            self.armed = False          # re-armare curata din faza, mai jos

        if self.latched != Action.NONE:
            # Singura escaladare permisa peste un zavor: pilotul. 15.1.7 ii da
            # autoritate oricand, inclusiv peste un RTL pe care tocmai l-am
            # comandat noi - altfel ne-am bate cu el pe comenzi. Un geofence
            # breach se declanseaza instantaneu, override-ul are nevoie de
            # OVERRIDE_HOLD_S, deci fara asta RTL ar castiga mereu cursa.
            if self.latched != Action.OVERRIDE and self.override.update(now):
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
            in_segment = phase in AUTONOMOUS_PHASES
            if in_segment and not self.armed:
                self.arm(now, self.v.x, self.v.y, self.v.alt)
            elif not in_segment and self.armed:
                self.disarm(now, f"faza {phase}")

        if not self.armed:
            return Action.NONE

        worst, monitor, detail = Action.NONE, None, ''
        for mon in (self._mon_override, self._mon_detection_age,
                    self._mon_link, self._mon_radius, self._mon_ceiling,
                    self._mon_descent_rate, self._mon_tilt):
            act, name, why = mon(now, detection_age_s, phase)
            if act > worst:
                worst, monitor, detail = act, name, why

        if worst != Action.NONE:
            self._trigger(now, worst, monitor, detail, phase)
        return worst

    def _trigger(self, now, action, monitor, detail, phase):
        self.latched = action
        self._mode_confirmed = None
        if action == Action.OVERRIDE:
            self.passive = True
        self.latched_monitor = monitor
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
        if self.v.request_mode(self._want_mode) is False:
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
        if phase not in DETECTION_MONITORED_PHASES:
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
        if phase not in AUTONOMOUS_PHASES:
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
        if phase not in AUTONOMOUS_PHASES:
            return Action.NONE, None, ''
        agl = self.v.alt - self.origin_alt
        if agl > self.ceiling_agl_m:
            return (Action.RTL, 'ceiling',
                    f"{agl:.1f} m AGL (prag {self.ceiling_agl_m:.1f} m)")
        return Action.NONE, None, ''

    def _mon_descent_rate(self, now, age, phase):
        """Coborare mai rapida decat poate comanda controlerul de aterizare.

        Singura parghie a unui companion e schimbarea de mod; limitarea
        propriu-zisa a vitezei sta in FC (LAND_SPD_MS, WP_SPD_DN). Aici doar
        oprim coborarea."""
        if phase not in AUTONOMOUS_PHASES:
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
        if phase not in AUTONOMOUS_PHASES:
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


# Modurile in care supervizorul considera ca vehiculul nu mai e in segmentul
# autonom. Expus ca sa il poata folosi si aplicatia.
HANDBACK_MODES = (MODE_BRAKE, MODE_RTL, MODE_LAND)
