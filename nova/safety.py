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

#: 15.2.4 - raza fata de punctul de handover si plafonul deasupra lui.
GEOFENCE_RADIUS_M = 10.0
CEILING_AGL_M = 30.0

#: Rata de coborare peste care ceva e in neregula. Coborarea normala e
#: limitata de LAND_SPD_MS (0.5 m/s) si WP_SPD_DN; 2.0 m/s inseamna ca nu mai
#: comanda controlerul de aterizare. PROVIZORIU.
MAX_DESCENT_RATE_MS = 2.0
DESCENT_RATE_HOLD_S = 0.5

#: Inclinare peste care vehiculul nu mai e intr-o coborare controlata.
MAX_TILT_DEG = 30.0
TILT_HOLD_S = 0.3

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
                 max_tilt_deg=MAX_TILT_DEG):
        self.v = vehicle
        self.on_event = on_event
        self.verbose = verbose

        self.detection_max_age_s = detection_max_age_s
        self.geofence_radius_m = geofence_radius_m
        self.ceiling_agl_m = ceiling_agl_m
        self.max_descent_rate_ms = max_descent_rate_ms
        self.max_tilt_deg = max_tilt_deg

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

        self._descent_since = None
        self._tilt_since = None
        self._mode_req_t = 0.0
        self._mode_req_n = 0
        self._want_mode = None
        self._mode_confirmed = None

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
    def update(self, now=None, detection_age_s=None, phase='IDLE'):
        """De apelat la viteza buclei, inaintea masinii de stari.

        detection_age_s: secunde de la ultima detectie valida, sau None daca
        nu a existat niciuna. NU primeste detectia in sine (15.2.10).
        """
        now = now if now is not None else time.monotonic()

        # Zavorul se verifica INAINTEA armarii, deliberat. Cand supervizorul
        # comanda BRAKE, masina de stari vede ca nu mai e in LAND si isi
        # incheie secventa, iar aplicatia dezarmeaza supervizorul. Daca
        # dezarmarea ar opri si reincercarile de mod, o comanda pierduta ar
        # ramane definitiv nelivrata - exact actiunea de siguranta ar esua
        # tacut. Odata declansata, ducem comanda la capat indiferent de rest.
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
                    self._mon_radius, self._mon_ceiling,
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
        if now - self._mode_req_t < MODE_CONFIRM_S:
            return
        if self._mode_req_n >= MODE_RETRY_MAX:
            if self._mode_req_n == MODE_RETRY_MAX:
                self._mode_req_n += 1
                self._emit(now, 'mode_fail', self.latched,
                           f"FC ramane in {self.v.mode_name()} dupa "
                           f"{MODE_RETRY_MAX} comenzi", 'FAIL')
            return
        self._mode_req_t = now
        self._mode_req_n += 1
        self.v.request_mode(self._want_mode)

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
        if age > self.detection_max_age_s:
            return (Action.BRAKE, 'detection_age',
                    f"ultima detectie acum {age:.2f} s "
                    f"(prag {self.detection_max_age_s:.2f} s)")
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
            return f"SAFETY activ | {self.override.status()}"
        conf = 'confirmat' if self._mode_confirmed else 'NECONFIRMAT'
        return (f"SAFETY {Action.NAMES[self.latched]} "
                f"({self.latched_monitor}), mod {conf}")


# Modurile in care supervizorul considera ca vehiculul nu mai e in segmentul
# autonom. Expus ca sa il poata folosi si aplicatia.
HANDBACK_MODES = (MODE_BRAKE, MODE_RTL, MODE_LAND)
