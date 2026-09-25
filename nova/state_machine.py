#!/usr/bin/env python3
"""
Masina de stari a segmentului autonom (sectiunea 8 din CLAUDE.md).

Consuma Detection, comanda vehiculul. Nu stie nimic despre pixeli, camere
sau despre cum au fost obtinute detectiile: pe desktop vin din geometrie
sintetica, pe Raspberry Pi din ArUco + solvePnP, iar acest fisier ramane
neschimbat.

Starile:

    IDLE -> HANDOVER_CHECK -> ACQUIRE -> DESCEND_TRACK -> SCORING_CAPTURE
         -> FINAL_DESCENT -> TOUCHDOWN_CONFIRM -> ASCENT -> HANDBACK
                \-> REJECT -> IDLE

**Intrarea in segmentul autonom se face DOAR prin poarta de handover**
(nova/handover.py), pe frontul crescator al comutatorului AUX. Nu exista cale
alternativa, deliberat: o a doua intrare care ocoleste validarea ar face
imposibil de demonstrat, la scrutineering, ca activarea s-a produs in
fereastra ceruta de 15.2.3 - si intrebarea evidenta ar fi care dintre cele
doua cai a fost folosita in cursa.

Dupa ACCEPT, companion-ul cere LAND si asteapta confirmarea in ACQUIRE.
Diagrama din §8 spune "ACQUIRE: GUIDED activat"; aceea precede arhitectura A.
Cu A, coborarea se face in LAND cu PLND, deci ACQUIRE confirma LAND.

TOUCHDOWN_CONFIRM si ASCENT implementeaza cerinta 15.2.7 (arhitectura A):
raman in LAND pana la contact, impiedic dezarmarea iesind din LAND, tin
>= 1 s pe sol, apoi urc la >= 5 m deasupra markerului FARA re-armare.

De ce merge (verificat in sursa ArduCopter, vezi 5.6 din CLAUDE.md):
  - ModeLand::gps_run() dezarmeaza neconditionat cand
    "land_complete && spool == GROUND_IDLE". Nu exista parametru de
    intarziere. Singura solutie e sa nu mai fim in LAND in acel moment.
  - Intre land_complete si GROUND_IDLE trece rampa de spool-down
    (MOT_SPOOL_TIME, implicit 0.5 s), deci exista o fereastra reala.
  - In GUIDED, cu land_complete activ, se apeleaza make_safe_ground_handling():
    motoarele coboara la ground idle si NU se dezarmeaza. Mai departe
    dezarmeaza doar auto_disarm_check(), dupa DISARM_DELAY secunde.
  - Mode::do_user_takeoff_U_m() CERE land_complete == true, deci decolarea
    din GUIDED dupa contact e chiar cazul suportat, fara re-armare.

Parametri ArduPilot necesari (toti in config/nova_sitl.parm):
    RNGFND1_TYPE 10 / RNGFND1_ORIENT 25 / RNGFND1_MIN 0.05 / RNGFND1_MAX 30
    RNGFND1_GNDCLR 0.0745
    PLND_ENABLED 1 / PLND_TYPE 1 / PLND_EST_TYPE 1 / PLND_STRICT 2
    PLND_ALT_MIN 0.35
    SURFTRAK_MODE 0
    WPNAV_RFND_USE 0      # altfel urcarea din GUIDED e "deasupra terenului"
    DISARM_DELAY 20       # marja pentru pauza de pe sol
"""

import math
import time
from dataclasses import dataclass

from .vehicle import MODE_GUIDED, MODE_LAND, MODE_RTL

# --- Praguri ale masinii de stari ----------------------------------------
#
# Captura de scoring si trecerea la coborare verticala se decid pe
# INCADRARE (`Detection.fill`: cat din cadru ocupa cutia markerului), nu pe
# latura in pixeli. Motivul e masurat, nu estetic.
#
# Ce trebuie sa incapa in cadru e cutia unui patrat rotit, mai mare cu
# `|cos| + |sin|` - pana la 41% la 45 grade (§5.49). Un prag fix in pixeli
# are deci o marja care se prabuseste exact la rotatiile pe care nu le
# controlam, fiindca rotatia markerului in cadru depinde de capul
# vehiculului la handover, adica de pilot:
#
#   prag    marja pana la pierderea detectiei, la rotatie 0 / 41 grade
#   980 px  de neatins peste ~18 grade (§5.51)
#   800 px  +48% / **-5%**  - o rulare din 10 a picat 8.3.3 (§5.54)
#   700 px  +69% / +8%      - 10/10, dar marja tot se subtiaza cu rotatia
#
# Pe incadrare, marja e aceeasi la orice rotatie, fiindca `fill` e chiar
# marimea care decide daca markerul iese din cadru.
#
# Pierderea detectiei, masurata in Gazebo (fill fata de cadrul real, 1296 px):
#   rotatie  0 grade : 1184 px            -> fill 0.914   (§5.49)
#   rotatie 41 grade :  759 px x 1.410    -> fill 0.826   (§5.54)
#
#: Captura 8.3.3. La 0.62 marja pana la pierdere e >= 1.33x la ORICE rotatie.
#: Mai jos ar fi si mai sigur, dar markerul ar ocupa mai putin din imaginea
#: predata juriului; 0.62 inseamna ~62% din inaltimea cadrului.
SCORING_FILL = 0.62
#: Coborare verticala. > SCORING_FILL prin constructie, deci captura se face
#: INTOTDEAUNA inaintea trecerii - ordonare pe care doua constante
#: independente (una in px, alta in metri) nu o pot garanta. Exact asa s-a
#: pierdut captura in toate cele 10 rulari din §5.51.
FINAL_FILL = 0.72
#: Prag de rezerva, pentru un detector care nu raporteaza incadrarea
#: (`fill is None`). Necunoscut nu inseamna zero: fara incadrare nu se poate
#: sti cat de rotit e markerul, deci se ia cazul cel mai prost.
SCORING_PX = 700
#: Plasa de altitudine, pentru cazul in care semnalul de incadrare nu vine
#: deloc. Sub pragul la care `FINAL_FILL` s-ar atinge la rotatie zero
#: (0.565 m), ca sa nu il preia si sa taie captura.
#:
#: Ce face de fapt FINAL_DESCENT: iese din fazele in care supervizorul cere
#: detectie valida (§8). Deci pragul nu opreste corectii laterale - alea se
#: opresc singure cand markerul iese din cadru - ci decide cand pierderea
#: markerului inceteaza sa mai fie o defectiune.
NO_LATERAL_ALT_M = 0.50

TD_DEBOUNCE_S = 0.20      # cat trebuie mentinute conditiile de contact
TD_TIMEOUT_S = 8.0        # contact -> comanda de urcare
MODE_RETRY_S = 0.15       # reincercare DO_SET_MODE
TAKEOFF_RETRY_S = 1.50    # reincercare NAV_TAKEOFF (peste MOT_SPOOL_TIME)
TAKEOFF_TRIES = 6
ASCENT_TIMEOUT_S = 25.0   # comanda de urcare -> 5 m AGL

# --- Handover (§8, 15.2.3) ------------------------------------------------
#: Canalul AUX pe care pilotul cere segmentul autonom.
AUX_CHANNEL = 7
#: Peste asta consideram comutatorul pe "sus". Comutatoarele de 3 pozitii dau
#: ~1000/1500/2000; 1700 e clar in treapta de sus si departe de mijloc.
AUX_HIGH_PWM = 1700
#: ACQUIRE: cat asteptam ca FC-ul sa confirme LAND dupa ACCEPT.
ACQUIRE_TIMEOUT_S = 3.0
ACQUIRE_RETRY_S = 0.2

#: Fazele in care detectiile ajung pe MAVLink (15.2.3).
#:
#: Lista e POZITIVA, nu o negatie (§5.25): o faza noua nu emite pana cand
#: cineva o adauga aici deliberat. Cu o negatie, ar emite din prima zi.
#:
#: Pana la J5, `on_detection` trimitea `LANDING_TARGET` si `DISTANCE_SENSOR`
#: NECONDITIONAT, in toate starile - inclusiv `IDLE`, adica in tot zborul
#: pilotului. Verificat direct: 5 detectii in IDLE -> 5 LT si 5 DS.
#:
#: `LANDING_TARGET` era inert acolo (PLND_ENABLED e 0 pana la handover,
#: §5.8). `DISTANCE_SENSOR` nu: FC-ul avea telemetru in tot zborul, iar
#: §5.9 arata masurat ca o citire de telemetru schimba
#: `get_alt_above_ground_m()`, deci incetinirea de dinainte de contact - si
#: ca poate face `NAV_TAKEOFF` sa fie respins prin gardul "can't takeoff
#: downwards" (§5.7). Cu `WP_RFND_USE = 0` al doilea efect nu se manifesta,
#: dar primul e independent de orice setare de navigatie.
#:
#: Mai important decat efectul: randul din Compliance Matrix pentru 15.2.3 -
#: "companion-ul nu comanda nimic in afara segmentului autonom" - nu era
#: sustinut de cod. Acum e.
#:
#: De ce ACQUIRE e inauntru: PLND e deja armat, iar estimatorul primeste
#: masuratori inainte ca FC-ul sa confirme LAND. De ce TOUCHDOWN_CONFIRM si
#: ASCENT nu: vehiculul e pe sol si apoi urca, iar acolo un telemetru care
#: raporteaza sub tinta de decolare e exact cazul masurat in §5.9.
EMITTING_PHASES = ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE',
                   'FINAL_DESCENT')

# --- A1: precision landing se armeaza doar pentru segmentul autonom -------
# PLND_ENABLED e consultat la fiecare ciclu de Mode::land_run_normal_or_precland(),
# iar AC_PrecLand::init() creeaza backend-ul dupa PLND_TYPE, nu dupa
# PLND_ENABLED. Deci parametrul se poate comuta in zbor, fara reboot, si
# putem porni cu el pe 0 (vezi config/nova_sitl.parm).
PRECLAND_PARAM = 'PLND_ENABLED'


class State:
    IDLE = 'IDLE'
    HANDOVER_CHECK = 'HANDOVER_CHECK'
    REJECT = 'REJECT'
    ACQUIRE = 'ACQUIRE'
    DESCEND_TRACK = 'DESCEND_TRACK'
    SCORING_CAPTURE = 'SCORING_CAPTURE'
    FINAL_DESCENT = 'FINAL_DESCENT'
    TOUCHDOWN_CONFIRM = 'TOUCHDOWN_CONFIRM'
    ASCENT = 'ASCENT'
    HANDBACK = 'HANDBACK'
    ABORT = 'ABORT'


@dataclass
class SequenceConfig:
    """Tot ce se regleaza din linia de comanda, intr-un singur loc."""
    do_ascent: bool = True          # 15.2.7; False = comportamentul Fazei 1
    ascent_m: float = 5.0           # inaltime ceruta deasupra markerului
    ascent_margin: float = 0.3      # marja peste tinta la comanda de takeoff
    hold_s: float = 1.2             # pauza pe sol dupa contact (regulament >= 1.0)
    touchdown_alt: float = 0.20     # prag de altitudine pentru contact
    touchdown_vz: float = 0.12      # viteza verticala reziduala acceptata
    conv: int = 0                   # conventie axe LANDING_TARGET (5.1)
    aux_channel: int = AUX_CHANNEL  # canalul de cerere a handover-ului
    aux_high_pwm: int = AUX_HIGH_PWM
    send_range: bool = True         # trimite DISTANCE_SENSOR din detectii
    manage_precland: bool = True    # A1: armeaza/dezarmeaza PLND_ENABLED
    scoring_fill: float = SCORING_FILL      # captura 8.3.3, pe incadrare
    final_fill: float = FINAL_FILL          # trecere la coborare verticala
    scoring_px: float = SCORING_PX          # rezerva, cand fill lipseste
    no_lateral_alt_m: float = NO_LATERAL_ALT_M


class LandingStateMachine:
    """Consuma detectii, comanda vehiculul.

        sm = LandingStateMachine(vehicle, SequenceConfig(...))
        ...
        sm.on_detection(det)   # cand detectorul publica ceva
        sm.update(now)         # la viteza buclei

    on_event(nume, info) e optional si primeste evenimentele de interes
    pentru aplicatie: 'state', 'scoring_capture', 'touchdown', 'ascent_done',
    'abort', 'disarm_early'. Pe vehiculul real, 'scoring_capture' e cirligul
    de care se leaga captura full-res (8.3.3).
    """

    def __init__(self, vehicle, cfg=None, on_event=None, verbose=True,
                 gate=None):
        self.v = vehicle
        self.cfg = cfg or SequenceConfig()
        self.on_event = on_event
        self.verbose = verbose
        #: Poarta de handover (nova/handover.py). E SINGURA cale de intrare in
        #: segmentul autonom: fara ea nu exista dovada ca activarea s-a facut
        #: in fereastra ceruta de 15.2.3, si nu exista cale de refuz.
        self.gate = gate
        self._aux_was_high = False

        # Tot ce tine de timp foloseste ceasul injectat prin update() /
        # on_detection(), niciodata time.monotonic() direct din interiorul
        # logicii: asa masina de stari se poate rula si in timp accelerat,
        # fara SITL.
        self.now = time.monotonic()
        self.state = State.IDLE
        self.state_since = self.now

        self.last_det = None
        self.last_det_rx = 0.0
        self.contact_since = None
        self.t_contact = None
        self.z_touchdown = None
        self.rel_alt_touchdown = None
        self.scoring_shot = None      # (alt, marker_px) la captura

        self.last_mode_req = 0.0
        self.last_takeoff_req = 0.0
        self.takeoff_tries = 0
        self.takeoff_alt_cmd = None
        self.was_armed = False
        self.prev_mode = None
        self.acquire_req_t = 0.0
        self.precland_on = None       # None = necunoscut, nu s-a comandat inca

    # -- evenimente --------------------------------------------------------
    def _emit(self, name, **info):
        if self.on_event:
            self.on_event(name, info)

    def set_state(self, new, note=''):
        if new == self.state:
            return
        if self.verbose:
            print(f"  >> {self.state} -> {new}" + (f"   ({note})" if note else ""))
        self._emit('state', old=self.state, new=new, note=note)
        self.state = new
        self.state_since = self.now

    def reset_sequence(self, note=''):
        # Iesirea din secventa (dezarmare, preluare de pilot, abort) lasa
        # intotdeauna PLND pe 0, ca un RTL sau un LAND de urgenta de mai
        # tarziu sa nu fie atras de marker.
        self.set_precland(False)
        if self.gate is not None:
            self.gate.reset()
        self.set_state(State.IDLE, note)
        self.scoring_shot = None
        self.contact_since = None
        self.t_contact = None
        self.z_touchdown = None
        self.rel_alt_touchdown = None
        self.takeoff_tries = 0
        self.takeoff_alt_cmd = None

    # -- incadrare: cat din cadru ocupa markerul ---------------------------
    def _incadrare(self, det, prag, prag_px):
        """True cand markerul umple cel putin `prag` din cadru.

        `det.fill` e masura buna: cutia markerului fata de cadru, deci
        include rotatia. Cand detectorul nu o raporteaza (`None`), se cade
        pe latura in pixeli - conservator prin constructie, fiindca fara
        incadrare nu se poate sti cat de rotit e markerul.

        Necunoscut nu inseamna zero: `fill = None` NU trebuie sa iasa
        "markerul e mic", adica exact raspunsul gresit cand e pe cale sa
        iasa din cadru (§5.53)."""
        if det is None:
            return False
        if det.fill is not None:
            return det.fill >= prag
        return det.marker_px > prag_px

    def _gata_de_captura(self, det):
        return self._incadrare(det, self.cfg.scoring_fill, self.cfg.scoring_px)

    def _gata_de_verticala(self, alt):
        """(da, motiv) - cand se iese din fazele supravegheate de detectie.

        Doua criterii, in ordinea increderii: incadrarea masurata, apoi
        plasa de altitudine. A doua exista pentru cazul in care semnalul de
        incadrare nu mai vine deloc."""
        det = self.last_det
        if det is not None and det.fill is not None:
            if det.fill >= self.cfg.final_fill:
                return True, f"incadrare {det.fill:.2f}"
        elif det is not None and det.marker_px > self.cfg.scoring_px:
            # fara incadrare: pragul in pixeli e tot ce avem
            return True, f"{det.marker_px:.0f} px"
        if alt < self.cfg.no_lateral_alt_m:
            return True, f"alt {alt:.2f} m (plasa de altitudine)"
        return False, ''

    # -- intrare: detectii -------------------------------------------------
    def on_detection(self, det, now=None):
        """Singura cale prin care viziunea intra in control."""
        now = now if now is not None else time.monotonic()
        self.now = now
        self.last_det = det
        self.last_det_rx = now

        # Captura de scoring: declansata pe dimensiunea markerului in pixeli,
        # nu pe altitudine (8.3.3). Captura propriu-zisa o face detectorul,
        # prin on_event; evenimentul poarta det.t, adica momentul CAPTURARII
        # cadrului, nu cel al deciziei. De aceea imaginea se scoate din ring
        # buffer-ul de ~2 s dupa timestamp: asa latenta de procesare nu muta
        # cadrul predat juriului mai jos decat trebuie.
        #
        # Conditia pe stare nu e decorativa: fara ea, orice trecere joasa
        # peste marker (inclusiv coborarea unui RTL) declanseaza captura, iar
        # cum reset_sequence sterge scoring_shot se intra intr-un ciclu
        # IDLE <-> SCORING_CAPTURE la fiecare cadru. Vazut in SITL la testul A1.
        if (self.scoring_shot is None
                and self._gata_de_captura(det)
                and self.state in (State.DESCEND_TRACK, State.FINAL_DESCENT)):
            self.scoring_shot = (self.v.alt, det.marker_px)
            self._emit('scoring_capture', alt=self.v.alt,
                       marker_px=det.marker_px, fill=det.fill, t=det.t)
            # Daca am ajuns deja in FINAL_DESCENT (coborare rapida, pragul de
            # 980 px sarit intre doua cadre), captura tot se face, dar starea
            # nu se intoarce inapoi.
            if self.state == State.DESCEND_TRACK:
                self.set_state(State.SCORING_CAPTURE,
                               f"{self.v.alt:.3f} m, {det.marker_px:.0f} px")

        # Detectia se inregistreaza INTOTDEAUNA (mai sus): poarta si
        # monitorul de varsta a detectiei depind de ea in orice stare. Ce se
        # filtreaza e EMISIA pe MAVLink, adica singurul lucru care ajunge la
        # FC in afara segmentului autonom.
        if self.state not in EMITTING_PHASES:
            return
        angle_x, angle_y = self._apply_conv(det.angle_x, det.angle_y)
        self.v.send_landing_target(angle_x, angle_y, det.distance_m)
        if self.cfg.send_range:
            self.v.send_distance(det.range_m)

    def _apply_conv(self, angle_x, angle_y):
        """5.1: conventia de axe difera intre versiuni ArduPilot. Miscarea in
        elipsa = corectie perpendiculara pe eroare = rotatie de 90 grade.
        Rotatia sta aici, nu in detector, ca sa se aplice identic datelor
        sintetice si celor reale din solvePnP."""
        if self.cfg.conv == 1:
            return -angle_y, angle_x
        if self.cfg.conv == 2:
            return angle_y, -angle_x
        if self.cfg.conv == 3:
            return -angle_x, -angle_y
        return angle_x, angle_y

    # -- A1: precision landing armat doar cat tine segmentul autonom ------
    def set_precland(self, on, now=None):
        """PLND_ENABLED 1 doar pe durata segmentului autonom.

        Cu el lasat pe 1 permanent, precision landing e activ si in faza de
        coborare a RTL-ului (mode_rtl.cpp apeleaza acelasi
        land_run_normal_or_precland) si in orice LAND comandat manual. Un RTL
        pornit pentru ca ceva a mers prost ar ateriza pe marker in loc de
        home - exact unde nu vrei sa fie vehiculul (15.2.4)."""
        if not self.cfg.manage_precland:
            return
        on = bool(on)
        if self.precland_on == on:
            return
        self.precland_on = on
        self.v.set_param(PRECLAND_PARAM, 1 if on else 0, now)

    def arm_precland(self, now=None):
        """Punctul in care segmentul autonom preia. Cand HANDOVER_CHECK va
        exista (grupul B), apelul se muta acolo."""
        self.set_precland(True, now)

    def abort_to_rtl(self, reason, now=None):
        """Singura cale catre RTL. Dezactiveaza intai PLND: RTL urca la
        RTL_ALT inainte sa coboare, deci parametrul are timp sa se propage."""
        now = now if now is not None else self.now
        self.now = now
        self.set_precland(False, now)
        self.v.request_mode(MODE_RTL)
        self.set_state(State.ABORT, f"RTL: {reason}")
        self._emit('abort', reason=reason, action='RTL')

    # -- bucla -------------------------------------------------------------
    def update(self, now=None):
        """De apelat la viteza buclei, nu la 20 Hz: iesirea din LAND trebuie
        sa incapa in fereastra de spool-down de ~0.5 s."""
        now = now if now is not None else time.monotonic()
        self.now = now

        if self.was_armed and not self.v.armed and self.v.have_pos:
            self._on_disarm()
        self.was_armed = self.v.armed

        if not self.v.have_pos:
            return

        self._supervise(now)

    def _supervise(self, now):
        alt = self.v.alt

        # Cererea de handover e FRONTUL CRESCATOR al comutatorului AUX, nu
        # simpla lui stare: altfel un comutator lasat sus ar reporni secventa
        # imediat dupa orice iesire din ea.
        aux_high = self.aux_high()
        aux_rising = aux_high and not self._aux_was_high
        self._aux_was_high = aux_high
        self.prev_mode = self.v.mode

        if not self.v.armed:
            if self.state != State.IDLE:
                self.reset_sequence('dezarmat')
            # Pe teren, comutatorul ridicat inainte de armare parea "nu face
            # nimic" si trimitea cautarea spre RC. Se spune, si se aminteste
            # ca frontul s-a consumat: in aer trebuie coborat si ridicat.
            if aux_rising:
                if self.verbose:
                    print("  !! AUX sus IGNORAT: vehiculul e dezarmat. "
                          "In aer, coboara si ridica din nou comutatorul.")
                self._emit('aux_ignored_disarmed')
            return

        # HANDBACK / ABORT: se iese doar printr-o cerere noua de handover.
        # Fara asta, al doilea tur din acelasi slot de 15 minute nu ar mai
        # porni niciodata segmentul autonom.
        if self.state in (State.HANDBACK, State.ABORT):
            if aux_rising:
                self.reset_sequence('cerere noua de handover')
                self._request_handover(now, alt)
            return

        # REJECT ramane afisat pana cand pilotul lasa comutatorul jos. Asa
        # vede refuzul si trebuie sa reia deliberat, nu intra intr-un ciclu
        # de reincercari la fiecare cadru.
        if self.state == State.REJECT:
            if not aux_high:
                self.reset_sequence('AUX eliberat dupa refuz')
            return

        if self.state == State.IDLE:
            if aux_rising:
                self._request_handover(now, alt)
            return

        if self.state == State.HANDOVER_CHECK:
            self._run_handover_check(now, alt, aux_high)
            return

        if self.state == State.ACQUIRE:
            self._run_acquire(now, alt)
            return

        # Starile de coborare presupun ca ArduPilot e inca in LAND. Daca
        # pilotul a preluat (sau Safety Supervisor-ul a comandat altceva),
        # secventa autonoma s-a incheiat.
        if self.state in (State.DESCEND_TRACK, State.SCORING_CAPTURE,
                          State.FINAL_DESCENT) and self.v.mode != MODE_LAND:
            self.reset_sequence(f"mod schimbat ({self.v.mode})")
            return

        if self.state in (State.DESCEND_TRACK, State.SCORING_CAPTURE):
            if self._contact_detected(now, alt) or self.v.on_ground():
                self._enter_touchdown(now, alt)
                return
            # Markerul e pe cale sa iasa din cadru (5.2, 5.49): de aici
            # incolo pierderea lui nu mai e o defectiune, deci se iese din
            # fazele supravegheate de monitorul de detectie (§8).
            gata, motiv = self._gata_de_verticala(alt)
            if gata:
                self.set_state(State.FINAL_DESCENT, motiv)

        elif self.state == State.FINAL_DESCENT:
            if self._contact_detected(now, alt) or self.v.on_ground():
                self._enter_touchdown(now, alt)

        elif self.state == State.TOUCHDOWN_CONFIRM:
            self._run_touchdown(now)

        elif self.state == State.ASCENT:
            self._run_ascent(now, alt)

    # -- handover (§8, 15.2.3) ---------------------------------------------
    def aux_high(self):
        ch = self.cfg.aux_channel
        if self.v.rc is None or len(self.v.rc) < ch:
            return False
        return self.v.rc[ch - 1] >= self.cfg.aux_high_pwm

    def marker_offset_m(self):
        """Distanta orizontala pana la marker, din ultima detectie.

        DOAR din camera: distanta 3D si distanta pe axa optica pana la planul
        markerului vin din ACELASI solvePnP. Prima varianta scadea altitudinea
        barometrica din distanta camerei - doua instrumente diferite intr-o
        formula prost conditionata langa verticala: in zborul b4 (§5.63)
        barometrul arata 6.15 m, camera 8.86 m pana la plan, iar poarta a
        calculat 6.6 m lateral si a refuzat cu drona practic DEASUPRA
        markerului (1.7 m). 19 din 25 de refuzuri din zborurile din
        25.09.2026 au fost pe acest motiv.

        None daca nu avem detectie."""
        if self.last_det is None:
            return None
        d = self.last_det.distance_m
        h = getattr(self.last_det, 'range_m', None)
        if h is None:
            h = self.v.alt              # detector fara range: vechiul calcul
        return math.sqrt(max(0.0, d * d - h * h))

    def detection_age(self, now):
        if self.last_det is None:
            return None
        return now - self.last_det.t

    def _request_handover(self, now, alt):
        if self.gate is None:
            self._abort('handover cerut fara poarta configurata')
            return
        self.gate.on_aux_requested(now)
        self.set_state(State.HANDOVER_CHECK, f"AUX sus, alt {alt:.2f} m")

    def _run_handover_check(self, now, alt, aux_high):
        if not aux_high:
            self.reset_sequence('AUX eliberat inainte de validare')
            return
        ok, why = self.gate.check(now, self.marker_offset_m(),
                                  self.detection_age(now))
        if ok is None:
            return                      # fereastra de asezare
        if not ok:
            self._emit('handover_reject', reason=why, alt=alt)
            self.set_state(State.REJECT, why)
            return
        # ACCEPT. Abia acum armam PLND si cerem LAND; pana la confirmarea
        # modului stam in ACQUIRE, altfel garda "mode != LAND" din starile de
        # coborare ne-ar scoate imediat din secventa.
        self._emit('handover_accept', alt=alt,
                   dist=self.marker_offset_m(),
                   neutral=self.gate.ov.neutral)
        self.arm_precland(now)
        self.acquire_req_t = now
        self.v.request_mode(MODE_LAND)
        self.set_state(State.ACQUIRE, f"alt {alt:.2f} m")

    def _run_acquire(self, now, alt):
        if self.v.mode == MODE_LAND:
            self.set_state(State.DESCEND_TRACK, f"alt {alt:.2f} m")
            return
        if now - self.acquire_req_t > ACQUIRE_RETRY_S:
            self.acquire_req_t = now
            self.v.request_mode(MODE_LAND)
        if now - self.state_since > ACQUIRE_TIMEOUT_S:
            self._abort('FC nu a confirmat LAND dupa ACCEPT')

    # -- contact si 15.2.7 -------------------------------------------------
    def _contact_detected(self, now, alt):
        """Detectie proprie de contact, independenta de ArduPilot: altitudine
        sub prag si viteza verticala stinsa, mentinute TD_DEBOUNCE_S."""
        if alt > self.cfg.touchdown_alt or abs(self.v.vz) > self.cfg.touchdown_vz:
            self.contact_since = None
            return False
        if self.contact_since is None:
            self.contact_since = now
        return (now - self.contact_since) >= TD_DEBOUNCE_S

    def _enter_touchdown(self, now, alt):
        self.t_contact = now
        self.z_touchdown = self.v.z
        self.rel_alt_touchdown = (self.v.rel_alt
                                  if self.v.rel_alt is not None else alt)
        self.set_state(State.TOUCHDOWN_CONFIRM, f"alt {alt:.3f} m")
        self._emit('touchdown', alt=alt, t=now,
                   scoring_shot=self.scoring_shot)

    def _run_touchdown(self, now):
        """15.2.7 pas 1: nu lasa ArduPilot sa dezarmeze, apoi tine pe sol."""
        if not self.cfg.do_ascent:
            return

        if self.v.mode != MODE_GUIDED:
            # Iesim din LAND exact cand land_complete devine adevarat.
            # Mai devreme ar fi gresit: GUIDED fara land_complete tine
            # pozitia activ si ar ridica vehiculul de pe sol.
            if self.v.on_ground():
                if now - self.last_mode_req > MODE_RETRY_S:
                    self.last_mode_req = now
                    self.v.request_mode(MODE_GUIDED)
            if now - self.state_since > TD_TIMEOUT_S:
                self._abort('nu am reusit comutarea in GUIDED')
            return

        # Stabilizare >= 1 s de la contact (cerinta 15.2.7).
        if now - self.t_contact < self.cfg.hold_s:
            return

        if not self.v.on_ground():
            # ArduPilot cere land_complete pentru user takeoff.
            if now - self.state_since > TD_TIMEOUT_S:
                self._abort('land_complete nu s-a confirmat')
            return

        # Altitudinea comandata e deasupra HOME, nu deasupra markerului (5.7):
        # adunam relative_alt masurat la contact.
        alt_cmd = (self.rel_alt_touchdown + self.cfg.ascent_m
                   + self.cfg.ascent_margin)
        self.takeoff_alt_cmd = alt_cmd
        self.takeoff_tries = 1
        self.last_takeoff_req = now
        self.v.send_takeoff(alt_cmd)
        self.set_state(State.ASCENT,
                       f"pauza {now - self.t_contact:.2f} s, "
                       f"takeoff la {alt_cmd:.2f} m peste home")

    def _run_ascent(self, now, alt):
        """15.2.7 pas 2: urcare la >= 5 m deasupra markerului."""
        agl = alt - (-self.z_touchdown)

        if agl >= self.cfg.ascent_m:
            # Segmentul autonom s-a incheiat: PLND inapoi pe 0 pentru restul
            # turului, altfel un RTL de mai tarziu ar fi atras de marker.
            self.set_precland(False, now)
            self.set_state(State.HANDBACK, f"{agl:.2f} m peste marker")
            self._emit('ascent_done', agl=agl, t=now,
                       hold_s=self.cfg.hold_s,
                       contact_to_agl_s=now - self.t_contact)
            return

        # Daca ArduPilot inca se considera pe sol si nu am urcat, comanda a
        # fost refuzata (sau pierduta): reincearca. Cand decolarea chiar a
        # pornit, land_complete cade si nu mai trimitem nimic.
        if (self.v.on_ground() and agl < 0.30
                and self.takeoff_tries < TAKEOFF_TRIES
                and now - self.last_takeoff_req > TAKEOFF_RETRY_S):
            self.takeoff_tries += 1
            self.last_takeoff_req = now
            self.v.send_takeoff(self.takeoff_alt_cmd)
            if self.verbose:
                print(f"  .. reincerc NAV_TAKEOFF ({self.takeoff_tries})")

        if now - self.state_since > ASCENT_TIMEOUT_S:
            self._abort(f"urcare blocata la {agl:.2f} m")

    def _abort(self, reason):
        self.set_precland(False)
        self.set_state(State.ABORT, reason)
        self._emit('abort', reason=reason, action=None)

    def _on_disarm(self):
        if self.state in (State.TOUCHDOWN_CONFIRM, State.ASCENT):
            if self.verbose:
                print("\n!! DEZARMAT inainte de urcare: 15.2.7 NEINDEPLINIT\n")
            self._emit('disarm_early', state=self.state)
        self.reset_sequence('dezarmat')

    # -- raportare ---------------------------------------------------------
    def status_line(self):
        det = self.last_det
        fresh = det is not None and (self.now - self.last_det_rx) < 0.5
        vis = (f"marker {det.marker_px:6.1f} px  range {det.range_m:5.2f} m"
               if fresh else "MARKER PIERDUT")
        # Canalul de handover, valoarea bruta si SUS/jos. Cererea se face pe
        # frontul crescator, deci o singura linie de tranzitie la ridicare;
        # asta arata pe fiecare linie de stare daca comutatorul chiar ajunge
        # la Pi - un canal gresit in config sau un RCn_OPTION care il
        # ocupa se vad aici, nu doar prin lipsa unei tranzitii.
        ch = self.cfg.aux_channel
        rc = getattr(self.v, 'rc', None)
        if rc is not None and len(rc) >= ch:
            aux = (f"AUX{ch} {rc[ch - 1]:4d} "
                   f"{'SUS' if self.aux_high() else 'jos'}")
        else:
            aux = f"AUX{ch} -"
        return (f"[{self.state:<17}] alt {self.v.alt:6.2f} m | {vis} | "
                f"{aux} | LT {self.v.n_lt} DS {self.v.n_ds}")


def run_loop(vehicle, detector, sm, supervisor=None, on_status=None,
             status_s=2.0, sleep_s=0.002, authority=None):
    """Bucla comuna sim / Raspberry Pi.

    detector.poll(now) intoarce zero sau mai multe Detection. Restul e
    identic indiferent daca pixelii vin din geometrie sintetica sau din
    picamera2.

    Supervizorul de siguranta ruleaza INAINTEA masinii de stari si primeste
    varsta detectiei calculata aici, nu de la masina de stari: asa nu depinde
    de starea interna a controlului (15.2.10). Varsta se masoara fata de
    det.t, adica momentul CAPTURARII cadrului, nu al sosirii lui - varsta
    informatiei, nu a mesajului.
    """
    last_status = time.monotonic()
    last_det_t = None
    while True:
        vehicle.pump()
        now = time.monotonic()
        # H1: reconectarea se incearca din bucla, nu dintr-un fir separat, si
        # nu blocheaza niciodata - o singura incercare per apel, distantata
        # prin backoff. Detectorul si supervizorul continua intre incercari;
        # doar emisia catre FC e suspendata (Vehicle.link_healthy).
        if hasattr(vehicle, 'check_link'):
            vehicle.check_link(now)
        vehicle.update_params(now)

        dets = detector.poll(now)
        for det in dets:
            last_det_t = det.t if last_det_t is None else max(last_det_t, det.t)

        if supervisor is not None:
            age = None if last_det_t is None else (now - last_det_t)
            supervisor.update(now, age, sm.state,
                              miss_streak=getattr(detector, 'miss_streak', None))

        # Modularea de autoritate ruleaza DUPA supervizor si INAINTEA masinii
        # de stari, ca si el, si se armeaza/elibereaza tot din faza (§5.14).
        # E deliberat sub supervizor in ordine: daca supervizorul tocmai a
        # comandat BRAKE, faza se schimba si modularea incepe restaurarea in
        # acelasi ciclu, nu in urmatorul.
        if authority is not None:
            lat = None
            st = getattr(detector, 'stats', None)
            if st is not None:
                try:
                    p50 = st().get('latency_p50_ms')
                    lat = None if p50 is None else p50 / 1000.0
                except Exception:                            # noqa: BLE001
                    lat = None
            authority.update(now, vehicle.alt, sm.state, latency_s=lat)

        for det in dets:
            sm.on_detection(det, now)

        sm.update(now)

        if on_status and now - last_status > status_s:
            last_status = now
            on_status(now)

        time.sleep(sleep_s)
