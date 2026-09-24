#!/usr/bin/env python3
"""
HANDOVER_CHECK: poarta de intrare in segmentul autonom (§8, 15.3.1 B3.1).

Secventa, si de ce arata asa:

    AUX comutat
       |  1.0 s - fereastra de asezare
       |          Pilotul comuta, apoi are nevoie de timp sa ia mainile de pe
       |          manse. Arcurile de revenire produc un tranzitoriu care ar
       |          declansa override fals imediat dupa activare si ar anula
       |          incercarea - 10 puncte pierdute pentru un artefact mecanic.
       v
    HANDOVER_CHECK - valideaza abia acum
       |- REJECT  vreo mansa in afara neutrului
       |- REJECT  altitudine in afara ferestrei
       |- REJECT  prea departe de marker
       |- REJECT  marker nedetectat
       \\- ACCEPT  memoreaza pozitia curenta ca referinta de neutru

**Referinta de neutru se memoreaza la ACCEPT, nu la comutarea AUX.** Altfel
am memora tocmai tranzitoriul de arc drept "neutru", si orice revenire
ulterioara la centrul real ar arata ca o miscare de mansa.

**Refuzul trebuie semnalizat pilotului**, cu motiv. Un handover refuzat costa
cateva secunde; o incercare anulata in zbor costa 10 puncte. Semnalizarea se
face prin `on_reject` - pe vehiculul real buzzer/LED, in simulare STATUSTEXT.
"""

import time

from . import config as nova_config
from .rc import HANDOVER_SETTLE_S

# --- PRAGURI DE ACCEPTARE. Se transcriu in Safety Case. -------------------

#: Fereastra de altitudine la handover. Plafonul de 12 m e mai strict decat
#: regulamentul (20 m), deliberat: la 20 m markerul are 22 px, prea putin
#: pentru detectie 4x4 fiabila; la 12 m are 37 px.
HANDOVER_ALT_MIN_M = 5.0
HANDOVER_ALT_MAX_M = 12.0

#: Raza in care markerul trebuie sa fie, la handover.
HANDOVER_DIST_MAX_M = 6.5

#: Cat de recenta trebuie sa fie detectia ca sa conteze drept "marker vazut".
HANDOVER_DETECTION_MAX_AGE_S = 0.3


class Reject:
    #: E0. Nu e o conditie de zbor, e o politica: detectorul real nu a trecut
    #: inca validarea offline (E2). Se ridica din config/nova.json, cu commit.
    AUTONOMY_DISABLED = ('autonomie DEZACTIVATA (E0): config/nova.json '
                         'autonomy_enabled=false')
    #: Poarta inchisa FORTAT pentru rularea asta (`nova_pi.py --monitor`,
    #: folosit de pornirea automata). Motiv separat de E0: dupa ce E0 se
    #: deschide, un pilot care ar citi "autonomy_enabled=false" ar merge sa
    #: verifice fisierul, l-ar gasi `true`, si n-ar mai intelege nimic.
    MONITOR = ('MONITOR: pornirea automata nu zboara autonom - porneste '
               'pi/descent_test.sh')
    STICKS = 'mansa in afara neutrului la handover'
    ALTITUDE = 'altitudine in afara ferestrei'
    DISTANCE = 'in afara zonei de 6.5 m'
    NO_MARKER = 'marker nedetectat'
    NO_RC = 'fara flux RC'
    SETTLING = 'inca se asaza'


class HandoverGate:
    """
        gate = HandoverGate(vehicle, override_monitor)
        gate.on_aux_requested(now)                    # la comutarea AUX
        ...
        ok, reason = gate.check(now, dist_to_marker_m, detection_age_s)

    `check()` intoarce (None, Reject.SETTLING) cat timp fereastra de asezare e
    deschisa - nu e nici accept, nici refuz, doar "inca nu".
    """

    def __init__(self, vehicle, override_monitor, on_reject=None,
                 autonomy_enabled=None,
                 settle_s=HANDOVER_SETTLE_S,
                 alt_min_m=HANDOVER_ALT_MIN_M, alt_max_m=HANDOVER_ALT_MAX_M,
                 dist_max_m=HANDOVER_DIST_MAX_M,
                 detection_max_age_s=HANDOVER_DETECTION_MAX_AGE_S,
                 monitor=False):
        self.v = vehicle
        #: Poarta inchisa FORTAT pentru rularea asta, indiferent de E0
        #: (`nova_pi.py --monitor`, folosit de pornirea automata). Parametru
        #: separat de `autonomy_enabled`, nu o valoare a lui: `False` acolo
        #: inseamna deja "E0 inchis", cu motivul lui. Aici poarta ramane
        #: inchisa CHIAR SI cu E0 deschis - altfel pornirea automata ar deveni
        #: o a doua cale spre autonomie, cu alte setari decat proba.
        self.monitor = bool(monitor)
        self.ov = override_monitor
        self.on_reject = on_reject
        #: E0. None = citeste config/nova.json la fiecare cerere de handover
        #: (asa un commit care il activeaza conteaza fara repornire, si logul
        #: arata valoarea vazuta atunci). True/False explicit e pentru teste
        #: si pentru simulare, unde nu exista vehicul de distrus.
        self.autonomy_enabled = autonomy_enabled
        self.settle_s = settle_s
        self.alt_min_m = alt_min_m
        self.alt_max_m = alt_max_m
        self.dist_max_m = dist_max_m
        self.detection_max_age_s = detection_max_age_s

        self.requested_t = None
        self.decided = None          # True = accept, False = reject
        self.reason = ''
        self.rejects = []            # istoricul refuzurilor, pentru log

    # -- ciclu de viata ----------------------------------------------------
    def on_aux_requested(self, now=None):
        """Pilotul a comutat AUX. Porneste fereastra de asezare."""
        now = now if now is not None else time.monotonic()
        self.requested_t = now
        self.decided = None
        self.reason = ''
        self.ov.begin_settle(now)
        self.ov.request_trims()

    def reset(self):
        self.requested_t = None
        self.decided = None
        self.reason = ''

    def _reject(self, now, reason):
        self.decided = False
        self.reason = reason
        self.rejects.append((now, reason))
        if self.on_reject:
            self.on_reject(reason)
        return False, reason

    # -- validare ----------------------------------------------------------
    def check(self, now=None, dist_to_marker_m=None, detection_age_s=None):
        """(None, motiv) cat timp se asaza; (True, '') la accept;
        (False, motiv) la refuz."""
        now = now if now is not None else time.monotonic()
        if self.requested_t is None:
            return None, 'AUX necomutat'
        if self.decided is not None:
            return self.decided, self.reason

        # E0, inaintea oricarei alte conditii si fara sa astepte asezarea:
        # daca autonomia e dezactivata, pilotul afla imediat, nu dupa 1 s.
        if not self._autonomy_allowed():
            motiv = (Reject.MONITOR if self.monitor
                     else Reject.AUTONOMY_DISABLED)
            return self._reject(now, motiv)

        if not self.ov.settled(now):
            # Esantionam continuu: throttle-ul se valideaza pe amplitudinea
            # din fereastra asta, nu pe pozitia absoluta.
            self.ov.sample_settle(now)
            return None, Reject.SETTLING

        # Ordinea conteaza doar pentru mesajul afisat; toate sunt eliminatorii.
        neutral, why = self.ov.sticks_neutral()
        if neutral is None:
            return self._reject(now, f"{Reject.NO_RC} ({why})")
        if not neutral:
            return self._reject(now, f"{Reject.STICKS} - {why}")

        alt = self.v.alt
        if not (self.alt_min_m <= alt <= self.alt_max_m):
            return self._reject(
                now, f"{Reject.ALTITUDE}: {alt:.1f} m "
                     f"(cerut {self.alt_min_m:.0f}-{self.alt_max_m:.0f} m)")

        if dist_to_marker_m is None:
            return self._reject(now, f"{Reject.NO_MARKER} (fara pozitie)")
        if dist_to_marker_m > self.dist_max_m:
            return self._reject(
                now, f"{Reject.DISTANCE}: {dist_to_marker_m:.2f} m")

        if (detection_age_s is None
                or detection_age_s > self.detection_max_age_s):
            age = ('niciodata' if detection_age_s is None
                   else f"{detection_age_s:.2f} s")
            return self._reject(now, f"{Reject.NO_MARKER} (ultima acum {age})")

        # ACCEPT: abia acum memoram neutrul, dupa ce manetele s-au asezat.
        if self.ov.capture_neutral(now) is None:
            return self._reject(now, Reject.NO_RC)
        self.decided = True
        self.reason = ''
        return True, ''

    def _autonomy_allowed(self):
        if self.monitor:
            return False
        if self.autonomy_enabled is None:
            return nova_config.autonomy_enabled()
        return self.autonomy_enabled is True

    def status(self):
        if self.requested_t is None:
            return 'handover: in asteptare'
        if self.decided is None:
            return 'handover: se asaza'
        if self.decided:
            return f"handover: ACCEPTAT, neutru {self.ov.neutral}"
        return f"handover: REFUZAT - {self.reason}"
