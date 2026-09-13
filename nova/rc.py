#!/usr/bin/env python3
"""
Detectia de override pe manse (15.3.1, 15.1.7).

In GUIDED, ArduPilot **ignora** intrarile de mansa. Companion-ul trebuie sa
le detecteze singur din `RC_CHANNELS` si sa comande schimbarea de mod.

Bugetul de latenta e 250 ms (15.3.1). Cum se cheltuie:

    streaming RC_CHANNELS la 50 Hz         20 ms   (implicit 10 Hz = 100 ms!)
    confirmare de persistenta              100 ms  (OVERRIDE_HOLD_S)
    bucla companion-ului + DO_SET_MODE     ~5 ms
    ---------------------------------------------
    total                                  ~125 ms

Ramane marja. Cifra reala se masoara, nu se afirma - vezi
`OverrideMonitor.latency_s` si testul din runda B.

**Calea robusta de siguranta ramane comutatorul de abort mapat direct pe un
mod de zbor prin `FLTMODE_CH`**: acolo comutarea o face FC-ul si nu trece
prin Pi deloc. Detectia de aici e necesara pentru 15.1.7 (pilotul trebuie sa
poata prelua oricand), dar nu e singurul strat.
"""

import time

# --- PRAGURI. Se transcriu in Safety Case. --------------------------------

#: Fluctuatia tipica a unei manse in repaus, masurata. PROVIZORIU pana la
#: rularea lui tools/calibrate_sticks.py pe emitatorul de concurs.
STICK_NOISE_PWM = 30

#: Pragul de declansare. ~2.7x zgomotul: destul de sus ca zgomotul sa nu
#: treaca, destul de jos ca o miscare reala sa fie prinsa imediat.
#: Se seteaza la cel putin 3*sigma peste maximul observat la calibrare.
STICK_DEADBAND_PWM = 80

#: Depasire sustinuta, nu un varf izolat. O miscare reala de mansa dureaza
#: mult peste 100 ms; un spike de zgomot, nu.
OVERRIDE_HOLD_S = 0.1

#: Cat poate lipsi fluxul RC inainte sa nu ne mai putem pronunta.
RC_STALE_S = 0.5

#: Canalele supravegheate: roll, pitch, throttle, yaw (indici in RC_CHANNELS).
STICK_CHANNELS = (0, 1, 2, 3)

#: Axele care se auto-centreaza. Pentru ele, "in neutru" inseamna "aproape de
#: trim" si comparatia cu RCx_TRIM are sens.
SELF_CENTERING_IDX = (0, 1, 3)

#: Throttle-ul NU se auto-centreaza pe un emitator real: pilotul il tine
#: acolo unde ii trebuie pentru hover, iar RC3_TRIM e adesea la capatul de
#: jos al cursei. A-l compara cu trim-ul ar refuza un handover perfect normal.
#: Pentru el, "in neutru" inseamna "nemiscat" - amplitudinea pe fereastra de
#: asezare sub deadband.
THROTTLE_IDX = 2

#: Parametrii de trim ai FC-ului. NU presupune 1500: mansele au offset.
TRIM_PARAMS = ('RC1_TRIM', 'RC2_TRIM', 'RC3_TRIM', 'RC4_TRIM')

#: Fereastra de asezare la handover. Pilotul comuta AUX, apoi are nevoie de
#: timp sa ia mainile de pe manse; arcurile de revenire produc un tranzitoriu
#: care ar declansa override fals imediat dupa activare, anuland incercarea.
HANDOVER_SETTLE_S = 1.0


class OverrideMonitor:
    """Compara canalele 1-4 fata de o referinta de neutru memorata **la
    momentul validarii handover-ului**, nu fata de centrul teoretic.

    De ce nu 1500: mansele au offset de trim, iar throttle-ul nu se
    auto-centreaza pe toate emitatoarele. Referinta corecta e ce raporta
    emitatorul chiar atunci, dupa ce s-au asezat arcurile.

        ov = OverrideMonitor(vehicle)
        ov.begin_settle(now)          # la comutarea AUX
        ...
        ov.capture_neutral(now)       # la validare, dupa asezare
        ...
        ov.update(now)                # la viteza buclei -> True daca override
    """

    def __init__(self, vehicle, deadband_pwm=STICK_DEADBAND_PWM,
                 hold_s=OVERRIDE_HOLD_S, settle_s=HANDOVER_SETTLE_S):
        self.v = vehicle
        self.deadband_pwm = deadband_pwm
        self.hold_s = hold_s
        self.settle_s = settle_s

        self.neutral = None
        self.settle_until = None
        self.trims = None

        self.exceed_since = None      # prima depasire a pragului
        self._settle_min = None       # min/max pe fereastra de asezare
        self._settle_max = None
        self.triggered = False
        self.trigger_t = None
        self.latency_s = None         # prima depasire -> mod confirmat de FC
        self.peak_deviation = 0

    # -- referinta de neutru ----------------------------------------------
    def request_trims(self):
        """Cere RC1_TRIM..RC4_TRIM. Valorile ajung in vehicle.params."""
        for name in TRIM_PARAMS:
            self.v.request_param(name)

    def trims_available(self):
        return all(n in self.v.params for n in TRIM_PARAMS)

    def load_trims(self):
        if not self.trims_available():
            return None
        self.trims = tuple(int(self.v.params[n]) for n in TRIM_PARAMS)
        return self.trims

    def begin_settle(self, now):
        """La comutarea AUX: porneste fereastra de asezare."""
        self.settle_until = now + self.settle_s
        self.neutral = None
        self._settle_min = None
        self._settle_max = None
        self.exceed_since = None
        self.triggered = False
        self.trigger_t = None
        self.latency_s = None
        self.peak_deviation = 0

    def sample_settle(self, now):
        """Urmareste amplitudinea fiecarui canal pe fereastra de asezare.
        Pentru throttle e singurul criteriu care are sens (vezi
        THROTTLE_IDX)."""
        if self.v.rc is None:
            return
        vals = [self.v.rc[c] for c in STICK_CHANNELS]
        if self._settle_min is None:
            self._settle_min = list(vals)
            self._settle_max = list(vals)
            return
        for i, val in enumerate(vals):
            self._settle_min[i] = min(self._settle_min[i], val)
            self._settle_max[i] = max(self._settle_max[i], val)

    def settle_span(self, idx):
        """Amplitudinea canalului `idx` pe fereastra de asezare, sau None."""
        if self._settle_min is None:
            return None
        return self._settle_max[idx] - self._settle_min[idx]

    def settled(self, now):
        return self.settle_until is not None and now >= self.settle_until

    def capture_neutral(self, now):
        """Memoreaza pozitia curenta ca referinta. De apelat DUPA asezare."""
        if self.v.rc is None:
            return None
        self.neutral = tuple(self.v.rc[i] for i in STICK_CHANNELS)
        self.exceed_since = None
        return self.neutral

    # -- masurare ----------------------------------------------------------
    def deviation(self):
        """Cea mai mare abatere fata de neutru, pe canalele 1-4, in PWM.
        None daca nu avem referinta sau fluxul RC e vechi."""
        if self.neutral is None or self.v.rc is None:
            return None
        return max(abs(self.v.rc[c] - self.neutral[i])
                   for i, c in enumerate(STICK_CHANNELS))

    def rc_stale(self, now):
        return self.v.rc_t is None or (now - self.v.rc_t) > RC_STALE_S

    def sticks_neutral(self, tolerance_pwm=None):
        """Pentru HANDOVER_CHECK: sunt mansele libere?

        Doua criterii diferite, pentru ca axele sunt diferite:

        - roll / pitch / yaw se auto-centreaza, deci "liber" = aproape de
          RCx_TRIM citit din FC (nu de 1500 - mansele au offset);
        - throttle nu se auto-centreaza. Pe un emitator real pilotul il tine
          la mijlocul cursei pentru hover, iar RC3_TRIM e adesea la capatul
          de jos. Comparatia cu trim-ul ar refuza un handover perfect normal,
          deci pentru el criteriul e "nemiscat pe fereastra de asezare".

        Prima varianta compara toate patru canalele cu trim-ul si a refuzat
        primul handover din SITL, cu throttle la 500 PWM de RC3_TRIM."""
        tol = self.deadband_pwm if tolerance_pwm is None else tolerance_pwm
        if self.v.rc is None:
            return None, 'fara flux RC'
        trims = self.trims or self.load_trims()
        if trims is None:
            return None, 'RC*_TRIM necitit'

        for i in SELF_CENTERING_IDX:
            c = STICK_CHANNELS[i]
            d = abs(self.v.rc[c] - trims[i])
            if d > tol:
                return False, (f"canalul {c + 1} la {d} PWM de trim "
                               f"(prag {tol})")

        span = self.settle_span(THROTTLE_IDX)
        if span is None:
            return None, 'fereastra de asezare neesantionata'
        if span > tol:
            return False, (f"throttle-ul s-a miscat {span} PWM in fereastra "
                           f"de asezare (prag {tol})")
        return True, ''

    # -- bucla -------------------------------------------------------------
    def update(self, now=None):
        """True daca s-a detectat override. Odata declansat, ramane True."""
        now = now if now is not None else time.monotonic()
        if self.triggered:
            return True
        if self.neutral is None or self.rc_stale(now):
            return False

        dev = self.deviation()
        if dev is None:
            return False
        self.peak_deviation = max(self.peak_deviation, dev)

        if dev <= self.deadband_pwm:
            self.exceed_since = None
            return False

        if self.exceed_since is None:
            self.exceed_since = now
        if now - self.exceed_since >= self.hold_s:
            self.triggered = True
            self.trigger_t = now
            return True
        return False

    def note_mode_confirmed(self, now):
        """Inchide masuratoarea de latenta 15.3.1: de la PRIMA depasire de
        prag pana la modul confirmat de FC. Prima depasire, nu declansarea -
        cele 100 ms de confirmare fac parte din buget."""
        if self.latency_s is None and self.exceed_since is not None:
            self.latency_s = now - self.exceed_since
        return self.latency_s

    def status(self):
        dev = self.deviation()
        if self.neutral is None:
            return 'override: fara referinta de neutru'
        if self.triggered:
            lat = ('-' if self.latency_s is None
                   else f"{self.latency_s * 1000:.0f} ms")
            return f"override DECLANSAT (latenta {lat})"
        return (f"override: abatere {dev} PWM "
                f"(prag {self.deadband_pwm}, varf {self.peak_deviation})")
