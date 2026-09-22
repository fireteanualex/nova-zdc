#!/usr/bin/env python3
"""
15.2.5: niciun estimator alimentat cu GNSS in segmentul autonom.

Din momentul activarii autonome, niciun estimator sau filtru care contribuie
la ghidare sau control nu are voie sa primeasca date GNSS. ArduPilot ofera
mecanismul: trei seturi de surse EKF3 (`EK3_SRC1_*`, `EK3_SRC2_*`,
`EK3_SRC3_*`), comutabile in zbor cu `MAV_CMD_SET_EKF_SOURCE_SET`.

    src = EkfSourceManager(vehicle)
    src.update(now, phase)        # se armeaza si elibereaza singur din faza

PREDICATUL DE CONFORMITATE NU E CEL DIN ARDUPILOT

`AP_NavEKF_Source::usingGPS()` verifica:

    POSXY == GPS || POSZ == GPS || VELXY == GPS || VELZ == GPS
    || YAW == GSF

Lipsesc `YAW == GPS` (2) si `YAW == GPS_COMPASS_FALLBACK` (3). Amandoua
alimenteaza estimatorul de cap cu GNSS, si amandoua ar trece de `usingGPS()`.
Pentru 15.2.5 aia nu e o subtilitate academica: un set cu `EK3_SRC2_YAW = 2`
ar fi raportat de firmware ca "fara GPS" si ar incalca regula.

Deci `contine_gnss()` de mai jos e mai strict decat firmware-ul, deliberat.

CE FACE SI CE NU

Comuta setul de surse la intrarea in segment si il pune inapoi la iesire -
handback, abort, dezarmare, sau o faza pe care nimeni nu a prevazut-o
(§5.14: armare din FAZA, nu din tranzitii).

NU comuta daca setul tinta contine GNSS la citirea inapoi. Un `PARAM_SET`
acceptat tacit nu e o garantie (§5.10), iar aici garantia e chiar afirmatia
din Compliance Matrix.
"""

import time

from pymavlink import mavutil

#: Valorile din `AP_NavEKF_Source.h`, verificate in sursa.
SURSA_XY_GPS = 3
SURSA_Z_GPS = 3
SURSA_YAW_GNSS = (2, 3, 8)     # GPS, GPS_COMPASS_FALLBACK, GSF

#: Parametrii care descriu un set, si ce valoare inseamna GNSS pentru
#: fiecare. Lista e scrisa POZITIV, ca o sursa noua sa nu scape neverificata
#: (§5.25).
TERMENI = (
    ('POSXY', (SURSA_XY_GPS,)),
    ('VELXY', (SURSA_XY_GPS,)),
    ('POSZ', (SURSA_Z_GPS,)),
    ('VELZ', (SURSA_Z_GPS,)),
    ('YAW', SURSA_YAW_GNSS),
)

#: Setul pe care il cerem in segmentul autonom.
SET_AUTONOM = 2
SET_NORMAL = 1

#: Fazele in care setul fara GNSS trebuie sa fie activ. Tot lista pozitiva.
FAZE_AUTONOME = ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE',
                 'FINAL_DESCENT', 'TOUCHDOWN_CONFIRM', 'ASCENT')

PARAM_TIMEOUT_S = 3.0
ACK_TIMEOUT_S = 2.0
NICIODATA = float('-inf')


def nume_param(set_n, termen):
    return f"EK3_SRC{set_n}_{termen}"


def contine_gnss(valori):
    """(bool, [motive]) pentru un dict {termen: valoare}.

    Mai strict decat `usingGPS()` din firmware - vezi docstring-ul modulului.
    O valoare lipsa NU se considera curata: necunoscut nu inseamna conform."""
    motive = []
    for termen, rele in TERMENI:
        v = valori.get(termen)
        if v is None:
            motive.append(f"{termen} necitit")
        elif int(v) in rele:
            motive.append(f"{termen}={int(v)} e GNSS")
    return bool(motive), motive


class EkfSourceManager:
    """Comuta setul de surse EKF pentru segmentul autonom si il restaureaza.

    Starile sunt aceleasi ca la `nova/authority.py`, si din acelasi motiv:
    nimic nu se comuta inainte de a sti ce se restaureaza."""

    IDLE, CITESTE, ACTIV, RESTAURAT, REFUZAT = (
        'IDLE', 'CITESTE', 'ACTIV', 'RESTAURAT', 'REFUZAT')

    def __init__(self, vehicle, set_autonom=SET_AUTONOM,
                 set_normal=SET_NORMAL, on_event=None, verbose=True):
        self.v = vehicle
        self.set_autonom = set_autonom
        self.set_normal = set_normal
        self.on_event = on_event
        self.verbose = verbose
        self.state = self.IDLE
        self.log = []
        self.valori = {}
        self._cerut_t = NICIODATA
        self._comutat = False
        self.motive_refuz = []

    # -- log ---------------------------------------------------------------
    def _emit(self, now, kind, detail):
        ev = {'t': now, 'kind': kind, 'detail': detail,
              'time_boot_ms': getattr(self.v, 'time_boot_ms', None)}
        self.log.append(ev)
        if self.verbose:
            tb = ev['time_boot_ms']
            print(f"[EKF t={now:.3f} boot_ms={tb} {kind}] {detail}")
        if self.on_event:
            self.on_event(ev)
        return ev

    def log_lines(self):
        return [f"[EKF {e['kind']}] {e['detail']}" for e in self.log]

    # -- citirea setului tinta ---------------------------------------------
    def _cere_parametrii(self, now):
        for termen, _ in TERMENI:
            self.v.request_param(nume_param(self.set_autonom, termen))
        self._cerut_t = now

    def _aduna(self):
        gata = True
        for termen, _ in TERMENI:
            nume = nume_param(self.set_autonom, termen)
            val = getattr(self.v, 'params', {}).get(nume)
            if val is None:
                gata = False
            else:
                self.valori[termen] = val
        return gata

    # -- ciclul de viata ---------------------------------------------------
    def update(self, now=None, phase='IDLE'):
        """De apelat din bucla. Se armeaza si elibereaza din FAZA (§5.14)."""
        now = time.monotonic() if now is None else now
        in_segment = phase in FAZE_AUTONOME

        if not in_segment:
            if self.state in (self.ACTIV, self.CITESTE):
                self.release(now, f"faza {phase}")
            return self.state

        if self.state == self.IDLE:
            self.state = self.CITESTE
            self.valori = {}
            self._cere_parametrii(now)
            self._emit(now, 'arm', f"citesc EK3_SRC{self.set_autonom}_* "
                                   f"inainte de a comuta")
            return self.state

        if self.state == self.CITESTE:
            if self._aduna():
                rau, motive = contine_gnss(self.valori)
                if rau:
                    self.state = self.REFUZAT
                    self.motive_refuz = motive
                    self._emit(now, 'refuz',
                               f"NU comut: setul {self.set_autonom} contine "
                               f"GNSS ({', '.join(motive)})")
                else:
                    if self.v.send_ekf_source_set(self.set_autonom):
                        self._comutat = True
                        self.state = self.ACTIV
                        self._emit(now, 'switch',
                                   f"set {self.set_autonom} cerut; "
                                   + ', '.join(f"{k}={int(v)}" for k, v
                                               in sorted(self.valori.items())))
                    # legatura cazuta: reincercam la ciclul urmator
            elif now - self._cerut_t > PARAM_TIMEOUT_S:
                self.state = self.REFUZAT
                self.motive_refuz = ['FC-ul nu raspunde la citire']
                self._emit(now, 'refuz',
                           "NU comut: FC-ul nu a raspuns la citirea "
                           "parametrilor de surse")
            return self.state

        return self.state

    def release(self, now=None, reason=''):
        """Inapoi la setul normal. Se apeleaza si pe calea de abort."""
        now = time.monotonic() if now is None else now
        if self._comutat:
            self.v.send_ekf_source_set(self.set_normal)
            self._comutat = False
            self._emit(now, 'restore',
                       f"set {self.set_normal} cerut inapoi; {reason}")
        self.state = self.RESTAURAT if self.state == self.ACTIV else self.IDLE
        return self.state

    # -- ce se raporteaza --------------------------------------------------
    @property
    def conform(self):
        """True doar daca setul activ a fost CITIT si nu contine GNSS.

        `False` inainte de comutare si dupa restaurare - nu e o stare de
        eroare, e raspunsul corect la "esti acum in regim fara GNSS?"."""
        if self.state != self.ACTIV:
            return False
        rau, _ = contine_gnss(self.valori)
        return not rau

    @property
    def ack(self):
        """Raspunsul FC-ului la ultima comutare, sau None."""
        return getattr(self.v, 'ekf_src_ack', None)

    @property
    def ack_ok(self):
        return self.ack == mavutil.mavlink.MAV_RESULT_ACCEPTED

    def raport(self):
        return {
            'stare': self.state,
            'conform': self.conform,
            'ack': self.ack,
            'ack_ok': self.ack_ok,
            'set_autonom': self.set_autonom,
            'valori': {k: int(v) for k, v in sorted(self.valori.items())},
            'motive_refuz': list(self.motive_refuz),
        }
