#!/usr/bin/env python3
"""
Modularea autoritatii pe praguri de altitudine, in segmentul autonom.

Ideea: autoritatea de control potrivita la 10 m nu e cea potrivita la 1 m.
Sus, estimarea de pozitie e zgomotoasa (§5.23: eroarea relativa scade cu
`marker_px`, deci la 45 px e de ~1%), asa ca un castig mare nu urmareste
markerul, ci zgomotul. Jos, estimarea e precisa (900 px la 0.5 m) dar
vehiculul e langa sol, deci acceleratiile laterale mari sunt exact ce nu
vrei. Deci castigurile urca si acceleratia scade pe masura ce coboara.

    sched = AuthorityScheduler(vehicle)
    ...
    sched.update(now, agl_m, phase, latency_s=det_p50)

Se armeaza si se elibereaza singur din **faza**, ca SafetySupervisor
(§5.14): nicio aplicatie nu trebuie sa ghiceasca tranzitia corecta, deci
nici nu o poate gresi. Orice iesire din segmentul autonom - HANDBACK,
ABORT, dezarmare, o faza noua la care nu ne-am gandit - declanseaza
restaurarea.

CE GARANTEAZA MODULUL

1. **Nu modifica niciodata un parametru a carui valoare originala nu a
   citit-o.** Daca FC-ul nu raspunde la cererea de citire, parametrul nu
   exista pe acest firmware (§5.4) sau legatura e proasta - in ambele
   cazuri, a-l scrie ar insemna o modificare pe care nu o putem anula.
2. **Restaurarea nu renunta.** `Vehicle.set_param` abandoneaza dupa
   PARAM_TRIES si scrie un avertisment; pentru restaurare asta nu ajunge.
   Modulul compara valoarea CITITA INAPOI cu tinta si reemite pana se
   confirma, iar cat timp legatura e cazuta nu consuma incercari (H1).
3. **`restored` e o afirmatie verificata**, nu o intentie. Ramane False
   pana cand FIECARE parametru atins a fost confirmat inapoi la valoarea
   lui originala. Aplicatia o poate arata pilotului.

CE NU ATINGE, DELIBERAT

`ANGLE_MAX` / `PSC_ANGLE_MAX` sunt in `FORBIDDEN` si un test verifica lista.
Unghiul maxim de inclinare e plafonul de autoritate al vehiculului, nu un
parametru de reglaj fin: schimbat in zbor, schimba comportamentul pe care
pilotul il asteapta la preluare, si interactioneaza cu monitorul de
inclinare din supervizor (`MAX_TILT_DEG` 30 deg) - am ajunge ca vehiculul
sa poata comanda legal o inclinare pe care supervizorul o trateaza ca
defectiune.

NUMELE PARAMETRILOR - VERIFICATE IN SURSA, NU DIN MEMORIE

Trei dintre numele "evidente" nu exista pe ArduCopter 4.8 (§5.4):

    WPNAV_ACCEL   -> WP_ACC          obiectul AC_WPNav are prefixul WP_,
                                     iar numele scurt e "ACC"
    PSC_POSXY_P   -> PSC_NE_POS_P    subgrupul s-a redenumit XY -> NE
    PSC_VELXY_D   -> PSC_NE_VEL_D    idem

Un `PARAM_SET` pe numele vechi nu produce eroare si nu produce
`PARAM_VALUE`: pur si simplu nu face nimic. Verificarea de la pasul 1 de
mai sus prinde asta la prima armare, nu in zbor.
"""

import time

#: Parametrii pe care FC-ul nu are voie sa-i vada schimbati de noi.
#: Lista e verificata de un test; vezi docstring-ul modulului pentru motiv.
FORBIDDEN = ('ANGLE_MAX', 'PSC_ANGLE_MAX')

#: Fazele in care modularea are sens - aceleasi in care segmentul autonom e
#: activ. Scrisa POZITIV (§8): o faza noua nu e modulata pana cand cineva
#: decide explicit ca trebuie.
MODULATED_PHASES = ('ACQUIRE', 'DESCEND_TRACK', 'SCORING_CAPTURE',
                    'FINAL_DESCENT')

#: Histereza pe praguri. Fara ea, zgomotul de altitudine in jurul unui prag
#: ar produce zeci de PARAM_SET pe secunda, exact in faza in care legatura
#: are altceva de facut. 0.5 m e peste zgomotul tipic de altitudine si sub
#: latimea celei mai inguste benzi.
BAND_HYSTERESIS_M = 0.5

#: Viteza de coborare validata in SITL (13 rulari, §4). Peste ea NU urcam
#: fara masuratoarea de distanta de franare ceruta de §6/15.2.9: la 0.5 m/s
#: franarea consuma 1.00 m, iar componenta de reactie creste liniar cu
#: viteza. Se ridica prin `allow_fast_descent=True`, dupa D1.
DESCENT_VALIDATED_MS = 0.5

#: PLND_LAG: domeniul din sursa (AC_PrecLand.cpp: @Range 0.02 0.250).
PLND_LAG_MIN_S = 0.02
PLND_LAG_MAX_S = 0.25

#: WP_ACC: @Range 0.50 5.00 m/s/s.
WP_ACC_MIN = 0.5
WP_ACC_MAX = 5.0

#: Cat asteptam un PARAM_VALUE la citirea initiala, inainte de a recere.
READ_TIMEOUT_S = 1.0
READ_TRIES = 5

#: Cat asteptam confirmarea unei scrieri inainte de a o reemite.
WRITE_CONFIRM_S = 0.4
#: Cate reemiteri pe banda. Restaurarea foloseste RESTORE_TRIES.
APPLY_TRIES = 6
RESTORE_TRIES = 40

#: Toleranta la compararea valorii citite inapoi. PARAM_VALUE e float32,
#: deci egalitatea exacta pe un float64 calculat de noi nu tine.
PARAM_TOL_REL = 1e-4
PARAM_TOL_ABS = 1e-5


#: "Nu s-a intamplat niciodata". NU 0.0: `time.monotonic()` poate porni de
#: oriunde, iar in teste bucla incepe chiar la 0.0 - cu 0.0 ca santinela,
#: prima cerere si prima scriere ar fi blocate de propriul lor timeout.
NICIODATA = float('-inf')


def _close(a, b):
    if a is None or b is None:
        return False
    return abs(a - b) <= max(PARAM_TOL_ABS, PARAM_TOL_REL * abs(b))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Spec:
    """Un parametru gestionat.

    `mode`:
      'mul' - tinta = original * factorul benzii. Deliberat relativ: nu
              cunoastem reglajul acestui airframe, iar o valoare absoluta
              luata de pe alt vehicul ar fi un numar inventat. Un factor de
              1.0 inseamna "nominal", indiferent cat e nominalul.
      'abs' - tinta = valoarea benzii, ca atare (viteze in m/s, lag in s).
      'lag' - caz special: vine din latenta MASURATA a detectorului.
    """

    __slots__ = ('name', 'mode', 'lo', 'hi', 'why')

    def __init__(self, name, mode, lo=None, hi=None, why=''):
        assert name not in FORBIDDEN, f"{name} e in FORBIDDEN"
        self.name = name
        self.mode = mode
        self.lo = lo
        self.hi = hi
        self.why = why

    def clamp(self, v):
        if self.lo is not None:
            v = max(self.lo, v)
        if self.hi is not None:
            v = min(self.hi, v)
        return v


#: Parametrii gestionati. Numele sunt cele REALE de pe 4.8 - vezi docstring.
MANAGED = (
    Spec('PSC_NE_POS_P', 'mul', 0.1, 5.0,
         'castig P pozitie orizontala; sus zgomotul de estimare domina'),
    Spec('PSC_NE_VEL_D', 'mul', 0.0, 1.0,
         'castig D viteza orizontala; D amplifica zgomotul, deci sus e mic'),
    Spec('WP_ACC', 'mul', WP_ACC_MIN, WP_ACC_MAX,
         'acceleratie laterala; jos, langa sol, o vrem mica'),
    Spec('WP_SPD_DN', 'abs', 0.1, 5.0,
         'viteza de coborare a controlerului de pozitie'),
    Spec('LAND_SPD_MS', 'abs', 0.1, 5.0,
         'viteza de coborare in LAND sub LAND_ALT_LOW_M'),
    Spec('PLND_LAG', 'lag', PLND_LAG_MIN_S, PLND_LAG_MAX_S,
         'latenta masurata captura->publicare a detectorului'),
)

MANAGED_BY_NAME = {s.name: s for s in MANAGED}


class Band:
    """O banda de altitudine si autoritatea din ea.

    `min_agl` e limita de JOS a benzii; benzile se dau descrescator.
    """

    __slots__ = ('nume', 'min_agl', 'valori')

    def __init__(self, nume, min_agl, **valori):
        self.nume = nume
        self.min_agl = min_agl
        necunoscute = set(valori) - set(MANAGED_BY_NAME)
        assert not necunoscute, f"parametri necunoscuti in banda: {necunoscute}"
        self.valori = valori

    def __repr__(self):
        return f"<Band {self.nume} >= {self.min_agl} m>"


#: Profilul implicit.
#:
#: Factorii sunt PROVIZORII: nu exista inca o masuratoare pe hardware care
#: sa-i justifice. Sunt alesi ca sa fie **conservatori in ambele directii** -
#: niciun factor nu iese din [0.6, 1.4] - si ca sa exprime relatia fizica din
#: §5.23, nu ca sa optimizeze ceva. Reglajul fin vine dupa E2, cu date.
#:
#: Vitezele de coborare sunt toate <= DESCENT_VALIDATED_MS: profilul livrat
#: NU coboara mai repede decat ce s-a validat deja. Treapta mai rapida de sus
#: exista in PROFIL_RAPID si cere intai masuratoarea de franare (§6/D1).
PROFIL_IMPLICIT = (
    Band('sus', 8.0,
         # -30%: treapta din procedura de diagnostic (pasul 2,
         # docs/DIAGNOSTIC_OSCILATIE.md). Suficient de mare cat sa se vada
         # pe o singura rulare, suficient de mica sa nu strice altceva.
         PSC_NE_POS_P=0.70,     # estimarea e zgomotoasa: nu urmari zgomotul
         PSC_NE_VEL_D=0.60,     # D amplifica cel mai tare
         WP_ACC=1.00,
         WP_SPD_DN=0.5,
         LAND_SPD_MS=0.5),
    Band('mijloc', 3.0,
         PSC_NE_POS_P=1.00,     # nominal
         PSC_NE_VEL_D=1.00,
         WP_ACC=1.00,
         WP_SPD_DN=0.5,
         LAND_SPD_MS=0.5),
    Band('jos', 0.5,
         PSC_NE_POS_P=1.25,     # 900 px la 0.5 m: estimarea merita urmarita
         PSC_NE_VEL_D=1.10,
         WP_ACC=0.70,           # langa sol, acceleratii laterale mici
         WP_SPD_DN=0.35,
         LAND_SPD_MS=0.35),
    Band('contact', 0.0,
         PSC_NE_POS_P=1.25,
         PSC_NE_VEL_D=1.10,
         WP_ACC=0.50,           # autoritatea laterala scoasa din joc
         WP_SPD_DN=0.35,
         LAND_SPD_MS=0.35),
)

#: Profilul agresiv (I5). Difera de cel implicit **numai pe coloana de
#: viteza**; benzile si castigurile sunt aceleasi. Asta e deliberat: cand se
#: compara doua rulari, singura variabila schimbata trebuie sa fie profilul
#: de coborare, nu si reglajul controlerului.
#:
#: | banda | AGL | viteza | ce se schimba fata de nominal |
#: |---|---|---|---|
#: | sus     | > 8 m     | 1.5 m/s | `PSC_NE_POS_P` x0.70, `PSC_NE_VEL_D` x0.60 |
#: | mijloc  | 8 - 3 m   | 0.8 m/s | `PSC_NE_POS_P` nominal |
#: | jos     | 3 - 0.5 m | 0.3 m/s | `WP_ACC` limitat la 0.70 |
#: | contact | < 0.5 m   | 0.2 m/s | `WP_ACC` la podea, fara autoritate laterala |
#:
#: Ultima banda NU e cea care opreste corectiile laterale - aia e
#: `FINAL_DESCENT` din masina de stari (§8, sub 0.4 m). Aici doar se scoate
#: autoritatea care ar ramane disponibila daca ceva ar cere-o.
#:
#: BLOCAT pana la masuratoarea de distanta de franare la FIECARE treapta
#: (§6/15.2.9, elementul deschis 11 din §7). `allow_fast_descent` refuza
#: pana atunci, si refuza pe buna dreptate: la 1.5 m/s doar timpul de
#: reactie masurat (0.48 s) inseamna 0.72 m, iar pragul sub care garantia
#: de hover nu mai tine urca odata cu viteza.
PROFIL_RAPID = (
    Band('sus', 8.0, PSC_NE_POS_P=0.70, PSC_NE_VEL_D=0.60, WP_ACC=1.00,
         WP_SPD_DN=1.5, LAND_SPD_MS=1.5),
    Band('mijloc', 3.0, PSC_NE_POS_P=1.00, PSC_NE_VEL_D=1.00, WP_ACC=1.00,
         WP_SPD_DN=0.8, LAND_SPD_MS=0.8),
    Band('jos', 0.5, PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.70,
         WP_SPD_DN=0.3, LAND_SPD_MS=0.3),
    Band('contact', 0.0, PSC_NE_POS_P=1.25, PSC_NE_VEL_D=1.10, WP_ACC=0.50,
         WP_SPD_DN=0.2, LAND_SPD_MS=0.2),
)


class AuthorityEvent:
    """O intrare de log. Poarta `time_boot_ms` ca sa se alinieze cu .bin
    (6.2.1.30), la fel ca evenimentele de supervizor."""

    __slots__ = ('t', 'time_boot_ms', 'kind', 'detail')

    def __init__(self, t, time_boot_ms, kind, detail):
        self.t = t
        self.time_boot_ms = time_boot_ms
        self.kind = kind
        self.detail = detail

    def __str__(self):
        tb = '-' if self.time_boot_ms is None else str(self.time_boot_ms)
        return f"[AUTH t={self.t:.3f} boot_ms={tb}] {self.kind}: {self.detail}"


class AuthorityScheduler:

    IDLE = 'IDLE'
    SAVING = 'SAVING'          # citim valorile originale
    ACTIVE = 'ACTIVE'          # modulam pe benzi
    RESTORING = 'RESTORING'    # punem originalele inapoi
    DONE = 'DONE'              # restaurare confirmata

    def __init__(self, vehicle, bands=PROFIL_IMPLICIT, on_event=None,
                 verbose=True, allow_fast_descent=False,
                 hysteresis_m=BAND_HYSTERESIS_M):
        self.v = vehicle
        self.bands = tuple(bands)
        self.on_event = on_event
        self.verbose = verbose
        self.allow_fast_descent = allow_fast_descent
        self.hysteresis_m = hysteresis_m

        self.state = self.IDLE
        #: {nume: valoare} citita de pe FC INAINTE de orice modificare.
        self.original = {}
        #: Parametri pe care NU ii gestionam: FC-ul nu a raspuns la citire.
        #: Aproape sigur nu exista pe acest firmware (§5.4).
        self.unavailable = []
        #: {nume: tinta} ce am scris ultima data.
        self.applied = {}
        self.band = None
        self.log = []

        self._read_t = NICIODATA
        self._read_tries = 0
        self._write_t = {}
        self._write_tries = {}
        self._latency_s = None
        self._arm_t = None

    # -- log ---------------------------------------------------------------
    def _emit(self, now, kind, detail):
        ev = AuthorityEvent(now, getattr(self.v, 'time_boot_ms', None), kind,
                            detail)
        self.log.append(ev)
        if self.verbose:
            print(str(ev))
        if self.on_event:
            self.on_event(ev)
        return ev

    def log_lines(self):
        return [str(e) for e in self.log]

    # -- proprietati -------------------------------------------------------
    @property
    def restored(self):
        """True doar daca fiecare parametru atins a fost CONFIRMAT inapoi la
        valoarea lui originala. Nu 'am trimis comenzile'."""
        if self.state in (self.IDLE, self.DONE):
            return True
        if self.state == self.SAVING:
            return True          # nu am modificat inca nimic
        return not self._outstanding(self.original)

    @property
    def modified(self):
        """Parametrii care sunt ACUM diferiti de original, dupa citire
        inapoi. Lista goala = vehiculul e la autoritate nominala."""
        out = []
        for name, orig in self.original.items():
            cur = self._read(name)
            if cur is not None and not _close(cur, orig):
                out.append(name)
        return out

    def _read(self, name):
        return getattr(self.v, 'params', {}).get(name)

    def _read_fresh(self, name):
        """Valoarea, dar NUMAI daca a fost vazuta dupa `arm()`.

        `Vehicle.params` e un cache: poate contine o valoare de acum zece
        minute, iar intre timp cineva poate sa o fi schimbat din GCS. Pentru
        salvare asta nu e acceptabil - am restaura la o valoare care nu a
        fost niciodata cea de la handover. Prima varianta citea direct din
        cache si "salva" instantaneu, fara sa fi cerut nimic de la FC."""
        if self._arm_t is None:
            return None
        vazut = getattr(self.v, 'params_t', {}).get(name)
        if vazut is None or vazut < self._arm_t:
            return None
        return self._read(name)

    def _link_ok(self):
        # Cu legatura cazuta, un PARAM_SET nu pleaca (H1). Nu consumam
        # incercari vorbind cu un port inchis.
        return getattr(self.v, 'link_healthy', True)

    # -- ciclu de viata ----------------------------------------------------
    def arm(self, now):
        """Incepe salvarea valorilor originale. Nu modifica nimic inca."""
        if self.state in (self.SAVING, self.ACTIVE):
            return
        self.state = self.SAVING
        self._arm_t = now
        self.original = {}
        self.unavailable = []
        self.applied = {}
        self.band = None
        self._read_t = NICIODATA
        self._read_tries = 0
        self._write_t = {}
        self._write_tries = {}
        self._emit(now, 'arm', f"citesc {len(MANAGED)} parametri originali")

    def release(self, now, reason=''):
        """Restaureaza. Idempotent: se poate apela de oricate ori."""
        if self.state in (self.IDLE, self.DONE, self.RESTORING):
            return
        if self.state == self.SAVING:
            # Nu am apucat sa modificam nimic, deci nu e nimic de restaurat.
            self.state = self.DONE
            self._emit(now, 'release', f"{reason} (nimic nu fusese modificat)")
            return
        self.state = self.RESTORING
        self._write_t = {}
        self._write_tries = {}
        self._emit(now, 'release',
                   f"{reason}; restaurez {len(self.original)} parametri")

    # -- bucla -------------------------------------------------------------
    def update(self, now=None, agl_m=None, phase='IDLE', latency_s=None):
        """De apelat din bucla principala, dupa supervizor.

        Se armeaza/elibereaza singur din `phase` (§5.14). Restaurarea
        continua indiferent de faza: odata inceputa, se duce la capat."""
        now = now if now is not None else time.monotonic()
        if latency_s is not None:
            self._latency_s = latency_s

        in_segment = phase in MODULATED_PHASES
        if in_segment and self.state in (self.IDLE, self.DONE):
            self.arm(now)
        elif not in_segment and self.state in (self.SAVING, self.ACTIVE):
            self.release(now, f"faza {phase}")

        if self.state == self.SAVING:
            self._save_step(now)
        elif self.state == self.ACTIVE:
            self._apply_step(now, agl_m)
        elif self.state == self.RESTORING:
            self._restore_step(now)
        return self.state

    # -- salvare -----------------------------------------------------------
    def _save_step(self, now):
        lipsa = [s.name for s in MANAGED
                 if s.name not in self.original and s.name not in self.unavailable]
        for name in list(lipsa):
            val = self._read_fresh(name)
            if val is not None:
                self.original[name] = float(val)
                lipsa.remove(name)

        if not lipsa:
            self.state = self.ACTIVE
            self._emit(now, 'saved',
                       ', '.join(f"{k}={v:g}" for k, v in
                                 sorted(self.original.items())))
            if self.unavailable:
                # §5.4/§5.10: un parametru care nu raspunde nu exista pe acest
                # firmware. NU il gestionam - a-l scrie ar fi o modificare pe
                # care nu am putea-o anula, si oricum nu ar face nimic.
                self._emit(now, 'unavailable',
                           f"{', '.join(self.unavailable)} - FC-ul nu "
                           f"raspunde; NU ii gestionez (nume gresit pe acest "
                           f"firmware?)")
            return

        if not self._link_ok():
            return
        if now - self._read_t < READ_TIMEOUT_S:
            return
        self._read_t = now
        self._read_tries += 1
        if self._read_tries > READ_TRIES:
            self.unavailable.extend(lipsa)
            return
        for name in lipsa:
            self.v.request_param(name)

    # -- aplicare ----------------------------------------------------------
    def band_for(self, agl_m):
        """Banda pentru o altitudine, CU histereza fata de banda curenta.

        Coborand, trecem in banda de jos cand agl < prag. Urcand, revenim
        abia la prag + histereza. Fara asta, zgomotul din jurul pragului
        produce un PARAM_SET pe ciclu."""
        if agl_m is None:
            return self.band
        candidata = self.bands[-1]
        for b in self.bands:
            if agl_m >= b.min_agl:
                candidata = b
                break
        if self.band is None or candidata is self.band:
            return candidata
        idx_nou = self.bands.index(candidata)
        idx_vechi = self.bands.index(self.band)
        if idx_nou < idx_vechi:
            # urcam intr-o banda mai sus: cerem sa fi depasit pragul + marja
            if agl_m < candidata.min_agl + self.hysteresis_m:
                return self.band
        return candidata

    def target_for(self, band, name):
        """Valoarea tinta pentru un parametru in banda data, sau None daca
        parametrul nu e gestionat (lipsa originalului = nu-l atingem)."""
        if name not in self.original:
            return None
        spec = MANAGED_BY_NAME[name]
        if spec.mode == 'lag':
            if self._latency_s is None:
                return None          # fara masuratoare, nu inventam o valoare
            val = float(self._latency_s)
        else:
            if name not in band.valori:
                return None
            val = float(band.valori[name])
            if spec.mode == 'mul':
                val *= self.original[name]
        val = spec.clamp(val)
        if spec.mode == 'abs' and name in ('WP_SPD_DN', 'LAND_SPD_MS'):
            if not self.allow_fast_descent:
                # §6/15.2.9: distanta de franare creste cu viteza, si e
                # masurata doar la 0.5 m/s. Peste, garantia de hover se
                # pierde mai sus decat stim noi.
                val = min(val, DESCENT_VALIDATED_MS)
        return val

    def _apply_step(self, now, agl_m):
        noua = self.band_for(agl_m)
        if noua is not None and noua is not self.band:
            vechea = self.band
            self.band = noua
            tinte = {}
            for s in MANAGED:
                t = self.target_for(noua, s.name)
                if t is not None:
                    tinte[s.name] = t
            self.applied = tinte
            self._write_t = {}
            self._write_tries = {}
            self._emit(now, 'band',
                       f"{'-' if vechea is None else vechea.nume} -> "
                       f"{noua.nume} la "
                       f"{'?' if agl_m is None else f'{agl_m:.2f} m'}; " +
                       ', '.join(f"{k}={v:g}" for k, v in sorted(tinte.items())))
        self._drive_writes(now, self.applied, APPLY_TRIES, 'apply')

    # -- restaurare --------------------------------------------------------
    def _restore_step(self, now):
        self._drive_writes(now, self.original, RESTORE_TRIES, 'restore')
        if not self._outstanding(self.original):
            self.state = self.DONE
            self.applied = {}
            self.band = None
            self._emit(now, 'restored',
                       f"{len(self.original)} parametri confirmati inapoi la "
                       f"valorile de la handover")

    # -- motorul de scriere ------------------------------------------------
    def _outstanding(self, tinte):
        """Parametrii care inca NU sunt confirmati la tinta, prin citire
        inapoi. Asta e definitia lui 'restaurat' - nu 'am trimis'."""
        return [n for n, t in tinte.items() if not _close(self._read(n), t)]

    def _drive_writes(self, now, tinte, max_tries, eticheta):
        """Reemite pana cand valoarea CITITA INAPOI se potriveste.

        `Vehicle.set_param` confirma si el prin PARAM_VALUE, dar renunta
        dupa PARAM_TRIES si sterge cererea. Pentru restaurare asta nu
        ajunge: bucla de aici o rearmeaza. Cat timp legatura e cazuta nu
        consumam incercari - altfel am epuiza bugetul vorbind cu un port
        inchis, exact ca in `safety._drive_mode` (H1)."""
        if not self._link_ok():
            return
        for name, tinta in tinte.items():
            if _close(self._read(name), tinta):
                continue
            if getattr(self.v, 'param_pending', None) and \
                    self.v.param_pending(name):
                continue
            if now - self._write_t.get(name, NICIODATA) < WRITE_CONFIRM_S:
                continue
            n = self._write_tries.get(name, 0)
            if n >= max_tries:
                if n == max_tries:
                    self._write_tries[name] = n + 1
                    self._emit(now, f'{eticheta}_fail',
                               f"{name}: FC-ul nu confirma {tinta:g} dupa "
                               f"{max_tries} incercari (citit "
                               f"{self._read(name)})")
                continue
            self._write_t[name] = now
            self._write_tries[name] = n + 1
            self.v.set_param(name, tinta, now)

    # -- raportare ---------------------------------------------------------
    def status(self):
        if self.state == self.IDLE:
            return 'AUTH inactiv'
        if self.state == self.SAVING:
            return f"AUTH salvez ({len(self.original)}/{len(MANAGED)})"
        if self.state == self.ACTIVE:
            b = '-' if self.band is None else self.band.nume
            restante = len(self._outstanding(self.applied))
            return (f"AUTH banda {b}" +
                    (f", {restante} neconfirmate" if restante else ''))
        if self.state == self.RESTORING:
            return f"AUTH RESTAUREZ ({len(self._outstanding(self.original))})"
        return 'AUTH restaurat'
