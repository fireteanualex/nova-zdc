#!/usr/bin/env python3
"""
8.3.3: imaginea predata juriului, si evidenta care o insoteste (J2).

Regula cere o imagine **la momentul contactului**, iar aterizarea e valida
daca centrul imaginii contine un pixel de pe suprafata markerului. Problema
e ca la 74.5 mm amprenta camerei e 184x99 mm pe un marker de 480 mm: se vede
sub un sfert din el, si juriul nu poate masura nimic.

Solutia din §6/8.3.3, fara modificari mecanice: doua imagini, amandoua
scoase din acelasi ring buffer, dupa **timestamp-ul capturii**:

    captura de scoring   declansata pe dimensiunea markerului in pixeli,
                         deci mai sus - markerul se vede intreg
    cadrul de contact    exact la touchdown, pentru litera cerintei

CE FACE DIFERENTA INTRE EVIDENTA SI O POZA

- Cadrul se alege dupa timestamp-ul de CAPTURA purtat de eveniment, nu dupa
  cel al deciziei si nici dupa "ultimul cadru de acum". Intre ele sunt zeci
  de milisecunde de coborare.
- Daca ringul nu contine niciun cadru la acel moment, NU se salveaza altul:
  se raporteaza lipsa. O imagine gresita predata ca dovada e mai rea decat
  una lipsa, fiindca nimeni nu mai poate afla ca e gresita.
- Fiecare imagine primeste un fisier `.json` alaturi, cu `time_boot_ms` de
  la FC, ca sa se poata alinia cu `.bin` (6.2.1.30). Un `.png` singur nu se
  poate pune in relatie cu nimic.
"""

import json
import os
import time


class ScoringRecorder:
    """Scoate din ring cadrele cerute de 8.3.3 si le scrie cu evidenta lor.

        rec = ScoringRecorder(out_dir, ring, vehicle)
        sm = LandingStateMachine(..., on_event=rec.on_event)

    `ring` e orice obiect cu `since(t)` care intoarce [(t, cadru)]
    cronologic - `nova.frame_ring.FrameRing`, sau altceva in teste.
    """

    #: Cat de departe de timestamp-ul cerut acceptam un cadru. La 30 fps un
    #: cadru e la 33 ms; peste 0.2 s inseamna ca ringul nu acoperea momentul,
    #: iar un cadru "apropiat" ar fi de fapt alt moment al coborarii.
    TOLERANTA_S = 0.2

    EVENIMENTE = ('scoring_capture', 'touchdown')

    def __init__(self, out_dir, ring, vehicle=None, verbose=True,
                 on_note=None):
        self.out_dir = out_dir
        self.ring = ring
        self.v = vehicle
        self.verbose = verbose
        self.on_note = on_note
        self.salvate = {}
        self.lipsa = {}

    # -- alegerea cadrului -------------------------------------------------
    def cadru_la(self, t):
        """(timestamp, cadru) cel mai apropiat DUPA `t`, sau None.

        Se cauta inainte, nu in jur: cadrul cerut e cel care a produs
        detectia, iar unul de dinainte arata un alt moment al coborarii."""
        if self.ring is None:
            return None
        try:
            candidati = self.ring.since(t)
        except Exception:                                    # noqa: BLE001
            return None
        if not candidati:
            return None
        ts, frame = candidati[0]
        if ts - t > self.TOLERANTA_S:
            return None
        return ts, frame

    # -- scrierea ----------------------------------------------------------
    def _noteaza(self, text):
        if self.verbose:
            print(text)
        if self.on_note:
            self.on_note(text)

    def _meta(self, nume, t_cerut, t_cadru, info):
        m = {
            'eveniment': nume,
            't_cerut': t_cerut,
            't_cadru': t_cadru,
            'decalaj_ms': None if t_cadru is None else (t_cadru - t_cerut) * 1e3,
            'scris_la': time.time(),
        }
        for k, v in (info or {}).items():
            if isinstance(v, (int, float, str, bool)) or v is None:
                m[k] = v
        if self.v is not None:
            # 6.2.1.30: ancora catre .bin. Ceasul FC-ului, nu al Pi-ului.
            m['time_boot_ms'] = getattr(self.v, 'time_boot_ms', None)
            m['alt_m'] = getattr(self.v, 'alt', None)
            for ax in ('roll', 'pitch', 'yaw'):
                m[ax] = getattr(self.v, ax, None)
        return m

    def salveaza(self, nume, t, info=None):
        """Scrie imaginea si evidenta ei. (cale_png, cale_json) sau None."""
        gasit = self.cadru_la(t)
        if gasit is None:
            self.lipsa[nume] = t
            self._noteaza(f"!! {nume}: niciun cadru in ring la t={t:.3f} "
                          f"(toleranta {self.TOLERANTA_S:g} s) - NU salvez "
                          f"altul")
            return None
        ts, frame = gasit
        os.makedirs(self.out_dir, exist_ok=True)
        baza = f"{nume}_{int(ts * 1000):015d}"
        png = os.path.join(self.out_dir, baza + '.png')
        js = os.path.join(self.out_dir, baza + '.json')
        try:
            import cv2
            if not cv2.imwrite(png, frame):
                self._noteaza(f"!! {nume}: imwrite a esuat pentru {png}")
                return None
        except Exception as e:                               # noqa: BLE001
            self._noteaza(f"!! {nume}: nu s-a putut scrie imaginea: {e}")
            return None
        with open(js, 'w') as f:
            json.dump(self._meta(nume, t, ts, info), f, indent=2,
                      sort_keys=True)
        self.salvate[nume] = (png, js)
        self._noteaza(f"  >> {nume}: {os.path.basename(png)} "
                      f"(decalaj {(ts - t) * 1e3:+.0f} ms)")
        return png, js

    # -- carligul din masina de stari --------------------------------------
    def on_event(self, nume, info):
        """De dat ca `on_event` lui `LandingStateMachine`.

        Ambele evenimente poarta `t` = momentul CAPTURII cadrului care le-a
        declansat, nu al deciziei."""
        if nume not in self.EVENIMENTE:
            return
        t = (info or {}).get('t')
        if t is None:
            self._noteaza(f"!! {nume}: evenimentul nu poarta timestamp; "
                          f"nu pot alege cadrul")
            self.lipsa[nume] = None
            return
        self.salveaza(nume, t, info)

    # -- ce se raporteaza --------------------------------------------------
    def raport(self):
        return {
            'salvate': {k: os.path.basename(v[0])
                        for k, v in sorted(self.salvate.items())},
            'lipsa': sorted(self.lipsa),
            'complet_8_3_3': all(e in self.salvate for e in self.EVENIMENTE),
        }
