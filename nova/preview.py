#!/usr/bin/env python3
"""
Previzualizare pe ecran (H3). Trei contexte, comportament diferit.

    1. UNELTE DE BANC (calibrare, masurare)
       Fullscreen. Operatorul are nevoie de detaliu: colturile detectate,
       marginea tintei, daca markerul e clar. Ecranul Pi-ului e mic si de
       obicei se lucreaza prin VNC.

    2. PRIN VNC
       Cadrul plin, 2304x1296, e lent pe VNC. Sacadarea nu inseamna ca
       detectia e lenta - dar asa arata, si operatorul incepe sa caute
       problema unde nu e. Redimensionarea e DOAR pentru afisare: detectia
       ruleaza pe cadrul plin, intotdeauna.

    3. PE VEHICULUL DE CONCURS
       Fara fereastra deloc. Fara `vc4-kms-v3d` nu exista accelerare
       grafica, deci fiecare `imshow` consuma CPU direct din bugetul
       detectiei. `show_window` e implicit FALS in nova_pi.py si in serviciu;
       cand se aprinde, se scrie in log ca degradeaza performanta.

Trei capcane, toate costa timp daca le redescoperi:

**`WINDOW_NORMAL` e obligatoriu pentru fullscreen.** O fereastra creata cu
`WINDOW_AUTOSIZE` (implicitul lui `imshow` fara `namedWindow`) **ignora**
`setWindowProperty(WND_PROP_FULLSCREEN)`. Nu da eroare - pur si simplu
ramane mica, si pare ca fullscreen-ul nu merge pe Pi.

**Iesirea trebuie legata si pe Escape, nu doar pe `q`.** O fereastra
fullscreen nu are decoratiuni, deci nu exista buton de inchidere. Daca
singura iesire e `q` si fereastra nu are focus tastatura, unealta nu se mai
poate opri decat din alt terminal.

**`waitKey` intoarce -1 cand nu s-a apasat nimic**, iar pe unele sisteme
codul vine cu biti in plus - de aceea comparam `& 0xFF`.
"""

import os

import cv2

#: Tastele de iesire: 'q', 'Q' si Escape.
EXIT_KEYS = (ord('q'), ord('Q'), 27)

#: Fara asta, `imshow` arunca `cv2.error: ... Can't initialize GUI backend`
#: pe un Pi fara sesiune grafica (rulat prin SSH, fara X forwarding).
#: Verificam inainte, ca sa dam un mesaj util in loc de un traceback.
GUI_ENV_VARS = ('DISPLAY', 'WAYLAND_DISPLAY')


def gui_available():
    """True daca exista o sesiune grafica in care sa apara fereastra."""
    return any(os.environ.get(v) for v in GUI_ENV_VARS)


#: Rotirea imaginii brute SPRE STANGA (pe ecran) -> codul cv2 care o face.
#: Aceeasi definitie ca `camera_rotation_deg` din config: cu cat rotesti ca
#: nasul dronei sa ajunga sus.
_ROTIRI_CV2 = {90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180,
               270: cv2.ROTATE_90_CLOCKWISE}


def roteste_pentru_afisare(img, rotatie_deg):
    """Copie rotita SPRE STANGA cu `rotatie_deg`, numai pentru ecran.

    Detectia nu trece niciodata pe aici: ea lucreaza pe cadrul brut, cu
    rotatia aplicata pe axe (`nova.detector_pi.axe_corp`), ca sa nu
    invalideze calibrarea. Aici doar operatorul vede imaginea cu nasul sus."""
    r = int(rotatie_deg) % 360
    if r == 0:
        return img
    if r not in _ROTIRI_CV2:
        raise ValueError(f"rotatie {rotatie_deg}: doar 0/90/180/270")
    return cv2.rotate(img, _ROTIRI_CV2[r])


class Preview:
    """O fereastra, sau nimic.

        pv = Preview('NOVA calibrare', enabled=True, fullscreen=True)
        ...
        if pv.show(frame):      # False cand operatorul a cerut iesirea
            break
        pv.close()

    Cu `enabled=False` toate metodele sunt no-op si nu se importa niciun
    backend grafic. Asta e regimul de pe vehicul, si e implicit acolo -
    codul apelant nu are nevoie de `if`-uri.
    """

    def __init__(self, title, enabled=False, fullscreen=False, scale=1.0,
                 logger=None, rotate_deg=0):
        self.title = title
        if int(rotate_deg) % 360 not in (0, 90, 180, 270):
            raise ValueError(f"rotatie {rotate_deg}: doar 0/90/180/270")
        self.rotate_deg = int(rotate_deg) % 360
        self.scale = float(scale)
        self.fullscreen = fullscreen
        self.enabled = bool(enabled)
        self.created = False
        self.log = logger or (lambda msg: print(msg))
        self.disabled_reason = None

        if self.enabled and not gui_available():
            self.enabled = False
            self.disabled_reason = (
                f"nicio sesiune grafica ({'/'.join(GUI_ENV_VARS)} nesetate). "
                f"Prin SSH foloseste `ssh -X`, sau ruleaza din desktop-ul "
                f"Pi-ului / prin VNC.")
            self.log(f"[preview] OPRITA: {self.disabled_reason}")

    # -- ciclu de viata ----------------------------------------------------
    def _create(self):
        # WINDOW_NORMAL, nu AUTOSIZE: altfel cererea de fullscreen de mai jos
        # e ignorata TACUT si fereastra ramane mica.
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        if self.fullscreen:
            cv2.setWindowProperty(self.title, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN)
        self.created = True

    def show(self, frame, wait_ms=1, text=None):
        """Afiseaza un cadru. False daca operatorul a cerut iesirea.

        `frame` NU se modifica: redimensionarea produce o copie si e doar
        pentru afisare. Ce se masoara si ce se deseneaza mai departe ramane
        in coordonatele cadrului plin."""
        if not self.enabled:
            return True
        if not self.created:
            self._create()
        img = frame
        if self.scale != 1.0:
            img = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                             interpolation=cv2.INTER_AREA)
        # dupa redimensionare: rotim cadrul mic, nu pe cel de 3 MB
        img = roteste_pentru_afisare(img, self.rotate_deg)
        # textul se scrie DUPA rotire, altfel s-ar roti si el si nu s-ar mai
        # citi. Pe o copie: cadrul primit nu se modifica niciodata.
        if text:
            img = img.copy()
            for i, linie in enumerate(text):
                cv2.putText(img, linie, (12, 32 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
                            cv2.LINE_AA)
        cv2.imshow(self.title, img)
        key = cv2.waitKey(wait_ms) & 0xFF
        if key in (k & 0xFF for k in EXIT_KEYS):
            return False
        return True

    def close(self):
        if self.created:
            try:
                cv2.destroyWindow(self.title)
                # Fara un waitKey dupa destroy, fereastra ramane pe ecran pe
                # unele backend-uri: distrugerea se proceseaza abia la
                # urmatorul ciclu de evenimente.
                cv2.waitKey(1)
            except cv2.error:
                pass
            self.created = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def bench_preview(title, enabled=True, scale=1.0, logger=None, rotate_deg=0):
    """Contextul 1: unealta de banc. Fullscreen, iesire pe q si Escape."""
    return Preview(title, enabled=enabled, fullscreen=True, scale=scale,
                   logger=logger, rotate_deg=rotate_deg)


def onboard_preview(title, enabled=False, scale=0.5, logger=None,
                    fullscreen=False, rotate_deg=0):
    """Contextul 3: pe vehicul. Implicit OPRITA; la aprindere, avertizeaza.

    Avertismentul nu e politete. Pe un Pi fara vc4-kms-v3d, `imshow` merge
    prin software rendering si ia CPU din exact bugetul care trebuie sa
    tina 30 fps de detectie. Cine aprinde fereastra intr-o cursa trebuie sa
    stie ce plateste.

    `fullscreen` e pentru bring-up la sol, unde monitorul e singurul mod de
    a vedea ce vede camera si nimeni nu cronometreaza. Ramane fals implicit:
    pe un ecran de 1080p, fullscreen inseamna si mai mult CPU de scalare."""
    pv = Preview(title, enabled=enabled, fullscreen=fullscreen, scale=scale,
                 logger=logger, rotate_deg=rotate_deg)
    if pv.enabled:
        pv.log("[preview] ATENTIE: fereastra PORNITA pe bord. Fara "
               "vc4-kms-v3d nu exista accelerare grafica, deci afisarea "
               "consuma CPU din bugetul detectiei. Verifica FPS-ul in "
               "linia de stare; opreste fereastra pentru cursa.")
    return pv
