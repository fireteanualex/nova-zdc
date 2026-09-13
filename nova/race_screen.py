#!/usr/bin/env python3
"""
Ecranul de concurs (H4).

Un singur ecran, citibil **de la un metru, in soare, pe un ecran mic**. Asta
nu e o cerinta estetica, e ce decide daca informatia ajunge la operator in
secundele dintre curse.

Consecinte de proiectare, fiecare contra instinctului normal:

- **Putine linii, late.** Un tabel dens e ilizibil de la un metru. Sub zece
  linii, fiecare cu un singur lucru.
- **Culoarea e binara, nu nuantata.** Verde = bine, rosu = rau, galben =
  atentie. Fara "verde inchis pentru starea X". In soare, pe un LCD ieftin,
  nuantele dispar; contrastul ramane.
- **Verdictul e scris cu litere, nu doar colorat.** Un operator care nu
  distinge rosu de verde, sau un ecran spalat de soare, trebuie sa poata citi
  `NU ZBURA` / `GATA`.
- **Cifra si unitatea, nu doar cifra.** "5.2" nu inseamna nimic; "5.2 m" da.
- **Ce lipseste se scrie `-`, nu 0.** Un FPS de 0 si un FPS necunoscut sunt
  situatii diferite, iar a le confunda pe teren trimite cautarea aiurea.

Nu deseneaza cu OpenCV: e terminal pur, deci merge prin SSH, nu are nevoie de
sesiune grafica si nu consuma CPU din bugetul detectiei (§5.28).
"""

import os
import shutil

# --- ANSI -------------------------------------------------------------------
RESET = '\033[0m'
BOLD = '\033[1m'
#: Fundal plin + text negru. Pe un ecran spalat de soare, un bloc de culoare
#: se vede cand un text colorat nu se mai vede.
BG = {'verde': '\033[42;30m', 'rosu': '\033[41;37m', 'galben': '\033[43;30m',
      'gri': '\033[47;30m', 'albastru': '\033[44;37m'}
FG = {'verde': '\033[32m', 'rosu': '\033[31m', 'galben': '\033[33m',
      'gri': '\033[90m', 'alb': '\033[97m'}

CLEAR = '\033[2J\033[H'
HIDE_CURSOR = '\033[?25l'
SHOW_CURSOR = '\033[?25h'

#: Sub atata spatiu liber, o sesiune de capturi poate umple cardul la mijlocul
#: unei curse. 500 MB = ~160 de cadre de scoring la rezolutie nativa.
DISK_WARN_MB = 500.0
DISK_CRIT_MB = 150.0

#: Pi 4 fara radiator ajunge aici sub sarcina. Peste 80 C firmware-ul
#: limiteaza frecventa, ceea ce arata ca un detector lent (§ run_e2).
TEMP_WARN_C = 70.0
TEMP_CRIT_C = 80.0


def term_width(default=80):
    try:
        return max(shutil.get_terminal_size().columns, 40)
    except OSError:
        return default


def bar(text, culoare, width=None):
    """O banda plina, pe toata latimea. Ce se vede prima data."""
    width = width or term_width()
    txt = text.center(width - 1)
    return f"{BG[culoare]}{BOLD}{txt}{RESET}"


def spaced(text):
    """`GATA` -> `G A T A`. Litere rarite se citesc de mai departe."""
    return ' '.join(text)


#: Latimea coloanei de etichete. Trebuie sa fie mai mare decat cea mai lunga
#: eticheta, altfel valoarea se lipeste de ea: cu 11, `LEGATURA FC` si
#: `TEMPERATURA` (ambele exact 11) dadeau `LEGATURA FCOK  0.3 s`. La un metru,
#: asta nu se mai citeste ca doua lucruri.
LABEL_W = 14


def line(eticheta, valoare, culoare='alb', width=None):
    width = width or term_width()
    et = f"  {eticheta:<{LABEL_W}}"
    return f"{BOLD}{et}{RESET}{FG.get(culoare, '')}{BOLD}{valoare}{RESET}"


def disk_free_mb(path='/'):
    try:
        st = os.statvfs(path)
        return st.f_bavail * st.f_frsize / 1e6
    except OSError:
        return None


def cpu_temp_c(path='/sys/class/thermal/thermal_zone0/temp'):
    try:
        with open(path) as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def _fmt(v, fmt, unit='', lipsa='-'):
    """Valoarea cu unitate. Cand lipseste, `- fps`, nu doar `-`.

    Unitatea ramane deliberat: pe un rand cu doua cifre (`29 fps  detectie
    97%`), un `-` singur nu spune care dintre ele lipseste."""
    if v is None:
        return f"{lipsa}{unit}" if unit else lipsa
    return f"{format(v, fmt)}{unit}"


class RaceScreen:
    """Starea curenta -> ecranul. `render()` e pur, ca sa poata fi testat.

    Nu tine nicio stare proprie in afara ultimului verdict de handover:
    totul vine din `snapshot`, ca ecranul sa nu poata ramane in urma
    realitatii."""

    def __init__(self, printer=print, width=None):
        self.printer = printer
        self.width = width
        self.handover_verdict = None      # (acceptat: bool, motiv, t)

    def note_handover(self, acceptat, motiv, t=None):
        self.handover_verdict = (acceptat, motiv, t)

    # -- evaluare ----------------------------------------------------------
    @staticmethod
    def verdict(snap):
        """(text, culoare) - randul de sus, cel care se citeste primul.

        Ordinea conteaza: se raporteaza cel mai grav lucru, nu primul gasit.
        Un operator care vede `GATA` trebuie sa poata sa nu mai citeasca
        restul."""
        if snap.get('preflight_ok') is False:
            return 'NU ZBURA - PREFLIGHT PICAT', 'rosu'
        if not snap.get('link_healthy', True):
            return 'NU ZBURA - LEGATURA CU FC CAZUTA', 'rosu'
        temp = snap.get('temp_c')
        if temp is not None and temp >= TEMP_CRIT_C:
            return f'NU ZBURA - {temp:.0f} C, FIRMWARE LIMITEAZA', 'rosu'
        disk = snap.get('disk_mb')
        if disk is not None and disk < DISK_CRIT_MB:
            return f'NU ZBURA - CARD PLIN ({disk:.0f} MB)', 'rosu'
        if snap.get('det_rate') == 0.0 and snap.get('fps'):
            return 'ATENTIE - CAMERA MERGE, MARKER NEVAZUT', 'galben'
        if temp is not None and temp >= TEMP_WARN_C:
            return f'ATENTIE - {temp:.0f} C', 'galben'
        if disk is not None and disk < DISK_WARN_MB:
            return f'ATENTIE - {disk:.0f} MB liberi', 'galben'
        if not snap.get('autonomy_enabled'):
            # NU e o eroare: E0 e inchis pana trece E2. Dar operatorul
            # trebuie sa stie, altfel asteapta un handover care nu vine.
            return 'GATA - MONITOR (autonomie OPRITA, E0)', 'albastru'
        return 'GATA - AUTONOMIE ARMATA', 'verde'

    # -- desenare ----------------------------------------------------------
    def render(self, snap):
        w = self.width or term_width()
        text, culoare = self.verdict(snap)
        out = [bar(spaced(text), culoare, w), '']

        stare = snap.get('state', '-')
        out.append(line('STARE', spaced(stare), 'alb', w))

        px = snap.get('marker_px')
        varsta = snap.get('det_age_s')
        if px is None:
            det = f"{FG['rosu']}FARA MARKER{RESET}"
        else:
            cul = 'verde' if (varsta is not None and varsta < 0.5) else 'galben'
            det = (f"{FG[cul]}{px:.0f} px, acum "
                   f"{_fmt(varsta, '.1f', ' s')}{RESET}")
        out.append(line('MARKER', det, 'alb', w))

        link_ok = snap.get('link_healthy', True)
        hb = snap.get('hb_age_s')
        out.append(line('LEGATURA FC',
                        'OK  ' + _fmt(hb, '.1f', ' s')
                        if link_ok else 'CAZUTA',
                        'verde' if link_ok else 'rosu', w))

        fps = snap.get('fps')
        rate = snap.get('det_rate')
        out.append(line('CAMERA',
                        f"{_fmt(fps, '.0f', ' fps')}   detectie "
                        f"{'-' if rate is None else f'{rate:.0%}'}",
                        'verde' if (fps or 0) >= 20 else 'galben', w))

        temp = snap.get('temp_c')
        c_temp = ('rosu' if temp is not None and temp >= TEMP_CRIT_C
                  else 'galben' if temp is not None and temp >= TEMP_WARN_C
                  else 'verde')
        disk = snap.get('disk_mb')
        c_disk = ('rosu' if disk is not None and disk < DISK_CRIT_MB
                  else 'galben' if disk is not None and disk < DISK_WARN_MB
                  else 'verde')
        out.append(line('TEMPERATURA', _fmt(temp, '.0f', ' C'), c_temp, w))
        out.append(line('CARD', _fmt(disk, '.0f', ' MB liberi'), c_disk, w))
        out.append(line('AUTONOMIE',
                        'ARMATA' if snap.get('autonomy_enabled')
                        else 'OPRITA (E0)',
                        'verde' if snap.get('autonomy_enabled') else 'gri', w))

        if self.handover_verdict is not None:
            acceptat, motiv, _t = self.handover_verdict
            out.append('')
            if acceptat:
                out.append(bar(spaced('HANDOVER ACCEPTAT'), 'verde', w))
            else:
                out.append(bar(spaced('HANDOVER REFUZAT'), 'rosu', w))
                # Motivul e tot rostul randului. Un refuz fara motiv nu se
                # poate repara in cele cateva secunde dintre incercari.
                out.append(f"{BOLD}  {motiv}{RESET}")
        return out

    def draw(self, snap):
        self.printer(CLEAR + '\n'.join(self.render(snap)))


def snapshot(cfg, sm=None, detector=None, vehicle=None, preflight_ok=None,
             last_det=None, now=None):
    """Aduna starea din piesele care exista. Ce lipseste ramane None.

    Deliberat tolerant: ecranul trebuie sa se deseneze si cand detectorul
    inca porneste, sau cand FC-ul nu e conectat. Un ecran care arunca fiindca
    o piesa lipseste e mai rau decat unul cu liniute."""
    snap = {
        'preflight_ok': preflight_ok,
        'autonomy_enabled': cfg.get('autonomy_enabled') is True,
        'state': getattr(sm, 'state', None) or '-',
        'temp_c': cpu_temp_c(),
        'disk_mb': disk_free_mb(),
    }
    if detector is not None:
        try:
            st = detector.stats()
            snap['fps'] = st.get('fps')
            snap['det_rate'] = st.get('detection_rate')
        except Exception:                                    # noqa: BLE001
            pass
    if last_det is not None and now is not None:
        snap['marker_px'] = last_det.marker_px
        snap['det_age_s'] = now - last_det.t
    if vehicle is not None:
        try:
            snap['link_healthy'] = bool(vehicle.link_healthy)
            snap['hb_age_s'] = vehicle.time_since_heartbeat()
        except Exception:                                    # noqa: BLE001
            pass
    return snap
