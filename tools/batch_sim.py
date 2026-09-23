#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Campanie de rulari in Gazebo, cu conditii variate (I4).

    ~/nova-sim-venv/bin/python tools/batch_sim.py --n 20
    ~/nova-sim-venv/bin/python tools/batch_sim.py --plan-only --n 50 --seed 7
    ~/nova-sim-venv/bin/python tools/batch_sim.py --n 5 --dry-run

Fiecare rulare primeste conditii trase dintr-un plan reproductibil:

    pozitia markerului    disc in jurul punctului de decolare (vezi mai jos)
    altitudinea handover  5-12 m (fereastra portii, §8)
    vant                  SIM_WIND_SPD 0-6 m/s, SIM_WIND_TURB pana la 15
    unghi de lumina       azimut 0-360 deg, elevatie 15-75 deg
    roughness marker      0.9 (mata) si 0.3 (lucios, 15.4.7), echilibrat

Scrie un CSV cu o linie pe rulare si raporteaza **p50 si p95**, nu media.
O medie ascunde exact coada care decide daca o incercare pica, iar evidenta
de tip `A - Analysis` se puncteaza pe verificabilitate (8.4.2).

VEHICULUL AJUNGE ZBURAND, NU DE SUS

Prima varianta il lasa sa decoleze vertical si sa planeze: markerul era
deplasat, vehiculul stationar. Geometric identic, si gresit - "nu exista un
zbor lateral care sa introduca propria lui tranzitorie" era motivul scris
atunci, iar tranzitoria aia e exact ce trebuie testat. In cursa pilotul
ajunge la punctul de handover ZBURAND; viteza laterala reziduala din acel
moment e conditia initiala pe care segmentul autonom trebuie sa o anuleze,
si e cea mai probabila sursa de oscilatie de pendul (vezi
docs/DIAGNOSTIC_OSCILATIE.md).

Deci doua pozitii independente, nu una:

    markerul       la `raza_marker` de punctul de decolare - unde sta in lume
    handover-ul    la `raza_m` de MARKER, pe un azimut propriu

Vehiculul decoleaza de acasa si zboara la punctul de handover, deci directia
de apropiere nu e aliniata cu offsetul final - nu vine "perfect drept" peste
marker. `--no-approach` reproduce comportamentul vechi, pentru comparatie.

Ce NU s-a schimbat: `raza_m` ramane plafonat de conul camerei (§5.37).
Masurat pe randare sintetica, la 12 m si 6.5 m lateral markerul are 37 px si
inca ~127 px de margine pana la marginea cadrului - deci plafonul e
conservator, nu la limita.

REPRODUCTIBILITATE

`--seed` fixeaza planul. Aceeasi samanta, aceleasi conditii, in aceeasi
ordine - deci o rulare care a picat se reia identic cu `--only <index>`.
Planul se poate si doar tipari (`--plan-only`), fara sa porneasca nimic.

CE INSEAMNA "ESEC" AICI

Un handover refuzat NU e un bug: poarta a facut ce trebuie. Motivul intra
in CSV si se numara separat, pentru ca distributia refuzurilor e ea insasi
un rezultat. Ce nu are voie sa se intample e o rulare care se termina in
alta stare decat HANDBACK dupa ce poarta a acceptat.
"""

import argparse
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARDUPILOT_DIR = os.path.expanduser('~/ardupilot')

#: §8: poarta refuza peste 6.5 m de marker si in afara ferestrei 5-12 m.
MARKER_RADIUS_M = 6.5
ALT_MIN_M, ALT_MAX_M = 5.0, 12.0

#: Jumatate de VFOV, din fisa tehnica (67 deg). Folosita doar ca sa nu
#: planificam rulari in care markerul nu e in cadru la handover - vezi
#: `raza_max`. Pentru detectia propriu-zisa conteaza calibrarea, nu asta.
HALF_VFOV_DEG = 33.5
MARKER_HALF_M = 0.24           # latura codata 480 mm / 2

#: Cat unghi de inclinare lasam DISPONIBIL pentru corectia laterala.
#:
#: Nu e o marja de siguranta, e o resursa: vehiculul se inclina ca sa
#: corecteze, iar inclinarea muta amprenta camerei in directia gresita. Un
#: plan care pune markerul chiar la limita cadrului nu lasa niciun grad, si
#: secventa pica din prima corectie - masurat: handover la 7.17 m cu 2.95 m
#: lateral, admis 14.0 grade, folosit 19.3, marker iesit din cadru (§5.48).
TILT_BUDGET_DEG = 15.0

#: Sub ce altitudine coborarea devine verticala (FINAL_DESCENT).
#:
#: Implicitul din `SequenceConfig` e 0.40 m, derivat din pragul de nadir al
#: §5.2. Masurat, prea jos: detectia moare INAINTE, din cauza rotatiei
#: markerului in cadru (§5.49), iar supervizorul - care monitorizeaza
#: detectia pana in FINAL_DESCENT - abortaza. 11 esecuri din 18 in prima
#: campanie, toate intre 0.46 si 0.56 m, toate cu centrarea perfecta.
#:
#: Campania NU mai suprascrie pragul. Cat timp l-a suprascris (0.60 m fata
#: de 0.40 m din cod), cifrele masurate descriau o configuratie pe care
#: vehiculul nu ar fi zburat-o niciodata - iar scopul rundei 8 e exact
#: invers: ce se masoara sa fie ce zboara. Trecerea in FINAL_DESCENT se
#: decide acum pe INCADRARE (`SequenceConfig.final_fill`), deci pragul de
#: altitudine a redevenit ce trebuia sa fie: o plasa, nu criteriul.
#:
#: Ramane reglabil din linia de comanda, pentru experimente cu o singura
#: variabila (§5.40).

#: Cat de departe de punctul de decolare sta markerul. Nu e o limita de
#: regulament - e lungimea piciorului de zbor dinainte de handover. Destul
#: cat vehiculul sa aiba viteza reala cand ajunge, destul de scurt cat o
#: campanie de 20 de rulari sa nu tina o zi.
MARKER_PLACEMENT_MAX_M = 15.0
MARKER_PLACEMENT_MIN_M = 4.0

WIND_SPD_MAX = 6.0
WIND_TURB_MAX = 15.0
SUN_EL_MIN, SUN_EL_MAX = 15.0, 75.0
ROUGHNESS = (0.9, 0.3)

#: Porturi dedicate campaniei, ca sa nu se bata cu o sesiune interactiva
#: pornita din start_sim.sh (14550/14552/14553/14554).
PORT_FLY = 14560
PORT_HANDOVER = 14561
PORT_SIM = 14562

#: Codurile lui sim_fly_to.py, traduse in ce scrie in CSV. Un singur
#: "decolarea a esuat" ar amesteca un SITL care inca compileaza cu un prearm
#: respins - si distributia motivelor e ea insasi un rezultat.
MOTIV_FLY = {
    1: 'decolare: nu a ajuns la tinta',
    2: 'decolare: niciun HEARTBEAT de la SITL',
    3: 'decolare: nu s-a armat (prearm / EKF)',
}

CSV_HEADER = [
    'idx', 'seed', 'marker_n', 'marker_e', 'raza_marker_m',
    'ho_n', 'ho_e', 'dist_zbor_m', 'raza_m', 'alt_handover',
    'wind_spd', 'wind_turb', 'sun_az', 'sun_el', 'roughness',
    'succes', 'motiv', 'stare_finala',
    'eroare_finala_cm', 'deriva_cm', 'alt_scoring_m', 'scoring_px',
    'scoring_fill',
    't_descend_s', 't_final_s', 't_touchdown_s', 't_ascent_s', 't_total_s',
    'rata_detectie', 'range_p95', 'angle_p95', 'lat_p99_ms', 'n_detectii',
    'gnss_conform', 'imagini_8_3_3',
]


# --- planul -----------------------------------------------------------------

def raza_max(alt_m, tilt_budget_deg=TILT_BUDGET_DEG):
    """Cat de departe poate fi markerul la handover, ca secventa sa fie
    RECUPERABILA - nu doar ca markerul sa incapa in cadru stand drept.

        d_max = h * (tan(VFOV/2) - tan(buget_inclinare)) - 0.24

    Prima varianta folosea `0.9 * h * tan(VFOV/2) - 0.24`, adica "incape cu
    10% marja, la NADIR". Masurat, e prea permisiv: la limita ei bugetul de
    inclinare ramas e ~4 grade, iar prima corectie laterala il depaseste
    imediat. Rularea care a scos asta la iveala: handover la 7.17 m cu
    markerul la 2.95 m: cadrul admitea 14.0 grade, vehiculul a folosit 19.3,
    markerul a iesit si supervizorul a comandat BRAKE (§5.48).

    | altitudine | d_max nou | vechi | limita portii |
    |---|---|---|---|
    | 5 m  | 1.9 m | 2.7 m | 6.5 m |
    | 8 m  | 3.2 m | 4.5 m | 6.5 m |
    | 12 m | 4.9 m | 6.5 m | 6.5 m |

    Coloana din dreapta e cea care ramane deschisa: poarta accepta 6.5 m la
    orice altitudine din fereastra, dar nu exista altitudine la care 6.5 m
    sa fie recuperabil. Element deschis 28."""
    k = math.tan(math.radians(HALF_VFOV_DEG))
    d = alt_m * (k - math.tan(math.radians(tilt_budget_deg))) - MARKER_HALF_M
    return max(0.0, min(MARKER_RADIUS_M, d))


def plan(n, seed=0):
    """[dict] cu conditiile fiecarei rulari. Determinist pentru o samanta.

    Doua alegeri care nu sunt evidente:

    - Pozitia markerului e uniforma **pe disc**, nu pe (raza, unghi). Tras
      naiv, `r = U(0, R)` aglomereaza punctele in centru si campania ar
      testa mai ales cazul usor; `r = R*sqrt(U)` da densitate uniforma.
    - `roughness` are doua valori, deci nu se trage independent: pe 4 rulari,
      `choice` poate da 4x aceeasi valoare si conditia ramane netestata.
      Lista e echilibrata si apoi amestecata."""
    rng = random.Random(seed)
    rough = [ROUGHNESS[i % len(ROUGHNESS)] for i in range(n)]
    rng.shuffle(rough)
    out = []
    for i in range(n):
        # Rotunjit INAINTE de a calcula raza: altfel conditia scrisa in CSV
        # nu mai e consistenta cu ea insasi (raza trasa pentru 8.294 m,
        # raportata langa 8.29 m, poate depasi limita celei raportate).
        alt = round(rng.uniform(ALT_MIN_M, ALT_MAX_M), 2)

        # 1. unde sta markerul in lume, fata de punctul de decolare
        rm = rng.uniform(MARKER_PLACEMENT_MIN_M, MARKER_PLACEMENT_MAX_M)
        thm = rng.uniform(0, 2 * math.pi)
        m_n, m_e = rm * math.cos(thm), rm * math.sin(thm)

        # 2. unde e vehiculul la handover, fata de MARKER. Azimut propriu,
        #    deci directia de apropiere (acasa -> handover) nu e aliniata cu
        #    offsetul final: vehiculul nu vine drept peste marker.
        r = raza_max(alt) * math.sqrt(rng.random())
        th = rng.uniform(0, 2 * math.pi)
        ho_n, ho_e = m_n + r * math.cos(th), m_e + r * math.sin(th)

        out.append({
            'idx': i,
            'seed': seed,
            'marker_n': round(m_n, 3),
            'marker_e': round(m_e, 3),
            'raza_marker_m': round(rm, 3),
            'ho_n': round(ho_n, 3),
            'ho_e': round(ho_e, 3),
            'dist_zbor_m': round(math.hypot(ho_n, ho_e), 3),
            'raza_m': round(r, 3),
            'alt_handover': alt,
            'wind_spd': round(rng.uniform(0.0, WIND_SPD_MAX), 2),
            'wind_turb': round(rng.uniform(0.0, WIND_TURB_MAX), 1),
            'sun_az': round(rng.uniform(0.0, 360.0), 1),
            'sun_el': round(rng.uniform(SUN_EL_MIN, SUN_EL_MAX), 1),
            'roughness': rough[i],
        })
    return out


def plan_summary(p):
    """Acopera planul chiar plaja ceruta? Tiparit inainte de orice rulare."""
    if not p:
        return {}

    def rng_of(k):
        v = [x[k] for x in p]
        return min(v), max(v)

    razele = [x['raza_m'] for x in p]
    zbor = [x['dist_zbor_m'] for x in p]
    return {
        'n': len(p),
        'raza_max': max(razele), 'raza_medie': sum(razele) / len(razele),
        'zbor': (min(zbor), max(zbor)),
        'marker': rng_of('raza_marker_m'),
        'alt': rng_of('alt_handover'),
        'vant': rng_of('wind_spd'), 'turb': rng_of('wind_turb'),
        'az': rng_of('sun_az'), 'el': rng_of('sun_el'),
        'roughness': sorted({x['roughness'] for x in p}),
    }


# --- comenzile (functii pure, ca sa fie testabile fara Gazebo) --------------

def gz_cmd(world):
    """Gazebo cu GUI, deliberat: headless nu randeaza (§5.31,-r ruleaza)."""
    return ['gz', 'sim', '-v', '1', '-r', world]


def sitl_cmd(param_file, wipe=True):
    out = [f'--out=udp:127.0.0.1:{PORT_FLY}',
           f'--out=udp:127.0.0.1:{PORT_HANDOVER}',
           f'--out=udp:127.0.0.1:{PORT_SIM}']
    c = [os.path.join(ARDUPILOT_DIR, 'Tools', 'autotest', 'sim_vehicle.py'),
         '-v', 'ArduCopter', '-f', 'gazebo-iris', '--model', 'JSON',
         f'--add-param-file={param_file}',
         '--mavproxy-args=--daemon']
    if wipe:
        # §5.13: fara -w, --add-param-file doar seteaza valori implicite,
        # iar eeprom-ul pastreaza ce era. Vantul rulari precedente ar
        # ramane setat si campania ar masura altceva decat crede.
        c.append('-w')
    return c + out


def fly_cmd(alt_m, north_m=0.0, east_m=0.0, python=sys.executable):
    """Decolare + zbor pana la punctul de handover.

    `north/east` sunt fata de punctul de decolare, in NED - adica exact
    sistemul in care lucreaza restul codului (§5.31)."""
    return [python, os.path.join(REPO, 'tools', 'sim_fly_to.py'),
            '--conn', f'udpin:127.0.0.1:{PORT_FLY}', '--alt', str(alt_m),
            '--north', str(north_m), '--east', str(east_m),
            # `guided`, nu `loiter`: intre momentul in care fly_to iese si
            # cel in care porneste injectorul RC nu e nimeni pe manse, iar
            # in LOITER throttle-ul simulat al SITL-ului (jos) comanda
            # coborare. Prima campanie a aterizat asa, orb, inainte de
            # handover.
            '--end-mode', 'guided']


#: Secunde de la pornirea injectorului pana la ridicarea AUX. Acopera
#: fereastra de asezare de 1 s a portii plus doua cicluri de detectie.
HANDOVER_AFTER_S = 3.0


def handover_cmd(python=sys.executable, after=HANDOVER_AFTER_S):
    """`--after` NU e optional aici.

    Fara el, `sim_handover.py` cere Enter - potrivit cand il rulezi de mana,
    fatal intr-o campanie: procesul traieste, nu tipareste nimic si asteapta
    la infinit o tasta pe care nu o apasa nimeni. Prima rulare cap-coada a
    ramas blocata exact aici, dupa ce Gazebo, SITL si decolarea trecusera."""
    return [python, os.path.join(REPO, 'tools', 'sim_handover.py'),
            '--conn', f'udpin:127.0.0.1:{PORT_HANDOVER}',
            '--after', str(after)]


def nova_sim_cmd(run_dir, calib, seconds, python=sys.executable,
                 no_lateral_alt=None, authority=False, fast_descent=False,
                 scoring_px=None, no_gnss=False, scoring_fill=None,
                 final_fill=None):
    """Optiunile de experiment se DAU MAI DEPARTE, nu se redeclara aici.

    Campania e doar orchestrare; ce se regleaza, se regleaza in aplicatie.
    Un knob care exista in `nova_sim.py` si nu se poate atinge din campanie
    inseamna ca experimentul se poate face doar de mana, pe o singura
    rulare - adica exact ce nu vrei cand incerci o valoare noua."""
    extra = []
    if scoring_px is not None:
        extra += ['--scoring-px', str(scoring_px)]
    if no_lateral_alt is not None:
        extra += ['--no-lateral-alt', str(no_lateral_alt)]
    if scoring_fill is not None:
        extra += ['--scoring-fill', str(scoring_fill)]
    if final_fill is not None:
        extra += ['--final-fill', str(final_fill)]
    if no_gnss:
        extra.append('--no-gnss')
    if authority:
        extra.append('--authority')
    if fast_descent:
        extra.append('--fast-descent')
    return [python, os.path.join(REPO, 'tools', 'nova_sim.py'),
            '--conn', f'udpin:127.0.0.1:{PORT_SIM}',
            '--calib', calib, '--provisional',
            '--seconds', str(seconds)] + extra + [
            '--csv', os.path.join(run_dir, 'frames.csv'),
            # 8.3.3: imaginile predate juriului, cu evidenta lor.
            '--scoring-dir', os.path.join(run_dir, 'scoring'),
            # Cadrul in care s-a pierdut detectia, daca se pierde. Fara el,
            # "detection_age: BRAKE" nu spune daca markerul a iesit din
            # cadru sau imaginea nu mai e detectabila.
            '--dump-dir', os.path.join(run_dir, 'cadre'),
            '--json', os.path.join(run_dir, 'report.json')]


def param_file_for(cond, run_dir, baza=None):
    """nova_sitl.parm + vantul acestei rulari, intr-un fisier propriu."""
    baza = baza or os.path.join(REPO, 'config', 'nova_sitl.parm')
    with open(baza) as f:
        text = f.read()
    cale = os.path.join(run_dir, 'run.parm')
    with open(cale, 'w') as f:
        f.write(text)
        f.write(f"\n# conditii batch_sim, rularea {cond['idx']}\n")
        f.write(f"SIM_WIND_SPD,{cond['wind_spd']}\n")
        f.write(f"SIM_WIND_TURB,{cond['wind_turb']}\n")
    return cale


# --- procese ----------------------------------------------------------------

def cleanup(verbose=True):
    """Procesele ramase din rularea anterioara.

    §5.5: un `gz sim` zombie tine porturile 9002/9003 si SITL-ul urmator da
    timeout; §5.33: un al doilea server randeaza in paralel si contamineaza
    orice masuratoare."""
    for tipar in ('arducopter', 'gz sim', 'mavproxy', 'nova_sim.py',
                  'sim_fly_to.py', 'sim_handover.py'):
        subprocess.run(['pkill', '-f', tipar], capture_output=True)
    if verbose:
        print("    curatat")
    time.sleep(2.0)


def spawn(cmd, log_path, cwd=None, env=None):
    """Proces in propriul grup, ca sa se poata omori cu tot cu copii.

    §5.33: `terminate()` pe lansator lasa in viata `gz sim server` si
    `gz sim gui`, care sunt forkate de el.

    Doua detalii care par cosmetice si nu sunt:

    - **`stdin` inchis.** Niciun copil nu are voie sa astepte o tasta: intr-o
      campanie nu e nimeni la tastatura. Inchis, un `input()` da EOFError -
      adica esec vizibil - in loc de asteptare tacuta la infinit.
    - **`PYTHONUNBUFFERED`.** Cu iesirea redirectata intr-un fisier, Python
      tamponeaza pe blocuri: logul ramane GOL pana la iesirea procesului.
      Tocmai in timpul unei rulari care pare blocata ai nevoie sa vezi unde
      a ajuns."""
    f = open(log_path, 'w')
    mediu = dict(env if env is not None else os.environ)
    mediu['PYTHONUNBUFFERED'] = '1'
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=cwd,
                         env=mediu, stdin=subprocess.DEVNULL,
                         start_new_session=True)
    p._nova_log = f
    return p


def kill_group(p, timeout=5.0):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return
    t0 = time.time()
    while time.time() - t0 < timeout:
        if p.poll() is not None:
            break
        time.sleep(0.2)
    else:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    log = getattr(p, '_nova_log', None)
    if log is not None:
        try:
            log.close()
        except OSError:
            pass


def wait_topic(topic, timeout=60.0):
    """Asteapta ca un topic gz sa publice CHIAR date.

    §5.33: un topic ANUNTAT nu inseamna date. `gz topic -l` il arata si
    cand randarea nu se intampla deloc."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', '1'],
                           capture_output=True, text=True, timeout=10)
        if r.returncode == 0 and r.stdout.strip():
            return True
        time.sleep(2.0)
    return False


# --- o rulare ---------------------------------------------------------------

def build_world(cond, out_dir, calib, python=sys.executable):
    """Lumea si modelele pentru aceste conditii. (cale_lume, mesaj)."""
    cmd = [python, os.path.join(REPO, 'tools', 'make_marker_model.py'),
           '--out', out_dir,
           '--north', str(cond['marker_n']), '--east', str(cond['marker_e']),
           '--sun-az', str(cond['sun_az']), '--sun-el', str(cond['sun_el']),
           '--roughness', str(cond['roughness'])]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        return None, f"make_marker_model: {r.stderr.strip()[:200]}"
    cmd2 = [python, os.path.join(REPO, 'tools', 'make_camera_model.py'),
            '--out', out_dir, '--calib', calib, '--provisional']
    r2 = subprocess.run(cmd2, capture_output=True, text=True, timeout=300)
    if r2.returncode != 0:
        return None, f"make_camera_model: {r2.stderr.strip()[:200]}"
    return os.path.join(out_dir, 'worlds', 'nova_marker.sdf'), 'ok'


def row_from(cond, motiv, raport=None, succes=False):
    r = {k: '' for k in CSV_HEADER}
    r.update(cond)
    r['succes'] = 1 if succes else 0
    r['motiv'] = motiv
    if raport:
        t = raport.get('t_state') or {}
        r.update({
            'stare_finala': raport.get('stare_finala', ''),
            'eroare_finala_cm': _r(raport.get('eroare_finala_cm'), 2),
            'deriva_cm': _r(raport.get('deriva_cm'), 2),
            'alt_scoring_m': _r(raport.get('alt_scoring_m'), 3),
            'scoring_px': _r(raport.get('scoring_px'), 0),
            'scoring_fill': _r(raport.get('scoring_fill'), 3),
            't_descend_s': _r(t.get('DESCEND_TRACK'), 2),
            't_final_s': _r(t.get('FINAL_DESCENT'), 2),
            't_touchdown_s': _r(t.get('TOUCHDOWN_CONFIRM'), 2),
            't_ascent_s': _r(t.get('ASCENT'), 2),
            't_total_s': _r(sum(v for v in t.values()), 2) if t else '',
            'rata_detectie': _r(raport.get('rata_detectie'), 4),
            'range_p95': _r(raport.get('range_p95'), 6),
            'angle_p95': _r(raport.get('angle_p95'), 4),
            'lat_p99_ms': _r(raport.get('lat_p99_ms'), 2),
            'n_detectii': raport.get('n_detectii', ''),
            'gnss_conform': (1 if (raport.get('gnss') or {}).get('ack_ok')
                             else 0) if raport.get('gnss') else '',
            'imagini_8_3_3': (1 if (raport.get('imagini') or {})
                              .get('complet_8_3_3') else 0)
            if raport.get('imagini') else '',
        })
    return r


def _r(v, n):
    return '' if v is None else round(v, n)


def run_one(cond, run_dir, lume, calib, seconds, python=sys.executable,
            gz_timeout=90.0, verbose=True, approach=True, sim_opts=None):
    """O rulare completa. Intoarce randul de CSV.

    NETESTAT PE HARDWARE: secventa de mai jos nu a fost niciodata rulata
    cap-coada - mediul in care a fost scrisa nu poate tine un server Gazebo
    in viata (`libEGL: failed to create dri2 screen`). Fiecare pas e
    verificat separat; ORDINEA lor nu."""
    gz = sitl = sim = hand = None
    try:
        cleanup(verbose=False)
        # Mediul se construieste pentru FIECARE rulare, nu se acumuleaza in
        # os.environ: altfel dupa 20 de rulari variabila are 20 de intrari,
        # iar modelele rularilor vechi raman pe cale. Lumea generata
        # foloseste oricum cai absolute (§5.31); intrarea de aici e doar
        # pentru cine ar scrie `model://`.
        env = dict(os.environ)
        env['PYTHONUNBUFFERED'] = '1'
        env['GZ_SIM_RESOURCE_PATH'] = (
            os.path.join(run_dir, 'models') + os.pathsep
            + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

        gz = spawn(gz_cmd(lume), os.path.join(run_dir, 'gazebo.log'), env=env)
        if verbose:
            print("    Gazebo pornit, astept cadre...")
        if not wait_topic('/down_cam/camera_info', timeout=gz_timeout):
            return row_from(cond, 'gazebo: camera nu publica (randare?)')

        parm = param_file_for(cond, run_dir)
        sitl = spawn(sitl_cmd(parm), os.path.join(run_dir, 'sitl.log'),
                     cwd=ARDUPILOT_DIR, env=env)
        if verbose:
            print("    SITL pornit, decolez...")

        r = subprocess.run(fly_cmd(cond['alt_handover'],
                                   cond.get('ho_n', 0.0) if approach else 0.0,
                                   cond.get('ho_e', 0.0) if approach else 0.0,
                                   python),
                           capture_output=True, text=True, timeout=300,
                           stdin=subprocess.DEVNULL, env=env)
        with open(os.path.join(run_dir, 'fly.log'), 'w') as f:
            f.write(r.stdout + r.stderr)
        if r.returncode != 0:
            return row_from(cond, MOTIV_FLY.get(
                r.returncode, f"decolare: cod {r.returncode}"))

        sim = spawn(nova_sim_cmd(run_dir, calib, seconds, python,
                                 **(sim_opts or {})),
                    os.path.join(run_dir, 'nova_sim.log'))
        time.sleep(5.0)          # detectorul sa apuce sa vada markerul

        # Pornit in fundal si TINUT pornit pana la final, nu rulat si
        # inchis: `sim_handover.py` injecteaza continuu manse in neutru,
        # iar daca se opreste, override-ul expira dupa RC_OVERRIDE_TIME si
        # FC-ul revine la RC-ul simulat. Manetele lui nu sunt in neutrul
        # memorat la handover, deci monitorul de override ar vedea un pilot
        # care preia - si ar opri secventa in fiecare rulare, din unealta
        # de test, nu din ce testam.
        hand = spawn(handover_cmd(python),
                     os.path.join(run_dir, 'handover.log'))

        sim.wait(timeout=seconds * 4 + 120)
        cale_json = os.path.join(run_dir, 'report.json')
        if not os.path.exists(cale_json):
            return row_from(cond, 'nova_sim: fara raport (vezi nova_sim.log)')
        with open(cale_json) as f:
            raport = json.load(f)
        # Succesul cere SI captura de scoring, nu doar HANDBACK. Fara ea,
        # campania raporta 100% pe rulari care NU indeplineau 8.3.3 - adica
        # exact cerinta care aduce puncte (§5.51). Un criteriu de succes
        # care nu poate detecta lipsa lucrului masurat nu e criteriu.
        ajuns = bool(raport.get('succes'))
        capturat = bool(raport.get('scoring_ok'))
        succes = ajuns and capturat
        if succes:
            motiv = 'ok'
        elif not ajuns:
            motiv = f"oprit in {raport.get('stare_finala')}"
        else:
            motiv = 'HANDBACK dar FARA captura de scoring (8.3.3)'
        return row_from(cond, motiv, raport, succes)

    except subprocess.TimeoutExpired:
        return row_from(cond, 'timeout')
    except OSError as e:
        return row_from(cond, f"eroare de proces: {e}")
    finally:
        for p in (sim, hand, sitl, gz):
            kill_group(p)


# --- statistici -------------------------------------------------------------

def pct(vals, p):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return None
    k = (len(v) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def summarize(rows):
    """p50 si p95 peste rulari, plus rata de succes si motivele esecurilor."""
    reusite = [r for r in rows if r.get('succes')]
    esecuri = [r for r in rows if not r.get('succes')]
    motive = {}
    for r in esecuri:
        m = r.get('motiv') or 'necunoscut'
        motive[m] = motive.get(m, 0) + 1

    def col(k):
        out = []
        for r in reusite:
            v = r.get(k)
            if v in ('', None):
                continue
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                pass
        return out

    s = {'n': len(rows), 'reusite': len(reusite), 'esecuri': len(esecuri),
         'motive': motive}
    if rows:
        s['rata_succes'] = len(reusite) / len(rows)
    for k in ('eroare_finala_cm', 'deriva_cm', 'alt_scoring_m', 't_total_s',
              'rata_detectie', 'range_p95', 'angle_p95', 'lat_p99_ms'):
        v = col(k)
        s[k] = {'p50': pct(v, 50), 'p95': pct(v, 95),
                'min': min(v) if v else None, 'max': max(v) if v else None,
                'n': len(v)}
    return s


def print_summary(s):
    print(f"\n  --- {s['n']} rulari: {s['reusite']} reusite, "
          f"{s['esecuri']} esecuri ---")
    if s.get('rata_succes') is not None:
        print(f"    rata de succes: {s['rata_succes']:.0%}")
    if s['motive']:
        print("    motive de esec:")
        for m, k in sorted(s['motive'].items(), key=lambda x: -x[1]):
            print(f"      {k:>3}x  {m}")
    print(f"\n    {'metrica':<20} {'p50':>10} {'p95':>10}   (n)")
    etichete = [
        ('eroare_finala_cm', 'eroare finala (cm)'),
        ('deriva_cm', 'deriva (cm)'),
        ('alt_scoring_m', 'alt captura (m)'),
        ('scoring_px', 'captura (px)'),
        ('scoring_fill', 'captura (incadrare)'),
        ('t_total_s', 'durata (s)'),
        ('rata_detectie', 'rata detectie'),
        ('range_p95', 'eroare range p95/rul'),
        ('angle_p95', 'eroare unghi p95/rul'),
        ('lat_p99_ms', 'coada p99 (ms)'),
    ]
    for k, et in etichete:
        d = s.get(k) or {}
        if not d.get('n'):
            continue
        p50 = '-' if d['p50'] is None else f"{d['p50']:.3f}"
        p95 = '-' if d['p95'] is None else f"{d['p95']:.3f}"
        print(f"    {et:<20} {p50:>10} {p95:>10}   ({d['n']})")
    print("\n    p50 si p95, nu media: media ascunde coada care decide daca o")
    print("    incercare pica. Evidenta A - Analysis pentru 8.4.2.")
    print("\n    ATENTIE la ultimele doua: valoarea dintr-o rulare e deja un")
    print("    p95 pe cadrele ei, iar coloanele de mai sus sunt p50 si p95")
    print("    PESTE RULARI. Deci 'p50' acolo inseamna 'rularea mediana, la")
    print("    percentila 95 a ei' - o coada, nu o valoare tipica.")


# --- CLI --------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--n', type=int, default=10)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out', default=os.path.join(REPO, 'data', 'batch'))
    p.add_argument('--calib', default=os.path.join(REPO, 'config',
                                                   'camera_sim.yaml'))
    p.add_argument('--seconds', type=float, default=120.0,
                   help='durata maxima a unei rulari, in timp de simulare')
    p.add_argument('--only', type=int, default=None,
                   help='ruleaza doar indexul dat din plan')
    p.add_argument('--plan-only', action='store_true',
                   help='tipareste planul si iesi')
    p.add_argument('--dry-run', action='store_true',
                   help='genereaza lumile, nu porneste Gazebo')
    p.add_argument('--scoring-px', type=float, default=None,
                   help='prag de REZERVA in pixeli pentru captura, folosit '
                        'doar cand detectorul nu raporteaza incadrarea')
    p.add_argument('--scoring-fill', type=float, default=None,
                   help='cat din cadru trebuie sa ocupe cutia markerului ca '
                        'sa se faca captura 8.3.3 (§5.49, §5.51, §5.54)')
    p.add_argument('--final-fill', type=float, default=None,
                   help='incadrarea la care coborarea devine verticala. '
                        'Trebuie > --scoring-fill')
    p.add_argument('--no-lateral-alt', type=float, default=None,
                   help='plasa de altitudine sub care coborarea devine '
                        'verticala chiar fara semnal de incadrare. Implicit '
                        'cel din SequenceConfig - campania nu il mai '
                        'suprascrie, ca sa masoare ce zboara')
    p.add_argument('--no-gnss', action='store_true',
                   help='15.2.5: segmentul autonom ruleaza pe setul de surse '
                        'EKF fara GNSS')
    p.add_argument('--authority', action='store_true',
                   help='modularea de autoritate pe praguri de altitudine '
                        '(I5): limiteaza WP_ACC si viteza sub 3 m')
    p.add_argument('--fast-descent', action='store_true',
                   help='PROFIL_RAPID; cere --authority si distanta de '
                        'franare masurata pe fiecare treapta')
    p.add_argument('--no-approach', action='store_true',
                   help='vehiculul planeaza deasupra punctului de decolare '
                        'in loc sa zboare la handover (comportamentul vechi, '
                        'pentru comparatie)')
    p.add_argument('--python', default=sys.executable)
    a = p.parse_args(argv)

    sim_opts = {'scoring_px': a.scoring_px, 'no_gnss': a.no_gnss,
                'no_lateral_alt': a.no_lateral_alt,
                'scoring_fill': a.scoring_fill,
                'final_fill': a.final_fill,
                'authority': a.authority,
                'fast_descent': a.fast_descent}

    conditii = plan(a.n, a.seed)
    if a.only is not None:
        conditii = [c for c in conditii if c['idx'] == a.only]
        if not conditii:
            print(f"  indexul {a.only} nu e in plan (0..{a.n - 1})")
            return 1

    s = plan_summary(plan(a.n, a.seed))
    print(f"\n  plan: {s['n']} rulari, samanta {a.seed}")
    print(f"    marker in lume: {s['marker'][0]:.1f}-{s['marker'][1]:.1f} m "
          f"de punctul de decolare")
    print(f"    zbor pana la handover: {s['zbor'][0]:.1f}-{s['zbor'][1]:.1f} m "
          f"(vehiculul ajunge in miscare, nu planand)")
    print(f"    offset la handover: pana la {s['raza_max']:.2f} m de marker "
          f"(medie {s['raza_medie']:.2f}), plafonat de cadrul camerei")
    print(f"    handover: {s['alt'][0]:.1f}-{s['alt'][1]:.1f} m")
    print(f"    vant: {s['vant'][0]:.1f}-{s['vant'][1]:.1f} m/s, "
          f"turbulenta {s['turb'][0]:.0f}-{s['turb'][1]:.0f}")
    print(f"    soare: az {s['az'][0]:.0f}-{s['az'][1]:.0f} deg, "
          f"el {s['el'][0]:.0f}-{s['el'][1]:.0f} deg")
    print(f"    roughness: {s['roughness']}")
    schimbate = [k for k, v in sim_opts.items() if v]
    if schimbate:
        print(f"    EXPERIMENT: {', '.join(schimbate)}")
        if len(schimbate) > 1:
            print(f"    ATENTIE: {len(schimbate)} variabile schimbate odata. "
                  f"Un rezultat diferit nu se va putea atribui niciuneia "
                  f"(§5.40).")

    if a.plan_only:
        print()
        for c in conditii:
            print(f"    {json.dumps(c, sort_keys=True)}")
        print()
        return 0

    os.makedirs(a.out, exist_ok=True)
    ses = os.path.join(a.out, time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(ses, exist_ok=True)
    csv_path = os.path.join(ses, 'runs.csv')

    randuri = []
    for c in conditii:
        print(f"\n  --- rularea {c['idx'] + 1}/{len(conditii)} "
              f"(idx {c['idx']}) ---")
        run_dir = os.path.join(ses, f"run_{c['idx']:03d}")
        os.makedirs(run_dir, exist_ok=True)
        lume, mesaj = build_world(c, run_dir, a.calib, python=a.python)
        if lume is None:
            print(f"    lumea NU s-a generat: {mesaj}")
            randuri.append(row_from(c, f"lume: {mesaj}"))
            continue
        print(f"    lume: {os.path.relpath(lume, ses)}")
        if a.dry_run:
            randuri.append(row_from(c, 'dry-run'))
            continue
        try:
            rand = run_one(c, run_dir, lume, a.calib, a.seconds,
                           python=a.python, approach=not a.no_approach,
                           sim_opts=sim_opts)
        except KeyboardInterrupt:
            # §5.47 la nivelul campaniei: cine opreste nu trebuie sa piarda
            # rezumatul rularilor deja facute. CSV-ul e scris oricum dupa
            # fiecare rulare; aici se salveaza sinteza.
            print("\n  oprit de utilizator; rezumat pe ce s-a rulat:")
            break
        print(f"    -> {'REUSIT' if rand['succes'] else 'ESEC'}: "
              f"{rand['motiv']}")
        randuri.append(rand)

        # CSV scris dupa fiecare rulare, nu la final: o campanie de 20 de
        # rulari tine ore, iar un Ctrl-C la rularea 19 nu are voie sa
        # arunce tot ce s-a masurat.
        _scrie_csv(csv_path, randuri)

    _scrie_csv(csv_path, randuri)
    print_summary(summarize(randuri))
    print(f"\n  CSV: {csv_path}\n")
    return 0


def _scrie_csv(cale, randuri):
    with open(cale, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, extrasaction='ignore')
        w.writeheader()
        for r in randuri:
            w.writerow({k: r.get(k, '') for k in CSV_HEADER})


if __name__ == '__main__':
    sys.exit(main())
