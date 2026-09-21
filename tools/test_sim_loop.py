#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru bucla inchisa in Gazebo (I4).

    python3 tools/test_sim_loop.py

Fara Gazebo, fara SITL, fara pymavlink pe fir: adevarul se injecteaza prin
`StaticTruth`, procesele nu se pornesc niciodata, iar mesajele MAVLink se
verifica pe un dublu care le inregistreaza.

Ce conteaza cel mai mult, in ordine:

  - ORDINEA din `SimApp.step()` fata de `run_loop`. §5.14: un supervizor
    inert nu s-a vazut in nicio suita de piese, pentru ca nimeni nu testa
    cablajul. Aici e al doilea cablaj din proiect, deci exact clasa de bug.
  - offsetul de 1 cm al planului markerului (§5.31), care la 0.5 m inseamna
    2% dintr-un buget de 3%.
  - semnul lui z in SET_POSITION_TARGET_LOCAL_NED: pozitiv inseamna SUB
    home, iar ArduPilot accepta comanda fara sa se planga.
  - planul campaniei acopera chiar plaja pe care scrie ca o acopera.
"""

import ast
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova import sim_truth                                  # noqa: E402
from nova.detection import Detection                        # noqa: E402

import batch_sim                                            # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --- adevarul din simulare --------------------------------------------------

def test_enu_devine_ned():
    p = sim_truth.pose_from_enu(1.5, 2.0, 0.01)
    assert abs(p.north - 2.0) < 1e-9, f"north={p.north}"
    assert abs(p.east - 1.5) < 1e-9, f"east={p.east}"
    assert abs(p.alt - 0.01) < 1e-9, f"alt={p.alt}"
    # Valori asimetrice, deliberat: cu north == east o inversiune ar trece.
    return "ENU(1.5,2.0) -> N=2.0 E=1.5"


def test_planul_markerului_se_scade_din_range():
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0.0, 0.0, 5.01),
        marker=sim_truth.pose_from_enu(0.0, 0.0, 0.01))
    tr = t.truth()
    assert abs(tr['range_m'] - 5.0) < 1e-6, tr['range_m']
    # Si la altitudine mica, unde offsetul conteaza cel mai mult:
    t.set('iris_with_gimbal', sim_truth.pose_from_enu(0, 0, 0.51))
    tr = t.truth()
    assert abs(tr['range_m'] - 0.5) < 1e-6, tr['range_m']
    return "5.01 -> 5.000 si 0.51 -> 0.500"


def test_offsetul_ignorat_ar_fi_2_la_suta():
    """Cifra din §5.31, verificata aici ca sa nu ramana doar in text."""
    alt = 0.51
    eroare = (alt - (alt - sim_truth.MARKER_PLANE_Z)) / (alt - 0.01)
    assert eroare > 0.019, eroare
    assert eroare < 0.021, eroare
    return f"{eroare:.1%} la 0.5 m, dintr-un buget de 3%"


def test_sub_planul_markerului_nu_exista_adevar():
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 0.005),
        marker=sim_truth.pose_from_enu(0, 0, 0.01))
    assert t.truth() is None, "dz <= 0 trebuie sa dea None, nu o cifra"
    return "dz <= 0 -> None"


def test_eroarea_de_range_are_semn():
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.01),
        marker=sim_truth.pose_from_enu(0, 0, 0.01))
    det = Detection(t=0.0, angle_x=0.0, angle_y=0.0, distance_m=5.10,
                    marker_px=90.0, range_m=5.10)
    e = t.error_vs(det)
    assert e['range_rel'] > 0, e['range_rel']
    assert abs(e['range_rel'] - 0.02) < 1e-6, e['range_rel']
    det2 = Detection(t=0.0, angle_x=0.0, angle_y=0.0, distance_m=4.90,
                     marker_px=90.0, range_m=4.90)
    assert t.error_vs(det2)['range_rel'] < 0
    return "supraestimarea da +, subestimarea da -"


def test_eroarea_unghiulara_e_distanta_nu_suma():
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 10.01),
        marker=sim_truth.pose_from_enu(0, 0, 0.01))
    # adevarul e 0 pe ambele axe; detectia greseste cu 3 si 4 grade
    det = Detection(t=0.0, angle_x=math.radians(3.0),
                    angle_y=math.radians(4.0), distance_m=10.0,
                    marker_px=45.0, range_m=10.0)
    e = t.error_vs(det)
    assert abs(e['angle_deg'] - 5.0) < 1e-6, e['angle_deg']
    return "3 si 4 grade -> 5 grade, nu 7"


def test_rata_de_detectie_ignora_ce_e_sub_prag():
    """§5.2: sub 0.38 m markerul nu incape in cadru PRIN CONSTRUCTIE.

    Numarat ca ratare, ar transforma o proprietate fizica a camerei intr-o
    defectiune inventata - exact eroarea din §8 despre
    DETECTION_MONITORED_PHASES."""
    date = [(0.3, False), (0.2, False),        # sub fereastra: nu conteaza
            (5.0, True), (6.0, True), (7.0, True), (8.0, False),
            (20.0, False)]                     # peste fereastra: nu conteaza
    rata, n = sim_truth.detection_rate(date, 3.0, 12.0)
    assert n == 4, f"in fereastra ar trebui 4 cadre, nu {n}"
    assert abs(rata - 0.75) < 1e-9, rata
    return "4 cadre in fereastra, 75%"


def test_rata_fara_cadre_e_None_nu_zero():
    rata, n = sim_truth.detection_rate([(0.2, False)], 3.0, 12.0)
    assert rata is None, "fara cadre in fereastra: 'nu stiu', nu 0%"
    assert n == 0
    return "0 cadre -> None, nu 0.0"


def test_summarize_da_percentile_nu_medie():
    err = [{'range_rel': 0.001, 'angle_deg': 0.01} for _ in range(19)]
    err.append({'range_rel': 0.50, 'angle_deg': 40.0})      # o aberanta
    s = sim_truth.summarize(err)
    medie = sum(abs(e['range_rel']) for e in err) / len(err)
    assert s['range_p50'] < 0.002, s['range_p50']
    assert medie > 0.02, medie
    assert s['range_p50'] < medie / 10, "p50 trebuie sa reziste la o aberanta"
    return f"p50 {s['range_p50']:.4f} vs medie {medie:.4f}"


def test_pragurile_sunt_verificate_nu_doar_raportate():
    bun = [{'range_rel': 0.01, 'angle_deg': 0.1}] * 10
    s = sim_truth.summarize(bun)
    assert s['range_ok'] is True and s['angle_ok'] is True
    rau = [{'range_rel': 0.05, 'angle_deg': 0.9}] * 10
    s2 = sim_truth.summarize(rau)
    assert s2['range_ok'] is False, "5% peste pragul de 3% trebuie sa pice"
    assert s2['angle_ok'] is False, "0.9 deg peste pragul de 0.5 trebuie sa pice"
    return "3% si 0.5 deg chiar resping"


def test_ceasul_stie_sa_spuna_ca_nu_stie():
    """Pana la primul cadru nu exista timp de simulare. O valoare de rezerva
    (ceasul de perete, ~1e4) urmata de primul timp real (~10 s) inseamna un
    SALT INAPOI de patru ordine de marime: `--seconds` nu s-ar mai indeplini
    niciodata si rularea ar atarna."""
    class S:
        last_sim_t = None
    assert sim_truth.latest_sim_t([S(), None]) is None
    import time as _t
    inainte = _t.monotonic()
    caz = sim_truth.now_sim([S()])
    # Comparat cu ceasul de perete, nu cu o cifra. Prima varianta cerea
    # `> 1000.0`, pornind de la ideea ca `time.monotonic()` e mare - ceea ce
    # e adevarat doar cu uptime peste ~17 minute. Dupa o repornire testul
    # pica fara ca nimic sa se fi stricat.
    assert abs(caz - inainte) < 1.0, (
        f"now_sim trebuie sa cada pe ceasul de perete; a dat {caz}, "
        f"monotonic e {inainte}")

    class T:
        last_sim_t = 12.5
    assert sim_truth.latest_sim_t([S(), T()]) == 12.5
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'sim_clock_ok' in src, (
        "bucla trebuie sa-si ia originea abia cand ceasul e real")
    return "None inainte de primul cadru, cifra dupa"


def test_ceasul_e_cel_mai_recent_timp_de_simulare():
    class S:
        def __init__(self, t):
            self.last_sim_t = t
    assert sim_truth.now_sim([S(10.0), S(12.5)]) == 12.5
    assert sim_truth.now_sim([S(12.5), None]) == 12.5
    assert sim_truth.now_sim([None], fallback=lambda: 99.0) == 99.0
    return "max peste surse, fallback cand nu stie nimeni"


# --- cablajul: SimApp fata de run_loop --------------------------------------

def _apeluri(sursa, nume_fn, obiect_self='self'):
    """Secventa de apeluri 'x.metoda(' dintr-o functie, in ordinea sursei."""
    arbore = ast.parse(sursa)
    tinta = None
    for nod in ast.walk(arbore):
        if isinstance(nod, ast.FunctionDef) and nod.name == nume_fn:
            tinta = nod
            break
    assert tinta is not None, f"nu gasesc {nume_fn}"
    out = []
    for nod in ast.walk(tinta):
        if isinstance(nod, ast.Call) and isinstance(nod.func, ast.Attribute):
            baza = nod.func.value
            if isinstance(baza, ast.Name):
                out.append((baza.id, nod.func.attr, nod.lineno))
            elif (isinstance(baza, ast.Attribute)
                  and isinstance(baza.value, ast.Name)
                  and baza.value.id == obiect_self):
                out.append((baza.attr, nod.func.attr, nod.lineno))
    out.sort(key=lambda x: x[2])
    return [(a, b) for a, b, _ in out]


#: Ce trebuie sa se intample, in ordine, in ambele bucle. Numele obiectului
#: difera (`vehicle` vs `v`), pasul nu.
PASI = [
    ('pump', ('vehicle', 'v')),
    ('check_link', ('vehicle', 'v')),
    ('update_params', ('vehicle', 'v')),
    ('poll', ('detector',)),
    ('update', ('supervisor', 'sup')),
    ('update', ('authority',)),
    ('on_detection', ('sm',)),
    ('update', ('sm',)),
]


def _indici(apeluri, pasi):
    idx = []
    for metoda, obiecte in pasi:
        gasit = None
        for i, (obj, m) in enumerate(apeluri):
            if m == metoda and obj in obiecte:
                gasit = i
                break
        assert gasit is not None, f"lipseste {obiecte}.{metoda}()"
        idx.append(gasit)
    return idx


def test_ordinea_din_SimApp_o_oglindeste_pe_run_loop():
    """§5.14, aplicat preventiv.

    Supervizorul a ajuns inert pentru ca aplicatia isi construia singura
    cablajul si nimeni nu compara cele doua. `SimApp.step()` e al doilea
    cablaj: daca ordinea diverge, supervizorul sau modularea de autoritate
    ar vedea o alta stare decat cea pe care o vede masina de stari."""
    sm_src = open(os.path.join(REPO, 'nova', 'state_machine.py')).read()
    sim_src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    a_run = _apeluri(sm_src, 'run_loop')
    a_sim = _apeluri(sim_src, 'step')
    i_run = _indici(a_run, PASI)
    i_sim = _indici(a_sim, PASI)
    assert i_run == sorted(i_run), f"run_loop nu mai e in ordinea asteptata"
    assert i_sim == sorted(i_sim), (
        "SimApp.step() a divergat de run_loop; ordinea gasita: "
        + ', '.join(f"{o}.{m}" for o, m in a_sim))
    return f"{len(PASI)} pasi, aceeasi ordine in ambele"


def test_bucla_de_sim_nu_foloseste_ceasul_de_perete():
    """§5.36: la RTF 0.56, o varsta pe ceas de perete apare de ~1.8x mai
    mare, iar DETECTION_MAX_AGE_S s-ar declansa fals la fiecare coborare.

    `step()` nu are voie sa atinga deloc ceasul de perete. `now()` are, dar
    numai ca valoare de rezerva pana la primul cadru - si numai pe ramura
    care marcheaza ca ceasul NU e inca real."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    arbore = ast.parse(src)

    def apeluri_perete(nod):
        return [sub for sub in ast.walk(nod)
                if isinstance(sub, ast.Call)
                and isinstance(sub.func, ast.Attribute)
                and sub.func.attr in ('monotonic', 'time')]

    fn = {n.name: n for n in ast.walk(arbore)
          if isinstance(n, ast.FunctionDef)}
    assert not apeluri_perete(fn['step']), "step() foloseste ceasul de perete"

    # in now(), fiecare apel de perete trebuie sa fie pe o ramura care
    # stinge sim_clock_ok
    perete = apeluri_perete(fn['now'])
    for ap in perete:
        acoperit = False
        for nod in ast.walk(fn['now']):
            if not isinstance(nod, ast.If):
                continue
            corp = [x for x in ast.walk(nod) if x is ap]
            if not corp:
                continue
            for sub in ast.walk(nod):
                if (isinstance(sub, ast.Attribute)
                        and sub.attr == 'sim_clock_ok'):
                    acoperit = True
        assert acoperit, (
            "now() cade pe ceasul de perete fara sa marcheze sim_clock_ok")
    return f"step() curat, now() marcheaza {len(perete)} cadere pe rezerva"


def test_run_loop_a_ramas_neatins():
    """Regula rundei 7: nova/state_machine.py nu se modifica."""
    import subprocess
    r = subprocess.run(['git', 'diff', '--stat', 'HEAD', '--',
                        'nova/state_machine.py', 'nova/safety.py',
                        'nova/handover.py'],
                       capture_output=True, text=True, cwd=REPO)
    assert not r.stdout.strip(), (
        "runda 7 interzice modificarea acestor fisiere:\n" + r.stdout)
    return "state_machine, safety, handover nemodificate"


def test_E0_e_ocolita_explicit_si_anuntata():
    """§5.16: simularea ocoleste garda, dar o SPUNE. Aplicatia de bord nu
    are optiunea deloc."""
    sim = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'autonomy_enabled=True' in sim, "simularea trebuie sa ocoleasca"
    assert 'OCOLITA' in sim, "ocolirea trebuie anuntata la pornire"
    pi = open(os.path.join(REPO, 'tools', 'nova_pi.py')).read()
    assert 'autonomy_enabled=True' not in pi, (
        "nova_pi.py NU are voie sa ocoleasca garda E0")
    cfg = open(os.path.join(REPO, 'config', 'nova.json')).read()
    assert '"autonomy_enabled": false' in cfg, (
        "config/nova.json trebuie sa ramana inchis pana la E2")
    return "sim ocoleste si anunta, bordul nu poate, fisierul e inchis"


# --- urmarirea secventei ----------------------------------------------------

class _V:
    def __init__(self, alt=5.0):
        self.alt = alt
        self.have_pos = True


def _app_gol():
    """SimApp fara __init__: nu vrem Gazebo, MAVLink si camera pentru a
    testa contabilitatea starilor."""
    import importlib
    mod = importlib.import_module('nova_sim')
    app = object.__new__(mod.SimApp)
    app.t_state = {}
    app.prev_state = None
    app.prev_t = None
    app.alt_scoring_m = None
    app.pos_scoring = None
    app.pos_touchdown = None
    app.eroare_finala_m = None
    app.deriva_m = None
    app._prev_frames = 0
    app.n_det_total = 0
    app.tranzitii = []
    app.v = _V()

    class _SM:
        state = 'IDLE'
    app.sm = _SM()
    app.truth = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.01),
        marker=sim_truth.pose_from_enu(0, 0, 0.01))
    return app


def test_vehiculul_foloseste_acelasi_ceas_ca_bucla():
    """Altfel `time_since_heartbeat(now_sim)` iese negativa si monitorul de
    legatura din supervizor (H1) nu se declanseaza niciodata.

    Verificat pe comportament, nu pe sursa: se construieste un SimVehicle cu
    un ceas fals si se cere varsta heartbeat-ului."""
    import importlib
    mod = importlib.import_module('nova_sim')
    v = object.__new__(mod.SimVehicle)
    ceas = [100.0]
    v._clock = lambda: ceas[0]
    v.hb_t = None
    v.link_healthy = True
    v.reconnect_attempts = 0
    v.reconnects = 0
    v._reconnect_next_t = None
    v.time_boot_ms = None
    v.link_verbose = False
    v.on_link_event = None

    v._note_heartbeat()                      # fara `now`: trebuie luat ceasul
    assert v.hb_t == 100.0, (
        f"heartbeat notat la {v.hb_t}, nu la ceasul buclei (100.0)")
    ceas[0] = 102.5
    assert v.time_since_heartbeat(ceas[0]) == 2.5, (
        "varsta trebuie pozitiva si in aceleasi unitati ca bucla")
    return "heartbeat notat pe ceasul buclei, varsta 2.5 s"


def test_ceasul_gresit_ar_da_varsta_negativa():
    """Cazul negativ: un Vehicle obisnuit, intrebat cu timp de simulare.

    Fara perechea asta, testul de mai sus ar putea trece si daca nimic nu
    s-ar fi reparat (§5.11: fiecare test de siguranta are nevoie de un caz
    negativ)."""
    from nova.vehicle import Vehicle
    v = object.__new__(Vehicle)
    v.hb_t = 98765.0                         # time.monotonic(), ca in pump()
    varsta = v.time_since_heartbeat(12.5)    # intrebat cu timp de simulare
    assert varsta < 0, varsta
    from nova.safety import LINK_MAX_AGE_S
    assert varsta < LINK_MAX_AGE_S, (
        "cu ceasuri amestecate, pragul de legatura devine imposibil de atins")
    return f"varsta {varsta:.0f} s: pragul de {LINK_MAX_AGE_S} s nu s-ar atinge niciodata"


def test_timpul_per_stare_se_aduna():
    app = _app_gol()
    app.sm.state = 'DESCEND_TRACK'
    for t in (0.0, 1.0, 2.0, 3.0):
        app._track(t)
    app.sm.state = 'FINAL_DESCENT'
    app._track(4.0)
    app._track(5.0)
    assert abs(app.t_state['DESCEND_TRACK'] - 4.0) < 1e-9, app.t_state
    assert abs(app.t_state['FINAL_DESCENT'] - 1.0) < 1e-9, app.t_state
    return "4 s in DESCEND_TRACK, 1 s in FINAL_DESCENT"


def test_eroarea_si_deriva_se_masoara_din_adevar():
    app = _app_gol()
    app.sm.state = 'DESCEND_TRACK'
    app._track(0.0)
    # la SCORING_CAPTURE vehiculul e la 8 cm nord de marker, 0.44 m alt
    app.truth.set('iris_with_gimbal', sim_truth.pose_from_enu(0.0, 0.08, 0.45))
    app.v.alt = 0.45
    app.sm.state = 'SCORING_CAPTURE'
    app._track(1.0)
    # la contact, 10 cm nord si 3 cm est
    app.truth.set('iris_with_gimbal', sim_truth.pose_from_enu(0.03, 0.10, 0.08))
    app.sm.state = 'TOUCHDOWN_CONFIRM'
    app._track(2.0)
    assert abs(app.alt_scoring_m - 0.45) < 1e-9, app.alt_scoring_m
    er = app.eroare_finala_m
    assert abs(er - math.hypot(0.10, 0.03)) < 1e-6, er
    dv = app.deriva_m
    assert abs(dv - math.hypot(0.02, 0.03)) < 1e-6, dv
    return f"eroare {er * 100:.1f} cm, deriva {dv * 100:.1f} cm"


def test_fara_adevar_nu_se_inventeaza_cifre():
    app = _app_gol()
    app.truth = None
    app.sm.state = 'TOUCHDOWN_CONFIRM'
    app._track(0.0)
    assert app.eroare_finala_m is None, (
        "fara adevar, eroarea trebuie sa lipseasca, nu sa fie 0")
    return "fara SimTruth -> None, nu 0.0"


def test_rata_de_detectie_poate_scadea_sub_100():
    """§5.11: o metrica ce nu poate esua nu e metrica.

    Prima varianta avea o metoda `note_miss()` pe care nu o apela nimeni,
    deci `det_by_alt` primea numai True si rata iesea 100% orice s-ar fi
    intamplat - inclusiv cu detectorul mort."""
    app = _app_gol()
    app.det_by_alt = []

    class _Det:
        n_frames = 0
    app.detector = _Det()
    app.v.alt = 6.0

    # 10 cadre citite, 4 cu detectie
    app.detector.n_frames = 10
    app._note_frames(4)
    for _ in range(4):
        app.det_by_alt.append((6.0, True))
    rata, n = sim_truth.detection_rate(app.det_by_alt, 3.0, 12.0)
    assert n == 10, f"ar trebui 10 cadre in fereastra, sunt {n}"
    assert abs(rata - 0.4) < 1e-9, rata
    return "10 cadre, 4 detectii -> 40%, nu 100%"


def test_contorul_nu_da_ratari_negative():
    """Contorul creste in firul detectorului, deci o detectie poate ajunge
    in bucla inaintea incrementarii lui."""
    app = _app_gol()
    app.det_by_alt = []

    class _Det:
        n_frames = 2
    app.detector = _Det()
    app._note_frames(5)          # mai multe detectii decat cadre numarate
    assert app.det_by_alt == [], app.det_by_alt
    return "cadre < detectii -> zero ratari, nu numar negativ"


def test_CSV_acopera_ce_produce_randul():
    lipsa = [k for k in batch_sim.row_from({'idx': 0}, 'x')
             if k not in batch_sim.CSV_HEADER]
    assert not lipsa, f"coloane lipsa din CSV_HEADER: {lipsa}"
    r = batch_sim.row_from(
        {'idx': 1}, 'ok', succes=True,
        raport={'stare_finala': 'HANDBACK', 'eroare_finala_cm': 1.234,
                'deriva_cm': 0.5, 'alt_scoring_m': 0.43,
                't_state': {'DESCEND_TRACK': 12.0, 'FINAL_DESCENT': 3.0},
                'rata_detectie': 0.97, 'n_detectii': 400})
    assert r['t_descend_s'] == 12.0 and r['t_total_s'] == 15.0, r
    assert r['succes'] == 1
    return "raportul JSON se mapeaza pe coloane"


# --- planul campaniei -------------------------------------------------------

def test_nicio_suita_nu_uita_sa_inregistreze_un_test():
    """Un test definit si neinregistrat in `TESTS` nu ruleaza niciodata, si
    nimic nu o spune: suita raporteaza N/N trecute, doar ca N e mai mic.

    Aceeasi forma ca §5.11 - verificarea care lipseste e cea care ar fi
    invalidat rezultatul. Se aplica pe TOATE suitele, nu doar pe asta."""
    import glob
    import re
    probleme = []
    for cale in sorted(glob.glob(os.path.join(REPO, 'tools', 'test_*.py'))):
        src = open(cale).read()
        defs = {n.name for n in ast.parse(src).body
                if isinstance(n, ast.FunctionDef)
                and n.name.startswith('test_')}
        if not defs:
            continue
        inreg = set(re.findall(r'\b(test_\w+)\b', src.split('TESTS = [')[-1]))
        lipsa = defs - inreg
        if lipsa:
            probleme.append(f"{os.path.basename(cale)}: {sorted(lipsa)}")
    assert not probleme, "teste definite si neinregistrate:\n  " + \
        '\n  '.join(probleme)
    return "toate suitele isi inregistreaza toate testele"


def test_planul_e_reproductibil():
    a = batch_sim.plan(8, seed=3)
    b = batch_sim.plan(8, seed=3)
    c = batch_sim.plan(8, seed=4)
    assert a == b, "aceeasi samanta trebuie sa dea acelasi plan"
    assert a != c, "seminte diferite trebuie sa dea planuri diferite"
    return "--seed fixeaza campania"


def test_raza_e_plafonata_de_cadrul_camerei():
    """Limita efectiva nu e cea din poarta (6.5 m), ci a camerei."""
    assert batch_sim.raza_max(5.0) < 3.0, batch_sim.raza_max(5.0)
    assert batch_sim.raza_max(12.0) == batch_sim.MARKER_RADIUS_M
    assert batch_sim.raza_max(8.0) > batch_sim.raza_max(5.0)
    for c in batch_sim.plan(60, seed=11):
        lim = batch_sim.raza_max(c['alt_handover'])
        assert c['raza_m'] <= lim + 1e-9, (
            f"raza {c['raza_m']} peste limita {lim} la "
            f"{c['alt_handover']} m")
    return (f"5 m -> {batch_sim.raza_max(5.0):.1f} m, "
            f"12 m -> {batch_sim.raza_max(12.0):.1f} m")


def test_pozitia_e_uniforma_pe_disc_nu_pe_raza():
    """r = R*sqrt(U), nu r = R*U: altfel campania testeaza mai ales
    cazul usor, cu markerul aproape sub vehicul."""
    p = batch_sim.plan(400, seed=5)
    fractii = [c['raza_m'] / batch_sim.raza_max(c['alt_handover'])
               for c in p]
    afara = sum(1 for f in fractii if f > 0.7071)
    # jumatate din aria discului e peste r/R = 1/sqrt(2)
    assert 0.40 < afara / len(fractii) < 0.60, (
        f"{afara}/{len(fractii)} peste jumatatea de arie - distributia nu e "
        f"uniforma pe disc")
    return f"{afara / len(fractii):.0%} in jumatatea exterioara (asteptat 50%)"


def test_roughness_e_echilibrat_nu_tras_la_intamplare():
    """Doua valori trase independent pot da 4x aceeasi valoare pe o
    campanie mica, iar conditia ramane netestata."""
    for n in (2, 4, 6, 10):
        val = {c['roughness'] for c in batch_sim.plan(n, seed=7)}
        assert val == set(batch_sim.ROUGHNESS), (
            f"pe {n} rulari lipseste o valoare: {val}")
    return "ambele valori prezente de la 2 rulari in sus"


def test_planul_ramane_in_fereastra_portii():
    for c in batch_sim.plan(200, seed=9):
        assert batch_sim.ALT_MIN_M <= c['alt_handover'] <= batch_sim.ALT_MAX_M
        assert c['raza_m'] <= batch_sim.MARKER_RADIUS_M
        assert 0.0 <= c['wind_spd'] <= batch_sim.WIND_SPD_MAX
        assert 0.0 <= c['sun_az'] <= 360.0
        assert batch_sim.SUN_EL_MIN <= c['sun_el'] <= batch_sim.SUN_EL_MAX
    return "200 de rulari, toate in plaja declarata"


def test_statisticile_sunt_percentile_nu_medie():
    randuri = [{'succes': 1, 'eroare_finala_cm': 1.0} for _ in range(19)]
    randuri.append({'succes': 1, 'eroare_finala_cm': 100.0})
    s = batch_sim.summarize(randuri)
    medie = sum(r['eroare_finala_cm'] for r in randuri) / len(randuri)
    assert abs(s['eroare_finala_cm']['p50'] - 1.0) < 1e-9
    assert medie > 5.0, medie
    assert s['eroare_finala_cm']['p95'] > 1.0, "p95 trebuie sa vada coada"
    return f"p50 1.0, p95 {s['eroare_finala_cm']['p95']:.1f}, medie {medie:.1f}"


def test_esecurile_nu_intra_in_statistici():
    randuri = [{'succes': 1, 'eroare_finala_cm': 2.0},
               {'succes': 0, 'motiv': 'oprit in ABORT',
                'eroare_finala_cm': 500.0}]
    s = batch_sim.summarize(randuri)
    assert s['eroare_finala_cm']['n'] == 1, (
        "o rulare esuata nu are voie sa contribuie la eroarea tipica")
    assert s['motive'] == {'oprit in ABORT': 1}
    assert abs(s['rata_succes'] - 0.5) < 1e-9
    return "esecurile se numara, nu se mediaza"


# --- comenzile campaniei ----------------------------------------------------

def test_SITL_porneste_cu_eeprom_sters():
    """§5.13: fara -w, vantul rularii precedente ramane in eeprom si
    campania masoara altceva decat crede."""
    c = batch_sim.sitl_cmd('/tmp/x.parm')
    assert '-w' in c, c
    assert any(a.startswith('--add-param-file=') for a in c), c
    return "-w prezent"


def test_porturile_sunt_distincte_intre_ele_si_de_cele_interactive():
    porturi = {batch_sim.PORT_FLY, batch_sim.PORT_HANDOVER,
               batch_sim.PORT_SIM}
    assert len(porturi) == 3, porturi
    interactive = {14550, 14552, 14553, 14554}
    assert not (porturi & interactive), (
        f"se bat cu o sesiune din start_sim.sh: {porturi & interactive}")
    c = batch_sim.sitl_cmd('/tmp/x.parm')
    for p in porturi:
        assert any(f':{p}' in a for a in c), f"portul {p} nu e in --out"
    return f"{sorted(porturi)}, niciunul din tabelul interactiv"


def test_fisierul_de_parametri_pastreaza_baza_si_adauga_vantul():
    with tempfile.TemporaryDirectory() as d:
        baza = os.path.join(d, 'baza.parm')
        with open(baza, 'w') as f:
            f.write("SURFTRAK_MODE,0\nPLND_ENABLED,0\n")
        cale = batch_sim.param_file_for(
            {'idx': 3, 'wind_spd': 4.5, 'wind_turb': 9.0}, d, baza=baza)
        text = open(cale).read()
    assert 'SURFTRAK_MODE,0' in text, "baza s-a pierdut"
    assert 'PLND_ENABLED,0' in text, "baza s-a pierdut"
    assert 'SIM_WIND_SPD,4.5' in text, text
    assert 'SIM_WIND_TURB,9.0' in text, text
    return "baza + doua linii de vant"


def test_curatenia_acopera_toate_procesele_campaniei():
    src = open(os.path.join(REPO, 'tools', 'batch_sim.py')).read()
    arbore = ast.parse(src)
    tipare = None
    for nod in ast.walk(arbore):
        if isinstance(nod, ast.FunctionDef) and nod.name == 'cleanup':
            for sub in ast.walk(nod):
                if isinstance(sub, ast.Tuple):
                    tipare = [e.value for e in sub.elts
                              if isinstance(e, ast.Constant)]
                    break
    assert tipare, "nu gasesc lista de tipare din cleanup()"
    for cerut in ('arducopter', 'gz sim', 'nova_sim.py'):
        assert cerut in tipare, f"{cerut} nu se curata (§5.5, §5.33)"
    return f"{len(tipare)} tipare"


# --- mesajele catre FC ------------------------------------------------------

class _Rec:
    """Inregistreaza ce s-a trimis, in loc sa trimita."""

    def __init__(self):
        self.trimise = []
        self.target_system = 1
        self.target_component = 1
        self.mav = self

    def command_long_send(self, *a):
        self.trimise.append(('command_long', a))

    def set_position_target_local_ned_send(self, *a):
        self.trimise.append(('pos_target', a))


def test_altitudinea_se_trimite_negativ_in_NED():
    """Semnul lui z: pozitiv inseamna SUB home, iar ArduPilot accepta
    comanda fara sa se planga."""
    import sim_fly_to
    m = _Rec()
    sim_fly_to.send_goto_ned(m, 2.0, -1.0, 8.0)
    tip, a = m.trimise[-1]
    assert tip == 'pos_target'
    north, east, down = a[5], a[6], a[7]
    assert north == 2.0 and east == -1.0, (north, east)
    assert down == -8.0, f"z trebuie -8.0 (sus), nu {down}"
    return "alt 8 m -> z = -8.0"


def test_masca_ignora_viteza_acceleratia_si_yaw():
    import sim_fly_to
    m = sim_fly_to.TYPE_MASK_POS
    for bit in range(3):
        assert not (m >> bit) & 1, f"bitul {bit} (pozitie) nu are voie ignorat"
    for bit in range(3, 12):
        assert (m >> bit) & 1, f"bitul {bit} ar trebui ignorat"
    return f"masca {m:#014b}"


def test_takeoff_pune_altitudinea_pe_param7():
    import sim_fly_to
    from pymavlink import mavutil
    m = _Rec()
    sim_fly_to.send_takeoff(m, 7.5)
    tip, a = m.trimise[-1]
    assert a[2] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, a[2]
    assert a[-1] == 7.5, f"param7 trebuie sa fie altitudinea, e {a[-1]}"
    return "NAV_TAKEOFF param7 = 7.5"


def test_schimbarea_de_mod_cere_CUSTOM_MODE():
    import sim_fly_to
    from pymavlink import mavutil
    m = _Rec()
    sim_fly_to.set_mode(m, sim_fly_to.MODE_LOITER)
    tip, a = m.trimise[-1]
    assert a[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE
    assert a[4] & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, a[4]
    assert a[5] == sim_fly_to.MODE_LOITER, a[5]
    return "DO_SET_MODE cu CUSTOM_MODE_ENABLED"


def test_codurile_de_iesire_ale_decolarii_sunt_distincte():
    """Un singur 'a esuat' amesteca un SITL care inca compileaza cu un
    prearm respins. Distributia motivelor e ea insasi un rezultat."""
    import sim_fly_to
    coduri = {sim_fly_to.EXIT_OK, sim_fly_to.EXIT_TINTA,
              sim_fly_to.EXIT_FARA_LEGATURA, sim_fly_to.EXIT_FARA_ARMARE}
    assert len(coduri) == 4, coduri
    assert sim_fly_to.EXIT_OK == 0, "succesul trebuie sa fie 0"
    for cod in coduri - {0}:
        assert cod in batch_sim.MOTIV_FLY, (
            f"codul {cod} nu are traducere in batch_sim.MOTIV_FLY")
    return f"{sorted(coduri)}, toate traduse in CSV"


def test_partea_manuala_nu_e_o_a_doua_cale_spre_autonom():
    """§8: poarta e SINGURA intrare. O unealta de banc care ar comanda LAND
    sau ar atinge PLND ar fi exact calea alternativa refuzata acolo."""
    src = open(os.path.join(REPO, 'tools', 'sim_fly_to.py')).read()
    for interzis in ('MODE_LAND', 'PLND_', 'LandingStateMachine',
                     'HandoverGate'):
        assert interzis not in src, (
            f"sim_fly_to.py contine {interzis}: e o a doua cale spre "
            f"segmentul autonom")
    return "nu comanda LAND, nu atinge PLND"


TESTS = [
    ('ENU devine NED', test_enu_devine_ned),
    ('planul markerului se scade din range',
     test_planul_markerului_se_scade_din_range),
    ('offsetul ignorat ar fi 2% la 0.5 m',
     test_offsetul_ignorat_ar_fi_2_la_suta),
    ('sub planul markerului nu exista adevar',
     test_sub_planul_markerului_nu_exista_adevar),
    ('eroarea de range are semn', test_eroarea_de_range_are_semn),
    ('eroarea unghiulara e distanta, nu suma',
     test_eroarea_unghiulara_e_distanta_nu_suma),
    ('rata de detectie ignora ce e sub prag',
     test_rata_de_detectie_ignora_ce_e_sub_prag),
    ('rata fara cadre e None, nu zero', test_rata_fara_cadre_e_None_nu_zero),
    ('summarize da percentile, nu medie',
     test_summarize_da_percentile_nu_medie),
    ('pragurile chiar resping', test_pragurile_sunt_verificate_nu_doar_raportate),
    ('ceasul stie sa spuna ca nu stie', test_ceasul_stie_sa_spuna_ca_nu_stie),
    ('ceasul e cel mai recent timp de simulare',
     test_ceasul_e_cel_mai_recent_timp_de_simulare),
    ('ordinea din SimApp o oglindeste pe run_loop',
     test_ordinea_din_SimApp_o_oglindeste_pe_run_loop),
    ('bucla de sim nu foloseste ceasul de perete',
     test_bucla_de_sim_nu_foloseste_ceasul_de_perete),
    ('run_loop a ramas neatins', test_run_loop_a_ramas_neatins),
    ('E0 e ocolita explicit si anuntata',
     test_E0_e_ocolita_explicit_si_anuntata),
    ('vehiculul foloseste acelasi ceas ca bucla',
     test_vehiculul_foloseste_acelasi_ceas_ca_bucla),
    ('ceasul gresit ar da varsta negativa',
     test_ceasul_gresit_ar_da_varsta_negativa),
    ('timpul per stare se aduna', test_timpul_per_stare_se_aduna),
    ('eroarea si deriva se masoara din adevar',
     test_eroarea_si_deriva_se_masoara_din_adevar),
    ('fara adevar nu se inventeaza cifre',
     test_fara_adevar_nu_se_inventeaza_cifre),
    ('rata de detectie poate scadea sub 100%',
     test_rata_de_detectie_poate_scadea_sub_100),
    ('contorul nu da ratari negative', test_contorul_nu_da_ratari_negative),
    ('CSV acopera ce produce randul', test_CSV_acopera_ce_produce_randul),
    ('nicio suita nu uita sa inregistreze un test',
     test_nicio_suita_nu_uita_sa_inregistreze_un_test),
    ('planul e reproductibil', test_planul_e_reproductibil),
    ('raza e plafonata de cadrul camerei',
     test_raza_e_plafonata_de_cadrul_camerei),
    ('pozitia e uniforma pe disc', test_pozitia_e_uniforma_pe_disc_nu_pe_raza),
    ('roughness e echilibrat', test_roughness_e_echilibrat_nu_tras_la_intamplare),
    ('planul ramane in fereastra portii',
     test_planul_ramane_in_fereastra_portii),
    ('statisticile sunt percentile, nu medie',
     test_statisticile_sunt_percentile_nu_medie),
    ('esecurile nu intra in statistici',
     test_esecurile_nu_intra_in_statistici),
    ('SITL porneste cu eeprom sters', test_SITL_porneste_cu_eeprom_sters),
    ('porturile sunt distincte',
     test_porturile_sunt_distincte_intre_ele_si_de_cele_interactive),
    ('fisierul de parametri pastreaza baza',
     test_fisierul_de_parametri_pastreaza_baza_si_adauga_vantul),
    ('curatenia acopera toate procesele',
     test_curatenia_acopera_toate_procesele_campaniei),
    ('altitudinea se trimite negativ in NED',
     test_altitudinea_se_trimite_negativ_in_NED),
    ('masca ignora viteza, acceleratia si yaw',
     test_masca_ignora_viteza_acceleratia_si_yaw),
    ('takeoff pune altitudinea pe param7',
     test_takeoff_pune_altitudinea_pe_param7),
    ('schimbarea de mod cere CUSTOM_MODE',
     test_schimbarea_de_mod_cere_CUSTOM_MODE),
    ('codurile de iesire ale decolarii sunt distincte',
     test_codurile_de_iesire_ale_decolarii_sunt_distincte),
    ('partea manuala nu e a doua cale spre autonom',
     test_partea_manuala_nu_e_o_a_doua_cale_spre_autonom),
]


def main():
    fails = 0
    for name, fn in TESTS:
        try:
            note = fn()
            print(f"  OK    {name}" + (f"   ({note})" if note else ""))
        except AssertionError as e:
            fails += 1
            print(f"  ESEC  {name}\n        {e}")
        except Exception as e:                              # noqa: BLE001
            fails += 1
            import traceback
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
            traceback.print_exc(limit=3)
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
