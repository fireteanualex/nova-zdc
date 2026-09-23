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
    """Range-ul e de la CAMERA la PLANUL markerului: se scad amandoua
    offseturile, montajul camerei si cei 1 cm ai planului."""
    dz = sim_truth.CAM_MOUNT_M + sim_truth.MARKER_PLANE_Z
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0.0, 0.0, 5.0 + dz),
        marker=sim_truth.pose_from_enu(0.0, 0.0, sim_truth.MARKER_PLANE_Z))
    tr = t.truth()
    assert abs(tr['range_m'] - 5.0) < 1e-6, tr['range_m']
    # Si la altitudine mica, unde offseturile conteaza cel mai mult:
    t.set('iris_with_gimbal', sim_truth.pose_from_enu(0, 0, 0.5 + dz))
    tr = t.truth()
    assert abs(tr['range_m'] - 0.5) < 1e-6, tr['range_m']
    return f"ambele offseturi ({dz * 100:.1f} cm) scazute, la 5 m si la 0.5 m"


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


def test_adevarul_e_al_CAMEREI_nu_al_vehiculului():
    """Detectorul masoara de la camera, aflata la 74.5 mm sub originea
    vehiculului. Ignorat, apare ca bias pe range: 0.7% la 10 m, dar **15%
    la 0.5 m** - exact acolo unde se decide aterizarea.

    Prima campanie a raportat `eroare range p50 = 10.8%`, imposibil pentru o
    aterizare cu 0.55 cm eroare finala. Nu detectorul gresea, ci
    comparatia."""
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + sim_truth.CAM_MOUNT_M
                                        + sim_truth.MARKER_PLANE_Z),
        marker=sim_truth.pose_from_enu(0, 0, sim_truth.MARKER_PLANE_Z))
    assert abs(t.truth()['range_m'] - 5.0) < 1e-9, t.truth()['range_m']
    # cat ar fi fost eroarea daca montajul se ignora, la 0.5 m
    gresit = (0.5 + sim_truth.CAM_MOUNT_M) / 0.5 - 1.0
    assert gresit > 0.14, gresit
    return f"montajul scazut; ignorat ar fi dat +{gresit:.0%} la 0.5 m"


def test_yaw_ul_vine_de_la_FC_nu_din_cuaternionul_ENU():
    """ENU si NED numara capul din axe diferite: yaw = 0 inseamna EST in
    Gazebo si NORD la noi. Derivat din cuaternion, unghiul iese rotit cu
    90 de grade, iar eroarea unghiulara raportata devine chiar offsetul de
    convenție.

    Masurat: campania a raportat `eroare unghi p50 = 15.7 deg` pe un sistem
    care ateriza cu 0.7 cm. Adevarul se cere acum cu atitudinea din
    `ATTITUDE` raportat de FC, care e NED prin definitie."""
    import math as _m
    dz = sim_truth.CAM_MOUNT_M + sim_truth.MARKER_PLANE_Z
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + dz),
        marker=sim_truth.pose_from_enu(0.0, 1.0, sim_truth.MARKER_PLANE_Z))

    # cap spre NORD: markerul de la nord e IN FATA
    tr = t.truth(yaw=0.0)
    assert abs(tr['fwd_off_m'] - 1.0) < 1e-9, tr['fwd_off_m']
    assert abs(tr['right_off_m']) < 1e-9, tr['right_off_m']
    assert tr['angle_x'] > 0 and abs(tr['angle_y']) < 1e-9

    # cap spre EST: acelasi marker e la STANGA
    tr = t.truth(yaw=_m.radians(90))
    assert abs(tr['fwd_off_m']) < 1e-9, tr['fwd_off_m']
    assert abs(tr['right_off_m'] + 1.0) < 1e-9, tr['right_off_m']

    # CAZUL NEGATIV: convenția gresita da tocmai raspunsul celalalt
    psi = _m.radians(90)
    gresit_fwd = 1.0 * _m.cos(psi) + 0.0 * _m.sin(psi)
    assert abs(gresit_fwd) < 1e-9, "aici cele doua coincid; alege alt unghi"
    tr45 = t.truth(yaw=_m.radians(45))
    psi45 = _m.radians(45)
    gresit = 1.0 * _m.cos(psi45)
    assert abs(tr45['fwd_off_m'] - gresit) < 1e-9, (
        "la 45 deg cele doua convenții coincid din intamplare; testul nu "
        "distinge nimic")
    return "cap nord -> in fata; cap est -> la stanga"


def test_range_ul_adevarat_e_pe_axa_optica_nu_vertical():
    """`detector_pi` calculeaza `range_m = (t·n)/n_z`, adica distanta pe axa
    optica pana la PLANUL markerului. Cu vehiculul inclinat, aia e
    `vertical / cos(inclinare)` - 1.2% la 8.7 grade, 3.5% la 15.

    Adevarul comparat vertical raporta diferenta asta ca eroare de range."""
    import math as _m
    dz = sim_truth.CAM_MOUNT_M + sim_truth.MARKER_PLANE_Z
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + dz),
        marker=sim_truth.pose_from_enu(0, 0, sim_truth.MARKER_PLANE_Z))
    drept = t.truth()
    assert abs(drept['range_m'] - 5.0) < 1e-9, drept['range_m']
    for grade in (8.7, 15.0):
        tr = t.truth(pitch=_m.radians(grade))
        astept = 5.0 / _m.cos(_m.radians(grade))
        assert abs(tr['range_m'] - astept) < 1e-9, (grade, tr['range_m'])
        assert abs(tr['tilt_deg'] - grade) < 1e-6, tr['tilt_deg']
        assert abs(tr['vert_m'] - 5.0) < 1e-9, "verticala ramane disponibila"
    # cat ar fi fost eroarea raportata gresit
    gresit = 1.0 / _m.cos(_m.radians(15.0)) - 1.0
    assert gresit > 0.03, gresit
    return f"5/cos(15 deg) exact; comparat vertical ar da +{gresit:.1%}"


def test_unghiurile_se_compara_in_cadrul_corpului():
    """Detectorul raporteaza inainte/dreapta; adevarul are nord/est. Cele
    doua coincid DOAR cand capul e la zero.

    Fara rotatie, eroarea unghiulara raportata e practic chiar yaw-ul
    vehiculului - de aceea prima campanie a dat `p50 = 12 deg` pe un sistem
    care ateriza cu 0.55 cm."""
    import math as _m
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + sim_truth.CAM_MOUNT_M
                                        + sim_truth.MARKER_PLANE_Z),
        marker=sim_truth.pose_from_enu(0, 1.0, sim_truth.MARKER_PLANE_Z))
    tr = t.truth(yaw=_m.radians(90))
    assert abs(tr['yaw_deg'] - 90.0) < 1e-6, tr['yaw_deg']
    assert abs(tr['north_off_m'] - 1.0) < 1e-9
    # cu capul spre est, un marker la nord e la STANGA, nu in fata
    assert abs(tr['fwd_off_m']) < 1e-9, tr['fwd_off_m']
    assert abs(tr['right_off_m'] + 1.0) < 1e-9, tr['right_off_m']
    assert abs(tr['angle_x']) < 1e-9, "unghiul inainte trebuie sa fie zero"
    assert tr['angle_y'] < 0, "markerul e la stanga: unghi dreapta negativ"

    # CAZUL NEGATIV: cu yaw zero, cele doua cadre coincid
    t2 = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + sim_truth.CAM_MOUNT_M
                                        + sim_truth.MARKER_PLANE_Z),
        marker=sim_truth.pose_from_enu(0, 1.0, sim_truth.MARKER_PLANE_Z))
    tr2 = t2.truth(yaw=0.0)
    assert abs(tr2['fwd_off_m'] - 1.0) < 1e-9, tr2['fwd_off_m']
    return "yaw 90 deg: marker la nord -> 1 m la stanga, 0 in fata"


def test_eroarea_de_range_are_semn():
    _dz = sim_truth.CAM_MOUNT_M + sim_truth.MARKER_PLANE_Z
    t = sim_truth.StaticTruth(
        vehicle=sim_truth.pose_from_enu(0, 0, 5.0 + _dz),
        marker=sim_truth.pose_from_enu(0, 0, sim_truth.MARKER_PLANE_Z))
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
        vehicle=sim_truth.pose_from_enu(
            0, 0, 10.0 + sim_truth.CAM_MOUNT_M + sim_truth.MARKER_PLANE_Z),
        marker=sim_truth.pose_from_enu(0, 0, sim_truth.MARKER_PLANE_Z))
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


def test_cele_trei_fisiere_raman_acoperite(): 
    """J0: interdicția rundei 7 se ridică, acoperirea nu.

    Testul de dinainte rula `git diff` pe `state_machine.py`, `safety.py` si
    `handover.py` si pica daca erau atinse. Runda 8 le deschide deliberat -
    dar sters pur si simplu, ar fi disparut si semnalul ca sunt fisiere care
    cer grija.

    Ce apara acum e PROPRIETATEA, nu fisierul: fiecare are suita lui, si
    ordinea din `run_loop` vs `SimApp.step()` ramane verificata separat."""
    import subprocess
    perechi = (('nova/state_machine.py', 'tools/test_state_machine.py'),
               ('nova/safety.py', 'tools/test_safety.py'),
               ('nova/handover.py', 'tools/test_handover.py'))
    for sursa, suita in perechi:
        assert os.path.exists(os.path.join(REPO, sursa)), sursa
        cale = os.path.join(REPO, suita)
        assert os.path.exists(cale), f"{sursa} nu mai are suita {suita}"
        r = subprocess.run([sys.executable, cale], capture_output=True,
                           text=True, cwd=REPO, timeout=300)
        ultima = [l for l in r.stdout.splitlines() if 'teste trecute' in l]
        assert ultima, f"{suita} nu raporteaza un rezumat"
        n, d = ultima[-1].strip().split()[0].split('/')
        assert n == d, f"{suita}: {ultima[-1].strip()}"
    return f"{len(perechi)} fisiere sensibile, fiecare cu suita lui verde"


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
        # atitudinea vine de la FC (NED); vehiculul fals o raporteaza zero
        self.roll = self.pitch = self.yaw = 0.0


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
    app.scoring_alt_m = None
    app.scoring_px = None
    app.rec = None            # 8.3.3: fara recorder in testul de urmarire
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
    # captura de scoring: numarata din EVENIMENT, nu din tranzitia de stare
    # (§5.51) - poate sa se produca si in FINAL_DESCENT
    app.truth.set('iris_with_gimbal', sim_truth.pose_from_enu(0.0, 0.08, 0.55))
    app.v.alt = 0.45
    app._on_event('scoring_capture', {'alt': 0.45, 'marker_px': 985.0})
    app.sm.state = 'FINAL_DESCENT'
    app._track(1.0)
    # la contact, 10 cm nord si 3 cm est
    app.truth.set('iris_with_gimbal', sim_truth.pose_from_enu(0.03, 0.10, 0.20))
    app.sm.state = 'TOUCHDOWN_CONFIRM'
    app._track(2.0)
    assert abs(app.scoring_alt_m - 0.45) < 1e-9, app.scoring_alt_m
    er = app.eroare_finala_m
    assert abs(er - math.hypot(0.10, 0.03)) < 1e-6, er
    dv = app.deriva_m
    assert abs(dv - math.hypot(0.02, 0.03)) < 1e-6, dv
    return f"eroare {er * 100:.1f} cm, deriva {dv * 100:.1f} cm"


def test_latenta_se_masoara_in_ceasul_sursei():
    """§5.43: `PiDetector` scadea `t_capture` (timp de simulare) din
    `time.monotonic()` (uptime). Diferenta nu e o latenta.

    Masurat in prima campanie: `lat p50 3385850 ms` = 3386 s, exact cat
    rula masina. Verificat aici pe comportament, cu un ceas fals."""
    from nova.detector_pi import PiDetector

    class _Sursa:
        def __init__(self):
            self.cadre = [(None, 10.0), (None, 10.1)]
        def read(self):
            return self.cadre.pop(0) if self.cadre else None
        def close(self):
            pass

    class _Det:
        def detect(self, gray, t):
            return object()

    ceas = [10.05]
    d = PiDetector(_Sursa(), _Det(), threaded=False, clock=lambda: ceas[0])
    d._process_one()
    assert d.latencies, "nicio latenta inregistrata"
    assert abs(d.latencies[0] - 0.05) < 1e-9, (
        f"latenta {d.latencies[0]} - ceasul sursei nu a fost folosit")

    # implicitul ramane ceasul de perete, pentru vehicul
    d2 = PiDetector(_Sursa(), _Det(), threaded=False)
    import time as _t
    assert d2.clock is _t.monotonic, "implicitul s-a schimbat pe vehicul"
    return "latenta 50 ms in ceasul sursei; implicit neschimbat pe Pi"


def test_inclinarea_intra_in_verificarea_de_incadrare():
    """§5.2 presupune camera la NADIR. Cu vehiculul inclinat, axa optica
    bate solul la `h*tan(inclinare)` de punctul de sub el, iar marja pana la
    marginea cadrului scade cu atat.

    La 0.5 m marja e de ~11 cm, deci o inclinare de 12 grade scoate singura
    markerul din cadru - fara ca nimic din imagine sa fie in neregula."""
    import math as _m
    q = (0.9962, 0.0872, 0.0, 0.0)          # roll 10 grade
    p = sim_truth.pose_from_enu(0, 0, 1.0, q)
    roll, pitch = p.roll_pitch_deg
    assert abs(roll - 10.0) < 0.1, roll
    assert abs(pitch) < 0.1, pitch

    vfov = 69.5
    for h, incl, asteptat in ((1.0, 10.0, True), (0.5, 15.0, False)):
        demi = h * _m.tan(_m.radians(vfov / 2))
        deviere = h * _m.tan(_m.radians(incl))
        marja = demi - deviere - 0.24
        assert (marja > 0) is asteptat, (h, incl, marja)
    # Cazul masurat: pierdere la 1.06 m cu ~20 cm lateral si ~15 deg
    # inclinare. Pragul de 0.38 m din §5.2 e o cifra de NADIR, cu eroare
    # zero; bugetul real e lateral + h*tan(inclinare) + jumatate de marker.
    h, lat, incl = 1.06, 0.20, 15.0
    demi = h * _m.tan(_m.radians(vfov / 2))
    assert demi - lat - h * _m.tan(_m.radians(incl)) - 0.24 < 0.05, (
        "cazul masurat ar trebui sa fie chiar la limita")
    return ("roll 10 deg din cuaternion; la 0.5 m, 15 deg scot markerul; "
            "cazul de la 1.06 m e chiar la limita")


def test_diagnosticul_de_pierdere_distinge_cauza():
    """`detection_age: BRAKE` nu spune DE CE. Diagnosticul raporteaza
    geometria din adevar si salveaza cadrul, o singura data per pierdere."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'DETECTIE PIERDUTA' in src
    assert 'e GEOMETRIE, nu imagine' in src, "nu distinge cauza geometrica"
    assert 'cauza e in IMAGINE' in src, "nu distinge cauza de imagine"
    # pragul trebuie sa fie SUB cel al supervizorului, altfel raportul se
    # scrie dupa ce secventa a fost deja oprita si geometria s-a schimbat
    from nova.safety import DETECTION_MAX_AGE_S
    import importlib
    mod = importlib.import_module('nova_sim')
    assert mod.SimApp.PRAG_DIAGNOSTIC_S < DETECTION_MAX_AGE_S, (
        f"diagnostic la {mod.SimApp.PRAG_DIAGNOSTIC_S} s, supervizor la "
        f"{DETECTION_MAX_AGE_S} s: raportul vine prea tarziu")
    assert '--dump-dir' in batch_sim.nova_sim_cmd('/tmp/x', 'c.yaml', 10), (
        "campania nu cere salvarea cadrului")
    return (f"prag {mod.SimApp.PRAG_DIAGNOSTIC_S} s < "
            f"{DETECTION_MAX_AGE_S} s al supervizorului")


def test_optiunile_de_experiment_ajung_din_campanie_in_aplicatie():
    """Un knob care exista in `nova_sim.py` si nu se poate atinge din
    `batch_sim.py` inseamna ca experimentul se poate face doar de mana, pe o
    singura rulare - exact ce nu vrei cand incerci o valoare noua.

    Prins cu mana pe tastatura: `--no-lateral-alt` a fost expus in aplicatie
    si uitat in campanie, iar prima incercare a picat cu
    `unrecognized arguments`."""
    import argparse
    base = batch_sim.nova_sim_cmd('/tmp/x', 'c.yaml', 10)
    assert '--no-lateral-alt' not in base, "implicit nu se trimite nimic"

    c = batch_sim.nova_sim_cmd('/tmp/x', 'c.yaml', 10, no_lateral_alt=0.8,
                               authority=True)
    assert c[c.index('--no-lateral-alt') + 1] == '0.8', c
    assert '--authority' in c, c

    # si CLI-ul campaniei chiar le accepta
    p = argparse.ArgumentParser()
    src = open(os.path.join(REPO, 'tools', 'batch_sim.py')).read()
    for optiune in ('--no-lateral-alt', '--authority', '--fast-descent'):
        assert f"'{optiune}'" in src, f"{optiune} lipseste din CLI-ul campaniei"
    del p
    return "3 optiuni, din campanie pana in aplicatie"


def test_pragul_de_coborare_verticala_e_reglabil_fara_cod():
    """Trecerea in FINAL_DESCENT e o VALOARE in SequenceConfig, nu o
    constanta ingropata.

    Conteaza pentru §5.45: fereastra de incadrare se inchide mai sus decat
    cei 0.40 m impliciti daca eroarea laterala nu e mica. Sa poti incerca
    0.5 sau 0.8 m fara sa atingi masina de stari inseamna un experiment cu
    o singura variabila, nu o modificare de cod validat."""
    from nova import state_machine as sm_mod
    c = sm_mod.SequenceConfig()
    # valoarea nu se scrie aici: duplicata, ar ramane in urma la prima
    # reglare si testul ar trece din inertie (§5.40). Ce se verifica e ca
    # implicitul vine din constanta de modul si ca se poate schimba.
    assert c.no_lateral_alt_m == sm_mod.NO_LATERAL_ALT_M, c.no_lateral_alt_m
    assert c.scoring_fill == sm_mod.SCORING_FILL, c.scoring_fill
    assert c.final_fill == sm_mod.FINAL_FILL, c.final_fill
    c.no_lateral_alt_m = 0.8
    assert c.no_lateral_alt_m == 0.8
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    for knob in ('--no-lateral-alt', '--scoring-fill', '--final-fill'):
        assert knob in src, f"knob-ul {knob} nu e expus in aplicatie"
    return "reglabile din SequenceConfig si din linia de comanda"


def test_campania_masoara_ce_zboara():
    """Campania nu are voie sa porneasca cu alte praguri decat vehiculul.

    A avut: `batch_sim.NO_LATERAL_ALT_M = 0.60` peste cei 0.40 m din
    `SequenceConfig`. Deci toate cifrele din §5.52 si §5.54 - 19/20, 10/10,
    eroare finala p50 0.64 cm - descriu o configuratie pe care `nova_pi.py`
    nu ar fi zburat-o niciodata, fiindca el ia implicitele.

    Nu era o valoare gresita, era o valoare care nu exista decat in campanie.
    Un rand din Compliance Matrix sprijinit pe ea ar fi afirmat ceva
    nemasurat despre ce zboara.

    Regula: ce se regleaza dintr-un experiment se da EXPLICIT din linia de
    comanda, si atunci campania o si anunta ca experiment (§5.40). Implicit,
    orice knob e None si aplicatia isi ia valoarea din cod."""
    import argparse
    import inspect

    # 1. implicitele CLI ale campaniei sunt toate None: nimic suprascris tacit
    src = inspect.getsource(batch_sim.main)
    assert "'no_lateral_alt': a.no_lateral_alt" in src, (
        "campania pune altceva decat valoarea din linia de comanda in "
        "no_lateral_alt - deci suprascrie pragul vehiculului")

    # 2. si nu a ramas o constanta de campanie care sa umbreasca una de cod
    assert not hasattr(batch_sim, 'NO_LATERAL_ALT_M'), (
        "batch_sim are propriul NO_LATERAL_ALT_M: doua surse de adevar "
        "pentru acelasi prag, iar campania o foloseste pe a ei")

    # 3. aplicatia de bord ia implicitele, deci ele SUNT ce zboara
    pi_src = open(os.path.join(REPO, 'tools', 'nova_pi.py')).read()
    assert 'SequenceConfig(conv=a.conv)' in pi_src, (
        "nova_pi.py nu mai construieste SequenceConfig doar cu implicitele; "
        "verifica daca pragurile de zbor mai sunt cele masurate")
    for knob in ('scoring_fill', 'final_fill', 'no_lateral_alt_m'):
        assert f'seq.{knob}' not in pi_src and f'{knob}=' not in pi_src, (
            f"nova_pi.py regleaza {knob}: vehiculul zboara alte praguri "
            f"decat cele masurate in campanie")
    return "campania si bordul pornesc de la aceleasi praguri"


def test_raportul_se_scrie_si_la_oprire_din_afara():
    """Prima secventa completa reusita s-a pierdut exact asa: rularea a fost
    oprita din afara, iar raportul si CSV-ul se scriu DUPA bucla.

    SIGTERM omoara procesul pe loc; ridicat ca KeyboardInterrupt, bucla iese
    normal si masuratorile ajung pe disc. Cine opreste - campania la
    cleanup, operatorul care inchide Gazebo - nu trebuie sa piarda ce s-a
    masurat."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'signal.SIGTERM' in src, "SIGTERM nu e tratat"
    arbore = ast.parse(src)
    fn = {n.name: n for n in ast.walk(arbore)
          if isinstance(n, ast.FunctionDef)}
    corp = ast.unparse(fn['main'])
    i_bucla = corp.index('while True')
    i_json = corp.index("a.json")
    assert i_json > i_bucla, "raportul se scrie inainte de bucla?"
    # si ridicarea trebuie sa fie o exceptie prinsa de bucla, nu un exit
    assert 'raise KeyboardInterrupt' in src, (
        "semnalul trebuie ridicat ca exceptie, ca bucla sa iasa normal")
    return "SIGTERM si SIGINT -> iesire normala, raportul se scrie"


def test_rularea_se_opreste_dupa_handback():
    """Campania ruleaza o SINGURA secventa per rulare. Odata in HANDBACK nu
    mai e nimic de masurat, iar la RTF 0.25 restul bugetului de --seconds
    inseamna minute de asteptare reala."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert '--stop-after-handback' in src
    assert "'HANDBACK'" in src
    # implicit pornit: o rulare care nu se opreste singura blocheaza
    # campania pe prima conditie
    import re
    m = re.search(r"--stop-after-handback'[^)]*default=([0-9.]+)", src,
                  re.S)
    assert m and float(m.group(1)) > 0, (
        "oprirea dupa HANDBACK trebuie sa fie pornita implicit")
    return f"implicit {m.group(1)} s dupa HANDBACK"


def test_statisticile_se_iau_doar_pe_fazele_care_conduc_controlul():
    """Eroarea detectorului e o masura a SISTEMULUI doar acolo unde detectia
    conduce controlul. Cadrele din `IDLE` - de dinainte de handover, cu
    vehiculul zburand spre punct si markerul mult in afara axei - au p95 de
    14.6 grade, fata de 2.3 in `DESCEND_TRACK`. Incluse, mutau statistica
    intregii campanii si aratau ca o problema de detector (§5.52).

    Lista e POZITIVA (§5.25): o faza noua nu intra in statistici din
    greseala, la fel ca `DETECTION_MONITORED_PHASES` din supervizor."""
    import importlib
    mod = importlib.import_module('nova_sim')
    faze = set(mod.FAZE_MASURATE)
    assert 'DESCEND_TRACK' in faze and 'FINAL_DESCENT' in faze, faze
    for afara in ('IDLE', 'ASCENT', 'HANDBACK', 'HANDOVER_CHECK', 'REJECT',
                  'ACQUIRE', 'TOUCHDOWN_CONFIRM'):
        assert afara not in faze, f"{afara} nu conduce controlul prin detectie"
    # si lista chiar filtreaza, nu e doar declarata
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'self.sm.state in FAZE_MASURATE' in src, (
        "lista e declarata dar nu filtreaza nimic")
    assert 'if masurabil:' in src
    # iar CSV-ul pastreaza TOATE cadrele, ca analiza sa poata vedea si restul
    assert 'self.rows.append(rand)' in src
    return f"{len(faze)} faze masurate; CSV-ul pastreaza tot"


def test_adevarul_tacut_e_semnalat_nu_ignorat():
    """§5.33 aplicat pozelor: abonarea la un topic inexistent REUSESTE.

    Cu `--world` gresit, SimTruth tace la nesfarsit si raportul iese cu zero
    comparatii, fara sa spuna de ce. Bucla verifica o data, dupa 5 s de
    simulare, si numeste cauza probabila."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert 'n_msgs == 0' in src, "nimeni nu verifica daca adevarul soseste"
    assert 'pose/info' in src, "avertismentul nu spune ce sa verifice"
    assert '--world gresit' in src, "avertismentul nu numeste cauza probabila"
    # si contorul pe care se sprijina exista chiar in SimTruth
    t = sim_truth.StaticTruth()
    assert t.n_msgs == 0, t.n_msgs
    return "verificat o data, cu cauza numita"


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


def test_detectorul_din_productie_expune_contorul_de_cadre():
    """Testul de mai sus isi injecteaza un obiect fals cu `n_frames`.

    Deci verifica aritmetica lui `_note_frames`, nu faptul ca detectorul
    REAL are ce sa-i dea. Intre timp `nova_sim` a trecut de la
    `ArucoMarkerDetector` (care are contorul) la `PiDetector` (care il
    tinea ascuns in `self.det`), iar `getattr(detector, 'n_frames', None)`
    a inceput sa intoarca None. Rezultatul: numararea ratarilor se oprea in
    prima instructiune, iar campania raporta `rata_detectie = 1.000` in
    toate cele 10 rulari - inclusiv daca detectorul ar fi fost mort.

    Cazul negativ al testului precedent exista, dar il declansa pe o clasa
    care nu zboara nicaieri (§5.40). Asta se uita la cea care zboara."""
    from nova import detector_pi

    assert hasattr(detector_pi.PiDetector, 'n_frames'), (
        "PiDetector nu expune n_frames, deci _note_frames nu poate numara "
        "ratarile si rata de detectie iese 100% orice s-ar intampla")

    # si chiar urmareste contorul interior, nu e un zero decorativ
    d = detector_pi.PiDetector.__new__(detector_pi.PiDetector)

    class _Aruco:
        n_frames = 0
    d.det = _Aruco()
    assert d.n_frames == 0, d.n_frames
    d.det.n_frames = 7
    assert d.n_frames == 7, (
        f"n_frames nu urmareste detectorul interior: {d.n_frames}")

    # ce citeste chiar nova_sim, pe acelasi obiect
    assert getattr(d, 'n_frames', None) == 7, (
        "getattr - exact apelul din _note_frames - nu vede contorul")
    return "PiDetector.n_frames deleaga la ArucoMarkerDetector"


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


def test_pragul_de_scoring_e_atins_la_orice_rotatie():
    """8.3.3 se declanseaza pe `marker_px > scoring_px`. Dar markerul nu
    poate creste oricat: cutia lui de incadrare trebuie sa ramana in cadru
    (§5.49), deci exista un `marker_px` MAXIM detectabil, care scade cu
    rotatia.

    Cu pragul implicit de 980 px, captura e fizic imposibila peste ~18 grade
    de yaw - iar rotatia la handover e intamplatoare. Prima campanie de 10
    rulari: HANDBACK in toate, captura in NICIUNA."""
    import math as _m
    H = 1296
    limita = 0.95 * H

    def px_maxim(yaw):
        f = abs(_m.cos(_m.radians(yaw))) + abs(_m.sin(_m.radians(yaw)))
        return limita / f

    assert px_maxim(0) > 980, "la nadir pragul implicit e atins"
    assert px_maxim(20) < 980, (
        f"la 20 grade maximul e {px_maxim(20):.0f} px: 980 e de neatins")
    assert px_maxim(45) < 980
    # pragul de rezerva trebuie atins la ORICE rotatie
    from nova import state_machine as sm_mod
    propus = sm_mod.SCORING_PX
    assert px_maxim(45) > propus, (
        f"nici {propus:.0f} px nu e atins la 45 grade "
        f"(maxim {px_maxim(45):.0f})")
    return (f"980 px: imposibil peste ~18 deg; {propus:.0f} px: atins pana "
            f"la 45 deg (maxim {px_maxim(45):.0f})")


def test_incadrarea_nu_se_subtiaza_cu_rotatia():
    """De ce pragul e pe `fill` si nu pe pixeli.

    Un prag fix in pixeli are o marja care se prabuseste cu rotatia, fiindca
    ce iese din cadru e CUTIA, nu latura. Pe incadrare marja e aceeasi la
    orice rotatie, fiindca `fill` e chiar marimea care decide iesirea.

    Cifrele de pierdere sunt masurate in Gazebo (§5.49 la rotatie 0,
    §5.54 la 41 grade), nu deduse."""
    import math as _m
    from nova import state_machine as sm_mod

    H = 1296.0
    # fill la care s-a pierdut detectia, masurat
    PIERDERE = {0.0: 1184.0 / H, 41.0: 759.0 * 1.410 / H}

    def k(yaw):
        return abs(_m.cos(_m.radians(yaw))) + abs(_m.sin(_m.radians(yaw)))

    marje_px, marje_fill = {}, {}
    for yaw, fill_pierdere in PIERDERE.items():
        px_pierdere = fill_pierdere * H / k(yaw)
        marje_px[yaw] = px_pierdere / sm_mod.SCORING_PX
        marje_fill[yaw] = fill_pierdere / sm_mod.SCORING_FILL

    # pragul in pixeli: marja se strange vizibil cu rotatia
    assert marje_px[0.0] > 1.5, marje_px
    assert marje_px[41.0] < 1.15, (
        f"pragul in px ar trebui sa aiba marja subtire la 41 grade: "
        f"{marje_px[41.0]:.2f}")

    # pragul pe incadrare: marja ramane utilizabila la ORICE rotatie
    for yaw, m in marje_fill.items():
        assert m >= 1.30, (
            f"marja pe incadrare la {yaw} grade e doar {m:.2f}x")

    # si nu se strange nici macar de doua ori intre cele doua rotatii,
    # spre deosebire de cea in pixeli
    #
    # `fill` NU egalizeaza marja complet: la 41 grade randarea reala pierde
    # markerul ceva mai devreme decat prezice incadrarea pura (fill 0.826
    # fata de 0.914 la nadir, §5.54). Deci ramane o imprastiere - doar ca
    # mult mai mica decat cea a pragului in pixeli, si in partea sigura.
    imprastiere_px = max(marje_px.values()) / min(marje_px.values())
    imprastiere_fill = max(marje_fill.values()) / min(marje_fill.values())
    assert imprastiere_fill < imprastiere_px, (
        f"incadrarea nu strange deloc imprastierea: px {imprastiere_px:.2f}x, "
        f"fill {imprastiere_fill:.2f}x")
    assert imprastiere_fill < 1.25, (
        f"marja pe incadrare inca variaza cu {imprastiere_fill:.2f}x intre "
        f"rotatii - prea mult ca sa fie un criteriu independent de rotatie")
    return (f"marja px {marje_px[0.0]:.2f}x -> {marje_px[41.0]:.2f}x; "
            f"fill {marje_fill[0.0]:.2f}x -> {marje_fill[41.0]:.2f}x")


def test_captura_e_garantat_inaintea_coborarii_verticale():
    """§5.51: cu o constanta in pixeli si una in metri, ordinea celor doua
    depinde de rotatie - iar cand s-a inversat, captura 8.3.3 s-a pierdut in
    TOATE cele 10 rulari, cu campania raportand 100% succes.

    Pe incadrare ordinea e o proprietate a constantelor, nu un noroc de
    geometrie: acelasi criteriu, doua praguri, deci nu se pot incrucisa."""
    from nova import state_machine as sm_mod
    assert sm_mod.SCORING_FILL < sm_mod.FINAL_FILL, (
        f"captura ({sm_mod.SCORING_FILL}) trebuie sa se declanseze INAINTEA "
        f"coborarii verticale ({sm_mod.FINAL_FILL})")

    # plasa de altitudine nu are voie sa preia inaintea capturii: sub ea,
    # FINAL_DESCENT ar porni fara sa se fi facut poza (exact bug-ul din 5.51)
    from nova.detection import CameraModel, CAM_HEIGHT_M
    cam = CameraModel()
    cel_mai_jos = None
    for yaw in (0.0, 15.0, 30.0, 45.0):
        import math as _m
        k = abs(_m.cos(_m.radians(yaw))) + abs(_m.sin(_m.radians(yaw)))
        px = sm_mod.FINAL_FILL * cam.height_px / k
        raza = cam.focal_px * cam.marker_size_m / px
        alt = raza + CAM_HEIGHT_M + 0.01          # 0.01: planul markerului
        cel_mai_jos = alt if cel_mai_jos is None else min(cel_mai_jos, alt)
    assert sm_mod.NO_LATERAL_ALT_M < cel_mai_jos, (
        f"plasa de altitudine ({sm_mod.NO_LATERAL_ALT_M} m) e peste "
        f"altitudinea la care incadrarea ar declansa oricum "
        f"({cel_mai_jos:.3f} m): ar taia captura")
    return (f"captura {sm_mod.SCORING_FILL} < verticala {sm_mod.FINAL_FILL}; "
            f"plasa {sm_mod.NO_LATERAL_ALT_M} m sub {cel_mai_jos:.2f} m")


def test_succesul_cere_si_captura_nu_doar_HANDBACK():
    """Criteriul era `stare_finala == HANDBACK`, care nu poate detecta
    lipsa capturii. Campania a raportat 100% pe 10 rulari in care 8.3.3 nu
    a fost indeplinit in niciuna - §5.11, de data asta in criteriul de
    succes al campaniei, nu intr-un test."""
    r = batch_sim.row_from(
        {'idx': 0}, 'x', succes=True,
        raport={'succes': True, 'scoring_ok': True, 'stare_finala': 'HANDBACK'})
    assert r['succes'] == 1
    src = open(os.path.join(REPO, 'tools', 'batch_sim.py')).read()
    assert "raport.get('scoring_ok')" in src, (
        "succesul nu verifica captura de scoring")
    assert 'FARA captura de scoring' in src, (
        "motivul de esec nu distinge lipsa capturii de o secventa oprita")
    assert '--scoring-px' in src, "pragul nu e reglabil din campanie"
    return "HANDBACK fara captura = ESEC, cu motiv distinct"


def test_captura_se_numara_din_eveniment_nu_din_stare():
    """`on_detection` emite `scoring_capture` si in DESCEND_TRACK, si in
    FINAL_DESCENT, dar schimba starea doar din prima. Cu `no_lateral_alt`
    ridicat, coborarea intra in FINAL_DESCENT inainte de pragul in pixeli -
    deci o instrumentare legata de stare raporteaza `None` desi captura se
    poate produce."""
    src = open(os.path.join(REPO, 'tools', 'nova_sim.py')).read()
    assert "nume != 'scoring_capture'" in src, (
        "captura nu se numara din eveniment")
    assert "if st == 'SCORING_CAPTURE'" not in src, (
        "a ramas instrumentarea legata de tranzitia de stare")
    sm = open(os.path.join(REPO, 'nova', 'state_machine.py')).read()
    assert 'State.DESCEND_TRACK, State.FINAL_DESCENT' in sm, (
        "presupunerea testului despre fazele de captura nu mai tine")
    return "numarata din on_event, in ambele faze"


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


def test_raza_lasa_buget_de_inclinare():
    """Limita nu e "markerul incape stand drept", ci "secventa e
    RECUPERABILA": vehiculul se inclina ca sa corecteze, iar inclinarea muta
    amprenta camerei in directia gresita.

    Masurat: la limita vechii formule bugetul ramas era ~4 grade, iar prima
    corectie l-a depasit imediat - handover la 7.17 m cu 2.95 m lateral,
    admis 14.0 grade, folosit 19.3 (§5.48)."""
    import math as _m
    k = _m.tan(_m.radians(batch_sim.HALF_VFOV_DEG))
    for h in (5.0, 8.0, 12.0):
        lat = batch_sim.raza_max(h)
        ramas = _m.degrees(_m.atan(k - (lat + batch_sim.MARKER_HALF_M) / h))
        assert ramas >= batch_sim.TILT_BUDGET_DEG - 0.1, (
            f"la {h} m si {lat:.2f} m lateral raman doar {ramas:.1f} deg")
    # si nu mai atinge limita portii la nicio altitudine din fereastra
    assert batch_sim.raza_max(12.0) < batch_sim.MARKER_RADIUS_M, (
        "6.5 m nu e recuperabil la nicio altitudine; vezi elementul 28")
    assert batch_sim.raza_max(5.0) < 2.0, batch_sim.raza_max(5.0)
    assert batch_sim.raza_max(8.0) > batch_sim.raza_max(5.0)
    for c in batch_sim.plan(60, seed=11):
        lim = batch_sim.raza_max(c['alt_handover'])
        assert c['raza_m'] <= lim + 1e-9, (
            f"raza {c['raza_m']} peste limita {lim} la "
            f"{c['alt_handover']} m")
    return (f"5 m -> {batch_sim.raza_max(5.0):.1f} m, "
            f"12 m -> {batch_sim.raza_max(12.0):.1f} m, "
            f"cu {batch_sim.TILT_BUDGET_DEG:.0f} deg de buget")


def test_vehiculul_ajunge_zburand_nu_planand():
    """Conditia initiala a segmentului autonom e viteza reziduala de la
    handover, nu un hover perfect.

    Prima varianta lasa vehiculul sa decoleze vertical si sa astepte, cu
    markerul deplasat: geometric identic, dar fara piciorul de zbor. In
    cursa pilotul AJUNGE zburand, iar tranzitoria aia e cea mai probabila
    sursa de oscilatie de pendul."""
    for c in batch_sim.plan(60, seed=13):
        assert c['dist_zbor_m'] > 1.0, (
            f"rularea {c['idx']}: zbor de {c['dist_zbor_m']} m - practic "
            f"decolare verticala")
    cmd = batch_sim.fly_cmd(8.0, 3.0, -2.0)
    assert '--north' in cmd and '--east' in cmd, cmd
    assert cmd[cmd.index('--north') + 1] == '3.0'
    assert cmd[cmd.index('--east') + 1] == '-2.0'
    return "toate rularile au un picior de zbor real"


def test_apropierea_nu_e_dreapta_peste_marker():
    """Directia de apropiere (acasa -> handover) si offsetul final
    (handover -> marker) au azimuturi INDEPENDENTE.

    Daca vehiculul ar veni mereu de-a lungul offsetului, ar ajunge cu viteza
    indreptata exact spre marker - un singur caz, si cel mai favorabil.
    Unghiul dintre ele trebuie sa acopere tot cercul."""
    unghiuri = []
    for c in batch_sim.plan(200, seed=17):
        # directia de zbor, la sosire
        zn, ze = c['ho_n'], c['ho_e']
        # directia catre marker, din punctul de handover
        mn, me = c['marker_n'] - c['ho_n'], c['marker_e'] - c['ho_e']
        a = math.atan2(ze, zn) - math.atan2(me, mn)
        unghiuri.append(abs(math.degrees((a + math.pi) % (2 * math.pi)
                                         - math.pi)))
    assert max(unghiuri) > 150.0, (
        f"nicio apropiere dinspre partea opusa: max {max(unghiuri):.0f} deg")
    assert min(unghiuri) < 30.0, (
        f"nicio apropiere aliniata: min {min(unghiuri):.0f} deg")
    medie = sum(unghiuri) / len(unghiuri)
    assert 60.0 < medie < 120.0, (
        f"unghiurile nu acopera cercul: medie {medie:.0f} deg")
    return f"unghi zbor-vs-offset: {min(unghiuri):.0f}-{max(unghiuri):.0f} deg"


def test_offsetul_de_handover_ramane_in_conul_camerei():
    """Markerul poate sta oriunde in lume, dar offsetul de la handover
    ramane plafonat de cadrul camerei (§5.37). Cele doua sunt variabile
    diferite; doar a doua e constransa."""
    for c in batch_sim.plan(100, seed=19):
        lim = batch_sim.raza_max(c['alt_handover'])
        assert c['raza_m'] <= lim + 1e-9, (c['idx'], c['raza_m'], lim)
        d = math.hypot(c['ho_n'] - c['marker_n'], c['ho_e'] - c['marker_e'])
        assert abs(d - c['raza_m']) < 1e-2, (
            f"raza_m {c['raza_m']} nu e distanta reala handover-marker {d}")
    return "offsetul plafonat; plasarea markerului, libera"


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


def test_niciun_pas_din_campanie_nu_asteapta_o_tasta():
    """Prima rulare cap-coada a ramas blocata aici.

    `sim_handover.py` cere Enter cand nu i se da `--after`. Gazebo, SITL si
    decolarea trecusera; procesul traia, nu tiparea nimic si astepta la
    infinit. Intr-o campanie nesupravegheata asta nu e un esec - e o rulare
    care nu se termina niciodata, ceea ce e mai rau."""
    c = batch_sim.handover_cmd()
    assert '--after' in c, (
        "handover-ul din campanie trebuie sa ridice AUX singur: " + ' '.join(c))
    i = c.index('--after')
    assert float(c[i + 1]) > 0, c
    # fereastra de asezare a portii e 1.0 s (§8); ridicarea trebuie sa o
    # depaseasca, altfel poarta nu apuca sa masoare amplitudinea manselor
    assert float(c[i + 1]) >= 1.0, (
        f"--after {c[i + 1]} e sub fereastra de asezare de 1 s a portii")
    return f"--after {c[i + 1]} s, fara tastatura"


def test_copiii_nu_mostenesc_tastatura_si_nu_tamponeaza():
    """Doua gărzi in `spawn`, amandoua invizibile pana cand doare.

    `stdin` inchis: un `input()` da EOFError - esec vizibil - in loc de
    asteptare tacuta. `PYTHONUNBUFFERED`: fara el, logul unui proces care
    scrie intr-un fisier ramane GOL pana iese, adica exact cand ai nevoie
    de el."""
    src = open(os.path.join(REPO, 'tools', 'batch_sim.py')).read()
    fn = None
    for nod in ast.parse(src).body:
        if isinstance(nod, ast.FunctionDef) and nod.name == 'spawn':
            fn = nod
    assert fn is not None
    text = ast.unparse(fn)
    assert 'stdin=subprocess.DEVNULL' in text, "spawn lasa stdin deschis"
    assert 'PYTHONUNBUFFERED' in text, "spawn nu opreste tamponarea"
    return "stdin inchis, iesire netamponata"


def test_campania_nu_lasa_vehiculul_in_LOITER_fara_manse():
    """In LOITER, throttle-ul comanda urcare/coborare, iar emitatorul
    simulat al SITL-ului il tine JOS.

    Intre iesirea lui `fly_to` si pornirea injectorului RC nu e nimeni pe
    manse. Prima campanie cap-coada a aterizat exact asa: 10.59 m -> 0.19 m,
    orb, inainte de handover. Poarta a refuzat corect, iar in Gazebo arata
    ca o aterizare fara centrare."""
    c = batch_sim.fly_cmd(8.0, 1.0, 2.0)
    assert '--end-mode' in c, "campania nu spune in ce mod ramane vehiculul"
    assert c[c.index('--end-mode') + 1] == 'guided', (
        "campania lasa vehiculul in LOITER fara injector activ: " + ' '.join(c))
    # uzul manual pastreaza LOITER, care e modul realist pentru un pilot
    import sim_fly_to
    assert sim_fly_to.main.__doc__ is None or True
    src = open(os.path.join(REPO, 'tools', 'sim_fly_to.py')).read()
    assert "default='loiter'" in src, (
        "implicitul pentru uz manual nu mai e LOITER")
    return "campanie: guided; manual: loiter"


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
    ('adevarul e al CAMEREI, nu al vehiculului',
     test_adevarul_e_al_CAMEREI_nu_al_vehiculului),
    ('yaw-ul vine de la FC, nu din cuaternionul ENU',
     test_yaw_ul_vine_de_la_FC_nu_din_cuaternionul_ENU),
    ('range-ul adevarat e pe axa optica, nu vertical',
     test_range_ul_adevarat_e_pe_axa_optica_nu_vertical),
    ('unghiurile se compara in cadrul corpului',
     test_unghiurile_se_compara_in_cadrul_corpului),
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
    ('cele trei fisiere raman acoperite',
     test_cele_trei_fisiere_raman_acoperite),
    ('E0 e ocolita explicit si anuntata',
     test_E0_e_ocolita_explicit_si_anuntata),
    ('vehiculul foloseste acelasi ceas ca bucla',
     test_vehiculul_foloseste_acelasi_ceas_ca_bucla),
    ('ceasul gresit ar da varsta negativa',
     test_ceasul_gresit_ar_da_varsta_negativa),
    ('timpul per stare se aduna', test_timpul_per_stare_se_aduna),
    ('eroarea si deriva se masoara din adevar',
     test_eroarea_si_deriva_se_masoara_din_adevar),
    ('latenta se masoara in ceasul sursei',
     test_latenta_se_masoara_in_ceasul_sursei),
    ('inclinarea intra in verificarea de incadrare',
     test_inclinarea_intra_in_verificarea_de_incadrare),
    ('diagnosticul de pierdere distinge cauza',
     test_diagnosticul_de_pierdere_distinge_cauza),
    ('optiunile de experiment ajung din campanie in aplicatie',
     test_optiunile_de_experiment_ajung_din_campanie_in_aplicatie),
    ('pragul de coborare verticala e reglabil fara cod',
     test_pragul_de_coborare_verticala_e_reglabil_fara_cod),
    ('raportul se scrie si la oprire din afara',
     test_raportul_se_scrie_si_la_oprire_din_afara),
    ('rularea se opreste dupa handback',
     test_rularea_se_opreste_dupa_handback),
    ('statisticile se iau doar pe fazele care conduc controlul',
     test_statisticile_se_iau_doar_pe_fazele_care_conduc_controlul),
    ('adevarul tacut e semnalat, nu ignorat',
     test_adevarul_tacut_e_semnalat_nu_ignorat),
    ('fara adevar nu se inventeaza cifre',
     test_fara_adevar_nu_se_inventeaza_cifre),
    ('rata de detectie poate scadea sub 100%',
     test_rata_de_detectie_poate_scadea_sub_100),
    ('contorul nu da ratari negative', test_contorul_nu_da_ratari_negative),
    ('pragul de scoring e atins la orice rotatie',
     test_pragul_de_scoring_e_atins_la_orice_rotatie),
    ('succesul cere si captura, nu doar HANDBACK',
     test_succesul_cere_si_captura_nu_doar_HANDBACK),
    ('captura se numara din eveniment, nu din stare',
     test_captura_se_numara_din_eveniment_nu_din_stare),
    ('CSV acopera ce produce randul', test_CSV_acopera_ce_produce_randul),
    ('nicio suita nu uita sa inregistreze un test',
     test_nicio_suita_nu_uita_sa_inregistreze_un_test),
    ('planul e reproductibil', test_planul_e_reproductibil),
    ('raza lasa buget de inclinare', test_raza_lasa_buget_de_inclinare),
    ('vehiculul ajunge zburand, nu pland',
     test_vehiculul_ajunge_zburand_nu_planand),
    ('apropierea nu e dreapta peste marker',
     test_apropierea_nu_e_dreapta_peste_marker),
    ('offsetul de handover ramane in conul camerei',
     test_offsetul_de_handover_ramane_in_conul_camerei),
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
    ('niciun pas din campanie nu asteapta o tasta',
     test_niciun_pas_din_campanie_nu_asteapta_o_tasta),
    ('copiii nu mostenesc tastatura si nu tamponeaza',
     test_copiii_nu_mostenesc_tastatura_si_nu_tamponeaza),
    ('campania masoara ce zboara', test_campania_masoara_ce_zboara),
    ('incadrarea nu se subtiaza cu rotatia',
     test_incadrarea_nu_se_subtiaza_cu_rotatia),
    ('captura e garantat inaintea coborarii verticale',
     test_captura_e_garantat_inaintea_coborarii_verticale),
    ('detectorul din productie expune contorul de cadre',
     test_detectorul_din_productie_expune_contorul_de_cadre),
    ('campania nu lasa vehiculul in LOITER fara manse',
     test_campania_nu_lasa_vehiculul_in_LOITER_fara_manse),
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
