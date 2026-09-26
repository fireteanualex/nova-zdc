#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru estimatorul ExtNav (nova/extnav.py).

    python3 tools/test_extnav.py

Fara camera si fara FC: detectiile se construiesc din geometrie CUNOSCUTA
(marker la o pozitie data, vehicul la o poza data), trec prin estimator si
rezultatul se compara cu adevarul. Vehiculul e clasa din productie
(nova.vehicle.Vehicle, fara conexiune), ca interpolarea atitudinii sa fie
cea care zboara, nu un dublu (§5.56).
"""

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import extnav as ex                               # noqa: E402
from nova.detection import Detection                        # noqa: E402
from nova.vehicle import Vehicle                            # noqa: E402


def vehicul():
    """Vehicle real, neconectat, cu o legatura falsa care doar noteaza."""
    v = Vehicle('udpin:127.0.0.1:1')
    trimise = []

    class _Mav:
        def vision_position_estimate_send(self, *a):
            trimise.append(a)

    v.m = types.SimpleNamespace(mav=_Mav(), target_system=1, target_component=1)
    v.link_healthy = True
    v.link_verbose = False
    v.trimise = trimise
    return v


def detectie_din_adevar(t, veh_ned, marker_ned, roll, pitch, yaw):
    """Detection pe care ar publica-o un detector perfect: raza spre marker,
    din NED in corp cu atitudinea data, apoi unghiurile din nova/detection.py."""
    d = (marker_ned[0] - veh_ned[0], marker_ned[1] - veh_ned[1],
         marker_ned[2] - veh_ned[2])
    b = ex.ned_to_body(d, roll, pitch, yaw)
    assert b[2] > 0, "markerul e deasupra vehiculului"
    return Detection(t=t, angle_x=math.atan2(b[0], b[2]),
                     angle_y=math.atan2(b[1], b[2]),
                     distance_m=math.sqrt(sum(c * c for c in d)),
                     marker_px=100.0, range_m=-d[2], fill=None)


def aproape(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


# --- teste -----------------------------------------------------------------

def test_rotatia_corp_NED_e_inversabila_si_corecta():
    """Conventia ArduPilot: R = Rz(yaw) Ry(pitch) Rx(roll), corp FRD -> NED.
    Verificat pe cazuri cu raspuns cunoscut, nu doar dus-intors."""
    # yaw 90: nasul spre EST -> "inainte" in corp devine est in NED
    assert aproape(ex.body_to_ned((1, 0, 0), 0, 0, math.pi / 2), (0, 1, 0))
    # pitch +10 grade (nasul sus): burta priveste INAINTE si in jos, deci
    # axa "jos" a corpului capata componenta spre nord (camera vede in fata)
    n = ex.body_to_ned((0, 0, 1), 0, math.radians(10), 0)
    assert n[0] > 0 and abs(n[2] - math.cos(math.radians(10))) < 1e-9, n
    assert abs(n[0] - math.sin(math.radians(10))) < 1e-9, n
    # roll +10 (aripa dreapta jos): "dreapta" in corp capata componenta in jos
    n = ex.body_to_ned((0, 1, 0), math.radians(10), 0, 0)
    assert n[2] > 0 and abs(n[1] - math.cos(math.radians(10))) < 1e-9, n
    for rpy in ((0.1, -0.2, 0.7), (-0.3, 0.05, -2.5), (0.0, 0.0, 3.1)):
        v = (0.3, -0.7, 1.2)
        assert aproape(ex.ned_to_body(ex.body_to_ned(v, *rpy), *rpy), v, 1e-9)
    return "yaw/pitch/roll cu semnul asteptat; dus-intors exact"


def test_pozitia_in_cadrul_markerului_la_nivel():
    """Vehicul la 10 m, drept, cu nasul spre nord; markerul la 2 m nord si
    1 m est de el. Vehiculul e deci la (-2, -1, -10) in cadrul markerului."""
    v = vehicul()
    v._on_attitude(5.0, 0.0, 0.0, 0.0)
    v.rel_alt = 10.0
    est = ex.ExtNavEstimator(v)
    det = detectie_din_adevar(5.0, (0, 0, -10), (2, 1, 0), 0, 0, 0)
    e = est.estimate(det)
    assert e is not None, est.last_refusal
    assert aproape((e.x, e.y, e.z), (-2.0, -1.0, -10.0), 1e-9), (e.x, e.y, e.z)
    assert abs(e.lateral_m - math.hypot(2, 1)) < 1e-9
    assert est.send(e) is True
    usec, x, y, z, r, p, yw = v.trimise[-1]
    assert usec == 5_000_000, usec
    assert aproape((x, y, z), (-2.0, -1.0, -10.0), 1e-9), (x, y, z)
    assert (r, p, yw) == (0.0, 0.0, 0.0) and v.n_vpe == 1
    return "(-2, -1, -10) recuperat exact; VPE cu usec = t_captura"


def test_yaw_ul_vehiculului_nu_schimba_pozitia_in_NED():
    """Acelasi marker, vehiculul rotit spre est: unghiurile din corp se
    schimba, pozitia in cadrul markerului nu. Iar yaw-ul trimis e al FC-ului,
    nu al markerului (brief §4)."""
    v = vehicul()
    yaw = math.radians(90)
    v._on_attitude(5.0, 0.0, 0.0, yaw)
    v.rel_alt = 10.0
    est = ex.ExtNavEstimator(v)
    det = detectie_din_adevar(5.0, (0, 0, -10), (2, 1, 0), 0, 0, yaw)
    # in corp, nordul e la stanga: markerul (2 N, 1 E) e 1 inainte, -2 dreapta
    assert abs(math.tan(det.angle_x) * 10 - 1.0) < 1e-9
    assert abs(math.tan(det.angle_y) * 10 + 2.0) < 1e-9
    e = est.estimate(det)
    assert aproape((e.x, e.y), (-2.0, -1.0), 1e-9), (e.x, e.y)
    assert abs(e.yaw - yaw) < 1e-12
    return "yaw 90: unghiuri rotite, pozitie identica, yaw trimis = al FC-ului"


def test_inclinarea_se_compenseaza_cu_atitudinea_de_la_captura():
    """CAZUL CARE CONTEAZA (brief §3): in hover, 3 grade la 10 m muta
    centrul imaginii cu ~0.5 m la sol. Un prag in pixeli fara compensare
    ar confunda inclinarea cu eroarea. Vehicul EXACT deasupra markerului,
    inclinat 3 grade: estimarea trebuie sa dea 0, nu 0.52 m."""
    v = vehicul()
    roll, pitch = math.radians(3.0), math.radians(-2.0)
    v._on_attitude(5.0, roll, pitch, 0.4)
    v.rel_alt = 10.0
    est = ex.ExtNavEstimator(v)
    det = detectie_din_adevar(5.0, (0, 0, -10), (0, 0, 0), roll, pitch, 0.4)
    # fara compensare, raza (unghiurile) ar da ~0.5 m
    necompensat = 10.0 * math.hypot(math.tan(det.angle_x), math.tan(det.angle_y))
    assert 0.4 < necompensat < 0.7, necompensat
    e = est.estimate(det)
    assert abs(e.x) < 1e-9 and abs(e.y) < 1e-9, (e.x, e.y)
    # si un caz general: inclinat, rotit, marker deplasat
    m = (-3.0, 4.0, 0.0)
    det = detectie_din_adevar(5.0, (1.0, 1.0, -8.0), m, roll, pitch, 0.4)
    v.rel_alt = 8.0
    e = est.estimate(det)
    assert aproape((e.x, e.y, e.z), (1.0 - m[0], 1.0 - m[1], -8.0), 1e-9), (e.x, e.y, e.z)
    return f"3 grade la 10 m: necompensat {necompensat:.2f} m, estimat 0.00 m"


def test_atitudinea_e_cea_de_la_captura_nu_ultima():
    """Cadrul e capturat la t=5.0 cu vehiculul drept; pana la publicare
    (t=5.1) vehiculul s-a inclinat 5 grade. Folosita ultima atitudine,
    estimarea ar fi cu ~0.9 m alaturi la 10 m."""
    v = vehicul()
    v._on_attitude(4.95, 0.0, 0.0, 0.0)
    v._on_attitude(5.05, 0.0, 0.0, 0.0)
    v._on_attitude(5.10, math.radians(5.0), 0.0, 0.0)
    v.rel_alt = 10.0
    est = ex.ExtNavEstimator(v)
    det = detectie_din_adevar(5.0, (0, 0, -10), (0, 0, 0), 0, 0, 0)
    e = est.estimate(det)
    assert abs(e.x) < 1e-9 and abs(e.y) < 1e-9, (e.x, e.y)
    assert e.roll == 0.0, "a folosit ultima atitudine, nu cea de la captura"
    # NEGATIV: fara atitudine in jurul lui t -> nicio estimare, spusa
    det2 = detectie_din_adevar(2.0, (0, 0, -10), (0, 0, 0), 0, 0, 0)
    assert est.estimate(det2) is None and 'atitudine' in est.last_refusal
    return "atitudinea interpolata la t_c; fara istoric -> refuz explicit"


def test_refuzurile_nu_trimit_nimic():
    """Fara barometru, sub 0.3 m, sau raza spre orizont: None, contorizat,
    si niciun VPE. O pozitie inventata trimisa EKF-ului ar fi mai rea decat
    niciuna."""
    v = vehicul()
    v._on_attitude(5.0, 0.0, 0.0, 0.0)
    est = ex.ExtNavEstimator(v)
    det = detectie_din_adevar(5.0, (0, 0, -10), (1, 0, 0), 0, 0, 0)
    v.rel_alt = None
    assert est.estimate(det) is None and 'barometric' in est.last_refusal
    v.rel_alt = 0.2
    assert est.estimate(det) is None and 'sub' in est.last_refusal
    v.rel_alt = 10.0
    # raza aproape orizontala: vehicul inclinat 88 de grade
    v._on_attitude(6.0, math.radians(88.0), 0.0, 0.0)
    det_h = Detection(t=6.0, angle_x=0.0, angle_y=0.0, distance_m=10,
                      marker_px=50, range_m=10, fill=None)
    assert est.estimate(det_h) is None and 'planul' in est.last_refusal
    assert est.n_refused == 3 and est.n_estimates == 0 and not v.trimise
    return "3 refuzuri, 0 estimari, 0 mesaje"


def test_tolerantele_din_brief():
    assert abs(ex.tol_m(1.0) - 0.15) < 1e-12
    assert abs(ex.tol_m(12.0) - 1.2) < 1e-12
    assert abs(ex.tol_m(0.5) - 0.15) < 1e-12
    assert abs(ex.consistency_tol_m(2.0) - 0.3) < 1e-12
    assert abs(ex.consistency_tol_m(10.0) - 0.5) < 1e-12
    return "tol(1)=0.15, tol(12)=1.2; consistenta 0.3 / 0.5 la 10 m"


def test_consistenta_a_doua_detectii_compenseaza_miscarea():
    """D7. Vehiculul se misca 1 m intre capturi; markerul e acelasi, deci
    cele doua detectii sunt consistente - DACA se compenseaza miscarea din
    LOCAL_POSITION_NED. O detectie a altui punct nu e consistenta. Fara
    pozitie la momentul capturii: nu e consistenta (necunoscut != de acord)."""
    v = vehicul()
    v._on_attitude(5.0, 0.0, 0.0, 0.0)
    v._on_attitude(7.0, 0.0, 0.0, 0.0)
    v._on_position(5.0, 0.0, 0.0, -10.0, 0.5, 0.0, 0.0)
    v._on_position(7.0, 1.0, 0.0, -10.0, 0.5, 0.0, 0.0)
    v.rel_alt = 10.0
    est = ex.ExtNavEstimator(v)
    m = (3.0, 2.0, 0.0)
    e1 = est.estimate(detectie_din_adevar(5.0, (0, 0, -10), m, 0, 0, 0))
    e2 = est.estimate(detectie_din_adevar(7.0, (1.0, 0, -10), m, 0, 0, 0))
    assert e1 is not None and e2 is not None
    assert abs(e1.off_n - e2.off_n - 1.0) < 1e-9, "offset-urile difera cu miscarea"
    assert est.consistent(e1, e2), "aceeasi tinta, miscare compensata"
    # alta tinta, la 0.6 m: peste max(0.3, 0.05*10)=0.5
    e3 = est.estimate(detectie_din_adevar(7.0, (1.0, 0, -10), (3.6, 2.0, 0), 0, 0, 0))
    assert not est.consistent(e1, e3)
    # in toleranta (0.4 m la 10 m)
    e4 = est.estimate(detectie_din_adevar(7.0, (1.0, 0, -10), (3.4, 2.0, 0), 0, 0, 0))
    assert est.consistent(e1, e4)
    # fara pozitie la momentul capturii
    e5 = est.estimate(detectie_din_adevar(20.0, (1.0, 0, -10), m, 0, 0, 0)) \
        if v.attitude_at(20.0) is not None else None
    v._on_attitude(20.0, 0.0, 0.0, 0.0)
    e5 = est.estimate(detectie_din_adevar(20.0, (1.0, 0, -10), m, 0, 0, 0))
    assert e5 is not None and est.marker_in_ekf_frame(e5) is None
    assert not est.consistent(e1, e5), "fara pozitie nu e consistenta"
    # fereastra: perechea e cea mai noua + cea mai recenta cu care e de acord
    w = ex.DetectionWindow(est)
    assert w.pair() is None
    w.add(e1)
    assert w.pair() is None
    w.add(e3)                         # nu e de acord cu e1
    assert w.pair() is None
    w.add(e2)                         # de acord cu e1, nu cu e3
    p = w.pair()
    assert p is not None and p[0] is e1 and p[1] is e2
    w.clear()
    assert len(w) == 0 and w.pair() is None
    return "miscare de 1 m compensata; 0.6 m respins, 0.4 m acceptat; fara pozitie: nu"


def test_consemnul_de_yaw():
    """Orientarea markerului serveste DOAR consemnului de yaw (brief §4):
    marker rotit 30 de grade spre dreapta in corp -> tinta = yaw + 30. Iar
    markerul e patrat: 100 de grade inseamna 10, nu o intoarcere de 100."""
    y = ex.yaw_setpoint(math.radians(170), 30.0)
    assert abs(math.degrees(y) - (-160.0)) < 1e-9, math.degrees(y)
    assert ex.yaw_setpoint(0.5, None) is None
    assert abs(math.degrees(ex.yaw_setpoint(0.0, 100.0)) - 10.0) < 1e-9
    assert abs(math.degrees(ex.yaw_setpoint(0.0, -100.0)) + 10.0) < 1e-9
    assert abs(math.degrees(ex.yaw_setpoint(0.0, 179.0)) + 1.0) < 1e-9
    assert ex.smallest_align_deg(45.0) == -45.0 and ex.smallest_align_deg(-45.0) == -45.0
    for d in (-720, -100, -46, -45, -10, 0, 10, 44, 45, 100, 359):
        r = ex.smallest_align_deg(d)
        assert -45.0 <= r < 45.0 and abs((d - r) % 90.0) < 1e-9, (d, r)
    return "yaw + orientare, rotatia minima mod 90, in [-45, 45); None fara orientare"


TESTS = [
    ('rotatia corp <-> NED', test_rotatia_corp_NED_e_inversabila_si_corecta),
    ('pozitia in cadrul markerului, la nivel',
     test_pozitia_in_cadrul_markerului_la_nivel),
    ('yaw-ul vehiculului nu schimba pozitia in NED',
     test_yaw_ul_vehiculului_nu_schimba_pozitia_in_NED),
    ('inclinarea se compenseaza cu atitudinea de la captura',
     test_inclinarea_se_compenseaza_cu_atitudinea_de_la_captura),
    ('atitudinea de la captura, nu ultima',
     test_atitudinea_e_cea_de_la_captura_nu_ultima),
    ('refuzurile nu trimit nimic', test_refuzurile_nu_trimit_nimic),
    ('tolerantele din brief', test_tolerantele_din_brief),
    ('consistenta a doua detectii compenseaza miscarea',
     test_consistenta_a_doua_detectii_compenseaza_miscarea),
    ('consemnul de yaw', test_consemnul_de_yaw),
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
        except Exception as e:                      # noqa: BLE001
            fails += 1
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
