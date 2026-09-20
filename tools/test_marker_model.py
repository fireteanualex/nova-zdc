#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru modelul Gazebo al markerului (I1).

    python3 tools/test_marker_model.py

Doua teste fac cea mai mare parte din munca:

  - textura generata se **detecteaza** ca ID 26, iar latura detectata
    corespunde zonei CODATE de 480 mm, nu colii de 600 mm. Confundate, toate
    distantele din `solvePnP` ar iesi cu 25% eroare - si ar arata ca o
    problema de calibrare, nu ca o problema de textura.
  - maparea NED -> ENU se verifica pe fisierul de lume GENERAT, nu pe
    intentie. Nord si est inversate produc exact simptomul unui bug de
    conventie in detector (§5.31).
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402

import make_marker_model as mm                              # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO, 'sim', 'models', 'aruco_26')
WORLD = os.path.join(REPO, 'sim', 'worlds', 'nova_marker.sdf')
TEX = os.path.join(MODEL_DIR, 'materials', 'textures', 'aruco_26.png')


# --- textura ----------------------------------------------------------------

def test_geometria_texturii():
    img, meta = mm.make_texture()
    assert img.shape == (3000, 3000), img.shape
    assert meta['px_per_mm'] == 5.0, meta['px_per_mm']
    assert meta['quiet_mm'] == 60.0, meta['quiet_mm']
    # 6 module (4x4 + bordura), 400 px fiecare, fara rest
    assert meta['module_px'] == 400.0, meta['module_px']
    assert meta['coded_mm'] / meta['sheet_mm'] == 0.8

    # zona codata e exact in mijloc
    off = (3000 - 2400) // 2
    assert off == 300
    rama = np.concatenate([img[:off].ravel(), img[-off:].ravel(),
                           img[:, :off].ravel(), img[:, -off:].ravel()])
    assert (rama == 255).all(), "zona linistita nu e complet alba"

    # o rezolutie care nu da px/mm intreg trebuie refuzata, nu rotunjita
    try:
        mm.make_texture(marker_px=2401)
        raise AssertionError('a acceptat o rezolutie care nu se imparte exact')
    except ValueError:
        pass
    return ("3000x3000 px, 5 px/mm, 400 px/modul, zona linistita 60 mm alba; "
            "rezolutie neintreaga refuzata")


def test_doar_alb_si_negru():
    """Orice nuanta intermediara ar insemna interpolare - muchii inmuiate,
    exact ce localizeaza detectorul (§5.20)."""
    img, _ = mm.make_texture()
    niveluri = np.unique(img)
    assert set(niveluri.tolist()) == {0, 255}, niveluri[:10]
    return f"2 niveluri: {niveluri.tolist()}"


def test_textura_se_detecteaza_si_latura_e_zona_CODATA():
    """Testul central. Latura detectata trebuie sa fie 2400 px (zona codata
    de 480 mm), nu 3000 px (coala de 600 mm).

    Daca cineva ar genera markerul la dimensiunea colii, `detectMarkers` ar
    reusi la fel de bine, dar `solvePnP` cu marker_size_m = 0.48 ar da
    distante cu 25% eroare - si ar arata ca o calibrare proasta."""
    img, meta = mm.make_texture()
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(mm.ARUCO_DICT),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    assert ids is not None, "textura generata nu se detecteaza deloc"
    assert len(ids) == 1, f"{len(ids)} markeri detectati"
    assert int(ids.flatten()[0]) == mm.MARKER_ID, ids

    pts = corners[0].reshape(4, 2)
    laturi = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
    latura = sum(laturi) / 4.0
    assert abs(latura - meta['marker_px']) < 2.0, (
        f"latura detectata {latura:.1f} px, zona codata {meta['marker_px']} px")
    assert abs(latura - meta['sheet_px']) > 500, (
        "latura detectata e cat coala - zona codata a fost generata gresit")

    # si latura in mm, prin px_per_mm, trebuie sa fie 480
    mm_detectat = latura / meta['px_per_mm']
    assert abs(mm_detectat - 480.0) < 0.5, mm_detectat
    return (f"ID {mm.MARKER_ID}, latura {latura:.1f} px = "
            f"{mm_detectat:.1f} mm (zona codata, nu coala de 600)")


def test_fisierul_scris_pe_disc_e_acelasi():
    """Ce e in repo trebuie sa fie ce genereaza unealta acum."""
    assert os.path.exists(TEX), f"lipseste {TEX}"
    pe_disc = cv2.imread(TEX, cv2.IMREAD_GRAYSCALE)
    proaspat, _ = mm.make_texture()
    assert pe_disc is not None and pe_disc.shape == proaspat.shape
    assert (pe_disc == proaspat).all(), (
        "textura din repo difera de ce genereaza unealta - regenereaza")
    return f"{os.path.relpath(TEX, REPO)} identica cu generarea curenta"


# --- SDF --------------------------------------------------------------------

def test_model_sdf_bine_format():
    cfg = os.path.join(MODEL_DIR, 'model.config')
    sdf = os.path.join(MODEL_DIR, 'model.sdf')
    for f in (cfg, sdf):
        assert os.path.exists(f), f"lipseste {f}"
        ET.parse(f)                              # arunca daca nu e XML valid

    root = ET.parse(sdf).getroot()
    model = root.find('model')
    assert model.get('name') == 'aruco_26', model.get('name')
    assert model.find('static').text.strip() == 'true', (
        "markerul trebuie sa fie static, altfel cade sub actiunea gravitatiei")

    vis = model.find('link/visual')
    plane = vis.find('geometry/plane')
    assert plane is not None, "geometria nu e <plane>"
    assert plane.find('normal').text.split() == ['0', '0', '1']
    assert [float(x) for x in plane.find('size').text.split()] == [0.6, 0.6], (
        "planul trebuie sa fie de 0.6 m - coala, nu zona codata")

    albedo = vis.find('material/pbr/metal/albedo_map')
    assert albedo is not None, "lipseste albedo_map"
    cale = os.path.join(MODEL_DIR, albedo.text.strip())
    assert os.path.exists(cale), (
        f"albedo_map arata spre {albedo.text.strip()}, care nu exista relativ "
        f"la model.sdf")

    metal = vis.find('material/pbr/metal/metalness')
    assert float(metal.text) == 0.0, "hartia nu e metalica"
    return "XML valid, static, plan 0.6 m, albedo_map rezolva, metalness 0"


def test_roughness_e_parametru():
    """15.4.7 semnaleaza reflexia; 0.3 trebuie sa fie la un flag distanta."""
    import tempfile
    sdf = os.path.join(MODEL_DIR, 'model.sdf')
    r_implicit = float(ET.parse(sdf).getroot()
                       .find('model/link/visual/material/pbr/metal/roughness').text)
    assert r_implicit == mm.ROUGHNESS_MATE == 0.9, r_implicit

    tmp = tempfile.mkdtemp()
    mm.write_model(tmp, roughness=0.3)
    r = float(ET.parse(os.path.join(tmp, 'model.sdf')).getroot()
              .find('model/link/visual/material/pbr/metal/roughness').text)
    assert r == 0.3, r
    return "implicit 0.9 (mata); --roughness 0.3 ajunge in SDF"


# --- lumea ------------------------------------------------------------------

def _world_root():
    assert os.path.exists(WORLD), f"lipseste {WORLD}"
    return ET.parse(WORLD).getroot().find('world')


def test_NED_la_ENU_in_lumea_generata():
    """§5.31. Verificat pe fisierul GENERAT, cu valori asimetrice: cu
    north == east, o inversiune ar trece testul fara sa fie prinsa."""
    import tempfile
    base = open(mm.find_base_world()).read()
    text = mm.build_world(base, north=7.0, east=-3.0, az=0.0, el=90.0)
    tmp = os.path.join(tempfile.mkdtemp(), 'w.sdf')
    open(tmp, 'w').write(text.replace('{sursa}', 'test'))

    world = ET.parse(tmp).getroot().find('world')
    inc = [i for i in world.findall('include')
           if i.find('uri').text.strip() == 'model://aruco_26']
    assert len(inc) == 1, f"{len(inc)} includeri de marker"
    poza = [float(x) for x in inc[0].find('pose').text.split()]
    x, y, z = poza[0], poza[1], poza[2]
    assert x == -3.0, f"x = {x}, asteptat east = -3.0"
    assert y == 7.0, f"y = {y}, asteptat north = 7.0"
    assert z == mm.MARKER_Z == 0.01, f"z = {z}; 0.01 evita z-fighting"

    # si in lumea din repo, cu valorile implicite din start_sim.sh
    w = _world_root()
    inc0 = [i for i in w.findall('include')
            if i.find('uri').text.strip() == 'model://aruco_26'][0]
    p0 = [float(v) for v in inc0.find('pose').text.split()]
    assert (p0[0], p0[1]) == (1.5, 2.0), (
        f"lumea din repo: x={p0[0]} y={p0[1]}; start_sim.sh are N=2.0 E=1.5")
    return "north=7 east=-3 -> x=-3 y=7 z=0.01; lumea din repo N=2 E=1.5 -> x=1.5 y=2"


def test_directia_soarelui():
    """Cazuri cu raspuns evident, ca formula sa nu poata fi gresita tacut."""
    cazuri = [
        # (az, el) -> (est, nord, sus), lumina CALATORESTE in jos
        ((0.0, 90.0), (0.0, 0.0, -1.0), 'soare la zenit: drept in jos'),
        ((0.0, 0.0), (0.0, -1.0, 0.0), 'soare la nord, la orizont: spre sud'),
        ((90.0, 0.0), (-1.0, 0.0, 0.0), 'soare la est: lumina spre vest'),
        ((180.0, 0.0), (0.0, 1.0, 0.0), 'soare la sud: lumina spre nord'),
        ((270.0, 0.0), (1.0, 0.0, 0.0), 'soare la vest: lumina spre est'),
    ]
    for (az, el), astept, eticheta in cazuri:
        got = mm.sun_direction(az, el)
        for g, a in zip(got, astept):
            assert abs(g - a) < 1e-9, f"{eticheta}: {got} != {astept}"
    # vector unitar la orice unghi
    for az in (0.0, 37.0, 135.0, 300.0):
        for el in (0.0, 15.0, 45.0, 75.0):
            n = math.sqrt(sum(c * c for c in mm.sun_direction(az, el)))
            assert abs(n - 1.0) < 1e-9, (az, el, n)

    w = _world_root()
    d = [float(x) for x in w.find('light/direction').text.split()]
    assert abs(math.sqrt(sum(c * c for c in d)) - 1.0) < 1e-6, d
    assert d[2] < 0, "lumina nu merge in jos"
    return f"{len(cazuri)} cazuri cardinale + unitar; lumea din repo: {d}"


def test_lumea_e_DERIVATA_nu_rescrisa():
    """Plugin-urile si coordonatele sferice trebuie sa ramana EXACT ce
    foloseste ardupilot_gazebo. O lume scrisa de la zero ar diverge tacut la
    prima lor actualizare."""
    baza = ET.parse(mm.find_base_world()).getroot().find('world')
    w = _world_root()

    p_baza = sorted(p.get('filename') for p in baza.findall('plugin'))
    p_nou = sorted(p.get('filename') for p in w.findall('plugin'))
    assert p_baza == p_nou, f"plugin-uri diferite:\n  {p_baza}\n  {p_nou}"

    for camp in ('latitude_deg', 'longitude_deg', 'elevation'):
        a = baza.find(f'spherical_coordinates/{camp}').text.strip()
        b = w.find(f'spherical_coordinates/{camp}').text.strip()
        assert a == b, f"{camp}: {a} vs {b}"

    uri_baza = {i.find('uri').text.strip() for i in baza.findall('include')}
    uri_nou = {i.find('uri').text.strip() for i in w.findall('include')}
    assert uri_baza < uri_nou, f"lipsesc modele din baza: {uri_baza - uri_nou}"
    assert uri_nou - uri_baza == {'model://aruco_26'}, uri_nou - uri_baza

    assert w.get('name') == 'nova_marker', w.get('name')
    return (f"{len(p_nou)} plugin-uri identice, coordonate sferice identice, "
            f"singurul model in plus: aruco_26")


def test_o_singura_lumina_si_e_parametrizabila():
    w = _world_root()
    lumini = w.findall('light')
    assert len(lumini) == 1, f"{len(lumini)} lumini"
    assert lumini[0].get('name') == 'sun'
    assert lumini[0].find('direction') is not None
    assert lumini[0].find('cast_shadows').text.strip() == 'true', (
        "fara umbre, reflexia si unghiul de lumina nu inseamna nimic")
    return "o lumina directionala 'sun', cu umbre si directie parametrizata"


TESTS = [
    ('geometria texturii', test_geometria_texturii),
    ('doar alb si negru', test_doar_alb_si_negru),
    ('textura se detecteaza, latura e zona CODATA',
     test_textura_se_detecteaza_si_latura_e_zona_CODATA),
    ('fisierul de pe disc e acelasi', test_fisierul_scris_pe_disc_e_acelasi),
    ('model.sdf bine format', test_model_sdf_bine_format),
    ('roughness e parametru', test_roughness_e_parametru),
    ('NED -> ENU in lumea generata', test_NED_la_ENU_in_lumea_generata),
    ('directia soarelui', test_directia_soarelui),
    ('lumea e DERIVATA, nu rescrisa', test_lumea_e_DERIVATA_nu_rescrisa),
    ('o singura lumina, parametrizabila',
     test_o_singura_lumina_si_e_parametrizabila),
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
