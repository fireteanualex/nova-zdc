#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Generatorul modelului Gazebo pentru markerul de aterizare (I1).

    python3 tools/make_marker_model.py
    python3 tools/make_marker_model.py --north 2.0 --east 1.5 --roughness 0.3
    python3 tools/make_marker_model.py --sun-az 135 --sun-el 25

Produce:

    sim/models/aruco_26/model.config
    sim/models/aruco_26/model.sdf
    sim/models/aruco_26/materials/textures/aruco_26.png
    sim/worlds/nova_marker.sdf

GEOMETRIA TEXTURII

Coala e de 600 mm, zona codata de 480 mm, centrata - deci codul ocupa 80% din
latura, iar in jur raman 60 mm de zona linistita pe fiecare parte. Markerul se
genereaza la 2400 px, deci **5 px/mm**, iar coala iese la 3000 px. Raportul e
exact, nu rotunjit: DICT_4X4_50 are 4+2 = 6 module, iar 2400/6 = 400 px pe
modul, fara rest.

`marker_size_m` din config/nova.json (0.48) e latura zonei CODATE, inclusiv
bordura neagra - aceeasi cu ce masoara `solvePnP`. Coala de 600 mm nu intra
nicaieri in calcule; exista doar ca sa existe zona linistita.

CONVENTIA DE AXE - CAPCANA PRINCIPALA A ACESTUI FISIER

Detectorul si mașina de stări lucreaza in **NED** (`--north`, `--east`).
Gazebo lucreaza in **ENU**. Deci:

    pose_gazebo_x = east
    pose_gazebo_y = north
    pose_gazebo_z = sus

Un marker pus gresit aici - cu N si E inversate - produce exact simptomul unui
bug de conventie in detector: vehiculul coboara langa marker, nu pe el, iar
eroarea e transpusa pe axe. Ore pierdute cautand in `solvePnP` ceva ce e de
fapt in fisierul de lume. Vezi §5.31 din CLAUDE.md.

UNGHIUL SOARELUI

`--sun-az` e azimutul busolei (0 = nord, crescator spre est), `--sun-el`
elevatia deasupra orizontului. Vectorul `<direction>` din SDF e directia in
care CALATORESTE lumina, adica opusul directiei catre soare:

    est   = -cos(el) * sin(az)
    nord  = -cos(el) * cos(az)
    sus   = -sin(el)

15.4.7 semnaleaza explicit reflexia, iar `--roughness 0.3` (fata de 0.9,
hartie mata) e modul de a o vedea.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Geometria colii. Vezi docstring: 5 px/mm, exact.
SHEET_MM = 600.0
CODED_MM = 480.0
MARKER_PX = 2400

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 26

#: Hartie mata. 0.3 face markerul lucios si se vede stralucirea (15.4.7).
ROUGHNESS_MATE = 0.9

#: Peste sol, ca sa nu existe z-fighting cu planul de la z=0.
MARKER_Z = 0.01

MODEL_NAME = 'aruco_26'
WORLD_NAME = 'nova_marker'

#: Lumea de la care pornim (ardupilot_gazebo).
BASE_WORLD_CANDIDATES = (
    os.path.expanduser('~/ardu_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf'),
    os.path.expanduser('~/ardupilot_gazebo/worlds/iris_runway.sdf'),
)


# --- textura ----------------------------------------------------------------

#: Versiunea minima de OpenCV pentru TOATA unealta. `generateImageMarker` si
#: `ArucoDetector` au aparut in 4.7.0; inainte, API-ul era altul (§5.24).
MIN_CV2 = (4, 7)


def _cv2_version():
    return tuple(int(x) for x in cv2.__version__.split('.')[:2])


def _generate_marker(dictionary, marker_id, side_px):
    """Imaginea markerului, cu mesaj util pe OpenCV vechi.

    Fara garda asta, `python3` de sistem (Ubuntu 22.04: cv2 4.5.4) pica cu
    `AttributeError: module 'cv2.aruco' has no attribute
    'generateImageMarker'` - un mesaj care nu spune nici ca e o problema de
    versiune, nici ca exista un venv in care merge. §3: doua medii Python
    separate, nu le amesteca."""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    raise RuntimeError(
        f"cv2 {cv2.__version__} nu are cv2.aruco.generateImageMarker "
        f"(introdus in {MIN_CV2[0]}.{MIN_CV2[1]}).\n"
        f"  Python-ul folosit: {sys.executable}\n"
        f"  Codul NOVA ruleaza in venv (§3). Incearca:\n"
        f"    ~/nova-venv/bin/python tools/make_marker_model.py ...\n"
        f"  Restul uneltelor au oricum nevoie de ArucoDetector, tot din 4.7.")


def make_texture(marker_px=MARKER_PX, sheet_mm=SHEET_MM, coded_mm=CODED_MM,
                 marker_id=MARKER_ID, dictionary=ARUCO_DICT):
    """Coala alba cu markerul centrat. Intoarce (imagine, metadate).

    Nu se interpoleaza nimic: markerul se genereaza direct la rezolutia
    finala si se copiaza, pixel cu pixel, in mijlocul colii. Orice
    redimensionare ar inmuia muchiile, iar muchiile sunt exact ce
    localizeaza `detectMarkers` (§5.20)."""
    px_per_mm = marker_px / coded_mm
    sheet_px = int(round(sheet_mm * px_per_mm))
    if abs(sheet_mm * px_per_mm - sheet_px) > 1e-9:
        raise ValueError(
            f"coala de {sheet_mm} mm la {px_per_mm} px/mm da {sheet_mm * px_per_mm} "
            f"px, care nu e intreg; alege alt marker_px")

    d = cv2.aruco.getPredefinedDictionary(dictionary)
    marker = _generate_marker(d, marker_id, marker_px)

    sheet = np.full((sheet_px, sheet_px), 255, np.uint8)
    off = (sheet_px - marker_px) // 2
    sheet[off:off + marker_px, off:off + marker_px] = marker

    meta = {
        'sheet_mm': sheet_mm, 'coded_mm': coded_mm,
        'sheet_px': sheet_px, 'marker_px': marker_px,
        'px_per_mm': px_per_mm,
        'quiet_mm': (sheet_mm - coded_mm) / 2.0,
        'module_px': marker_px / (d.markerSize + 2),
        'marker_id': marker_id,
    }
    return sheet, meta


# --- SDF --------------------------------------------------------------------

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>NOVA - ZDC 2026</name>
  </author>
  <description>
    Marker ArUco {marker_id} (DICT_4X4_50) pe coala de {sheet_mm:g} mm, cu
    zona codata de {coded_mm:g} mm centrata. Generat de
    tools/make_marker_model.py - nu edita de mana.
  </description>
</model>
"""

MODEL_SDF = """<?xml version="1.0" ?>
<!-- Generat de tools/make_marker_model.py. Nu edita de mana: regenereaza. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{sheet_m:g} {sheet_m:g}</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/{tex}</albedo_map>
              <!-- 0.9 = hartie mata; 0.3 face markerul lucios si scoate la
                   iveala stralucirea semnalata de 15.4.7 -->
              <roughness>{roughness:g}</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>
"""

MARKER_INCLUDE = """
    <!-- Markerul de aterizare.
         ATENTIE LA AXE: detectorul lucreaza in NED, Gazebo in ENU.
             x = east = {east:g}
             y = north = {north:g}
         Inversate, vehiculul coboara langa marker si eroarea arata exact ca
         un bug de conventie in detector (§5.31). z = {z:g} evita
         z-fighting cu solul.

         URI: masurat, `<include><uri>` accepta `model://nume` (si atunci
         cere GZ_SIM_RESOURCE_PATH) sau o CALE ABSOLUTA. O cale relativa la
         fisierul lumii NU se rezolva. Implicit scriem calea absoluta, ca
         `gz sim <lume>` sa mearga fara nicio variabila de mediu; fisierul e
         oricum generat, deci calea locala nu e o problema - dar pe alta
         masina TREBUIE regenerat. Vezi optiunea uri-mode a generatorului.

         (Fara liniuta dubla in comentariile XML: standardul o interzice.
         libsdformat o accepta tacut, expat/ElementTree nu - deci un fisier
         pe care Gazebo il incarca fara reclamatii poate fi totusi XML
         invalid.) -->
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{east:g} {north:g} {z:g} 0 0 {yaw:g}</pose>
    </include>
"""

LIGHT_BLOCK = """    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <!-- azimut {az:g} deg (0 = nord, spre est), elevatie {el:g} deg.
           Directia e cea in care CALATORESTE lumina, deci opusul directiei
           catre soare. Vezi tools/make_marker_model.py. -->
      <direction>{dx:.6f} {dy:.6f} {dz:.6f}</direction>
    </light>
"""


def sun_direction(az_deg, el_deg):
    """Vectorul `<direction>` in ENU pentru un soare la (azimut, elevatie)."""
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    return (-math.cos(el) * math.sin(az),      # est
            -math.cos(el) * math.cos(az),      # nord
            -math.sin(el))                     # sus


def find_base_world(explicit=None):
    for cale in ([explicit] if explicit else []) + list(BASE_WORLD_CANDIDATES):
        if cale and os.path.exists(cale):
            return cale
    raise FileNotFoundError(
        "nu gasesc iris_runway.sdf. Da-l explicit cu --base-world.\n"
        "  Cautat in: " + ', '.join(BASE_WORLD_CANDIDATES))


def model_uri(uri_mode, models_dir, name=MODEL_NAME):
    """URI-ul de pus in `<include>`.

    `absolute` merge fara GZ_SIM_RESOURCE_PATH, deci `gz sim <lume>` tastat
    direct functioneaza - cazul in care se pierde cel mai mult timp, pentru
    ca eroarea (`Unable to find uri`) apare in mijlocul unei sesiuni si nu
    spune ce variabila lipseste."""
    if uri_mode == 'model':
        return f"model://{name}"
    if uri_mode == 'absolute':
        return os.path.abspath(os.path.join(models_dir, name))
    raise ValueError(f"uri-mode necunoscut: {uri_mode}")


def build_world(base_text, north, east, az, el, z=MARKER_Z, yaw=0.0,
                name=MODEL_NAME, world_name=WORLD_NAME, uri=None):
    """Lumea derivata: acelasi continut, cu lumina inlocuita si markerul
    adaugat inainte de `</world>`.

    Derivata, nu rescrisa: plugin-urile, coordonatele sferice si modelul de
    vehicul trebuie sa ramana EXACT ce foloseste ardupilot_gazebo. O lume
    scrisa de la zero ar diverge tacut la prima actualizare a lor."""
    text = base_text
    marca = '<world name="iris_runway">'
    if marca not in text:
        raise ValueError("lumea de baza nu are <world name=\"iris_runway\">")
    text = text.replace(marca, f'<world name="{world_name}">', 1)

    # lumina: inlocuim blocul intreg, ca directia sa fie parametrizabila
    i = text.find('<light type="directional" name="sun">')
    j = text.find('</light>', i)
    if i < 0 or j < 0:
        raise ValueError("nu gasesc blocul <light ... name=\"sun\">")
    dx, dy, dz = sun_direction(az, el)
    # pastram indentarea liniei pe care incepe blocul
    start = text.rfind('\n', 0, i) + 1
    text = (text[:start]
            + LIGHT_BLOCK.format(az=az, el=el, dx=dx, dy=dy, dz=dz)
            + text[j + len('</light>'):].lstrip('\n'))

    inc = MARKER_INCLUDE.format(name=name, north=north, east=east, z=z,
                                yaw=yaw,
                                uri=uri or f"model://{name}")
    text = text.replace('  </world>', inc + '\n  </world>', 1)

    antet = ("<!-- Generat de tools/make_marker_model.py din\n"
             "     {sursa}\n"
             "     Nu edita de mana: regenereaza. -->\n")
    text = text.replace('<sdf version="1.9">', antet + '<sdf version="1.9">', 1)
    return text


# --- scriere ----------------------------------------------------------------

def write_model(out_dir, roughness=ROUGHNESS_MATE, **kw):
    img, meta = make_texture(**kw)
    tex_dir = os.path.join(out_dir, 'materials', 'textures')
    os.makedirs(tex_dir, exist_ok=True)
    tex_name = f"{MODEL_NAME}.png"
    tex_path = os.path.join(tex_dir, tex_name)
    if not cv2.imwrite(tex_path, img):
        raise IOError(f"nu pot scrie {tex_path}")

    with open(os.path.join(out_dir, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG.format(name=MODEL_NAME, marker_id=meta['marker_id'],
                                    sheet_mm=meta['sheet_mm'],
                                    coded_mm=meta['coded_mm']))
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF.format(name=MODEL_NAME,
                                 sheet_m=meta['sheet_mm'] / 1000.0,
                                 tex=tex_name, roughness=roughness))
    return meta, tex_path


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--north', type=float, default=2.0,
                   help='pozitia markerului in NED; ajunge pe y in Gazebo')
    p.add_argument('--east', type=float, default=1.5,
                   help='pozitia markerului in NED; ajunge pe x in Gazebo')
    p.add_argument('--yaw', type=float, default=0.0, help='radiani')
    p.add_argument('--z', type=float, default=MARKER_Z)
    p.add_argument('--roughness', type=float, default=ROUGHNESS_MATE,
                   help=f'{ROUGHNESS_MATE} = hartie mata; 0.3 = lucios (15.4.7)')
    p.add_argument('--sun-az', type=float, default=135.0,
                   help='azimut busola, grade (0 = nord)')
    p.add_argument('--sun-el', type=float, default=45.0,
                   help='elevatie deasupra orizontului, grade')
    p.add_argument('--marker-px', type=int, default=MARKER_PX)
    p.add_argument('--out', default=os.path.join(REPO, 'sim'))
    p.add_argument('--base-world', default=None)
    p.add_argument('--uri-mode', choices=('absolute', 'model'),
                   default='absolute',
                   help='absolute: lumea merge cu `gz sim` simplu, dar e '
                        'legata de masina asta (regenereaza dupa clonare). '
                        'model: portabil, dar cere GZ_SIM_RESOURCE_PATH.')
    a = p.parse_args(argv)

    model_dir = os.path.join(a.out, 'models', MODEL_NAME)
    os.makedirs(model_dir, exist_ok=True)
    meta, tex = write_model(model_dir, roughness=a.roughness,
                            marker_px=a.marker_px)

    base = find_base_world(a.base_world)
    with open(base) as f:
        base_text = f.read()
    uri = model_uri(a.uri_mode, os.path.join(a.out, 'models'))
    world = build_world(base_text, a.north, a.east, a.sun_az, a.sun_el,
                        z=a.z, yaw=a.yaw, uri=uri)
    world = world.replace('{sursa}', base)
    worlds_dir = os.path.join(a.out, 'worlds')
    os.makedirs(worlds_dir, exist_ok=True)
    world_path = os.path.join(worlds_dir, f"{WORLD_NAME}.sdf")
    with open(world_path, 'w') as f:
        f.write(world)

    dx, dy, dz = sun_direction(a.sun_az, a.sun_el)
    print(f"\n  model : {model_dir}")
    print(f"  textura: {os.path.relpath(tex, REPO)}  "
          f"{meta['sheet_px']}x{meta['sheet_px']} px")
    print(f"           coala {meta['sheet_mm']:g} mm, zona codata "
          f"{meta['coded_mm']:g} mm ({meta['coded_mm']/meta['sheet_mm']:.0%}), "
          f"zona linistita {meta['quiet_mm']:g} mm")
    print(f"           {meta['px_per_mm']:g} px/mm, {meta['module_px']:g} px pe modul")
    print(f"  roughness: {a.roughness:g}"
          + ("  (mata)" if a.roughness >= 0.8 else "  (LUCIOS - 15.4.7)"))
    print(f"\n  lume  : {world_path}")
    print(f"           derivata din {base}")
    print(f"           marker NED n={a.north:g} e={a.east:g}  ->  "
          f"Gazebo x={a.east:g} y={a.north:g} z={a.z:g}")
    print(f"           soare az {a.sun_az:g} deg el {a.sun_el:g} deg  ->  "
          f"direction {dx:.3f} {dy:.3f} {dz:.3f}")
    print(f"           uri marker: {uri}")
    if a.uri_mode == 'absolute':
        print(f"\n  Ruleaza direct, fara variabile de mediu:\n"
              f"    gz sim -v4 -r {world_path}\n"
              f"  (calea e absoluta: pe alta masina REGENEREAZA)\n")
    else:
        print(f"\n  Ca Gazebo sa gaseasca modelul:\n"
              f"    export GZ_SIM_RESOURCE_PATH=\"{os.path.join(a.out, 'models')}"
              f":$GZ_SIM_RESOURCE_PATH\"\n"
              f"    gz sim -v4 -r {world_path}\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
