#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Senzorul de camera orientat in jos, pe modelul de vehicul (I2).

    python3 tools/make_camera_model.py                     # cere calibrarea
    python3 tools/make_camera_model.py --calib /cale/camera_pi.yaml
    python3 tools/make_camera_model.py --scale 0.5         # rezolutie redusa

Produce `sim/models/iris_nova/` - vehiculul ardupilot_gazebo cu un senzor de
camera in plus - si leaga lumea de el.

INTRINSECII VIN DIN CALIBRARE, NU DIN FISA TEHNICA

Unealta CITESTE `config/camera_pi.yaml` si genereaza SDF-ul. Nu exista valori
scrise de mana: daca cineva recalibreaza, simularea urmeaza la urmatoarea
rulare. Fara o calibrare REALA (E1.2), unealta refuza - la fel ca detectorul
de bord, si din acelasi motiv: o focala geometrica pusa ca sa treaca ceva
face simularea sa masoare altceva decat vehiculul.

`horizontal_fov` se calculeaza din fx, nu se ia din fisa:

    hfov = 2 * atan(width / (2 * fx))

Gazebo accepta si `<intrinsics>`, si `<horizontal_fov>`. Daca nu sunt
consistente, rezultatul depinde de ordinea in care le citeste - deci le
scriem derivate una din alta, nu independent.

REZOLUTIE REDUSA: CE SE SCALEAZA SI CE NU

`--scale 0.5` injumatateste latimea, inaltimea, fx, fy, cx, cy. Coeficientii
de distorsiune **NU** se scaleaza: k1..k3 sunt definiti pe coordonate
normalizate, deci sunt adimensionali.

**Pragurile de detectie NU se transfera.** `marker_px = fx * 0.48 / Z`, deci
la jumatate de rezolutie markerul are jumatate de pixeli la aceeasi
altitudine. Pragul de scoring (980 px) si limita de incadrare din §5.2 sunt
exprimate in pixeli: la scale 0.5 ele corespund altor altitudini. Unealta
tipareste noul tabel, ca sa nu fie nevoie de calcul mental in mijlocul unei
sesiuni.

MONTAJ

`--mount-z` e distanta sub `base_link`, implicit 0.0745 (74.5 mm, din CAD).
ATENTIE: pe vehiculul real asta e inaltimea DEASUPRA SOLULUI la contact. In
simulare, `base_link` al lui iris nu e la sol cand vehiculul e asezat, deci
inaltimea rezultata deasupra solului e ALTA. Unealta o raporteaza; vezi
§5.33.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nova import config as nova_config                      # noqa: E402
from nova.detector_pi import CameraCalibration              # noqa: E402


REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MODEL_NAME = 'iris_nova'
SENSOR_NAME = 'down_cam'
CAM_LINK = 'down_cam_link'

#: Modelul de la care pornim (ardupilot_gazebo). Il DERIVAM, nu il rescriem:
#: contine plugin-ul ArduPilot, lift-drag pe fiecare rotor si referinte de
#: link pe care nu vrem sa le reproducem de mana.
BASE_MODEL_CANDIDATES = (
    os.path.expanduser('~/ardupilot_gazebo/models/iris_with_gimbal/model.sdf'),
    os.path.expanduser(
        '~/ardu_ws/src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf'),
)

#: Din CAD: camera la 74.5 mm deasupra solului la contact (§2).
MOUNT_Z_M = 0.0745

#: §5.15: modul binned al IMX708 da pana la ~56 fps; rulam la 30.
UPDATE_RATE_HZ = 30

#: Pragul de scoring din §8, in pixeli. Se recalculeaza la alta rezolutie.
SCORING_PX = 980
#: §5.2: markerul trebuie sa incapa intreg in cadru.
FRAME_FILL = 0.95

#: L8 = o singura componenta, exact ce consuma detectorul. Un format color ar
#: insemna randare si transport de trei ori mai scumpe, degeaba.
IMAGE_FORMAT = 'L8'

SENSOR_BLOCK = """      <sensor name="{sensor}" type="camera">
        <!-- Intrinseci CITITI din {calib_src}.
             Nu edita aici: regenereaza cu tools/make_camera_model.py. -->
        <camera>
          <!-- derivat din fx: 2*atan(width/(2*fx)) -->
          <horizontal_fov>{hfov:.6f}</horizontal_fov>
          <image>
            <width>{w}</width>
            <height>{h}</height>
            <format>{fmt}</format>
          </image>
          <lens>
            <intrinsics>
              <fx>{fx:.4f}</fx>
              <fy>{fy:.4f}</fy>
              <cx>{cx:.4f}</cx>
              <cy>{cy:.4f}</cy>
              <s>0</s>
            </intrinsics>
          </lens>
          <distortion>
            <k1>{k1:.6f}</k1>
            <k2>{k2:.6f}</k2>
            <k3>{k3:.6f}</k3>
            <p1>{p1:.6f}</p1>
            <p2>{p2:.6f}</p2>
            <center>0.5 0.5</center>
          </distortion>
          <clip>
            <near>0.05</near>
            <far>60</far>
          </clip>
        </camera>
        <update_rate>{rate:g}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <topic>{topic}</topic>
      </sensor>
"""

CAM_LINK_BLOCK = """
    <!-- Camera orientata in jos.
         Pozitie: {mz:g} m sub base_link. Pe vehiculul real asta e inaltimea
         DEASUPRA SOLULUI la contact (74.5 mm, §2); in simulare base_link nu e
         la sol, deci inaltimea efectiva difera - vezi §5.33.
         Orientare: pitch +90 deg duce axa optica (+X a camerei) pe -Z, adica
         drept in jos. -->
    <link name="{link}">
      <pose>0 0 {mz_neg:g} 0 {pitch:.6f} 0</pose>
      <!-- Masa simbolica: un senzor fara inertie face solverul sa se planga,
           dar nu vrem sa schimbam masa vehiculului. -->
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
{sensor}    </link>

    <joint name="{link}_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>{link}</child>
    </joint>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>NOVA - ZDC 2026</name></author>
  <description>
    iris_with_gimbal (ardupilot_gazebo) cu un senzor de camera orientat in
    jos, cu intrinsecii din calibrarea reala. Generat de
    tools/make_camera_model.py - nu edita de mana.
  </description>
</model>
"""


# --- intrinseci -------------------------------------------------------------

def load_intrinsics(path, scale=1.0, provisional=False, max_rms=None):
    """Intrinsecii din calibrare, optional scalati.

    Implicit refuza o calibrare care nu e reala (E1.2), la fel ca detectorul
    de bord. `provisional=True` ocoleste verificarea - EXPLICIT, ca la E0 in
    `fake_detector.py`: garda exista ca sa protejeze un vehicul real, iar
    aici vehiculul e Gazebo. Ocolirea se cere din linia de comanda si se
    anunta, nu se strecoara prin editarea unui fisier de config."""
    if provisional:
        cal = CameraCalibration.load(path, require_real=False)
    else:
        kw = {} if max_rms is None else {'max_rms': max_rms}
        cal = CameraCalibration.load(path, require_real=True, **kw)
    dist = list(cal.dist.reshape(-1)) + [0.0] * 5
    w = int(round(cal.width * scale))
    h = int(round(cal.height * scale))
    out = {
        'w': w, 'h': h,
        'fx': cal.fx * scale, 'fy': cal.fy * scale,
        'cx': cal.cx * scale, 'cy': cal.cy * scale,
        # k1..k3, p1, p2 sunt definiti pe coordonate NORMALIZATE, deci nu se
        # scaleaza odata cu rezolutia.
        'k1': dist[0], 'k2': dist[1], 'p1': dist[2], 'p2': dist[3],
        'k3': dist[4],
        'scale': scale,
        'sursa': cal.source, 'rms': cal.rms, 'n_images': cal.n_images,
    }
    out['hfov'] = 2.0 * math.atan(w / (2.0 * out['fx']))
    out['vfov'] = 2.0 * math.atan(h / (2.0 * out['fy']))
    return out, cal


def altitude_for_marker_px(fx, px, marker_m=0.48):
    """Z la care markerul are `px` pixeli: marker_px = fx * latura / Z."""
    return fx * marker_m / px


def threshold_table(intr, marker_m=0.48):
    """Pragurile exprimate in pixeli, traduse in altitudini pentru ACEST
    senzor. La rezolutie redusa se muta, si asta e capcana principala."""
    fx = intr['fx']
    frame_limit_px = FRAME_FILL * intr['h']
    return {
        'scoring_px': SCORING_PX,
        'scoring_alt_m': altitude_for_marker_px(fx, SCORING_PX, marker_m),
        'frame_limit_px': frame_limit_px,
        'frame_limit_alt_m': altitude_for_marker_px(fx, frame_limit_px,
                                                    marker_m),
        'px_at': {z: fx * marker_m / z for z in (12.0, 5.0, 1.0)},
    }


# --- model ------------------------------------------------------------------

def find_base_model(explicit=None):
    for cale in ([explicit] if explicit else []) + list(BASE_MODEL_CANDIDATES):
        if cale and os.path.exists(cale):
            return cale
    raise FileNotFoundError(
        "nu gasesc iris_with_gimbal/model.sdf. Da-l cu --base-model.\n"
        "  Cautat in: " + ', '.join(BASE_MODEL_CANDIDATES))


def scoate_gimbalul(text):
    """Scoate `gimbal_small_3d` din modelul derivat. (text, mesaj).

    DE CE. `iris_with_gimbal` atarna un gimbal la **-0.125 m** sub
    `base_link`, iar camera noastra sta la -0.0745 m - deci gimbalul e
    exact in campul ei, si ocluzioneaza solul.

    Masurat: la 0.93 m, cu centrarea perfecta (0.8 cm lateral, 0.1 grade
    inclinare) si 39 cm de marja pana la marginea cadrului, detectia s-a
    pierdut oricum. Cadrul salvat arata de ce: corpul gimbalului taie
    **zona linistita** a markerului, iar bordura neagra fuzioneaza cu
    fundalul intunecat. Exact mecanismul din §5.18, produs fizic.

    Zona linistita e ingusta prin constructie: (600-480)/2 = 60 mm, adica
    0.75 dintr-un modul ArUco (480/6 = 80 mm). Orice o atinge rupe conturul.

    NOVA nu are gimbal. Il pastram doar cu `--keep-gimbal`, pentru
    comparatie cu modelul stock."""
    import xml.etree.ElementTree as ET
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    parser.feed(text)
    radacina = parser.close()

    scoase = []

    def curata(parinte):
        for copil in list(parinte):
            brut = ET.tostring(copil, encoding='unicode')
            eticheta = copil.tag
            nume = copil.get('name', '')
            sters = False
            if eticheta == 'include' and 'gimbal' in brut:
                sters = True
            elif eticheta == 'joint' and nume == 'gimbal_joint':
                sters = True
            elif eticheta == 'control' and 'gimbal::' in brut:
                # Un bloc <control> e mic si se poate judeca pe tot subarborele.
                sters = True
            elif eticheta == 'plugin':
                # NU pe subarbore: ArduPilotPlugin CONTINE canalele de gimbal,
                # iar o regula pe subarbore l-ar sterge cu totul - adica
                # vehiculul ar ramane fara motoare. Prima varianta a facut
                # exact asta, si a iesit la iveala doar pentru ca modelul
                # generat a fost verificat, nu presupus (§5.10).
                sters = any(copil2.tag == 'joint_name'
                            and 'gimbal::' in (copil2.text or '')
                            for copil2 in copil)
            if sters:
                scoase.append(f"{eticheta}{'/' + nume if nume else ''}")
                parinte.remove(copil)
            else:
                curata(copil)

    curata(radacina)
    if not scoase:
        return text, 'gimbal: nu era in model'
    iesire = ET.tostring(radacina, encoding='unicode')
    return iesire, f"gimbal scos: {len(scoase)} elemente ({', '.join(scoase)})"


def build_model(base_text, intr, calib_src, mount_z=MOUNT_Z_M,
                rate=UPDATE_RATE_HZ, name=MODEL_NAME, topic=None,
                keep_gimbal=False, on_note=None):
    """Modelul derivat: acelasi continut, cu un link de camera in plus.

    Derivat, nu rescris (§5.31): plugin-ul ArduPilot, lift-drag pe fiecare
    rotor si referintele de link trebuie sa ramana EXACT ce foloseste
    ardupilot_gazebo."""
    marca = '<model name="iris_with_gimbal">'
    if marca not in base_text:
        raise ValueError("modelul de baza nu are <model name=\"iris_with_gimbal\">")
    text = base_text.replace(marca, f'<model name="{name}">', 1)

    sensor = SENSOR_BLOCK.format(
        sensor=SENSOR_NAME, calib_src=calib_src, rate=rate,
        topic=topic or f"/{SENSOR_NAME}/image", fmt=IMAGE_FORMAT, **intr)
    bloc = CAM_LINK_BLOCK.format(link=CAM_LINK, mz=mount_z, mz_neg=-mount_z,
                                 pitch=math.pi / 2.0, sensor=sensor)

    # inaintea comentariului de plugin-uri, ca sa ramana grupate
    ancora = '    <!-- plugins -->'
    if ancora in text:
        text = text.replace(ancora, bloc + '\n' + ancora, 1)
    else:
        text = text.replace('  </model>', bloc + '\n  </model>', 1)

    if not keep_gimbal:
        text, mesaj = scoate_gimbalul(text)
        if on_note:
            on_note(f"  {mesaj}")

    antet = ("<!-- Generat de tools/make_camera_model.py din\n"
             "     {sursa}\n"
             "     Nu edita de mana: regenereaza. -->\n")
    text = text.replace('<sdf version="1.9">', antet + '<sdf version="1.9">', 1)
    return text


def patch_world(world_path, model_dir, name=MODEL_NAME, dry_run=False):
    """Leaga lumea de vehiculul nostru, in loc de cel stock.

    Idempotent: rulata de doua ori, a doua oara nu schimba nimic."""
    if not os.path.exists(world_path):
        return None, (f"lumea {world_path} nu exista; ruleaza intai "
                      f"tools/make_marker_model.py")
    text = open(world_path).read()
    uri_nou = os.path.abspath(os.path.join(model_dir, name))
    if f"<uri>{uri_nou}</uri>" in text:
        return text, 'lumea era deja legata de vehiculul nostru'
    if '<uri>model://iris_with_gimbal</uri>' not in text:
        return text, ('lumea nu contine model://iris_with_gimbal; nu o ating')
    nou = text.replace(
        '<uri>model://iris_with_gimbal</uri>',
        f"<uri>{uri_nou}</uri>\n      <name>iris_with_gimbal</name>", 1)
    if not dry_run:
        open(world_path, 'w').write(nou)
    return nou, f"vehicul inlocuit cu {os.path.relpath(uri_nou, REPO)}"


def write_model(out_dir, base_text, intr, calib_src, sursa='', **kw):
    os.makedirs(out_dir, exist_ok=True)
    text = build_model(base_text, intr, calib_src, **kw)
    text = text.replace('{sursa}', sursa or '(necunoscut)')
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(text)
    with open(os.path.join(out_dir, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG.format(name=kw.get('name', MODEL_NAME)))
    return text


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--calib', default=None,
                   help='implicit: din config/nova.json')
    p.add_argument('--scale', type=float, default=1.0,
                   help='rezolutie redusa; intrinsecii se scaleaza, '
                        'distorsiunea nu, pragurile se muta')
    p.add_argument('--mount-z', type=float, default=MOUNT_Z_M,
                   help='metri sub base_link (CAD: 0.0745)')
    p.add_argument('--rate', type=float, default=UPDATE_RATE_HZ)
    p.add_argument('--out', default=os.path.join(REPO, 'sim'))
    p.add_argument('--base-model', default=None)
    p.add_argument('--world', default=None,
                   help='implicit sim/worlds/nova_marker.sdf')
    p.add_argument('--no-patch-world', action='store_true')
    p.add_argument('--provisional', action='store_true',
                   help='accepta o calibrare care NU e reala (config/'
                        'camera_sim.yaml). Doar pentru simulare: garda E1.2 '
                        'protejeaza un vehicul real, iar aici e Gazebo.')
    p.add_argument('--max-rms', type=float, default=None,
                   help='prag de reproiectie; implicit cel din '
                        'nova/detector_pi.py (MAX_REPROJ_ERR_PX)')
    p.add_argument('--keep-gimbal', action='store_true',
                   help='pastreaza gimbalul din modelul stock. Implicit e '
                        'scos: atarna la -0.125 m si ocluzioneaza camera '
                        'orientata in jos (§5.46)')
    a = p.parse_args(argv)

    cfg = nova_config.load()
    calib = a.calib or nova_config.resolve(cfg, 'camera_calibration')
    try:
        intr, cal = load_intrinsics(calib, a.scale, provisional=a.provisional,
                                    max_rms=a.max_rms)
    except (FileNotFoundError, ValueError) as e:
        print(f"\n  NU GENEREZ: {e}\n"
              f"  Intrinsecii senzorului trebuie sa vina din calibrarea "
              f"REALA (E1.2).\n"
              f"  Fara ea, simularea ar masura alta camera decat vehiculul.\n"
              f"  Pentru simulare, cu o calibrare provizorie:\n"
              f"    python3 tools/make_camera_model.py "
              f"--calib config/camera_sim.yaml --provisional\n")
        return 2

    if a.provisional:
        print(f"\n  ATENTIE: calibrare PROVIZORIE, acceptata explicit.\n"
              f"    sursa: {cal.source}\n"
              f"    rms {cal.rms} px, n_images {cal.n_images}\n"
              f"  Simularea va folosi intrinsecii astia; ZBORUL nu ii vede - "
              f"detectorul\n"
              f"  de bord citeste config/camera_pi.yaml si refuza orice nu e "
              f"real (E1.2).")

    base = find_base_model(a.base_model)
    model_dir = os.path.join(a.out, 'models', MODEL_NAME)
    # Sursa reala se pune in antet INAINTE de scriere. Prima varianta scria
    # fisierul si apoi facea `open(f,'w').write(open(f).read()...)` - dar
    # modul 'w' trunchiaza fisierul inainte ca argumentul sa fie evaluat,
    # deci se citea un fisier deja gol si se scria nimic. Modelul iesea de
    # zero octeti, iar Gazebo se plangea abia la incarcare.
    write_model(model_dir, open(base).read().replace('{sursa}', base), intr,
                os.path.relpath(calib, REPO), mount_z=a.mount_z, rate=a.rate,
                sursa=base, keep_gimbal=a.keep_gimbal,
                on_note=print)

    print(f"\n  calibrare: {calib}")
    print(f"             {cal}")
    print(f"  model    : {model_dir}")
    print(f"             derivat din {base}")
    print(f"\n  senzor {SENSOR_NAME}: {intr['w']}x{intr['h']} @ {a.rate:g} Hz, "
          f"format {IMAGE_FORMAT}")
    print(f"    fx={intr['fx']:.2f} fy={intr['fy']:.2f} "
          f"cx={intr['cx']:.2f} cy={intr['cy']:.2f}")
    print(f"    hfov={math.degrees(intr['hfov']):.2f} deg  "
          f"vfov={math.degrees(intr['vfov']):.2f} deg")
    print(f"    k1={intr['k1']:.4f} k2={intr['k2']:.4f} k3={intr['k3']:.4f} "
          f"(adimensionali: NU se scaleaza)")
    if a.scale != 1.0:
        print(f"    scale {a.scale:g}: latime/inaltime/fx/fy/cx/cy scalate")

    t = threshold_table(intr)
    print("\n  Praguri, pentru ACEST senzor:")
    print("    marker_px la 12 m / 5 m / 1 m: "
          + ' / '.join(f"{t['px_at'][z]:.0f}" for z in (12.0, 5.0, 1.0)))
    print(f"    SCORING_CAPTURE ({SCORING_PX} px) la {t['scoring_alt_m']:.2f} m")
    print(f"    marker nu mai incape ({t['frame_limit_px']:.0f} px) sub "
          f"{t['frame_limit_alt_m']:.2f} m")
    if a.scale != 1.0:
        print(f"    ATENTIE: la scale {a.scale:g} pragurile in PIXELI raman, "
              f"dar altitudinile de mai sus s-au mutat.")

    if not a.no_patch_world:
        world = a.world or os.path.join(a.out, 'worlds', 'nova_marker.sdf')
        _t, mesaj = patch_world(world, os.path.join(a.out, 'models'))
        print(f"\n  lume: {mesaj}")

    print(f"\n  Verifica in Gazebo:\n"
          f"    gz sim -v4 -r {os.path.join(a.out, 'worlds', 'nova_marker.sdf')}\n"
          f"    gz topic -l | grep {SENSOR_NAME}\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
