#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Factorul de timp real al unei lumi Gazebo (I2).

    python3 tools/measure_rtf.py sim/worlds/nova_marker.sdf
    python3 tools/measure_rtf.py --compare sim/worlds/a.sdf sim/worlds/b.sdf

DE CE O UNEALTA SI NU O COMANDA

Masuratoarea naiva - `time gz sim --iterations N` - include pornirea
procesului, care e de ordinul secundelor si domina la N mic. Prima mea
masuratoare a dat RTF 0.33 pentru ceva ce rula de fapt la 0.53, si diferenta
era doar pornirea.

Unealta ruleaza DOUA durate si elimina pornirea algebric:

    t = pornire + N * dt / RTF

Doua ecuatii, doua necunoscute. Raporteaza si pornirea, ca sa se vada daca
masuratoarea are sens.

VERIFICA INTAI CA CAMERA CHIAR RANDEAZA

Un senzor de camera care nu randeaza nu costa nimic, deci RTF-ul arata
excelent si nu inseamna nimic. Masurat: intr-o sesiune headless fara EGL
functional, topicurile `/down_cam/image` si `/down_cam/camera_info` sunt
ANUNTATE, dar nu soseste niciun mesaj - iar timpul cu si fara camera iese
identic. Foloseste `--check-topic` inainte de a crede orice cifra.
"""

import argparse
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: max_step_size din lumile noastre. Se citeste din SDF daca se poate.
DEFAULT_STEP_S = 0.001


def step_size(world, implicit=DEFAULT_STEP_S):
    try:
        import xml.etree.ElementTree as ET
        w = ET.parse(world).getroot().find('world')
        v = w.find('physics/max_step_size')
        return float(v.text) if v is not None else implicit
    except Exception:                                        # noqa: BLE001
        return implicit


def run_iterations(world, n, timeout=1800):
    """Secunde de perete pentru n pasi. None daca gz a esuat."""
    t0 = time.monotonic()
    try:
        r = subprocess.run(['gz', 'sim', '-s', '-r', '--iterations', str(n),
                            world],
                           capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as e:
        print(f"    gz a esuat: {e}")
        return None
    dt = time.monotonic() - t0
    if r.returncode != 0:
        print(f"    gz a iesit cu {r.returncode}")
        return None
    return dt


def measure(world, n_scurt=3000, n_lung=30000, printer=print):
    """(rtf, pornire_s, detalii) cu pornirea eliminata algebric."""
    dt = step_size(world)
    printer(f"    pas {dt * 1000:g} ms; rulez {n_scurt} si {n_lung} iteratii")
    t1 = run_iterations(world, n_scurt)
    if t1 is None:
        return None, None, {}
    t2 = run_iterations(world, n_lung)
    if t2 is None:
        return None, None, {}
    if t2 <= t1:
        printer("    a doua rulare nu a durat mai mult; masuratoare "
                "nefolositoare")
        return None, None, {'t1': t1, 't2': t2}
    rtf = (n_lung - n_scurt) * dt / (t2 - t1)
    pornire = t1 - n_scurt * dt / rtf
    return rtf, pornire, {'t1': t1, 't2': t2, 'dt': dt,
                          'n1': n_scurt, 'n2': n_lung}


def check_topic(topic, world=None, wait=25.0, printer=print):
    """True daca soseste macar un mesaj pe `topic`.

    Porneste serverul daca nu ruleaza deja. Un topic ANUNTAT dar fara mesaje
    inseamna senzor creat si randare care nu se intampla."""
    pornit = None
    if subprocess.run(['pgrep', '-f', 'gz sim'],
                      capture_output=True).returncode != 0:
        if world is None:
            printer("    niciun gz sim si nicio lume data")
            return False
        printer(f"    pornesc serverul pentru {os.path.basename(world)}")
        pornit = subprocess.Popen(['gz', 'sim', '-s', '-r', world],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL,
                                  start_new_session=True)
        time.sleep(18.0)
    try:
        r = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', '1'],
                           capture_output=True, text=True, timeout=wait)
        ok = r.returncode == 0 and bool(r.stdout.strip())
    except subprocess.TimeoutExpired:
        ok = False
    finally:
        if pornit is not None:
            pornit.terminate()
            try:
                pornit.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pornit.kill()
    return ok


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('worlds', nargs='+')
    p.add_argument('--short', type=int, default=3000)
    p.add_argument('--long', type=int, default=30000)
    p.add_argument('--check-topic', default=None,
                   help='verifica intai ca soseste macar un mesaj, '
                        'ex. /down_cam/image')
    a = p.parse_args(argv)

    if a.check_topic:
        print(f"\n  verific {a.check_topic} ...")
        ok = check_topic(a.check_topic, a.worlds[0])
        print(f"    {'SOSESC mesaje' if ok else 'NICIUN mesaj'}")
        if not ok:
            print("\n  Un senzor care nu randeaza nu costa nimic: orice RTF\n"
                  "  masurat mai jos NU spune nimic despre costul camerei.\n"
                  "  Verifica randarea (EGL/GPU) inainte.\n")

    rezultate = []
    for w in a.worlds:
        if not os.path.exists(w):
            print(f"\n  {w}: nu exista")
            continue
        print(f"\n  {os.path.relpath(w, REPO)}")
        rtf, pornire, d = measure(w, a.short, a.long)
        if rtf is None:
            continue
        rezultate.append((w, rtf))
        print(f"    pornire {pornire:.2f} s (exclusa din RTF)")
        print(f"    RTF in regim stabil: {rtf:.2f}")
        if rtf < 0.5:
            print("    SUB 0.5: ia in calcul --scale in "
                  "tools/make_camera_model.py.\n"
                  "    ATENTIE: marker_px se injumatateste, deci pragurile "
                  "in pixeli\n"
                  "    corespund altor altitudini - unealta le retipareste.")

    if len(rezultate) == 2:
        (w1, r1), (w2, r2) = rezultate
        print(f"\n  {os.path.basename(w1)} {r1:.2f}  vs  "
              f"{os.path.basename(w2)} {r2:.2f}   "
              f"(raport {r2 / r1:.2f})")
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
