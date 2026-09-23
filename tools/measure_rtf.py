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
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: max_step_size din lumile noastre. Se citeste din SDF daca se poate.
DEFAULT_STEP_S = 0.001


def gz_processes(exclude_pids=()):
    """PID-urile serverelor gz care ruleaza ACUM, in afara celor date.

    Un `gz sim` concurent - mai ales unul cu GUI, care randeaza continuu -
    fura CPU si falsifica masuratoarea. Masurat: aceeasi lume a dat 0.50 cu
    un server GUI pornit alaturi si 0.96 fara. Un factor de doi, adica exact
    ordinul de marime al concluziilor pe care le-am trage din cifra."""
    try:
        out = subprocess.run(['pgrep', '-f', 'gz sim'],
                             capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return []
    pids = []
    for linie in (out.stdout or '').split():
        try:
            pid = int(linie)
        except ValueError:
            continue
        if pid in exclude_pids or pid == os.getpid():
            continue
        pids.append(pid)
    return pids


def refuse_if_busy(printer=print, exclude=()):
    """True daca se poate masura. Refuza daca ruleaza alt gz sim."""
    pids = gz_processes(exclude)
    if not pids:
        return True
    printer(f"\n  NU MASOR: ruleaza deja {len(pids)} proces(e) gz sim "
            f"({', '.join(str(p) for p in pids[:5])}).")
    printer("  Un server concurent - mai ales cu GUI, care randeaza - fura")
    printer("  CPU si falsifica rezultatul cu un factor de pana la doi.")
    printer("  Opreste-le si reia:")
    printer("    pkill -f 'gz sim'\n")
    return False


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
            # `gz sim` isi porneste propriile procese (server, gui). Un
            # terminate() pe lansator le lasa in viata, iar ele raman sa
            # randeze in timpul masuratorii care urmeaza. Omoram GRUPUL.
            _kill_group(pornit)
    return ok


def _kill_group(proc, timeout=10.0):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            proc.kill()
    # asteptam sa dispara si copiii
    for _ in range(20):
        if not gz_processes():
            return
        time.sleep(0.5)


# --- masuratoare "vie": citim RTF-ul raportat de Gazebo -----------------------

def world_name(world, implicit='default'):
    try:
        import xml.etree.ElementTree as ET
        w = ET.parse(world).getroot().find('world')
        return w.get('name') or implicit
    except Exception:                                        # noqa: BLE001
        return implicit


def parse_rtf(text):
    """RTF dintr-un mesaj WorldStatistics, sau None.

    Preferam campul `real_time_factor` daca exista; altfel il calculam din
    sim_time/real_time, care sunt mereu acolo."""
    import re
    m = re.search(r'real_time_factor:\s*([0-9.eE+-]+)', text)
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            pass

    def bloc(nume):
        b = re.search(nume + r'\s*\{([^}]*)\}', text, re.S)
        if not b:
            return None
        sec = re.search(r'sec:\s*(-?\d+)', b.group(1))
        nsec = re.search(r'nsec:\s*(-?\d+)', b.group(1))
        return ((int(sec.group(1)) if sec else 0)
                + (int(nsec.group(1)) if nsec else 0) * 1e-9)

    st, rt = bloc('sim_time'), bloc('real_time')
    if st is None or rt is None or rt <= 0:
        return None
    return st / rt


def measure_live(world, subscribe=(), warmup=10.0, samples=8,
                 printer=print):
    """RTF cu abonati atasati pe TOATA fereastra de masurare.

    De ce exista. Masurat: o camera 2304x1296 la 30 Hz si aceeasi camera la
    240 Hz dau EXACT acelasi RTF intr-un `gz sim -s` fara abonat - adica nu
    randeaza deloc, in ciuda lui `<always_on>1</always_on>`. Deci
    cronometrarea pe `--iterations`, care nu poate tine un abonat pe toata
    fereastra, nu poate masura costul camerei.

    Aici serverul ruleaza continuu, abonatii stau atasati, iar RTF-ul se
    citeste din ce raporteaza Gazebo insusi - deci pornirea procesului nu
    mai intra in socoteala."""
    nume = world_name(world)
    topicuri = [f"/world/{nume}/stats", '/stats']
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', world],
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL,
                              start_new_session=True)
    abonati = []
    try:
        time.sleep(warmup * 0.6)
        for t in subscribe:
            abonati.append(subprocess.Popen(
                ['gz', 'topic', '-e', '-t', t],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True))
            printer(f"    abonat la {t}")
        time.sleep(warmup * 0.4)

        valori = []
        for topic in topicuri:
            for _ in range(samples):
                try:
                    r = subprocess.run(['gz', 'topic', '-e', '-t', topic,
                                        '-n', '1'],
                                       capture_output=True, text=True,
                                       timeout=8)
                except subprocess.TimeoutExpired:
                    break
                v = parse_rtf(r.stdout or '')
                if v is not None:
                    valori.append(v)
            if valori:
                break
        if not valori:
            printer("    niciun mesaj de statistici; nu pot masura")
            return None, {}
        valori.sort()
        median = valori[len(valori) // 2]
        return median, {'n': len(valori), 'min': valori[0],
                        'max': valori[-1], 'topic': topic}
    finally:
        for a in abonati:
            _kill_group(a, timeout=5)
        _kill_group(server)


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('worlds', nargs='+')
    p.add_argument('--short', type=int, default=3000)
    p.add_argument('--long', type=int, default=30000)
    p.add_argument('--force', action='store_true',
                   help='masoara chiar daca ruleaza alt gz sim (cifra va fi '
                        'falsa; exista doar pentru depanare)')
    p.add_argument('--subscribe', action='append', default=[],
                   metavar='TOPIC',
                   help='tine un abonat atasat pe toata fereastra de '
                        'masurare (ex. /down_cam/image). Comuta pe '
                        'masuratoarea "vie": fara abonat, camera nu '
                        'randeaza deloc si RTF-ul nu spune nimic despre ea.')
    p.add_argument('--warmup', type=float, default=10.0)
    p.add_argument('--check-topic', default=None,
                   help='verifica intai ca soseste macar un mesaj, '
                        'ex. /down_cam/image')
    a = p.parse_args(argv)

    if not a.force and not refuse_if_busy():
        return 2

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
        # inca o data, chiar inainte de cronometru: --check-topic a pornit si
        # a oprit un server, iar daca a ramas ceva in viata cifra e gunoi
        if not a.force and not refuse_if_busy():
            return 2
        if a.subscribe:
            rtf, d = measure_live(w, a.subscribe, warmup=a.warmup)
            if rtf is None:
                continue
            rezultate.append((w, rtf))
            print(f"    RTF raportat de Gazebo: {rtf:.2f}  "
                  f"({d['n']} esantioane, {d['min']:.2f}-{d['max']:.2f})")
        else:
            rtf, pornire, d = measure(w, a.short, a.long)
            if rtf is None:
                continue
            rezultate.append((w, rtf))
            print(f"    pornire {pornire:.2f} s (exclusa din RTF)")
            print(f"    RTF in regim stabil: {rtf:.2f}")
            print("    (fara --subscribe: o camera fara abonat NU randeaza, "
                  "deci\n     cifra asta nu spune nimic despre costul ei)")
        if rtf < 0.5:
            # NU recomanda --scale automat: costul camerei trebuie masurat
            # intai (o lume fara camera, in acelasi mediu). Daca fizica
            # domina - si domina, la pas de 1 ms cu lift-drag pe patru
            # rotoare - rezolutia redusa nu cumpara nimic si muta toate
            # pragurile exprimate in pixeli.
            print("    SUB 0.5. Inainte de a schimba ceva, masoara pe ce se\n"
                  "    duce timpul: ruleaza si o lume FARA camera, in acelasi\n"
                  "    mediu, si compara. Rezolutia redusa ajuta doar daca\n"
                  "    randarea domina.")

    if len(rezultate) == 2:
        (w1, r1), (w2, r2) = rezultate
        print(f"\n  {os.path.basename(w1)} {r1:.2f}  vs  "
              f"{os.path.basename(w2)} {r2:.2f}   "
              f"(raport {r2 / r1:.2f})")
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
