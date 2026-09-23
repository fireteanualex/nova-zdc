#!/usr/bin/env python3
"""
Cine ocupa portul serial (H2).

`nova-monitor.service` si `tools/nova_pi.py` folosesc amandoua /dev/serial0,
si nu pot simultan. A doua pornire esueaza cu un `SerialException: could not
open port /dev/serial0: [Errno 16] Device or resource busy` - un mesaj care
spune CE s-a intamplat si nu spune nimic despre DE CE sau ce sa faci.

Pe teren, sub presiune, intre doua curse, asta e genul de eroare care costa
un slot intreg: cineva porneste aplicatia de bord, vede "resource busy",
crede ca e o problema de cablu sau de permisiuni, si incepe sa caute in
directia gresita.

Modulul raspunde la o singura intrebare - **cine tine portul?** - si o face
inainte de a incerca deschiderea, ca mesajul sa vina la timp.

Doua surse, in ordinea increderii:

1. `systemctl is-active nova-monitor` - daca serviciul ruleaza, el e.
   Raspunsul e clar si vine cu comanda de reparare.
2. `fuser` / `/proc/*/fd` - orice alt proces. Mai general, dar identifica
   doar PID-ul si linia de comanda.

Ambele sunt optionale: pe o masina fara systemd, sau fara drept de citire in
/proc, functiile intorc "nu stiu", nu o eroare. Un diagnostic care nu se
poate face nu are voie sa blocheze pornirea - blocarea o decide apelantul,
si doar cand stie sigur.
"""

import glob
import os
import subprocess

SERVICE_NAME = 'nova-monitor'

#: Serviciul de UTILIZATOR din pi/nova-bringup.service (pornirea automata cu
#: fereastra). Tine si /dev/serial0, si camera. Pe vehicul, pornit la login,
#: a facut ca un `pi/bringup.sh` rulat de mana sa gaseasca portul ocupat si
#: camera luata ("Pipeline handler in use by another process") - iar
#: --stop-service oprea doar serviciul de SISTEM de mai sus, nu pe acesta.
USER_SERVICE_NAME = 'nova-bringup'
DEFAULT_PORT = '/dev/serial0'


class PortBusy(Exception):
    """Portul e ocupat si stim de cine. Mesajul contine si repararea."""

    def __init__(self, message, service=None, pids=()):
        super().__init__(message)
        self.service = service
        self.pids = tuple(pids)


def _run(cmd, timeout=3.0):
    """(rc, stdout) sau (None, '') daca unealta lipseste."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout)
        return p.returncode, (p.stdout or '').strip()
    except (OSError, subprocess.SubprocessError):
        return None, ''


def service_active(name=SERVICE_NAME, runner=_run):
    """True / False / None (nu putem sti: fara systemd, sau unitate absenta)."""
    rc, out = runner(['systemctl', 'is-active', name])
    if rc is None:
        return None
    if out == 'active':
        return True
    if out in ('inactive', 'failed', 'activating', 'deactivating'):
        return out == 'activating'
    # 'unknown' sau unitate inexistenta
    return None


def user_service_active(name=USER_SERVICE_NAME, runner=_run):
    """True/False/None, ca `service_active`, dar pentru serviciul de UTILIZATOR."""
    rc, out = runner(['systemctl', '--user', 'is-active', name])
    if rc is None:
        return None
    if out == 'active' or out == 'activating':
        return True
    if out in ('inactive', 'failed', 'unknown'):
        return False
    return None


def _ppid(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            # campul 4; numele (campul 2) poate contine spatii, deci dupa ')'
            return int(f.read().rsplit(')', 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def in_user_service(name=USER_SERVICE_NAME, runner=_run, pid=None):
    """Suntem chiar procesul serviciului (sau un descendent al lui)?

    Fara asta, `nova_pi.py` pornit DE serviciu s-ar vedea pe sine ca
    "serviciul ocupa portul" si ar refuza sa porneasca. Se urca pe arborele
    de procese pana la MainPID-ul serviciului - mai robust decat o variabila
    de mediu, pe care o pot mosteni si terminale pornite altfel."""
    rc, out = runner(['systemctl', '--user', 'show', '-p', 'MainPID',
                      '--value', name])
    if rc != 0 or not (out or '').strip().isdigit():
        return False
    main = int(out.strip())
    if main <= 1:
        return False
    p = os.getpid() if pid is None else pid
    for _ in range(64):                  # adancime maxima rezonabila
        if p == main:
            return True
        if p <= 1:
            return False
        p = _ppid(p)
    return False


def holders(port=DEFAULT_PORT, runner=_run):
    """[(pid, cmdline)] pentru procesele care tin portul deschis.

    `fuser` e calea scurta; daca lipseste, cautam prin /proc, care nu are
    nevoie de niciun pachet in plus. Procesele altui utilizator nu se vad
    fara root - de aceea lista goala NU inseamna "portul e liber"."""
    out_pids = []
    rc, out = runner(['fuser', port])
    if rc is not None and out:
        for tok in out.replace(port + ':', '').split():
            tok = tok.strip().rstrip('crwxf')
            if tok.isdigit():
                out_pids.append(int(tok))

    if not out_pids:
        try:
            real = os.path.realpath(port)
        except OSError:
            real = port
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            fd_dir = os.path.join('/proc', entry, 'fd')
            try:
                for fd in os.listdir(fd_dir):
                    try:
                        tinta = os.readlink(os.path.join(fd_dir, fd))
                    except OSError:
                        continue
                    if tinta in (port, real):
                        out_pids.append(int(entry))
                        break
            except OSError:
                continue          # proces disparut sau fara drepturi

    rezultat = []
    for pid in sorted(set(out_pids)):
        if pid == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline") as f:
                cmd = f.read().replace('\x00', ' ').strip()
        except OSError:
            cmd = '?'
        rezultat.append((pid, cmd))
    return rezultat


def camera_holders(runner=_run, paths=None):
    """[(pid, cmdline)] pentru procesele care tin camera deschisa.

    libcamera accepta un singur proces pe camera. Al doilea primeste
    "Pipeline handler in use by another process" urmat de "Camera __init__
    sequence did not complete" - adica CE, nu si CINE. Nodurile /dev/media*
    si /dev/video* sunt cele pe care le tine procesul care a luat-o."""
    if paths is None:
        paths = sorted(glob.glob('/dev/media*')) + sorted(glob.glob('/dev/video*'))
    gasiti = {}
    for p in paths:
        for pid, cmd in holders(p, runner=runner):
            gasiti[pid] = cmd
    return sorted(gasiti.items())


def describe_camera_conflict(runner=_run, paths=None):
    """Mesaj cu CINE tine camera, sau None daca nu se vede nimeni."""
    if (user_service_active(runner=runner)
            and not in_user_service(runner=runner)):
        return (f"camera e luata de pornirea automata ({USER_SERVICE_NAME}).\n"
                f"        systemctl --user stop {USER_SERVICE_NAME}")
    ocupanti = camera_holders(runner=runner, paths=paths)
    if ocupanti:
        linii = '\n'.join(f"        PID {pid}: {cmd[:70]}"
                          for pid, cmd in ocupanti)
        return (f"camera e deja deschisa de alt proces:\n{linii}\n"
                f"        Opreste-l (kill <PID>) si reia.")
    return None


def describe_conflict(port=DEFAULT_PORT, service=SERVICE_NAME, runner=_run):
    """Mesajul de eroare, sau None daca portul pare liber.

    Nu deschide portul: e o interogare, si trebuie sa poata fi rulata si
    cand portul chiar e ocupat."""
    if service_active(service, runner=runner):
        return (f"{service} ocupa {port}.\n"
                f"        sudo systemctl stop {service}\n"
                f"        (sau porneste cu --stop-service, care o face "
                f"singur)")
    if (user_service_active(runner=runner)
            and not in_user_service(runner=runner)):
        return (f"pornirea automata ({USER_SERVICE_NAME}) ruleaza deja si "
                f"tine {port} SI camera.\n"
                f"        systemctl --user stop {USER_SERVICE_NAME}\n"
                f"        (sau porneste cu --stop-service, care o face "
                f"singur;\n"
                f"         o repornesti cu: systemctl --user start "
                f"{USER_SERVICE_NAME})")
    ocupanti = holders(port, runner=runner)
    if ocupanti:
        linii = '\n'.join(f"        PID {pid}: {cmd[:70]}"
                          for pid, cmd in ocupanti)
        return (f"{port} e deja deschis de alt proces:\n{linii}\n"
                f"        Opreste-l, sau foloseste --conn cu alt port.")
    return None


def stop_service(service=SERVICE_NAME, runner=_run):
    """(reusit, mesaj). Apelantul confirma INAINTE - aici doar executam."""
    rc, out = runner(['sudo', 'systemctl', 'stop', service], timeout=20.0)
    if rc is None:
        return False, 'systemctl lipseste'
    if rc != 0:
        return False, f"systemctl stop a iesit cu {rc}: {out}"
    if service_active(service, runner=runner):
        return False, f"{service} inca e activ dupa stop"
    return True, f"{service} oprit"


def ensure_port_free(port=DEFAULT_PORT, service=SERVICE_NAME,
                     stop_service_ok=False, confirm=None, runner=_run,
                     printer=print):
    """Verifica portul inainte de a-l deschide. Ridica `PortBusy` daca e luat.

    `stop_service_ok` + `confirm` implementeaza `--stop-service`: oprirea
    unui serviciu e o actiune cu efect in afara procesului nostru, deci nu se
    face din inertie. Cine porneste cu flagul a cerut-o; `confirm` mai
    intreaba o data, ca sa nu fie o surpriza intr-un scriptulet uitat.

    Portul care nu exista deloc NU e treaba noastra: `Vehicle.connect()` da
    deja un mesaj bun pentru asta, iar aici l-am dubla."""
    if port.startswith(('udp', 'tcp')):
        return True                      # SITL: nu exista conflict de port

    motiv = describe_conflict(port, service, runner=runner)
    if motiv is None:
        return True

    e_utilizator = (bool(user_service_active(runner=runner))
                    and not in_user_service(runner=runner))
    if e_utilizator and stop_service_ok:
        # Fara sudo: e al utilizatorului curent. Si fara confirmare in plus:
        # nu opreste nimic in afara sesiunii lui, iar cine a dat
        # --stop-service a cerut exact asta.
        printer(f"[serial] opresc pornirea automata ({USER_SERVICE_NAME}) - "
                f"tine portul si camera")
        rc, out = runner(['systemctl', '--user', 'stop', USER_SERVICE_NAME],
                         timeout=20.0)
        if rc == 0 and not user_service_active(runner=runner):
            printer(f"[serial] {USER_SERVICE_NAME} oprit. Il repornesti cu: "
                    f"systemctl --user start {USER_SERVICE_NAME}")
            return True
        raise PortBusy(f"nu am putut opri {USER_SERVICE_NAME}: {out}\n"
                       f"        {motiv}", service=USER_SERVICE_NAME)

    e_serviciul = bool(service_active(service, runner=runner))
    if e_serviciul and stop_service_ok:
        if confirm is not None and not confirm(service):
            raise PortBusy(f"oprirea lui {service} a fost refuzata.\n"
                           f"        {motiv}", service=service)
        printer(f"[serial] opresc {service} ...")
        ok, mesaj = stop_service(service, runner=runner)
        printer(f"[serial] {mesaj}")
        if ok:
            return True
        raise PortBusy(f"{mesaj}\n        {motiv}", service=service)

    raise PortBusy(motiv, service=service if e_serviciul else None,
                   pids=[p for p, _ in holders(port, runner=runner)])
