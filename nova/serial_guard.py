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

import os
import subprocess

SERVICE_NAME = 'nova-monitor'
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


def describe_conflict(port=DEFAULT_PORT, service=SERVICE_NAME, runner=_run):
    """Mesajul de eroare, sau None daca portul pare liber.

    Nu deschide portul: e o interogare, si trebuie sa poata fi rulata si
    cand portul chiar e ocupat."""
    if service_active(service, runner=runner):
        return (f"{service} ocupa {port}.\n"
                f"        sudo systemctl stop {service}\n"
                f"        (sau porneste cu --stop-service, care o face "
                f"singur)")
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
