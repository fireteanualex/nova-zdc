#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru setul de parametri CUAV X7+ (ArduCopter 4.7.1), fara FC.

    python3 tools/test_params_x7.py

The flight controller is a CUAV X7+ running ArduCopter 4.7.1, with no
hardware safety switch fitted; the code targets 4.7.1 ONLY (user decision,
04.10.2026). Three things are checked here, each with a negative case:

  (a) the X7+ set in config/nova_flight.parm (the one parm file, loaded as
      it is): BRD_SAFETY_DEFLT 0 and BRD_SAFETYOPTION 0, the SERIAL2 link
      values, and NO line for what the team sets by hand or what we never
      put on the FC: SERIALx_BAUD / SERIALx_OPTIONS, ARMING_* (arming
      checks), FENCE_* (no fence on the FC);
  (b) tools/check_params.py on that set, against a fake FC: everything
      read back -> OK; BRD_SAFETY_DEFLT read back as 1 -> mismatch; a
      hand-set name (SERIALx_BAUD / SERIALx_OPTIONS / ARMING_CHECK /
      ARMING_SKIPCHK) in a parm file is never written and never a mismatch;
  (c) every parameter name used by the vehicle-side code (the parm file,
      Python, the SITL EXTRA_PARAMS, pi/*.sh with their embedded Python)
      exists in tools/params_copter_4.7.1.txt (the name list of the
      Copter-4.7.1 tag), with NO exception list. A name that does not exist
      on the board gets no PARAM_VALUE and no error - the failure is silent
      (§5.4/§5.10).
"""

import ast
import contextlib
import glob
import io
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import check_params as cp                                   # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARM = os.path.join(REPO, 'config', 'nova_flight.parm')
NAMES_FILE = os.path.join(REPO, 'tools', 'params_copter_4.7.1.txt')

#: What the X7+ without a safety switch needs.
X7_SET = {'BRD_SAFETY_DEFLT': 0.0, 'BRD_SAFETYOPTION': 0.0}
#: The Pi link: since 05.10.2026 on SERIAL4, every serial-port setting set
#: by hand by the team - nothing of it in the parm file.
LINK_SET = {}
#: Names that must have NO line in the parm file: hand-set by the team
#: (serial speed/options, 02.10.2026; arming checks, 04.10.2026) or never
#: put on the FC (no fence, 02.10.2026).
INTERZISE_RE = re.compile(r'^(SERIAL\d+_(BAUD|OPTIONS|PROTOCOL)|BRD_SER\d+_RTSCTS'
                          r'|ARMING_\w+|FENCE_\w+)$')


def _parm(path):
    return {n: v for n, v, _ in cp.parse_parm(path)}


# --- (a) the X7+ set --------------------------------------------------------

def probleme_x7(path):
    """What is wrong with a parm file against the X7+ set; [] = nothing."""
    f = _parm(path)
    out = []
    for name, want in {**X7_SET, **LINK_SET}.items():
        if f.get(name) != want:
            out.append(f"{name}={f.get(name)}, cerut {want:g}")
    # bit 3 (force safety on at disarm) must be OFF: re-arming without a
    # button would be impossible after any disarm
    if int(f.get('BRD_SAFETYOPTION', 0)) & (1 << 3):
        out.append('BRD_SAFETYOPTION bitul 3 aprins')
    if 'BRD_SAFETY_MASK' in f:
        out.append('BRD_SAFETY_MASK nu se scrie')
    out += [f"{n}: nu se scrie din fisier" for n in f if INTERZISE_RE.match(n)]
    return out


def test_setul_X7_in_fisier():
    assert probleme_x7(PARM) == [], probleme_x7(PARM)
    txt = open(PARM).read()
    # the 4.7.1 citations of the TELEM2 mapping, not the 4.5.7 ones
    assert 'Copter-4.7.1' in txt and 'CUAV-X7/hwdef.dat:22' in txt, (
        "citarea SERIAL_ORDER pe Copter-4.7.1 lipseste")
    assert 'USART6 is telem2' in txt and 'hwdef.dat:141' in txt
    assert 'Pixhawk6C/hwdef.dat' not in txt
    assert 'NOT confirmed' in txt, "pinout-ul TELEM2 nu e confirmat pe X7+"
    # loaded as it is: no generated variant any more
    for vechi in ('nova_flight_4.5.parm', 'make_parm_45'):
        assert vechi not in txt, f"{vechi} mentionat in {PARM}"
    assert 'running ArduCopter\n# 4.7.1' in txt or '4.7.1 ONLY' in txt
    # the user's own comment on the arming checks, in place of the old line
    assert ('# Arming checks are set by hand by the team (user decision '
            '04.10.2026).') in txt
    return "BRD_SAFETY_DEFLT 0, BRD_SAFETYOPTION 0; fara SERIALx_*, " \
           "BRD_SERx_RTSCTS, ARMING_*, FENCE_*"


def test_NEGATIV_setul_X7_stricat_e_prins():
    """The (a) checks must be able to fail: a copy without the two safety
    lines and with the forbidden lines put back is caught, line by line."""
    tmp = tempfile.mkdtemp()
    cale = os.path.join(tmp, 'nova_flight.parm')
    with open(cale, 'w') as f:
        for line in open(PARM):
            if not line.startswith(('BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION')):
                f.write(line)
        f.write('SERIAL2_BAUD,921\nSERIAL2_OPTIONS,0\nARMING_SKIPCHK,32768\n'
                'ARMING_CHECK,1\nFENCE_ENABLE,1\n')
    p = probleme_x7(cale)
    assert len(p) == 7, p
    for n in ('BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION', 'SERIAL2_BAUD',
              'SERIAL2_OPTIONS', 'ARMING_SKIPCHK', 'ARMING_CHECK',
              'FENCE_ENABLE'):
        assert any(x.startswith(n) for x in p), (n, p)
    # ...and bit 3 alone is caught too
    with open(cale, 'w') as f:
        f.write(open(PARM).read().replace('BRD_SAFETYOPTION,0',
                                          'BRD_SAFETYOPTION,8'))
    p = probleme_x7(cale)
    assert 'BRD_SAFETYOPTION bitul 3 aprins' in p, p
    return "2 lipsa + 5 interzise -> 7 probleme; bitul 3 prins"


def test_check_params_descrie_BRD_SAFETYOPTION():
    d = cp.describe_mask('BRD_SAFETYOPTION', 3)       # the X7 default
    aprinse, stinse = d.split('APRINS :')[1].split('STINS  :')
    assert '0:' in aprinse and '1:' in aprinse and '3:' in stinse, d
    d0 = cp.describe_mask('BRD_SAFETYOPTION', 0)
    assert '(niciunul)' in d0.split('STINS')[0], d0
    # the arming-check mask is the team's: not described by the tool
    assert not [n for n in cp.BITMASKS if n.startswith('ARMING')], (
        list(cp.BITMASKS))
    assert not hasattr(cp, 'BITI_ARMARE')
    return "3 -> biti 0,1 aprinsi, 3 stins; 0 -> niciunul; nimic de armare"


# --- (b) check_params against a fake FC --------------------------------------

class FakeFC:
    """Answers PARAM_REQUEST_READ / PARAM_SET by name, like ArduPilot; an
    unknown name gets silence. Same approach as tools/test_pi_tooling.py."""

    def __init__(self, params, armat=False):
        self.p = dict(params)
        self.armat = armat
        self.scrieri = []
        self.citiri = []
        self.reboot = False
        self._coada = []
        self.target_system, self.target_component = 1, 1
        fc = self

        class _Mav:
            def param_request_read_send(self, s, c, name, idx):
                n = name.decode()
                fc.citiri.append(n)
                if n in fc.p:
                    fc._coada.append((n, fc.p[n]))

            def param_set_send(self, s, c, name, value, typ):
                n = name.decode()
                fc.scrieri.append(n)
                if n in fc.p:
                    fc.p[n] = float(value)

            def command_long_send(self, *a):
                fc.reboot = True
        self.mav = _Mav()

    def wait_heartbeat(self, *a, **kw):
        return True

    def recv_match(self, type=None, blocking=True, timeout=None):
        from pymavlink import mavutil
        if type == 'HEARTBEAT':
            fc = self

            class _HB:
                base_mode = (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                             if fc.armat else 0)

                def get_srcSystem(s):
                    return 1
            return _HB()
        if self._coada:
            n, v = self._coada.pop(0)

            class _PV:
                param_id = n
                param_value = v
            return _PV()
        return None


def _fc_ca_fisierul(path=PARM, **schimbari):
    p = _parm(path)
    p.update(schimbari)
    return FakeFC(p)


def _quiet(fn, *a):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        r = fn(*a)
    return r, out.getvalue()


def test_check_params_totul_se_potriveste():
    assert os.path.samefile(cp.DEFAULT_PARM, PARM), cp.DEFAULT_PARM
    wanted = cp.parse_parm(PARM)
    fc = _fc_ca_fisierul()
    (ok, mismatch, missing, manual), out = _quiet(
        cp.verify_params, fc, wanted, PARM)
    assert not mismatch and not missing and not manual, (mismatch, missing)
    assert len(ok) == len(wanted) and 'BRD_SAFETY_DEFLT' in ok
    assert cp.write_params(fc, wanted) and not fc.scrieri, (
        f"a rescris parametri deja corecti: {fc.scrieri}")
    return f"{len(ok)}/{len(wanted)} ok, 0 scrieri"


def test_NEGATIV_BRD_SAFETY_DEFLT_1_e_nepotrivire():
    """The X7 default (1) left on the board: outputs blocked, arming fails on
    'Hardware safety switch'. Must be a mismatch, not ok."""
    wanted = cp.parse_parm(PARM)
    fc = _fc_ca_fisierul(BRD_SAFETY_DEFLT=1.0, BRD_SAFETYOPTION=3.0)
    (ok, mismatch, missing, _), out = _quiet(
        cp.verify_params, fc, wanted, PARM)
    gresite = {n: g for n, _w, g, _l in mismatch}
    assert gresite == {'BRD_SAFETY_DEFLT': 1.0, 'BRD_SAFETYOPTION': 3.0}, (
        gresite)
    assert 'NU SE POTRIVESTE' in out and 'BRD_SAFETYOPTION' in out
    # the write path fixes both, confirmed by read-back
    rez = cp.write_params(fc, wanted)
    assert set(fc.scrieri) == {'BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION'}
    assert all(g is not None and cp.close_enough(w, g) for _, w, g in rez)
    return "DEFLT 1 / OPTION 3 -> 2 nepotriviri; --write le corecteaza"


#: Every hand-set name, with the value a parm file asks for and the value
#: the team left on the FC (different on purpose).
DE_MANA = {'SERIAL2_BAUD': (57.0, 921.0), 'SERIAL2_OPTIONS': (4.0, 0.0),
           'SERIAL4_PROTOCOL': (1.0, 2.0), 'BRD_SER4_RTSCTS': (2.0, 0.0),
           'ARMING_CHECK': (1.0, 0.0), 'ARMING_SKIPCHK': (32768.0, 0.0)}


def _parm_cu_de_mana(tmp):
    cale = os.path.join(tmp, 'de_mana.parm')
    with open(cale, 'w') as f:
        f.write("FS_THR_ENABLE,1\n")
        for n, (cerut, _) in DE_MANA.items():
            f.write(f"{n},{cerut:g}\n")
        f.write("BRD_SAFETY_DEFLT,0\n")
    return cale


def _fc_de_mana():
    p = {n: pe_fc for n, (_, pe_fc) in DE_MANA.items()}
    p.update({'FS_THR_ENABLE': 1.0, 'BRD_SAFETY_DEFLT': 1.0})
    return FakeFC(p)


def test_NEGATIV_garda_setati_de_mana():
    """User decisions: SERIALx_BAUD / SERIALx_OPTIONS (02.10.2026) and the
    arming checks ARMING_CHECK / ARMING_SKIPCHK (04.10.2026) are set by hand.
    Even if a parm file asks for other values, check_params sends ZERO
    PARAM_SET for them, does not even read them, and reports no mismatch."""
    tmp = tempfile.mkdtemp()
    cale = _parm_cu_de_mana(tmp)
    fc = _fc_de_mana()
    wanted = cp.parse_parm(cale)
    rez = cp.write_params(fc, wanted)
    for n, (_, pe_fc) in DE_MANA.items():
        assert n not in fc.scrieri, f"PARAM_SET pe {n}: {fc.scrieri}"
        assert n not in fc.citiri, f"a citit {n}: {fc.citiri}"
        assert n not in {r[0] for r in rez}, rez
        assert fc.p[n] == pe_fc, (n, fc.p[n])
    assert fc.scrieri == ['BRD_SAFETY_DEFLT'], fc.scrieri

    (ok, mismatch, missing, manual), out = _quiet(
        cp.verify_params, fc, wanted, cale)
    assert not mismatch and not missing, (mismatch, missing)
    assert [m[0] for m in manual] == list(DE_MANA), manual
    assert out.count('setat de mana, ignorat') == len(DE_MANA), out
    for da in ('SERIAL12_BAUD', 'SERIAL0_OPTIONS', 'SERIAL2_PROTOCOL',
               'SERIAL4_PROTOCOL', 'BRD_SER2_RTSCTS', 'ARMING_CHECK',
               'ARMING_SKIPCHK'):
        assert cp.is_hand_set(da), da
    # only those: the rest of the ARMING_ group and look-alikes are not
    for nu in ('SERIAL2_PROTOCOLX', 'XSERIAL2_BAUD', 'ARMING_RUDDER',
               'ARMING_OPTIONS', 'ARMING_CHECKX', 'XARMING_CHECK',
               'BRD_SAFETY_DEFLT'):
        assert not cp.is_hand_set(nu), nu
    return (f"{len(DE_MANA)} cerute -> 0 PARAM_SET, 0 citiri, 0 nepotriviri, "
            f"{len(DE_MANA)} 'setat de mana'")


def test_NEGATIV_garda_prin_CLI_write():
    """The same through main() - the path an operator and the preflight
    (subprocess) take - with --write --yes, then a plain read-back."""
    from pymavlink import mavutil
    tmp = tempfile.mkdtemp()
    cale = _parm_cu_de_mana(tmp)
    fc = _fc_de_mana()
    orig_conn, orig_argv = mavutil.mavlink_connection, sys.argv
    mavutil.mavlink_connection = lambda *a, **kw: fc
    try:
        sys.argv = ['check_params.py', '--conn', 'fake', '--parm', cale,
                    '--write', '--yes']
        cod_w, out_w = _quiet(cp.main)
        sys.argv = ['check_params.py', '--conn', 'fake', '--parm', cale]
        cod_r, out_r = _quiet(cp.main)
    finally:
        mavutil.mavlink_connection, sys.argv = orig_conn, orig_argv
    assert cod_w == 0 and cod_r == 0, (cod_w, cod_r, out_w, out_r)
    assert not set(DE_MANA) & set(fc.scrieri), fc.scrieri
    assert '2/2 scrisi si confirmati' in out_w, out_w
    assert '0 nepotriviri' in out_r and 'setat de mana, ignorat' in out_r
    # the preflight keeps only the summary: nothing hand-set shows as a diff
    import preflight_check as pf
    r = pf.rezumat_parametri(out_r)
    assert r.startswith('2 ok | 0 nepotriviri') and 'SERIAL2' not in r \
        and 'ARMING' not in r, r
    assert os.path.samefile(pf.FLIGHT_PARM, PARM), pf.FLIGHT_PARM
    return f"--write: {fc.scrieri}; citire: {r}"


# --- (c) every parameter name exists on Copter-4.7.1 -------------------------

#: Group prefixes that existed on Copter-4.5.7 and are GONE on 4.7.1
#: (computed once from the two tags' name lists, 04.10.2026, before the
#: 4.5.7 list was deleted). Without them the prefix filter below would hide
#: exactly the old names - WPNAV_ACCEL, ANGLE_MAX, SYSID_MYGCS - whose group
#: was renamed. Every 4.7.1 prefix comes from the vendored list itself.
EXTRA_PREFIXES = {'ANGLE', 'SR0', 'SR1', 'SR2', 'SR3', 'SR4', 'SR5', 'SR6',
                  'SYSID', 'TELEM', 'VIEP', 'WPNAV'}

#: Tokens that pass every filter below but are not parameters. Keep it small
#: and say why for each entry.
NOT_PARAMS = {}

#: AP_MAX_NAME_SIZE, Copter-4.7.1 libraries/AP_Param/AP_Param.h:37.
MAX_NAME = 16

TOKEN = re.compile(r'(?<![A-Za-z0-9_\x00])[A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+'
                   r'(?![A-Za-z0-9_\x00])')


def nume_471():
    with open(NAMES_FILE) as f:
        return {s.strip() for s in f if s.strip() and not s.startswith('#')}


def _mavlink_names():
    """Message and enum-entry names of the ardupilotmega dialect: RC_CHANNELS,
    LOG_DATA, MAV_STATE_ACTIVE... look like parameters and are not."""
    from pymavlink.dialects.v20 import ardupilotmega as mav
    msgs = {k[len('MAVLINK_MSG_ID_'):] for k in dir(mav)
            if k.startswith('MAVLINK_MSG_ID_')}
    enums = {e.name for en in mav.enums.values() for e in en.values()}
    return msgs | enums


def _python_strings(src):
    """(line, text) for every string literal that is NOT a docstring, every
    f-string (placeholders become \\x00, so a name built at run time such as
    f"EK3_SRC{n}_POSXY" is cut and not checked) and every keyword-argument
    name (Band(..., PSC_NE_POS_P=0.7))."""
    tree = ast.parse(src)
    skip = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            body = node.body
            if body and isinstance(body[0], ast.Expr) and \
                    isinstance(body[0].value, ast.Constant) and \
                    isinstance(body[0].value.value, str):
                skip.add(id(body[0].value))
        if isinstance(node, ast.JoinedStr):
            for v in node.values:
                skip.add(id(v))
            yield node.lineno, ''.join(
                v.value if isinstance(v, ast.Constant) else '\x00'
                for v in node.values)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) \
                and id(node) not in skip:
            yield node.lineno, node.value
        elif isinstance(node, ast.keyword) and node.arg:
            yield node.value.lineno, node.arg


def nume_parametri(texte, known):
    """{name: line} of the parameter-like tokens in `texte` [(line, text)].

    Filters, in order (documented, so a miss can be argued about):
      1. UPPER_CASE token with at least one underscore, whole word;
      2. at most MAX_NAME characters (ArduPilot cannot store longer names);
      3. its first segment is a parameter group prefix of 4.7.1 (from the
         vendored list) or in EXTRA_PREFIXES - drops our own state names
         (DESCEND_TRACK), env vars, mode names;
      4. not a MAVLink message / enum name, not in NOT_PARAMS.
    """
    prefixes = {n.split('_')[0] for n in known if '_' in n} | EXTRA_PREFIXES
    mav = _mavlink_names()
    out = {}
    for line, text in texte:
        for tok in TOKEN.findall(text):
            if len(tok) > MAX_NAME or tok.split('_')[0] not in prefixes:
                continue
            if tok in mav or tok in NOT_PARAMS:
                continue
            out.setdefault(tok, line)
    return out


def nume_din_python(path, known):
    with open(path) as f:
        return nume_parametri(_python_strings(f.read()), known)


def nume_din_parm(path):
    return {n: line for n, _, line in cp.parse_parm(path)}


#: Calls whose first argument is a parameter name, in the SITL tools.
SET_CALLS = {'set_param', 'param_set_send', 'request_param'}


def nume_din_sitl(path):
    """tools/sitl_*.py: the EXTRA_PARAMS block ('NAME value' lines), if the
    file defines one, plus the literal first argument of every
    set_param()/request_param() call - the rest of those files is SITL
    plumbing (mode-reason names, MAVLink types), not vehicle params."""
    tree = ast.parse(open(path).read())
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'EXTRA_PARAMS'
                for t in node.targets):
            for i, line in enumerate(node.value.value.splitlines()):
                if line.split():
                    out[line.split()[0]] = node.lineno + i
        elif isinstance(node, ast.Call) and node.args:
            f = node.func
            nume = f.attr if isinstance(f, ast.Attribute) else getattr(
                f, 'id', None)
            a0 = node.args[0]
            if nume in SET_CALLS and isinstance(a0, ast.Constant) and \
                    isinstance(a0.value, str):
                out.setdefault(a0.value, node.lineno)
    return out


#: Shell variables: NAME=... or $NAME / ${NAME in the same script.
SHELL_VAR = re.compile(r'(?:^|[\s;(])([A-Z][A-Z0-9_]*)=|\$\{?([A-Z][A-Z0-9_]*)')


def nume_din_shell(path, known, text=None):
    """pi/*.sh: the shell text and the Python embedded in it (heredocs),
    scanned line by line like a string. Comment-only lines are skipped, as
    Python comments are; so are the script's own shell variables (LOG_DIR,
    AUTO_SRC: assigned or expanded in the same file), which share the
    UPPER_CASE look and sometimes a parameter prefix."""
    if text is None:
        with open(path) as f:
            text = f.read()
    shell_vars = {a or b for a, b in SHELL_VAR.findall(text)}
    lines = [(i, s) for i, s in enumerate(text.splitlines(), 1)
             if not s.lstrip().startswith('#')]
    return {n: line for n, line in nume_parametri(lines, known).items()
            if n not in shell_vars}


def surse_vehicul():
    """[(relative path, kind)] - the vehicle-side code and configs."""
    out = [('config/nova_flight.parm', 'parm')]
    out += sorted((os.path.relpath(p, REPO), 'py')
                  for p in glob.glob(os.path.join(REPO, 'nova', '*.py')))
    out += [('tools/check_params.py', 'py'), ('tools/preflight_check.py', 'py'),
            ('tools/collect_session.py', 'py'), ('tools/nova_pi.py', 'py')]
    out += sorted((os.path.relpath(p, REPO), 'sitl')
                  for p in glob.glob(os.path.join(REPO, 'tools', 'sitl_*.py')))
    out += sorted((os.path.relpath(p, REPO), 'sh')
                  for p in glob.glob(os.path.join(REPO, 'pi', '*.sh')))
    return out


def _nume(rel, kind, known):
    path = os.path.join(REPO, rel)
    if kind == 'parm':
        return nume_din_parm(path)
    if kind == 'sitl':
        return nume_din_sitl(path)
    if kind == 'sh':
        return nume_din_shell(path, known)
    return nume_din_python(path, known)


def test_lista_vendorizata_e_4_7_1():
    head = ''.join(l for l in open(NAMES_FILE) if l.startswith('#'))
    assert 'Copter-4.7.1' in head and 'param_parse.py' in head, head
    known = nume_471()
    assert len(known) > 5000, f"lista 4.7.1 are doar {len(known)} nume"
    # spot checks against the 4.7.1 source (cited in config/nova_flight.parm)
    assert {'WP_ACC', 'WP_SPD_DN', 'LAND_SPD_MS', 'RNGFND1_MIN',
            'ARMING_SKIPCHK', 'PSC_NE_POS_P', 'ATC_ANGLE_MAX'} <= known
    assert not {'WPNAV_ACCEL', 'LAND_SPEED', 'RNGFND1_MIN_CM', 'ARMING_CHECK',
                'PSC_POSXY_P', 'ANGLE_MAX'} & known
    # EXTRA_PREFIXES really are gone on 4.7.1
    assert not [n for n in known if n.split('_')[0] in EXTRA_PREFIXES]
    assert not os.path.exists(os.path.join(REPO, 'tools',
                                           'params_copter_4.5.7.txt'))
    return f"{len(known)} nume, Copter-4.7.1"


def test_niciun_nume_inexistent_pe_4_7_1():
    known = nume_471()
    surse = surse_vehicul()
    total, gresite = 0, []
    pe_fisier = {}
    for rel, kind in surse:
        nume = _nume(rel, kind, known)
        pe_fisier[rel] = nume
        total += len(nume)
        gresite += [f"{rel}:{line} {n}" for n, line in sorted(nume.items())
                    if n not in known]
    assert not gresite, (
        "nume care NU exista pe Copter-4.7.1 (pe placa: tacere, fara "
        "eroare):\n          " + "\n          ".join(gresite))
    # not vacuous: the scanner does see the names that matter
    assert {'PSC_NE_POS_P', 'WP_ACC', 'LAND_SPD_MS'} <= set(
        pe_fisier['nova/authority.py']), pe_fisier['nova/authority.py']
    assert len(pe_fisier['tools/collect_session.py']) >= 25
    assert {'BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION', 'WP_SPD_DN',
            'RNGFND1_GNDCLR'} <= set(pe_fisier['config/nova_flight.parm'])
    assert 'WP_SPD_DN' in pe_fisier['tools/sitl_touchdown.py']
    assert 'SIM_RC_FAIL' in pe_fisier['tools/sitl_pi_failure.py']
    assert {'FLTMODE_CH', 'FENCE_ENABLE'} <= set(
        pe_fisier['pi/descent_test.sh']), pe_fisier['pi/descent_test.sh']
    # the 4.5 file and its generator are gone, and nothing points at them
    for rel in ('config/nova_flight_4.5.parm', 'tools/make_parm_45.py'):
        assert not os.path.exists(os.path.join(REPO, rel)), rel
    return f"{total} nume in {len(surse)} fisiere, toate pe 4.7.1"


def test_NEGATIV_nume_inexistent_injectat_e_prins():
    """A 4.5-only name (WPNAV_ACCEL) and a nonsense name injected in a copy
    of real code must be caught; prose, docstrings, MAVLink names and
    run-time built names must not."""
    known = nume_471()
    cod = open(os.path.join(REPO, 'nova', 'authority.py')).read()
    assert not [n for n in nume_parametri(_python_strings(cod), known)
                if n not in known]
    injectat = cod + "\nX = ('WPNAV_ACCEL', 'PLND_BOGUS')\n" \
                     "Band('rea', 0.0, LAND_SPEED=50)\n"
    gasit = nume_parametri(_python_strings(injectat), known)
    rele = sorted(n for n in gasit if n not in known)
    assert rele == ['LAND_SPEED', 'PLND_BOGUS', 'WPNAV_ACCEL'], rele

    # the same in a parm file copy (temp), and in sitl EXTRA_PARAMS form
    tmp = tempfile.mkdtemp()
    parm = os.path.join(tmp, 'x.parm')
    with open(parm, 'w') as f:
        f.write(open(PARM).read() + 'WPNAV_ACCEL,150\nRNGFND1_BOGUS,1\n')
    assert sorted(n for n in nume_din_parm(parm) if n not in known) == \
        ['RNGFND1_BOGUS', 'WPNAV_ACCEL']
    extra = os.path.join(tmp, 'sitl.py')
    open(extra, 'w').write('EXTRA_PARAMS = """\nLAND_SPEED 50\n'
                           'RNGFND1_GNDCLR 0.07\n"""\n'
                           "pilot.set_param('SIM_RC_FAIL', 1)\n"
                           "pilot.set_param('WPNAV_SPEED_DN', 50)\n")
    assert sorted(n for n in nume_din_sitl(extra) if n not in known) == \
        ['LAND_SPEED', 'WPNAV_SPEED_DN']

    # pi/*.sh: a copy of descent_test.sh with the old arming query put back
    dt = open(os.path.join(REPO, 'pi', 'descent_test.sh')).read()
    sh_rau = dt.replace("'FENCE_ENABLE'):", "'FENCE_ENABLE', 'ARMING_CHECK'):")
    assert sh_rau != dt
    assert [n for n in nume_din_shell(None, known, sh_rau)
            if n not in known] == ['ARMING_CHECK']
    # ...while its own shell variables and comments are not parameters
    assert nume_din_shell(None, known, 'LOG_DIR="$HOME/x"\necho "$LOG_DIR"\n'
                          '# WPNAV_ACCEL in a comment\n') == {}

    # what must NOT count as a parameter name
    curat = ('"""Docstring: WPNAV_ACCEL was the 4.5 name."""\n'
             "m = 'RC_CHANNELS'\nk = 'LOG_DATA'\nf = 'DESCEND_TRACK'\n"
             "n = f'EK3_SRC{s}_POSXY'\n# WPNAV_ACCEL in a comment\n")
    assert nume_parametri(_python_strings(curat), known) == {}, (
        nume_parametri(_python_strings(curat), known))
    # an old name from a group 4.7.1 does not have at all (EXTRA_PREFIXES)
    for vechi in ('ANGLE_MAX', 'SYSID_MYGCS', 'SR2_POSITION'):
        assert list(nume_parametri([(1, vechi)], known)) == [vechi] and \
            vechi not in known, vechi
    return "nume 4.5 + nume inventate prinse in py/parm/sitl/sh; " \
           "docstring/comentariu/MAVLink/f-string nu"


def test_descent_test_nu_mai_spune_nimic_de_armare():
    """pi/descent_test.sh --check: the arming block is gone (text and
    query), user decision 04.10.2026."""
    dt = open(os.path.join(REPO, 'pi', 'descent_test.sh')).read()
    cod = '\n'.join(l for l in dt.splitlines()
                    if not l.lstrip().startswith('#'))
    for rau in ('ARMING_', 'Rangefinder 1: No', 'bitul 15', '1 << 15'):
        assert rau not in cod, rau
    return "fara ARMING_* si fara mesajul de rangefinder"


TESTS = [
    ('setul X7+ in config/nova_flight.parm', test_setul_X7_in_fisier),
    ('NEGATIV: setul X7 stricat e prins',
     test_NEGATIV_setul_X7_stricat_e_prins),
    ('check_params descrie BRD_SAFETYOPTION',
     test_check_params_descrie_BRD_SAFETYOPTION),
    ('check_params: totul se potriveste', test_check_params_totul_se_potriveste),
    ('NEGATIV: BRD_SAFETY_DEFLT 1 e nepotrivire',
     test_NEGATIV_BRD_SAFETY_DEFLT_1_e_nepotrivire),
    ('NEGATIV: garda setati de mana (SERIALx, ARMING)',
     test_NEGATIV_garda_setati_de_mana),
    ('NEGATIV: garda prin CLI --write', test_NEGATIV_garda_prin_CLI_write),
    ('lista vendorizata e 4.7.1', test_lista_vendorizata_e_4_7_1),
    ('niciun nume inexistent pe 4.7.1', test_niciun_nume_inexistent_pe_4_7_1),
    ('NEGATIV: nume inexistent injectat e prins',
     test_NEGATIV_nume_inexistent_injectat_e_prins),
    ('descent_test nu mai spune nimic de armare',
     test_descent_test_nu_mai_spune_nimic_de_armare),
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
