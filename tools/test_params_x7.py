#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste pentru setul de parametri CUAV X7+ (ArduCopter 4.5.7), fara FC.

    python3 tools/test_params_x7.py

The flight controller changed from Pixhawk 6C to CUAV X7+, with no hardware
safety switch fitted. Three things are checked here, each with a negative
case:

  (a) the X7+ set: both parm files carry BRD_SAFETY_DEFLT 0 and
      BRD_SAFETYOPTION 0, the SERIAL2 link values are unchanged, the
      hand-set SERIALx_BAUD / SERIALx_OPTIONS are in neither file, and the
      generated 4.5 file is what the generator produces;
  (b) tools/check_params.py on that set, against a fake FC: everything
      read back -> OK; BRD_SAFETY_DEFLT read back as 1 -> mismatch; a
      hand-set name in a parm file is never written and never a mismatch;
  (c) no parameter name used by the vehicle-side code (Python, parm files,
      pi/*.sh with their embedded Python) is a 4.7+/4.8 name:
      every one must exist in tools/params_copter_4.5.7.txt (the name list
      of the Copter-4.5.7 tag). A 4.7 name on a 4.5.7 board gets no
      PARAM_VALUE and no error - the failure is silent (§5.4/§5.10).
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
import make_parm_45 as mp                                   # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARM_47 = os.path.join(REPO, 'config', 'nova_flight.parm')
PARM_45 = os.path.join(REPO, 'config', 'nova_flight_4.5.parm')
NAMES_FILE = os.path.join(REPO, 'tools', 'params_copter_4.5.7.txt')

#: What the X7+ without a safety switch needs in BOTH files.
X7_SET = {'BRD_SAFETY_DEFLT': 0.0, 'BRD_SAFETYOPTION': 0.0}
#: The Pi link, unchanged from the Pixhawk 6C (TELEM2 = SERIAL2 on both).
LINK_SET = {'SERIAL2_PROTOCOL': 2.0, 'BRD_SER2_RTSCTS': 0.0}
HAND_SET_RE = re.compile(r'^SERIAL\d+_(BAUD|OPTIONS)$')


def _parm(path):
    return {n: v for n, v, _ in cp.parse_parm(path)}


# --- (a) the X7+ set --------------------------------------------------------

def test_setul_X7_in_ambele_fisiere():
    for path in (PARM_47, PARM_45):
        f = _parm(path)
        for name, want in {**X7_SET, **LINK_SET}.items():
            assert f.get(name) == want, (
                f"{os.path.basename(path)}: {name}={f.get(name)}, cerut {want:g}")
        # bit 3 (force safety on at disarm) must be OFF: re-arming without a
        # button would be impossible after any disarm
        assert not int(f['BRD_SAFETYOPTION']) & (1 << 3)
        assert 'BRD_SAFETY_MASK' not in f, "BRD_SAFETY_MASK nu se scrie"
        hand = [n for n in f if HAND_SET_RE.match(n)]
        assert not hand, f"{os.path.basename(path)}: {hand} sunt setati de mana"
    txt = open(PARM_47).read()
    assert 'CUAV-X7/hwdef.dat:24' in txt and 'USART6 is telem2' in txt
    assert 'Pixhawk6C/hwdef.dat' not in txt and 'AP_BoardConfig.cpp:70' not in txt
    assert 'NOT confirmed' in txt, "pinout-ul TELEM2 nu e confirmat pe X7+"
    return "BRD_SAFETY_DEFLT 0, BRD_SAFETYOPTION 0, SERIAL2_PROTOCOL 2, " \
           "BRD_SER2_RTSCTS 0, fara SERIALx_BAUD/OPTIONS, in ambele"


def test_fisierul_generat_e_la_zi():
    assert open(mp.TINTA).read() == mp.genereaza(), (
        "config/nova_flight_4.5.parm nu corespunde sursei: "
        "python3 tools/make_parm_45.py")
    # the two new names are identical on 4.7 and 4.5.7: copied, not renamed
    for name in X7_SET:
        assert name not in mp.REDENUMIRI and name not in mp.OMISI
    return "genereaza() == fisierul de pe disc"


def test_NEGATIV_setul_X7_lipsa_e_prins():
    """The (a) checks must be able to fail: a copy of the source without the
    two lines gives a generated text without them."""
    tmp = tempfile.mkdtemp()
    sursa = os.path.join(tmp, 'nova_flight.parm')
    with open(sursa, 'w') as f:
        for line in open(PARM_47):
            if not line.startswith(('BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION')):
                f.write(line)
        f.write('SERIAL2_BAUD,921\n')
    orig = mp.SURSA
    mp.SURSA = sursa
    try:
        text = mp.genereaza()
    finally:
        mp.SURSA = orig
    assert text != open(mp.TINTA).read()
    gen = os.path.join(tmp, 'gen.parm')
    open(gen, 'w').write(text)
    f = _parm(gen)
    assert f.get('BRD_SAFETY_DEFLT') is None
    assert [n for n in f if HAND_SET_RE.match(n)] == ['SERIAL2_BAUD']
    return "sursa fara cele doua linii -> generat diferit, setul X7 lipsa"


def test_check_params_descrie_BRD_SAFETYOPTION():
    d = cp.describe_mask('BRD_SAFETYOPTION', 3)       # the X7 default
    aprinse, stinse = d.split('APRINS :')[1].split('STINS  :')
    assert '0:' in aprinse and '1:' in aprinse and '3:' in stinse, d
    d0 = cp.describe_mask('BRD_SAFETYOPTION', 0)
    assert '(niciunul)' in d0.split('STINS')[0], d0
    # arming check bit 11 = hardware safety switch, unchanged
    assert cp.BITI_ARMARE[11] == 'comutator de siguranta'
    return "3 -> biti 0,1 aprinsi, 3 stins; 0 -> niciunul aprins"


# --- (b) check_params against a fake FC --------------------------------------

class FakeFC:
    """Answers PARAM_REQUEST_READ / PARAM_SET by name, like ArduPilot; an
    unknown name gets silence. Same approach as tools/test_pi_tooling.py."""

    def __init__(self, params, armat=False):
        self.p = dict(params)
        self.armat = armat
        self.scrieri = []
        self.reboot = False
        self._coada = []
        self.target_system, self.target_component = 1, 1
        fc = self

        class _Mav:
            def param_request_read_send(self, s, c, name, idx):
                n = name.decode()
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


def _fc_ca_fisierul(path=PARM_45, **schimbari):
    p = _parm(path)
    p.update(schimbari)
    return FakeFC(p)


def _quiet(fn, *a):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        r = fn(*a)
    return r, out.getvalue()


def test_check_params_totul_se_potriveste():
    wanted = cp.parse_parm(PARM_45)
    fc = _fc_ca_fisierul()
    (ok, mismatch, missing, manual), out = _quiet(
        cp.verify_params, fc, wanted, PARM_45)
    assert not mismatch and not missing and not manual, (mismatch, missing)
    assert len(ok) == len(wanted) and 'BRD_SAFETY_DEFLT' in ok
    assert cp.write_params(fc, wanted) and not fc.scrieri, (
        f"a rescris parametri deja corecti: {fc.scrieri}")
    return f"{len(ok)}/{len(wanted)} ok, 0 scrieri"


def test_NEGATIV_BRD_SAFETY_DEFLT_1_e_nepotrivire():
    """The X7 default (1) left on the board: outputs blocked, arming fails on
    'Hardware safety switch'. Must be a mismatch, not ok."""
    wanted = cp.parse_parm(PARM_45)
    fc = _fc_ca_fisierul(BRD_SAFETY_DEFLT=1.0, BRD_SAFETYOPTION=3.0)
    (ok, mismatch, missing, _), out = _quiet(
        cp.verify_params, fc, wanted, PARM_45)
    gresite = {n: g for n, _w, g, _l in mismatch}
    assert gresite == {'BRD_SAFETY_DEFLT': 1.0, 'BRD_SAFETYOPTION': 3.0}, (
        gresite)
    assert 'NU SE POTRIVESTE' in out and 'BRD_SAFETYOPTION' in out
    # the write path fixes both, confirmed by read-back
    rez = cp.write_params(fc, wanted)
    assert set(fc.scrieri) == {'BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION'}
    assert all(g is not None and cp.close_enough(w, g) for _, w, g in rez)
    return "DEFLT 1 / OPTION 3 -> 2 nepotriviri; --write le corecteaza"


def _parm_cu_baud(tmp):
    cale = os.path.join(tmp, 'baud.parm')
    with open(cale, 'w') as f:
        f.write("SERIAL2_PROTOCOL,2\nSERIAL2_BAUD,57\nSERIAL2_OPTIONS,4\n"
                "BRD_SAFETY_DEFLT,0\n")
    return cale


def test_NEGATIV_garda_SERIALx_BAUD_OPTIONS():
    """User decision, 02.10.2026: SERIALx_BAUD / SERIALx_OPTIONS are set by
    hand. Even if a parm file asks for other values, check_params writes
    NOTHING to them and reports no mismatch for them."""
    tmp = tempfile.mkdtemp()
    cale = _parm_cu_baud(tmp)
    fc = FakeFC({'SERIAL2_PROTOCOL': 2.0, 'SERIAL2_BAUD': 921.0,
                 'SERIAL2_OPTIONS': 0.0, 'BRD_SAFETY_DEFLT': 1.0})
    wanted = cp.parse_parm(cale)
    rez = cp.write_params(fc, wanted)
    for n in ('SERIAL2_BAUD', 'SERIAL2_OPTIONS'):
        assert n not in fc.scrieri, f"PARAM_SET pe {n}: {fc.scrieri}"
        assert n not in {r[0] for r in rez}, rez
    assert fc.p['SERIAL2_BAUD'] == 921.0 and fc.p['SERIAL2_OPTIONS'] == 0.0
    assert fc.scrieri == ['BRD_SAFETY_DEFLT'], fc.scrieri

    (ok, mismatch, missing, manual), out = _quiet(
        cp.verify_params, fc, wanted, cale)
    assert not mismatch and not missing, (mismatch, missing)
    assert [m[0] for m in manual] == ['SERIAL2_BAUD', 'SERIAL2_OPTIONS']
    assert out.count('setat de mana, ignorat') == 2, out
    assert cp.is_hand_set('SERIAL12_BAUD') and not cp.is_hand_set(
        'SERIAL2_PROTOCOL') and not cp.is_hand_set('XSERIAL2_BAUD')
    return "57 / 4 cerute -> 0 PARAM_SET, 0 nepotriviri, 2 'setat de mana'"


def test_NEGATIV_garda_prin_CLI_write():
    """The same through main() - the path an operator and the preflight
    (subprocess) take - with --write --yes, then a plain read-back."""
    from pymavlink import mavutil
    tmp = tempfile.mkdtemp()
    cale = _parm_cu_baud(tmp)
    fc = FakeFC({'SERIAL2_PROTOCOL': 2.0, 'SERIAL2_BAUD': 921.0,
                 'SERIAL2_OPTIONS': 0.0, 'BRD_SAFETY_DEFLT': 1.0})
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
    assert 'SERIAL2_BAUD' not in fc.scrieri and \
        'SERIAL2_OPTIONS' not in fc.scrieri, fc.scrieri
    assert '2/2 scrisi si confirmati' in out_w, out_w
    assert '0 nepotriviri' in out_r and 'setat de mana, ignorat' in out_r
    # the preflight keeps only the summary: nothing hand-set shows as a diff
    import preflight_check as pf
    r = pf.rezumat_parametri(out_r)
    assert r.startswith('2 ok | 0 nepotriviri') and 'SERIAL2' not in r, r
    return f"--write: {fc.scrieri}; citire: {r}"


# --- (c) no 4.7+/4.8 parameter name in the vehicle-side code ----------------

#: Files whose PURPOSE is the 4.7+ names. Nothing else is exempt.
EXCEPTII = {
    'config/nova_flight.parm':
        'the 4.7+ source file; config/nova_flight_4.5.parm is generated from '
        'it and is what the 4.5.7 board loads (config/nova.json flight_parm)',
    'tools/make_parm_45.py':
        'the 4.7+ -> 4.5.7 translator: the 4.7+ names are its input table',
}

#: The 4.7+ group prefixes that do not exist on 4.5.7 at all. Without them
#: the prefix filter below would hide exactly those renames (master: the
#: GCS sysid moved from SYSID_MYGCS to MAV_GCS_SYSID).
EXTRA_PREFIXES = {'MAV'}

#: Tokens that pass every filter below but are not parameters. Keep it small
#: and say why for each entry.
NOT_PARAMS = {}

#: AP_MAX_NAME_SIZE, Copter-4.5.7 libraries/AP_Param/AP_Param.h:36.
MAX_NAME = 16

TOKEN = re.compile(r'(?<![A-Za-z0-9_\x00])[A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+'
                   r'(?![A-Za-z0-9_\x00])')


def nume_45():
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
    name (Band(..., PSC_POSXY_P=0.7))."""
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
      3. its first segment is a parameter group prefix of 4.5.7 (from the
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


def nume_din_extra_params(path):
    """tools/sitl_touchdown.py: only the EXTRA_PARAMS block ('NAME value'
    lines) - the rest of that file is SITL plumbing, not vehicle params."""
    tree = ast.parse(open(path).read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'EXTRA_PARAMS'
                for t in node.targets):
            out = {}
            for i, line in enumerate(node.value.value.splitlines()):
                if line.split():
                    out[line.split()[0]] = node.lineno + i
            return out
    raise AssertionError(f"EXTRA_PARAMS lipseste din {path}")


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
    out = [('config/nova_flight_4.5.parm', 'parm')]
    out += sorted((os.path.relpath(p, REPO), 'py')
                  for p in glob.glob(os.path.join(REPO, 'nova', '*.py')))
    out += [('tools/check_params.py', 'py'), ('tools/preflight_check.py', 'py'),
            ('tools/collect_session.py', 'py'), ('tools/nova_pi.py', 'py'),
            ('tools/sitl_touchdown.py', 'extra')]
    out += sorted((os.path.relpath(p, REPO), 'sh')
                  for p in glob.glob(os.path.join(REPO, 'pi', '*.sh')))
    return out


def _nume(rel, kind, known):
    path = os.path.join(REPO, rel)
    if kind == 'parm':
        return nume_din_parm(path)
    if kind == 'extra':
        return nume_din_extra_params(path)
    if kind == 'sh':
        return nume_din_shell(path, known)
    return nume_din_python(path, known)


def test_nicio_cheie_nu_e_nume_de_4_7():
    known = nume_45()
    assert len(known) > 4000, f"lista 4.5.7 are doar {len(known)} nume"
    surse = surse_vehicul()
    for rel, _ in surse:
        assert rel not in EXCEPTII, f"{rel} e si scanat, si exceptat"
    total, gresite = 0, []
    pe_fisier = {}
    for rel, kind in surse:
        nume = _nume(rel, kind, known)
        pe_fisier[rel] = nume
        total += len(nume)
        gresite += [f"{rel}:{line} {n}" for n, line in sorted(nume.items())
                    if n not in known]
    assert not gresite, (
        "nume care NU exista pe Copter-4.5.7 (pe placa: tacere, fara "
        "eroare):\n          " + "\n          ".join(gresite))
    # not vacuous: the scanner does see the names that matter
    assert {'PSC_POSXY_P', 'WPNAV_ACCEL', 'LAND_SPEED'} <= set(
        pe_fisier['nova/authority.py']), pe_fisier['nova/authority.py']
    assert len(pe_fisier['tools/collect_session.py']) >= 25
    assert {'BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION'} <= set(
        pe_fisier['config/nova_flight_4.5.parm'])
    assert 'WPNAV_SPEED_DN' in pe_fisier['tools/sitl_touchdown.py']
    assert {'FLTMODE_CH', 'ARMING_CHECK'} <= set(
        pe_fisier['pi/descent_test.sh']), pe_fisier['pi/descent_test.sh']
    return f"{total} nume in {len(surse)} fisiere, toate pe 4.5.7"


def test_exceptiile_chiar_sunt_4_7():
    """The two exempt files really are the 4.7+ side - and the scanner,
    pointed at them, finds the 4.7 names (a live negative case)."""
    known = nume_45()
    src = nume_din_parm(os.path.join(REPO, 'config', 'nova_flight.parm'))
    doar_47 = sorted(n for n in src if n not in known)
    assert {'WP_ACC', 'ARMING_SKIPCHK', 'LAND_SPD_MS'} <= set(doar_47), doar_47
    tr = nume_din_python(os.path.join(REPO, 'tools', 'make_parm_45.py'), known)
    assert 'WP_SPD_DN' in tr and 'WP_SPD_DN' not in known
    assert set(EXCEPTII) == {'config/nova_flight.parm', 'tools/make_parm_45.py'}
    return f"nova_flight.parm: {len(doar_47)} nume 4.7+ ({', '.join(doar_47)})"


def test_NEGATIV_nume_4_7_injectat_e_prins():
    """A 4.7 name injected in a copy of real code must be caught; prose,
    docstrings, MAVLink names and run-time built names must not."""
    known = nume_45()
    cod = open(os.path.join(REPO, 'nova', 'authority.py')).read()
    assert not [n for n in nume_parametri(_python_strings(cod), known)
                if n not in known]
    injectat = cod + "\nX = ('PSC_NE_POS_P', 'WP_ACC')\n" \
                     "Band('rea', 0.0, LAND_SPD_MS=0.5)\n"
    gasit = nume_parametri(_python_strings(injectat), known)
    rele = sorted(n for n in gasit if n not in known)
    assert rele == ['LAND_SPD_MS', 'PSC_NE_POS_P', 'WP_ACC'], rele

    # the same in a parm file copy (temp), and in sitl EXTRA_PARAMS form
    tmp = tempfile.mkdtemp()
    parm = os.path.join(tmp, 'x.parm')
    with open(parm, 'w') as f:
        f.write(open(PARM_45).read() + 'WP_SPD_DN,0.5\n')
    assert [n for n in nume_din_parm(parm) if n not in known] == ['WP_SPD_DN']
    extra = os.path.join(tmp, 'sitl.py')
    open(extra, 'w').write('EXTRA_PARAMS = """\nLAND_SPEED 50\n'
                           'RNGFND1_GNDCLR 0.07\n"""\n')
    assert [n for n in nume_din_extra_params(extra) if n not in known] == \
        ['RNGFND1_GNDCLR']

    # pi/*.sh: a copy of descent_test.sh with the old 4.7+ query put back
    dt = open(os.path.join(REPO, 'pi', 'descent_test.sh')).read()
    sh_rau = dt.replace("'ARMING_CHECK'):", "'ARMING_CHECK', 'ARMING_SKIPCHK'):")
    assert sh_rau != dt
    assert [n for n in nume_din_shell(None, known, sh_rau)
            if n not in known] == ['ARMING_SKIPCHK']
    # ...while its own shell variables and comments are not parameters
    assert nume_din_shell(None, known, 'LOG_DIR="$HOME/x"\necho "$LOG_DIR"\n'
                          '# WP_ACC in a comment\n') == {}

    # what must NOT count as a parameter name
    curat = ('"""Docstring: WP_ACC was the 4.8 name."""\n'
             "m = 'RC_CHANNELS'\nk = 'LOG_DATA'\nf = 'DESCEND_TRACK'\n"
             "n = f'EK3_SRC{s}_POSXY'\n# WP_ACC in a comment\n")
    assert nume_parametri(_python_strings(curat), known) == {}, (
        nume_parametri(_python_strings(curat), known))
    # a 4.7+ name from a group 4.5.7 does not have (EXTRA_PREFIXES)
    assert list(nume_parametri([(1, 'MAV_GCS_SYSID')], known)) == \
        ['MAV_GCS_SYSID'] and 'MAV_GCS_SYSID' not in known
    return "3 nume injectate prinse; docstring/comentariu/MAVLink/f-string nu"


TESTS = [
    ('setul X7+ in ambele fisiere', test_setul_X7_in_ambele_fisiere),
    ('fisierul generat e la zi', test_fisierul_generat_e_la_zi),
    ('NEGATIV: setul X7 lipsa e prins', test_NEGATIV_setul_X7_lipsa_e_prins),
    ('check_params descrie BRD_SAFETYOPTION',
     test_check_params_descrie_BRD_SAFETYOPTION),
    ('check_params: totul se potriveste', test_check_params_totul_se_potriveste),
    ('NEGATIV: BRD_SAFETY_DEFLT 1 e nepotrivire',
     test_NEGATIV_BRD_SAFETY_DEFLT_1_e_nepotrivire),
    ('NEGATIV: garda SERIALx_BAUD/OPTIONS',
     test_NEGATIV_garda_SERIALx_BAUD_OPTIONS),
    ('NEGATIV: garda prin CLI --write', test_NEGATIV_garda_prin_CLI_write),
    ('nicio cheie nu e nume de 4.7', test_nicio_cheie_nu_e_nume_de_4_7),
    ('exceptiile chiar sunt 4.7', test_exceptiile_chiar_sunt_4_7),
    ('NEGATIV: nume 4.7 injectat e prins',
     test_NEGATIV_nume_4_7_injectat_e_prins),
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
