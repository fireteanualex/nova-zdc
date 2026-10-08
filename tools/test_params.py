#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Teste minimale pentru config/nova_flight.parm (Cube Orange+, ArduCopter
4.6.2, de la 08.10.2026), fara FC.

    python3 tools/test_params.py

The parm file holds only what the FC needs to follow the Pi (ExtNav as the
EKF3 position source and the GNSS-free source set 2). Checked: the file is
exactly that set; check_params reads it back OK and flags a wrong value;
the hand-set names (serial ports, arming checks) are never written.
"""

import contextlib
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

#: The minimal set (team decision, 08.10.2026): the FC can follow the Pi.
MINIMAL = {'VISO_TYPE': 1.0, 'VISO_DELAY_MS': 100.0, 'VISO_POS_M_NSE': 0.5,
           'EK3_SRC2_POSXY': 6.0, 'EK3_SRC2_VELXY': 0.0, 'EK3_SRC2_POSZ': 1.0,
           'EK3_SRC2_VELZ': 0.0, 'EK3_SRC2_YAW': 1.0, 'EK3_SRC_OPTIONS': 0.0}


def _parm(path):
    return {n: v for n, v, _ in cp.parse_parm(path)}


def test_fisierul_e_setul_minimal():
    f = _parm(PARM)
    assert f == MINIMAL, f
    assert not [n for n in f if cp.is_hand_set(n) or n.startswith('FENCE_')]
    return f"{len(f)} parametri: VISO_* si setul EKF 2"


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
    assert len(ok) == len(wanted) and 'VISO_TYPE' in ok
    assert cp.write_params(fc, wanted) and not fc.scrieri, (
        f"a rescris parametri deja corecti: {fc.scrieri}")
    return f"{len(ok)}/{len(wanted)} ok, 0 scrieri"




def test_NEGATIV_valoare_gresita_e_nepotrivire():
    """EK3_SRC_OPTIONS 1 (the 4.6.2 default): GPS velocity on set 2."""
    wanted = cp.parse_parm(PARM)
    fc = _fc_ca_fisierul(EK3_SRC_OPTIONS=1.0)
    (ok, mismatch, missing, _), out = _quiet(
        cp.verify_params, fc, wanted, PARM)
    assert [m[0] for m in mismatch] == ['EK3_SRC_OPTIONS'], mismatch
    return "EK3_SRC_OPTIONS 1 -> nepotrivire"



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



TESTS = [
    ('fisierul e setul minimal', test_fisierul_e_setul_minimal),
    ('check_params: totul se potriveste', test_check_params_totul_se_potriveste),
    ('NEGATIV: valoare gresita e nepotrivire',
     test_NEGATIV_valoare_gresita_e_nepotrivire),
    ('NEGATIV: garda setati de mana', test_NEGATIV_garda_setati_de_mana),
    ('NEGATIV: garda prin CLI --write', test_NEGATIV_garda_prin_CLI_write),
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
