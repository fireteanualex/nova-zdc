#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru masina de stari ExtNav (nova/extnav_landing.py).

    python3 tools/test_extnav_landing.py

Fara FC si fara camera: vehiculul e clasa Vehicle din productie (fara
conexiune) peste care sta o fizica de mucava - EKF cu doua cadre (GPS
inainte de comutare, cadrul markerului dupa primul VPE), moduri adoptate cu
intarziere, pozitie care converge spre consemnul GUIDED, coborare in LAND.
Detectiile vin din ADEVAR (marker la pozitie cunoscuta), rarite la 2-5%,
asa cum cere brief-ul §9.3. Cablajul e cel din tools/nova_pi.py: supervizor
ExtNav inaintea masinii de stari, managerul de surse EKF din faza.
"""

import math
import os
import random
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil                               # noqa: E402

from nova import extnav as ex                               # noqa: E402
from nova.detection import Detection                        # noqa: E402
from nova.ekf_source import EkfSourceManager                # noqa: E402
from nova.extnav_landing import (ExtNavConfig, ExtNavLanding,  # noqa: E402
                                 Phase, SRC2_PHASES)
from nova.handover import HandoverGate                      # noqa: E402
from nova.rc import OverrideMonitor                         # noqa: E402
from nova.safety import Action, ExtNavSupervisor            # noqa: E402
from nova.vehicle import (MODE_ALT_HOLD, MODE_GUIDED, MODE_LAND,  # noqa: E402
                          MODE_LOITER, MODE_STABILIZE, Vehicle)

AUX = 8
GPS_OFFSET = (20.0, -5.0)          # cadrul GPS fata de cadrul markerului
LANDED_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
LANDED_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR


class SimVeh(Vehicle):
    """Vehicle real + fizica de mucava. Adevarul: pozitia in cadrul
    MARKERULUI (`p`), altitudinea `h`, yaw-ul `yaw_true`."""

    def __init__(self, h=8.0, p=(2.0, -1.0), yaw=0.3):
        super().__init__('udpin:127.0.0.1:1')
        self.link_verbose = False
        self.link_healthy = True
        self.p = [float(p[0]), float(p[1])]
        self.h = float(h)
        self.yaw_true = float(yaw)
        self.vel = [0.0, 0.0, 0.0]
        self.src = 1
        self.bias = None                 # EKF: marker = truth + bias
        self.target = None               # (x, y, z, yaw) din SET_POSITION_TARGET
        self.armed = True
        self.mode = MODE_LOITER
        self.mode_delay_s = 0.15
        self.refuse_modes = ()
        self.mode_pending = []
        self.mode_reqs = []
        self.ekf_valid = True
        self.ekf_valid_after_switch = True
        self.src_cmds = []
        self.statustexts = []
        self.vpe = []
        self.targets = []
        self.disarm_on_ground = True
        self.params = {'EK3_SRC2_POSXY': 6, 'EK3_SRC2_VELXY': 0,
                       'EK3_SRC2_POSZ': 1, 'EK3_SRC2_VELZ': 0,
                       'EK3_SRC2_YAW': 1, 'EK3_SRC_OPTIONS': 0,
                       'RC1_TRIM': 1500, 'RC2_TRIM': 1500, 'RC3_TRIM': 1100,
                       'RC4_TRIM': 1500}
        self.rc = (1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000)
        self.rc_t = 0.0
        self.landed_state = LANDED_IN_AIR
        self.ground_t = None
        self.time_boot_ms = 1000
        self.now = 0.0
        outer = self

        class _Mav:
            def statustext_send(self, sev, text):
                outer.statustexts.append(text.decode('ascii', 'replace'))

            def vision_position_estimate_send(self, *a):
                outer.vpe.append(a)
                if outer.src == 2:
                    # EKF-ul preia pozitia noastra: cadrul markerului
                    outer.bias = [a[1] - outer.p[0], a[2] - outer.p[1]]

            def set_position_target_local_ned_send(self, *a):
                outer.targets.append(a)
                outer.target = (a[5], a[6], a[7], a[14])

            def command_long_send(self, *a):
                pass

            def param_request_read_send(self, *a):
                pass

            def param_set_send(self, *a):
                pass

        self.m = types.SimpleNamespace(mav=_Mav(), target_system=1,
                                       target_component=1)
        self._publish(0.0)

    # -- ce vede companion-ul ----------------------------------------------
    def _publish(self, now):
        if self.src == 1 or self.bias is None:
            x = self.p[0] + GPS_OFFSET[0]
            y = self.p[1] + GPS_OFFSET[1]
        else:
            x = self.p[0] + self.bias[0]
            y = self.p[1] + self.bias[1]
        self._on_position(now, x, y, -self.h, self.vel[0], self.vel[1],
                          self.vel[2])
        self._on_attitude(now, 0.0, 0.0, self.yaw_true)
        self.rel_alt = self.h
        self.rc_t = now
        self.hb_t = now                  # legatura vie (monitorul de link)
        valid = self.ekf_valid if self.src == 1 else self.ekf_valid_after_switch
        self.ekf_flags = (mavutil.mavlink.EKF_ATTITUDE
                          | (mavutil.mavlink.EKF_POS_HORIZ_REL if valid else 0))
        self.ekf_t = now

    def ekf_pos_horiz_ok(self):
        return super().ekf_pos_horiz_ok()

    def on_ground(self):
        return self.landed_state == LANDED_ON_GROUND

    def request_param(self, name):
        return True                      # valorile sunt deja in params

    def send_ekf_source_set(self, n):
        self.src_cmds.append(n)
        self.ekf_src_ack = mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.src = n
        if n == 1:
            self.bias = None
        return True

    def request_mode(self, m):
        self.mode_reqs.append(m)
        self.last_mode_req = m
        if m in self.refuse_modes:
            return True
        self.mode_pending.append((self.now + self.mode_delay_s, m))
        return True

    def set_aux(self, pwm):
        rc = list(self.rc)
        rc[AUX - 1] = pwm
        self.rc = tuple(rc)

    # -- fizica ------------------------------------------------------------
    def step(self, now, dt):
        self.now = now
        while self.mode_pending and self.mode_pending[0][0] <= now:
            self.mode = self.mode_pending.pop(0)[1]
        if self.mode == MODE_GUIDED and self.target is not None:
            tx, ty, tz, tyaw = self.target
            # consemnul e in cadrul EKF; il traducem in adevar prin bias
            bx, by = self.bias if self.bias is not None else GPS_OFFSET
            gx, gy, gh = tx - bx, ty - by, -tz
            for i, (cur, goal) in enumerate(((self.p[0], gx), (self.p[1], gy))):
                v = max(-1.0, min(1.0, 1.2 * (goal - cur)))
                self.vel[i] = v
                self.p[i] = cur + v * dt
            vz = max(-0.5, min(0.5, 1.0 * (gh - self.h)))
            self.h += vz * dt
            self.vel[2] = -vz
            d = ex.wrap_pi(tyaw - self.yaw_true)
            self.yaw_true = ex.wrap_pi(self.yaw_true + max(-0.8, min(0.8, 2.0 * d)) * dt)
        elif self.mode == MODE_LAND:
            self.vel = [0.0, 0.0, 0.5]
            self.h = max(0.0, self.h - 0.5 * dt)
            if self.h <= 0.02:
                # land_complete, apoi dezarmarea dupa rampa de spool-down
                # (~0.5 s, §5.6) - exact fereastra in care se vede ON_GROUND
                if self.landed_state != LANDED_ON_GROUND:
                    self.landed_state = LANDED_ON_GROUND
                    self.ground_t = now
                elif self.disarm_on_ground and now - self.ground_t >= 0.5:
                    self.armed = False
        else:
            self.vel = [0.0, 0.0, 0.0]
        self._publish(now)


class TruthDetector:
    """Detectii din adevar, la `rate` din cadre (10 cadre/s), cu latenta."""

    def __init__(self, veh, rate=0.05, latency_s=0.08, seed=1, fps=10.0):
        self.v = veh
        self.rate = rate
        self.latency = latency_s
        self.rng = random.Random(seed)
        self.period = 1.0 / fps
        self.next_t = None
        self.n_frames = 0
        self.n_dets = 0
        self.enabled = True

    def poll(self, now):
        if self.next_t is None:
            self.next_t = now
        out = []
        while now >= self.next_t:
            self.next_t += self.period
            self.n_frames += 1
            if not self.enabled or self.rng.random() >= self.rate:
                continue
            t_c = now - self.latency
            d = (-self.v.p[0], -self.v.p[1], self.v.h)      # spre marker, NED
            b = ex.ned_to_body(d, 0.0, 0.0, self.v.yaw_true)
            if b[2] <= 0.05:
                continue
            self.n_dets += 1
            out.append(Detection(
                t=t_c, angle_x=math.atan2(b[0], b[2]),
                angle_y=math.atan2(b[1], b[2]),
                distance_m=math.sqrt(sum(c * c for c in d)),
                marker_px=100.0, range_m=self.v.h, fill=None,
                marker_yaw_deg=math.degrees(-self.v.yaw_true)))
        return out


class App:
    """Cablajul de bord (ordinea din run_loop), pe ceas injectat."""

    def __init__(self, h=8.0, p=(2.0, -1.0), rate=0.05, seed=1, yaw=0.3,
                 cfg=None):
        self.v = SimVeh(h=h, p=p, yaw=yaw)
        self.det = TruthDetector(self.v, rate=rate, seed=seed)
        self.est = ex.ExtNavEstimator(self.v)
        self.ekf = EkfSourceManager(self.v, verbose=False, faze=SRC2_PHASES)
        self.ov = OverrideMonitor(self.v)
        self.gate = HandoverGate(self.v, self.ov, autonomy_enabled=True,
                                 alt_min_m=1.0, alt_max_m=12.0,
                                 dist_max_m=None, detection_max_age_s=None)
        self.events = []
        self.sm = ExtNavLanding(self.v, self.est, self.ekf, self.gate,
                                cfg or ExtNavConfig(aux_channel=AUX),
                                on_event=lambda n, i: self.events.append((n, i)),
                                verbose=False)
        self.sup = ExtNavSupervisor(self.v, verbose=False, override=self.ov,
                                    on_exit=self.sm.request_exit)
        self.t = 0.0
        self.states = [self.sm.state]
        self.h_targets = []
        self.v.step(0.0, 0.0)

    def run(self, seconds, dt=0.02, stop=(), on_step=None):
        end = self.t + seconds
        while self.t < end:
            self.t += dt
            now = 1000.0 + self.t
            self.v.step(now, dt)
            self.sup.update(now, None, self.sm.state)
            for d in self.det.poll(now):
                self.sm.on_detection(d, now)
            self.sm.update(now)
            if self.states[-1] != self.sm.state:
                self.states.append(self.sm.state)
                if self.sm.state == Phase.DESCEND:
                    self.h_targets.append(round(self.sm.h_target, 2))
            if on_step:
                on_step(self)
            if self.sm.state in stop:
                break
        return self.sm.state

    def ev(self, name):
        return [i for n, i in self.events if n == name]

    def handover(self):
        """AUX jos apoi sus, cu o pauza ca prima citire sa fie referinta."""
        self.v.set_aux(1000)
        self.run(0.3)
        self.v.set_aux(2000)


# --- teste -----------------------------------------------------------------

def test_secventa_completa_cu_detectii_rare():
    """Brief §9.3: secventa completa cu detectii la 5%. De la 8 m: trepte
    8 -> 4 -> 2 -> 1, captura de scoring la 1 m, LAND vertical, TOUCHDOWN.
    Fara LANDING_TARGET, fara PLND, VPE doar dupa ENGAGE, SRC 2 -> 1 la
    dezarmare."""
    app = App(h=8.0, p=(2.0, -1.0), rate=0.05)
    app.handover()
    st = app.run(120.0, stop=(Phase.TOUCHDOWN, Phase.ABORT, Phase.GATE_FAIL))
    assert st == Phase.TOUCHDOWN, (st, app.states, app.sm.exit_reason)
    seq = [s for s in app.states if s not in (Phase.CENTER_CHECK, Phase.MOVE)]
    assert seq[:3] == [Phase.IDLE, Phase.GATE_SEARCH, Phase.ENGAGE], app.states
    assert seq[-3:] == [Phase.FINAL_ALIGN, Phase.LAND, Phase.TOUCHDOWN], app.states
    assert app.h_targets == [4.0, 2.0, 1.0], app.h_targets
    assert app.v.n_lt == 0 and app.v.n_ds == 0, "a trimis LANDING_TARGET / DISTANCE_SENSOR"
    assert not any(n == 'PLND_ENABLED' and val != 0 for n, val in
                   [(k, v_) for k, v_ in app.v.params.items()])
    # VPE doar din ENGAGE: primul VPE dupa comanda de comutare pe setul 2
    assert app.v.src_cmds[0] == 2 and app.v.vpe, (app.v.src_cmds, len(app.v.vpe))
    # eroarea finala la contact
    err = math.hypot(*app.v.p)
    assert err < ex.tol_m(1.0), f"eroare la contact {err:.2f} m"
    sc = app.ev('scoring_capture')
    assert len(sc) == 1 and abs(sc[0]['alt'] - 1.0) <= 0.2, sc
    td = app.ev('touchdown')
    assert len(td) == 1
    # dezarmarea de pe sol readuce setul 1 si masina in IDLE
    app.run(1.0)
    assert app.sm.state == Phase.IDLE and app.v.src_cmds[-1] == 1, (
        app.sm.state, app.v.src_cmds)
    # yaw aliniat: markerul "sus" spre nord, vehiculul a ajuns la yaw ~0
    assert abs(math.degrees(app.v.yaw_true)) < 6.0, math.degrees(app.v.yaw_true)
    return (f"{app.det.n_dets} detectii din {app.det.n_frames} cadre "
            f"({100.0 * app.det.n_dets / app.det.n_frames:.1f}%), trepte "
            f"{app.h_targets}, contact la {err * 100:.1f} cm, {len(app.v.vpe)} VPE")


def test_poarta_fara_detectii_reincearca_apoi_esueaza_fara_comenzi():
    """0 detectii -> un retry (inca 5 s) -> GATE_FAIL, pilotul ramane in
    LOITER, STATUSTEXT. Companion-ul nu a comandat NIMIC."""
    app = App(rate=0.0)
    app.handover()
    st = app.run(20.0, stop=(Phase.GATE_FAIL, Phase.ENGAGE))
    assert st == Phase.GATE_FAIL, app.states
    t_fail = app.t
    assert 9.0 < t_fail < 12.0, f"GATE_FAIL la {t_fail:.1f} s (asteptat ~10 s)"
    assert app.v.mode_reqs == [] and app.v.src_cmds == [] and not app.v.vpe
    assert app.v.mode == MODE_LOITER
    assert any('marker negasit' in s for s in app.v.statustexts), app.v.statustexts
    assert app.ev('gate_fail')
    # AUX jos apoi sus: cautare noua
    app.handover()
    app.run(0.5)
    assert app.sm.state == Phase.GATE_SEARCH, app.sm.state
    return f"GATE_FAIL la {t_fail:.1f} s, 0 comenzi, STATUSTEXT, cautare noua pe front"


def test_o_singura_detectie_porneste_segmentul():
    """Decizia echipei, 27.09.2026 (zborul b29: patru cereri la 3.5 m fara
    pereche). O singura estimare in fereastra deschide segmentul; nu
    asteapta a doua. Si fara ea, nimic - nici mai devreme de fereastra de
    asezare a portii (1 s, manse/E0/altitudine)."""
    app = App(rate=0.0)
    app.handover()
    app.run(1.5)                                   # poarta asezata, 0 detectii
    assert app.sm.state == Phase.GATE_SEARCH
    app.det.rate = 1.0                             # un singur cadru cu marker
    app.run(0.1)
    app.det.rate = 0.0
    assert app.det.n_dets == 1, app.det.n_dets
    app.run(0.5, stop=(Phase.ENGAGE,))
    assert app.sm.state == Phase.ENGAGE, app.states
    e = app.ev('engage')[0]
    assert e['n'] == 1 and e['pair_dt'] is None, e
    return "o estimare dupa asezarea portii -> ENGAGE"


def test_regula_de_doua_detectii_ramane_optiune():
    """gate_detections=2 pastreaza regula initiala (brief D7): o singura
    detectie nu ajunge, doua consistente da."""
    app = App(rate=0.0, cfg=ExtNavConfig(aux_channel=AUX, gate_detections=2))
    app.handover()
    app.run(1.5)
    app.det.rate = 1.0
    app.run(0.1)
    app.det.rate = 0.0
    app.run(1.0, stop=(Phase.ENGAGE,))
    assert app.sm.state == Phase.GATE_SEARCH, "a pornit pe o singura detectie"
    app.det.rate = 1.0
    app.run(0.1)
    app.det.rate = 0.0
    app.run(0.5, stop=(Phase.ENGAGE,))
    assert app.sm.state == Phase.ENGAGE and app.ev('engage')[0]['n'] == 2
    return "cu 2: una nu ajunge, doua consistente da"


def test_poarta_refuza_altitudinea_in_afara_1_12_m():
    for h, ok in ((0.6, False), (13.0, False), (1.5, True), (11.5, True)):
        app = App(h=h, rate=0.5)
        app.handover()
        st = app.run(3.0, stop=(Phase.REJECT, Phase.ENGAGE))
        if ok:
            assert st == Phase.ENGAGE, (h, app.states)
        else:
            assert st == Phase.REJECT, (h, app.states)
            assert app.v.mode_reqs == [] and app.v.src_cmds == []
            assert app.ev('handover_reject') and 'altitudine' in app.ev('handover_reject')[0]['reason']
    return "0.6 si 13 m refuzate; 1.5 si 11.5 m acceptate"


def test_AUX_jos_in_coborare_iese_ordonat_SRC1_apoi_LOITER():
    """EXIT are ordine obligatorie: SRC1 (cu ACK) INAINTE de LOITER.
    Niciodata RTL. Apoi ABORT, si o cerere noua doar pe front."""
    app = App(rate=0.2)
    app.handover()
    st = app.run(60.0, stop=(Phase.DESCEND, Phase.ABORT))
    assert st == Phase.DESCEND, app.states
    n_src = len(app.v.src_cmds)
    n_mode = len(app.v.mode_reqs)
    app.v.set_aux(1000)
    app.run(3.0, stop=(Phase.ABORT,))
    assert app.sm.state == Phase.ABORT, app.sm.state
    assert app.v.src_cmds[n_src:] == [1], app.v.src_cmds
    assert app.v.mode_reqs[n_mode:] == [MODE_LOITER], app.v.mode_reqs[n_mode:]
    assert app.v.mode == MODE_LOITER and 6 not in app.v.mode_reqs
    # SRC1 a fost cerut INAINTE de LOITER (ordinea in evenimentele EKF)
    ex_ev = app.ev('exit')
    assert ex_ev and 'AUX jos' in ex_ev[0]['reason'] and not ex_ev[0]['passive']
    assert app.ev('exit_done')[0]['mode'] == MODE_LOITER
    assert any('EXIT' in s for s in app.v.statustexts)
    return "AUX jos in DESCEND: SRC1 -> LOITER -> ABORT; fara RTL"


def test_modul_pus_de_altcineva_da_EXIT_pasiv():
    """Regula de mod (brief §6): pilotul pune STABILIZE in MOVE. SRC1 se
    restaureaza, dar NICIO comanda de mod nu pleaca de la noi."""
    app = App(rate=0.2)
    app.handover()
    st = app.run(60.0, stop=(Phase.MOVE, Phase.ABORT))
    assert st == Phase.MOVE, app.states
    n_mode = len(app.v.mode_reqs)
    app.v.mode = MODE_STABILIZE
    app.run(3.0, stop=(Phase.ABORT,))
    assert app.sm.state == Phase.ABORT, app.sm.state
    assert app.v.src_cmds[-1] == 1
    assert len(app.v.mode_reqs) == n_mode, f"a comandat peste pilot: {app.v.mode_reqs[n_mode:]}"
    e = app.ev('exit')[0]
    assert e['passive'] and 'altcineva' in e['reason'], e
    # si in ENGAGE, in timp ce asteptam GUIDED: al treilea mod = pasiv
    app = App(rate=0.3)
    app.v.mode_delay_s = 1.0
    app.handover()
    marks = {}

    def on_step(a):
        if a.sm.state == Phase.ENGAGE and a.sm._sub == 'mode' and 'n' not in marks:
            marks['n'] = len(a.v.mode_reqs)
            a.v.mode = MODE_ALT_HOLD          # pilotul, in fereastra noastra
            a.v.mode_pending = []

    app.run(30.0, stop=(Phase.ABORT, Phase.MOVE), on_step=on_step)
    assert 'n' in marks, "scenariul nu s-a produs"
    assert app.sm.state == Phase.ABORT and app.ev('exit')[0]['passive']
    assert len(app.v.mode_reqs) == marks['n'], app.v.mode_reqs[marks['n']:]
    return "STABILIZE in MOVE si ALT_HOLD in ENGAGE: EXIT pasiv, SRC1, 0 moduri"


def test_supervizorul_cere_EXIT_la_EKF_invalid():
    app = App(rate=0.2)
    app.handover()
    st = app.run(60.0, stop=(Phase.MOVE, Phase.ABORT))
    assert st == Phase.MOVE
    app.v.ekf_valid_after_switch = False
    app.run(5.0, stop=(Phase.ABORT,))
    assert app.sm.state == Phase.ABORT, app.sm.state
    assert app.sup.latched == Action.EXIT and app.sup.latched_monitor == 'ekf_position'
    assert 'ekf_position' in app.ev('exit')[0]['reason']
    assert app.v.src_cmds[-1] == 1 and app.v.mode_reqs[-1] == MODE_LOITER
    return "EKF invalid in MOVE -> supervizor EXIT -> SRC1 -> LOITER"


def test_LOITER_refuzat_cade_pe_ALT_HOLD():
    app = App(rate=0.2)
    app.v.refuse_modes = (MODE_LOITER,)
    app.handover()
    st = app.run(60.0, stop=(Phase.MOVE, Phase.ABORT))
    assert st == Phase.MOVE
    app.v.set_aux(1000)
    app.run(6.0, stop=(Phase.ABORT,))
    assert app.sm.state == Phase.ABORT and app.v.mode == MODE_ALT_HOLD, (
        app.sm.state, app.v.mode)
    assert app.v.mode_reqs.count(MODE_LOITER) == 5 and app.v.mode_reqs[-1] == MODE_ALT_HOLD
    assert app.ev('exit_fallback')
    return "LOITER x5 refuzat -> ALT_HOLD confirmat"


def test_fara_detectie_in_centrare_reincearca_apoi_iese():
    app = App(rate=0.3)
    app.handover()
    st = app.run(60.0, stop=(Phase.CENTER_CHECK, Phase.ABORT))
    assert st == Phase.CENTER_CHECK, app.states
    app.det.enabled = False
    t0 = app.t
    app.run(20.0, stop=(Phase.ABORT,))
    assert app.sm.state == Phase.ABORT, app.sm.state
    assert 9.0 < app.t - t0 < 12.0, app.t - t0
    assert 'centrare' in app.ev('exit')[0]['reason']
    return f"2 ferestre fara detectie ({app.t - t0:.1f} s) -> EXIT"


def test_ENGAGE_fara_EKF_valid_expira_si_iese():
    app = App(rate=0.3)
    app.v.ekf_valid_after_switch = False
    app.handover()
    st = app.run(30.0, stop=(Phase.ABORT, Phase.MOVE))
    assert st == Phase.ABORT, app.states
    assert 'ENGAGE' in app.ev('exit')[0]['reason']
    assert MODE_GUIDED not in app.v.mode_reqs, app.v.mode_reqs
    assert app.v.src_cmds == [2, 1], app.v.src_cmds
    return "EKF nevalid dupa comutare: fara GUIDED, SRC1 inapoi, EXIT"


def test_setul_EKF_cu_GNSS_nu_se_comuta_si_se_iese():
    app = App(rate=0.3)
    app.v.params['EK3_SRC_OPTIONS'] = 1          # FuseAllVelocities
    app.handover()
    st = app.run(30.0, stop=(Phase.ABORT, Phase.MOVE))
    assert st == Phase.ABORT and app.v.src_cmds == [], (st, app.v.src_cmds)
    assert 'refuzat' in app.ev('exit')[0]['reason']
    return "EK3_SRC_OPTIONS bit 0: nicio comutare, EXIT"


def test_centrarea_cere_detectie_proaspata_nu_EKF():
    """Brief §3: EKF-ul poate crede ca e centrat doar din deriva. In
    CENTER_CHECK, offset-ul masurat peste tol(h) trimite inapoi in MOVE."""
    app = App(h=6.0, p=(0.5, 0.0), rate=0.3)
    app.handover()
    st = app.run(60.0, stop=(Phase.CENTER_CHECK, Phase.ABORT))
    assert st == Phase.CENTER_CHECK
    # EKF-ul deriveaza: adevarul se muta 1.5 m fara ca EKF-ul sa vada
    app.v.p[0] += 1.5
    app.v.bias[0] -= 1.5
    app.run(6.0, stop=(Phase.MOVE, Phase.DESCEND, Phase.ABORT))
    assert app.sm.state == Phase.MOVE, app.states[-4:]
    return "deriva de 1.5 m prinsa de detectia proaspata: inapoi in MOVE"


def test_a_doua_incercare_din_acelasi_zbor_se_angajeaza():
    """Defect 27.09.2026: dupa un EXIT, EkfSourceManager ramanea RESTAURAT
    si a doua incercare expira in ENGAGE (8 s, pas src2) - dupa orice EXIT,
    pilotul nu mai putea reporni segmentul in acelasi zbor. Doua incercari
    consecutive: EXIT cerut de pilot in DESCEND (SRC1, LOITER, ABORT), apoi
    AUX jos si sus -> ENGAGE reusit: setul recitit, sursa comutata a doua
    oara, primul VPE trimis din nou, coborare pana la DESCEND."""
    app = App(rate=0.2)
    app.handover()
    assert app.run(60.0, stop=(Phase.DESCEND, Phase.ABORT)) == Phase.DESCEND, app.states
    app.v.set_aux(1000)
    assert app.run(5.0, stop=(Phase.ABORT,)) == Phase.ABORT, app.states
    assert app.v.src_cmds == [2, 1], app.v.src_cmds
    assert app.ekf.state == app.ekf.RESTAURAT and app.v.bias is None
    n_vpe, n_sent = len(app.v.vpe), app.sm.n_vpe_sent
    app.run(0.3)                          # AUX jos: referinta pentru front
    app.v.set_aux(2000)
    st = app.run(60.0, stop=(Phase.DESCEND, Phase.ABORT, Phase.GATE_FAIL))
    assert st == Phase.DESCEND, (st, app.states, app.sm.exit_reason)
    assert app.v.src_cmds == [2, 1, 2], app.v.src_cmds
    assert app.ekf.state == app.ekf.ACTIV and app.ekf.conform
    # primul VPE al incercarii a doua a plecat DUPA a doua comutare: FC-ul
    # de mucava reancoreaza cadrul (bias) doar cand primeste VPE pe setul 2
    assert len(app.v.vpe) > n_vpe and app.sm.n_vpe_sent > n_sent
    assert app.v.bias is not None
    assert app.states.count(Phase.ENGAGE) == 2 and app.states.count(Phase.DESCEND) == 2
    return (f"EXIT apoi ENGAGE reusit: surse {app.v.src_cmds}, "
            f"VPE {n_vpe} -> {len(app.v.vpe)}")


TESTS = [
    ('secventa completa cu detectii la 5%', test_secventa_completa_cu_detectii_rare),
    ('poarta: fara detectii -> retry -> GATE_FAIL, 0 comenzi',
     test_poarta_fara_detectii_reincearca_apoi_esueaza_fara_comenzi),
    ('poarta: o singura detectie porneste segmentul', test_o_singura_detectie_porneste_segmentul),
    ('poarta: regula de doua ramane optiune', test_regula_de_doua_detectii_ramane_optiune),
    ('poarta: altitudinea 1-12 m', test_poarta_refuza_altitudinea_in_afara_1_12_m),
    ('AUX jos: SRC1 apoi LOITER', test_AUX_jos_in_coborare_iese_ordonat_SRC1_apoi_LOITER),
    ('mod pus de altcineva: EXIT pasiv', test_modul_pus_de_altcineva_da_EXIT_pasiv),
    ('supervizor: EKF invalid -> EXIT', test_supervizorul_cere_EXIT_la_EKF_invalid),
    ('LOITER refuzat -> ALT_HOLD', test_LOITER_refuzat_cade_pe_ALT_HOLD),
    ('centrare fara detectie -> retry -> EXIT',
     test_fara_detectie_in_centrare_reincearca_apoi_iese),
    ('ENGAGE fara EKF valid -> EXIT', test_ENGAGE_fara_EKF_valid_expira_si_iese),
    ('set EKF cu GNSS: nu se comuta', test_setul_EKF_cu_GNSS_nu_se_comuta_si_se_iese),
    ('centrarea cere detectie proaspata', test_centrarea_cere_detectie_proaspata_nu_EKF),
    ('a doua incercare din acelasi zbor se angajeaza',
     test_a_doua_incercare_din_acelasi_zbor_se_angajeaza),
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
            import traceback
            traceback.print_exc()
            print(f"  EROARE {name}\n        {type(e).__name__}: {e}")
    print(f"\n  {len(TESTS) - fails}/{len(TESTS)} teste trecute")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
