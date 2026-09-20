#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Declansarea portii de handover in SITL, fara gamepad (I0).

    python3 tools/sim_handover.py                    # interactiv: Enter = AUX sus
    python3 tools/sim_handover.py --after 3.0        # AUX sus dupa 3 s
    python3 tools/sim_handover.py --after 3 --hold 25 --then-low

De ce exista. Din runda 3, poarta de handover e SINGURA cale catre segmentul
autonom (§8): `mode land` comandat direct nu mai porneste secventa, iar
`PLND_ENABLED` e 0 pana cand poarta ACCEPTA. Deci un test automat nu poate
intra in secventa fara sa comute AUX 7 - adica fara gamepad, sau fara scriptul
asta.

CE TREBUIE SA FACA BINE, SI DE CE

**Manetele stau NEMISCATE.** Poarta refuza daca vreo mansa e in afara
neutrului, iar dupa ACCEPT monitorul de override memoreaza pozitia curenta ca
referinta. Orice tremur pe care l-am injecta ar deveni fie un refuz, fie un
override fals imediat dupa activare - 10 puncte pierdute pentru un artefact
al uneltei de test.

**Throttle-ul NU se trimite la RC3_TRIM.** Roll, pitch si yaw se
auto-centreaza, deci "liber" inseamna "la trim". Throttle-ul nu: pe un
emitator real e la MIJLOCUL cursei pentru hover, iar `RC3_TRIM` e adesea la
capatul de jos (1100 in configuratia noastra). Injectat la trim intr-un
vehicul care planeaza in LOITER, ar comanda o coborare rapida. Trimitem
mijlocul lui RC3_MIN..RC3_MAX. Poarta oricum nu compara throttle-ul cu trim,
ci ii masoara AMPLITUDINEA pe fereastra de asezare (§8).

**Canalul de mod nu se atinge**, implicit. Trimitem UINT16_MAX pe el, ceea ce
in ArduPilot inseamna "nu schimba starea de override a acestui canal" - deci
`mode guided` / `takeoff` din MAVProxy raman functionale. Cu `--mode-pwm` se
poate forta, daca chiar vrei.

**Precontitie tacuta: `MAV_GCS_SYSID`.** `GCS_MAVLINK::handle_rc_channels_
override` respinge TACIT orice `RC_CHANNELS_OVERRIDE` venit de la alt sysid
decat cel din `MAV_GCS_SYSID`. Fara potrivire, scriptul pare ca merge -
trimite fara eroare - si poarta nu vede niciodata AUX-ul. Verificam la
pornire si refuzam cu mesaj, in loc sa lasam testul sa esueze inexplicabil
(acelasi tipar ca §5.10: nu presupune ca s-a aplicat).

**Override-ul expira.** `RC_OVERRIDE_TIME` (implicit 3 s) sterge override-ul
daca nu mai trimitem. De aceea bucla merge la RATE_HZ pana la Ctrl-C, si la
iesire elibereaza explicit canalele.
"""

import argparse
import sys
import time

from pymavlink import mavutil

#: Ca in tools/gamepad_rc.py: destul de des cat override-ul sa nu expire.
RATE_HZ = 25

#: §8: canalul 7, prag 1700. Trimitem 2000/1000, clar de o parte si de alta.
AUX_CHANNEL = 7
AUX_HIGH_PWM = 2000
AUX_LOW_PWM = 1000

#: Canalul de mod din config/nova_sitl.parm (FLTMODE_CH).
MODE_CHANNEL = 5

#: UINT16_MAX = "ignora acest canal". 0 ar ELIBERA override-ul, ceea ce e
#: altceva: ar lasa canalul pe RC-ul simulat al SITL-ului, iar o schimbare
#: de valoare pe canalul de mod poate comuta modul de zbor.
IGNORE = 65535

TRIM_PARAMS = ('RC1_TRIM', 'RC2_TRIM', 'RC3_TRIM', 'RC4_TRIM')
THROTTLE_RANGE_PARAMS = ('RC3_MIN', 'RC3_MAX')


def get_param(m, name, timeout=3.0, tries=3):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode('ascii'), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('ascii', 'ignore')
            if got.rstrip('\x00') == name:
                return msg.param_value
    return None


def check_gcs_sysid(m, our_sysid, printer=print):
    """(ok, mesaj). Precontitia tacuta din docstring."""
    val = get_param(m, 'MAV_GCS_SYSID')
    if val is None:
        return True, ('MAV_GCS_SYSID nu a raspuns; continui, dar daca poarta '
                      'nu vede AUX-ul, asta e primul lucru de verificat')
    if int(val) != int(our_sysid):
        return False, (
            f"MAV_GCS_SYSID = {int(val)}, iar noi trimitem de pe sysid "
            f"{our_sysid}.\n"
            f"  ArduPilot RESPINGE TACIT RC_CHANNELS_OVERRIDE de la alt "
            f"sysid: scriptul ar parea ca merge si poarta nu ar vedea nimic.\n"
            f"  Reporneste cu --source-system {int(val)}, sau seteaza "
            f"MAV_GCS_SYSID={our_sysid} pe FC.")
    return True, f"MAV_GCS_SYSID = {int(val)}, se potriveste"


def neutral_channels(m, printer=print):
    """Cele 8 valori de trimis, cu manetele in neutru.

    Roll/pitch/yaw la RCx_TRIM citit de pe FC; throttle la MIJLOCUL cursei,
    nu la trim - vezi docstring-ul modulului."""
    trims = {}
    for name in TRIM_PARAMS:
        trims[name] = get_param(m, name)
    lipsa = [n for n, v in trims.items() if v is None]
    if lipsa:
        printer(f"  ATENTIE: {', '.join(lipsa)} nu au raspuns; folosesc 1500")

    def trim(name):
        v = trims.get(name)
        return 1500 if v is None else int(v)

    rc3_min = get_param(m, 'RC3_MIN')
    rc3_max = get_param(m, 'RC3_MAX')
    if rc3_min is None or rc3_max is None:
        thr = 1500
        printer("  ATENTIE: RC3_MIN/RC3_MAX nu au raspuns; throttle la 1500")
    else:
        thr = int(round((float(rc3_min) + float(rc3_max)) / 2.0))

    ch = [IGNORE] * 8
    ch[0] = trim('RC1_TRIM')
    ch[1] = trim('RC2_TRIM')
    ch[2] = thr
    ch[3] = trim('RC4_TRIM')
    ch[AUX_CHANNEL - 1] = AUX_LOW_PWM
    return ch, {'trim': {k: (None if v is None else int(v))
                         for k, v in trims.items()},
                'throttle_mid': thr,
                'rc3_min': None if rc3_min is None else int(rc3_min),
                'rc3_max': None if rc3_max is None else int(rc3_max)}


def send(m, ch):
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *ch)


def release(m, n=5):
    """Elibereaza override-ul pe toate canalele. 0 = eliberare."""
    for _ in range(n):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.05)


def drain_statustext(m, printer=print, prefix='  [FC] '):
    """Afiseaza STATUSTEXT-urile: acolo apare verdictul portii.

    Aplicatia trimite `NOVA handover refuzat: <motiv>` prin STATUSTEXT, deci
    scriptul nu trebuie sa ghiceasca ce s-a intamplat."""
    vazute = []
    while True:
        msg = m.recv_match(type='STATUSTEXT', blocking=False)
        if msg is None:
            return vazute
        txt = msg.text
        if isinstance(txt, bytes):
            txt = txt.decode('ascii', 'ignore')
        txt = txt.strip('\x00').strip()
        if txt:
            vazute.append(txt)
            printer(f"{prefix}{txt}")


def run(m, args, printer=print, input_fn=input, now_fn=time.monotonic,
        sleep_fn=time.sleep):
    ch, info = neutral_channels(m, printer)
    if getattr(args, 'mode_pwm', None) is not None:
        ch[MODE_CHANNEL - 1] = int(args.mode_pwm)
    printer(f"  neutru: roll {ch[0]} pitch {ch[1]} yaw {ch[3]} | "
            f"throttle {ch[2]} (mijloc {info['rc3_min']}..{info['rc3_max']}, "
            f"NU trim {info['trim'].get('RC3_TRIM')})")
    printer(f"  AUX {AUX_CHANNEL} pornit jos ({AUX_LOW_PWM}); "
            f"prag poarta 1700")

    period = 1.0 / RATE_HZ
    t0 = now_fn()
    ridicat_la = None
    coborat = False
    # Cateva cicluri cu AUX jos INAINTE de comutare: poarta cere FRONTUL
    # crescator (§8), deci trebuie sa fi vazut mai intai starea de jos.
    prag_jos = t0 + args.settle_low

    while True:
        now = now_fn()
        if ridicat_la is None and now >= prag_jos:
            if args.after is not None:
                if now - t0 >= args.after:
                    ridicat_la = now
            else:
                printer("\n  Enter = ridic AUX 7 (Ctrl-C = iesire)")
                try:
                    input_fn('')
                except EOFError:
                    printer("  stdin inchis; ridic acum")
                ridicat_la = now_fn()

        if ridicat_la is not None:
            de_cand = now - ridicat_la
            if args.then_low and de_cand >= args.then_low_after:
                # Coborarea e DEFINITIVA: prima varianta punea AUX sus la
                # inceputul fiecarui ciclu si abia apoi verifica --then-low,
                # deci canalul urca inapoi la urmatoarea iteratie si REJECT-ul
                # nu se mai stergea niciodata.
                if not coborat:
                    coborat = True
                    printer(f"\n  AUX jos la {de_cand:.1f} s de la ridicare "
                            f"(REJECT se sterge doar cu AUX jos)")
            ch[AUX_CHANNEL - 1] = (AUX_LOW_PWM if coborat else AUX_HIGH_PWM)

        send(m, ch)
        drain_statustext(m, printer)

        if ridicat_la is not None:
            de_cand = now - ridicat_la
            if de_cand < 1.2:
                printer(f"\r  AUX SUS de {de_cand:.2f} s "
                        f"(fereastra de asezare 1.0 s)   ", end='', flush=True)
            if args.hold and de_cand >= args.hold:
                printer(f"\n  --hold {args.hold:.0f} s atins")
                return 0
        sleep_fn(period)


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--conn', default='udpin:127.0.0.1:14553',
                   help='implicit: portul de gamepad din start_sim.sh')
    p.add_argument('--source-system', type=int, default=255,
                   help='sysid-ul nostru; trebuie sa fie MAV_GCS_SYSID')
    p.add_argument('--after', type=float, default=None,
                   help='ridica AUX dupa N secunde (implicit: la Enter)')
    p.add_argument('--hold', type=float, default=None,
                   help='iesi la N secunde de la ridicare')
    p.add_argument('--settle-low', type=float, default=1.0,
                   help='cat tinem AUX jos inainte, ca frontul sa existe')
    p.add_argument('--then-low', action='store_true',
                   help='coboara AUX dupa --then-low-after (sterge REJECT)')
    p.add_argument('--then-low-after', type=float, default=5.0)
    p.add_argument('--mode-pwm', type=int, default=None,
                   help=f'forteaza canalul {MODE_CHANNEL}; implicit nu se '
                        f'atinge, ca `mode guided` sa ramana functional')
    a = p.parse_args(argv)

    print(f"\n  conectare la {a.conn} (sysid {a.source_system}) ...")
    m = mavutil.mavlink_connection(a.conn, source_system=a.source_system)
    m.wait_heartbeat()
    print(f"  heartbeat sys={m.target_system} comp={m.target_component}")

    ok, mesaj = check_gcs_sysid(m, a.source_system)
    print(f"  {mesaj}")
    if not ok:
        print()
        return 2

    if a.mode_pwm is not None:
        print(f"  ATENTIE: fortez canalul {MODE_CHANNEL} la {a.mode_pwm}; "
              f"comenzile de mod din MAVProxy pot fi suprascrise")

    try:
        rc = run(m, a)
    except KeyboardInterrupt:
        print("\n  intrerupt")
        rc = 130
    finally:
        print("  eliberez override-ul ...")
        release(m)
        print("  gata.\n")
    return rc


if __name__ == '__main__':
    sys.exit(main())
