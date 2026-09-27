#!/usr/bin/env python3
"""
NOVA - ZDC 2026
Suita offline pentru primitivele de concurenta (nova/concurrency.py).

    python3 tools/test_concurrency.py

Fara hardware. Ce conteaza: Latest nu amesteca valoarea cu timpul ei sub
acces concurent; interpolarea yaw trece prin +-pi pe arcul scurt; o
exceptie in pasul unei bucle e logata si bucla continua; oprirea e curata.
"""

import logging
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nova import concurrency as cc                          # noqa: E402


class _Log(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


def test_latest_nu_amesteca_valoarea_cu_timpul():
    """Perechea (valoare, t) se scrie si se citeste impreuna. Un scriitor
    rapid si trei cititori: fiecare pereche citita e una scrisa."""
    lt = cc.Latest()
    assert lt.get() == (None, None)
    stop = threading.Event()
    rele = []

    def scrie():
        i = 0
        while not stop.is_set():
            lt.set((i, i * 2), t=float(i))
            i += 1

    def citeste():
        while not stop.is_set():
            v, t = lt.get()
            if v is not None and (v[1] != v[0] * 2 or t != float(v[0])):
                rele.append((v, t))

    fire = [threading.Thread(target=scrie)] + [threading.Thread(target=citeste) for _ in range(3)]
    for f in fire:
        f.start()
    time.sleep(0.3)
    stop.set()
    for f in fire:
        f.join(2.0)
    assert not rele, rele[:3]
    assert lt.count > 100
    v, t = lt.get()
    assert v is not None and t == float(v[0])
    return f"{lt.count} scrieri, 3 cititori, 0 perechi amestecate"


def test_interpolarea_yaw_peste_pi():
    b = cc.AttitudeBuffer()
    assert b.at(1.0) is None and len(b) == 0
    b.push(1.0, 0.0, 0.10, math.radians(170))
    b.push(1.2, 0.2, 0.30, math.radians(-170))
    r, p, y = b.at(1.1)
    assert abs(r - 0.1) < 1e-9 and abs(p - 0.2) < 1e-9
    assert abs(abs(math.degrees(y)) - 180.0) < 1e-6, math.degrees(y)
    # 350 -> 10 prin 0, nu prin 180
    b2 = cc.AttitudeBuffer()
    b2.push(0.0, 0, 0, math.radians(350))
    b2.push(1.0, 0, 0, math.radians(10))
    assert abs(math.degrees(b2.at(0.5)[2])) < 1e-6
    # in afara intervalului: capatul pana la max_gap_s, apoi None
    assert b2.at(1.0 + 0.2) == (0.0, 0.0, b2.latest()[3])
    assert b2.at(1.0 + 0.3) is None and b2.at(-0.3) is None
    assert b2.at(-0.2)[2] == b2.at(0.0)[2]
    # marginit
    b3 = cc.AttitudeBuffer(maxlen=5)
    for i in range(20):
        b3.push(float(i), 0, 0, 0)
    assert len(b3) == 5 and b3.at(3.0) is None
    return "170 -> -170 da 180; 350 -> 10 da 0; None in afara; marginit"


def test_heartbeat():
    t = [100.0]
    hb = cc.Heartbeat('x', clock=lambda: t[0])
    assert hb.age() is None and hb.count == 0 and hb.last is None
    hb.beat()
    t[0] = 100.4
    assert abs(hb.age() - 0.4) < 1e-9 and hb.count == 1
    assert abs(hb.age(now=101.0) - 1.0) < 1e-9
    return "age None inainte de prima bataie; apoi masurat"


def test_exceptia_in_pas_e_logata_si_bucla_continua():
    h = _Log()
    cc.log.addHandler(h)
    try:
        stop = threading.Event()
        hb = cc.Heartbeat('t')
        n = [0]
        erori = []

        def pas(now):
            n[0] += 1
            if n[0] == 3:
                raise ValueError('pas stricat')

        fir = cc.run_thread('test', pas, period=0.005, heartbeat=hb,
                            stop_event=stop, on_error=erori.append)
        time.sleep(0.25)
        stop.set()
        fir.join(2.0)
        assert not fir.is_alive()
        assert n[0] > 10, n[0]
        assert hb.count == n[0], (hb.count, n[0])
        assert len(erori) == 1 and isinstance(erori[0], ValueError)
        msgs = [r.getMessage() for r in h.records if 'pas stricat' in r.getMessage()]
        assert msgs and 'Traceback' in msgs[0], msgs
        return f"{n[0]} pasi, 1 exceptie logata cu traceback, bucla a continuat"
    finally:
        cc.log.removeHandler(h)


def test_oprirea_curata_si_perioada():
    stop = threading.Event()
    hb = cc.Heartbeat('p')
    momente = []
    fir = cc.run_thread('perioada', lambda now: momente.append(now),
                        period=0.02, heartbeat=hb, stop_event=stop)
    time.sleep(0.31)
    ramase = cc.stop_threads([fir], stop, timeout_s=1.0)
    assert ramase == [] and not fir.is_alive()
    assert 10 <= len(momente) <= 20, len(momente)
    dif = [b - a for a, b in zip(momente, momente[1:])]
    assert 0.015 < sorted(dif)[len(dif) // 2] < 0.03, sorted(dif)[len(dif) // 2]
    # un pas lent nu produce o rafala de recuperare
    lent = []

    def pas_lent(now):
        lent.append(now)
        if len(lent) == 2:
            time.sleep(0.1)

    stop2 = threading.Event()
    fir2 = cc.run_thread('lent', pas_lent, period=0.02, stop_event=stop2)
    time.sleep(0.25)
    cc.stop_threads([fir2], stop2)
    dif2 = [b - a for a, b in zip(lent, lent[1:])]
    assert all(d > 0.015 for d in dif2[2:]), dif2
    return f"{len(momente)} iteratii in 0.3 s la 50 Hz; stop curat; fara rafala dupa un pas lent"


def test_excepthook_instalat_o_data():
    h = _Log()
    cc.log.addHandler(h)
    vechi = threading.excepthook
    try:
        cc.install_excepthook()
        hook = threading.excepthook
        cc.install_excepthook()
        assert threading.excepthook is hook, "reinstalat"

        def moare():
            raise RuntimeError('fir fara run_loop')

        fir = threading.Thread(target=moare, name='muribund')
        import contextlib, io
        with contextlib.redirect_stderr(io.StringIO()):
            fir.start()
            fir.join(2.0)
        msgs = [r.getMessage() for r in h.records if 'muribund' in r.getMessage()]
        assert msgs and 'fir fara run_loop' in msgs[0]
        return "excepthook logheaza numele firului si traceback-ul; idempotent"
    finally:
        cc.log.removeHandler(h)
        threading.excepthook = vechi
        cc._excepthook_installed = False


TESTS = [
    ('Latest: acces concurent', test_latest_nu_amesteca_valoarea_cu_timpul),
    ('AttitudeBuffer: yaw peste pi', test_interpolarea_yaw_peste_pi),
    ('Heartbeat', test_heartbeat),
    ('run_loop: exceptia in pas e logata, bucla continua',
     test_exceptia_in_pas_e_logata_si_bucla_continua),
    ('run_loop: oprire curata si perioada', test_oprirea_curata_si_perioada),
    ('excepthook instalat o data', test_excepthook_instalat_o_data),
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
