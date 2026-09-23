#!/usr/bin/env python3
"""
Ultimele N cadre, cu timestamp-ul de captura (8.3.3).

Mutat din `tools/nova_service.py` in `nova/` la J2: aceeasi structura e
folosita si de serviciul RACE_MONITOR, si de aplicatia de bord, si de
simulare. Doua copii ale aceluiasi ring ar fi divergat la prima
modificare - exact clasa de bug din §5.14.
"""

import collections
import os

#: ~1 s la 30 fps. §6/8.3.3 cere ~2 s; pe Pi fiecare cadru e 3 MB, deci
#: valoarea se alege la apelant, nu aici.
RING_FRAMES = 30


class FrameRing:
    """Ultimele N cadre, cu timestamp-ul de captura (pregatire pentru 8.3.3).

    Pastreaza referinte, nu copii: cadrele vin din picamera2 si nu sunt
    refolosite dupa `release()`-ul cererii, deci pot fi tinute ca atare.
    `nbytes` e util in log ca sa se vada cat RAM consuma efectiv, nu cat am
    estimat noi."""

    def __init__(self, maxlen=RING_FRAMES):
        self.buf = collections.deque(maxlen=maxlen)

    def push(self, frame, t):
        self.buf.append((t, frame))

    def __len__(self):
        return len(self.buf)

    def nbytes(self):
        return sum(f.nbytes for _, f in self.buf)

    def span_s(self):
        if len(self.buf) < 2:
            return 0.0
        return self.buf[-1][0] - self.buf[0][0]

    def dump(self, out_dir, prefix='ring'):
        """Scrie cadrele pe disc. Intoarce lista de fisiere.

        Numele contine timestamp-ul de CAPTURA in milisecunde, nu un index:
        asa cadrul se poate alinia cu logul supervizorului si cu .bin, care e
        tot rostul lui 8.3.3 / 6.2.1.30. Un `0001.png` nu se poate pune in
        relatie cu nimic."""
        import cv2
        os.makedirs(out_dir, exist_ok=True)
        scrise = []
        for t, frame in list(self.buf):
            nume = f"{prefix}_{int(t * 1000):015d}.png"
            cale = os.path.join(out_dir, nume)
            if cv2.imwrite(cale, frame):
                scrise.append(cale)
        return scrise

    def since(self, t):
        """Cadrele capturate dupa `t`, cronologic. Grupul C scoate de aici
        cadrul de touchdown, dupa timestamp-ul capturii - nu dupa cel al
        deciziei (vezi §3 din CLAUDE.md)."""
        return [(ts, f) for ts, f in self.buf if ts >= t]
