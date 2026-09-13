#!/usr/bin/env python3
"""
Geofence pentru segmentul autonom (15.2.4).

Regulamentul cere monitorizare **onboard**, in FC sau intr-un mission
computer independent. Varianta cea mai solida e sa lasam firmware-ul sa
monitorizeze: companion-ul incarca, inainte de handover, un fence circular de
10 m centrat pe **marker**, prin protocolul de misiune
(`MAV_MISSION_TYPE_FENCE`), cu actiune RTL. De acolo incolo verificarea e
determinista si nu depinde de Pi.

Supervizorul pastreaza si o verificare de raza calculata din pozitie
(`nova/safety.py: _mon_radius`), ca plasa de siguranta daca incarcarea
esueaza. Doua straturi, deliberat.

**Capcana de ordine.** In restul turului e activ fence-ul de limita a
traseului (16.3.2). Cel de 10 m are sens doar in segmentul autonom. Deci:
salveaza geometria existenta inainte de a incarca, si restaureaz-o la
handback. Un handback care lasa vehiculul fara fence de traseu e mai rau
decat sa nu fi schimbat nimic.

**Verificarea prin citire inapoi e obligatorie** (in spiritul §5.10):
protocolul de misiune poate raporta `MISSION_ACK` cu ACCEPTED si lasa
geometria veche activa, iar `FENCE_*` sunt parametri obisnuiti, deci supusi
aceleiasi capcane a numelor.
"""

import math
import time

from pymavlink import mavutil

# --- PRAGURI SI PARAMETRI. Se transcriu in Compliance Matrix. -------------

#: 15.2.4 - raza fata de marker.
FENCE_RADIUS_M = 10.0

#: FENCE_TYPE e o masca de biti (AC_Fence.cpp):
#:   bit 0 (1) Max altitude
#:   bit 1 (2) Circle Centered on Home
#:   bit 2 (4) Inclusion/Exclusion Circles+Polygons
#: Implicit e 7, adica toate trei.
FENCE_TYPE_ALT_MAX = 1
FENCE_TYPE_CIRCLE_HOME = 2
FENCE_TYPE_INCLUSION = 4

#: Ce vrem in segmentul autonom: cercul de incluziune pe marker SI plafonul.
#: NU cercul centrat pe home (bit 1) - acela e FENCE_RADIUS in jurul
#: punctului de decolare, nu in jurul markerului, si ar taia zona utila.
#:
#: ATENTIE: 15.2.4 cere si raza de 10 m, SI plafonul de 30 m. Daca lasam
#: doar bitul de incluziune (4), plafonul de altitudine se dezactiveaza
#: exact in segmentul in care regulamentul il cere. Prima varianta a acestui
#: modul avea fix greseala asta; a iesit la iveala citind valoarea implicita
#: de pe FC (7) si intrebandu-ne ce anume stingem.
FENCE_TYPE_AUTONOM = FENCE_TYPE_ALT_MAX | FENCE_TYPE_INCLUSION      # = 5

#: FENCE_ACTION 1 = "RTL or Land" (AC_Fence.cpp). 15.2.4 cere abort SI
#: Return-to-Home.
FENCE_ACTION_RTL = 1

#: Plafonul din 15.2.4, ca plasa in firmware. Supervizorul masoara deasupra
#: punctului de handover (CEILING_AGL_M); asta e deasupra lui home, deci o
#: aproximatie - de aceea exista ambele straturi.
FENCE_ALT_MAX_M = 30

FENCE_PARAMS_AUTONOM = {
    'FENCE_ENABLE': 1,
    'FENCE_TYPE': FENCE_TYPE_AUTONOM,
    'FENCE_ACTION': FENCE_ACTION_RTL,
    'FENCE_ALT_MAX': FENCE_ALT_MAX_M,
}

MISSION_TYPE_FENCE = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
CMD_CIRCLE_INCLUSION = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION

OP_TIMEOUT_S = 6.0
ITEM_TIMEOUT_S = 2.0

EARTH_R = 6378137.0


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    """Deplaseaza o pozitie cu north/east metri. Aproximatie de pamant plat,
    buna sub cativa km - aici vorbim de metri."""
    dlat = north_m / EARTH_R
    dlon = east_m / (EARTH_R * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


class FenceItem:
    """Un element de fence, in forma in care il trimite MISSION_ITEM_INT."""

    __slots__ = ('seq', 'command', 'param1', 'lat', 'lon')

    def __init__(self, seq, command, param1, lat, lon):
        self.seq = seq
        self.command = command
        self.param1 = param1
        self.lat = int(lat)          # grade * 1e7
        self.lon = int(lon)

    def __eq__(self, other):
        return (isinstance(other, FenceItem)
                and self.command == other.command
                and abs(self.param1 - other.param1) < 1e-3
                and self.lat == other.lat and self.lon == other.lon)

    def __str__(self):
        return (f"cmd={self.command} raza={self.param1:g} "
                f"lat={self.lat / 1e7:.7f} lon={self.lon / 1e7:.7f}")


class FenceManager:
    """
        fm = FenceManager(vehicle)
        fm.save()                                  # inainte de handover
        fm.upload_marker_fence(lat, lon)           # 10 m in jurul markerului
        ...
        fm.restore()                               # la handback
    """

    def __init__(self, vehicle, radius_m=FENCE_RADIUS_M, verbose=True):
        self.v = vehicle
        self.radius_m = radius_m
        self.verbose = verbose
        self.inbox = []
        self.saved = None            # geometria dinainte de handover
        self.saved_params = None
        self.active = None           # ce am incarcat noi
        self.v.mission_handler = self._on_mission

    # -- transport ---------------------------------------------------------
    def _on_mission(self, msg):
        self.inbox.append(msg)

    def _wait(self, types, timeout, match=None):
        """Asteapta un mesaj de misiune, continuand sa pompam bucla."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.v.pump()
            for i, msg in enumerate(self.inbox):
                if msg.get_type() in types and (match is None or match(msg)):
                    return self.inbox.pop(i)
            time.sleep(0.002)
        return None

    def _log(self, text):
        if self.verbose:
            print(f"[fence] {text}")

    # -- download ----------------------------------------------------------
    def download(self, timeout=OP_TIMEOUT_S):
        """Geometria de fence pe care o are FC-ul acum, sau None la esec."""
        self.inbox.clear()
        self.v.m.mav.mission_request_list_send(
            self.v.m.target_system, self.v.m.target_component,
            MISSION_TYPE_FENCE)
        msg = self._wait(('MISSION_COUNT',), timeout)
        if msg is None:
            self._log("download: niciun MISSION_COUNT")
            return None
        count = msg.count
        items = []
        for seq in range(count):
            self.v.m.mav.mission_request_int_send(
                self.v.m.target_system, self.v.m.target_component, seq,
                MISSION_TYPE_FENCE)
            it = self._wait(('MISSION_ITEM_INT',), ITEM_TIMEOUT_S,
                            match=lambda m, s=seq: m.seq == s)
            if it is None:
                self._log(f"download: lipseste elementul {seq}")
                return None
            items.append(FenceItem(it.seq, it.command, it.param1, it.x, it.y))
        self.v.m.mav.mission_ack_send(
            self.v.m.target_system, self.v.m.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED, MISSION_TYPE_FENCE)
        self._log(f"download: {count} elemente")
        return items

    # -- upload ------------------------------------------------------------
    def upload(self, items, timeout=OP_TIMEOUT_S):
        self.inbox.clear()
        self.v.m.mav.mission_count_send(
            self.v.m.target_system, self.v.m.target_component,
            len(items), MISSION_TYPE_FENCE)
        if not items:
            ack = self._wait(('MISSION_ACK',), timeout)
            return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED

        sent = 0
        t0 = time.monotonic()
        while sent < len(items) and time.monotonic() - t0 < timeout:
            req = self._wait(('MISSION_REQUEST_INT', 'MISSION_REQUEST',
                              'MISSION_ACK'), ITEM_TIMEOUT_S)
            if req is None:
                continue
            if req.get_type() == 'MISSION_ACK':
                break
            seq = req.seq
            if seq >= len(items):
                continue
            it = items[seq]
            self.v.m.mav.mission_item_int_send(
                self.v.m.target_system, self.v.m.target_component,
                seq, mavutil.mavlink.MAV_FRAME_GLOBAL,
                it.command, 0, 1,
                it.param1, 0, 0, 0, it.lat, it.lon, 0, MISSION_TYPE_FENCE)
            sent += 1

        ack = self._wait(('MISSION_ACK',), timeout)
        if ack is None:
            self._log("upload: niciun MISSION_ACK")
            return False
        ok = ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
        self._log(f"upload: {len(items)} elemente, ack={ack.type} "
                  f"({'ACCEPTED' if ok else 'RESPINS'})")
        return ok

    # -- operatii de nivel inalt ------------------------------------------
    def save(self):
        """Salveaza geometria si parametrii dinainte de handover.

        Fara asta, restaurarea de la handback ar lasa vehiculul fara fence-ul
        de traseu (16.3.2) pentru restul turului."""
        self.saved = self.download()
        for name in FENCE_PARAMS_AUTONOM:
            self.v.request_param(name)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            self.v.pump()
            if all(n in self.v.params for n in FENCE_PARAMS_AUTONOM):
                break
            time.sleep(0.01)
        self.saved_params = {n: self.v.params.get(n)
                             for n in FENCE_PARAMS_AUTONOM}
        n = '?' if self.saved is None else len(self.saved)
        self._log(f"salvat: {n} elemente, parametri {self.saved_params}")
        return self.saved is not None

    def upload_marker_fence(self, lat_deg, lon_deg, now=None):
        """Fence circular de `radius_m` centrat pe marker, actiune RTL."""
        item = FenceItem(0, CMD_CIRCLE_INCLUSION, self.radius_m,
                         round(lat_deg * 1e7), round(lon_deg * 1e7))
        if not self.upload([item]):
            return False
        for name, val in FENCE_PARAMS_AUTONOM.items():
            self.v.set_param(name, val, now)
        self.active = [item]
        return True

    def restore(self, now=None):
        """La handback: pune inapoi geometria si parametrii de dinainte."""
        ok = True
        if self.saved is not None:
            ok = self.upload(self.saved)
        if self.saved_params:
            for name, val in self.saved_params.items():
                if val is not None:
                    self.v.set_param(name, val, now)
        self._log(f"restaurat: {'ok' if ok else 'ESEC'}")
        return ok

    # -- verificare --------------------------------------------------------
    def verify(self, expected=None, check_params=True):
        """Citeste inapoi si compara. MISSION_ACK cu ACCEPTED nu e dovada ca
        geometria s-a schimbat - vezi §5.10."""
        expected = self.active if expected is None else expected
        got = self.download()
        if got is None:
            return False, 'download esuat'
        if expected is None:
            return False, 'nimic de comparat'
        if len(got) != len(expected):
            return False, f"{len(got)} elemente pe FC, asteptam {len(expected)}"
        for a, b in zip(got, expected):
            if a != b:
                return False, f"element diferit: FC are '{a}', asteptam '{b}'"
        if check_params:
            for name, val in FENCE_PARAMS_AUTONOM.items():
                self.v.request_param(name)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                self.v.pump()
                if all(n in self.v.params for n in FENCE_PARAMS_AUTONOM):
                    break
                time.sleep(0.01)
            for name, val in FENCE_PARAMS_AUTONOM.items():
                got_val = self.v.params.get(name)
                if got_val is None:
                    return False, f"{name} nu exista pe acest firmware"
                if abs(got_val - val) > 1e-6:
                    return False, f"{name}={got_val:g}, asteptam {val:g}"
        return True, f"{len(got)} elemente confirmate prin citire inapoi"
