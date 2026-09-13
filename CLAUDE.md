# NOVA — Zero Delta Challenge 2026

Context de proiect. Citește-l integral înainte de a modifica ceva.
Scris după o sesiune lungă de depanare în SITL; fiecare "capcană" de
mai jos a costat ore reale. Nu le redescoperi.

---

## 1. Ce construim

Echipa studențească NOVA participă la **Zero Delta Challenge 2026**
(Munchen), parte din seria Student AirRace. Proba: un traseu de curgă
cu un segment autonom obligatoriu la mijloc.

### Secvența unui tur

1. Decolare, traversare poartă start (cronometrul pornește)
2. Zbor manual pe traseu până la punctul de handover
3. **Segment autonom**: coborâre pe marker ArUco → contact → stabilizare
   ≥1 s → captură imagine → urcare la ≥5 m
4. Handback la pilot, restul traseului, poartă finish
5. Aterizare în zona desemnată

Se pot repeta tururi în slotul de 15 minute. Se ia turul cu cel mai bun
rezultat combinat (timp + statut autonom); nu se pot amesteca.

### Punctaj (100 total)

| Disciplină | Puncte |
|---|---|
| Timp de parcurgere | 40 |
| Provocarea autonomă | 20 (binar, 10/cursă) |
| Documentație inginerie & siguranță | 20 |
| Bonus inovație | 20 |

Bonus de timp pentru secvență autonomă completă: **−15 s**.
Principiul **all-or-nothing**: orice intervenție manuală în segmentul
autonom îl transformă în tur manual, zero puncte pe autonomie.

---

## 2. Hardware

| Componentă | Model | Note |
|---|---|---|
| Flight controller | CUAV X7+ | ArduPilot (Copter 4.8.0-dev testat) |
| Companion | Raspberry Pi 4 | Python + pymavlink + OpenCV |
| Cameră | Raspberry Pi Camera Module 3 **Wide** | IMX708, rolling shutter |
| Frame | Quad X | ~2.5–3.0 kg (vezi §7 — contradicție deschisă) |
| Baterii | 2× 6S LiPo 4000 mAh | 25.2 V max |

### Cameră — cifre de lucru

- Senzor IMX708, 4608×2592 nativ, pixel 1.4 um
- Obiectiv Wide: focală 2.75 mm, f/2.2
- **HFOV 102°, VFOV 67°** (fișa tehnică). Din focala geometrică la 16:9,
  VFOV-ul derivat e **69.6°**; diferența e a decupajului 2304×1296 față de
  senzorul 4:3. Valoarea care contează operațional e cea din calibrare.
- Rezoluție de lucru: **2304×1296**, focală echivalentă **933 px**
- Cameră la **74.5 mm** deasupra solului la contact (tren de aterizare)
- Autofocus PDAF — **trebuie blocat manual**, altfel caută focus exact
  în timpul coborârii

Dimensiunea markerului (480 mm) în imagine: `marker_px = 933 * 0.48 / Z`

| Altitudine | marker_px |
|---|---|
| 20 m | 22 |
| 15 m | 30 |
| 10 m | 45 |
| 5 m | 90 |
| 1 m | 448 |
| 0.45 m | 995 |

**Nu există rangefinder hardware.** Singurul senzor orientat în jos e
camera. Vezi §5 pentru consecințe.

---

## 3. Mediu de dezvoltare

Ubuntu **22.04 jammy** (nu 24.04).

| Componentă | Versiune |
|---|---|
| Gazebo | Harmonic (gz-harmonic 1.0.0-1~jammy) |
| ArduPilot | branch principal, ArduCopter 4.8.0-dev |
| ardupilot_gazebo | build local în `~/ardupilot_gazebo/build` |
| Python viziune | venv la `~/nova-venv` (opencv-contrib-python, pymavlink, pygame) |
| MAVProxy | în `~/.local/bin`, Python de sistem |

**Două medii Python separate, nu le amesteca:**
- SITL, MAVProxy, `gz sim` → terminal **fără** venv
- cod NOVA (detector, gamepad) → terminal **cu** `nova-venv` activat

**Cum se manifestă amestecul.** `waf` alege `python` din `PATH`. Cu venv-ul
activ, îl găsește pe cel din `nova-venv`, care nu are `empy`, și build-ul
moare după ~2 minute cu:

```
Checking for program 'python' : /home/fireteanualex/nova-venv/bin/python3
...
you need to install empy with 'python3 -m pip install empy==3.3.4'
SIM_VEHICLE: Build failed
```

`empy` e instalat în Python-ul de **sistem** (3.3.4), nu în venv — și acolo
trebuie să rămână. A-l instala în venv ar masca problema, nu ar rezolva-o.

Capcana e insidioasă pentru că `gnome-terminal` **moștenește mediul**: e
destul ca `start_sim.sh` să fie lansat dintr-un terminal cu venv activ, și
toate ferestrele copil îl primesc. `start_sim.sh` scoate acum explicit venv-ul
din mediul ferestrelor Gazebo și SITL (`unset VIRTUAL_ENV PYTHONHOME
PYTHONPATH` + `PATH` filtrat) și verifică `import em` **înainte** de build,
ca să pice în două secunde cu un mesaj util, nu în două minute cu unul
criptic. Dacă pornești `sim_vehicle.py` de mână, dă întâi `deactivate`.

### Structura repo

```
~/nova-zdc/
├── start_sim.sh              # lansare completă a mediului (--gamepad, --wipe)
├── config/
│   ├── nova_sitl.parm        # parametri ArduPilot SITL, încărcați la boot
│   ├── nova.json             # config companion; E0: autonomy_enabled=false
│   ├── gamepad.json          # maparea gamepad-ului, din --calibrate
│   └── camera_pi.yaml        # calibrarea camerei, din calibrate_camera.py (evidență: se commit-uiește)
├── nova/                     # cod companion, identic sim ↔ Raspberry Pi
│   ├── config.py             # config/nova.json + garda E0
│   ├── detection.py          # Detection + modelul de cameră (contractul)
│   ├── detector_pi.py        # detectorul real: picamera2 → ArUco → solvePnP
│   ├── vehicle.py            # legătura MAVLink: telemetrie + comenzi + parametri
│   ├── handover.py           # poarta de intrare (singura) + E0
│   ├── safety.py             # Safety Supervisor
│   ├── rc.py                 # override pe manșe
│   ├── fence.py              # geofence prin protocolul de misiune
│   └── state_machine.py      # mașina de stări a segmentului autonom
├── tools/
│   ├── nova_pi.py            # aplicația de BORD (detector real, fără ocolire E0)
│   ├── fake_detector.py      # detector sintetic + aplicația de SIM (ocolește E0)
│   ├── calibrate_camera.py   # E1.2: tablă de șah → camera_pi.yaml
│   ├── calibrate_sticks.py   # zgomotul manșelor → deadband
│   ├── check_params.py       # citire înapoi a parametrilor (§5.10)
│   ├── check_rc_override.py  # RC_CHANNELS_OVERRIDE se reflectă în RC_CHANNELS?
│   ├── gamepad_rc.py         # punte gamepad → RC_CHANNELS_OVERRIDE
│   └── test_*.py             # suite offline: state_machine, safety, handover,
│                             #   detector_pi, calibrate_camera
└── docs/
```

**Separarea detector ↔ control.** Detectorul *publică* doar detecții
(`poll(now)` → `Detection`: offset unghiular, distanță, `marker_px`, range,
timestamp). Mașina de stări le consumă și e singura care comandă vehiculul.
Pe Pi, `fake_detector.py` e înlocuit de detectorul ArUco real; `nova/` rămâne
neschimbat. Consecințe pentru cine scrie detectorul real:

- convenția de axe e cea din `nova/detection.py`; rotația din §5.1 se aplică
  în `state_machine.py`, la emiterea `LANDING_TARGET` — **nu** o repeta în
  `solvePnP`
- verificarea de încadrare în cadru (§5.2) e în `CameraModel.fits_in_frame`
- evenimentul `scoring_capture` poartă timestamp-ul **capturii**, nu al
  deciziei; cadrul se scoate din ring buffer după acel timestamp
- mașina de stări folosește doar ceasul primit prin `update(now)` /
  `on_detection(det, now)`, deci se poate rula în timp accelerat, fără SITL

### Porturi MAVLink

| Port | Consumator |
|---|---|
| 14550 | MAVProxy / QGroundControl |
| 14552 | `fake_detector.py` |
| 14553 | `gamepad_rc.py` |
| 14554 | `check_params.py`, rulat automat de `start_sim.sh` |

### Rulare

```bash
cd ~/nova-zdc && ./start_sim.sh [--noise-px 1.5] [--latency-ms 120] [--dropout 0.15]
```

Deschide Gazebo, SITL și detectorul în ferestre separate, aplică
parametrii automat. Apoi în MAVProxy:
`mode guided` → `arm throttle` → `takeoff 15` → (așteaptă) → `mode land`

---

## 4. Ce funcționează deja (Faza 1 — validat)

Aterizare de precizie în SITL, prin `LANDING_TARGET` + `DISTANCE_SENSOR`
trimise de companion, cu PLND nativ ArduPilot.

**13 rulări, toate sub 2.5 cm eroare.** Regulamentul nu punctează
precizia — 8.3.2 e binar (secvență completă = 10 puncte), iar 8.3.3
cere doar ca centrul imaginii de touchdown să conțină un pixel de pe
suprafața markerului. La 74.5 mm, amprenta camerei (184×99 mm) e
integral pe marker pentru orice eroare sub ~19 cm. Precizia e deci
rezolvată cu marjă mare; **completarea secvenței e tot ce contează.**

Din acest motiv codul **nu mai mapează eroarea la puncte** (A3). Pragurile
de 10/25/50 cm raportate până acum ca „scor estimat 8.3.4" erau inventate —
nu există în regulament. Eroarea rămâne raportată în cm ca metrică internă,
cu un singur prag cu sens fizic: `CAM_FOOTPRINT_MARGIN_M = 0.19` m, peste
care centrul imaginii de touchdown poate cădea în afara markerului și 8.3.3
nu mai e garantat.

| Configurație | Erori (cm) | Captură (m) | Derivă (cm) |
|---|---|---|---|
| nominal | 0.8, 2.0 | 0.41 | 0.0 |
| `--noise-px 1.5` | 1.2, 1.2, 1.9, 0.8 | 0.39–0.44 | 0.0–1.1 |
| `--latency-ms 120` | 2.2, 0.8, 1.9 | 0.41–0.42 | 0.5–1.2 |
| `--dropout 0.15` | 2.0, 0.8, 2.0 | 0.39–0.45 | 0.3–1.2 |
| combinat | 1.6, 2.4, 1.6 | 0.42–0.44 | 0.0–0.5 |

Medie 1.57 cm, maxim 2.4 cm.

> **Ce NU validează rulările astea.** Au fost făcute cu `WP_RFND_USE` scris
> greșit ca `WPNAV_RFND_USE`, deci parametrul nu era aplicat (§5.4, §5.10).
> Protecția din §5.7 — urcarea din GUIDED interpretată „deasupra terenului" —
> **nu era activă**. A funcționat pentru că planul markerului coincidea cu
> planul lui home, nu pentru că mecanismul era corect configurat.
>
> Rezultatele rămân valide ca validare a buclei de control și a arhitecturii
> software. **Nu** validează comportamentul pe teren denivelat, adică exact
> cazul în care markerul e la altă cotă decât punctul de decolare. Acolo
> altitudinea de urcare s-ar fi raportat la o referință greșită. Cazul e
> netestat; de refăcut cu parametrul corect înainte de a-l declara acoperit.

**Observație cheie:** zgomotul, latența și dropout-ul nu au avut efect
practic. Motivul e fizic — eroarea unghiulară scade cu
`noise_px / marker_px`, iar la 0.5 m markerul are ~900 px. Precizia e
limitată de ultimii 50 cm, nu de calitatea detecției la altitudine.

**Atenție:** detectorul sintetic e optimist. Nu modelează motion blur,
detecții false, variații de expunere, marker ocluzat. Cifrele sunt
validare de arhitectură software, nu predicție de performanță reală.

---

## 5. Capcane descoperite empiric — NU le redescoperi

### 5.1 Convenția de axe LANDING_TARGET

Rotația corectă, determinată prin bisecție (`--conv 2`):

```python
angle_x, angle_y = angle_y, -angle_x
```

Fără ea, drona orbitează în elipsă în loc să convergă — semnătura unei
corecții perpendiculare pe eroare. Aceeași rotație va trebui aplicată
datelor reale din `solvePnP`.

### 5.2 Camera nu poate încadra markerul sub 0.38 m

Verificarea de FOV trebuie să testeze dacă markerul **încape întreg**,
nu doar dacă centrul e vizibil:

```python
frame_h_px = 2.0 * focal_px * math.tan(math.radians(VFOV_DEG / 2))
if marker_px > frame_h_px * 0.95:
    return None    # markerul nu mai încape în cadru
```

Cu focal 933 și VFOV 67°: `frame_h_px ≈ 1235`, deci markerul se pierde
peste ~1173 px, adică **sub 0.38 m**.

Fără acest fix: `SCORING_CAPTURE` se declanșa la 0.19 m cu 2344 px
(fizic imposibil), iar takeoff-ul rămânea blocat după aterizare.

### 5.3 Telemetrul derivat din viziune

Camera **este** rangefinder-ul: `solvePnP` dă distanța din dimensiunea
cunoscută a markerului, trimisă ca `DISTANCE_SENSOR` cu
`RNGFND1_TYPE=10`.

Acuratețea are forma potrivită — proastă la altitudine (±50 cm la 15 m),
excelentă aproape de sol (±2 mm la 1 m). Mai bună decât un TFmini sub 2 m.

**Corecția de înclinare se inversează:** ArduPilot înmulțește citirea cu
`cos(tilt)`, deci trimite `alt / cos(tilt)`, nu `alt`.

**Când markerul nu e vizibil nu se trimite nimic** (A2). Varianta veche —
20 m în aer, altitudinea reală sub 1 m — era o iluzie de simulare: pe
vehiculul real „altitudinea reală" nu există, pentru că fără marker camera
nu are nicio sursă de distanță. Acolo valoarea de rezervă ar fi 20 m pe tot
parcursul, inclusiv pe sol. Măsurat în SITL, asta strică **două** lucruri
independente — vezi §5.9.

Consecința asupra detectorului de aterizare rămâne valabilă, dar cu semnul
invers față de cum era scris aici: pericolul e telemetrul **mincinos**, nu
telemetrul **lipsă**. `land_detector.cpp:134` cere
`!rangefinder_alt_ok() || alt_filt < 2 m`; fără date, prima condiție e
adevărată și detectorul lucrează normal pe celelalte criterii. Cu o valoare
constantă peste 2 m, ambele sunt false și `land_complete` nu se confirmă
niciodată.

### 5.4 Parametri ArduPilot

- `RNGFND1_MIN` / `RNGFND1_MAX` sunt în **metri**, nu centimetri
  (redenumite din `_CM` în 4.6+)
- Sub-parametrii `PLND_*` și `RNGFND1_*` apar **doar după reboot**.
  `param fetch` nu e suficient. Rezolvat prin `--add-param-file` în
  `start_sim.sh`, care îi aplică înainte de inițializare.
- `SURFTRAK_MODE 0` **obligatoriu**. Telemetrul nostru e intermitent
  prin construcție; cu urmărirea de suprafață activă se declanșează
  `Failsafe: Terrain Rangefinder Unhealthy` → RTL.
- `ARMING_CHECK` a fost înlocuit cu `ARMING_SKIPCHK` (mască de biți
  inversată: verificările pe care le **sari**). Bit 15 = Rangefinder.
- Parametrii de aterizare au fost redenumiți și trecuți în **metri**, la fel
  ca `RNGFND1_MIN`/`_MAX`: `LAND_SPEED` → `LAND_SPD_MS` (0.5),
  `LAND_SPEED_HIGH` → `LAND_SPD_HIGH_MS` (0 = folosește viteza de coborâre a
  controlerului de poziție), `LAND_ALT_LOW` → `LAND_ALT_LOW_M` (10 m).
  Numele vechi pur și simplu nu mai există — o cerere pe ele nu întoarce
  eroare, ci tăcere.
- **Obiectul `AC_WPNav` e înregistrat cu prefixul `WP_`, nu `WPNAV_`**
  (`ArduCopter/Parameters.cpp:370`: `GOBJECTPTR(wp_nav, "WP_", AC_WPNav)`).
  Deci `WP_RFND_USE`, `WP_SPD_DN` — iar `WPNAV_RFND_USE` **nu există**.
  Descoperit citind înapoi parametrii din SITL: cererea întorcea `None`,
  adică linia noastră din `nova_sitl.parm` nu făcea nimic de la început.
  Un `PARAM_SET` pe un nume inexistent nu produce eroare și nu produce
  `PARAM_VALUE` — **verifică întotdeauna prin citire înapoi**, nu presupune
  că s-a aplicat pentru că fișierul s-a încărcat fără reclamații.
- Valorile de telemetru prea mici (0.19 m după scăderea `GNDCLR`) sau
  prea mari (25 m, aproape de `MAX` 30) fac takeoff-ul să eșueze fără
  mesaj.

### 5.5 Ordinea de pornire

**Gazebo primul, SITL al doilea.** Plugin-ul deschide porturile
9002/9003 și așteaptă. Ordinea inversă → timeout.

Curăță întotdeauna procesele zombie:
```bash
pkill -f arducopter; pkill -f "gz sim"; pkill -f mavproxy
```

---

### 5.6 Dezarmarea din LAND nu se poate întârzia din parametri

`ModeLand::gps_run()` (și `nogps_run()`) dezarmează necondiționat:

```cpp
if (copter.ap.land_complete && motors->get_spool_state() == GROUND_IDLE) {
    copter.arming.disarm(AP_Arming::Method::LANDED);
}
```

Nu există `LAND_DISARMDELAY` în Copter — e parametru de Plane. `DISARM_DELAY`
acționează doar prin `auto_disarm_check()`, care nu e calea folosită în LAND.

Consecință pentru arhitectura A: singura soluție e să **nu mai fim în LAND**
în acel moment. Fereastra utilă e între `land_complete` (setat de detectorul
de aterizare) și `GROUND_IDLE`, adică rampa de spool-down: `MOT_SPOOL_TIME`
0.5 s implicit, sau `MOT_SPOOL_TIM_DN` dacă e setat. Cu `EXTENDED_SYS_STATE`
la 50 Hz reacția companion-ului e sub 100 ms, deci marja e ~5×. Dacă pe Pi se
dovedește strâmtă, se mărește `MOT_SPOOL_TIM_DN` (nu `MOT_SPOOL_TIME`, care ar
încetini și spool-up-ul la decolare).

`land_complete` nu e expus direct prin MAVLink; se citește din
`EXTENDED_SYS_STATE.landed_state == MAV_LANDED_STATE_ON_GROUND`.

**Nu comuta în GUIDED înainte de `land_complete`.** `ModeGuided` apelează
`make_safe_ground_handling()` doar când `is_disarmed_or_landed()`; altfel
rulează controlerul de poziție, care pe sol cere throttle de hover și ridică
vehiculul înapoi.

Urcarea de după e cazul suportat nativ: `Mode::do_user_takeoff_U_m()` cere
explicit `ap.land_complete == true` și `motors->armed()`, deci `NAV_TAKEOFF`
în GUIDED urcă **fără re-armare**.

### 5.7 Takeoff-ul din GUIDED poate fi interpretat „deasupra terenului”

`ModeGuided::do_user_takeoff_start_m()` folosește altitudinea ca alt-above-
terrain dacă `wp_nav->rangefinder_used_and_healthy()`, adică dacă
**`WP_RFND_USE = 1`** (implicit). Parametrul se numește `WP_RFND_USE`, nu
`WPNAV_RFND_USE` — vezi §5.4; până la runda A am setat un nume inexistent,
deci protecția asta nu era de fapt activă.

În plus, aceeași funcție conține un gard care respinge comanda direct:

```cpp
if (takeoff_alt_m <= copter.rangefinder_state.alt_m) {
    return false;   // can't takeoff downwards
}
```

`WP_RFND_USE 0` forțează interpretarea deasupra HOME și ocolește și gardul.
Aceeași motivație ca la `SURFTRAK_MODE`.

Altitudinea comandată e deasupra **home**, nu deasupra markerului: companion-ul
adună `relative_alt` măsurat la contact peste cei 5 m ceruți.

### 5.8 Precision landing e activ și în RTL și AUTO, nu doar în LAND

`PLND_ENABLED 1` nu înseamnă „precision landing în modul LAND". Aceeași
funcție e apelată din trei locuri:

```
ArduCopter/mode_land.cpp:144   land_run_normal_or_precland(land_pause)
ArduCopter/mode_rtl.cpp:439    land_run_normal_or_precland()
ArduCopter/mode_auto.cpp:1124  land_run_normal_or_precland()
```

iar `Mode::land_run_normal_or_precland()` (`mode.cpp:864`) citește
`copter.precland.enabled()` la **fiecare ciclu**.

**Măsurat în SITL:** marker la 2.0 m de home, decolare la 15 m, deplasare
deasupra markerului, apoi RTL. Cu `PLND_ENABLED 1` vehiculul a aterizat la
**2.02 m de home și 0.02 m de marker** — complet atras. Cu `PLND_ENABLED 0`
comandat înainte de RTL, a aterizat la **0.03 m de home, 1.97 m de marker**,
deși companion-ul a continuat să trimită `LANDING_TARGET` pe tot parcursul
RTL-ului (574 de mesaje). **Parametrul e ce contează, nu tăcerea pe stream.**

Exact invers față de ce cere 15.2.4 — și periculos, pentru că un RTL se
declanșează tocmai când ceva a mers prost.

**Parametrul se poate comuta în zbor, fără reboot.** Spre deosebire de
sub-parametrii din §5.4, `AC_PrecLand::init()` creează backend-ul după
`PLND_TYPE`, nu după `PLND_ENABLED` — singurul `switch` din init e pe
`_type`. Deci `PARAM_SET` la runtime funcționează.

Consecință arhitecturală: `PLND_ENABLED` pornește pe **0** în
`config/nova_sitl.parm` și îl armează companion-ul doar pentru segmentul
autonom (`nova/state_machine.py: set_precland`). Orice ieșire din secvență —
dezarmare, preluare de pilot, abort — îl pune înapoi pe 0, iar
`abort_to_rtl()` îl stinge **înainte** de a comanda RTL.

Asta acoperă și modul de urgență din 16.2.3: comutatorul e legat direct de
FC, deci companion-ul nu îl poate intercepta, dar în afara segmentului
autonom PLND e deja 0. Dacă se declanșează *în* segment, vehiculul e oricum
în raza de 6.5 m de marker, iar markerul e suprafața de aterizare desemnată.

Verificat și cazul a două încercări în aceeași sesiune, fără dezarmare:
`PLND_ENABLED` urmează `1 → 0 → 1`, ambele secvențe parcurg
`DESCEND_TRACK → SCORING_CAPTURE → FINAL_DESCENT → TOUCHDOWN_CONFIRM →
ASCENT → HANDBACK` și se opresc la 5.02 m, respectiv 5.03 m deasupra
markerului. Fără asta, `HANDBACK` și `ABORT` se ieșeau doar prin dezarmare,
deci al doilea tur din același slot de 15 minute nu ar fi pornit niciodată
segmentul autonom.

### 5.9 Telemetrul de rezervă strica două lucruri, nu unul

Valoarea de rezervă de 20 m (trimisă când markerul nu e vizibil) a fost
eliminată în runda A. Măsurat în SITL, făcea două rele independente:

**1. Dezactiva încetinirea de dinainte de contact.**
`Mode::land_run_vertical_control()` calculează viteza de coborâre cu
`sqrt_controller(MAX(land_alt_low_m,1) - get_alt_above_ground_m(), ...)`, iar
`Mode::get_alt_above_ground_m()` preferă telemetrul oricărei alte surse. Cu o
citire constantă de 20 m, termenul `10 - 20` rămâne negativ pe tot parcursul,
deci vehiculul coboară cu viteza maximă până la impact și
`ignore_descent_limit` nu devine niciodată adevărat.

Fără `DISTANCE_SENSOR`, aterizare oarbă de la 15 m (fără marker):

| Felie | viteză medie |
|---|---|
| 15 → 12 m | 0.78 m/s |
| 12 → 10.2 m | 0.96 m/s |
| **10.2 → 9.8 m** | **0.43 m/s** ← încetinirea, exact la `LAND_ALT_LOW_M` |
| sub 9.8 m | 0.49–0.50 m/s (`LAND_SPD_MS`) |

27.2 s până la contact, `landed_state` ajunge `ON_GROUND`, dezarmare normală,
**niciun mesaj de teren, telemetru sau failsafe**. Deci cauza pentru care
fusese introdusă valoarea de rezervă (`Failsafe: Terrain Rangefinder
Unhealthy`) nu mai există, odată cu `SURFTRAK_MODE 0` și `TERRAIN_ENABLE 0`.

**2. Făcea `NAV_TAKEOFF` să fie respins.** Același test, pe sol, cu și fără
telemetru fals:

```
A (fără DISTANCE_SENSOR): landed_state=ON_GROUND rngfnd=0.0
    NAV_TAKEOFF result=0  -> urcă la 4.99 m
B (DISTANCE_SENSOR 20 m):  landed_state=ON_GROUND rngfnd=20.0
    NAV_TAKEOFF result=4  -> rămâne la 0.01 m, auto-disarm după DISARM_DELAY
```

`land_complete` era confirmat în ambele cazuri (`landed_state=ON_GROUND`) —
deci **nu** detectorul de aterizare era vinovatul, cum presupunea versiunea
veche a §5.3. Vinovatul e gardul „can't takeoff downwards" din §5.7:
`takeoff_alt (5 m) <= rangefinder_state.alt (20 m)` → `return false`.
Rezultatul 4 (`MAV_RESULT_FAILED`) e singurul semn; nu apare niciun
`STATUSTEXT`.

Important pentru interpretare: măsurătoarea de mai sus a fost făcută **cu
`WP_RFND_USE` neaplicat** (numele greșit din §5.4). Repetată după corecție,
cu `WP_RFND_USE = 0` citit înapoi din FC, ambele regimuri dau
`NAV_TAKEOFF result=0`. Deci punctul 2 e un simptom al parametrului mort, nu
un motiv independent de a scoate valoarea de rezervă.

**Motivul care rămâne în picioare e punctul 1**, și e independent de
`WP_RFND_USE`: `get_alt_above_ground_m()` citește telemetrul direct, nu prin
`wp_nav`, deci nicio setare de navigație nu îl protejează. Plus argumentul de
la care a pornit A2: pe vehiculul real valoarea de rezervă ar fi o minciună
curată, fiindcă fără marker camera nu măsoară nimic.

### 5.10 Un parametru nu e setat până nu a fost citit înapoi

**Regulă de proces, nu sugestie.** ArduPilot acceptă tăcut `PARAM_SET` pe
nume inexistente: fără eroare, fără `COMMAND_ACK`, fără `PARAM_VALUE`.
Fișierul de parametri se încarcă „cu succes" și linia nu face absolut nimic.
La fel prin `--add-param-file`: SITL scrie `Loaded defaults from ...` chiar și
pentru linii pe care le ignoră complet.

Am pierdut așa `WP_RFND_USE` — scris `WPNAV_RFND_USE` — în **toate** cele 13
rulări din Faza 1 (§5.4, §5.7). Nimic nu a semnalat problema; a ieșit la
iveală abia când am citit parametrii înapoi și cererea a întors `None`.

Consecința pentru Compliance Matrix e mai gravă decât cea tehnică: fiecare
rând care spune „configurat prin parametru X" este o afirmație pe care
scrutineering-ul o poate cere demonstrată. Un parametru inexistent transformă
afirmația în abatere, nu în scăpare.

```bash
python3 tools/check_params.py                       # SITL pe TCP
python3 tools/check_params.py --conn /dev/serial0 --baud 921600
```

Unealta parcurge `config/nova_sitl.parm`, cere fiecare parametru de la FC și
raportează `LIPSESTE` pentru cele care nu răspund. Cod de ieșire 0 numai dacă
toate există **și** au valoarea cerută. De rulat înainte de orice campanie de
teste și înainte de scrutineering; ieșirea ei e dovada.

**Audit complet, după corectarea lui `WP_RFND_USE`: 18/18 parametri există și
se potrivesc.** Verificați explicit, fiindcă fuseseră scriși din memorie:
`SURFTRAK_MODE`, `TERRAIN_ENABLE`, `PLND_ENABLED`, `PLND_TYPE`,
`PLND_EST_TYPE`, `PLND_STRICT`, `PLND_ALT_MIN`, `PLND_YAW_ALIGN`,
`ARMING_SKIPCHK`. Toți reali.

Când un nume lipsește: caută `AP_GROUPINFO` în sursa ArduPilot pentru numele
scurt, apoi prefixul obiectului în `ArduCopter/Parameters.cpp` (`GOBJECT` /
`GOBJECTPTR`). Numele complet e prefix + nume scurt, iar prefixul **nu** e
neapărat cel din documentația veche.

#### Limita structurală: citirea înapoi confirmă valoarea, nu efectul

Auditul dovedește că parametrul **există** și **are valoarea cerută**. Nu
dovedește că valoarea e cea potrivită. Pentru măștile de biți diferența e
periculoasă: `FENCE_TYPE = 4` ar fi trecut orice audit — și ar fi stins tăcut
plafonul de altitudine cerut de 15.2.4, pentru că bitul 0 lipsea (§6/15.2.4).

**Pentru orice parametru-mască, documentează ce bit aprinzi, ce bit stingi și
efectul fiecăruia.** Nu doar valoarea finală. `check_params.py` descompune
acum măștile cunoscute și listează separat `APRINS` și `STINS`:

```
  ARMING_SKIPCHK       cerut 32768      citit 32768        ok
      = 0b1000000000000000
      APRINS : 15:rangefinder
      STINS  : 0:toate verificarile, 1:baro, 2:busola, ...
```

Lista `STINS` e cea care merită citită. `FENCE_TYPE` și `ARMING_SKIPCHK` sunt
în `BITMASKS`; orice mască nouă se adaugă acolo, altfel scapă neanalizată.

### 5.11 Un test de siguranță care nu poate eșua nu e test

În campania rundelor A–B, harness-ul de test a produs **trei** rezultate
false, toate în direcția „pare că merge":

1. o cursă între faze raporta a doua încercare autonomă ca reușită înainte ca
   noua comandă `LAND` să fie procesată — starea rămăsese `HANDBACK` din
   încercarea anterioară;
2. întrerupătorul de detecție din testul 15.2.9 se declanșa la 0 m, pe sol,
   înainte de decolare (`alt <= 6.0` e adevărat și la sol), deci „pierderea
   detecției" se producea într-un scenariu care nu exista;
3. un timestamp `DISTANCE_SENSOR` construit din `time.time()*1000` depășea
   uint32 și arunca excepție — singurul care s-a manifestat zgomotos.

Două reguli, ambele scumpe de reînvățat:

- **Dovada vine din ce raportează FC-ul, nu din ce a decis codul nostru.**
  Un supervizor care a *comandat* BRAKE nu demonstrează că vehiculul s-a
  oprit. Testul trebuie să citească modul din `HEARTBEAT` și altitudinea din
  `GLOBAL_POSITION_INT`.
- **Fiecare test are nevoie de o precondiție care îl invalidează dacă
  scenariul nu s-a produs.** Testul 15.2.9 verifică acum explicit că tăierea
  detecției s-a făcut în aer, la altitudinea intenționată, și se declară
  INVALID altfel. Bug-ul 2 a fost prins exact de această verificare, nu de
  citirea rezultatului.
- **Fiecare test de siguranță are nevoie de un caz negativ.** Varianta „fără
  supervizor" a testului 15.2.9, care arată aterizarea la −0.03 m, e ce
  dovedește că testul chiar măsoară ceva. Un test care trece mereu nu e test;
  fără perechea lui, „a rămas în aer" putea însemna la fel de bine că
  scenariul nu s-a produs.
- **Un render sintetic e adevăr doar acolo unde l-ai construit.** Testul de
  calibrare (E1) randa tabla de șah printr-o homografie definită de cele 4
  colțuri exterioare, proiectate corect prin distorsiune. Cele 54 de colțuri
  interioare însă nu urmau distorsiunea radială — homografia e liniară — și
  unealta raporta RMS 1.9 px pe un set „perfect". Testul detectorului nu a
  suferit, pentru că ArUco folosește exact cele 4 colțuri. Renderul fidel
  (dezdistorsionare pixel cu pixel, intersecție cu planul, `remap`) a dus
  RMS-ul la 0.15 px. Dacă un test verifică un model, randarea trebuie să
  treacă prin **același** model, nu printr-o aproximare a lui.
- **O singură poză proastă din 24 duce RMS-ul de la 0.15 la 1.8 px.** În
  setul sintetic, o poză înclinată în colțul cadrului avea un colț localizat
  greșit cu 8 px. Fără respingerea aberanțelor unealta ar fi refuzat un set
  altfel bun — sau, mai rău, l-ar fi acceptat cu coeficienți trași de un punct
  fals. `calibrate_camera.py` scoate acum pozele cu eroare peste
  `max(1.0 px, 2.5 × mediana)` și recalibrează. Pe teren cauza e o poză
  mișcată; simptomul e identic.

### 5.12 Maparea axelor de gamepad diferă între dispozitive

`tools/gamepad_rc.py` are `AXIS_ROLL = 2`. Pe DualSense cu `hid-playstation`,
**axa 2 este trigger-ul L2**, nu stick-ul drept: în repaus raportează −1.0,
adică PWM 1000 constant pe roll. Stick-ul drept e pe axele **3 și 4**.

Maparea corectă pe acest dispozitiv:

```
axa 0 = stick stânga X (yaw)      axa 3 = stick dreapta X (roll)
axa 1 = stick stânga Y (throttle) axa 4 = stick dreapta Y (pitch)
axa 2 = L2                        axa 5 = R2
```

Descoperit rulând `tools/calibrate_sticks.py`, care a raportat roll blocat la
1000 PWM.

**Maparea nu mai e în cod.** `gamepad_rc.py` o citește din
`config/gamepad.json`, generat de `--calibrate`. Asistentul calibrează după
**intenție**, nu după convenție: cere „împinge manșa de pitch înainte" și
deduce singur indexul și semnul, deci iese corect indiferent cum numerotează
SDL axele. `--preset dualsense` scrie un punct de plecare pentru start rapid,
iar `--check` arată valorile mapate fără MAVLink — de rulat înainte de orice
zbor.

Verificat end-to-end în SITL: manete în repaus → 1500 pe toate patru,
AUX 7 = 2000 (peste pragul de 1700 al porții), canalul de mod 1500 → FC
raportează LOITER.

Tot de acolo: valorile de repaus nu sunt exact centrate (0.0196, 0.0039,
−0.0275 pe axe), adică până la ~14 PWM offset. Încă un motiv pentru care
referința de neutru se memorează, nu se presupune 1500 (§6/15.3.1).

### 5.13 `--add-param-file` nu suprascrie ce e deja în eeprom-ul SITL

Continuarea directă a §5.10, și mai perfidă: acolo parametrul nu exista;
aici **există, e scris corect în fișier, iar FC-ul raportează altceva**.

`sim_vehicle.py --add-param-file` se traduce în `--defaults` pentru binarul
SITL, adică setează **valori implicite**. O valoare deja salvată în
`eeprom.bin` are prioritate. Fișierul de parametri se încarcă „cu succes",
iar modificarea făcută în el pur și simplu nu ajunge pe vehicul.

**Dovadă (SITL):**

```
pas 1: param set ARMING_SKIPCHK 0        -> salvat in eeprom
pas 2: repornire cu acelasi --defaults care cere 32768
       FC raporteaza ARMING_SKIPCHK = 0
```

Simptomul care ne-a costat: cu verificarea de rangefinder reactivată,
armarea eșuează cu `Arm: Rangefinder 1: No Data` și `ACK result=4`. Mesajul
arată ca o problemă de senzor; e de fapt un parametru neaplicat.

```
ARMING_SKIPCHK=0      -> Arm: Rangefinder 1: No Data   result=4  armat=False
ARMING_SKIPCHK=32768  -> Arming motors                 result=0  armat=True
```

Deci `ARMING_SKIPCHK,32768` (bit 15 = `RANGEFINDER = 1U << 15`, verificat în
`AP_Arming.h`) e corect — dar numai dacă ajunge pe FC.

**Consecințe de proces:**

- După **orice** modificare în `config/nova_sitl.parm`, pornește cu
  `./start_sim.sh --wipe`. Fără asta, modificarea poate fi invizibilă.
- `start_sim.sh` rulează acum `tools/check_params.py` automat după pornire,
  pe un port dedicat (14554), și afișează un avertisment mare dacă ceva nu
  s-a aplicat. Lecția din §5.10 mutată în lansator: nu presupune, citește
  înapoi — de fiecare dată, nu doar când te gândești la asta.
- Pe vehiculul real, aceeași regulă cu Mission Planner / QGC: încărcarea unui
  fișier de parametri nu garantează că toți au fost scriși.

### 5.14 Testele acopereau piesele, nu cablajul — Safety Supervisor inert

Raportat din zbor: **manșa mișcată la maxim în timpul aterizării autonome
mișca drona, dar nu oprea secvența.** Cauza nu era în monitorul de override,
care avea 8 teste proprii și o măsurătoare SITL de 150 ms. Era în **cablajul
aplicației**.

`tools/fake_detector.py` arma supervizorul pe o *tranziție*:

```python
if info['new'] == State.DESCEND_TRACK and info['old'] == State.IDLE:
    self.sup.arm(...)
```

Când intrarea a devenit
`IDLE → HANDOVER_CHECK → ACQUIRE → DESCEND_TRACK`, tranziția
`IDLE → DESCEND_TRACK` a dispărut. Condiția nu s-a mai potrivit niciodată,
`arm()` nu s-a mai apelat, iar `update()` ieșea imediat pe `if not
self.armed`. **Toate** monitoarele au rămas oprite — override, vârsta
detecției, rază, plafon, înclinare, rată de coborâre — fără niciun mesaj.

De ce nu l-a prins nimic:

- testele de supervizor își armau singure obiectul (`sup.arm(...)`);
- testele SITL B2/B3 la fel, cu `if sm.state == DESCEND_TRACK`, deci
  **pe stare, nu pe tranziție** — exact diferența care s-a rupt;
- testul de poartă nu folosea deloc supervizor;
- **nicio suită nu instanția cablajul din aplicație.**

**Reparat prin eliminarea deciziei, nu prin corectarea ei.** Supervizorul se
armează și se dezarmează singur din faza primită în `update()`
(`AUTONOMOUS_PHASES`). Aplicația nu mai trebuie să ghicească tranziția
corectă, deci nu o mai poate greși la următoarea restructurare.

Verificat în SITL cu cablajul real: manșă la 400 PWM peste neutru la 3.99 m →
`OVERRIDE` după 100 ms → FC în LOITER la **202 ms** de la mișcare → secvența
oprită în `IDLE`.

**Regula care lipsea:** o suită care testează fiecare piesă separat nu spune
nimic despre felul în care sunt legate. `tools/test_state_machine.py` are
acum `build_app()` / `run_app()`, care reproduc exact ordinea din
`run_loop()` cu același `OverrideMonitor` partajat între poartă și
supervizor, plus două teste de regresie pe ele.

### 5.15 Camera Module 3: „dual-stream" la 30 fps nu există

Tabelul din E1.1 cere `lores` 2304×1296 la 30 fps **și** `main` 4608×2592 la
cerere. Pe IMX708 astea sunt **moduri de senzor diferite**: 4608×2592 merge la
~14 fps, 2304×1296 (binned 2×2) la ~56 fps. Ambele fluxuri picamera2 vin din
același mod de senzor, iar ISP-ul poate doar să scaleze în jos — în modul
binned nu există de unde scoate un cadru nativ.

Consecință: `nova/detector_pi.py` rulează tracking-ul la 2304×1296 din modul
binned, iar cadrul de scoring se obține prin **comutare de mod la cerere**
(`PiCameraSource.capture_scoring_frame`, `switch_mode_and_capture_array`),
care oprește fluxul de tracking ~0.3–0.5 s. Exact de aceea 8.3.3 se rezolvă
cu ring buffer (grupul C), nu cu o captură sincronă la contact.

Cifrele sunt din fișa senzorului, nu măsurate — se confirmă în E2 (FPS
efectiv, temperatură, durata comutării).

Tot de aici: controalele camerei (`LensPosition`, `ExposureTime`,
`AnalogueGain`) pot fi acceptate și apoi **limitate tăcut** de driver.
`PiCameraSource` le citește înapoi din metadate și avertizează — §5.10
aplicat camerei.

### 5.16 E0: garda de autonomie e în poartă, nu în mașina de stări

`config/nova.json: autonomy_enabled` e citit de `HandoverGate` **la fiecare
cerere de handover**, înaintea oricărei alte condiții și fără să aștepte
fereastra de așezare. Fals ⇒ `REJECT` imediat, cu motiv explicit, fără neutru
memorat. Implicit fals; lipsa fișierului e fals; `"true"` ca string e fals.
Nu există activare din linia de comandă sau din mediu — doar din fișierul
versionat, cu commit.

Simularea (`fake_detector.py`) ocolește garda **explicit** cu
`autonomy_enabled=True` și o spune la pornire; `nova_pi.py` nu are această
opțiune. Așa `nova/state_machine.py` și `nova/safety.py` rămân neatinse (sunt
validate), iar garda are trei teste, inclusiv cel care verifică că fișierul
din repo e în starea închisă.

---

## 6. Cerințe care constrâng software-ul

Referințele sunt la `ZDC_Regulations_V0_5.pdf`.

### 15.2.5 — interdicția GNSS (RISCUL CEL MAI MARE, NEREZOLVAT)

Din momentul activării autonome, **niciun estimator sau filtru care
contribuie la ghidare sau control** nu are voie să primească date GNSS.

Configurația testată de noi **NU respectă** asta: LAND cu PLND folosește
controlerul de poziție orizontală → EKF3 → care fuzionează GPS implicit.

Mecanism candidat: seturi de surse EKF comutabile
(`EK3_SRC1_*`, `EK3_SRC2_*`, `EK3_SRC3_*`), cu `EK3_SRC2_POSXY = 0`.
Comutare la handover prin `RCx_OPTION 90` sau MAVLink.

**Netestat.** Riscul: fără GPS și fără flux optic, poziția orizontală
derivă rapid pe inerțial pur. Poate impune hardware suplimentar.
Verificat la scrutineering.

### 15.2.7 — urcare la 5 m după touchdown (IMPLEMENTAT, DE VALIDAT)

Secvența trebuie să se încheie cu stabilizare ≥1 s și urcare la ≥5 m
deasupra markerului. Oprirea mai jos invalidează încercarea.

În toate cele 13 rulări, logurile arată:
```
AP: PrecLand: Target Lost
AP: Disarming motors
```
ArduPilot dezarmează imediat după aterizare în LAND. **Secvența noastră
se oprește exact unde regula cere să continue** — proba ar fi zero, în
ciuda erorii de 0.8 cm.

Două arhitecturi:
- **A** — rămâi în LAND, împiedici dezarmarea, comuți în GUIDED și urci
  fără re-armare (preferată, modificare mică)
- **B** — toată coborârea în GUIDED cu setpoint-uri proprii; control
  total dar renunți la fuziunea PLND validată

**Aleasă A**, implementată în `fake_detector.py` ca stările
`TOUCHDOWN_CONFIRM` și `ASCENT`. Dezarmarea nu se poate întârzia din
parametri (§5.6); secvența reală e:

1. contact detectat independent (altitudine sub prag + viteză verticală
   stinsă, menținute 0.2 s) → `TOUCHDOWN_CONFIRM`, raport de aterizare
2. la `landed_state == ON_GROUND` (adică `land_complete`) → `DO_SET_MODE`
   GUIDED, în fereastra de spool-down, înainte de dezarmare
3. pauză ≥1 s de la contact, pe sol, în GUIDED (`make_safe_ground_handling`,
   motoare la ground idle, fără dezarmare)
4. `NAV_TAKEOFF` la `relative_alt(contact) + 5 m + 0.3 m` → `ASCENT`
5. confirmare la ≥5 m peste planul markerului, măsurat pe `LOCAL_POSITION_NED`
   → `HANDBACK`

Praguri reglabile: `--hold-s` (implicit 1.2 s), `--ascent-alt`,
`--ascent-margin`, `--touchdown-alt`, `--touchdown-vz`. `--no-ascent`
reproduce comportamentul validat în Faza 1.

**Deriva laterală în ASCENT, 4 rulări SITL.** Întrebarea era dacă urcarea e
verticală: dacă deriva ar crește cu altitudinea, pilotul ar prelua la handback
un vehicul care se mișcă lateral.

| AGL | run1 | run2 | run3 | run4 |
|---|---|---|---|---|
| contact | 4.8 | 3.6 | 4.4 | 6.6 |
| 2.0 m | 6.6 | 6.6 | 6.5 | 8.9 |
| 3.0 m | 6.8 | 6.1 | 6.7 | 9.8 |
| 4.5 m | 7.1 | 4.9 | 5.2 | 8.4 |
| handback | 7.0 | 4.7 | 4.5 | 7.8 |

Toate în cm. Tiparul e același în toate rulările: un tranzitoriu de ~3 cm care
culminează la 2–3 m AGL și apoi **se resoarbe**. Deriva nu crește cu
altitudinea, deci urcarea e verticală; ce rămâne e eroarea de la contact dusă
mai sus. Delta contact → handback: **+0.1 … +2.2 cm**.

Acceptabil: 15.2.7 cere doar încheierea la ≥5 m deasupra markerului, fără
constrângere de precizie, iar vehiculul predat pilotului e practic staționar.
De reverificat pe hardware — tranzitoriul vine din trecerea LAND → GUIDED →
`NAV_TAKEOFF` și depinde de reglajul controlerului de poziție.

### 15.2.9 / 15.2.10 — Safety Supervisor determinist (MONITOR DE DETECȚIE VALIDAT)

Sub pragul de încredere al detecției, aeronava trebuie să **planeze**,
nu să aterizeze. 15.2.10 cere explicit un supervizor determinist rulând
în paralel.

**De ce nu ne putem baza pe ArduPilot:** cu `PLND_STRICT 2` și pierdere
totală a detecției, vehiculul a coborât de la 6.6 m și a aterizat cu 33.4 cm
eroare, în loc să rămână în hover. ArduPilot a raportat
`PrecLand: Failsafe Measures` apoi `Disarming motors`.

Implementat în `nova/safety.py`; monitoarele și pragurile sunt în §8.

**Testul care picase, refăcut în SITL.** Detecția tăiată complet la 6 m, în
`DESCEND_TRACK`, aceeași configurație (`PLND_STRICT 2`):

| | fără supervizor | cu supervizor |
|---|---|---|
| tăiere | 5.99 m | 6.00 m |
| altitudine minimă după | **−0.03 m — a aterizat** | **4.99 m** |
| coborâre după tăiere | 6.02 m | 1.00 m |
| mod raportat de FC | GUIDED (secvența a continuat) | **BRAKE** |
| încă armat | da | da |

FC-ul a confirmat BRAKE la **0.48 s** de la tăiere; vehiculul a mai coborât
1.00 m până s-a oprit, apoi a plutit acolo restul celor 20 s de observație.
Cifrele astea merg direct în Safety Case: prag `DETECTION_MAX_AGE_S = 0.5 s`,
latență până la mod confirmat 0.48 s, pierdere de altitudine la frânare 1.00 m.

Verificarea e făcută din `HEARTBEAT`-ul FC-ului și `GLOBAL_POSITION_INT`, nu
din ce a decis supervizorul — vezi §5.11.

#### Risc rezidual: autoritatea de frânare sub ~1.2 m AGL

**De declarat în Safety Case, Anexa B Partea 1.** Un risc cuantificat și
acceptat se punctează mai bine decât unul nemenționat.

Monitorul se aplică în `DESCEND_TRACK`, care coboară până la ~0.45 m. Dar
frânarea consumă 1.00 m de altitudine (măsurat, la 0.5 m/s). Deci dacă
detecția se pierde la 0.8 m, supervizorul comandă BRAKE corect și la timp,
iar vehiculul **atinge totuși solul**.

> **Garanția „planează în loc să aterizeze" nu se aplică sub ~1.2 m AGL**
> (1.00 m distanță de frânare măsurată la 0.5 m/s, plus marjă). Sub acest
> prag, pierderea detecției duce la contact, nu la hover.
>
> Risc acceptat. Intenția 15.2.9 este prevenirea aterizării oarbe *de la
> altitudine*; un contact de la 0.8 m nu constituie pericol. La acea
> înălțime eroarea laterală a fost sub 3 cm în toate rulările, iar amprenta
> camerei (184×99 mm la 74.5 mm) e integral pe marker pentru orice eroare
> sub ~19 cm — deci nici cerința 8.3.3 nu e compromisă.

#### Constrângere pentru D1 (profil de coborâre agresiv)

Cei 1.00 m sunt măsurați **la 0.5 m/s**. Distanța de frânare are două
componente: reacția, liniară în viteză (0.48 s × v), și decelerarea,
aproximativ pătratică. La 2.0 m/s — treapta propusă de D1 peste 8 m — doar
reacția înseamnă 0.96 m, iar totalul e de așteptat în zona 2.5–3 m.

**Pragul sub care garanția de hover nu mai ține urcă odată cu viteza.** Dacă
profilul se accelerează fără reevaluare, se poate ajunge ca detecția pierdută
la 3 m să ducă tot la contact.

Când se implementează D1: măsoară distanța de frânare la **fiecare treaptă**
de viteză din profil, nu doar la 0.5 m/s. Rezultatul probabil impune limitarea
vitezei în funcție de altitudine, ceea ce e oricum sănătos — și e exact
relația care trebuie să apară în Safety Case, nu o singură cifră.

**Încă nescrise:** detecția de override (15.3.1 / 15.1.7) și încărcarea
fence-ului circular prin protocolul de misiune (15.2.4). Monitorul de rază și
cel de plafon există deja în supervizor ca plasă de siguranță redundantă, dar
fence-ul din firmware nu e încă încărcat.

### 15.3.1 — override sub 250 ms (MĂSURAT: 150 ms)

În GUIDED, ArduPilot **ignoră** intrările de manșă. Companion-ul trebuie
să le detecteze din `RC_CHANNELS` și să comande schimbarea de mod.
Implementat în `nova/rc.py` + `nova/safety.py: _mon_override`.

Bugetul: la 10 Hz implicit, doar streaming-ul consumă 100 ms. `Vehicle`
cere acum `RC_CHANNELS` la **50 Hz** (`RC_HZ`).

**Măsurat în SITL**, pilot simulat prin `RC_CHANNELS_OVERRIDE`, detecție doar
din `RC_CHANNELS` raportat de FC:

| | |
|---|---|
| injecție → mod confirmat de FC | **150 ms** |
| din care în companion (prima depășire → confirmare) | 106 ms |
| din care calea RC → `RC_CHANNELS` → companion | 45 ms |
| buget 15.3.1 | 250 ms |

**Formularea pentru Compliance Matrix contează.** Din cele 106 ms interne,
100 ms sunt `OVERRIDE_HOLD_S` — o *decizie de design* care cumpără imunitate
la zgomot, nu timp de procesare. Latența tehnică e **sub 50 ms**; restul e un
parametru reglabil. O marjă care e un parametru se apără mult mai bine în
fața juriului decât una care e un plafon de performanță: dacă se cere mai
repede, se scade persistența și se recalibrează deadband-ul, fără schimbări
de arhitectură.

**Dublă protecție împotriva declanșării false:** amplitudine
(`STICK_DEADBAND_PWM` 80) **și** persistență (`OVERRIDE_HOLD_S` 100 ms).
Verificat că 30 PWM susținut 3 s nu declanșează nimic, și că un vârf de
50 ms nu declanșează.

**Referința de neutru nu e 1500.** Se memorează la validarea handover-ului,
după fereastra de așezare, din ce raportează emițătorul atunci. Măsurat pe
DualSense: offset-uri de până la 13.7 PWM față de 1500, cu `RC3_TRIM` la
1100 pe throttle. Comparația se face față de neutrul memorat, iar la
HANDOVER_CHECK față de `RC1_TRIM`…`RC4_TRIM` citite din FC.

**Zăvor ireversibil:** după override, companion-ul rămâne pasiv pentru restul
încercării, chiar dacă manșa revine la neutru. Override-ul e singura acțiune
care poate **escalada peste un zăvor deja închis** — inclusiv peste un RTL
comandat de supervizor. 15.1.7 dă pilotului autoritate oricând, iar un RTL
peste un pilot care zboară activ s-ar bate cu el.

**Soluția robustă rămâne** abort pe un comutator mapat direct pe un mod de
zbor prin `FLTMODE_CH`: comutarea o face FC-ul și nu trece prin Pi deloc.
Detecția pe manșe e necesară pentru 15.1.7, dar nu e singurul strat.

#### Risc rezidual: override-ul anulează un RTL de geofence

**De declarat în Safety Case, Anexa B Partea 1.**

Override-ul escaladează peste orice zăvor, inclusiv peste un RTL comandat de
supervizor la breach de geofence. Decizia e susținută de 6.2.1.4, care spune
că la un avertisment de limită echipa poate fie să zboare înapoi în traseu,
fie să declanșeze RTH — deci un pilot care preia manual și corectează e
explicit permis.

> **Riscul:** cu `STICK_DEADBAND_PWM = 80` și `OVERRIDE_HOLD_S = 100 ms`, o
> atingere accidentală de manșă în timpul unui breach de geofence anulează
> RTL-ul automat și lasă vehiculul în LOITER, în afara limitei. Puțin
> probabil — 80 PWM e ~2.7× zgomotul tipic și cere 100 ms susținuți — dar
> posibil.
>
> Atenuare: LOITER menține poziția, deci vehiculul nu se îndepărtează mai
> mult, și rămâne complet pilotabil. Semnalizarea către pilot trebuie să fie
> fără echivoc, pentru că el trebuie să acționeze imediat: evenimentul
> `pilot_override` intră în logul supervizorului cu `time_boot_ms`, iar
> aplicația trimite `STATUSTEXT` cu severitate WARNING.
>
> Risc acceptat: autoritatea pilotului peste automatisme e cerută de 15.1.7,
> iar alternativa — un RTL care se bate cu pilotul pe comenzi — e mai
> periculoasă decât riscul de mai sus.

**Rămas deschis pe hardware:** `STICK_DEADBAND_PWM = 80` e PROVIZORIU.
`tools/calibrate_sticks.py` măsoară zgomotul în repaus și recomandă pragul
(max observat + 3σ). Rulat pe DualSense: **σ = 0.00, amplitudine 0 PWM** —
dar asta e un gamepad cu ieșire cuantizată, nu un emițător analogic cu
gimbal-uri și link RC. **Cifra nu se transferă.** Valoarea finală se măsoară
pe emițătorul de concurs și se documentează în Safety Case împreună cu
metoda.

### 15.2.4 — geofence 10 m + plafon 30 m (IMPLEMENTAT ȘI VALIDAT)

Monitorizare onboard, în FC sau mission computer independent.
Implementat în `nova/fence.py`; două straturi, deliberat:

1. **În firmware.** Companion-ul încarcă înainte de handover un cerc de
   incluziune de 10 m centrat pe **marker**, prin `MAV_MISSION_TYPE_FENCE`,
   cu `FENCE_ACTION = 1` (RTL or Land). De acolo monitorizarea e determinist
   în FC și nu depinde de Pi.
2. **În supervizor.** `_mon_radius` și `_mon_ceiling` recalculează din
   poziție, ca plasă de siguranță dacă încărcarea eșuează.

**Validat în SITL**, ciclul complet: fence de traseu de 120 m → salvare →
fence autonom de 10 m → verificare prin citire înapoi (centrul la **0.00 m**
de marker, rază 10 m) → restaurare → verificare că fence-ul de traseu s-a
întors.

**Capcana de ordine e rezolvată prin salvare/restaurare.** Fence-ul de limită
a traseului (16.3.2) e activ în restul turului; cel de 10 m doar în segmentul
autonom. `FenceManager.save()` salvează geometria **și** parametrii
`FENCE_*` înainte de încărcare, `restore()` îi pune înapoi la handback. Un
handback care lasă vehiculul fără fence de traseu e mai rău decât să nu fi
schimbat nimic.

**`FENCE_TYPE` e o mască de biți, și aici era o capcană.** Implicit e 7
(max alt + cerc pe home + incluziune/excluziune). Prima variantă a modulului
o seta la 4 — doar incluziune — ceea ce **stingea plafonul de altitudine
exact în segmentul în care 15.2.4 îl cere**. Corect e
`FENCE_TYPE_AUTONOM = 1 | 4 = 5`: cercul de incluziune pe marker plus
plafonul, fără cercul centrat pe home (care e `FENCE_RADIUS` în jurul
punctului de decolare, nu al markerului). Plus `FENCE_ALT_MAX = 30`.

A ieșit la iveală citind valoarea implicită de pe FC și întrebând ce anume
stingem — nu dintr-un test picat. Verificarea prin citire înapoi arată ce
valoare **are** parametrul, nu dacă valoarea e cea potrivită.

### 8.3.3 — imaginea de touchdown (REZOLVAT ARHITECTURAL)

Imagine la momentul contactului; aterizarea e validă dacă centrul
imaginii conține pixel de pe suprafața markerului.

**Problema:** la 74.5 mm, amprenta camerei e 184×99 mm. Markerul are
480 mm. Se vede sub un sfert din el — juriul nu poate măsura nimic.

**Soluția (fără modificări mecanice):** stare `SCORING_CAPTURE` care
declanșează captura full-res pe **dimensiunea markerului în pixeli**
(>980 px, deci ~0.42–0.45 m), nu pe altitudine. Plus ring buffer cu
ultimele ~2 s de cadre, salvat la contact.

Deriva laterală între captură și contact: **maxim 1.2 cm pe 13 rulări**.

### 6.2.1.30 — loguri

`.bin` și `.tlog` acceptate (confirmat de organizatori; păstrează
dovada scrisă). Imaginea de touchdown se predă în același set.

---

## 7. Elemente deschise

| # | Ce | Blocant pentru |
|---|---|---|
| 1 | Masa reală: CAD zice 2.483 kg, matricea zice 3 kg | 11.1, model SDF |
| 2 | TWR: estimat 4–5:1; recomandat 2.5–3:1 pentru precizie | 11.2 |
| 3 | Motor/elice/baterie nefinalizate | model SDF, buget termic |
| 4 | Test EKF fără GNSS în SITL | 15.2.5 |
| 5 | ~~Fix urcare la 5 m după touchdown~~ validat în SITL | 15.2.7 |
| 6 | ~~Safety Supervisor~~ detecție + override + geofence validate în SITL | 15.2.9, 15.3.1, 15.2.4 |
| 11 | Distanța de frânare la fiecare treaptă din profilul D1 | 15.2.9 + D1 |
| 12 | Ring buffer + imaginea de touchdown (grupul C) | 8.3.3 — **obligatoriu** |
| 13 | `STICK_DEADBAND_PWM` pe emițătorul de concurs | 15.3.1 |
| 14 | **E2**: validare offline a detectorului real (marker printat, ruletă, 3 condiții de lumină); până atunci `autonomy_enabled=false` | E0, 15.2.3 |
| 15 | Confirmare pe hardware: moduri de senzor IMX708 (30 fps binned, ~14 fps nativ), durata comutării de mod, controale aplicate, temperatură | §5.15, E2 |
| 16 | Latența detectorului pe Pi 4 (desktop: 4/6 ms; criteriu E2: p99 < 150 ms) | E1.4 |
| 7 | ~~Măsurare latență override~~ 150 ms în SITL; deadband de măsurat pe emițătorul de concurs | 15.3.1 |
| 8 | Mail organizatori: imagine scoring la 0.45 m | 8.3.3 |
| 9 | Model SDF cu inerția reală (avem tensorul din Onshape) | fidelitate sim |
| 10 | Detector ArUco real, testat offline | Faza 2 |

### Tensor de inerție din Onshape (FLU, în CG)

Masă 2.483 kg. Nasul dronei e pe +Y în CAD, deci rotație de +90° în
jurul lui Z pentru FLU:

```xml
<inertial>
  <pose>0.006635 -0.000633 -0.007392 0 0 0</pose>
  <mass>2.483</mass>
  <inertia>
    <ixx>0.0236573</ixx>  <ixy>0.0001205</ixy>  <ixz>0.0003358</ixz>
    <iyy>0.0256090</iyy>  <iyz>0.0000285</iyz>
    <izz>0.0460664</izz>
  </inertia>
</inertial>
```

Iyy (pitch) > Ixx (roll) cu 8% — nu copia câștigurile PID de pe o axă
pe alta.

---

## 8. Arhitectura țintă (companion)

```
IDLE
 └→ RACE_MONITOR        detector activ, ZERO comenzi, ring buffer
     └→ HANDOVER_CHECK  pe FRONTUL CRESCĂTOR al AUX (canal 7), după 1.0 s
         │              fereastră de așezare — SINGURA cale de intrare
         ├→ REJECT      alt <5m sau >12m, sau dist >6.5m, sau marker nedetectat,
         │              sau manșă în afara neutrului; iese doar cu AUX jos
         └→ ACQUIRE     LAND cerut de companion, așteaptă confirmarea FC
             └→ DESCEND_TRACK      LANDING_TARGET @ 20 Hz
                 └→ SCORING_CAPTURE    marker_px > 980, captură full-res
                     └→ FINAL_DESCENT  vertical, fără corecții laterale
                         └→ TOUCHDOWN_CONFIRM   contact + ≥1 s stabil
                             └→ ASCENT          ≥5 m AGL
                                 └→ HANDBACK

Safety Supervisor rulează în paralel pe tot parcursul, cu autoritate
de a comanda BRAKE / LOITER / RTL peste orice stare.
```

**Safety Supervisor** (`nova/safety.py`, 15.2.9 / 15.2.10). Rulează înaintea
mașinii de stări, în aceeași buclă, și primește **doar vârsta** ultimei
detecții — niciun pixel nu ajunge în logica de decizie. Pragurile sunt
constante numite la începutul fișierului, pentru transcriere directă în
Safety Case.

| Monitor | Prag | Acțiune | Se aplică în |
|---|---|---|---|
| Override pilot | `STICK_DEADBAND_PWM` 80, `OVERRIDE_HOLD_S` 0.1 s | LOITER + pasiv definitiv | **toate** fazele |
| Vârsta ultimei detecții | `DETECTION_MAX_AGE_S` 0.5 s | BRAKE | `DESCEND_TRACK`, `SCORING_CAPTURE` |
| Rază față de handover | `GEOFENCE_RADIUS_M` 10.0 m | RTL | tot segmentul autonom |
| Plafon AGL | `CEILING_AGL_M` 30.0 m | RTL | tot segmentul autonom |
| Rată de coborâre | `MAX_DESCENT_RATE_MS` 2.0 m/s, 0.5 s | BRAKE | tot segmentul autonom |
| Înclinare | `MAX_TILT_DEG` 30°, 0.3 s | BRAKE | tot segmentul autonom |

Trei proprietăți care nu sunt evidente din tabel:

- **Monitorul de detecție nu se aplică în `FINAL_DESCENT` și după.** Sub
  0.38 m markerul iese din cadru prin construcție (§5.2), deci un supervizor
  care ar cere detecție validă acolo ar aborta în ultimul metru la *fiecare*
  încercare — ar transforma o proprietate fizică a camerei într-o defecțiune
  inventată. Lista e scrisă pozitiv (`DETECTION_MONITORED_PHASES`), ca o fază
  nouă să nu fie supravegheată din greșeală.
- **Acțiunile se zăvorăsc.** Nu există revenire automată dacă semnalul
  redevine bun. Zăvorul are prioritate și față de dezarmarea supervizorului:
  altfel, când mașina de stări își încheie secvența ca urmare a comenzii,
  o comandă de mod pierdută ar rămâne definitiv nelivrată.
- **Confirmarea vine din FC, nu din decizia proprie.** Supervizorul reîncearcă
  `DO_SET_MODE` până când `HEARTBEAT` raportează modul cerut, și scrie în log
  separat `mode_confirm` de `mode_fail`. Ce a decis supervizorul nu e dovadă
  că vehiculul s-a oprit.
- **Override-ul e singurul care escaladează peste un zăvor.** Un breach de
  geofence se declanșează instantaneu, override-ul are nevoie de 100 ms de
  persistență — fără escaladare, RTL ar câștiga mereu cursa și am lupta cu
  pilotul pe comenzi. 15.1.7 îi dă autoritate oricând.

**Poarta de intrare** (`nova/handover.py`) e **singura** cale către segmentul
autonom. AUX comutat → 1.0 s fereastră de așezare → validare (manșe în neutru
față de `RCx_TRIM`, altitudine 5–12 m, distanță ≤6.5 m, marker văzut în
ultimele 0.3 s) → ACCEPT, moment în care se memorează referința de neutru.
Refuzul se semnalizează pilotului, cu motiv: un handover refuzat costă câteva
secunde, o încercare anulată în zbor costă 10 puncte.

Nu există cale alternativă, deliberat. Un „pilotul comandă LAND" păstrat ca
rezervă ar ocoli validarea, iar rândul din Compliance Matrix pentru 15.2.3 ar
fi o afirmație fără evidență — la scrutineering întrebarea evidentă ar fi
care dintre cele două căi a fost folosită în cursă. Cu poarta, logul arată
condițiile exacte de la activare, acceptate sau respinse, cu motivul.

Cererea e **frontul crescător**, nu starea comutatorului: altfel un comutator
lăsat sus ar reporni secvența imediat după orice ieșire din ea. `REJECT`
rămâne afișat până când pilotul lasă comutatorul jos.

**Throttle-ul nu se validează la fel ca celelalte axe.** Schița inițială
compara toate patru canalele cu `RCx_TRIM`. Primul handover rulat în SITL a
fost refuzat: *„canalul 3 la 500 PWM de trim"*. Cauza nu e un bug, ci o
diferență fizică — roll, pitch și yaw se auto-centrează, deci „liber"
înseamnă „aproape de trim"; throttle-ul nu. Pe un emițător real pilotul îl
ține la mijlocul cursei pentru hover, iar `RC3_TRIM` e adesea la capătul de
jos. Comparat cu trim-ul, un handover perfect normal ar fi refuzat **de
fiecare dată, pe teren** — în SITL diferența era 500 PWM.

Criteriul corect pentru throttle e **„nemișcat"**, nu „la trim": amplitudinea
măsurată pe fereastra de așezare, sub `STICK_DEADBAND_PWM`. Fereastra de 1.0 s
are astfel un al doilea rol, pe lângă cel de așteptare — e intervalul de
măsurare.

**Geofence** (`nova/fence.py`): cerc de incluziune de 10 m pe marker, încărcat
prin protocolul de misiune înainte de handover, cu salvarea și restaurarea
geometriei de traseu (16.3.2). Vezi §6/15.2.4.

Fiecare eveniment din log poartă `time_boot_ms` de la FC, nu ceasul Pi-ului,
ca să se poată alinia cu `.bin`/`.tlog` (6.2.1.30).

ACQUIRE confirmă **LAND**, nu GUIDED: diagrama de mai sus precede arhitectura
A. Cu A, coborârea se face în LAND cu PLND nativ (§6/15.2.7).

Plafonul de 12 m la handover e mai strict decât regulamentul (20 m),
deliberat: la 20 m markerul are 22 px, prea puțin pentru detecție 4×4
fiabilă. La 12 m are 37 px.

**Sub 0.4 m nu se mai fac corecții laterale.** Autoritatea de corecție
acolo e 2–4 cm; precizia se decide la 0.5–1.0 m. Coborârea finală e
verticală și lentă, ceea ce elimină și forfecarea de rolling shutter
(care e `v_lateral × T_readout`, deci ~15 mm la 0.5 m/s).

### Paritate sim ↔ hardware

Același cod pe desktop și pe Pi. Diferențele, toate în aplicația de intrare:

| | `tools/fake_detector.py` (sim) | `tools/nova_pi.py` (bord) |
|---|---|---|
| sursa de `Detection` | geometrie din poziția cunoscută a markerului | `nova/detector_pi.py`: picamera2 → ArUco → `solvePnP` |
| legătura | `udpin:127.0.0.1:14552` | `/dev/serial0` @ 921600 |
| garda E0 | ocolită explicit, anunțat la pornire | citită din `config/nova.json`, fără ocolire |

`nova/detector_pi.py` are trei straturi: `FrameSource` (picamera2 / director
de imagini / cadre din memorie — o singură interfață `read() → (gray, t)`),
`ArucoMarkerDetector` (numai OpenCV, testabil pe desktop cu markere randate la
poziție cunoscută) și `PiDetector` (fir separat + instrumentare: latență
captură→publicare ca p50/p99, FPS, rată de detecție).

**Convenția de montaj**, distinctă de rotația din §5.1: camera privește în
jos cu **vârful imaginii spre nas**, deci `înainte = −y_cam`,
`dreapta = +x_cam`. `range_m` e distanța pe axa optică până la **planul**
markerului (`(t·n)/n_z`), nu până la marker — ArduPilot înmulțește apoi cu
`cos(tilt)` (§5.3). Verificarea de încadrare (§5.2) se face cu `CameraModel`
derivat din calibrare (`fits_in_frame ⇔ marker_px ≤ 0.95·H`), nu din fișa
tehnică.

Pe desktop, cadru întreg 2304×1296: p50/p99 = 4/6 ms; cu ROI 640×480 sub 5 m:
1/3 ms. Nu e Pi 4 — criteriul E2 (p99 < 150 ms) se măsoară acolo.

---

## 9. Convenții

- Comentarii în cod: română, fără diacritice
- Nu introduce dependențe noi fără motiv; stack-ul e pymavlink + OpenCV
- **Fără ROS 2.** Decizie luată deliberat, pentru paritate sim/hardware
- Orice parametru ArduPilot nou merge în `config/nova_sitl.parm`, cu
  comentariu care explică **de ce** — fișierul e sursă de evidență
  pentru Compliance Matrix
- Orice descoperire empirică se adaugă la §5 al acestui fișier
