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
| Flight controller | **Pixhawk 6C** | ArduPilot (Copter 4.8.0-dev testat). Legatura cu Pi-ul pe TELEM2 = **SERIAL2** (`hwdef.dat:36`). Placa din matricea initiala era CUAV X7+ |
| Companion | Raspberry Pi 4 | Python + pymavlink + OpenCV |
| Cameră | Raspberry Pi Camera Module 3 **Wide** | IMX708, rolling shutter |
| Frame | Quad X | ~2.5–3.0 kg (vezi §7 — contradicție deschisă) |
| Baterii | 2× 6S LiPo 4000 mAh | 25.2 V max |

### Cameră — cifre de lucru

- Senzor IMX708, 4608×2592 nativ, pixel 1.4 um
- Obiectiv Wide: focală 2.75 mm, f/2.2
- **Măsurat pe camera noastră** (`config/camera_pi.yaml`, ChArUco, 60 poze):
  `fy = 1038.7 px`, **HFOV 96.0°, VFOV 63.9°**
- Fișa tehnică dă HFOV 102° / VFOV 67°, iar focala derivată din ea e 933 px.
  **Camera reală e cu ~11% mai îngustă.** Nu e o eroare de calibrare — e
  toleranța obiectivului, și e în direcția care strânge, nu care ajută.
- Rezoluție de lucru: **2304×1296**
- Cameră la **74.5 mm** deasupra solului la contact (tren de aterizare)
- Autofocus PDAF — **trebuie blocat manual**, altfel caută focus exact
  în timpul coborârii

Dimensiunea markerului (480 mm) în imagine: `marker_px = fy * 0.48 / Z`

| Altitudine | cu 933 (fișă) | **cu 1038.7 (măsurat)** |
|---|---|---|
| 20 m | 22 | **25** |
| 15 m | 30 | **33** |
| 10 m | 45 | **50** |
| 5 m | 90 | **100** |
| 1 m | 448 | **499** |
| 0.45 m | 995 | **1108** |

**Ce se schimbă și ce nu, din cauza celor 11%:**

| | efect |
|---|---|
| `Detection.fill`, deci pragurile de captură și de coborâre verticală | **nimic** — se măsoară din colțuri față de cadrul real, deci focala se simplifică |
| detecția la altitudine | **mai bună**: markerul e mai mare la aceeași înălțime |
| raza maximă de handover | **mai strâmtă cu ~11%**: 4.03 m la 12 m, nu 4.49; 1.54 m la 5 m, nu 1.73 |
| bugetul de înclinare al camerei | **mai strâmt**: 15.8° la 1 m cu 10 cm lateral, nu 19.5° |

Primele două rânduri sunt exact argumentul pentru care criteriul de captură
a fost mutat de pe pixeli pe încadrare (§5.57): un prag în pixeli ar fi
trebuit recalculat acum, iar unul pe `fill` nu.

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
│   ├── nova_flight.parm      # parametri ArduPilot VEHICUL REAL (neverificat pe hardware)
│   └── camera_pi.yaml        # calibrarea camerei, din calibrate_camera.py (evidență: se commit-uiește)
├── nova/                     # cod companion, identic sim ↔ Raspberry Pi
│   ├── config.py             # config/nova.json + garda E0
│   ├── detection.py          # Detection + modelul de cameră (contractul)
│   ├── detector_pi.py        # detectorul real + GazeboFrameSource (§5.36)
│   ├── vehicle.py            # legătura MAVLink: telemetrie + comenzi + parametri
│   ├── handover.py           # poarta de intrare (singura) + E0
│   ├── safety.py             # Safety Supervisor
│   ├── rc.py                 # override pe manșe
│   ├── fence.py              # geofence prin protocolul de misiune
│   ├── serial_guard.py       # cine ocupă /dev/serial0 (§5.27)
│   ├── preview.py            # previzualizare: banc / VNC / bord (§5.28)
│   ├── race_screen.py        # ecranul de concurs (§5.29)
│   ├── authority.py          # modulare de autoritate pe praguri (§5.30)
│   ├── sim_truth.py          # adevarul din Gazebo, pentru validare (I4)
│   ├── ekf_source.py         # 15.2.5: surse EKF fara GNSS (§5.53)
│   ├── frame_ring.py         # ultimele N cadre, cu timestamp (8.3.3)
│   ├── scoring.py            # imaginile predate juriului + evidenta (§5.55)
│   └── state_machine.py      # mașina de stări a segmentului autonom
├── tools/
│   ├── nova_pi.py            # aplicația de BORD (detector real, fără ocolire E0)
│   ├── nova_service.py       # serviciul RACE_MONITOR (fără comenzi) + unitatea systemd
│   ├── setup_pi.sh           # instalare pe Raspberry Pi OS (Trixie/Bookworm)
│   ├── preflight_check.py    # verificare de banc; cod 0 doar dacă toate trec
│   ├── run_e2.py             # colectarea interactivă a datelor E2
│   ├── start_flight.sh       # pornire completa pe Pi: venv, port, E0, race_mode
│   ├── sim_handover.py       # declanseaza poarta in SITL, fara gamepad (§5.32)
│   ├── make_marker_model.py  # modelul Gazebo al markerului (§5.31)
│   ├── make_camera_model.py  # senzorul de camera, din calibrare (§5.33)
│   ├── measure_rtf.py        # factorul de timp real, fara pornire (§5.33)
│   ├── setup_sim_venv.sh     # mediul cu gz-transport + OpenCV 4.10 (§5.36)
│   ├── gz_frames.py          # verifica sursa de cadre din Gazebo (I3)
│   ├── check_handover_fov.py # incape markerul in cadru la handover? (§5.42)
│   ├── nova_sim.py           # aplicatia de SIM cu cadre din Gazebo (I4)
│   ├── sim_fly_to.py         # partea "manuala": decolare + pozitionare (I4)
│   ├── batch_sim.py          # campanie de rulari cu conditii variate (I4)
│   ├── race_mode.py          # ziua cursei: preflight + un singur ecran
│   ├── collect_session.py    # evidența 6.2.1.30: .bin, loguri, cadre, manifest
│   ├── fake_detector.py      # detector sintetic + aplicația de SIM (ocolește E0)
│   ├── calibrate_camera.py   # E1.2: ChArUco/tablă de șah → camera_pi.yaml
│   ├── calibrate_sticks.py   # zgomotul manșelor → deadband
│   ├── check_params.py       # citire înapoi a parametrilor (§5.10)
│   ├── check_rc_override.py  # RC_CHANNELS_OVERRIDE se reflectă în RC_CHANNELS?
│   ├── gamepad_rc.py         # punte gamepad → RC_CHANNELS_OVERRIDE
│   └── test_*.py             # suite offline: state_machine, safety, handover,
│                             #   detector_pi, calibrate_camera, link, ops,
│                             #   pi_tooling, make_calib_target,
│                             #   authority, sim_handover, marker_model,
│                             #   camera_model, gz_source, sim_loop,
│                             #   ekf_source, scoring
├── sim/
│   ├── models/aruco_26/      # marker ArUco 26, generat (nu edita de mână)
│   └── worlds/nova_marker.sdf  # derivată din iris_runway.sdf
├── docs/
│   ├── CHECKLIST_TEREN.md    # checklist + tabel simptom → cauză → fix
│   ├── ZBOR_FINAL.md         # segmentul autonom in concurs (pilot + operator)
│   ├── LIMITE_SIM.md         # ce NU poate spune Gazebo (I6)
│   └── DIAGNOSTIC_OSCILATIE.md  # oscilatia de pendul, un parametru pe rulare (I5)
├── pi/                       # bring-up pe hardware (Pi 4 + Pixhawk 6C)
│   ├── README.md             # runbook cu comenzile ssh, cap-coada
│   ├── setup_uart.sh         # GPIO 14/15: miniUART -> PL011, consola, dialout
│   ├── bringup.sh            # verificari + monitor cu fereastra fullscreen
│   ├── descent_test.sh       # proba de coborare autonoma pe vehiculul de test
│   ├── install.sh            # serviciul de utilizator, pornit la fiecare boot
│   └── nova-bringup.service  # unitate systemd (graphical-session)
├── systemd/
│   └── nova-monitor.service  # generat de nova_service.py --install-unit
├── requirements-pi.txt       # pip comun (fără picamera2/numpy/opencv)
├── requirements-pi-bookworm.txt  # + OpenCV din pip (§5.24)
└── docs/
```

**Două stive Python de producție, nu una.** Desktopul rulează Ubuntu 22.04 cu
OpenCV 5.0; vehiculul rulează **Raspberry Pi OS Trixie**, Python 3.13, cu
numpy 2.2.4 și OpenCV 4.10 **din apt**. Pe Pi nimic nu se instalează peste
pachetele de sistem, pentru că picamera2 e compilat împotriva lor — vezi
§5.24 pentru regula completă și pentru ce diferă între versiuni, măsurat.

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
| 14560 | `sim_fly_to.py` (campanie `batch_sim.py`) |
| 14561 | `sim_handover.py` (campanie) |
| 14562 | `nova_sim.py` (campanie) |

Porturile de campanie sunt separate deliberat de cele interactive: o
campanie pornita peste o sesiune `start_sim.sh` deschisa nu are voie sa
consume mesajele altcuiva. Un test verifica faptul ca cele doua multimi nu
se intersecteaza.

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

## 4b. Faza 3 — bucla închisă în Gazebo (prima secvență completă)

**21 septembrie 2026: segmentul autonom parcurs cap-coadă, cu pixeli reali.**

```
IDLE → HANDOVER_CHECK (10.91 m) → ACQUIRE → DESCEND_TRACK
  → SCORING_CAPTURE     alt 0.570 m, 982 px
  → FINAL_DESCENT       alt 0.51 m
  → TOUCHDOWN_CONFIRM   alt 0.190 m
  → ASCENT              pauză 1.52 s pe sol
  → HANDBACK            5.05 m deasupra markerului
```

Fiecare condiție de regulament, îndeplinită: captura de scoring peste pragul
de 980 px (8.3.3), pauza de 1.52 s ≥ 1.0 s și urcarea la 5.05 m ≥ 5 m
deasupra markerului (15.2.7).

Diferența față de Faza 1 e ce contează: acolo detecțiile veneau din
geometrie, corecte prin construcție. Aici trec prin randare, distorsiune,
cuantizare și `detectMarkers` → `solvePnP` real. Lanțul complet —
randare → detecție → `LANDING_TARGET` → controler → mișcare → randare — e
închis.

**Ce a mers, măsurat pe parcurs:** `det 95–100%` de la 10.9 m până sub 1 m;
`marker_px` 41 → 982; `range` în acord cu altitudinea pe tot parcursul
(la 5.07 m range → 88.4 px, geometria dă 88.7).

> **Atenție la citirea pragului de scoring.** Mesajul de tranziție tipărește
> `alt` (a vehiculului), iar pragul de 980 px corespunde lui `range`
> (cameră → planul markerului). Diferența e montajul camerei plus planul
> markerului: 0.570 − 0.0745 − 0.01 ≈ 0.46 m, adică exact fereastra de
> 0.42–0.45 m din §6/8.3.3. Nu sunt două cifre în contradicție.

**Ce NU spune rularea asta.** O singură rulare, o singură condiție
(vânt 2.4 m/s, soare la 43.6°, marker mat), pe vehiculul iris, nu NOVA. Nu
există încă distribuții — p50 și p95 cer campania. Și nu există **nicio**
cifră de eroare finală sau derivă: rularea a fost oprită din afară înainte
să-și scrie raportul, iar acela se scrie la ieșire (reparat: §5.47).

Elementele deschise 22 și 23 din §7 rămân, cu domeniul redus: bucla merge,
cifrele lipsesc.

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

> **Cifra de 0.38 m are trei calificative, adăugate pe rând, fiecare
> plătit cu o rundă de depanare.** E valabilă doar la nadir, cu eroare
> laterală zero **și** cu markerul nerotit în cadru:
>
> | ce lipsea | cât mută pragul | unde |
> |---|---|---|
> | înclinarea vehiculului și eroarea laterală | 0.38 → ~1 m | §5.45 |
> | rotația markerului în cadru (cutia, nu latura) | ×1.41 la 45° | §5.49 |
> | `frame_h_px` derivat din VFOV, nu pixelii reali | ~5% | §5.57 |
>
> Detectorul folosește acum `fill` — cutia colțurilor față de cadrul real —
> deci verificarea nu mai depinde de niciunul dintre cele trei. Formula de
> mai sus rămâne pentru intuiție, nu ca criteriu.

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

### 5.17 Grila pătrată de calibrare: ambiguă la rotație, dar **nu** strică intrinsecii

Sfatul obișnuit e „folosește o grilă asimetrică, altfel apar erori tăcute în
calibrare". Măsurat pe date sintetice, jumătatea a doua e falsă.

**Ce se schimbă cu adevărat.** Pe o grilă pătrată (8×8 colțuri interioare),
permutarea *colț detectat → colț fizic* depinde de cum cade tabla în cadru:

| rotație | 9×6 pătrate (8×5 colțuri) | 9×9 pătrate (8×8 colțuri) |
|---|---|---|
| 0° | permutare P | permutare identitate |
| 90° | **aceeași P** | alta (64/64 indici diferă) |
| 180° | **aceeași P** | inversă |
| 270° | **aceeași P** | alta |

Pe grila asimetrică ordinea e legată de tablă și se rotește odată cu ea.

**Ce NU se schimbă.** 25 de vederi sintetice, aceleași poze, aceeași cameră
(f = 932.87 px, k1 = −0.050):

| | fx recuperat | k1 recuperat | RMS |
|---|---|---|---|
| 9×6 pătrate | 932.97 (**+0.01%**) | −0.05014 (+0.3%) | 0.103 px |
| 9×9 pătrate | 933.40 (**+0.06%**) | −0.05037 (+0.7%) | 0.108 px |

Motivul e simplu odată văzut: o permutare cu 90° a unei grile pătrate
corespunde unei **rotații rigide a tablei**, iar `calibrateCamera` are câte
un `rvec`/`tvec` per imagine, deci o absoarbe. Intrinsecii nu simt nimic.

**Unde chiar contează:** oriunde tabla definește un cadru de referință —
estimare de poză, extrinseci, hand-eye. Acolo poza sare cu 90° între cadre.

Rămânem pe 9×6 implicit: nu costă nimic și elimină o clasă de erori la
utilizări viitoare. Dar nu invocăm „precizia calibrării" ca motiv, pentru că
măsurătoarea nu o susține. **ChArUco elimină ambiguitatea complet** —
fiecare colț are ID din markerii vecini, verificat identic la toate patru
rotațiile.

### 5.18 Zona liniștită contează doar pentru detectorul clasic

„`findChessboardCorners` are nevoie de margine albă" e adevărat, dar numai
pentru varianta clasică, și numai pe fundal închis. Măsurat, 5 poze cu
înclinări 0–40°:

| detector | fundal | cu margine | fără margine |
|---|---|---|---|
| `findChessboardCorners` | gri | 5/5 | 5/5 |
| `findChessboardCorners` | **negru** | **5/5** | **0/5** |
| `findChessboardCornersSB` | gri | 5/5 | 5/5 |
| `findChessboardCornersSB` | **negru** | 5/5 | **5/5** |

Fără margine, pătratele negre de pe marginea tablei fuzionează cu fundalul
și conturul nu mai poate fi delimitat. Varianta sector-based nu are această
sensibilitate.

Păstrăm marginea de o lățime de pătrat oricum: `calibrate_camera.py` cade pe
detectorul clasic dacă SB lipsește, alte unelte îl pot folosi direct, iar
marginea nu costă nimic pe hârtie.

### 5.19 `cv2.aruco.calibrateCameraCharuco` nu mai există în OpenCV 5

Nici `interpolateCornersCharuco`. Tutorialele și exemplele care le folosesc
sunt pentru OpenCV 4.x. Calea actuală, și cea din `tools/calibrate_camera.py`:

```python
detector = cv2.aruco.CharucoDetector(board)          # optional: params
corners, ids, m_corners, m_ids = detector.detectBoard(gray)
objp, imgp = board.matchImagePoints(corners, ids)    # per imagine
rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_list, img_list, size,
                                                  None, None)
```

`matchImagePoints` întoarce **doar colțurile văzute în acea imagine**, deci
fiecare poză are propriul set de puncte-obiect. Asta a cerut generalizarea
lui `calibrate()`: forma veche presupunea același tipar în toate pozele.
Funcția clasică rămâne disponibilă ca `calibrate(corner_sets, size, pattern,
square_mm)` pentru tabla de șah.

**Avantajul măsurat al ChArUco** — vederi parțiale, cu ținta depășind cadrul
(12 poze sintetice de aproape, scale 0.42):

| | poze utile | observație |
|---|---|---|
| ChArUco | **12/12** | 10 parțiale, minim 21 din 40 de colțuri |
| tablă de șah | 4/12 | pierde poza întreagă dacă un colț iese din cadru |

La 102° vrem ținta mare în cadru; ChArUco e singurul care permite asta.

### 5.20 Randarea sintetică fără antialiasing măsoară randorul, nu unealta

A treia oară când aceeași capcană apare sub altă formă (vezi §5.11).

O pagină A3 la 300 DPI are 4961 px lățime. Văzută de la 700 mm cu f = 933 px,
tabla de 333 mm ocupă ~444 px în cadru — **minificare de 8.9×**. `cv2.remap`
eșantionează punctual, fără prefiltrare, deci produce aliasing masiv.

Efectul nu e cosmetic. Reproiecția unei ținte ChArUco, aceeași poză:

| sursă | reproiecție |
|---|---|
| brută | 0.498 px |
| prefiltrată, σ = minificare/3 | 0.137 px |
| prefiltrată, σ = minificare/2 | **0.070 px** |

Pe 25 de vederi, RMS-ul calibrării a scăzut de la 0.341 px la **0.105 px**,
iar eroarea pe k1 de la +4.1% la +0.7%.

Aproape am tras concluzia greșită că „ChArUco e intrinsec mai zgomotos decât
detectorul de tablă" — detectorul de tablă sector-based e robust la aliasing,
decodarea markerilor nu. `synthetic.render_planar_target` prefiltrează acum
sursa cu un Gaussian potrivit minificării mediane, calculată analitic din
`px_per_mm · adâncime / focală`.

Limitarea rămasă: un singur Gaussian pe toată sursa. Pentru înclinări sub
~40° variația de scară în cadru e mică; la înclinări extreme ar trebui blur
variabil.

### 5.21 Un marker ArUco oglindit nu se detectează — și tace

Randorul sintetic mapează un punct `(x_mm, y_mm)` al țintei la pixelul
`origin + (x, y)·px_per_mm`, deci **axa y a cadrului țintei e în jos**, ca în
imagine. Dacă definești colțurile obiectului cu y în **sus** (convenția
uzuală în 3D) și le randezi prin acea mapare, iese o imagine **oglindită**.

Un marker ArUco oglindit nu produce o detecție greșită. Nu produce nimic:
`detectMarkers` întoarce pur și simplu zero markeri, la orice distanță și
orice rezoluție a sursei. Am pierdut o oră căutând în sampling și în
antialiasing înainte să mă uit la orientare.

Semnul care distinge cauzele: dacă e o problemă de rezoluție sau de blur,
detecția merge la unele distanțe și cade la altele. Dacă tace la **toate**
distanțele, inclusiv la 448 px unde markerul umple un sfert din cadru, e o
problemă de orientare sau de dicționar, nu de calitate a imaginii.

### 5.22 RMS-ul nu e un criteriu de valabilitate a calibrării

Măsurat: **24 de poze identice** dau `RMS = 0.061 px` — mai bun decât un set
bun (0.105 px) — cu **`fx` greșit cu +754%** și `k1` cu +429%.

Motivul e că RMS-ul măsoară cât de bine se potrivește modelul cu punctele
date, nu dacă punctele spun ceva despre cameră. Un set de poze care nu
constrânge geometria are o infinitate de soluții echivalente, iar
`calibrateCamera` alege una — care reproiectează perfect și nu descrie nimic.

Ăsta e un mod de eșec periculos: o calibrare care *arată* excelentă și e
inutilizabilă. `tools/calibrate_camera.py` are acum două gărzi, verificate
**după** pragul de RMS tocmai pentru că un set degenerat îl trece:

| gardă | prag | ce prinde |
|---|---|---|
| focala față de cea geometrică | `FOCAL_SANITY_REL` 30% | cazul de mai sus (+754%) |
| acoperirea cadrului | `MIN_COVERAGE_CELLS` 5 din 9 | poze îngrămădite într-o zonă |

Aceeași logică se aplică pe teren: dacă cineva ține tabla nemișcată și apasă
de 25 de ori, RMS-ul va fi superb.

### 5.23 Acuratețea distanței scade cu `marker_px`, nu cu distanța

Măsurat pe 15 poze sintetice, marker de 480 mm, colțuri fără zgomot adăugat:

| distanță | `marker_px` | eroare pe distanță | eroare pe colțuri |
|---|---|---|---|
| 2.45 m | 202 | +0.12% | 0.70 px |
| 6.00 m | 74 | +0.22% | 0.28 px |
| 9.80 m | 49 | +0.38% | 0.51 px |
| 12.7 m | 36 | +0.72% | 0.35 px |
| 15.0 m | **29** | **+1.99%** | 0.68 px |

Relația e aproximativ `eroare_relativă ≈ eroare_colțuri / marker_px`, ceea ce
se verifică: 0.68/29 = 2.3% față de 1.99% măsurat.

Consecințe practice:

- La 5 m (90 px) așteptăm ~0.5% eroare de range — criteriul E2 de sub 5% e
  confortabil.
- La 12 m (37 px) așteptăm 1–2% **din zgomotul de colțuri singur**, înainte
  de orice efect de lumină sau blur.
- Explică observația din §4: zgomotul la altitudine nu contează pentru
  aterizare, pentru că precizia se decide în ultimii 50 cm, unde `marker_px`
  e ~900 și aceeași eroare de colțuri înseamnă 0.08%.

Cifrele sunt sintetice, fără blur de mișcare și fără hârtie reală. E2 le
poate doar înrăutăți.

### 5.24 Două stive Python: ce e pe vehicul, și de ce

**Stiva reală, măsurată pe vehicul** (nu presupusă — vezi nota de la final):

| | desktop | Raspberry Pi |
|---|---|---|
| Distribuție | Ubuntu 22.04 | **Raspberry Pi OS Trixie** |
| Python | 3.10 | **3.13.5** |
| numpy | 2.2.6 (pip) | **2.2.4 (apt)** |
| OpenCV | 5.0.0 (pip) | **4.10.0 (apt, `python3-opencv`)** |

Regula pe Pi, indiferent de distribuție: **numpy și picamera2 vin din apt și
nimic nu se instalează peste ele.** picamera2 și `simplejpeg` sunt compilate
împotriva numpy-ului de sistem; o copie pip în venv le umbrește și le rupe,
iar eroarea (`_ARRAY_API not found`) apare la `import picamera2`, departe de
cauză. De aceea venv-ul se creează cu `--system-site-packages`.

**De unde vine OpenCV depinde de distribuție, și criteriul e versiunea din
apt:**

| distribuție | `python3-opencv` din apt | ce facem |
|---|---|---|
| Trixie | **4.10.0** | apt. Construit împotriva numpy 2.2.4 de sistem. |
| Bookworm | 4.6.0 | **pip**, fixat la 4.10.0.84 |
| Bullseye | 4.5.x | nesuportat |

Pragul e **4.7.0**, unde a apărut `cv2.aruco.ArucoDetector`. Codul îl
folosește peste tot, deci pe Bookworm apt-ul nu e o opțiune. Iar acolo
versiunea pip nu poate fi 5.x: `opencv-contrib-python 5.0.0.93` cere
`numpy>=2`, iar Bookworm are 1.24.2 din apt → pip ar instala numpy 2 în venv
→ picamera2 se rupe.

> Pe **Trixie** constrângerea aceea nu mai există: apt dă numpy 2.2.4, care
> satisface `numpy>=2`, deci OpenCV 5 prin pip ar merge. Rămânem totuși pe
> apt — o copie pip peste build-ul de sistem înseamnă două OpenCV-uri în
> același proces, iar cel din apt e cel împotriva căruia e construit restul
> distribuției. Deci pe Trixie diferența de versiune față de desktop e o
> **alegere**, nu o imposibilitate.

**Ce costă diferența de versiune, măsurat.** Un venv de paritate
(numpy 2.2.4 + OpenCV 4.10.0.84) față de desktop (numpy 2.2.6 + OpenCV 5.0):

| distanță | colțuri | distanță | unghiuri |
|---|---|---|---|
| 1.0 m | 0.023 px | 0.0002% | 0.024° |
| 5.0 m | 0.005 px | 0.0005% | 0.000° |
| 12.0 m | 0.004 px | 0.0059% | 0.000° |

Praguri E1: 1 px, 2%, 0.3°. Cifrele validate pe desktop se transferă, iar
suita trece integral pe ambele.

**Rezultatele sunt identice bit cu bit cu cele măsurate pe stiva Bookworm**
(numpy 1.24.2 + același OpenCV 4.10): versiunea de numpy nu schimbă nimic în
ieșirea detectorului, ceea ce e de așteptat — numpy e doar containerul de
array-uri. Ce contează e versiunea de **OpenCV**.

**Unde versiunea de OpenCV chiar contează.** Pe ținta de calibrare ChArUco,
detectorul `DICT_4X4_50` produce detecții false pe 4.10 și niciuna pe 5.0:

| | OpenCV 5.0 | OpenCV 4.10 (ambele stive Pi) |
|---|---|---|
| detecții false în `DICT_4X4_50` | 0 / 36 | **5 / 36** (ID 48) |
| ID 26 (markerul de misiune) | 0 | 0 |

Runda 4 raportase „0 detecții în `DICT_4X4_50`" ca dovadă a separării de
dicționare. Adevărat doar pe versiunea de pe desktop — exact cea pe care
vehiculul nu o rulează. Ce ne protejează pe toate versiunile e **filtrul de
ID** din `ArucoMarkerDetector`.

**Regula generală, plătită de două ori.** Prima oară testul măsura corect pe
platforma greșită. A doua oară am scris o rundă întreagă de unelte pentru
Bookworm, cu o gardă de platformă care **respingea** distribuția de pe
vehicul. Codul era corect; presupunerea despre mediu nu.

Deci: înainte de a scrie „verificat" despre ceva ce depinde de mediu,
**citește mediul de pe vehicul**, nu specificația din care crezi că vine.
`tools/preflight_check.py` raportează acum, la fiecare rulare, distribuția,
Python, numpy, OpenCV și **calea** fiecăruia — calea e cea care arată dacă
pip a pus o copie peste sistem.

Venv de paritate pentru teste, două minute:

```bash
python3 -m venv /tmp/trixie-sim
/tmp/trixie-sim/bin/pip install numpy==2.2.4 \
    opencv-contrib-python==4.10.0.84 pymavlink==2.4.49 PyYAML
for t in tools/test_*.py; do /tmp/trixie-sim/bin/python "$t"; done
```

Nu are picamera2 și rulează pe Python 3.10, nu 3.13, deci nu acoperă chiar
tot — dar acoperă versiunea de OpenCV, care e ce contează pentru detecție.

### 5.25 O barieră de siguranță se scrie ca listă albă

`ReadOnlyVehicle` din `tools/nova_service.py` împiedică serviciul de bord să
comande vehiculul. Prima variantă enumera metodele de **comandă** și lăsa
restul să treacă. A ratat imediat `Vehicle.update_params()`, care retrimite
`PARAM_SET` pentru valorile neconfirmate: nu începe cu niciun prefix de
„comandă" și arată ca o metodă de întreținere.

Direcția implicită e tot ce contează:

| | metodă nouă în `Vehicle` |
|---|---|
| listă neagră | **permisă** până își amintește cineva să o interzică |
| listă albă | **interzisă** până decide cineva că e sigură |

Pentru o barieră de siguranță, doar a doua e acceptabilă. Același raționament
ca la `DETECTION_MONITORED_PHASES` din §8, unde lista e scrisă pozitiv ca o
fază nouă să nu fie supravegheată din greșeală.

Două detalii care fac diferența între barieră și decor:

- **`m` se blochează explicit.** E conexiunea mavutil brută; fără asta,
  `vehicle.m.mav.command_long_send(...)` ocolește tot.
- **Un test citește sursa** fiecărei metode permise și caută apeluri
  `self.m.mav.*_send(` care nu sunt cereri. Altfel lista albă se degradează
  tăcut la următoarea adăugire.

### 5.26 `systemd-analyze verify` prinde chei puse în secțiunea greșită

`StartLimitIntervalSec` și `StartLimitBurst` sunt chei de **`[Unit]`**, nu de
`[Service]`. Puse în `[Service]`, systemd le **ignoră tăcut**: serviciul
pornește, `systemctl status` arată verde, iar limita de reporniri pur și
simplu nu există.

Exact tiparul din §5.10, într-un alt sistem: acceptat fără eroare nu înseamnă
aplicat. Verificarea costă o comandă și nu are nevoie de `sudo`:

```bash
systemd-analyze verify systemd/nova-monitor.service
```

Ieșire goală = unitate validă. Raportează `Unknown key name ... ignoring`
pentru orice cheie pusă unde nu trebuie.

Corolar pentru teste: **nu determina secțiunea unui fișier .ini cu
`text.split('[Service]')`.** Un comentariu care conține literalul `[Service]`
mută granița, iar verificarea „cheia nu e în `[Service]`" trece din motivul
greșit. Testul din `test_pi_tooling.py` parsează pe linii ancorate.

### 5.27 Două procese, un singur `/dev/serial0`

`nova-monitor.service` pornește la boot și ține portul serial. `nova_pi.py`
pornit după el primește:

```
SerialException: could not open port /dev/serial0: [Errno 16] Device or
resource busy
```

Mesajul spune **ce**, nu spune **cine** și nici **ce să faci**. Pe teren,
între două curse, arată ca o problemă de cablu sau de permisiuni și trimite
căutarea în direcția greșită. Costul nu e tehnic — reparația e o comandă —
ci timpul până când cineva se prinde ce e.

`nova/serial_guard.py` întreabă **înainte** de a deschide portul:

| sursă | ce află |
|---|---|
| `systemctl is-active nova-monitor` | serviciul nostru; răspuns clar + comanda de reparare |
| `fuser` sau `/proc/*/fd` | orice alt proces: PID + linia de comandă |

```
[bord] EROARE: nova-monitor ocupa /dev/serial0.
        sudo systemctl stop nova-monitor
        (sau porneste cu --stop-service, care o face singur)
```

Trei decizii care nu sunt evidente:

- **Diagnosticul care nu se poate face nu blochează pornirea.** Fără
  systemd, sau fără drept de citire în `/proc`, funcțiile întorc „nu știu",
  nu o eroare. Un proces al altui utilizator nu se vede fără root, deci
  lista goală **nu** înseamnă „portul e liber" — blocarea o decide
  apelantul, și doar când știe sigur.
- **`--stop-service` cere confirmare.** Oprirea unui serviciu are efect în
  afara procesului nostru. `--yes` există pentru scripturi.
- **Cod de ieșire 3**, distinct de 2 (refuz de pornire: calibrare lipsă).
  Un script de teren poate deosebi „repar portul și reîncerc" de „nu pot
  zbura".

Pe `udpin:`/`tcp:` verificarea se sare: în SITL nu există port exclusiv.

### 5.28 Previzualizarea: `WINDOW_AUTOSIZE` ignoră tăcut fullscreen-ul

`setWindowProperty(WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN)` **nu face nimic**
pe o fereastră creată cu `WINDOW_AUTOSIZE` — și `AUTOSIZE` e ce primești dacă
apelezi `imshow` fără `namedWindow`. Nu apare nicio eroare; fereastra rămâne
mică și pare că fullscreen-ul nu merge pe Pi.

```python
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)          # obligatoriu
cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
```

**Ieșirea se leagă și pe Escape, nu doar pe `q`.** O fereastră fullscreen nu
are decorațiuni, deci nu există buton de închidere; dacă singura ieșire e `q`
și fereastra pierde focusul tastaturii, unealta se oprește doar din alt
terminal.

**Trei contexte, trei comportamente** (`nova/preview.py`):

| context | fereastră | de ce |
|---|---|---|
| unelte de banc (calibrare) | fullscreen, pornită | operatorul are nevoie de detaliu, ecranul Pi-ului e mic |
| prin VNC | `--preview-scale 0.5` | cadrul plin e lent pe VNC, iar sacadarea **arată** ca o detecție lentă și trimite căutarea aiurea |
| vehicul de concurs | **oprită implicit** | fără `vc4-kms-v3d` nu există accelerare grafică: fiecare `imshow` ia CPU din bugetul detecției |

Redimensionarea e **numai pentru afișare** — detecția rulează pe cadrul plin,
iar overlay-ul se desenează pe o copie. Un overlay desenat peste cadrul de
intrare ar fi exact genul de bug care nu se vede până la reproiecție.

Fără sesiune grafică (`DISPLAY`/`WAYLAND_DISPLAY` nesetate, adică SSH fără
`-X`), `imshow` aruncă `Can't initialize GUI backend`. `Preview` verifică
înainte, se stinge singură și spune de ce.

### 5.29 Un ecran de teren se proiectează invers față de un dashboard

Ecranul de concurs (`nova/race_screen.py`) se citește **de la un metru, în
soare, pe un ecran mic**, în secundele dintre curse. Fiecare decizie e contra
instinctului normal de interfață:

| instinct | ce trebuie de fapt |
|---|---|
| cât mai multă informație | **sub 10 linii**, fiecare cu un singur lucru |
| nuanțe de culoare pe stări | **trei culori**, binare: verde / galben / roșu |
| culoarea transmite starea | **cuvintele** o transmit: `NU ZBURA`, `GATA` |
| cifra e de ajuns | cifra **și unitatea**: `5.2 m`, nu `5.2` |
| valoarea lipsă = 0 | valoarea lipsă = `-`; `0 fps` și „nu știu" sunt diferite |

Verdictul raportează **cel mai grav** lucru, nu primul găsit: cine vede
`GATA` trebuie să poată să nu mai citească restul.

Două lucruri prinse abia când s-a desenat ecranul, nu la citirea codului:

- **Eticheta lipită de valoare.** Coloana avea 11 caractere, iar
  `LEGATURA FC` și `TEMPERATURA` au exact 11 → `LEGATURA FCOK  0.3 s`. La un
  metru nu se mai citește ca două lucruri. Coloana trebuie **mai lată** decât
  cea mai lungă etichetă, nu egală.
- **Unitatea dispărea odată cu valoarea.** Pe un rând cu două cifre
  (`29 fps   detecție 97%`), un `-` singur nu spune care dintre ele lipsește.
  Corect: `- fps`.

Terminal pur, nu OpenCV: merge prin SSH, nu cere sesiune grafică și nu ia CPU
din bugetul detecției (§5.28).

**Modul de concurs refuză să pornească dacă preflight-ul nu e integral verde**
— inclusiv când singura problemă e o verificare **sărită**. Cod de ieșire 4.
`tools/race_mode.py` e un lansator subțire peste `nova_pi.py --race`,
deliberat: al doilea punct de intrare care și-ar construi singur piesele ar
reintroduce exact clasa de bug din §5.14. Un test citește sursa lui
`race_mode.py` și pică dacă apare `SafetySupervisor(`, `HandoverGate(`,
`LandingStateMachine(` sau `run_loop(`.

### 5.30 Modularea autorității: ce se restaurează nu e ce ai trimis

`nova/authority.py` schimbă în zbor șase parametri pe praguri de altitudine
și îi pune la loc la handback sau abort. Feature-ul în sine e simplu; tot ce
l-a făcut greu e **garanția de restaurare**.

**Trei nume din cinci nu existau.** Cerute inițial ca `WPNAV_ACCEL`,
`PSC_POSXY_P`, `PSC_VELXY_D`. Reale pe 4.8:

| cerut | real | de ce |
|---|---|---|
| `WPNAV_ACCEL` | **`WP_ACC`** | prefixul e `WP_` (§5.4), numele scurt e `ACC` |
| `PSC_POSXY_P` | **`PSC_NE_POS_P`** | subgrupul s-a redenumit XY → NE |
| `PSC_VELXY_D` | **`PSC_NE_VEL_D`** | idem |

Verificate în sursă, nu din memorie:

```bash
grep -oP 'AP_GROUPINFO\("\K[A-Z_0-9]+' libraries/AC_WPNav/AC_WPNav.cpp
grep -n 'AP_SUBGROUPINFO' libraries/AC_AttitudeControl/AC_PosControl.cpp
```

Modulul își prinde singur greșeala asta: dacă FC-ul nu răspunde la citirea
inițială, parametrul intră în `unavailable` și **nu e scris niciodată**.

**Patru reguli, fiecare plătită de un test.**

1. **Nu modifica ce nu ai citit.** Fără originalul citit înapoi, modificarea
   nu se poate anula. Deci salvarea e completă înainte de prima scriere.
2. **Cache-ul nu e o citire.** Prima variantă lua originalele din
   `Vehicle.params`, care e un cache — putea conține o valoare de acum zece
   minute, schimbată între timp din GCS, iar restaurarea ar fi dus vehiculul
   la o valoare care nu a fost niciodată cea de la handover. `Vehicle` are
   acum `params_t` (când a fost văzut fiecare), iar salvarea acceptă doar
   valori sosite **după** armare.
3. **Restaurarea nu are voie să renunțe.** `Vehicle.set_param` abandonează
   după `PARAM_TRIES` și scrie un avertisment — potrivit pentru o comandă
   normală, nu pentru restaurarea autorității. Modulul compară valoarea
   citită înapoi cu ținta și reemite; cu legătura căzută nu consumă
   încercări (același tipar ca `safety._drive_mode`, H1).
4. **`restored` e o măsurătoare, nu o intenție.** Rămâne `False` până când
   fiecare parametru atins e confirmat înapoi. „Am trimis comenzile" nu e
   același lucru cu „vehiculul e la autoritate nominală".

**Se armează din fază, nu din tranziții** (§5.14). Orice ieșire din segment
— `HANDBACK`, `ABORT`, dezarmare, **o fază pe care nimeni nu a prevăzut-o** —
declanșează restaurarea. Un test folosește o fază inventată ca să verifice
tocmai asta.

**Histereză obligatorie pe praguri.** Fără ea, zgomotul de ±0.3 m în jurul
unui prag produce un `PARAM_SET` pe ciclu. Măsurat cu histereza de 0.5 m:
2 treceri și 0.08 scrieri/ciclu.

**Santinela „niciodată" nu e 0.0.** `_read_t = 0.0` cu o buclă care pornește
la `now = 0.0` blochează *prima* cerere prin propriul ei timeout — bug prins
de teste, mascat inițial de bug-ul de cache. `float('-inf')`.

**Ce nu atinge, deliberat.** `ANGLE_MAX` și `PSC_ANGLE_MAX` sunt în
`FORBIDDEN`, verificat pe tot ce s-a trimis, nu pe intenție. Unghiul maxim de
înclinare e plafonul de autoritate al vehiculului, nu reglaj fin — și
interacționează cu `MAX_TILT_DEG` din supervizor: l-am putea ridica până unde
supervizorul tratează înclinarea ca defecțiune.

**Vitezele de coborâre sunt plafonate la 0.5 m/s** (`DESCENT_VALIDATED_MS`)
până la măsurătoarea de distanță de frânare pe fiecare treaptă, cerută de
§6/15.2.9. `PROFIL_RAPID` există, dar cere `allow_fast_descent=True`.

### 5.31 Gazebo e ENU, noi suntem NED — și markerul are două laturi

Detectorul și mașina de stări lucrează în **NED** (`--north`, `--east`).
Gazebo lucrează în **ENU**. Maparea, o singură dată, aici:

```
pose_gazebo_x = east
pose_gazebo_y = north
pose_gazebo_z = sus
```

**De ce merită un paragraf.** Un marker plasat cu N și E inversate produce
exact simptomul unui bug de convenție în detector: vehiculul coboară *lângă*
marker, iar eroarea apare transpusă pe axe. Ore pierdute căutând în `solvePnP`
ceva ce e de fapt în fișierul de lume. `tools/make_marker_model.py` face
maparea într-un singur loc, iar testul o verifică pe fișierul **generat**, cu
valori asimetrice (`north=7, east=-3`) — cu `north == east`, o inversiune ar
trece testul.

**Markerul are două laturi și doar una intră în calcule.** Coala e 600 mm;
zona **codată** (inclusiv bordura neagră) e 480 mm, centrată, deci 80% din
latură și 60 mm de zonă liniștită de jur împrejur. `marker_size_m = 0.48` din
`config/nova.json` e latura **codată** — aceeași pe care o măsoară `solvePnP`.
Generat din greșeală la dimensiunea colii, markerul s-ar detecta la fel de
bine, iar toate distanțele ar ieși cu **25% eroare** — și ar arăta ca o
calibrare proastă. Testul măsoară latura detectată și cere 480 mm.

Rezoluția nu e rotunjită: 2400 px pe 480 mm = **5 px/mm** exact, coala iese
3000 px, iar `DICT_4X4_50` are 6 module → 400 px pe modul, fără rest. O
rezoluție care nu se împarte exact e **refuzată**, nu rotunjită.

**`<include><uri>` acceptă `model://` sau o cale ABSOLUTĂ — nu una
relativă.** Măsurat, cu `GZ_SIM_RESOURCE_PATH` conținând doar modelele
ardupilot_gazebo:

| uri | rezultat |
|---|---|
| `model://aruco_26` | **eșec** — cere calea în `GZ_SIM_RESOURCE_PATH` |
| `../models/aruco_26` | **eșec** — relativul la fișierul lumii nu se rezolvă |
| `/abs/path/sim/models/aruco_26` | **merge**, fără nicio variabilă de mediu |

De aceea generatorul scrie implicit calea absolută: `gz sim <lume>` tastat
direct funcționează, iar eroarea `Unable to find uri` — care apare în mijlocul
unei sesiuni și nu spune ce variabilă lipsește — dispare. Fișierul e generat
oricum; pe altă mașină se **regenerează**, iar un test verifică pe lumea din
repo că URI-ul chiar duce la un director cu `model.config`.

**`--` e ilegal într-un comentariu XML, și libsdformat îl acceptă tăcut.**
Un comentariu care conținea `--uri-mode` făcea fișierul XML invalid;
`gz sim` îl încărca fără o vorbă, iar `ElementTree` refuza să-l parseze.
Deci „Gazebo îl încarcă" nu înseamnă „e XML valid" — încă o variantă de
§5.10.

**Un test care eșuează din alt motiv decât cel testat** (§5.11, a patra
oară): prima verificare a URI-urilor rula cu `env -u GZ_SIM_RESOURCE_PATH
bash -c 'source ~/.bashrc; ...'`, dar `.bashrc` iese devreme într-un shell
neinteractiv. Deci eșuau modelele **stock**, nu al nostru, iar toate cele
trei variante păreau la fel de rele. Mediul unui test se fixează explicit,
nu se moștenește.

**`gz sdf --check` nu rezolvă `model://`.** Raportează `Unable to find uri`
pentru *toate* includerile, inclusiv cele stock din lumea de bază — deci nu e
un semn că lumea ta e greșită. Verificarea care chiar contează e serverul
headless, care rezolvă URI-urile și spawnează entitățile:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/sim/models:$GZ_SIM_RESOURCE_PATH"
gz sim -s -r sim/worlds/nova_marker.sdf &
gz model --list                      # aruco_26 trebuie să apară
gz model -m aruco_26 --pose          # și la poziția corectă
```

Măsurat: `[1.500000 2.000000 0.010000]` pentru `N=2.0 E=1.5`. `z = 0.01`
evită z-fighting cu solul.

**Lumea e derivată, nu rescrisă.** `nova_marker.sdf` pornește din
`iris_runway.sdf` al lui ardupilot_gazebo și înlocuiește doar lumina și
adaugă markerul. Plugin-urile și coordonatele sferice trebuie să rămână exact
ce folosește ardupilot_gazebo; o lume scrisă de la zero ar diverge tăcut la
prima lor actualizare. Un test compară lista de plugin-uri și coordonatele cu
originalul.

**Randarea: confirmată vizual.** `<plane>` cu `albedo_map` produce textura
**pătrată, nerepetată, plană pe sol** — deci UV-urile generate de ogre2 sunt
corecte și nu e nevoie de un `<box>` subțire ca alternativă. Serverul headless
nu randează (`libEGL: failed to create dri2 screen`), deci asta se putea
verifica doar cu GUI.

**Consecință de care depinde I4: planul e la `z = 0.01`, nu la 0.** Adevărul
pentru eroarea de range e `altitudine_vehicul − 0.01`, nu altitudinea brută.
Ignorat, offsetul de 1 cm apare ca **bias sistematic**, cu atât mai mare cu
cât vehiculul e mai jos:

| altitudine | eroare dacă se ignoră |
|---|---|
| 12 m | 0.08% |
| 5 m | 0.20% |
| 1 m | 1.00% |
| 0.5 m | **2.00%** |
| 0.38 m (limita de detecție, §5.2) | **2.63%** |

Pragul I4 pentru eroarea de range e 3%, deci la capătul de jos offsetul
singur ar consuma aproape tot bugetul — și ar arăta ca o eroare de calibrare
sau de `solvePnP`, nu ca o constantă din fișierul de lume. Cei 1 cm nu se
elimină (fără ei apare z-fighting cu solul); se **scad din adevăr**.

### 5.32 `RC_CHANNELS_OVERRIDE` are două capcane tăcute

Din runda 3, poarta de handover e singura cale către segmentul autonom, deci
orice test automat trebuie să comute AUX 7. `tools/sim_handover.py` o face,
și două lucruri l-ar fi făcut să pară că merge fără să meargă:

**1. `MAV_GCS_SYSID`.** `GCS_MAVLINK::handle_rc_channels_override` respinge
**tăcut** orice `RC_CHANNELS_OVERRIDE` venit de la alt sysid decât cel din
`MAV_GCS_SYSID`. Fără potrivire: scriptul trimite fără eroare, poarta nu vede
niciodată AUX-ul, iar testul eșuează fără niciun indiciu. Se verifică la
pornire și se refuză cu comanda de reparare — același tipar ca §5.10.

**2. Throttle-ul nu se trimite la `RC3_TRIM`.** Roll, pitch și yaw se
auto-centrează, deci „liber" înseamnă „la trim". Throttle-ul nu: pe un
emițător real e la **mijlocul cursei** pentru hover, iar `RC3_TRIM` e la
capătul de jos (1100 la noi). Poarta ar accepta oricum — ea măsoară
*amplitudinea* throttle-ului, nu distanța față de trim (§8) — dar injectat
într-un vehicul care planează în LOITER, 1100 comandă **coborâre rapidă**.
Unealta de test ar face vehiculul să cadă. Se trimite mijlocul lui
`RC3_MIN..RC3_MAX`.

Trei detalii mai mici, fiecare cu test:

- **`UINT16_MAX` ≠ `0`.** 65535 pe un canal înseamnă „nu schimba starea de
  override"; 0 înseamnă „**eliberează**". Pe canalul de mod vrem primul,
  altfel `mode guided` / `takeoff` din MAVProxy devin inutilizabile.
- **Frontul, nu starea.** Poarta cere frontul crescător, deci AUX trebuie să
  fi fost văzut jos înainte de ridicare. Scriptul ține ~1 s jos întâi.
- **Manșele stau nemișcate, amplitudine zero.** Orice tremur ar fi fie un
  refuz, fie un override fals imediat după ACCEPT — încercarea anulată dintr-un
  artefact al uneltei de test.

Testul care contează trece canalele injectate prin **poarta reală** și cere
ACCEPT după fereastra de așezare, apoi verifică că 5 s de semnal identic nu
declanșează override. Un script care „pare că trimite ce trebuie" dar pe care
poarta îl refuză nu ajută la nimic.

### 5.33 Costul randării, măsurat — și de ce `--scale` nu e pârghia

**Intrinsecii ajung intacți în Gazebo.** `/down_cam/camera_info` raportează
înapoi exact ce scrie generatorul: `fx=937, fy=933.7, cx=1139.4, cy=649`, iar
distorsiunea în **ordinea OpenCV** (`k1 k2 p1 p2 k3`), adică exact cum o
consumă `nova/detector_pi.py`. Lanțul `camera_pi.yaml` → SDF → Gazebo →
`camera_info` e verificat dus-întors, nu presupus.

**Un senzor de cameră nu randează fără abonat** — nici cu `<always_on>1</always_on>`.

Testul care o dovedește, fără să depindă de costul transportului: aceeași
lume, singura diferență fiind `update_rate`.

| `update_rate` | RTF |
|---|---|
| 30 Hz | 0.57 |
| **240 Hz** | **0.57** |

De opt ori mai multe cadre, exact același cost. Într-un `gz sim -s` fără
abonat, randarea pur și simplu nu se întâmplă.

**Consecința asupra metodei.** Cronometrarea pe `--iterations` nu poate ține
un abonat pe toată fereastra — procesul rulează până se termină pașii. Deci
acea metodă **nu poate măsura costul camerei**, oricât de curat ai
cronometra-o. `tools/measure_rtf.py --subscribe /down_cam/image` comută pe o
măsurătoare „vie": server continuu, abonați atașați tot timpul, iar RTF-ul se
citește din `/world/<nume>/stats`, adică din ce raportează Gazebo — deci
pornirea procesului nici nu mai intră în socoteală.

**Ce știm măsurat curat, și ce nu.**

| | RTF |
|---|---|
| lume fără cameră | 0.56 |
| lume cu cameră 30 Hz, **fără abonat** | 0.56 |
| lume cu cameră 240 Hz, **fără abonat** | 0.57 |
| lume cu cameră **și abonat** | **nemăsurat** |

Deci: **fizica e costul dominant** — 0.56 fără nicio cameră, la pas de 1 ms
cu lift-drag pe patru rotoare. Asta e ferm, și e suficient ca să închidem
`--scale` ca pârghie: chiar dacă randarea ar fi gratis, tot rămâi la 0.56.
Cât costă camera în bucla reală rămâne deschis până la măsurătoarea cu abonat.

**Contaminarea, de două ori.** Primele cifre (0.50 și 0.96 pe aceeași lume,
factor de doi) au fost luate peste un `gz sim` cu GUI care randa în paralel —
iar unul dintre serverele concurente era pornit de **propria unealtă**:
`--check-topic` lansa un server, iar `terminate()` pe lansator lăsa în viață
procesele `gz sim server` și `gz sim gui` pe care acesta le forkează. Se
omoară **grupul**. Unealta refuză acum să măsoare dacă rulează alt `gz sim`.

**Regula generală:** o măsurătoare de performanță făcută pe o mașină pe care
rulează și altceva nu e o măsurătoare, iar harness-ul trebuie să verifice
asta — nu operatorul să își amintească. Aceeași lecție ca §5.11, mutată din
teste în cronometrare.

**Și verifică întâi că randarea chiar se întâmplă.** Headless, fără EGL
funcțional, topicurile `/down_cam/image` și `/down_cam/camera_info` sunt
**anunțate** dar nu sosește niciun mesaj — iar timpul cu și fără cameră iese
identic (9.067 vs 9.069 s), ceea ce citit greșit înseamnă „randarea e
gratis". Un topic anunțat nu înseamnă date:

```bash
gz topic -l | grep down_cam                   # apare si cand nu randeaza nimic
gz topic -e -t /down_cam/camera_info -n 1     # asta chiar masoara ceva
```

`camera_info` e mesajul de verificat, nu `image`: aceeași rată, câțiva
octeți, deci elimină transportul a 3 MB ca explicație alternativă.
`measure_rtf.py --check-topic` o face singur.

### 5.34 Calibrarea de simulare e un fișier separat, nu un prag ridicat

Prima calibrare reală a camerei a dat **RMS 0.85 px**, peste
`MAX_REPROJ_ERR_PX = 0.5`. Reacția evidentă — ridică pragul — ar fi greșită:
constanta e citită de `CameraCalibration.load(require_real=True)`, adică de
**detectorul de bord**. Ridicată global, ar slăbi tăcut exact garda care
decide dacă se zboară.

Separarea e aceeași ca la E0 în runda 3, și din același motiv: garda există ca
să protejeze un vehicul real; în simulare vehiculul e Gazebo.

| | zbor | simulare |
|---|---|---|
| fișier | `config/camera_pi.yaml` | `config/camera_sim.yaml` |
| citit de | `nova/config.py` → detector | doar `--calib` explicit |
| prag RMS | 0.5, neatins | ocolit cu `--provisional` |
| `is_real()` | trebuie `True` | `False`, deliberat |

Calibrarea provizorie are `n_images = 0` **intenționat**: așa `is_real()` e
fals și orice unealtă o refuză până cineva cere ocolirea din linia de
comandă, unde se vede. Pusă în `config/camera_pi.yaml` cu un `n_images`
inventat, ar trece toate gărzile și ar fi acceptată ca reală pentru zbor.

Când chiar trebuie un prag mai permisiv pentru o rulare anume, există
`--max-rms` — ridică pragul pentru *acea* rulare, nu pentru tot codul.

**Despre 0.85 px ca atare.** §5.22 spune că RMS-ul nu e criteriu de
valabilitate, dar rămâne indicator de calitate: sintetic, ChArUco dă 0.105 px,
iar o cameră reală bine calibrată stă tipic la 0.2–0.5. 0.85 sugerează ținta
neplană, poze mișcate, sau colțuri neacoperite — nu un obiectiv prost. De
refăcut înainte de zbor; pentru simulare, intrinsecii sunt destul de buni.

### 5.35 `open(f,'w').write(open(f).read())` golește fișierul

Python evaluează întâi obiectul pe care se apelează metoda, deci `open(f,'w')`
**trunchiază fișierul** înainte ca argumentul `open(f).read()` să fie
evaluat. Se citește un fișier deja gol și se scrie nimic.

Modelul de vehicul ieșea de **zero octeți**, iar simptomul apărea abia la
încărcare în Gazebo, ca o eroare de SDF. Corect e să citești în altă
instrucțiune, înainte de a deschide pentru scriere — sau, mai bine, să faci
substituția pe text înainte de a scrie vreodată.

Un test verifică acum că modelul generat are peste 5000 de octeți și că
niciun șablon `{...}` nu a rămas neînlocuit.

### 5.36 Trei medii Python, și de ce simularea îl rulează pe cel de pe vehicul

Sursa de cadre din Gazebo are nevoie de `gz.transport13` + `gz.msgs10`, care
vin din **apt** și există doar în `python3` de sistem. Detectorul are nevoie
de OpenCV ≥ 4.7, pentru `cv2.aruco.ArucoDetector`. Pe Ubuntu 22.04 cele două
nu se întâlnesc nicăieri:

| | `gz.transport` | OpenCV |
|---|---|---|
| `python3` de sistem | ✓ | **4.5.4** — fără `ArucoDetector` |
| `~/nova-venv` | ✗ | 5.0.0 |

**`PYTHONPATH=/usr/lib/python3/dist-packages` NU rezolvă.** Măsurat: acea
cale ajunge **înaintea** lui `site-packages` din venv, deci `cv2` de sistem
(4.5.4) îl umbrește pe cel bun. Soluția aparent evidentă face exact opusul a
ce vrei.

Soluția e un al treilea venv, cu `--system-site-packages` și OpenCV **4.10**
(`tools/setup_sim_venv.sh`):

| componentă | versiune | de unde |
|---|---|---|
| `cv2` | 4.10.0 | venv (pip) |
| `numpy` | 1.21.5 | **sistem (apt)** |
| `gz.transport13` + `gz.msgs10` | — | **sistem (apt)** |

De ce 4.10 și nu 5.x: 4.10 cere `numpy>=1.21`, deci se mulțumește cu cel din
apt și nu instalează unul propriu peste el. 5.x ar cere `numpy>=2`, l-ar pune
în venv peste cel de sistem, iar legăturile gz — compilate împotriva celui de
sistem — s-ar rupe. Aceeași regulă ca §5.24 pe Pi, aplicată pe desktop.

**Efectul secundar e un câștig, nu un compromis:** 4.10 e **exact versiunea
de pe vehicul**. Simularea rulează același detector ca zborul, iar
diferențele dintre 4.10 și 5.0 sunt reale (§5.24). `~/nova-venv` rămâne
neatins, pentru restul uneltelor de desktop.

**Calea 1 din două, și a mers.** Alternativa era `GstCameraPlugin` cu flux
UDP citit prin `cv2.VideoCapture`; H.264 ar fi introdus artefacte de
compresie care degradează exact localizarea colțurilor pe care vrem să o
măsurăm. Prin `gz-transport` cadrele vin **necomprimate**, iar timestamp-ul e
cel din simulare.

**Ceasul e cel de simulare, și asta obligă bucla.** `GazeboFrameSource`
întoarce implicit timpul din antetul mesajului. Consecința pentru I4: bucla
care consumă sursa trebuie să folosească **același** ceas, altfel
`now - det.t` compară două lumi. `nova/state_machine.run_loop` folosește
`time.monotonic()`, deci `nova_sim.py` are nevoie de propria buclă pe timp de
simulare — nu de o modificare în `run_loop`, care e validat.

**Sursa e abonatul.** Un senzor Gazebo nu randează fără abonat (§5.33), deci
randarea pornește când instanțiezi `GazeboFrameSource` și se oprește când o
închizi. Asta face și măsurătoarea de RTF cu abonat posibilă.

**Coada e de un cadru, deliberat.** Când detecția nu ține pasul, se pierde
cadrul **vechi**, nu cel nou: pe un vehicul care coboară, un cadru vechi e mai
rău decât niciunul.


### 5.37 Raza de 6.5 m a porții nu e utilizabilă la 5 m — limita e camera

Poarta acceptă handover până la **6.5 m** lateral față de marker, la orice
altitudine din fereastra 5–12 m (§8). Geometric, cele două limite nu sunt
compatibile la capătul de jos.

La 5 m altitudine și 6.5 m lateral, markerul e la `atan(6.5/5) = 52°` de
nadir — peste jumătatea de VFOV (33.5°). **Markerul nu e în cadru**, deci
poarta refuză pe „marker nedetectat" dintr-o cauză pur geometrică, nu
dintr-o problemă de detecție.

Limita efectivă e proporțională cu altitudinea:

```
d_max = 0.9 · h · tan(33.5°) − 0.24        # 0.9: §5.2; 0.24: jumătate de marker
```

| altitudine | d_max efectiv | ce spune poarta |
|---|---|---|
| 5 m | **2.7 m** | 6.5 m |
| 8 m | 4.5 m | 6.5 m |
| 12 m | 6.5 m | 6.5 m |

**Ce am schimbat și ce nu.** `tools/batch_sim.py` nu planifică rulări în
afara conului camerei — altfel o campanie ar raporta refuzuri care nu spun
nimic despre software. Poarta **nu** a fost modificată: regula rundei
interzice atingerea lui `nova/handover.py`, iar refuzul ei e corect oricum,
doar motivul raportat e mai puțin util decât ar putea fi.

De discutat la runda următoare: un refuz care spune „prea departe pentru
altitudinea asta" ar trimite pilotul să urce, nu să caute markerul. Pe teren
diferența e între două secunde și o încercare pierdută.

### 5.38 O campanie randomizată se poate înșela singură în trei feluri

Toate trei prinse scriind `tools/batch_sim.py`, toate trei cu test.

**1. `r = R·U` aglomerează punctele în centru.** Tras naiv, raza uniformă
înseamnă densitate *neuniformă* pe disc: jumătate din puncte cad în sfertul
interior de arie. Campania ar testa mai ales cazul ușor, cu markerul aproape
sub vehicul, și ar raporta o rată de succes optimistă. Corect: `r = R·√U`.
Testul cere ca ~50% din puncte să fie în jumătatea exterioară de arie.

**2. O condiție cu două valori nu se trage independent.** `roughness` are
exact două valori (0.9 mată, 0.3 lucioasă). `random.choice` pe o campanie de
4 rulări poate da de patru ori aceeași valoare — iar reflexia, care e chiar
riscul semnalat de 15.4.7, rămâne netestată fără ca nimic să spună asta.
Lista se construiește echilibrată și apoi se amestecă.

**3. Condiția scrisă în CSV trebuie să fie consistentă cu ea însăși.**
Raza era trasă din altitudinea neroturnjită și raportată lângă altitudinea
rotunjită la 2 zecimale. Diferența e sub un centimetru, dar înseamnă că o
linie din CSV putea arăta o rază peste limita altitudinii scrise pe aceeași
linie — adică evidența se contrazice singură. Rotunjirea se face **înainte**
de a calcula orice depinde de valoare.

Prins de testul care verifică `raza ≤ raza_max(alt)` pe 60 de rulări. Fără
el, cineva care ar fi verificat CSV-ul la scrutineering ar fi găsit
inconsistența înaintea noastră.

### 5.39 Un al doilea ceas face monitoarele inerte, fără să dea nicio eroare

`nova_sim.py` rulează pe timp de simulare (§5.36). `Vehicle.pump()` notează
ora heartbeat-ului cu `time.monotonic()`. Cele două diferă cu ordine de
mărime — `time.monotonic()` e de ordinul zecilor de mii de secunde, timpul
de simulare pornește de la zero.

Deci `vehicle.time_since_heartbeat(now_sim)` iese **negativă**, de exemplu
−98 752 s. Niciun prag pozitiv nu se atinge vreodată:

```
LINK_MAX_AGE_S = 1.0        ->  -98752 < 1.0  ->  "legatura e proaspata"
```

**`_mon_link` din supervizor (H1) nu s-ar fi declanșat niciodată în
simulare.** Fără eroare, fără avertisment, fără nimic de văzut în log —
doar un monitor care raportează sănătate la infinit. A treia oară aceeași
formă ca §5.14: piesele merg, cablajul nu.

Reparat aditiv, în `tools/nova_sim.py`, cu un `SimVehicle` care schimbă doar
**ceasul implicit** al metodelor care își notează singure ora
(`_note_heartbeat`, `set_param`). `nova/vehicle.py` rămâne neatins.

Două lucruri de reținut dincolo de bug:

- **Granița dintre ceasuri trebuie să fie o decizie, nu un accident.**
  Vârsta detecției și temporizările mașinii de stări aparțin lumii simulate;
  timpii de rețea ar aparține lumii reale. Alegerea aici e ca **totul** să
  fie pe ceasul buclei, ca să nu existe două unități în aceeași comparație.
  Consecința asumată: backoff-ul de reconectare se numără în secunde de
  simulare, adică ~1.8× mai mult timp de perete la RTF 0.56.
- **Un test pozitiv singur nu ar fi dovedit nimic.** Perechea lui — un
  `Vehicle` obișnuit întrebat cu timp de simulare, care dă −98 752 s — e cea
  care arată că testul măsoară ceva (§5.11).

Aceeași verificare a scos la iveală și o metrică moartă: `note_miss()` din
prima variantă a lui `nova_sim.py` nu era apelată de nimeni, deci rata de
detecție ar fi raportat **100% în orice condiții**, inclusiv cu detectorul
oprit. Cadrele se numără acum din contorul detectorului, iar un test cere ca
rata să poată scădea sub 100%.


### 5.40 Două profiluri care se compară trebuie să difere într-o singură coloană

`PROFIL_RAPID` (coborâre agresivă, I5) exista ca variantă a lui
`PROFIL_IMPLICIT` cu viteze mai mari — dar avea și `WP_ACC` 1.20 în loc de
1.00 pe banda de sus. Pare inofensiv: mai multă viteză, mai multă
accelerație laterală.

Consecința e că o oscilație apărută la 1.5 m/s **nu s-ar mai putea atribui
vitezei**. Iar exact atribuirea e ce trebuie să intre în Safety Case: §6
cere distanța de frânare *la fiecare treaptă de viteză*, nu o cifră globală.

Regula, scrisă ca test: cele două profiluri au aceleași benzi și aceleași
câștiguri, și diferă **numai** pe `WP_SPD_DN` / `LAND_SPD_MS`. Testul a
prins abaterea imediat ce a fost scris — nu o presupunere, ci diferența
reală din fișier.

Tot de acolo: benzile ambelor profiluri sunt acum aceleași (8 / 3 / 0.5 m),
cu o a patra bandă „contact" sub 0.5 m. Banda de contact **nu** e cea care
oprește corecțiile laterale — aia e `FINAL_DESCENT` din mașina de stări
(§8, sub 0.4 m); aici doar se scoate autoritatea care ar rămâne disponibilă.

Procedura de reglaj, cu ordinea și motivul fiecărui pas, e în
`docs/DIAGNOSTIC_OSCILATIE.md`.

#### Corolar: același bug de ceas făcea un test să **treacă** din coincidență

§5.39 descrie amestecul de ceasuri în codul de aplicație. Aceeași formă
exista de luni de zile în **harness-ul de test**, și acolo efectul era
invers — nu un monitor inert, ci un test verde fără acoperire.

`FakeVehicle.request_param()` stampila `params_t` cu `time.monotonic()`.
Bucla de test numără de la `1000.0 + t`. `authority._read_fresh` compară
cele două, deci parametrul părea „proaspăt" **numai cât timp uptime-ul
mașinii depășea 1000 s**.

După o repornire (uptime 14 minute, `monotonic ≈ 891`) niciun parametru nu
mai era acceptat la salvare, modularea de autoritate nu se mai aplica deloc,
și testul de cablaj a picat cu *„autoritatea nu a fost modificată
niciodată"*. Nimic nu se stricase: acoperirea lui fusese accidentală tot
timpul.

| uptime la rulare | ce măsura testul |
|---|---|
| > ~17 min | cablajul, corect |
| < ~17 min | nimic — modularea nu pornea |

Al doilea, în aceeași rulare: o verificare scria `assert now_sim([sursa_goală]) > 1000.0`, pornind de la ideea că `time.monotonic()` e „un număr mare". Aceeași dependență de uptime, în direcția cealaltă — testul pica după repornire deși codul era corect.

**Regula:** un test nu are voie să compare ceasul intern al scenariului cu
`time.monotonic()`, nici să presupună ceva despre mărimea lui. Harness-ul
avansează acum explicit ceasul vehiculului fals (`v.now`), iar o gardă de
regresie verifică faptul că `params_t` e stampilat de acolo, nu de la
perete.

**Al treilea caz, în aceeași rulare.** Cazul negativ al verificării de
stivă (`test_pi_tooling.py`) pretindea că suntem pe Pi și aștepta `ESEC`
pentru „numpy din venv". Rulat în venv-ul de simulare — care are
`--system-site-packages`, deci numpy vine legitim din apt (§5.36) — regula
nu se declanșa, iar testul trecea fără să verifice nimic. Acum ambele
condiții se declară explicit, nu se moștenesc din interpretorul care se
întâmplă să ruleze suita.

**De ce merită paragraful:** un test care pică se repară. Un test care trece
din coincidență se repară doar dacă cineva îi schimbă accidental condiția de
mediu — aici, o repornire și o schimbare de venv. Între timp, rândul din
Compliance Matrix care se sprijină pe el spune ceva ce nu a fost verificat.

**Tiparul comun al celor trei:** verificarea depindea de o proprietate a
mediului pe care nimeni nu o declarase — uptime-ul mașinii, mărimea lui
`time.monotonic()`, de unde vine numpy. Regula practică: dacă un test are un
caz negativ, condiția care îl declanșează se **injectează**, nu se speră.

**Și un test care a încetat să testeze când s-a mutat pragul.** Verificarea
de histereză oscila cu ±0.3 m „în jurul pragului de 2 m", cu 2.0 scris de
mână. Mutat pragul la 3.0, oscilația a căzut **integral** într-o singură
bandă: testul raporta 0 treceri și trecea — fără să testeze nimic. Acum ia
pragul din profil (`PROFIL_IMPLICIT[1].min_agl`), deci se mută odată cu el.

A patra formă a aceleiași lecții din §5.11: un test care nu poate eșua nu e
test, iar o constantă duplicată în test e felul cel mai ieftin de a ajunge
acolo.

### 5.41 Prima rulare cap-coadă: blocată pe o tastă pe care nu o apasă nimeni

Campania a pornit Gazebo, a pornit SITL, a armat după 5 încercări, a urcat
la 10.58 m și a intrat în LOITER — apoi s-a oprit acolo, la infinit.

Cauza, din `handover.log`:

```
  Enter = ridic AUX 7 (Ctrl-C = iesire)
```

`tools/sim_handover.py` cere Enter când nu primește `--after`, ceea ce e
potrivit când îl rulezi de mână și fatal într-o campanie. `batch_sim` nu îi
dădea `--after`.

**Forma eșecului contează mai mult decât cauza.** Procesul nu a murit, nu a
scris nicio eroare și nu a expirat: pur și simplu nu s-a mai întâmplat
nimic. Într-o campanie de 20 de rulări lăsată peste noapte, asta nu e o
rulare picată — e una care nu se termină niciodată, iar celelalte 19 nu
pornesc.

Trei reparații, nu una:

- **`--after` e obligatoriu** în comanda din campanie, cu un test care îl
  cere și verifică să depășească fereastra de așezare de 1 s a porții.
- **`stdin` se închide pentru toți copiii** (`subprocess.DEVNULL`). Un
  `input()` dă atunci `EOFError` — eșec vizibil — în loc de așteptare
  tăcută. Garda prinde și pasul următor care ar cere o tastă, nu doar ăsta.
- **`PYTHONUNBUFFERED=1`**, fiindcă `nova_sim.log` era de **zero octeți** în
  tot acest timp. Python tamponează pe blocuri când scrie într-un fișier,
  deci logul rămâne gol până la ieșirea procesului — exact în minutele în
  care vrei să vezi unde a ajuns.

Ultima e cea care a costat cel mai mult timp de diagnostic: cu logul gol,
singurul indiciu era că procesul trăiește.

**Și o gardă pentru clasa de eșec, nu doar pentru cazul ăsta.** Abonarea
gz-transport la un topic inexistent **reușește** (§5.33). Cu `--world`
greșit, `SimTruth` ar tăcea la nesfârșit, `error_vs` ar întoarce `None` la
fiecare cadru, iar raportul ar ieși cu zero comparații fără să spună de ce.
`nova_sim.py` verifică o dată, după 5 s de simulare, dacă au sosit mesaje de
poziție, și numește cauza probabilă.


### 5.42 Markerul se vede de la handover — dar campania testa cazul ușor

Două întrebări care par una singură: *e prea departe ca să vadă markerul?*
și *ajunge acolo cum ar ajunge pilotul?*

**Prima: nu.** Măsurat pe randare sintetică, cu calibrarea de simulare,
cameră nadir, marker deplasat pe axa **scurtă** a cadrului (cazul cel mai
strâns):

| altitudine | lateral | unghi față de nadir | `marker_px` | marjă până la marginea cadrului | detectat | eroare range |
|---|---|---|---|---|---|---|
| 5 m | 2.74 m | 28.7° | 89 | 96 px | da | +0.10% |
| 8 m | 4.53 m | 29.5° | 56 | 96 px | da | +0.15% |
| 10.9 m | 6.25 m | 29.8° | 41 | 96 px | da | +0.88% |
| 12 m | 6.50 m | 28.4° | 37 | 127 px | da | +0.40% |

Reproductibil: `tools/check_handover_fov.py`.

Plafonul din §5.37 e deci **conservator**: chiar la limita lui rămân ~100 px
între marker și marginea cadrului, iar eroarea de range stă sub 1%. La
handover-ul care a stârnit întrebarea — 10.9 m altitudine, 5.45 m lateral —
markerul are 41 px și ~340 px de margine.

Cifrele sunt sintetice: fără blur de mișcare, fără zgomot, iluminare
perfectă (§5.20). Pentru **geometrie** răspunsul e ferm; pentru detecție în
condiții reale, E2.

**A doua: nu, și asta era scăpat.** Prima variantă a campaniei lăsa
vehiculul să decoleze vertical și să planeze, cu markerul deplasat lateral.
Geometric identic, și motivul scris atunci era *„nu există un zbor lateral
care să introducă propria lui tranzitorie în condițiile inițiale"*.

Exact invers față de ce trebuie. În cursă pilotul **ajunge zburând**, iar
viteza laterală reziduală din momentul handover-ului e condiția inițială pe
care segmentul autonom trebuie să o anuleze — și cea mai probabilă sursă de
oscilație de pendul (`docs/DIAGNOSTIC_OSCILATIE.md`). Eliminând-o, campania
valida cazul ușor și nu spunea nimic despre cel real.

Acum sunt **două poziții independente**, nu una:

| | ce e | constrângere |
|---|---|---|
| `raza_marker_m` | unde stă markerul în lume, față de punctul de decolare | 4–15 m: destul cât să existe un picior de zbor |
| `raza_m` | unde e vehiculul la handover, față de **marker** | plafonat de conul camerei (§5.37) |

Azimuturile sunt independente, deci direcția de apropiere nu e aliniată cu
offsetul final: vehiculul **nu** vine drept peste marker. Un test verifică
faptul că unghiul dintre ele acoperă tot cercul (măsurat: 0–180°, medie
~90°) — altfel campania ar testa un singur caz, și cel mai favorabil.

`--no-approach` reproduce comportamentul vechi, pentru comparație.

**Lecția de proces:** motivul scris în comentariu — „ca să nu introducem o
tranzitorie" — suna a rigoare experimentală și era de fapt eliminarea
variabilei care conta. Când o simplificare scoate din test exact fenomenul
pe care restul rundei încearcă să-l diagnosticheze, simplificarea e greșită,
oricât de curat sună.


### 5.43 Prima aterizare în Gazebo nu a fost aterizarea noastră

Ce s-a văzut: drona pe sol, departe de marker, fără nicio urmă de centrare.
Ce spun logurile — și de ce merită citite înainte de a reacționa la imagine:

```
fly.log       [fly] la 10.59 m dupa 8.4 s
              [fly] la tinta N=-14.01 E=5.68
              [fly] in LOITER, gata de handover
nova_sim.log  t=34.81  [IDLE] alt 1.42 m
              >> IDLE -> HANDOVER_CHECK   (AUX sus, alt 0.45 m)
              !! HANDOVER REFUZAT: altitudine in afara ferestrei: 0.2 m
sitl.log      Mode GUIDED -> Mode LOITER        (niciun LAND, niciodată)
```

**Secvența autonomă nu a pornit.** Poarta a refuzat corect: vehiculul era
deja pe sol când a fost cerut handover-ul. Ce a aterizat vehiculul e
ArduPilot, în LOITER, nu codul nostru.

**Cauza: în LOITER manșa de throttle comandă urcare/coborâre, iar
emițătorul simulat al SITL-ului o ține JOS.** `sim_fly_to.py` comuta în
LOITER și ieșea; injectorul RC pornea ~20 s mai târziu. În fereastra aia nu
era nimeni pe manșe, deci FC-ul își citea propriul emițător simulat și
cobora cu viteză maximă de la 10.59 m.

Aceeași capcană ca §5.32, din partea cealaltă: acolo *injectam* throttle
1100 și vehiculul cădea; aici **nu injectam nimic** și cade la fel.

Reparat: în campanie `fly_to` lasă vehiculul în **GUIDED**, unde poziția e
ținută activ de controler și manșele sunt ignorate (§6/15.3.1). Pentru uz
manual implicitul rămâne LOITER, care e modul realist pentru un pilot.
Rămâne de făcut, pentru fidelitate: injectorul să comute el în LOITER după
ce a început să țină manșele — atunci pilotul emulat e prezent tot timpul.

**Regula:** într-un stand, orice interval în care niciun actor nu ține
manșele e un interval în care FC-ul ascultă de valori pe care nu le-a pus
nimeni. Nu e „stare neutră", e o comandă.

#### Al treilea ceas amestecat, de data asta în instrumentare

Din același log: `lat p50 3385850 p99 3386819 ms`. Adică 3386 secunde —
**exact cât rula mașina**.

`PiDetector` calcula latența ca `time.monotonic() - t_capture`, iar
`t_capture` vine de la sursă. Pe Pi ambele sunt `monotonic`, deci era
corect. Cu `GazeboFrameSource` cadrele poartă timp de **simulare**, deci
scădeam două lumi și obțineam uptime-ul.

A treia oară aceeași formă ca §5.39 — după codul de aplicație și după
harness-ul de test, acum în instrumentare. `PiDetector` primește un `clock`
opțional; implicitul rămâne `time.monotonic`, deci pe vehicul nu se schimbă
nimic.

#### Ce am găsit și NU am reparat: `LANDING_TARGET` se trimite și în `IDLE`

§8 spune despre `RACE_MONITOR`: *„detector activ, **ZERO comenzi**, ring
buffer"*. Logul arată `LT 18 DS 18` cu starea `IDLE`, iar verificarea
directă confirmă:

```
stare: IDLE | LT: 0 DS: 0
dupa 5 detectii in IDLE -> LT: 5 DS: 5
```

`LandingStateMachine.on_detection()` emite `LANDING_TARGET` și
`DISTANCE_SENSOR` **necondiționat**, indiferent de stare.

Cât de grav e:

| | efect în afara segmentului |
|---|---|
| `LANDING_TARGET` | inert: `PLND_ENABLED` e 0 până la handover (§5.8) |
| `DISTANCE_SENSOR` | **nu e inert**: FC-ul are telemetru în tot zborul pilotului |

Al doilea contează. §5.9 arată măsurat că o citire de telemetru schimbă
`get_alt_above_ground_m()`, deci încetinirea de dinainte de contact, și că
poate face `NAV_TAKEOFF` să fie respins prin gardul „can't takeoff
downwards" (§5.7). Aici valorile sunt reale, nu o constantă falsă, deci
efectul e mai blând — dar rândul din Compliance Matrix pentru 15.2.3
(„companion-ul nu comandă nimic în afara segmentului") nu e susținut de cod.

**Nu l-am reparat**, deliberat: `nova/state_machine.py` e cod validat în
SITL, e interzis explicit de regula rundei 7, iar un test de regresie
verifică prin `git diff` că nu a fost atins. Filtrul corect e o listă
**pozitivă** de faze în care se emite (§5.25), nu o negație — și merită
decis, nu strecurat. Element deschis 25 din §7.


### 5.44 Monitorul de legătură avea marjă zero — și a oprit o secvență reală

A doua campanie cap-coadă a mers: poarta a **acceptat** la 10.91 m, ACQUIRE
a confirmat LAND, `DESCEND_TRACK` a pornit și vehiculul a coborât 1.2 m.
Apoi:

```
[SAFETY faza=DESCEND_TRACK] link_age: BRAKE - fara HEARTBEAT de 1.02 s (prag 1.00 s)
[SAFETY faza=CONFIRM] mode_confirm: BRAKE - FC raporteaza BRAKE dupa 1 comenzi
>> DESCEND_TRACK -> IDLE   (mod schimbat (17))
```

Supervizorul a făcut exact ce trebuia, pe o informație falsă. Ce s-a văzut
în Gazebo — o coborâre scurtă, o mică corecție laterală, apoi hover
constant — e chiar semnătura asta.

**Cauza e aritmetică, nu software.** ArduPilot trimite `HEARTBEAT` la
**1 Hz**, iar `Vehicle._request_streams()` cerea rate pentru
`LOCAL_POSITION_NED`, `ATTITUDE`, `GLOBAL_POSITION_INT`,
`EXTENDED_SYS_STATE` și `RC_CHANNELS` — dar **nu pentru `HEARTBEAT`**,
tocmai mesajul de care atârnă un monitor de siguranță. Cu
`LINK_MAX_AGE_S = 1.0`:

| | |
|---|---|
| interval heartbeat | 1.00 s |
| prag „legătură căzută" | 1.00 s |
| marjă | **zero** |

Orice jitter — o iterație mai lentă, un pachet pierdut — declanșează BRAKE.
Iar BRAKE în `DESCEND_TRACK` înseamnă încercarea autonomă anulată: 10 puncte
(8.3.2).

**Nu e artefact de simulare.** Pe vehiculul real raportul e identic: 1 Hz
față de un prag de 1 s. Ar fi apărut la primul zbor, și ar fi arătat ca o
problemă de cablu sau de telemetrie.

Verificat în sursă, nu presupus (§5.10):

```cpp
// GCS_Common.cpp
set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT, 1000);
// get_default_interval_for_ap_message:
//   "handle heartbeat requests as a special case because heartbeat is
//    not streamed"  -> interval = 1000
```

Deci implicitul e 1 Hz, dar `SET_MESSAGE_INTERVAL` pe `HEARTBEAT` **este**
acceptat. `Vehicle` îl cere acum la `HEARTBEAT_HZ = 5`, primul din listă,
ceea ce transformă pragul în „5 heartbeat-uri pierdute la rând".

**Reparat în `nova/vehicle.py`, nu în `nova/safety.py`**, deliberat. Un prag
de siguranță nu se slăbește ca să încapă un stream pe care pur și simplu nu
l-am cerut. Direcția corectă e să ceri informația la rata de care depinzi.

Trei lucruri care fac reparația să țină:

- **Un test leagă cele două constante**: `LINK_MAX_AGE_S × HEARTBEAT_HZ ≥ 3`.
  Niciuna nu se mai poate schimba singură fără ca testul să pice. Constantele
  stau în fișiere diferite, deci nimic altceva nu le-ar fi legat.
- **Un test verifică faptul că `HEARTBEAT` chiar e în lista de rate cerute.**
  Prima variantă a testului l-a ratat: `MAVLINK_MSG_ID_HEARTBEAT` este **0**,
  iar un index greșit în parametrii lui `command_long_send` face
  `ids[0]` să existe oricum. Un id care e zero iartă greșeli de indexare.
- **Intervalul observat se măsoară**, nu se presupune (`heartbeat_interval()`,
  median pe ultimele 20). §5.10 aplicat unui stream: un interval de mesaj nu
  se poate citi înapoi, dar se poate cronometra ce sosește. `nova_sim.py` îl
  raportează o dată, cu marja calculată, și scrie `FARA MARJA` dacă cererea
  nu s-a aplicat.

**Ce mai spune același log, și e o veste bună:** detecția în Gazebo merge —
`det 100%`, marker 41–47 px la 9.5–10.9 m, `range` în acord cu altitudinea.
Lanțul randare → `detectMarkers` → `solvePnP` → `LANDING_TARGET` → controler
funcționează; ce lipsea era ca secvența să fie lăsată să continue.


### 5.45 Pragul de 0.38 m din §5.2 e o cifră de nadir — în coborâre reală e ~1 m

Coborârea a funcționat: 10.90 → 1.06 m, marker 41 → 423 px, `det 100%`,
`range` în acord cu altitudinea pe tot parcursul. Apoi, la ~1 m:

```
[SAFETY faza=DESCEND_TRACK] detection_age: BRAKE - ultima detectie acum 0.53 s
```

Detecția s-a pierdut la **1.06 m**, nu la 0.38 m cum prezice §5.2 — și nu a
mai revenit nici în hover, la 0.51 m.

**Detectorul nu e de vină.** Randat sintetic, la nadir, cu aceeași
calibrare, markerul se detectează până la **0.36 m** (1233 px), exact cât
spune formula. Deci cauza e geometrică, nu de imagine.

**Ce lipsea din §5.2: verificarea presupune camera la NADIR și eroare
laterală zero.** Bugetul real are trei termeni:

```
h · tan(VFOV/2)  ≥  lateral  +  h · tan(înclinare)  +  0.24
```

Al doilea termen e cel perfid: vehiculul se înclină **tocmai ca să corecteze
lateral**, deci exact când eroarea e mare, cadrul se mută în direcția
greșită. Altitudinea minimă la care markerul mai încape întreg:

| eroare laterală | 0° | 5° | 10° | 15° | 20° |
|---|---|---|---|---|---|
| 0 cm | **0.35 m** | 0.40 | 0.46 | 0.56 | 0.73 |
| 5 cm | 0.42 | 0.48 | 0.56 | 0.68 | 0.88 |
| 10 cm | 0.49 | 0.56 | 0.66 | 0.80 | 1.03 |
| 20 cm | 0.63 | 0.73 | 0.85 | **1.03** | 1.33 |
| 30 cm | 0.78 | 0.89 | 1.04 | 1.27 | 1.64 |

Cifra din §5.2 e colțul din stânga sus. `tools/check_handover_fov.py`
tipărește tabelul pentru calibrarea curentă.

> **Corecție: tabelul e corect, atribuirea a fost greșită.** Am pus
> pierderea de la 1.06 m pe seama a 20 cm laterali și ~15° înclinare,
> pentru că cifrele se potriveau. Rularea următoare, cu diagnostic, a
> măsurat de fapt **0.8 cm lateral și 0.1° înclinare** — centrare
> practic perfectă, cu 39 cm de marjă. Cauza reală e în §5.46.
>
> Bugetul de mai sus rămâne valabil ca fizică și e relevant pentru
> hardware, unde erorile sunt mai mari. Dar o potrivire numerică nu e o
> măsurătoare: aceeași lecție ca §5.11, de data asta în interpretare, nu
> în test.

Explică și de ce detecția **nu revine**: după BRAKE vehiculul rămâne unde
s-a oprit, cu eroarea laterală de atunci, la 0.51 m. Acolo marja e
0.35 − 0.24 = 11 cm; orice offset mai mare ține markerul în afara cadrului
la infinit.

**Consecință pentru arhitectură, de decis, nu de strecurat.** §8 oprește
corecțiile laterale sub 0.4 m, cifră aleasă din pragul de nadir. Bugetul de
mai sus spune că fereastra utilă se închide mult mai sus, și că închiderea
depinde de cât de bine a mers coborârea. Trei direcții, niciuna gratuită:

1. `FINAL_DESCENT` să înceapă mai sus (0.8–1.0 m), unde markerul sigur
   încape — cu prețul ultimilor centimetri de corecție;
2. limitarea înclinării în ultimul metru (prin `WP_ACC`, deja în profilul de
   autoritate) ca termenul al doilea să nu crească;
3. criteriul de încadrare să fie calculat din altitudine **și** din eroarea
   laterală curentă, nu doar din `marker_px`.

Prima e o **valoare**, nu cod: `SequenceConfig.no_lateral_alt_m`, expusă
acum ca `--no-lateral-alt`. Se poate încerca 0.5 sau 0.8 m fără să atingi
logica validată — adică un experiment cu o singură variabilă (§5.40).

Celelalte două ating `nova/state_machine.py`. Element deschis 26 din §7.

**Ce spune tabelul despre „ține-o fixă la 0.5 m, apoi coboară drept".**
Ideea e corectă și e chiar direcția 1, dar altitudinea contează: la 0.5 m
marja e 11 cm. Dacă vehiculul ajunge acolo cu 20 cm de eroare, markerul e
deja în afara cadrului și nu mai are cum să se centreze. Ordinea corectă e
**întâi fixarea, apoi coborârea**: oprește coborârea pe la 0.8–1.0 m, lasă
eroarea și înclinarea să se stingă, și abia apoi coboară vertical — de la
eroare zero, fereastra se închide la 0.35 m, deci drumul până la contact e
liber. Invers — cobori la 0.5 m și abia acolo încerci să te fixezi — cere ca
centrarea să fi reușit deja.

Partea de „așteaptă până e fixă" e logică nouă în mașina de stări, nu un
prag; de decis la runda următoare.

**Ce am făcut în schimb:** `nova_sim.py` raportează, la pierderea detecției,
geometria din adevărul simulării — altitudine, eroare laterală, înclinare,
deviere, marjă — și spune explicit *„e GEOMETRIE, nu imagine"* sau invers,
plus salvează cadrul care a picat (`--dump-dir`). Diagnosticul se declanșează
la 0.35 s, sub pragul de 0.5 s al supervizorului, ca raportul să fie scris
**înainte** ca BRAKE să schimbe geometria.


### 5.46 Gimbalul modelului stătea în câmpul camerei — și tăia zona liniștită

Diagnosticul de la §5.45 a răspuns din prima rulare, și a infirmat ipoteza
pe care o construisem:

```
[sim] DETECTIE PIERDUTA de 0.36 s in DESCEND_TRACK
  altitudine 0.93 m
  adevar: lateral 0.8 cm, inclinare 0.1 deg (roll +0.1, pitch +0.0)
  => markerul incape, cu 38.9 cm de marja: cauza e in IMAGINE
```

Centrare practic perfectă. Cadrul salvat arată de ce a picat totuși:
**corpul gimbalului**, atârnat sub vehicul, taie colțul din stânga-sus al
markerului. Nu umbra — umbra corpului cade la 46° de nadir, iar markerul
ocupă ±14.5°; obstacolul e la ~19°, adică ceva prins de vehicul.

`iris_with_gimbal` include `gimbal_small_3d` la **z = −0.125 m** față de
`base_link`, iar camera noastră stă la **−0.0745 m**. Gimbalul e deci exact
sub ea, în câmp.

**De ce rupe detecția, deși acoperă doar un colț.** Zona liniștită a
markerului e îngustă prin construcție: coala 600 mm, zona codată 480 mm,
deci `(600−480)/2 = 60 mm` — **0.75 dintr-un modul** ArUco (480/6 = 80 mm),
sub minimul uzual de un modul. Orice o atinge unește bordura neagră cu
fundalul întunecat și conturul nu se mai închide. E chiar mecanismul din
§5.18, produs aici fizic în loc de sintetic.

Și explică de ce efectul apare **jos**: obstacolul stă la unghi fix, iar
markerul crește în cadru pe măsură ce vehiculul coboară. Sub ~1 m se ating.

**Reparat:** `make_camera_model.py` scoate gimbalul din modelul derivat —
includerea, joint-ul, cele trei canale de control și cele trei
`JointPositionController`. NOVA nu are gimbal, deci modelul e și mai
fidel. `--keep-gimbal` păstrează varianta stock, pentru comparație.

**Două greșeli făcute în timpul reparației, ambele prinse de verificare, nu
de raționament:**

1. Prima regulă de ștergere era „orice `<plugin>` al cărui subarbore conține
   `gimbal::`". `ArduPilotPlugin` **conține** canalele de gimbal, deci a
   fost șters cu totul: modelul ieșea **fără motoare**. A ieșit la iveală
   pentru că modelul generat a fost verificat, nu presupus (§5.10).
2. Aceeași regulă, copiată în testul de derivare, excludea `ArduPilotPlugin`
   din comparație — adică fix divergența pe care testul trebuie să o
   prindă devenea invizibilă. Corect: se uită la `<joint_name>`-ul **propriu**
   al plugin-ului, nu la tot subarborele.

Testul de derivare (§5.31) **nu** a fost slăbit: declară explicit că singura
divergență permisă față de upstream e gimbalul, și verifică separat că au
rămas 4 controale de motor și `ArduPilotPlugin`. Un test slăbit la fiecare
schimbare nu mai prinde divergențele accidentale, care sunt tot ce apără.

**Pentru vehiculul real, întrebarea rămâne deschisă:** nimic nu trebuie să
atârne în conul camerei, iar zona liniștită de 60 mm nu iartă nici umbre,
nici murdărie, nici o piesă care intră puțin în cadru. De verificat la E2,
cu markerul tipărit și camera montată — element deschis 27.


### 5.47 Un raport care se scrie la ieșire se pierde la prima oprire din afară

Prima secvență completă reușită nu a lăsat nici CSV, nici JSON.

`nova_sim.py` scria raportul **după** buclă. `KeyboardInterrupt` era tratat,
deci un Ctrl-C mergea. Dar `SIGTERM` — de la `kill_group` al campaniei, sau
de la operatorul care închide Gazebo — omoară procesul pe loc: bucla nu mai
iese, codul de după ea nu rulează, iar tot ce s-a măsurat dispare.

Ce rămâne în urmă e doar logul: stările se văd, cifrele nu. Adică exact
inversul a ce vrei de la o rulare reușită.

Două reparații:

- **`SIGTERM` și `SIGINT` se ridică drept `KeyboardInterrupt`**, deci bucla
  iese normal și raportul se scrie. Cine oprește rularea nu pierde
  măsurătoarea.
- **Rularea se termină singură la 5 s după `HANDBACK`.** Campania rulează o
  singură secvență per rulare; odată în `HANDBACK` nu mai e nimic de
  măsurat, iar la RTF 0.25 restul bugetului de `--seconds` înseamnă minute
  de așteptare reală. Fără asta, fiecare rulare din campanie ar fi trebuit
  oprită de temporizator — adică exact pe calea care pierde raportul.

**Regula:** dacă un proces produce evidență, evidența trebuie să
supraviețuiască felului obișnuit în care procesul e oprit. Un raport scris
doar pe calea fericită nu e evidență, e noroc.


### 5.48 Corecția laterală își taie singură vederea — bugetul de înclinare

A doua rulare a picat altfel decât prima, iar diagnosticul a dat cifrele
direct:

```
[sim] DETECTIE PIERDUTA de 0.36 s in DESCEND_TRACK
  altitudine 7.16 m
  adevar: lateral 294.9 cm, inclinare 19.3 deg (roll +11.5, pitch +15.5)
  deviere din inclinare 250.6 cm; semi-cadru la sol 496.9 cm
  => markerul IESE din cadru cu 72.7 cm: e GEOMETRIE, nu imagine
```

Handover la 7.17 m cu markerul la 2.95 m lateral. Vehiculul se înclină 19.3°
ca să corecteze — și **înclinarea mută amprenta camerei cu 2.5 m**, exact în
direcția din care vine eroarea. Markerul iese din cadru, detecția se pierde,
supervizorul comandă BRAKE. De data asta §5.45 se aplică, și e măsurat.

**Bucla vicioasă, scrisă ca inegalitate:**

```
tan(înclinare)  ≤  tan(VFOV/2) − (lateral + 0.24) / h
```

Cu cât eroarea laterală e mai mare, cu atât ai voie să te înclini mai puțin
— adică exact atunci când ai vrea să corectezi mai tare. Iar ce guvernează
înclinarea e `WP_ACC`, prin `a = g·tan(înclinare)`:

```cpp
// ArduCopter/mode_land.cpp:68
pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(),
                                      wp_nav->get_wp_acceleration_mss());
```

Deci `WP_ACC` **este** plafonul de înclinare al coborârii autonome, nu un
reglaj de navigație. Verificat în sursă, nu presupus.

| | |
|---|---|
| `WP_ACC` implicit (iris) | 2.5 m/s² → **14.3°** în regim |
| buget la cazul măsurat (h 7.17, lat 2.95) | **14.0°** |
| marjă | **zero** — tranzitoriul a atins 19.3° |

**Două schimbări, pentru că inegalitatea are doi termeni.**

1. **`WP_ACC = 1.5`** în `config/nova_sitl.parm` *și* în
   `config/nova_flight.parm` — 8.7° în regim, deci ~6° pentru tranzitoriu. E
   o constrângere a **camerei**, nu a simulării: pe vehiculul real bugetul e
   mai strâns, fiindcă erorile de poziție sunt mai mari. Costă câteva
   secunde de corecție mai lentă; o încercare anulată costă 10 puncte.
2. **Raza planificată de campanie lasă buget de înclinare**, nu doar
   încadrare la nadir:
   `d_max = h·(tan(VFOV/2) − tan(15°)) − 0.24`.

| altitudine | nou | vechi | poarta |
|---|---|---|---|
| 5 m | **1.7 m** | 2.7 m | 6.5 m |
| 8 m | **2.9 m** | 4.5 m | 6.5 m |
| 12 m | **4.5 m** | 6.5 m | 6.5 m |

Formula veche — „încape cu 10% marjă, la nadir" — lăsa ~4° de înclinare la
limita ei. Prima corecție îi depășea imediat.

#### Consecința care nu se rezolvă din parametri: poarta acceptă mai mult decât se poate recupera

`nova/handover.py` acceptă **6.5 m** lateral la orice altitudine din
fereastra 5–12 m. Din tabelul de mai sus, 6.5 m nu e recuperabil la **nicio**
altitudine permisă: la 12 m limita e 4.5 m, iar la 5 m e 1.7 m.

Un handover acceptat la 6 m și 6 m altitudine nu e o încercare grea, e una
care **nu poate reuși** — și costă cele 10 puncte la fel ca un refuz, doar
că mai târziu și cu vehiculul jos.

Poarta nu se atinge aici (cod validat, regula rundei 7). Dar pragul ei ar
trebui să fie o **funcție de altitudine**, nu o constantă, iar un refuz care
spune „prea departe pentru altitudinea asta, urcă" trimite pilotul exact
unde trebuie. Element deschis 28.


### 5.49 Prima campanie: 7/18, și toate eșecurile au aceeași cauză

18 rulări (din 20, oprite manual), condiții variate. **7 reușite, 11
eșecuri** — iar eșecurile nu sunt împrăștiate, sunt identice:

| | |
|---|---|
| altitudinea la care s-a pierdut detecția | **0.46 – 0.56 m**, toate |
| eroare laterală în acel moment | **0.1 – 0.7 cm** |
| înclinare | **0.2 – 0.6°** |
| verdictul diagnosticului | **IMAGINE**, nu geometrie, la toate |

Deci bugetul de înclinare din §5.48 a funcționat: nu mai există niciun eșec
de geometrie, iar centrarea e practic perfectă la momentul pierderii.

**Cauza, din cadrul salvat: markerul e rotit în cadru.** Un pătrat rotit cu
θ are cutia de încadrare mai mare cu `|cos θ| + |sin θ|` — până la **41%**
la 45°. Criteriul din §5.2 măsoară **latura**, dar ce trebuie să încapă e
cutia.

Măsurat sintetic, cu aceeași calibrare:

| range | yaw | `marker_px` | cutie | `fits_in_frame` zicea | detectat |
|---|---|---|---|---|---|
| 0.42 m | 10° | 1057 | 1220 | DA | **da** |
| 0.42 m | 20° | 1056 | 1350 | DA | **nu** |
| 0.375 m | 0° | 1184 | 1180 | DA | **da** |
| 0.375 m | 10° | 1184 | 1367 | DA | **nu** |

Detecția pică exact când cutia depășește înălțimea cadrului (1296 px), nu
când latura depășește pragul. Pragul de 0.38 m din §5.2 e valabil doar la
rotație zero; la 45° urcă la **0.52 m range**, adică ~0.60 m altitudine.

Și explică de ce 11 din 18: rotația la handover e întâmplătoare — depinde de
capul vehiculului față de orientarea markerului.

**Trei schimbări:**

1. **`CameraModel.fits_in_frame(marker_px, yaw_deg=0)`** ține cont de
   rotație. Implicitul păstrează comportamentul vechi pentru apelanții care
   nu știu rotația.
2. **Campania trece în `FINAL_DESCENT` la 0.60 m**, nu la 0.40 m. Acolo
   coborârea e verticală și oarbă prin proiect, iar monitorul de detecție nu
   se mai aplică (§8) — deci pierderea detecției sub prag nu mai abortează.
   Exact ideea „ține-o fixă, apoi coboară drept", cu o cifră măsurată în loc
   de una aleasă.
3. Pragul din `SequenceConfig` rămâne 0.40 m: e cod validat, sub freeze-ul
   rundei. Element deschis 29.

#### Două cifre din raport erau greșite — ale comparației, nu ale sistemului

Campania a raportat `eroare range p50 = 10.8%` și `eroare unghi p50 = 12°`.
Ambele imposibile pentru un sistem care aterizează cu **0.55 cm** eroare
finală. Nu detectorul greșea, ci `nova/sim_truth.py`:

- **Range-ul se măsura de la originea vehiculului**, nu de la cameră, care e
  cu 74.5 mm mai jos. Bias de 0.7% la 10 m și **15% la 0.5 m** — exact acolo
  unde se decide aterizarea.
- **Unghiurile se comparau în cadre diferite.** Detectorul raportează
  înainte/dreapta (cadrul corpului); adevărul avea nord/est. Cele două
  coincid doar când capul e la zero, deci eroarea raportată era practic
  chiar yaw-ul vehiculului.

Reparate amândouă, cu un caz negativ care arată ce însemnau: cu vehiculul
orientat spre est, un marker la nord trebuie să iasă „1 m la stânga, 0 în
față".

**Cifrele valide din campanie** — cele calculate direct din pozițiile
adevărate, care nu treceau prin comparația stricată:

| metrică | p50 | p95 |
|---|---|---|
| eroare finală | **0.55 cm** | 1.99 cm |
| derivă captură → contact | 0.81 cm | 3.16 cm |
| altitudine `SCORING_CAPTURE` | 0.56 m | 0.58 m |
| durata secvenței | 36.0 s | 40.7 s |
| rată de detecție 3–12 m | 100% | 100% |

Comparabile cu Faza 1 (medie 1.57 cm, maxim 2.4 cm), de data asta cu pixeli
reali. Eroarea de range și cea unghiulară se remăsoară la campania
următoare, cu comparația reparată.

**Lecția, a treia oară în aceeași rundă:** o cifră care contrazice o altă
măsurătoare a aceluiași sistem e o eroare de măsurare până la proba
contrarie. 12° de eroare unghiulară și 0.55 cm de eroare de aterizare nu pot
fi ambele adevărate.


### 5.50 Adevărul se compară în cadrul în care măsoară detectorul, nu în NED

Campania de 10 rulări a raportat `eroare unghi p50 = 15.7°` și
`eroare range p50 = 3.5%` pe un sistem care ateriza cu **0.7 cm**. A doua
oară aceeași lecție ca §5.49: o cifră care contrazice altă măsurătoare a
aceluiași sistem e o eroare de măsurare până la proba contrarie.

Reparasem deja o parte (rotația în cadrul corpului), dar incomplet, în două
feluri:

**1. ENU și NED numără capul din axe diferite.** În Gazebo, `yaw = 0`
înseamnă nasul spre **est**; în NED, spre **nord**. Derivat din cuaternionul
ENU, unghiul iese rotit cu 90°, iar „eroarea" raportată devine chiar
offsetul de convenție. De aceea eroarea urmărea yaw-ul vehiculului în loc
să fie mică.

Reparat prin eliminarea sursei de ambiguitate, nu prin corectarea ei:
atitudinea se ia acum din **`ATTITUDE` raportat de FC**, care e NED prin
definiție. Gazebo rămâne sursă doar pentru poziții.

**2. `range_m` nu e distanța verticală.** `detector_pi` calculează
`(t·n)/n_z` — distanța pe **axa optică** până la planul markerului. Cu
vehiculul înclinat, aia e `vertical / cos(înclinare)`:

| înclinare | diferență |
|---|---|
| 8.7° (regim, cu `WP_ACC` 1.5) | 1.2% |
| 15° | **3.5%** |

Exact cât raporta campania. Adevărul folosește acum aceleași formule ca
detectorul: offsetul se rotește în cadrul corpului cu atitudinea completă
(3-2-1), iar range-ul se ia pe axa optică.

Trei cazuri verifică reparația, fiecare cu răspuns care nu poate ieși din
întâmplare: cap nord + marker la nord → **în față**; cap est + același
marker → **la stânga**; înclinat 15° cu markerul dedesubt → `5/cos(15°)`
exact. Plus `yaw_deg` și `tilt_deg` în CSV, ca o analiză ulterioară să nu
mai depindă de ce presupune cine o citește.

### 5.51 100% succes pe rulări care nu îndeplineau 8.3.3

Campania de 10 rulări după reparațiile din §5.48–§5.49: **10 din 10
reușite**, eroare finală p50 **0.71 cm**, p95 1.22 cm, rată de detecție
100%. Secvența completă, de fiecare dată.

Și totuși niciuna nu a făcut **captura de scoring**:

```
>> DESCEND_TRACK -> FINAL_DESCENT   (alt 0.54 m)
>> FINAL_DESCENT -> TOUCHDOWN_CONFIRM
```

`SCORING_CAPTURE` nu apare — nici ca stare, nici ca eveniment. Adică 8.3.3,
cerința care aduce punctele, a eșuat în toate zece, iar campania a raportat
100%.

**Criteriul de succes nu putea detecta lipsa lucrului măsurat.** Era
`stare_finala == HANDBACK`. §5.11, de data asta nu într-un test, ci în
definiția rezultatului unei campanii — cu atât mai periculos, fiindcă
numărul ăla ar fi ajuns direct în Compliance Matrix.

**Două cauze s-au suprapus.**

1. **`no_lateral_alt = 0.60` a mutat `FINAL_DESCENT` deasupra pragului.**
   980 px înseamnă range 0.46 m, adică ~0.54 m altitudine — sub pragul de
   0.60 m la care coborârea devine verticală. Ridicarea care a dus rata de
   succes de la 39% la 100% a scos tocmai captura din fereastră.
2. **Pragul de 980 px e fizic de neatins peste ~18° de yaw.** `marker_px`
   nu poate crește oricât: cutia lui de încadrare trebuie să rămână în cadru
   (§5.49), deci există un maxim detectabil, care scade cu rotația:

| yaw | `marker_px` maxim detectabil | pragul 980 atins? |
|---|---|---|
| 0° | 1231 | da |
| 15° | 1005 | da |
| **20°** | **961** | **nu** |
| 30° | 901 | nu |
| 45° | 871 | nu |

Iar rotația la handover e întâmplătoare. A doua cauză ar fi lovit oricum,
independent de prima.

**Reparat, în trei locuri:**

- **Captura se numără din EVENIMENT, nu din tranziția de stare.**
  `on_detection` emite `scoring_capture` și în `DESCEND_TRACK`, și în
  `FINAL_DESCENT`, dar schimbă starea doar din prima. O instrumentare legată
  de stare raportează `None` deși captura s-ar putea produce.
- **Succesul cere și captura**, cu motiv distinct în CSV: `HANDBACK dar
  FARA captura de scoring (8.3.3)` nu se mai confundă cu o secvență oprită.
- **`--scoring-px` e reglabil** din campanie. La **800 px** captura e atinsă
  până la 45° de yaw (maxim detectabil acolo: 871 px), iar range-ul
  corespunzător e 0.56 m — adică deasupra pragului de coborâre verticală,
  deci se produce în `DESCEND_TRACK`.

**Ce rămâne de decis, nu de reglat:** 980 e valoarea implicită din
`SequenceConfig`, aleasă în §6/8.3.3 din amprenta camerei la contact, fără
să știe de limita de încadrare la rotație. Coborârea ei la ~800 e o
schimbare de cod validat — element deschis 32. Alternativa e alinierea de
yaw cu markerul înainte de coborâre, care rezolvă și §5.49, dar e logică
nouă în mașina de stări.


### 5.52 Campania cu pragul coborât: 19/20 — și prima măsurătoare validă

`--scoring-px 800`, 20 de rulări: **19 reușite, 1 eșec** — iar eșecul e
tocmai cel pe care criteriul nou îl poate vedea: *„HANDBACK dar FARA captura
de scoring (8.3.3)"*. Înainte ar fi fost raportat ca succes.

| metrică | p50 | p95 |
|---|---|---|
| eroare finală | **0.64 cm** | 1.46 cm |
| derivă captură → contact | 0.69 cm | 1.54 cm |
| altitudinea capturii | 0.65 m | 0.67 m |
| durata secvenței | 36.3 s | 40.1 s |
| rată de detecție 3–12 m | 100% | 100% |

Captura se produce la 0.65 m, adică în `DESCEND_TRACK`, deasupra pragului de
coborâre verticală — exact cum prezicea calculul din §5.51.

#### Cifrele de detecție erau contaminate de fazele în care nu contează

Raportul inițial dădea `eroare unghi p50 = 1.8°` față de un prag de 0.5°.
Defalcat pe faze, tabloul e altul:

| fază | cadre | unghi p50 | unghi p95 | range p95 |
|---|---|---|---|---|
| `DESCEND_TRACK` | 11979 | 0.42° | 2.26° | 1.47% |
| `IDLE` | 1675 | 0.46° | **14.63°** | 1.73% |
| `ASCENT` | 1830 | 0.21° | 0.53° | **6.31%** |

Coada venea din `IDLE` — cadre de **dinainte de handover**, cu vehiculul
zburând spre punct și markerul mult în afara axei — și din `ASCENT`, unde
urcă și markerul redevine mic. În niciuna detecția nu conduce controlul,
deci eroarea ei nu spune nimic despre sistem.

Filtrul e acum o **listă pozitivă** (`FAZE_MASURATE`), ca
`DETECTION_MONITORED_PHASES` din supervizor și din același motiv (§5.25): o
fază nouă nu intră în statistici din greșeală. CSV-ul păstrează toate
cadrele, ca o analiză ulterioară să poată vedea și restul.

#### Și un tabel care se citea greșit, inclusiv de mine

Coloanele „eroare range" și „eroare unghi" din rezumatul campaniei conțin
deja un **p95 pe cadrele unei rulări**, iar tabelul aplică peste ele încă un
p50/p95 **peste rulări**. Deci `p50 = 2.8%` însemna „rularea mediană, la
percentila 95 a ei" — o coadă, nu o valoare tipică. Valoarea tipică reală,
pe cadre, e **0.48%**.

Le-am citit greșit prima dată exact ca oricine altcineva ar fi făcut-o.
Etichetele spun acum `p95/rul`, iar rezumatul explică ce e dedesubt. Un
număr agregat de două ori fără să scrie asta e o afirmație falsă cu formă de
tabel.

#### Ce rămâne nelămurit, și nu inventez o explicație

Pe `DESCEND_TRACK`, eroarea unghiulară e p50 **0.42°** (sub pragul I4 de
0.5°) dar p95 **2.26°**. Am verificat două ipoteze și niciuna nu ține:

| ipoteză | verificare | rezultat |
|---|---|---|
| decalaj de timp între adevăr și cadru | eroarea vs **rata** de înclinare | **infirmată** — p95 e cel mai mare tocmai la rata cea mai mică |
| efect de înclinare | eroarea vs **magnitudinea** înclinării | corelație reală, p50 0.31° → 0.69° |

Deci înclinarea contează, dar nu prin schimbarea ei în timp. Rămâne de
lămurit ce anume — nu scriu o cauză pentru că se potrivesc cifrele, greșeala
din §5.45. Ce se poate spune ferm: eroarea de range trece pragul de 3% la
p95 (2.88% agregat, 1.47% pe `DESCEND_TRACK`), iar cea unghiulară trece la
p50 și nu la p95.


### 5.53 Predicatul de conformitate al ArduPilot nu acoperă 15.2.5

`AP_NavEKF_Source::usingGPS()` e helper-ul pe care l-ai folosi instinctiv ca
să răspunzi la „primește estimatorul date GNSS?". Nu e suficient pentru
15.2.5:

```cpp
// AP_NavEKF_Source.cpp
return getPosXYSource(i) == SourceXY::GPS ||
       getPosZSource(i)  == SourceZ::GPS  ||
       getVelXYSource(i) == SourceXY::GPS ||
       getVelZSource(i)  == SourceZ::GPS  ||
       getYawSource(i)   == SourceYaw::GSF;
```

Ultimul termen verifică **doar** `GSF` (8). Dar `SourceYaw` are și:

| valoare | sursă | GNSS? | prinsă de `usingGPS()` |
|---|---|---|---|
| 1 | Compass | nu | — |
| **2** | **GPS** | **da** | **nu** |
| **3** | **GPS_COMPASS_FALLBACK** | **da** | **nu** |
| 8 | GSF | da | da |

Deci un set cu `EK3_SRC2_YAW = 2` ar fi raportat de firmware drept „fără
GPS" și ar alimenta estimatorul de cap direct din GNSS — exact ce interzice
regula. Iar yaw-ul contribuie la control: controlerul de poziție lucrează în
cadrul estimat.

`nova/ekf_source.py` folosește un predicat propriu, mai strict, cu toate
trei valorile de yaw. Lista de termeni e scrisă **pozitiv** (§5.25), ca o
sursă nouă adăugată de ArduPilot să nu scape neverificată.

**Trei reguli, fiecare cu test:**

- **Nu comuta ce nu ai citit.** Setul țintă se citește de pe FC înainte de
  comutare; un `PARAM_SET` acceptat tăcut nu e garanție (§5.10), iar aici
  garanția e chiar afirmația din Compliance Matrix.
- **Un set necitit nu e un set curat.** Un termen care lipsește produce
  refuz, nu acceptare tăcută. Necunoscut nu înseamnă conform.
- **Refuzul e total.** Dacă setul conține GNSS, nu se comută deloc — nu se
  comută și apoi se raportează. O comutare care încalcă regula e mai rea
  decât niciuna: vehiculul ar zbura autonom în afara conformității, iar
  logul ar arăta că am știut.

Restaurarea se face din **fază**, ca la supervizor și la modularea de
autoritate (§5.14): handback, abort, dezarmare, sau o fază pe care nimeni
nu a prevăzut-o. Un test folosește o fază inventată exact pentru asta.

`conform` e o **măsurătoare**: rămâne fals până când setul a fost citit,
verificat și comutat, cu `COMMAND_ACK` de la FC. „Am trimis comanda" nu e
același lucru cu „estimatorul nu mai primește GNSS".

**Ce nu spune încă nimic:** dacă vehiculul chiar poate ateriza fără GNSS.
Mecanismul e în loc și demonstrabil; comportamentul se măsoară cu o campanie
comparativă, aceeași sămânță, cu și fără `--no-gnss`.


### 5.54 15.2.5 nu cere hardware în plus — măsurat

Campania cu `--no-gnss`, 10 rulări, plan identic cu cel rulat cu GNSS
(aceeași sămânță, același n):

| | cu GNSS | **fără GNSS** |
|---|---|---|
| aterizări complete | 10/10 | **9/10** |
| eroare finală p50 | 0.715 cm | **0.67 cm** |
| eroare finală p95 | — | 1.39 cm |
| derivă captură → contact p50 | — | 0.68 cm |
| rată de detecție | 100% | 100% |

Singurul eșec a fost captura de scoring, nu aterizarea. **Precizia nu s-a
schimbat.**

Comutarea e verificată, nu presupusă — la fiecare rulare:

```
[EKF arm]     citesc EK3_SRC2_* inainte de a comuta
[EKF switch]  set 2 cerut; POSXY=0, POSZ=1, VELXY=0, VELZ=0, YAW=1
[EKF restore] set 1 cerut inapoi; faza HANDBACK
```

Valorile din linia de mijloc sunt **citite de pe FC**, nu din fișierul de
parametri. Asta e diferența dintre „am configurat" și „am demonstrat".

**De ce merge:** precision landing e relativă. Dacă estimarea derivă, ținta
derivă cu ea. Ce nu se anulează e viteza estimată, care amortizează — și
tocmai de aceea rezultatul nu era evident dinainte.

**Ce NU spune:** IMU-ul din SITL e idealizat. Deriva de pe vehicul, cu bias
real de accelerometru pe 35 s de segment, e necunoscută. Mecanismul e
demonstrat; magnitudinea derivei se măsoară la primul zbor.

#### Pragul de 800 px e marginal la rotație mare

Eșecul din campanie: yaw **−40.8°**, `marker_px` maxim atins **759** — sub
pragul de 800.

| | la yaw 40.8° |
|---|---|
| maxim teoretic (randare sintetică) | 873 px |
| maxim **măsurat în Gazebo** | **759 px** (87% din teoretic) |

Deci estimarea sintetică din §5.51 e optimistă cu ~13%. Randarea reală
pierde markerul mai devreme decât prezice geometria pură — plauzibil din
marginea colii care iese din cadru și din filtrarea texturii.

| prag | altitudine capturii | marjă față de 759 px |
|---|---|---|
| 800 px | 0.65 m | **−5%** |
| 750 px | 0.68 m | +1% |
| **700 px** | **0.73 m** | **+8%** |

700 px capturează mai sus, ceea ce pentru 8.3.3 e mai **bine**, nu mai rău:
amprenta camerei la 0.73 m e ~1.0 × 0.56 m, deci juriul vede markerul întreg
plus împrejurimi, nu un sfert din el.

**Soluția corectă nu e totuși un prag mai mic, ci alt criteriu.** Captura ar
trebui să se declanșeze pe *ultima detecție bună înainte de pierdere*, nu pe
o dimensiune fixă — pragul fix e o aproximare a acelei intenții, și e
sensibil la o variabilă (rotația) pe care nu o controlăm. Asta e logică nouă
în mașina de stări; element deschis 33.


### 5.55 O imagine greșită predată ca dovadă e mai rea decât una lipsă

J2: până acum secvența se încheia „cu succes" și nu preda juriului nimic.
`FrameRing` exista, dar numai în `tools/nova_service.py`; `nova_pi.py` nu
avea niciun handler pentru `scoring_capture`.

**Un singur ring, folosit de toți trei.** Mutat în `nova/frame_ring.py`:
serviciul RACE_MONITOR, aplicația de bord și simularea îl iau de acolo. Două
copii ar fi divergat la prima modificare — §5.14, de data asta prevenit în
loc de descoperit.

**Ringul se alimentează ÎNAINTE de detecție.** Cadrul cerut de 8.3.3 e
tocmai unul pe care markerul **nu** mai încape în cadru (§5.2): la contact,
camera e la 74.5 mm și vede sub un sfert din marker. Un ring alimentat doar
la detecție reușită ar pierde exact imaginea cerută. Un test verifică
ordinea celor două linii în sursă.

**Trei reguli pentru ca rezultatul să fie evidență, nu o poză:**

1. **Cadrul se alege după timestamp-ul CAPTURII**, purtat de eveniment — nu
   după cel al deciziei și nici după „ultimul cadru de acum". La 0.5 m/s,
   100 ms de întârziere înseamnă 5 cm de altitudine, adică alt moment al
   coborârii.
2. **Se caută înainte, nu în jur.** Un cadru de dinainte arată un moment mai
   sus. Dacă în fereastra de toleranță (0.2 s, șase cadre la 30 fps) nu
   există niciunul, nu se salvează nimic.
3. **Un cadru lipsă nu se înlocuiește.** Lipsa se raportează și se poate
   cere din nou; o imagine greșită predată ca dovadă nu se mai poate
   detecta de nimeni, niciodată.

Fiecare imagine primește un `.json` alături, cu `time_boot_ms` de la FC,
altitudinea și atitudinea. Un `.png` singur nu se poate pune în relație cu
`.bin` (6.2.1.30), deci nu e evidență.

**Se predau două imagini**, din același ring: cea de la `scoring_capture`
(mai sus, unde markerul se vede întreg — de aia există starea) și cea de la
`touchdown` (litera cerinței). Raportul spune `complet_8_3_3` doar dacă
există amândouă, iar campania scrie asta în CSV.

**Ringul e oprit implicit pe bord.** La 2304×1296 un cadru e ~3 MB, deci 30
de cadre înseamnă 90 MB din RAM-ul Pi-ului. Se pornește explicit, cu
`--ring-frames`, de aplicația care chiar predă imaginea — iar dacă lipsește,
aplicația o **spune** la pornire în loc să tacă și să nu producă nimic.



### 5.56 Campania măsura o configurație pe care vehiculul nu ar fi zburat-o

Trei lucruri găsite privind raportul unei campanii de 10 rulări care
arăta perfect: **10/10, eroare finală p50 0.555 cm, rată de detecție
1.000, imagini 8.3.3 în toate.** Primele două cifre sunt reale. A treia nu
măsura nimic.

**1. `rata_detectie` raporta 1.000 în orice condiții.** `_note_frames`
citea contorul cu `getattr(detector, 'n_frames', None)`. Contorul stă pe
`ArucoMarkerDetector`; detectorul primit e un `PiDetector`, care nu îl
expunea. Deci `getattr` întorcea `None`, funcția ieșea din prima
instrucțiune, iar `det_by_alt` primea numai `True`.

A patra oară aceeași formă ca §5.39 — și cea mai perfidă, pentru că
**testul care trebuia să o prindă exista și trecea.** Își injecta un obiect
fals cu `n_frames`:

```python
class _Det:
    n_frames = 0
app.detector = _Det()
```

Deci verifica aritmetica lui `_note_frames` pe o clasă care nu zboară
nicăieri. §5.40 spunea „condiția care declanșează cazul negativ se
**injectează**, nu se speră" — aici injectarea a fost chiar problema:
obiectul fals avea atributul pe care cel real nu îl avea.

Reparat cu o proprietate delegată pe `PiDetector` plus un test care se
uită la **clasa din producție**, nu la un dublu.

> **Semnul care trebuia citit:** `n_cadre = 0` scris lângă
> `rata_detectie = 1.000`, în același raport. Zero cadre procesate și 100%
> detecție nu pot fi ambele adevărate. Aceeași regulă ca §5.49: o cifră
> care contrazice altă cifră a aceluiași sistem e o eroare de măsurare
> până la proba contrarie.

**2. `lat_p99 = 0.000 ms`, raportat lângă un criteriu de 150 ms.** În sim
se măsoară `now - det.t`, ambele pe ceasul de simulare. Când detectorul
ține pasul cu randarea, ceasul nu a avansat între capturare și consumare,
deci iese exact 0. Aia e **întârziere de coadă**, nu latența de calcul
cerută de E1.4 — care se măsoară pe Pi 4 și nu are cum să fie zero.

Un `0.0` pus sub o coloană numită „latență" e o afirmație de conformitate
pe care simularea nu o poate susține. §5.29 aplicat unui raport: `-` și
`0` sunt lucruri diferite.

**3. Campania rula alte praguri decât vehiculul.**
`batch_sim.NO_LATERAL_ALT_M = 0.60` suprascria cei 0.40 m din
`SequenceConfig`. `tools/nova_pi.py` construiește `SequenceConfig(conv=...)`
— doar implicite. Deci **toate** cifrele din §5.52 și §5.54 descriu o
configurație pe care bordul nu ar fi zburat-o niciodată.

Nu era o valoare greșită. Era o valoare care exista doar în campanie.
Constanta a fost eliminată; knob-ul rămâne, dar trebuie dat explicit — și
atunci campania îl anunță ca experiment (§5.40). Un test compară acum
implicitele celor două.

**Tiparul comun al celor trei:** fiecare producea o cifră care arăta a
succes. Un test picat se repară în zece minute; o metrică moartă intră în
Compliance Matrix.

### 5.57 Un prag fix în pixeli nu putea fi corect la nicio valoare

Trei runde am reglat același număr — 980 → 800 → 700 — și de fiecare dată
marja se subția în același loc. Motivul e că numărul măsura altceva decât
condiția care contează.

`marker_px` e **latura**. Ce trebuie să încapă în cadru e **cutia de
încadrare**, mai mare cu `|cos θ| + |sin θ|` — până la 41% la 45° (§5.49).
Iar θ e rotația markerului în cadru, adică orientarea vehiculului la
handover: **o dă pilotul, nu o alegem noi.**

| prag | marjă până la pierderea detecției, la 0° / 41° |
|---|---|
| 980 px | de neatins peste ~18° (§5.51) |
| 800 px | +48% / **−5%** — o captură pierdută din 10 (§5.54) |
| 700 px | +69% / +8% |

Marja nu se subțiază puțin: se **prăbușește**, exact la rotațiile mari.

**Soluția nu e alt număr, e altă mărime.** `Detection` poartă acum `fill`:
cât din cadru ocupă cutia markerului, luată din **colțuri**. Colțurile
includ rotația, perspectiva și distorsiunea așa cum sunt, nu cum ar fi la
un pătrat ideal văzut de sus.

```
SCORING_FILL = 0.62   captura 8.3.3        marjă ≥ 1.33× la ORICE rotație
FINAL_FILL   = 0.72   coborâre verticală
SCORING_PX   = 700    rezervă, când fill lipsește
NO_LATERAL_ALT_M = 0.50   plasă, nu criteriu
```

Verificat pe 0–45°, secvență completă la fiecare: captura se produce de
fiecare dată, `fill` la captură e **0.63 constant**, iar `marker_px` la
captură merge 814 → 578. Cu prag fix de 700 px, jumătate din rotații ar fi
ratat captura.

**Ordinea celor două praguri e acum o proprietate, nu un noroc.** Captura
și trecerea la coborâre verticală se măsoară pe **același** criteriu, deci
nu se pot încrucișa. Când erau două mărimi diferite — pixeli și metri —
ordinea lor depindea de rotație, iar când s-a inversat, 8.3.3 a eșuat în
toate cele 10 rulări cu campania raportând 100% (§5.51).

**`None` nu înseamnă zero.** Un detector care nu raportează încadrarea
trimite `None` și consumatorul cade pe pragul în pixeli. Un `0.0` implicit
ar spune „markerul e mic" tocmai când e pe cale să iasă din cadru — §5.53
aplicat unui câmp de date.

**Ce rămâne nemăsurat:** `fill` nu egalizează marja complet. La 41°
randarea reală pierde markerul la `fill` 0.826, față de 0.914 la nadir —
deci rămâne o împrăștiere de 1.11×, față de 1.56× la pragul în pixeli.
Cauza celor 9% e aceeași bănuială ca în §5.54 (marginea colii, filtrarea
texturii), neconfirmată. Nu scriu o explicație pentru că se potrivesc
cifrele.

#### Bugetul de înclinare al camerei nu e pragul de integritate

Elementul 30, rezolvat pe jumătate — partea care se poate decide.
`safety.MAX_TILT_DEG = 30°` și limita camerei împărțeau un număr și nu
sunt același lucru:

| | ce e | formă |
|---|---|---|
| integritate | peste asta coborârea nu mai e controlată | **constantă**, 30° |
| cameră | peste asta markerul iese din cadru | **funcție** de altitudine și eroare laterală |

`CameraModel.tilt_budget_deg(alt, lateral)` reproduce exact cifrele
măsurate: 14.0° la 7.17 m cu 2.95 m lateral (§5.48, unde tranzitoriul a
atins 19.3° și markerul a ieșit cu 72.7 cm) și 19.5° la 1 m cu 10 cm.

| alt | 0 cm | 10 cm | 30 cm |
|---|---|---|---|
| 0.5 m | 12.1° | 0.8° | 0° |
| 1.0 m | 24.4° | 19.5° | 8.8° |
| 2.0 m | 29.9° | 27.7° | 23.0° |
| 12 m | 34.0° | 33.7° | 33.0° |

Sub ~2 m bugetul e mereu sub 30°, deci monitorul de înclinare **nu poate
proteja detecția**: până ajunge la prag, markerul a ieșit demult și a
declanșat monitorul de vârstă a detecției.

**Dar nu e o scăpare de reglat coborând pragul la 20°.** Un BRAKE pe
bugetul camerei ar anula un tranzitoriu recuperabil, și ar lovi exact când
controlerul face ce trebuie: se înclină ca să corecteze lateral. Bugetul se
apără **preventiv**, în trei locuri care există deja: `WP_ACC = 1.5`
(8.7° în regim), `final_fill` (iese din fazele supravegheate înainte ca
bugetul să se prăbușească sub 1 m), și poarta, când se face J4.

Deci bugetul se **măsoară**, nu se acționează pe el: `tilt_budget_deg` și
`tilt_margin_deg` per cadru în `frames.csv`. În Safety Case intră relația,
nu o singură cifră (§6/15.2.9).

### 5.58 `LANDING_TARGET` în IDLE: descoperit în runda 7, reparat în J5

§5.43 l-a găsit și l-a lăsat deliberat, sub freeze-ul rundei 7.
`on_detection` trimitea `LANDING_TARGET` **și** `DISTANCE_SENSOR`
necondiționat, în toate stările — deci și în `IDLE`, adică în tot zborul
pilotului.

Filtrul e o **listă pozitivă** (§5.25): `ACQUIRE`, `DESCEND_TRACK`,
`SCORING_CAPTURE`, `FINAL_DESCENT`. O fază nouă nu emite până nu o adaugă
cineva deliberat.

Două decizii care nu sunt evidente:

- **`ACQUIRE` e înăuntru.** PLND e deja armat, iar estimatorul primește
  măsurători înainte ca FC-ul să confirme LAND.
- **`TOUCHDOWN_CONFIRM` și `ASCENT` nu sunt.** Vehiculul e pe sol și apoi
  urcă, iar acolo un telemetru care raportează sub ținta de decolare e
  exact cazul măsurat în §5.9: `NAV_TAKEOFF` respins cu `result=4`, fără
  niciun `STATUSTEXT`. Cu `WP_RFND_USE = 0` efectul nu se manifestă, dar a
  depinde de asta e gratuit când lista poate fi corectă.

**Detecția se înregistrează în continuare în orice stare.** Se filtrează
doar emisia: poarta de handover și monitorul de vârstă a detecției depind
de `last_det` în `IDLE`, iar un filtru pus prea sus le-ar face oarbe. Un
test verifică ambele direcții — zero mesaje în `IDLE`, dar emisie în
`DESCEND_TRACK`, altfel filtrul ar rupe secvența fără ca nimic să spună.


### 5.59 Pe Pi 4, `/dev/serial0` nu e UART-ul pe care îl vrei

Primul lucru care se strică la bring-up, și cel mai greu de diagnosticat,
pentru că simptomul arată ca hardware.

Pe Raspberry Pi 4 sunt **două** UART-uri pe pinii GPIO 14/15:

| | dispozitiv | ceas |
|---|---|---|
| PL011 | `ttyAMA0` | propriu, stabil la orice baud |
| miniUART | `ttyS0` | **legat de frecvența miezului VPU** |

Implicit, PL011 e luat de Bluetooth, iar `/dev/serial0` arată spre
**miniUART**. Iar ceasul miezului se scalează cu încărcarea și cu
temperatura. La 921600 baud asta înseamnă o legătură care merge câteva
minute și apoi începe să dea caractere greșite — adică exact profilul unui
cablu prost sau al unui FC defect. Se caută în locul greșit ore întregi.

`dtoverlay=disable-bt` mută PL011 înapoi pe GPIO 14/15, și de acolo
`/dev/serial0 → ttyAMA0`.

Al doilea lucru, independent: Linux pune implicit o **consolă serială** pe
același port. Două programe pe un UART înseamnă că fiecare înghite din
mesajele celuilalt; pymavlink vede pachete tăiate, nu se plânge, și pur și
simplu nu ajunge niciodată la `HEARTBEAT`.

> Nu e același lucru cu §5.27. Acolo portul e **ocupat** și deschiderea dă
> `Errno 16`, adică un mesaj. Aici portul se deschide perfect și datele sunt
> doar corupte — genul de eșec care nu lasă nicio urmă.

`pi/setup_uart.sh` face ambele, plus grupul `dialout`, și are `--check`
care raportează starea fără să schimbe nimic. Caută `config.txt` în
`/boot/firmware` **și** în `/boot`: pe Bookworm/Trixie e primul, iar scris
în locul greșit fișierul se editează „cu succes" și nu are niciun efect —
§5.10 aplicat unui fișier de boot.

#### TELEM2 nu e SERIAL2 pentru că așa scrie pe carcasă

Verificat în sursă, nu presupus (§5.10):

```
libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6C/hwdef.dat:36
  SERIAL_ORDER OTG1 UART7 UART5 USART1 UART8 USART2 USART3 OTG2
                 0     1     2      3     4      5      6     7
```

iar `# telem2` e deasupra lui `UART5`. Deci TELEM1 = SERIAL1,
TELEM2 = **SERIAL2**. Pe alte plăci numerotarea nu se potrivește.

**Controlul de flux e capcana a treia.** `BRD_SER2_RTSCTS` are implicit
**2 = Auto** pe plăcile ChibiOS (`AP_BoardConfig.cpp:70`). Auto-detecția
testează dacă bufferul de ieșire se umple la pornire — iar cu RTS/CTS
nelegate, cum e cablajul de trei fire, rezultatul depinde de ce se întâmplă
să fie pe pini. Se pune **0 explicit**. Simptomul, dacă nu: legătura pare
moartă într-un sens, fără niciun mesaj nicăieri.

#### Fereastra fullscreen cere o sesiune grafică, deci un serviciu de utilizator

`nova-bringup.service` e **de utilizator**, legat de `graphical-session.target`.
Un serviciu de sistem pornește înaintea oricărei sesiuni: `imshow` aruncă
`Can't initialize GUI backend`, `nova/preview.py` se stinge singură și spune
de ce (§5.28) — deci serviciul ar porni, ar arăta verde în `systemctl
status`, și nu s-ar vedea nimic pe ecran. Cere autologin pe desktop.

Pentru vehicul, fără ecran, rămâne `systemd/nova-monitor.service`, care e de
sistem. **Nu se pornesc amândouă**: se bat pe `/dev/serial0` (§5.27).

---

## 6. Cerințe care constrâng software-ul

Referințele sunt la `ZDC_Regulations_V0_5.pdf`.

### 15.2.5 — interdicția GNSS (MECANISM IMPLEMENTAT, COMPORTAMENT MĂSURAT ÎN SIM)

Din momentul activării autonome, **niciun estimator sau filtru care
contribuie la ghidare sau control** nu are voie să primească date GNSS.

**Rezolvat prin seturi de surse EKF3 comutabile.** `nova/ekf_source.py`
comută pe `EK3_SRC2_*` la intrarea în segment și restaurează `EK3_SRC1_*`
la ieșire — handback, abort, dezarmare sau o fază neprevăzută (§5.14).

Setul autonom, citit înapoi de pe FC la fiecare rulare:

| parametru | valoare | sursă |
|---|---|---|
| `EK3_SRC2_POSXY` | 0 | None — nicio poziție orizontală |
| `EK3_SRC2_VELXY` | 0 | None |
| `EK3_SRC2_POSZ` | 1 | Baro |
| `EK3_SRC2_VELZ` | 0 | None |
| `EK3_SRC2_YAW` | 1 | Compass |

**Măsurat: 10 rulări, comutarea confirmată în toate, 9 aterizări complete
cu eroare finală p50 0.67 cm** — practic identică cu cele 0.715 cm de pe
exact același plan (aceeași sămânță, același n) **cu** GNSS. Singurul eșec a
fost captura de scoring, nu aterizarea (§5.54).

**De ce funcționează, și ce nu demonstrează asta.** Precision landing e o
măsurătoare **relativă**: ținta se calculează din poziția vehiculului plus
vectorul măsurat către marker. Dacă estimarea de poziție derivă, ținta
derivă odată cu ea, iar corecția relativă rămâne validă la ordinul întâi.
Ce nu se anulează e estimarea de viteză, care amortizează bucla.

Dar IMU-ul din SITL e idealizat. Pe vehiculul real, bias-ul de accelerometru
derivă mai repede, iar segmentul durează ~35 s. **Cifra de derivă e a
SITL-ului, nu a vehiculului** — de reverificat la primul zbor de test, cu
`.bin`-ul care arată poziția estimată față de cea reală.

Rândul din Compliance Matrix se poate scrie acum cu evidență: parametrii
citiți înapoi, `COMMAND_ACK` logat, restaurarea verificată. Vezi §5.53
pentru de ce predicatul din firmware nu e suficient.

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
declanșează captura pe **cât din cadru ocupă markerul**, nu pe altitudine,
plus ring buffer cu ultimele cadre. Se predau **două** imagini, amândouă
scoase din același ring după timestamp-ul capturii: cea de scoring
(markerul se vede întreg) și cea de contact (litera cerinței).

Implementat în `nova/scoring.py` + `nova/frame_ring.py` (§5.55).

**Criteriul nu mai e o dimensiune în pixeli** (§5.57). `marker_px` e
latura; ce trebuie să încapă în cadru e cutia de încadrare, mai mare cu
`|cos θ| + |sin θ|`, iar θ — rotația markerului în cadru — o dă orientarea
vehiculului la handover. Un prag fix are deci o marjă care se prăbușește
exact la rotațiile mari: 980 px e de neatins peste ~18° (§5.51), 800 px a
pierdut o captură din 10 la 41° (§5.54).

`Detection.fill` măsoară direct condiția care contează, din **colțurile**
detectate:

| | valoare | ce garantează |
|---|---|---|
| `SCORING_FILL` | 0.62 | marjă ≥ 1.33× până la pierdere, la **orice** rotație |
| `FINAL_FILL` | 0.72 | coborârea devine verticală; > `SCORING_FILL` prin construcție |
| `SCORING_PX` | 700 | rezervă, pentru un detector care nu raportează încadrarea |

Verificat pe 0–45°: captura se produce la fiecare rotație, `fill` la
captură e **0.63 constant**, iar `marker_px` la captură merge 814 → 578.

Deriva laterală între captură și contact: **maxim 1.2 cm pe 13 rulări**
(Faza 1); p50 0.375 cm, p95 0.786 cm pe ultimele 10 în Gazebo.

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
| 4 | ~~Test EKF fără GNSS în SITL~~ măsurat: 10 rulări, 9 aterizări, eroare neschimbată (§5.54). Rămâne: deriva reală pe hardware, unde IMU-ul nu e idealizat | 15.2.5 |
| 5 | ~~Fix urcare la 5 m după touchdown~~ validat în SITL | 15.2.7 |
| 6 | ~~Safety Supervisor~~ detecție + override + geofence validate în SITL | 15.2.9, 15.3.1, 15.2.4 |
| 11 | Distanța de frânare la fiecare treaptă din profilul D1 | 15.2.9 + D1 |
| 12 | ~~Ring buffer + imaginea de touchdown~~ implementat (§5.55): două imagini din același ring, alese după timestamp-ul capturii, cu evidență `.json`. Rămâne: verificat pe hardware că 30 de cadre încap în RAM-ul Pi-ului fără să fure din bugetul detecției | 8.3.3 |
| 13 | `STICK_DEADBAND_PWM` pe emițătorul de concurs | 15.3.1 |
| 14 | **E2**: validare offline a detectorului real (marker printat, ruletă, 3 condiții de lumină); până atunci `autonomy_enabled=false` | E0, 15.2.3 |
| 15 | Confirmare pe hardware: moduri de senzor IMX708 (30 fps binned, ~14 fps nativ), durata comutării de mod, controale aplicate, temperatură | §5.15, E2 |
| 16 | Latența detectorului pe Pi 4 (desktop: 4/6 ms; criteriu E2: p99 < 150 ms) | E1.4 |
| 17 | `tools/setup_pi.sh` rulat efectiv pe Bookworm (gărzile sunt testate, instalarea nu) | E2 |
| 18 | `nova-monitor.service` pornit sub systemd; `ProtectSystem=strict` poate bloca scrieri neanticipate | E2 |
| 19 | `check_params.py` pe FC-ul real, cu `config/nova_flight.parm` (fișier neverificat pe hardware) | scrutineering |
| 20 | `FLTMODE_CH` + `FLTMODE1..6` pe emițătorul de concurs; lipsesc deliberat din `nova_flight.parm` | 16.2.3, 15.3.1 |
| 21 | Paritatea OpenCV 4.10 verificată pe x86-64; Pi-ul e aarch64 (§5.24) | E2 |
| 22 | ~~Bucla închisă în Gazebo nu a rulat niciodată cap-coadă~~ — secvență completă la 21.09.2026 (§4b). Rămâne: campanie de rulări, nu o singură condiție | I4 |
| 22b | **Campania nu a rulat niciodată.** O secvență a mers; `batch_sim.py` cu N rulări și condiții variate nu a fost pornit, deci nu există distribuții. Mediul de dezvoltare nu poate rula Gazebo (`libEGL: failed to create dri2 screen`), deci rulează operatorul | I4, 8.4.2 |
| 23 | ~~Cifrele I4 — nicio măsurătoare~~ măsurate pe 20 de rulări (§5.52). Rămâne: coada erorii unghiulare pe `DESCEND_TRACK` (p95 2.26° față de pragul de 0.5°), cauză nelămurită; și latența, care se măsoară pe Pi, nu aici | 8.4.2, Safety Case |
| 24 | Distanța de frânare la 0.8 și 1.5 m/s, pentru `PROFIL_RAPID` (blocat până atunci) | 15.2.9, I5 |
| 36 | **15.2.5 nu e activ pe vehicul.** `nova/ekf_source.py` e implementat, testat și **măsurat** în sim (10 rulări, comutare confirmată prin citire înapoi, eroare finală neschimbată — §5.54), dar `tools/nova_pi.py` **nu îl instanțiază**. Deci pe aeronavă EKF-ul primește GNSS pe tot segmentul autonom, iar rândul din Compliance Matrix ar afirma ceva ce codul care zboară nu face. Lipsește o singură legătură în aplicația de bord, nu hardware. Blocant pentru orice încercare punctată | **15.2.5**, Compliance Matrix |
| 35 | **`nova/fence.py` nu e cablat nicăieri.** Modulul e validat în SITL — ciclu complet salvare → încărcare cerc de 10 m pe marker → citire înapoi → restaurare — dar nu îl instanțiază nici `nova_pi.py`, nici `nova_sim.py`, nici `fake_detector.py`. Deci stratul din firmware cerut de 15.2.4 **nu e activ**; rămân doar `_mon_radius` și `_mon_ceiling` din supervizor, care depind de Pi. Aceeași formă ca §5.14: piesa merge, cablajul nu există. Cere și lat/lon-ul markerului, care se deduce din poziția vehiculului plus offsetul măsurat la handover — logică nouă, deci de decis, nu de strecurat | **15.2.4**, Compliance Matrix |
| 34 | **Criteriul pe încadrare nu a rulat încă o campanie în Gazebo.** Pragurile (0.62 / 0.72 / 0.50) sunt derivate din geometrie plus două puncte măsurate de pierdere a detecției (§5.49, §5.54), și verificate pe o baleiere sintetică 0–45° care dă captură la fiecare rotație. Dar cifrele de eroare finală, derivă și rată de succes sunt încă cele de la pragul în pixeli. De rulat: `batch_sim.py --n 10`, cu `scoring_fill` și `tilt_margin_deg` în CSV | 8.3.3, 8.4.2 |
| 33 | ~~Captura pe o dimensiune fixă în pixeli~~ **REZOLVAT** (§5.57): criteriul e acum `Detection.fill` — cât din cadru ocupă cutia markerului, luată din colțuri. Marja e ≥ 1.33× la orice rotație, față de −5% la 41° cu pragul în pixeli. Rămâne: `fill` nu egalizează complet (0.826 la 41° față de 0.914 la nadir); cauza celor 9% e nemăsurată | 8.3.3 |
| 32 | ~~`scoring_px = 980` de neatins peste ~18° de yaw~~ **REZOLVAT** de 33: pragul în pixeli a devenit rezervă (700), iar criteriul e încadrarea. Alinierea de yaw cu markerul nu mai e necesară pentru 8.3.3 | 8.3.3 |
| 31 | **Plafonul de 12 m al porții e mai conservator decât măsurătoarea.** §8 l-a ales din estimarea „la 20 m markerul are 22 px, prea puțin"; măsurat sintetic cu calibrarea curentă, detecția merge până la **17 m** la orice rotație, iar la 20 m pică doar la yaw 45°. La 15 m `raza_max` crește de la 4.5 la 5.7 m, deci pilotul are mai multă libertate. Costă însă timp de coborâre (+10 s la 0.5 m/s de la 15 m față de 10 m) contra celor 40 de puncte de timp, iar eroarea de range la 30–35 px e 1–5% (§5.23) exact unde ArduPilot o folosește pentru încetinire. Propus de utilizator (altitudine aleatoare 5–15 m în campanie); cere întâi ridicarea plafonului porții, cod validat | 15.2.3, 8.4.2 |
| 30 | **Două praguri distincte împărțeau un număr** — separate (§5.57): integritatea rămâne `MAX_TILT_DEG = 30°`, constantă; limita camerei e `CameraModel.tilt_budget_deg(alt, lateral)`, care reproduce cifrele măsurate (14.0° la 7.17 m / 2.95 m; 19.5° la 1 m / 10 cm). Decizia luată: bugetul se **măsoară** (`tilt_margin_deg` în `frames.csv`), nu se comandă BRAKE pe el — ar anula un tranzitoriu recuperabil exact când controlerul corectează. Apărat preventiv de `WP_ACC 1.5`, `final_fill` și, când se face J4, de poartă. Rămâne: relația în Safety Case, din campania următoare | 15.2.9, 8.3.2 |
| 29 | ~~`no_lateral_alt_m = 0.40` sub pragul la care rotația omoară detecția~~ **REZOLVAT** (§5.57): trecerea la coborâre verticală se decide pe `FINAL_FILL`, nu pe altitudine. Pragul a rămas ca **plasă** la 0.50 m, sub altitudinea la care încadrarea ar declanșa oricum (0.565 m la rotație zero), ca să nu preia și să taie captura | 8.3.3 |
| 28 | Poarta acceptă 6.5 m lateral la orice altitudine, dar 6.5 m nu e recuperabil la niciuna: bugetul de înclinare dă 4.5 m la 12 m și 1.7 m la 5 m (§5.48). Pragul ar trebui să fie funcție de altitudine | 15.2.3, 8.3.2 |
| 27 | Nimic nu trebuie să atârne în conul camerei de pe vehiculul real; zona liniștită de 60 mm e sub un modul ArUco și nu iartă umbre sau ocluzii parțiale (§5.46) | 8.3.3, E2 |
| 26 | Fereastra de încadrare se închide la ~1 m cu erori realiste, nu la 0.38 m (§5.45). De decis: `FINAL_DESCENT` mai sus, limitare de înclinare, sau criteriu care include eroarea laterală | 8.3.3, 15.2.9 |
| 25 | ~~`on_detection()` emite în toate stările, inclusiv `IDLE`~~ **REZOLVAT** în J5 (§5.58): `EMITTING_PHASES` e o listă pozitivă — `ACQUIRE`, `DESCEND_TRACK`, `SCORING_CAPTURE`, `FINAL_DESCENT`. Detecția se înregistrează în continuare în orice stare, altfel poarta și monitorul de vârstă ar rămâne oarbe | 15.2.3, Compliance Matrix |
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
                 └→ SCORING_CAPTURE    fill > 0.62, captură full-res
                     └→ FINAL_DESCENT  fill > 0.72, vertical, fără corecții
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
| Override pilot | `STICK_DEADBAND_PWM` 80, `OVERRIDE_HOLD_S` 0.1 s | LOITER + pasiv definitiv | toate fazele **autonome** |
| Vârsta ultimei detecții | `DETECTION_MAX_AGE_S` 0.5 s | BRAKE | `DESCEND_TRACK`, `SCORING_CAPTURE` |
| Rază față de handover | `GEOFENCE_RADIUS_M` 10.0 m | RTL | tot segmentul autonom |
| Plafon AGL | `CEILING_AGL_M` 30.0 m | RTL | tot segmentul autonom |
| Rată de coborâre | `MAX_DESCENT_RATE_MS` 2.0 m/s, 0.5 s | BRAKE | tot segmentul autonom |
| Înclinare | `MAX_TILT_DEG` 30°, 0.3 s | BRAKE | tot segmentul autonom |

Pragul de înclinare e al **integrității vehiculului**, nu al camerei. Cel
al camerei e o funcție de altitudine și eroare laterală
(`CameraModel.tilt_budget_deg`), e sub 30° în tot regimul de sub ~2 m, și
se **măsoară** în loc să declanșeze BRAKE — vezi §5.57.

> **„Toate fazele" înseamnă toate fazele autonome, nu chiar toate.**
> Supervizorul se armează din fază (`AUTONOMOUS_PHASES`, §5.14) și
> `update()` iese pe `if not self.armed` — deci în `IDLE` niciun monitor nu
> rulează, inclusiv cel de override. Nu e o scăpare: în afara segmentului
> companion-ul nu comandă nimic, deci nu există de la ce să preia pilotul.
> Contează la bring-up, unde e prima întrebare care se pune văzând ecranul.

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

**Coborârea devine verticală când markerul e pe cale să iasă din cadru**
(`FINAL_FILL = 0.72`), nu la o altitudine fixă. Autoritatea de corecție
acolo e oricum 2–4 cm; precizia se decide la 0.5–1.0 m. Coborârea finală e
verticală și lentă, ceea ce elimină și forfecarea de rolling shutter
(care e `v_lateral × T_readout`, deci ~15 mm la 0.5 m/s).

Ce face de fapt `FINAL_DESCENT`: **iese din fazele în care supervizorul
cere detecție validă.** Nu oprește corecțiile laterale — alea se opresc
singure când detectorul nu mai publică. Deci pragul decide când pierderea
markerului încetează să mai fie o defecțiune, iar asta trebuie să se
întâmple **înainte** ca fizica să o producă (§5.57).

**Detecțiile ajung pe MAVLink doar în `EMITTING_PHASES`** — `ACQUIRE`,
`DESCEND_TRACK`, `SCORING_CAPTURE`, `FINAL_DESCENT`. Listă pozitivă
(§5.25, §5.58). În rest companion-ul nu trimite nimic către FC, ceea ce e
chiar afirmația din Compliance Matrix pentru 15.2.3. Înregistrarea
detecției continuă în toate stările: poarta și monitorul de vârstă depind
de ea.

### Paritate sim ↔ hardware

Același cod pe desktop și pe Pi. Diferențele, toate în aplicația de intrare:

| | `tools/fake_detector.py` (sim) | `tools/nova_sim.py` (Gazebo) | `tools/nova_pi.py` (bord) |
|---|---|---|---|
| sursa de `Detection` | geometrie din poziția cunoscută a markerului | `GazeboFrameSource` → ArUco → `solvePnP` | `PiCameraSource` → ArUco → `solvePnP` |
| legătura | `udpin:127.0.0.1:14552` | `udpin:127.0.0.1:14562` | `/dev/serial0` @ 921600 |
| ceasul | `time.monotonic()` | **timpul de simulare** (§5.36) | `time.monotonic()` |
| bucla | `run_loop` | buclă proprie, aceeași ordine, verificată de test | `run_loop` |
| garda E0 | ocolită explicit, anunțat la pornire | ocolită explicit, anunțat la pornire | citită din `config/nova.json`, fără ocolire |
| adevăr de comparat | poziția din care s-a generat detecția | `nova/sim_truth.py`, din Gazebo | ruleta, la E2 |

`nova_sim.py` e singurul care nu poate folosi `run_loop`: ceasul trebuie să
fie cel de simulare, iar `run_loop` e validat și nu se atinge. Consecința e
un al doilea cablaj, adică exact riscul din §5.14 — de aceea un test compară
**ordinea apelurilor** din cele două, nu doar prezența lor.

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
§9 ca regulă: testele contează pe platforma de producție, nu pe cea de dezvoltare.
