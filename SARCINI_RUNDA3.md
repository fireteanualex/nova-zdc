# Sarcini NOVA — rundă 3: varianta de bord

Citește `CLAUDE.md` întâi. Grupurile A și B sunt încheiate și validate în
SITL; poarta de handover e singura cale de intrare în secvență.

Runda asta produce **codul care rulează pe dronă** plus documentația de
montaj. Grupul C (ring buffer) rămâne pe loc — se face după E1, pentru
că are nevoie de cadre reale ca să fie testabil.

Ordinea nu e negociabilă: E1 → E2 → E3 → E4 → E5. Fiecare depinde de
precedentul.

---

## E0 — regula de siguranță care guvernează toată runda

Niciun zbor autonom pe hardware real până când E2 nu a trecut criteriile
de acceptare offline. Detecția nevalidată într-o buclă de control care
comandă coborâre e modul cel mai direct de a distruge vehiculul.

Implementează asta ca gardă în cod, nu ca notă în documentație: un flag
în configurație (`autonomy_enabled: false` implicit) pe care mașina de
stări îl verifică la `HANDOVER_CHECK` și refuză explicit dacă e fals.
Se activează manual, după validare, cu un commit separat.

---

## E1 — detectorul real (`nova/detector_pi.py`)

Înlocuiește `tools/fake_detector.py` ca sursă de `Detection`. Aceeași
interfață din `nova/detection.py`, deci mașina de stări și supervizorul
nu se modifică deloc.

### E1.1 Camera

**`picamera2`, nu `cv2.VideoCapture`.** Camera Module 3 e CSI, nu USB.
Stack-ul V4L2 legacy a fost eliminat; `VideoCapture(0)` nu va deschide
camera.

Dual-stream, pentru că rezoluția de tracking și cea de scoring diferă:

| Flux | Rezoluție | Rată | Scop |
|---|---|---|---|
| `lores` | 2304×1296 | 30 fps | buclă de tracking |
| `main` | 4608×2592 | la cerere | cadru de scoring (grup C) |

Controale fixate la pornire, toate obligatorii:

```python
picam2.set_controls({
    "AfMode": controls.AfModeEnum.Manual,
    "LensPosition": 1.63,        # hiperfocală: clar de la 0.31 m la infinit
    "ExposureTime": 2000,        # µs — vezi E1.2
    "AnalogueGain": 8.0,
    "AeEnable": False,
    "AwbEnable": False,
})
```

Motivele, documentează-le în cod:
- **Autofocus manual**: PDAF-ul ar căuta focus exact în timpul coborârii
- **Expunere fixă**: AE-ul ar oscila când markerul alb-negru intră în cadru
- **2 ms**: blur-ul de mișcare e `v · t_exp · f/Z`; la 2 ms rămâne sub
  1 px pe tot profilul. Markerul are contrast maxim, tolerează zgomot
  mult mai bine decât blur.

### E1.2 Calibrare

Scrie `tools/calibrate_camera.py`:
- captură asistată de imagini cu tablă de șah (minimum 20, indicator de
  acoperire a cadrului)
- `cv2.calibrateCamera`
- salvează matricea și coeficienții în `config/camera_pi.yaml`
- raportează eroarea de reproiecție; peste 0.5 px înseamnă calibrare
  proastă, refuză să salveze

**Fără calibrare reală nu rula detectorul.** La 102° FOV distorsiunea
radială e severă la margini, iar `solvePnP` cu `dist_coeffs` zero dă
erori de pose care cresc exact acolo unde markerul se află în timpul
apropierii. Focala derivată geometric (`W/2 / tan(HFOV/2)`) e un punct
de plecare, nu un substitut.

### E1.3 Detecția

- `cv2.aruco.ArucoDetector`, `DICT_4X4_50`, filtrare pe ID 26
- `CORNER_REFINE_SUBPIX`
- `solvePnP` cu `SOLVEPNP_IPPE_SQUARE` — soluție analitică pentru
  markeri plani, fără ambiguitate, mai rapidă decât iterativul
- `marker_px` din latura medie a pătratului detectat
- verificarea `fits_in_frame` din `nova/detection.py` (§5.2) se aplică
  identic: sub ~0.38 m markerul nu încape întreg

**Nu aplica rotația de axe aici.** Rămâne în mașina de stări, unde e
acum — e o proprietate a mesajului `LANDING_TARGET`, nu a viziunii.
Detectorul publică unghiuri în cadrul camerei, curat.

Sub 5 m, decupează un ROI centrat pe ultima detecție (640×480 e
suficient, markerul are peste 90 px acolo). Salt mare de FPS pe Pi 4.

### E1.4 Instrumentare

Fiecare `Detection` poartă timestamp-ul **capturii**, nu al procesării.
Bucla măsoară și expune, ca percentile nu ca medie: latența
captură→publicare (p50, p99), FPS efectiv, rata de detecție.

---

## E2 — validare offline, înainte de orice zbor

Fără zbor. Marker printat, cameră pe stativ sau în mână.

Scrie `tools/measure_detection.py`:
- rulează detectorul pe un director de imagini sau pe un flux live
- raportează per set: rată de detecție, `marker_px` mediu, deviația
  `solvePnP` față de o distanță de referință măsurată cu ruleta

**Protocol de măsurare** (scrie-l ca `docs/PROTOCOL_E2.md`):
marker de 480 mm, cameră la 2, 4, 6, 8, 10, 12, 15 m măsurate, 20 cadre
per distanță, repetat în trei condiții de lumină (soare direct, înnorat,
umbră) pentru că 15.4.7 menționează explicit reflexia hârtiei.

**Criterii de acceptare:**

| Metrică | Prag |
|---|---|
| Rată de detecție la 12 m | ≥ 95% |
| Eroare de range la 5 m | < 5% |
| Latență captură→publicare p99 | < 150 ms |
| Temperatură Pi după 10 min în carcasă | fără throttling |

Ăsta e testul care decide dacă arhitectura ține. Dacă pragul practic de
detecție iese peste 40 px, fereastra operațională se închide —
acoperirea zonei de 6.5 m cere peste 10.3 m altitudine, detecția ar cere
sub 9 m, și nu mai există altitudine validă. Atunci raportează
**imediat**, e decizie de hardware (rezoluție mai mare sau a doua
cameră), nu de implementare.

---

## E3 — starea `REACQUIRE`

Acum, dacă detecția se pierde la 8 m, supervizorul comandă LOITER și
secvența se oprește. O încercare pierdută care ar putea fi salvată.

### Primitiva e urcarea, nu rotația

Camera privește în jos; rotația pe yaw nu schimbă ce se află sub dronă.
Amprenta crește liniar cu altitudinea, iar urcarea e mult mai rapidă
decât o baleiere completă și nu perturbă cadrul corpului.

Plus: căutarea prin translație ar avea nevoie de poziție orizontală, pe
care 15.2.5 o interzice din GNSS după handover.

```
REACQUIRE (intrare din DESCEND_TRACK la pierderea detecției):
    z_start = altitudinea curentă
    pentru z în [z_start + 2, +4, ... până la 13 m]:
        urcă la z, stabilizează 0.5 s
        dacă marker detectat 3 cadre consecutive → DESCEND_TRACK
    rotație 90°, verifică încă 1 s
    dacă tot nimic → ABORT
```

**Rotația de 90° e plasa de siguranță, nu primitiva.** FOV-ul e
asimetric (102° orizontal, 67° vertical); la 12 m acoperirea garantată
fără yaw e 7.9 m rază, cu o rotație de 90° urcă la 14.8 m. Costă 2-3 s
și verifică dacă markerul e ascuns în direcția îngustă.

**Confirmarea pe 3 cadre consecutive** filtrează fals-pozitivele. La
30 px, o detecție falsă izolată pe iarbă nu e imposibilă.

### Interacțiunea cu supervizorul

`REACQUIRE` intră în `DETECTION_MONITORED_PHASES`? Nu — altfel
supervizorul ar comanda LOITER exact în starea care încearcă să
recupereze detecția.

Dar are nevoie de propriul timeout, altfel devine o stare fără ieșire.
Limită totală de 15 s, apoi `ABORT`. Documentează asta ca excepție
justificată, cu test propriu, în spiritul `DETECTION_MONITORED_PHASES`.

**Bugetul de timp:** urcarea de la 5 la 12 m la 2 m/s plus stabilizări
înseamnă sub 8 s în cazul cel mai rău. Cu bonusul de 15 s din 8.2.2, o
recuperare reușită rămâne rentabilă. Una eșuată costă 15 s din slot —
acceptabil, pentru că oricum urma un abort.

---

## E4 — integrarea pe Raspberry Pi

### E4.1 Legătura serială

Pi 4 ↔ CUAV X7+ pe UART, **nu USB**. USB-ul adaugă latență variabilă și
se poate re-enumera în zbor.

Pe Pi:
- dezactivează consola serială pe `/dev/serial0`
- activează UART-ul în `/boot/firmware/config.txt`
- 921600 baud

Pe FC, parametrii merg în `config/nova_flight.parm`:
- protocol MAVLink2 pe portul folosit
- baud 921600
- ratele de stream necesare: `RC_CHANNELS` la 50 Hz (bugetul de override
  din 15.3.1), `LOCAL_POSITION_NED` și `ATTITUDE` la 20 Hz

Verifică fiecare prin `check_params.py`.

### E4.2 `config/nova_flight.parm`

Fișier **separat** de `nova_sitl.parm`. Diferențele obligatorii:

| Parametru | SITL | Zbor | De ce |
|---|---|---|---|
| `FS_THR_ENABLE` | 0 | 1 | failsafe RC real (16.3.1) |
| `FS_GCS_ENABLE` | 0 | 1 | failsafe GCS (16.3.1) |
| `PLND_ENABLED` | 1 | **0** | implicit off; companion îl aprinde la handover (16.2.3) |
| `FENCE_ENABLE` | — | 1 | 16.3.2 |
| `FENCE_ACTION` | — | 1 | RTL la breach |

`PLND_ENABLED=0` implicit e ce face funcția de aterizare de urgență
distinctă de cea autonomă, fără dependență de companion. Companion-ul îl
setează la 1 la `ACCEPT` în poartă și la 0 la handback sau abort, cu
citire înapoi de fiecare dată.

`RNGFND1_GNDCLR` = înălțimea măsurată a camerei, din E5.

### E4.3 Serviciu

`systemd` cu restart automat, log persistent, pornire după ce camera și
serialul sunt disponibile.

**Pornește în `RACE_MONITOR`, nu în `IDLE` armat.** Detectorul rulează
și umple ring buffer-ul, dar nu comandă nimic până la handover. Asta e
și cerința din arhitectură: monitor-only pe tot parcursul turului
manual.

### E4.4 Instrument de bancă

`tools/preflight_check.py` — rulat înainte de fiecare ieșire:
- camera deschide și produce cadre la rata așteptată
- calibrarea e încărcată, eroarea de reproiecție sub prag
- heartbeat MAVLink primit, versiune firmware raportată
- `check_params.py` pe `nova_flight.parm`
- `RC_CHANNELS` sosește la rata cerută
- home position setat

Cod de ieșire 0 doar dacă toate trec. Ăsta e și evidența pentru 14.2.3b
(GCS setează și verifică failsafe-urile înainte de zbor).

---

## E5 — ghid de montaj (`docs/MONTAJ.md`)

Document pentru cineva din echipă care nu a scris codul.

### Cablaj
- Pi ↔ FC: ce pini, ce port, ce nivel logic (verifică dacă X7+ scoate
  3.3 V pe UART; Pi nu tolerează 5 V)
- masă comună obligatorie
- alimentarea Pi: sursă separată sau BEC dedicat, cu curentul necesar
- camera: cablu CSI, lungime, rutare departe de ESC-uri și de firele de
  putere

### Montajul camerei

Trei măsurători care intră direct în configurație:

1. **Înălțimea centrului optic deasupra solului la contact.**
   Valoarea actuală din CLAUDE.md e 74.5 mm — **confirm-o pe vehiculul
   real**. Intră în `RNGFND1_GNDCLR` și în calculul amprentei.
2. **Offset-ul față de CG** (X/Y/Z în cm) → `PLND_CAM_POS_*`.
   `PLND_LAND_OFS_*` rămân 0: scorul se măsoară față de centrul
   imaginii, deci vrei boresight-ul deasupra markerului, nu CG-ul.
3. **Alinierea cu nasul dronei.** Axa verticală a imaginii trebuie să
   corespundă cu axa înainte. O eroare aici se manifestă ca rotație
   constantă a vectorului de corecție — exact simptomul de orbitare
   eliptică din §5.1, dar de origine mecanică.

**Verificarea de tilt**, cu procedură scrisă: dronă pe o suprafață
plană, marker centrat dedesubt, verifică dacă centrul imaginii coincide
cu centrul markerului. O înclinare de 1° la 12 m înseamnă 21 cm de
eroare. Sub 0.5° e acceptabil.

### Procedură de recepție

Checklist pe care cineva îl poate urma fără să înțeleagă codul:

1. Verificări vizuale de cablaj
2. Alimentare, `preflight_check.py`, toate verde
3. Cu elicele **demontate**: armare, verificare kill switch (16.2.1),
   verificare că re-armarea e refuzată (16.2.2)
4. Verificare comutator de aterizare de urgență: `PLND_ENABLED` citit ca
   0, mod `LAND` comandat (16.2.3)
5. Detecție statică: marker pe sol, dronă ținută la 2 m, confirmă
   detecție și range corect
6. Fotografii pentru Compliance Matrix: montaj cameră, cablaj, Pi

---

## Reguli

- Nu modifica `nova/state_machine.py` sau `nova/safety.py` mai mult
  decât cere adăugarea lui `REACQUIRE`. Sunt validate.
- Nu introduce dependențe noi în afară de `picamera2`
- Fiecare descoperire empirică → `CLAUDE.md` §5
- Fiecare parametru nou → `nova_flight.parm`, cu comentariu, verificat
  prin `check_params.py`
- Testele de siguranță păstrează cazul negativ
- La final: raport cu ce rămâne deschis și ce nu s-a putut testa fără
  hardware
