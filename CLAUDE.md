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
- **HFOV 102°, VFOV 67°**
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

### Structura repo

```
~/nova-zdc/
├── start_sim.sh              # lansare completă a mediului
├── config/
│   └── nova_sitl.parm        # parametri ArduPilot, încărcați la boot
├── tools/
│   ├── fake_detector.py      # detector sintetic (Faza 1)
│   └── gamepad_rc.py         # punte gamepad → RC_CHANNELS_OVERRIDE
└── sim/
```

### Porturi MAVLink

| Port | Consumator |
|---|---|
| 14550 | MAVProxy / QGroundControl |
| 14552 | `fake_detector.py` |
| 14553 | `gamepad_rc.py` |

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

**13 rulări, toate 5 puncte** (prag: <10 cm):

| Configurație | Erori (cm) | Captură (m) | Derivă (cm) |
|---|---|---|---|
| nominal | 0.8, 2.0 | 0.41 | 0.0 |
| `--noise-px 1.5` | 1.2, 1.2, 1.9, 0.8 | 0.39–0.44 | 0.0–1.1 |
| `--latency-ms 120` | 2.2, 0.8, 1.9 | 0.41–0.42 | 0.5–1.2 |
| `--dropout 0.15` | 2.0, 0.8, 2.0 | 0.39–0.45 | 0.3–1.2 |
| combinat | 1.6, 2.4, 1.6 | 0.42–0.44 | 0.0–0.5 |

Medie 1.57 cm, maxim 2.4 cm.

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

**Comportament la sol:** sub 1 m, când markerul nu e vizibil, trimite
altitudinea reală (`max(alt, 0.10)`). Peste 1 m, trimite o valoare
validă din mijlocul intervalului (20 m).

De ce contează: ArduPilot are un detector de aterizare care folosește
telemetrul. Cu o valoare constantă de 5 m, nu confirmă niciodată starea
"landed", iar **următorul takeoff e refuzat fără niciun mesaj de eroare**.

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

### 15.2.7 — urcare la 5 m după touchdown (NEREZOLVAT)

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
- **A** — rămâi în LAND, întârzii dezarmarea, comuți în GUIDED și urci
  fără re-armare (preferată, modificare mică)
- **B** — toată coborârea în GUIDED cu setpoint-uri proprii; control
  total dar renunți la fuziunea PLND validată

### 15.2.9 / 15.2.10 — Safety Supervisor determinist (NEREZOLVAT)

Sub pragul de încredere al detecției, aeronava trebuie să **planeze**,
nu să aterizeze. 15.2.10 cere explicit un supervizor determinist rulând
în paralel.

**Testul nostru a picat:** cu `PLND_STRICT 2` și pierdere totală a
detecției, vehiculul a coborât de la 6.6 m și a aterizat cu 33.4 cm
eroare, în loc să rămână în hover. ArduPilot a raportat
`PrecLand: Failsafe Measures` apoi `Disarming motors`.

Deci comportamentul trebuie implementat în codul nostru. Monitoare
necesare, toate independente de viziune:
- vârsta ultimei detecții valide → peste prag, `BRAKE` sau `LOITER`
- rază față de punctul de handover (10 m) și plafon (30 m AGL)
- rată de coborâre și înclinare
- starea de override

Pragul de încredere trebuie documentat numeric în Safety Case.

### 15.3.1 — override sub 250 ms

În GUIDED, ArduPilot **ignoră** intrările de manșă. Companion-ul trebuie
să le detecteze din `RC_CHANNELS` și să comande schimbarea de mod.

Bugetul: la 10 Hz implicit, doar streaming-ul consumă 100 ms. Cere
`RC_CHANNELS` la 50 Hz.

**Soluția robustă:** abort pe un comutator mapat direct pe un mod de
zbor prin `FLTMODE_CH`. Comutarea e gestionată de FC, nu trece prin Pi.
Detecția pe manșe rămâne necesară pentru 15.1.7, dar nu mai e calea
critică de siguranță.

### 15.2.4 — geofence 10 m + plafon 30 m

Monitorizare onboard, în FC sau mission computer independent.
Implementarea se descrie în Compliance Matrix și se verifică la
scrutineering.

Varianta recomandată: companion-ul încarcă un fence circular centrat pe
marker înainte de handover (`MISSION_TYPE_FENCE`), acțiune RTL.

Atenție la conflict cu fence-ul de limită a traseului (16.3.2), activ în
restul turului. Două geometrii, momente diferite, comutare de companion.

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

**Element deschis:** mail către organizatori pentru confirmarea că
măsurătoarea se poate face pe imaginea de la 0.45 m. Trimis odată cu
confirmarea despre formatul logurilor.

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
| 5 | Fix urcare la 5 m după touchdown | 15.2.7 |
| 6 | Safety Supervisor | 15.2.9, 15.2.10 |
| 7 | Măsurare latență override | 15.3.1 |
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
     └→ HANDOVER_CHECK  la comutarea AUX (canal 7)
         ├→ REJECT      alt <5m sau >12m, sau dist >6.5m, sau marker nedetectat
         └→ ACQUIRE     GUIDED activat
             └→ DESCEND_TRACK      LANDING_TARGET @ 20 Hz
                 └→ SCORING_CAPTURE    marker_px > 980, captură full-res
                     └→ FINAL_DESCENT  vertical, fără corecții laterale
                         └→ TOUCHDOWN_CONFIRM   contact + ≥1 s stabil
                             └→ ASCENT          ≥5 m AGL
                                 └→ HANDBACK

Safety Supervisor rulează în paralel pe tot parcursul, cu autoritate
de a comanda BRAKE / LOITER / RTL peste orice stare.
```

Plafonul de 12 m la handover e mai strict decât regulamentul (20 m),
deliberat: la 20 m markerul are 22 px, prea puțin pentru detecție 4×4
fiabilă. La 12 m are 37 px.

**Sub 0.4 m nu se mai fac corecții laterale.** Autoritatea de corecție
acolo e 2–4 cm; precizia se decide la 0.5–1.0 m. Coborârea finală e
verticală și lentă, ceea ce elimină și forfecarea de rolling shutter
(care e `v_lateral × T_readout`, deci ~15 mm la 0.5 m/s).

### Paritate sim ↔ hardware

Același cod pe desktop și pe Pi. Singura diferență:

```python
m = mavutil.mavlink_connection('udpin:127.0.0.1:14552')   # sim
m = mavutil.mavlink_connection('/dev/serial0', baud=921600)  # Pi
```

Abstractizarea camerei (fișier / Gazebo / picamera2) trebuie să aibă o
singură interfață, ca restul codului să nu știe de unde vin pixelii.

---

## 9. Convenții

- Comentarii în cod: română, fără diacritice
- Nu introduce dependențe noi fără motiv; stack-ul e pymavlink + OpenCV
- **Fără ROS 2.** Decizie luată deliberat, pentru paritate sim/hardware
- Orice parametru ArduPilot nou merge în `config/nova_sitl.parm`, cu
  comentariu care explică **de ce** — fișierul e sursă de evidență
  pentru Compliance Matrix
- Orice descoperire empirică se adaugă la §5 al acestui fișier
