# Sarcini NOVA — rundă 7: bucla închisă în Gazebo (Faza 3)

Citește `CLAUDE.md`. Până acum bucla a fost validată cu un detector
sintetic care calcula geometric ce ar vedea camera. Runda asta pune
**pixeli reali** în buclă: Gazebo randează markerul, OpenCV îl detectează,
mașina de stări îl folosește.

Scopul nu e să înlocuiască E2 pe hardware. E să testăm lanțul complet —
randare, detecție, `solvePnP`, `LANDING_TARGET`, control — în condiții
repetabile, cu adevăr cunoscut din simulare, și să putem rula în batch.

---

## I0 — ce s-a schimbat de la ultima rundă în SITL

Două lucruri care afectează testele:

**`PLND_ENABLED` are acum valoarea implicită 0.** E activat de companion
doar la `ACCEPT` în poarta de handover. Deci `mode land` comandat direct
coboară vertical, fără precision landing — corect pentru 16.2.3, dar
înseamnă că testele vechi care porneau cu `mode land` nu mai exercită
PLND.

**Poarta de handover e singura cale de intrare.** Trebuie declanșată prin
AUX 7. În SITL asta înseamnă fie gamepad conectat, fie injectare de
`RC_CHANNELS_OVERRIDE` dintr-un script.

Scrie `tools/sim_handover.py`: injectează comutarea AUX 7 la comandă,
respectă fereastra de așezare de 1 s, ține manșele în neutru. Fără el,
niciun test automat nu poate intra în secvență.

`autonomy_enabled` trebuie fals în `config/nova.json` pentru operare
reală și adevărat doar în configurația de simulare — păstrează separarea
existentă din runda 3.

---

## I1 — modelul markerului

`sim/models/aruco_26/` cu `model.config`, `model.sdf` și textura.

Textura: coală de 600 mm cu zona codată de 480 mm centrată, deci codul
ocupă 80% din latură. Generează-l cu `cv2.aruco.generateImageMarker`,
`DICT_4X4_50`, ID 26, la 2400 px.

```xml
<visual name="visual">
  <geometry>
    <plane><normal>0 0 1</normal><size>0.6 0.6</size></plane>
  </geometry>
  <material>
    <diffuse>1 1 1 1</diffuse>
    <pbr><metal>
      <albedo_map>materials/textures/aruco_26.png</albedo_map>
      <roughness>0.9</roughness>
      <metalness>0.0</metalness>
    </metal></pbr>
  </material>
</visual>
```

`roughness` 0.9 e hârtie mată. Expune-l ca parametru — 15.4.7 semnalează
explicit reflexia, iar la 0.3 vei vedea strălucirea.

**Atenție la convenția de axe.** Detectorul sintetic folosește NED
(`--north`, `--east`), Gazebo folosește ENU. În lume, `x = east`,
`y = north`. Un marker plasat greșit aici arată exact ca un bug de
convenție în detector — scrie maparea în `CLAUDE.md` §5.

`z = 0.01` ca să eviți z-fighting cu solul.

Lume nouă `sim/worlds/nova_marker.sdf`, derivată din cea folosită acum,
cu markerul la poziția configurabilă și un `<light>` cu direcție
parametrizabilă.

---

## I2 — senzorul de cameră pe dronă

Adaugă un senzor orientat în jos în modelul vehiculului, cu intrinsecii
**din calibrarea reală**, nu cu valori nominale:

```xml
<sensor name="down_cam" type="camera">
  <camera>
    <horizontal_fov>1.7768</horizontal_fov>
    <image><width>2304</width><height>1296</height><format>L8</format></image>
    <lens><intrinsics>
      <fx>937.0</fx><fy>933.7</fy>
      <cx>1139.4</cx><cy>649.0</cy>
    </intrinsics></lens>
    <distortion><k1>-0.0529</k1><k2>0.0696</k2><k3>-0.0295</k3></distortion>
    <clip><near>0.05</near><far>60</far></clip>
  </camera>
  <update_rate>30</update_rate>
  <pose>0 0 -0.0745 0 1.5708 0</pose>
</sensor>
```

Citește valorile din `config/camera_pi.yaml` și generează SDF-ul, nu le
hardcoda. Dacă cineva recalibrează, simularea trebuie să urmeze.

Înălțimea de 74.5 mm e cea din CAD; parametrizeaz-o.

**Randarea la 2304×1296 la 30 Hz poate coborî factorul de timp real sub
1.** Măsoară-l și raportează. Dacă e sub 0.5, oferă o rezoluție redusă
ca opțiune, cu intrinsecii scalați corespunzător — dar notează că
`marker_px` se schimbă, deci pragurile de detecție nu se transferă
direct.

---

## I3 — sursa de cadre din Gazebo

`FrameSource` există deja ca interfață în `nova/detector_pi.py`. Adaugă o
implementare pentru Gazebo, fără să modifici restul.

Două căi posibile; încearcă-le în ordine și raportează care funcționează:

1. **`gz-transport` Python bindings** — abonare la topicul de imagine,
   conversie `gz.msgs.Image` → numpy. Cel mai curat: cadre
   necomprimate, timestamp din simulare.
2. **`GstCameraPlugin`** din `ardupilot_gazebo`, stream UDP, citit cu
   `cv2.VideoCapture` și pipeline GStreamer. Funcționează, dar H.264
   introduce artefacte de compresie care degradează localizarea
   colțurilor — exact ce vrem să măsurăm. Dacă ajungi aici, setează
   bitrate mare și `tune=zerolatency`, și notează limitarea.

Timestamp-ul cadrului trebuie să fie **timpul de simulare**, nu ceasul
de perete — altfel rularea accelerată strică măsurătorile de latență și
vârsta detecției.

---

## I4 — bucla închisă și validarea

`tools/nova_sim.py`: aceeași aplicație ca `nova_pi.py`, dar cu
`FrameSource` de Gazebo. Restul lanțului identic — detector, mașină de
stări, supervizor, poartă.

### Validare față de adevărul din simulare

Gazebo știe poziția reală a vehiculului și a markerului. Compară la
fiecare cadru:

| Metrică | Prag |
|---|---|
| Eroare de range față de adevăr | < 3% |
| Eroare unghiulară | < 0.5° |
| Rată de detecție, 3–12 m | > 95% |
| Latență cadru → `LANDING_TARGET` | măsurată, raportată p50/p99 |

Asta e verificarea pe care detectorul sintetic nu o putea da: acolo
geometria era prin construcție corectă. Aici trece prin randare,
distorsiune, cuantizare și detecție reală.

### Rulare în batch

`tools/batch_sim.py`, folosind ceasul injectat din mașina de stări:
- poziție de marker uniformă în raza de 6.5 m
- altitudine de handover 5–12 m
- vânt: `SIM_WIND_SPD` 0–6 m/s, `SIM_WIND_TURB` până la 15
- unghi de lumină: azimut 0–360°, elevație 15–75°
- `roughness` al markerului: 0.9 și 0.3

N rulări, CSV cu eroarea finală, timpul per stare, altitudinea
`SCORING_CAPTURE`, deriva, succes/eșec și motivul.

Raportează **p50 și p95**, nu media. Asta e evidența de tip
`A - Analysis` pentru Compliance Matrix, care punctează pe
verificabilitate (8.4.2).

---

## I5 — oscilația de tip pendul

Ultimul test cu 120 ms latență injectată a arătat oscilație constantă în
coborâre. Cu detecție reală în buclă, acum se poate diagnostica pe
cauza reală, nu pe una simulată.

Testează în ordine, **un parametru odată**:

1. `PLND_LAG` setat la latența măsurată efectiv (I4). Fără el,
   estimatorul fuzionează o observație veche ca și cum ar fi curentă —
   cauza clasică de pendul.
2. `PSC_POSXY_P` redus cu 30%
3. `PSC_VELXY_D` crescut

Dacă modifici mai multe simultan și oscilația dispare, nu știi care a
ajutat, iar pe vehiculul real tuning-ul se reia de la zero.

### Profilul de autoritate pe altitudine

Companion-ul modulează la trecerea între praguri, cu citire înapoi:

| Altitudine | Coborâre | Autoritate |
|---|---|---|
| peste 8 m | 1.5 m/s | nominală |
| 8–3 m | 0.8 m/s | `PSC_POSXY_P` redus |
| 3–0.5 m | 0.3 m/s | redus mai mult, `WPNAV_ACCEL` limitat |
| sub 0.5 m | 0.2 m/s | fără corecție laterală (deja implementat) |

**`ANGLE_MAX` rămâne neatins** — pilotul are nevoie de autoritate
nominală pe traseu.

Valorile originale se salvează la handover și se restaurează la
handback **și pe calea de abort**. Un abort care lasă vehiculul cu
`PSC_POSXY_P` redus îi dă pilotului un vehicul mai lent decât se
așteaptă, exact când are nevoie de el. Test dedicat pentru asta.

Interacțiune de verificat: distanța de frânare a supervizorului e 1.00 m
la 0.5 m/s. La 1.5 m/s va fi mai mare, deci pragul sub care garanția de
hover nu mai ține urcă odată cu viteza. Măsoară pe fiecare treaptă și
documentează.

---

## I6 — ce Gazebo NU poate spune

Scrie-le explicit în `docs/LIMITE_SIM.md`, ca nimeni să nu prezinte
cifrele de aici drept validare de percepție:

- **rolling shutter** — Gazebo randează cadre instantanee
- **motion blur** — inexistent
- **expunere și AGC** — randarea are iluminare perfectă
- **zgomot de senzor** — absent
- **reflexia reală a hârtiei** — `roughness` e o aproximare
- **latența reală a Pi 4** — se injectează, nu se măsoară

Concluzia care contează: Gazebo validează **bucla de control cu pixeli
reali**. Percepția se validează la E2, pe hardware. Cele două nu se
substituie.

---

## Reguli

- Nu modifica `nova/state_machine.py`, `nova/safety.py`, `nova/handover.py`
- `nova/detector_pi.py` doar pentru noul `FrameSource`, aditiv
- Commit după fiecare literă, suita completă rulată după fiecare
- Descoperirile empirice → `CLAUDE.md` §5
- La final: ce rămâne deschis, ce n-a putut fi testat
