# Sarcini NOVA — rundă 4: unelte pentru E2

Citește `CLAUDE.md` și `SARCINI_RUNDA3.md`. E0 și E1 sunt încheiate;
`autonomy_enabled` rămâne fals. Runda asta pregătește tot ce trebuie ca
E2 (validarea offline a detecției) să poată fi rulat pe teren.

Trei unelte, în ordine. Oprește-te după fiecare și raportează.

---

## F1 — generator de ținte de calibrare (`tools/make_calib_target.py`)

Nu avem tablă de șah. Trebuie printată, iar imaginile găsite pe internet
sunt inutilizabile: rezoluție de ecran, grile pătrate simetrice, borduri
colorate care ating pătratele de la margine.

Scrie un generator care produce PNG la 300 DPI, pregătit de tipar:

```
python3 tools/make_calib_target.py --type charuco --paper A3
python3 tools/make_calib_target.py --type checker --paper A3 --cols 9 --rows 6 --square-mm 40
```

Cerințe:

- **Margine albă** de cel puțin o lățime de pătrat pe toate laturile.
  `findChessboardCorners` are nevoie de zona albă ca să delimiteze
  tabla; fără ea, colțurile exterioare sunt deplasate sau ratate.
- **Fără borduri colorate.** Orice tranziție care nu e alb-negru poate
  confunda detectorul.
- **Grilă asimetrică implicit** — 9×6 pătrate, deci 8×5 colțuri
  interioare. O grilă pătrată (8×8) e ambiguă la rotație: OpenCV o
  poate detecta rotită cu 90° între poze, ceea ce introduce erori tăcute
  în calibrare.
- Formate A4, A3, A2 landscape. Refuză cu mesaj clar dacă ținta nu
  încape pe hârtia cerută.
- Afișează la final dimensiunile nominale și instrucțiunile de tipar:
  scalare 100% (nu „fit to page"), măsurare cu rigla după tipar,
  lipire pe carton rigid.

Pentru ChArUco folosește `cv2.aruco.CharucoBoard`, cu marker la ~75% din
latura pătratului. Dicționar `DICT_5X5_250` — **nu** `DICT_4X4_50`, ca să
nu existe nicio șansă de confuzie cu markerul de misiune ID 26.

---

## F2 — suport ChArUco în `tools/calibrate_camera.py`

Verifică ce format acceptă unealta acum. Dacă e doar tablă de șah
clasică, adaugă ChArUco ca opțiune (`cv2.aruco.CharucoDetector` plus
`cv2.aruco.calibrateCameraCharuco`).

Motivul: la 102° FOV vrem ținta mare în cadru, inclusiv poze de aproape
unde depășește marginile. Tabla clasică pierde poza întreagă dacă un
colț iese din cadru; ChArUco tolerează vederi parțiale și tot produce
puncte utile.

Păstrează tot ce există deja — grila de acoperire 3×3, pragul de 0.5 px
RMS, minimum 20 de poze, filtrarea pe mediană a pozelor aberante,
refuzul focalei geometrice la `load()`.

Adaugă în `config/camera_pi.yaml` metadatele care fac calibrarea
trasabilă: tipul de țintă, dimensiunea măsurată a pătratului, numărul de
poze acceptate și respinse, RMS-ul final, rezoluția și `LensPosition`
folosite. Sunt evidență pentru Compliance Matrix, nu doar comentarii.

---

## F3 — verificator independent (`tools/verify_detection.py`)

Un coleg a dezvoltat separat un detector ArUco care funcționează
(marker de 240 mm detectat până la 10 m, adică ~22 px la focala lui).
Nu îl integrăm — `nova/detector_pi.py` face deja același lucru, mai
complet. Îl folosim ca **a doua implementare, scrisă independent**, ca
să verificăm încrucișat rezultatele de la E2.

Două implementări care dau aceleași colțuri și același `tvec` pe
aceleași imagini sunt evidență mult mai puternică decât una singură care
se autoconfirmă. Iar dacă diverg, avem un bug găsit pe imagini statice,
nu în zbor.

Scrie unealta pornind de la scriptul lui (`tools/script_cristi.py` —
îl adaug eu în repo), cu aceste modificări:

- **citire dintr-un director de imagini**, nu `cv2.VideoCapture`
- **calibrarea încărcată din `config/camera_pi.yaml`**, nu derivată din
  formula geometrică `f = W · d_full / s`. Acea formulă presupune
  proiecție liniară și e greșită cu zeci de procente la FOV larg.
  `dist_coeffs` zero e la fel de greșit la 102°.
- `MARKER_SIZE_M = 0.48`
- ieșire CSV: fișier, cele patru colțuri, `tvec`, `marker_px`,
  `fits_in_frame`

**Nu importa nimic din `nova/`.** Independența e rostul uneltei; dacă
partajează cod cu `detector_pi.py`, verificarea încrucișată nu mai
dovedește nimic.

Adaugă apoi `tools/compare_detectors.py`, care rulează ambele
implementări pe același director și raportează:
- diferența maximă și medie pe colțuri, în pixeli
- diferența relativă pe distanță
- cazurile unde una detectează și cealaltă nu

Prag de acceptare: sub 1 px pe colțuri, sub 1% pe distanță. Peste
acestea, raportează — e un bug într-una dintre implementări.

---

## Reguli

- Nu atinge `nova/state_machine.py`, `nova/safety.py`, `nova/handover.py`
- `autonomy_enabled` rămâne fals
- Fiecare unealtă nouă: test offline cu caz negativ
- Descoperirile empirice → `CLAUDE.md` §5
