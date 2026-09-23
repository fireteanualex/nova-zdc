# Runda 8 — spre o configurație de competiție

Scop: **o variantă cât mai apropiată de cursă pentru zborul de test fizic.**
Nu „mai multe funcții" — ci ca ce zboară pe teren să fie ce zboară în
concurs, cu fiecare prag justificat de o măsurătoare și nu de o alegere.

Ce s-a schimbat față de runda 7: bucla închisă în Gazebo merge (19/20 cu
captură, eroare finală p50 0.64 cm), deci întrebările care erau teoretice
acum se pot **măsura**. Și, pentru prima dată, se pot atinge fișierele
înghețate — cu condiția ca fiecare atingere să vină cu motivul măsurat.

> **Regula rundei 7 se ridică, dar nu dispare.** `nova/state_machine.py`,
> `nova/safety.py` și `nova/handover.py` se pot modifica, însă fiecare
> modificare cere: motivul măsurat, un test de regresie, și o linie în §5.
> Testul care verifica prin `git diff` că nu au fost atinse se **înlocuiește**
> deliberat, nu se șterge — vezi J0.

---

## J0 — ridicarea înghețului, controlat

Testul `run_loop a ramas neatins` din `tools/test_sim_loop.py` rulează
`git diff` pe cele trei fișiere. Odată ce runda le deschide, el devine fals
și ar fi tentant de șters.

Se înlocuiește cu o gardă care apără **proprietatea**, nu fișierul: fiecare
dintre cele trei rămâne acoperit de suita lui, iar ordinea din `run_loop` vs
`SimApp.step()` rămâne verificată. Ce dispare e interdicția, nu acoperirea.

---

## J1 — 15.2.5 ✔ REZOLVAT (mecanism + măsurătoare în sim)

**Rezultat:** 10 rulări cu `--no-gnss`, comutare confirmată prin citire
înapoi în toate, 9 aterizări complete, eroare finală p50 **0.67 cm** față de
0.715 cm cu GNSS pe același plan. **Nu cere hardware în plus.** Detalii în
§5.53 (predicatul) și §5.54 (măsurătoarea). Rămâne de verificat deriva reală
pe hardware, unde IMU-ul nu e idealizat.

Ce urmează din text e contextul inițial, păstrat pentru trasabilitate.

**Cel mai mare risc deschis, și singurul care poate invalida tot.** Din
momentul activării autonome, niciun estimator care contribuie la ghidare sau
control nu are voie să primească date GNSS. Configurația actuală **nu
respectă** regula.

Mecanismul, verificat în sursă:

- `MAV_CMD_SET_EKF_SOURCE_SET`, param1 = 1..3 (`GCS_Common.cpp:5133`);
  comanda întoarce `ACCEPTED` sau `DENIED`.
- Predicatul de conformitate e `AP_NavEKF_Source::usingGPS()` și are
  **cinci** termeni: `POSXY`, `POSZ`, `VELXY`, `VELZ` și `YAW == GSF`.
  Ultimul e cel care se uită — GSF e estimatorul de yaw asistat de GPS.

De făcut:

1. `EK3_SRC2_*` scrise **explicit** în `config/nova_sitl.parm` și
   `config/nova_flight.parm` (nu lăsate pe implicit — §5.10), toate cinci
   termenele non-GNSS, fiecare cu comentariul lui.
2. Modul nou `nova/ekf_source.py`, după tiparul lui `nova/fence.py`:
   comută la handover, restaurează la handback **și la abort**, confirmă din
   `COMMAND_ACK`, și refuză să comute dacă citirea înapoi arată GNSS în setul
   țintă.
3. Cablat în `tools/nova_sim.py`; `nova/state_machine.py` neatins.
4. Campanie comparativă: aceeași sămânță, cu și fără comutare. Ce măsurăm:
   eroarea finală, deriva, și dacă secvența se mai încheie.

**Ce poate ieși prost, și de ce testul contează.** Fără GPS și fără flux
optic, poziția orizontală derivă pe inerțial pur. Dar precision landing e o
măsurătoare **relativă**: dacă EKF-ul derivă, ținta derivă odată cu el, iar
corecția relativă rămâne validă la ordinul întâi. Ce nu se anulează e
estimarea de viteză, care amortizează bucla. Dacă derivă prea mult, răspunsul
e hardware — flux optic — și **asta are termen de livrare**. De aceea J1 e
primul.

---

## J2 ✔ REZOLVAT — 8.3.3: imaginea de touchdown

**Rezultat:** `nova/frame_ring.py` (ring unic, folosit de serviciu, bord și
sim) + `nova/scoring.py` (alege cadrul după timestamp-ul capturii, scrie
`.png` + `.json` cu `time_boot_ms`). Se predau două imagini: scoring și
contact. Campania raportează `imagini_8_3_3` în CSV. Detalii în §5.55.

Rămâne de verificat pe hardware că ringul încape în RAM-ul Pi-ului fără să
fure din bugetul detecției.

Contextul inițial, păstrat pentru trasabilitate:

`FrameRing` există, dar numai în `tools/nova_service.py`. `tools/nova_pi.py`
nu are niciun handler pentru `scoring_capture`, deci secvența se încheie „cu
succes" și nu predă juriului nimic.

De făcut: ring buffer în aplicația de bord, cadrul scos după timestamp-ul
**capturii** (nu al deciziei), salvat cu numele și metadatele cerute de
6.2.1.30, alături de `.bin`. Plus un test care verifică faptul că imaginea
predată corespunde momentului capturii, nu celui în care s-a scris fișierul.

---

## J3 — praguri care au acum o măsurătoare în spate

Trei schimbări de o linie, fiecare cu cifra ei. Merg împreună pentru că
interacționează.

| ce | de la | la | de ce |
|---|---|---|---|
| `SequenceConfig.scoring_px` | 980 | ~800 | 980 e de neatins peste ~18° de yaw: cutia de încadrare a markerului iese din cadru înainte (§5.51) |
| `SequenceConfig.no_lateral_alt_m` | 0.40 | de rulat separat | 0.60 a fost necesar cu pragul de 980; cu 800 captura e la 0.65 m, deci s-ar putea să nu mai fie (§5.49) |
| prag de înclinare al camerei | nu există | funcție de altitudine | `MAX_TILT_DEG = 30°` e peste limita camerei în tot regimul care contează (§5.48, element 30) |

**Al doilea rând cere o campanie cu o singură variabilă schimbată.** Cele
două au fost schimbate odată; rezultatul e bun dar nu se poate atribui.

---

## J4 — poarta: praguri care descriu ce se poate recupera

Două lucruri, ambele în `nova/handover.py`:

- **Raza de 6.5 m nu e recuperabilă la nicio altitudine permisă** (§5.48).
  Bugetul de înclinare dă 4.5 m la 12 m și 1.7 m la 5 m. Pragul ar trebui să
  fie funcție de altitudine, iar refuzul să spună „prea departe pentru
  altitudinea asta, urcă" — pe teren, diferența e între două secunde și o
  încercare pierdută.
- **Plafonul de 12 m e mai conservator decât măsurătoarea** (§5.42, element
  31): detecția merge până la 17 m. Ridicarea dă pilotului libertate, dar
  costă timp de coborâre contra celor 40 de puncte de timp. De decis cu
  cifre, nu din instinct.

---

## J5 — `LANDING_TARGET` și `DISTANCE_SENSOR` se emit și în `IDLE`

§8 spune despre `RACE_MONITOR`: *„detector activ, ZERO comenzi"*. Verificat
direct: 5 detecții în `IDLE` → 5 `LANDING_TARGET` și 5 `DISTANCE_SENSOR`.

`LANDING_TARGET` e inert (PLND e 0), dar `DISTANCE_SENSOR` nu: FC-ul are
telemetru în tot zborul pilotului, iar §5.9 arată măsurat ce schimbă asta.
Filtru pe **listă pozitivă** de faze (§5.25), nu pe negație.

Rândul din Compliance Matrix pentru 15.2.3 nu e susținut de cod până atunci.

---

## J6 — distanța de frânare pe fiecare treaptă

Deblochează `PROFIL_RAPID`, adică o coborâre vizibil mai rapidă — direct în
cele 40 de puncte de timp. Cunoscut: 1.00 m la 0.5 m/s, cu 0.48 s până la mod
confirmat. De măsurat: 0.8 și 1.5 m/s, în bucla care acum funcționează.

Rezultatul probabil impune limitarea vitezei în funcție de altitudine — și
exact relația aia trebuie să apară în Safety Case, nu o singură cifră.

---

## J7 — coada erorii unghiulare, nelămurită

Pe `DESCEND_TRACK`: p50 **0.42°** (sub pragul I4 de 0.5°) dar p95 **2.26°**.
Două ipoteze verificate și rezultatul lor:

| ipoteză | rezultat |
|---|---|
| decalaj de timp adevăr / cadru | **infirmată** — p95 e cel mai mare la rata de înclinare cea mai mică |
| efect de înclinare | corelație reală: p50 0.31° la <1°, 0.69° la 3–10° |

Deci înclinarea contează, dar nu prin schimbarea ei în timp. Nu se scrie o
cauză pentru că se potrivesc cifrele — greșeala din §5.45, plătită o dată.

---

## Ce nu se poate face aici, oricât de bine merge simularea

| | de ce |
|---|---|
| **E2** — validare offline a detectorului, marker tipărit, ruletă, 3 condiții de lumină | `autonomy_enabled` rămâne **false** până atunci. Tot ce s-a măsurat în Gazebo e pregătire, nu înlocuitor |
| latența pe Pi 4 (criteriu p99 < 150 ms) | desktopul nu e Pi; §5.24 spune de două ori cât costă presupunerea |
| `STICK_DEADBAND_PWM` pe emițătorul de concurs | cifra de pe gamepad nu se transferă |
| nimic în conul camerei (§5.46) | zona liniștită de 60 mm nu iartă umbre, murdărie, sau o piesă care intră puțin în cadru |
| `FLTMODE_CH` + `FLTMODE1..6` | lipsesc deliberat din `nova_flight.parm` |

---

## Ordinea propusă

**J1 → J2 → J3 → J5 → J4 → J6 → J7**, cu J0 la început.

Motivul ordinii: J1 poate cere hardware (termen de livrare); J2 e singurul
livrabil lipsă din 8.3.3; J3 și J5 sunt ieftine și fac configurația să fie
cea de cursă; J4 cere o decizie de compromis timp/libertate; J6 și J7 sunt
optimizare și lămurire, nu blocante.
