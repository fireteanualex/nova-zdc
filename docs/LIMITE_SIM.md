# Ce NU poate spune Gazebo

Scris ca nimeni — noi inclusiv — să nu prezinte cifrele din simulare drept
validare de percepție. Dacă un rând din Compliance Matrix se sprijină pe un
număr obținut aici, rândul acela trebuie să spună „simulare", nu „măsurat".

> **Concluzia, dacă citești un singur lucru:** Gazebo validează **bucla de
> control cu pixeli reali**. Percepția se validează la **E2**, pe hardware.
> Cele două nu se substituie.

---

## Ce validează Gazebo, de fapt

Lucruri pe care detectorul sintetic nu le putea atinge, pentru că acolo
geometria era corectă prin construcție:

- **Lanțul complet**: randare → `detectMarkers` → `solvePnP` →
  `LANDING_TARGET` → controler → mișcare → randare. O eroare de convenție de
  axe, de scară sau de ordine a colțurilor se vede aici, nu în sinteză.
- **Comportamentul buclei închise**: oscilații, întârzieri, interacțiunea
  dintre autoritatea de control și rata de detecție.
- **Pragurile exprimate în pixeli**: `SCORING_CAPTURE` la 980 px, limita de
  încadrare din §5.2 — toate se produc din geometrie reală de proiecție, cu
  distorsiunea aplicată.
- **Repetabilitatea**: aceeași sămânță, același rezultat. Pe teren nu ai asta.

---

## Ce NU modelează

### Rolling shutter

IMX708 e rolling shutter: liniile se citesc secvențial, pe ~10–30 ms. Gazebo
randează **cadre instantanee** — toate liniile la același timp de simulare.

Efectul lipsă e forfecarea: `v_lateral × T_readout`. La 0.5 m/s și 20 ms
readout, ~10 mm de deplasare între prima și ultima linie. Coborârea finală e
deliberat verticală și lentă tocmai ca să o elimine (§8), dar în
`DESCEND_TRACK`, cu vânt lateral, ea există și **nu apare aici**.

### Motion blur

Inexistent. Gazebo nu integrează pe timpul de expunere.

Pe vehicul, `ExposureTime` e fixat la 2 ms tocmai ca blur-ul să rămână sub
1 px pe tot profilul de coborâre (§5.15). Dar asta e un calcul, nu o
măsurătoare — iar dacă expunerea reală ajunge mai lungă (lumină slabă,
driver care limitează tăcut), blur-ul apare pe vehicul și **nu va apărea
niciodată aici**.

### Expunere și AGC

Randarea are iluminare perfectă și interval dinamic infinit. Nu există:

- supraexpunere pe marker alb în soare direct
- subexpunere în umbră
- **oscilația AGC** când markerul alb-negru intră în cadru — motivul pentru
  care `AeEnable` e `False` pe vehicul (§5.15). Aici nu ai cum verifica dacă
  decizia aceea a fost bună.

### Zgomot de senzor

Absent. Fără zgomot de citire, fără zgomot de fotoni, fără pixeli morți.

Consecința e subtilă și optimistă: `detectMarkers` binarizează adaptiv, iar
zgomotul mută pragul. Rata de detecție de aici e o **limită superioară**.

### Reflexia reală a hârtiei

`roughness` e o aproximare PBR a unei hârtii mate. `--roughness 0.3` arată
*că* există o strălucire, nu *cum* arată strălucirea reală a colii tipărite,
la unghiul de incidență al zilei respective. 15.4.7 semnalează reflexia ca
risc; aici o poți doar ilustra, nu cuantifica.

Nu modelează nici: hârtia ondulată, umbre proprii din cute, murdărie, umezeală.

### Latența reală a Pi 4

Se **injectează**, nu se măsoară. Ce injectăm vine dintr-o măsurătoare făcută
pe desktop (p50/p99 = 4/6 ms pe cadru întreg), iar Pi 4 e altă mașină, cu
alt cache, alt termic și throttling la 80 °C (§5.15). Criteriul E2 pentru
latență (`p99 < 150 ms`) **nu se poate verifica aici**.

### Camera reală, ca dispozitiv

Nu există: comutarea de mod de senzor (~0.3–0.5 s fără tracking, §5.15),
autofocus care se mișcă, controale limitate tăcut de driver (§5.10 aplicat
camerei), sau temperatura care schimbă intrinsecii.

---

## Ce e „real" doar pe jumătate

| element | cât de real |
|---|---|
| intrinsecii | **din calibrare**, verificați dus-întors prin `camera_info` (§5.33) — dar calibrarea curentă e provizorie, cu RMS 0.85 (§5.34) |
| distorsiunea | aplicată de ogre2, cu coeficienții noștri — dar modelul de distorsiune al randorului nu e identic cu cel al OpenCV |
| markerul | 480 mm zonă codată pe coală de 600 mm, geometrie exactă (§5.31) — dar textură perfectă, fără imperfecțiuni de tipar |
| fizica vehiculului | iris din `ardupilot_gazebo`, **nu** NOVA: altă masă, alt tensor de inerție, alt TWR (§7) |
| ArduPilot | **firmware real**, în SITL — asta chiar e validare |

Ultimul rând e cel mai important: controlerul de zbor nu e simulat, e același
cod care zboară. De aceea bucla de control se poate valida aici.

---

## Cum se citește o cifră obținută în Gazebo

Trei întrebări, în ordine:

1. **Depinde de aspectul imaginii?** Rată de detecție, eroare de localizare
   a colțurilor, comportament în soare — atunci cifra e o **limită
   superioară optimistă**, iar E2 o poate doar înrăutăți.
2. **Depinde de geometrie și control?** Eroare de aterizare, timp per stare,
   oscilație, altitudine de `SCORING_CAPTURE` — atunci cifra e utilizabilă,
   cu rezerva că vehiculul e iris, nu NOVA.
3. **Depinde de hardware?** Latență, FPS, temperatură — atunci **nu are
   valoare**; se măsoară pe Pi.

---

## Starea de acum: harness scris, nicio măsurătoare

Onest, pentru că altfel documentul ăsta ar deveni chiar lucrul împotriva
căruia e scris: **bucla închisă nu a rulat încă niciodată cap-coadă.**

| piesă | stare |
|---|---|
| `nova/sim_truth.py`, `tools/nova_sim.py` | scrise, testate offline |
| `tools/sim_fly_to.py`, `tools/batch_sim.py` | scrise, testate pe bucăți |
| secvența de procese (Gazebo → SITL → decolare → poartă → rulare) | **netestată** |
| orice cifră de eroare, rată sau latență din Gazebo | **nu există încă** |

Mediul în care au fost scrise nu poate ține un server Gazebo în viață
(`libEGL: failed to create dri2 screen`), deci prima rulare cap-coadă e
supravegheată, pe mașina cu GUI. Până atunci, tabelele de mai sus descriu
ce *va* măsura harness-ul, nu ce a măsurat.

Un lucru l-a arătat totuși deja, fără să ruleze: la 5 m altitudine, limita
de 6.5 m a porții e în afara conului camerei (§5.37). Geometria se verifică
pe hârtie; comportamentul nu.

## Ce trebuie măsurat pe hardware, oricum

Lista scurtă, pentru E2:

- rata de detecție în trei condiții de lumină, la distanțe măsurate cu ruleta
- eroarea de range față de adevărul cu ruleta (criteriu: sub 5%)
- latența captură → publicare pe Pi 4 (criteriu: p99 sub 150 ms)
- FPS efectiv și temperatura sub sarcină, cu `vcgencmd get_throttled`
- durata comutării de mod de senzor
- comportamentul markerului tipărit în soare direct, la mai multe unghiuri

Niciuna nu se poate substitui cu o rulare în Gazebo, oricât de fidelă.
