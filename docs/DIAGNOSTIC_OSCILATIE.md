# Oscilația de pendul în coborâre — procedură de diagnostic

Simptomul: în `DESCEND_TRACK`, vehiculul nu converge lin pe marker, ci
oscilează lateral cu amplitudine aproximativ constantă sau crescătoare, de
obicei cu perioadă de 1–3 s. Pe log arată ca un `LANDING_TARGET` care se
plimbă simetric în jurul zeroului.

> **Regula care face procedura utilă: un singur parametru pe rulare.** Trei
> parametri schimbați odată și o oscilație dispărută nu spun care dintre ei
> a rezolvat-o — iar la următoarea reglare nu se știe ce se poate da înapoi.
> Costul e trei rulări în loc de una. Câștigul e că rezultatul intră în
> Safety Case ca relație, nu ca setare.

## Înainte de orice reglaj: e oscilație de control?

Trei cauze produc aceeași imagine și doar una se repară din câștiguri.

| dacă… | atunci nu e de reglat |
|---|---|
| eroarea unghiulară raportată de detector oscilează **și** când vehiculul stă pe loc | e detecția, nu controlul — vezi `range_p95` / `angle_p95` din `nova_sim.py` |
| oscilația apare doar sub ~1 m | e §5.2 (markerul iese din cadru) plus autoritatea de 2–4 cm; se rezolvă prin coborâre verticală, nu prin câștiguri |
| oscilația e perpendiculară pe direcția erorii, în elipsă | e convenția de axe din §5.1, nu reglajul |

Ultima e cea care păcălește: o corecție perpendiculară pe eroare arată
exact ca un sistem prea agresiv, iar reducerea câștigurilor o face mai
lentă fără să o vindece.

## Ordinea, și de ce e asta

### Pasul 1 — `PLND_LAG`, din latența măsurată

`PLND_LAG` spune filtrului cât de veche e observația. Setat prea mic,
corecția se aplică pe o poziție pe care vehiculul a depășit-o deja — ceea
ce **este** un pendul, prin construcție, indiferent de câștiguri.

E primul pentru că nu e reglaj, e o **măsurătoare**: valoarea corectă e
latența captură → publicare a detectorului, pe care o raportează
`PiDetector.stats()` ca `latency_p50_ms`. `nova/authority.py` o setează
singur (`Spec('PLND_LAG', 'lag', ...)`, limitată la 0.02–0.25 s).

Deci: citește latența, pune-o, repetă rularea. Dacă oscilația scade brusc,
cauza era asta și pașii 2–3 nu mai sunt necesari.

Pe Pi 4 latența nu e cea de pe desktop (4/6 ms) — elementul deschis 16 din
§7. Până la măsurătoarea de pe vehicul, valoarea pusă aici e o presupunere,
iar o presupunere prea mică e exact modul de eșec descris mai sus.

### Pasul 2 — `PSC_NE_POS_P` redus cu 30%

Câștigul P al poziției orizontale. Prea mare, sistemul depășește ținta la
fiecare corecție. 30% e o treaptă suficient de mare cât să se vadă și
suficient de mică să nu strice altceva; `nova/authority.py` o exprimă ca
factor pe banda de sus (`0.70`), unde estimarea e cea mai zgomotoasă.

Numele real pe 4.8 e `PSC_NE_POS_P`, nu `PSC_POSXY_P` (§5.30) — și, ca
întotdeauna, se citește înapoi după scriere (§5.10).

### Pasul 3 — `PSC_NE_VEL_D` crescut

Termenul D al vitezei orizontale amortizează. E ultimul deliberat: D
amplifică zgomotul de estimare, iar la altitudine estimarea noastră vine
dintr-un marker de 30–45 px. Crescut înainte de a fi încercat pasul 2,
poate transforma un pendul lent într-o vibrație rapidă — care arată mai
rău și e mai greu de atribuit.

Creștere în trepte de 20%, verificând de fiecare dată dacă apare zgomot pe
comanda de atitudine.

## Ce nu se atinge

`ANGLE_MAX` și `PSC_ANGLE_MAX` sunt în `FORBIDDEN` (`nova/authority.py`).
Unghiul maxim de înclinare e plafonul de autoritate al vehiculului, nu
reglaj fin — și interacționează cu `MAX_TILT_DEG` din supervizor, care
tratează 30° ca defecțiune. Ridicat ca să „meargă mai bine", ar putea duce
vehiculul exact în pragul la care supervizorul comandă BRAKE.

## Profilul de autoritate pe altitudine

Reglajul nu e o singură valoare: autoritatea potrivită la 10 m e greșită la
0.5 m, pentru că `marker_px` crește de la ~45 la ~900 (§5.23) și odată cu
el calitatea estimării.

| bandă | AGL | viteză (implicit / rapid) | ce se schimbă |
|---|---|---|---|
| sus | > 8 m | 0.5 / **1.5** m/s | `PSC_NE_POS_P` ×0.70, `PSC_NE_VEL_D` ×0.60 |
| mijloc | 8 – 3 m | 0.5 / **0.8** m/s | nominal |
| jos | 3 – 0.5 m | 0.35 / **0.3** m/s | `WP_ACC` ×0.70 |
| contact | < 0.5 m | 0.35 / **0.2** m/s | `WP_ACC` la podea |

Cele două profiluri diferă **numai pe coloana de viteză** — benzile și
câștigurile sunt identice, iar un test o verifică. Altfel o oscilație
apărută la 1.5 m/s nu s-ar mai putea atribui vitezei.

Banda de contact nu e cea care oprește corecțiile laterale: aia e
`FINAL_DESCENT` din mașina de stări (§8, sub 0.4 m). Aici doar se scoate
autoritatea care ar rămâne disponibilă.

## Condiția care blochează profilul rapid

`PROFIL_RAPID` cere `allow_fast_descent=True` și **nu trebuie deblocat**
până nu există distanța de frânare măsurată la fiecare treaptă de viteză
(§6/15.2.9, element deschis 11).

Cifra cunoscută e 1.00 m la 0.5 m/s, cu 0.48 s până la mod confirmat de FC.
Reacția singură e liniară în viteză:

| viteză | doar reacția (0.48 s × v) | total estimat |
|---|---|---|
| 0.5 m/s | 0.24 m | **1.00 m (măsurat)** |
| 0.8 m/s | 0.38 m | ~1.5 m (estimat) |
| 1.5 m/s | 0.72 m | ~2.5 m (estimat) |

Coloana din dreapta, în afara primului rând, e **extrapolare**, nu
măsurătoare. Consecința operațională: pragul sub care garanția „planează în
loc să aterizeze" nu mai ține urcă odată cu viteza — la 1.5 m/s, o detecție
pierdută la 3 m ar putea duce tot la contact. Rezultatul probabil al
măsurătorii e o limitare a vitezei în funcție de altitudine, ceea ce e
exact relația care trebuie să apară în Safety Case.

## Cum se rulează un pas

```bash
# fără modulare, referința
~/nova-sim-venv/bin/python tools/nova_sim.py --calib config/camera_sim.yaml \
    --provisional --csv ref.csv --json ref.json

# cu modulare, profilul implicit
~/nova-sim-venv/bin/python tools/nova_sim.py --calib config/camera_sim.yaml \
    --provisional --authority --csv p1.csv --json p1.json
```

CSV-ul are `angle_x_deg` / `angle_y_deg` pe fiecare cadru — acolo se vede
oscilația și i se măsoară perioada. Comparația se face pe **p50 și p95**,
nu pe medie: un pendul care se stinge lent are o medie liniștitoare.

Pentru o campanie cu condiții variate (vânt, lumină, poziția markerului),
`tools/batch_sim.py`. Vântul contează aici mai mult decât oriunde: o
oscilație care apare doar la `SIM_WIND_TURB` mare e altă problemă decât una
care apare pe vreme calmă.
