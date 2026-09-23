# Zborul de concurs — segmentul autonom

Pentru **echipa de la pistă**: pilot, operator Pi, responsabil siguranță.
Se citește înainte de slot, nu în slot.

`CHECKLIST_TEREN.md` acoperă ziua: ce se încarcă, ce se ia, preflight,
între curse, după sesiune. **Aici e doar segmentul autonom** — cele ~30 s
care decid 20 de puncte și bonusul de timp.

---

## Înainte de orice: două lucruri NU sunt în codul care zboară

Verificat în cod, nu presupus:

| cerință | modul | în simulare | **pe vehicul** |
|---|---|---|---|
| **15.2.5** — niciun estimator nu primește GNSS din momentul activării | `nova/ekf_source.py` | cablat, măsurat pe 10 rulări (§5.54) | **NU e cablat** |
| **15.2.4** — cerc de incluziune 10 m în firmware | `nova/fence.py` | — | **NU e cablat** (nicăieri) |

`tools/nova_pi.py` nu instanțiază niciunul. Consecințele sunt diferite și
amândouă contează:

- **15.2.5 e o cerință de regulament, nu o măsură de siguranță.** Cu codul
  actual, EKF-ul primește GNSS pe tot segmentul autonom. Rândul din
  Compliance Matrix ar afirma ceva ce vehiculul nu face. Mecanismul există
  și e demonstrat în sim — **lipsește o singură legătură în aplicația de
  bord**, nu hardware (element deschis 25 → rezolvat; ăsta e separat).
- **15.2.4**: rămân `_mon_radius` și `_mon_ceiling` din supervizor. Ele
  chiar funcționează, dar rulează pe Pi — deci dacă Pi-ul moare, nu mai
  există nicio limită de rază. Stratul din firmware exact asta acoperea
  (element deschis 35).

> **Nu zburați segmentul autonom în concurs până nu sunt cablate.** Pentru
> un zbor de test sunt acceptabile; pentru o încercare punctată, prima e o
> neconformitate declarată.

Restul listei, înainte de slot:

- [ ] **E2 trecut**, raport scris → abia apoi `autonomy_enabled = true`
- [ ] Calibrarea camerei cu **rms sub 0.5** (cea din repo e 0.829)
- [ ] `STICK_DEADBAND_PWM` măsurat pe emițătorul de concurs, nu cel implicit
- [ ] `FLTMODE_CH` + `FLTMODE1..6` setate; o poziție = abort
- [ ] `tools/check_params.py` → **cod 0** pe FC-ul real
- [ ] `git status` curat pe Pi — altfel nu se poate spune ce cod a zburat

---

## Ce se punctează, și ce anulează totul

| | |
|---|---|
| Provocarea autonomă | **20 p**, binar, 10 per cursă |
| Bonus de timp pentru secvență completă | **−15 s** |
| Orice intervenție manuală **în segment** | tur manual, **zero** pe autonomie |

Principiul **all-or-nothing** e cel care schimbă deciziile la manșă: un
abort nu costă „puțin". Costă tot ce ar fi adus segmentul. Deci:

- **un handover refuzat nu costă nimic** — câteva secunde, reiei
- **un abort la mijloc costă 10 puncte** și bonusul
- **o aterizare lângă marker, fără secvență completă, costă la fel**

Din slotul de 15 minute se ia turul cu cel mai bun rezultat combinat, și
**nu se amestecă tururi**. Deci o cursă cu segment autonom reușit și timp
mediocru poate bate una rapidă și manuală — sau invers. Decideți înainte
de slot câte încercări autonome faceți și în ce ordine.

---

## Secvența, și ce vede fiecare

Operatorul pornește **înainte de decolare** și lasă pornit:

```bash
ssh pi@nova
cd ~/nova-zdc && tools/race_mode.py
```

Citește doar banda de sus:

| bandă | ce faci |
|---|---|
| `GATA - AUTONOMIE ARMATA` | poți zbura segmentul |
| `GATA - MONITOR (autonomie OPRITA, E0)` | handover-ul **va** fi refuzat |
| `ATENTIE - ...` | poți zbura, dar află de ce înainte |
| `NU ZBURA - ...` | nu decolezi |

Pilotul, în segment:

| pas | ce face pilotul | ce trebuie să vadă |
|---|---|---|
| 1 | zboară traseul până la punctul de handover | — |
| 2 | stabilizează **5–12 m** deasupra markerului, **sub ~2 m lateral** | LOITER |
| 3 | manșe libere, în neutru, **~1 s** | fereastra de așezare |
| 4 | ridică **AUX 7** | `ACCEPT` sau `REJECT` cu motiv |
| 5 | **mâna pe comutatorul de mod**, nu pe ecran | coborârea pornește |
| 6 | nu atinge nimic ~30 s | stările curg |
| 7 | preia la `HANDBACK` | vehiculul e staționar la ≥5 m |

> **Cei 6.5 m ai porții nu sunt utilizabili.** Poarta acceptă 6.5 m lateral
> la orice altitudine din fereastră, dar geometria nu: bugetul de înclinare
> dă ~4.0 m la 12 m și ~1.5 m la 5 m (§5.48, cu VFOV-ul măsurat). Un
> handover acceptat la 6 m lateral și 6 m altitudine nu e o încercare grea,
> e una care **nu poate reuși** — și costă cele 10 puncte la fel, doar mai
> târziu și cu vehiculul jos. **Apropie-te.**

Stările, în ordine. Dacă una lipsește, segmentul nu e complet:

```
IDLE → HANDOVER_CHECK → ACQUIRE → DESCEND_TRACK
     → SCORING_CAPTURE      încadrare 0.62 — imaginea pentru 8.3.3
     → FINAL_DESCENT        încadrare 0.72 — coborâre verticală
     → TOUCHDOWN_CONFIRM    contact + ≥1 s pe sol
     → ASCENT               ≥5 m deasupra markerului
     → HANDBACK
```

---

## Abort — cine decide și cum

Trei căi, în ordinea încrederii. **Prima e singura care funcționează dacă
Pi-ul e mort:**

| | cale | latență | depinde de Pi |
|---|---|---|---|
| 1 | **comutatorul de mod** (`FLTMODE_CH`) | imediat | **nu** |
| 2 | manșele → companion comandă LOITER | 150 ms (SITL) | da |
| 3 | Safety Supervisor → BRAKE / RTL | 0.1–0.5 s | da |

**Când se abortează, fără discuție:**

- vehiculul se mișcă altfel decât „coboară drept spre marker"
- oscilație laterală care crește
- orice zgomot sau comportament care nu e al unei coborâri normale
- ai pierdut orientarea vehiculului

**Când NU se abortează:**

- pentru că pare încet — coborârea e 0.5 m/s prin proiect
- pentru că s-a oprit scurt din coborât — supervizorul a frânat, e
  comportamentul cerut de 15.2.9
- pentru că ecranul arată ceva ce nu înțelegi — ecranul nu zboară vehiculul

> **Riscul rezidual, de știut înainte:** garanția „planează în loc să
> aterizeze" la pierderea detecției **nu se aplică sub ~1.2 m AGL**.
> Frânarea consumă 1.00 m măsurat la 0.5 m/s. Sub pragul ăsta, pierderea
> detecției duce la contact, nu la hover. Acceptat: la acea înălțime
> eroarea laterală a fost sub 3 cm în toate rulările, iar amprenta camerei
> e integral pe marker pentru orice eroare sub ~19 cm.

---

## Între încercări, în același slot

Segmentul se poate relua **fără dezarmare**: `PLND_ENABLED` urmează
`1 → 0 → 1`, verificat în SITL pe două secvențe consecutive.

- [ ] AUX 7 **jos** înainte de o cerere nouă — poarta cere frontul
      crescător, nu starea
- [ ] dacă a fost `REJECT`, rămâne afișat până lași AUX jos
- [ ] dacă a fost override, companion-ul rămâne **pasiv definitiv** pentru
      încercarea aia; reia de la handover nou

---

## Evidența — se adună în slot, nu după

Fără ea, o încercare reușită nu se poate demonstra.

```bash
tools/collect_session.py --nota "cursa 2, segment autonom complet"
```

Adună `.bin`-ul de la FC, logurile, cadrele, parametrii **citiți de pe
vehicul** și calibrarea folosită.

- [ ] cele **două imagini** din `data/scoring/` — scoring și touchdown,
      fiecare cu `.json`-ul ei (fără `time_boot_ms` nu se aliniază cu
      `.bin` și nu e evidență, 6.2.1.30)
- [ ] `.bin` **complet** — verifică să nu scrie „octeți lipsă"
- [ ] ieșirea nu spune „arborele git e MURDAR"

---

## Dacă poarta refuză

Motivul e pe ecran și e specific. Nu ghici:

| motiv | ce faci |
|---|---|
| `altitudine in afara ferestrei` | urcă sau coboară în 5–12 m |
| `distanta prea mare` | apropie-te de marker |
| `marker nedetectat` | ești prea departe lateral pentru altitudinea asta, sau markerul e în umbră |
| `mansa in afara neutrului` | lasă manșele, așteaptă ~1 s, reia |
| `autonomy_enabled = false` | E0 e închis — nu se rezolvă la pistă |

Un refuz costă câteva secunde. Reia; nu forța.
