# NOVA — checklist de teren

Pentru cineva care **nu a scris codul**. Se urmează în ordine, nu se sare.

Dacă ceva pică, sari direct la **[Când ceva nu merge](#când-ceva-nu-merge)** —
e ultimul capitol și e cel care contează pe teren.

> **Regula care le acoperă pe toate:** dacă `preflight_check.py` nu întoarce
> `cod 0`, nu se zboară. Nu există „e doar o verificare sărită". O verificare
> sărită nu e o verificare trecută.

---

## 1. Înainte de plecare (acasă, cu timp)

### Se încarcă

- [ ] Bateriile de zbor, toate, la storage dacă zborul e peste >24 h
- [ ] Bateria emițătorului
- [ ] Powerbank pentru Pi la banc
- [ ] Laptop

### Se verifică, cu drona pe masă

```bash
ssh pi@nova
cd ~/nova-zdc && source ~/nova-venv/bin/activate
python3 tools/preflight_check.py          # trebuie să iasă cu 0
```

- [ ] `preflight_check.py` → **cod 0**, toate verzi
- [ ] `git status` curat pe Pi (altfel nu știm ce cod zboară — vezi §5 în
      raportul de sesiune)
- [ ] `config/nova.json`: `autonomy_enabled` are valoarea **intenționată**
      pentru ziua asta
- [ ] `config/camera_pi.yaml` există și e cel din calibrarea curentă
- [ ] Spațiu liber pe card: `df -h /` — **peste 2 GB**

### Se ia

- [ ] Marker ArUco printat (ID 26, 480 mm), curat, plan, fără cute
- [ ] Ținta de calibrare, dacă se recalibrează
- [ ] Ruletă (distanțele E2 se măsoară, nu se estimează)
- [ ] Cablu serial de rezervă + cablu camera de rezervă
- [ ] Șurubelnițe, elice de rezervă, bandă
- [ ] Cablu de rețea / hotspot pentru SSH la Pi

---

## 2. La sosire

### 2.1 Pornire

```bash
# 1. alimentează FC-ul și Pi-ul
# 2. conectează-te la Pi
ssh pi@nova
cd ~/nova-zdc && source ~/nova-venv/bin/activate
```

- [ ] Pi-ul pornește, SSH merge
- [ ] `ls /dev/serial0` — există

### 2.2 Preflight

```bash
python3 tools/preflight_check.py
```

- [ ] **Cod 0.** Dacă nu: capitolul de depanare, apoi reia.

Ce verifică și de ce contează fiecare:

| verificare | dacă pică |
|---|---|
| stivă | numpy/OpenCV din locul greșit — camera se va rupe |
| calibrare | detectorul nu pornește deloc (E1.2) |
| rezoluție | distanțele ies greșite, **tăcut** |
| cameră | rata de cadre, nu doar că se deschide |
| imagine | capac pe obiectiv / întuneric |
| controale | focus sau expunere **nu** s-au aplicat (§5.10) |
| mavlink | fără FC nu există nici telemetrie, nici log |
| parametri | un parametru neaplicat = afirmație falsă în Compliance Matrix |

### 2.3 Verificări la sol — **CU ELICELE DEMONTATE**

> Elicele se demontează. Nu „se ține drona bine". Toate testele de mai jos
> armează motoarele.

- [ ] **Kill switch.** Armează, ridică throttle-ul puțin, acționează
      kill switch-ul. Motoarele se opresc **instantaneu**.
- [ ] **Re-armarea e refuzată** cât timp kill switch-ul e activ.
- [ ] **Comutatorul de abort.** Verifică pe `FLTMODE_CH` că trecerea pe
      modul de abort e raportată de FC (vezi în MAVProxy / QGC că modul se
      schimbă). Comutarea o face FC-ul, **nu trece prin Pi** — de asta e
      stratul robust.
- [ ] **Override pe manșe.** Cu drona armată pe sol, în GUIDED: mișcă o manșă
      și verifică în logul companion-ului că apare `pilot_override` și că FC-ul
      trece în LOITER. Bugetul e 250 ms; măsurat în SITL, 150 ms.
- [ ] **Handover.** Acționează AUX 7. Ecranul trebuie să arate **ACCEPTAT**
      sau **REFUZAT cu motiv**. Cu `autonomy_enabled=false`, refuzul e
      așteptat și motivul trebuie să fie exact acela.
- [ ] Geofence încărcat: verifică prin citire înapoi, nu presupune.
      (Decizia 02.10.2026: **fără fence pe FC** — în cursă vehiculul zboară
      peste 30 m AGL; plafonul de 30 m există doar în secvența autonomă, în
      supervizorul companion-ului. Linia de mai sus se aplică doar dacă
      echipa reintroduce un fence.)

**X7+ fără safety switch:**

FC-ul e CUAV X7+, fără buton de siguranță hardware (detalii în
`pi/README.md`, pasul 4).

- [ ] Parametrii scriși cu `check_params.py --write --reboot`, apoi **citiți
      înapoi** după repornire (a doua rulare, fără `--write`):
      `BRD_SAFETY_DEFLT 0` (cere REBOOT al FC) și `BRD_SAFETYOPTION 0`
- [ ] **Armarea merge fără buton de siguranță** (nicio eroare „Hardware
      safety switch" / „Safety Switch"). Dacă pică pe baterie: pe X7
      implicitul e `BATT_MONITOR 8` (DroneCAN) — fără modul CAN de
      alimentare, monitorul se setează de mână
- [ ] `SERIAL2_BAUD` setat de mână pe FC și egal cu `--baud` de pe Pi
      (fișierele noastre nu îl scriu)
- [ ] **Comutatorul de RTL manual** (`RCn_OPTION 4` sau `FLTMODEn = 6`,
      setat de mână): acționează-l și vezi că FC-ul trece în RTL
- [ ] Funcția kill switch-ului e setată de mână (`RCn_OPTION`: **31**
      MOTOR_ESTOP oprește ieșirile dar vehiculul rămâne armat și dezarmează
      abia după `DISARM_DELAY`/2; **81** DISARM dezarmează imediat) — testul
      „Kill switch" de mai sus rămâne obligatoriu

**Plafonul de 30 m și fereastra de handover (config):**

- [ ] În liniile de pornire apar `[bord] plafon 15.2.4: 30 m deasupra home`
      și `[bord] fereastra de handover 4.0-20.0 m`; niciun `ATENTIE: plafon
      de test` (altfel `ceiling_agl_m` a rămas coborât din alt test)
- [ ] `preflight_check.py`, linia `secventa`, arată `plafon 30 m` și `poarta
      4.0-20.0 m`; orice `ATENTIE` e citit și explicat, nu ignorat
- [ ] **Test la sol / joasă altitudine al căii RTL** (o dată pe configurație
      nouă): coboară temporar `ceiling_agl_m` în `config/nova.json`,
      redeployează, zboară secvența jos; așteptat `STATUSTEXT` `NOVA PLAFON ..m
      DEPASIT: SRC1 -> RTL` și, în logul FC, `EV 85` (setul 1) **înainte** de
      schimbarea de mod în RTL. Apoi pune `ceiling_agl_m` înapoi la **30.0** și
      redeployează
- [ ] Linia de stare de pe OSD arată `h xx/30 m` când vehiculul e la mai puțin
      de 5 m de plafon (doar în secvența autonomă)

**Căutarea markerului prin rotire (YAW_SEARCH, `yaw_search` în `config/nova.json`):**

- [ ] În liniile de pornire apare `[bord] cautare prin rotire auto: doar sub
      h_cover ~15.6 m; 90 grd la 25 grd/s = 3.6 s, 4.4/5.0 s de la SRC2 ...`;
      la 90/20 pornirea e **refuzată** (nu se ocolește: se schimbă valorile)
- [ ] **Primul test, jos, cu `yaw_search: false`**: markerul scos din cadru →
      MARKER NEGASIT, rămâne LOITER (comportamentul de dinainte)
- [ ] Apoi `yaw_search: "auto"`, aceeași poziție: vehiculul se întoarce pe loc
      spre **stânga**, până la 90°, OSD `ROTIRE CAUTARE 90 grd la 25 grd/s`.
      Marker găsit: `MARKER GASIT la N grd` apoi ENGAGE. Marker lipsă: EXIT la
      cel mult 5.0 s, `MARKER NEGASIT dupa rotire`, LOITER. La `h ≥ h_cover`:
      `MARKER NEGASIT: fara rotire, h >= h_cover`
- [ ] **Mâna pe comutatorul de mod**; AUX jos / comutatorul oprește rotirea
      imediat, throttle la mijloc înainte
- [ ] În logul FC: **un singur** `EV 86` pe tot segmentul, niciun „is using
      GPS" între el și `COMPLETE` (acolo `EV 85`), niciun failsafe EKF
- [ ] Netestat pe camera reală: detecția în rotire și latența pe serial —
      notează la ce unghi a apărut prima detecție

### 2.4 Marker

- [ ] Așezat plan, fără cute, fără reflexii directe în soare
- [ ] Zona liniștită din jur curată (fără frunze, fără umbre tăioase pe margine)
- [ ] Verificat că se detectează: `python3 tools/nova_pi.py --camera-check`

---

## 3. Modul de cursă

```bash
python3 tools/race_mode.py
```

Rulează preflight-ul, **refuză să pornească** dacă ceva pică, apoi afișează un
singur ecran. Citește doar banda de sus:

| bandă | ce faci |
|---|---|
| `GATA - AUTONOMIE ARMATA` | poți zbura segmentul autonom |
| `GATA - MONITOR (autonomie OPRITA, E0)` | zbori manual; handover-ul **va** fi refuzat |
| `ATENTIE - ...` | poți zbura, dar află de ce înainte |
| `NU ZBURA - ...` | nu decolezi. Motivul e pe bandă. |

---

## 4. Între curse

- [ ] **Salvează ring bufferul** dacă a fost ceva interesant:
      ```bash
      kill -USR1 $(systemctl show -p MainPID --value nova-monitor)
      ```
- [ ] **Colectează evidența:**
      ```bash
      python3 tools/collect_session.py --nota "cursa 2, vant lateral"
      ```
- [ ] Baterie: tensiune la storage/zbor, temperatura pachetului (nu se
      reîncarcă un pachet cald)
- [ ] Elice: fisuri, joc în motoare
- [ ] Marker: nu s-a mișcat, nu s-a murdărit
- [ ] `df -h /` — dacă sub **500 MB**, mută datele pe laptop acum

---

## 5. După sesiune

- [ ] `python3 tools/collect_session.py --nota "final de sesiune"`
      — aduce `.bin`-ul de la FC, logurile, cadrele, parametrii citiți
      **de pe vehicul** și calibrarea folosită
- [ ] Verifică în ieșire că `.bin`-ul e **complet** (fără „octeți lipsă")
- [ ] Verifică că nu scrie „arborele git e MURDAR". Dacă scrie, notează ce
      era modificat — altfel nu se mai poate spune ce cod a zburat.
- [ ] Adu totul pe laptop:
      ```bash
      rsync -av pi@nova:~/nova-zdc/data/sessions/ ~/nova-zdc/data/sessions/
      ```
- [ ] Copie de siguranță în alt loc decât laptopul
- [ ] Bateriile la storage

---

## Când ceva nu merge

Simptom → cauză probabilă → ce faci. Referințele `§` sunt la `CLAUDE.md`.

### Pornire

| Simptom | Cauză probabilă | Fix |
|---|---|---|
| `Device or resource busy` pe `/dev/serial0` | `nova-monitor` ține portul (§5.27) | `sudo systemctl stop nova-monitor`, sau pornește cu `--stop-service` |
| `NU PORNESC: lipsește calibrarea` | `config/camera_pi.yaml` absent (E1.2) | rulează `tools/calibrate_camera.py`. Formula geometrică **nu** e substitut |
| `NU PORNESC: nu e o calibrare reală` | fișier scris de mână / focală geometrică | recalibrează; nu edita YAML-ul |
| `ImportError: _ARRAY_API not found` la `import picamera2` | numpy din venv peste cel din apt (§5.24) | `rm -rf ~/nova-venv && tools/setup_pi.sh` |
| `cv2.aruco` nu există | OpenCV fără contrib, sau < 4.7 | Trixie: `python3-opencv` din apt. Bookworm: `requirements-pi-bookworm.txt` |
| `setup_pi.sh` refuză distribuția | nu e Trixie/Bookworm | nu forța. Stiva picamera2 diferă real între ele |

### Cameră

| Simptom | Cauză probabilă | Fix |
|---|---|---|
| preflight: „cadre practic uniforme" | **capac pe obiectiv**, sau întuneric | scoate capacul. Serios — trece orice test de „camera se deschide" |
| preflight: FPS sub prag | throttling termic, sau altceva mănâncă CPU | `vcgencmd get_throttled`; răcește; oprește ce mai rulează |
| preflight: „control neaplicat" | driverul a limitat tăcut focus/expunere (§5.10) | verifică `LensPosition` cerut vs aplicat; nu ignora |
| Detecție 0% dar camera merge | focus (PDAF a căutat), marker prea departe, ID greșit | verifică `--camera-check`; markerul e **ID 26** |
| Detecția moare sub ~0.38 m | **normal, prin construcție** (§5.2) | nu e defect. Coborârea finală e deliberat oarbă |
| Previzualizarea nu apare | fără sesiune grafică prin SSH (§5.28) | `ssh -X`, sau rulează din VNC |
| Fereastra nu devine fullscreen | `WINDOW_AUTOSIZE` ignoră cererea (§5.28) | folosește uneltele noastre; ele creează `WINDOW_NORMAL` |
| Previzualizarea sacadează | VNC, nu detecția | `--preview-scale 0.5`. Detecția rulează oricum pe cadrul plin |

### Legătură și FC

| Simptom | Cauză probabilă | Fix |
|---|---|---|
| Ecranul: `LEGATURA FC CAZUTA` | cablu serial, FC nealimentat, baud greșit | verifică cablul. Reconectarea e automată (1/2/4/8 s), nu reporni procesul |
| `link_down` repetat în log | cablu care se mișcă / conector slab | fixează mecanic cablul. E cauza #1 pe teren |
| Supervizorul comandă BRAKE cu `link_age` | `LANDING_TARGET` nu ajungea la FC (H1) | **corect**. Verifică legătura înainte de următoarea încercare |
| `Arm: Rangefinder 1: No Data` | `ARMING_SKIPCHK` neaplicat (§5.13) | `tools/check_params.py`; un fișier încărcat ≠ parametru aplicat |
| Un parametru „nu se aplică" | numele nu există pe acest firmware (§5.4, §5.10) | citește-l înapoi. `WP_RFND_USE`, **nu** `WPNAV_RFND_USE` |
| `NAV_TAKEOFF result=4` | gardul „can't takeoff downwards" (§5.7, §5.9) | `WP_RFND_USE 0`; nu trimite range fals |
| RTL aterizează pe marker | `PLND_ENABLED` a rămas 1 (§5.8) | companion-ul îl stinge; verifică prin citire înapoi |

### Handover și segment autonom

| Simptom | Cauză probabilă | Fix |
|---|---|---|
| `HANDOVER REFUZAT: autonomie dezactivată` | `autonomy_enabled=false` (E0) | **așteptat** până trece E2. Se schimbă cu un commit, nu din linia de comandă |
| `REFUZAT: canalul 3 la ... PWM de trim` | throttle-ul nu se auto-centrează (§8) | ține throttle-ul **nemișcat** în fereastra de așezare; nu la trim |
| `REFUZAT: altitudine ...` | în afara 4–20 m (`handover_alt_min` / `handover_alt_max` din `config/nova.json`; 4 m e o abatere deliberată de la 5–20 m din regulament) | coboară sub 20 m / urcă peste 4 m; peste ~12 m markerul are prea puțini pixeli și poarta poate refuza |
| `REFUZAT: marker nedetectat` | markerul nu e văzut în ultimele 0.3 s | apropie-te; verifică lumina |
| Handover-ul nu pornește deloc | se cere pe **frontul crescător** al AUX (§8) | lasă comutatorul jos, apoi sus |
| Secvența se oprește singură | supervizorul a comandat ceva | citește logul: `detection_age`, `link_age`, `geofence_radius`, `tilt` |
| Drona coboară și nu oprește la pierderea detecției | sub ~1.2 m garanția nu se aplică (§6) | risc **cunoscut și acceptat**; nu e defect |

### Date și evidență

| Simptom | Cauză probabilă | Fix |
|---|---|---|
| `.bin` cu „octeți lipsă" | descărcare incompletă pe serial | reia: `collect_session.py --log-id <N>`. Un `.bin` cu găuri nu e dovadă |
| „arborele git e MURDAR" | cod necomis pe Pi | notează ce e modificat; commit înainte de următoarea sesiune |
| Cardul se umple | cadre brute din E2 | mută pe laptop; `run_e2.py` scrie toate cadrele, deliberat |
| O stație E2 iese cu detecție mică | focus/expunere/marker | **păstrează cadrele** — sunt datele cele mai valoroase din set |

### Ultima soluție

Dacă nimic nu se potrivește:

1. `journalctl -u nova-monitor -n 100 --no-pager`
2. `tail -100 ~/nova-zdc/logs/nova-monitor.log`
3. `python3 tools/preflight_check.py --json > /tmp/pf.json`
4. `python3 tools/collect_session.py --nota "incident"` — adună tot
5. **Nu zbura ca să vezi dacă mai apare.**
