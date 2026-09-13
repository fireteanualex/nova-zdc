# Raport — runda 6: pregătire operațională pe Pi

Stare: **H0 ✅ · H1 ✅ · H2 ✅ · H3 ✅ · H4 ✅ · H5 ✅**

`autonomy_enabled` a rămas `false`. Nimic din runda asta nu deschide o cale
către zbor autonom; H4 chiar adaugă o barieră în plus (preflight obligatoriu).

> **Dacă citești un singur lucru:** presupunerea din runda 5 despre mediul de
> pe vehicul era greșită, iar garda de platformă pe care o scrisesem
> **respingea distribuția reală**. Codul era corect; mediul presupus, nu.
> Asta e a doua oară în două runde când greșeala nu e în logică, ci în ce am
> crezut despre lumea din jurul ei (§5.24).

---

## H0 — stiva reală

| | presupus în runda 5 | real |
|---|---|---|
| Distribuție | Bookworm | **Trixie** |
| Python | 3.11 | **3.13.5** |
| numpy | 1.24.2 (apt) | **2.2.4 (apt)** |
| OpenCV | 4.10.0.84 (pip) | **4.10.0 (apt)** |

**Garda de platformă acceptă acum `trixie` și `bookworm`**, cu mesaj distinct
pentru fiecare; `bullseye` și orice necunoscut rămân refuzate, dar refuzul
pentru o distribuție *mai nouă* conține acum regula de urmat, nu doar un
„nu".

**Requirements pe două căi**, și criteriul nu e distribuția în sine, ci
versiunea de OpenCV din apt:

| distribuție | `python3-opencv` din apt | ce facem |
|---|---|---|
| Trixie | **4.10.0** | apt. Construit împotriva numpy-ului de sistem. |
| Bookworm | 4.6.0 | **pip**, `requirements-pi-bookworm.txt` |

Pragul e **4.7.0**, unde a apărut `cv2.aruco.ArucoDetector`. Pe Bookworm
versiunea pip nu poate fi 5.x, pentru că ar cere `numpy>=2` peste numpy 1.24
din apt și ar rupe picamera2 — motivul original din runda 5, încă valabil
**acolo**.

Corecție la §5.24: pe Trixie constrângerea aceea **nu mai există** (apt dă
numpy 2.2.4, care satisface `numpy>=2`). Rămânem pe apt oricum, dar diferența
de versiune față de desktop e acum o **alegere**, nu o imposibilitate. Merită
spus, pentru că versiunea veche a §5.24 suna mai categoric decât e cazul.

### Verificat, nu presupus

Venv de paritate reconstruit pentru **numpy 2.2.4 + OpenCV 4.10.0.84**:

| | Bookworm-sim (runda 5) | Trixie-sim (acum) |
|---|---|---|
| detecții false `DICT_4X4_50` pe ținta ChArUco | 5 / 36 (ID 48) | **5 / 36 (ID 48)** |
| paritate colțuri vs OpenCV 5.0 | 0.0225 px | **0.0225 px** |
| paritate distanță | 0.0059% | **0.0059%** |

Identice bit cu bit. Versiunea de numpy nu schimbă nimic în ieșirea
detectorului — de așteptat, numpy e doar containerul de array-uri. Ce
contează e versiunea de **OpenCV**, iar aceea e aceeași.

### Verificarea de stivă în preflight

Raportează distribuția, Python, numpy, OpenCV și **calea** fiecăruia. Calea e
partea utilă: arată dacă pip a pus o copie peste pachetul de sistem.

O decizie care nu e evidentă: **numpy din venv e `ESEC` doar pe Pi.** Pe
desktop e situația normală și nu există picamera2 pe care să o rupă. Dacă ar
fi roșu la fiecare rulare de dezvoltare, preflight-ul ar deveni zgomot — și
exact asta învață operatorul să treacă peste el.

---

## H1 — reconectare automată la FC

`nova/vehicle.py`:

- heartbeat lipsă peste **3 s** → închide și redeschide portul
- backoff **1, 2, 4, 8 s**, plafon 8 s
- fiecare încercare în log, cu `time_boot_ms` — ultima valoare dinainte de
  cădere, deci o **ancoră** spre `.bin`, nu o valoare curentă
- `check_link()` face **o singură încercare per apel** și se întoarce în
  microsecunde. Măsurat: cel mai lung apel **0.0 ms** pe 400 de cicluri cu
  portul mort. Bucla principală și supervizorul nu îngheață niciodată.
- `link_healthy`, `time_since_heartbeat()` expuse

Comenzile întorc acum `bool` și **nu trimit nimic** cu legătura căzută —
detectorul continuă, doar emisia se suspendă. `pump()` nu mai moare dacă
portul dispare sub el.

### Monitorul de link în supervizor

Sunt moduri de eșec diferite cu **aceeași consecință**: dacă `LANDING_TARGET`
nu ajunge la FC, vehiculul coboară în LAND fără corecție laterală — exact ca
la pierderea detecției. Deci același tratament:

| | prag | acțiune | faze |
|---|---|---|---|
| `detection_age` | 0.5 s | BRAKE | `ACQUIRE`, `DESCEND_TRACK`, `SCORING_CAPTURE` |
| `link_age` (nou) | **1.0 s** | BRAKE | **aceleași** |

Pragul e dublul celui de detecție (o cădere de link e mai rară și mai gravă
decât un cadru pierdut, și nu vrem declanșare pe o întârziere de planificare)
și o treime din timeout-ul de reconectare — frânarea nu are voie să aștepte
după un cablu.

Aceleași excepții de fază, și pentru același motiv: în `FINAL_DESCENT`
coborârea e deliberat oarbă și verticală, deci o legătură căzută acolo nu
schimbă traiectoria. Un test verifică explicit că cele două liste sunt
**identice**, nu doar asemănătoare.

### Trei bug-uri prinse de teste

1. **Backoff-ul pornea de la 2 s, nu de la 1 s** — indexul era citit după
   incrementarea contorului. Pierdeam exact prima secundă de reconectare,
   adică momentul cel mai probabil în care cablul e deja înapoi la loc.
2. **Fiecare reconectare emitea două evenimente `link_up`** — și din
   `_note_heartbeat`, și de la finalul lui `_try_reopen`. Un log de siguranță
   care numără greșit evenimentele e mai rău decât unul absent.
3. **Din runda 5:** `nova_service` apela `vehicle.alt()` pe o **proprietate**.
   Arunca `TypeError`, prins de un `except` larg, deci altitudinea lipsea
   tăcut din *toate* liniile de stare. Un `except Exception` în jurul unei
   citiri de telemetrie ascunde exact genul ăsta de greșeală.

---

## H2 — conflictul de port serial

`Device or resource busy` spune **ce**, nu **cine** și nici ce să faci.
`nova/serial_guard.py` întreabă înainte de a deschide portul:

```
[bord] EROARE: nova-monitor ocupa /dev/serial0.
        sudo systemctl stop nova-monitor
        (sau porneste cu --stop-service, care o face singur)
```

Trei decizii care nu sunt evidente:

- **Un diagnostic care nu se poate face nu blochează pornirea.** Fără
  systemd, sau fără drept de citire în `/proc`, funcțiile întorc „nu știu".
  Un proces al altui utilizator nu se vede fără root, deci lista goală **nu**
  înseamnă „portul e liber".
- **`--stop-service` cere confirmare** (`--yes` pentru scripturi). Oprirea
  unui serviciu are efect în afara procesului nostru.
- **Cod de ieșire 3**, distinct de 2 (calibrare lipsă) și 4 (preflight picat).
  Un script de teren poate deosebi „repar portul și reîncerc" de „nu pot
  zbura".

Pe `udpin:`/`tcp:` verificarea se sare — în SITL nu există port exclusiv.

---

## H3 — previzualizare, trei contexte

`nova/preview.py`:

| context | fereastră | scară |
|---|---|---|
| unelte de banc (`calibrate_camera --live`) | **fullscreen**, pornită | `--preview-scale` |
| prin VNC | aceeași, redimensionată | 0.5 implicit |
| vehicul (`nova_pi.py`, serviciu) | **oprită implicit** | — |

`WINDOW_NORMAL` înainte de `setWindowProperty`: cu `AUTOSIZE`, cererea de
fullscreen e ignorată **tăcut**. Ieșire pe `q`, `Q` **și Escape** — o
fereastră fullscreen nu are buton de închidere.

Redimensionarea e **numai pentru afișare**. Un test verifică explicit că
cadrul original rămâne 2304×1296 după ce s-a afișat la 1152×648.

Fără `DISPLAY`/`WAYLAND_DISPLAY` (adică SSH fără `-X`), `Preview` se stinge
singură cu motiv, în loc să arunce `Can't initialize GUI backend`.

`calibrate_camera --live` are acum overlay cu colțurile detectate și
acoperirea 3×3, desenat pe o **copie** — cadrul care intră în calibrare e
intact. Cadrele fără țintă se afișează și ele: altfel, exact când detecția nu
merge, ecranul îngheață și nu se mai vede ce filmează camera.

---

## H4 — mod de concurs

```bash
python3 tools/race_mode.py
```

1. verifică portul serial (H2)
2. rulează preflight-ul **integral**. Dacă ceva pică **sau e sărit** →
   cod 4, nu pornește, și listează exact ce.
3. pornește detectorul și poarta
4. un singur ecran

**Un bug de ordine, prins la recitire, nu de teste.** Prima variantă rula
preflight-ul **după** `build_pi_detector`, cu motivarea că „așa un eșec de
cameră se vede în preflight, nu ca excepție". Greșit: detectorul ține deja
camera, iar `check_camera` ar încerca să deschidă a doua oară același senzor.
Pe desktop nu se manifestă — nu există picamera2 — deci ar fi ajuns pe teren.
Același tipar și pe serial: `check_mavlink` nu închidea conexiunea, deci
`Vehicle.connect()` de după ar fi putut găsi portul ocupat **de noi înșine**.
Ambele reparate; un test verifică acum ordinea în sursă, fiindcă aici nu avem
cameră cu care să o reproducem.

`tools/race_mode.py` e un lansator subțire peste `nova_pi.py --race`,
deliberat: al doilea punct de intrare care și-ar construi singur piesele ar
reintroduce exact clasa de bug din §5.14 (supervizor inert pentru că
aplicația și testele aveau cablaje diferite). Un test citește sursa lui
`race_mode.py` și pică dacă apare `SafetySupervisor(`, `HandoverGate(`,
`LandingStateMachine(` sau `run_loop(`.

### Ecranul

```
        G A T A  -  M O N I T O R  ( a u t o n o m i e  O P R I T A ,  E 0 )

  STARE         R A C E _ M O N I T O R
  MARKER        148 px, acum 0.1 s
  LEGATURA FC   OK  0.3 s
  CAMERA        29 fps   detectie 97%
  TEMPERATURA   58 C
  CARD          12400 MB liberi
  AUTONOMIE     OPRITA (E0)
```

Proiectat invers față de un dashboard normal: sub 10 linii, trei culori
binare, verdictul scris **cu litere** (nu doar colorat), fiecare cifră cu
unitate, iar ce lipsește e `-`, nu `0`.

Verdictul raportează **cel mai grav** lucru, nu primul găsit — cine vede
`GATA` trebuie să poată să nu mai citească restul. Testat pe 10 combinații.

Handover-ul se citește **din poartă** la fiecare redesenare, nu se ține
într-o copie actualizată prin callback: un ecran cu propria copie a stării
poate rămâne în urmă, iar asta n-are voie să se întâmple cu un refuz.

Două probleme de lizibilitate prinse abia când s-a desenat ecranul:
`LEGATURA FC` și `TEMPERATURA` au exact 11 caractere, cât coloana → `LEGATURA
FCOK  0.3 s`; și unitatea dispărea odată cu valoarea (`-` în loc de `- fps`),
ceea ce pe un rând cu două cifre nu spune care lipsește.

### `docs/CHECKLIST_TEREN.md`

Cinci capitole: înainte de plecare, la sosire (inclusiv verificările la sol
**cu elicele demontate**), modul de cursă, între curse, după sesiune.

Partea care contează e ultima: **tabel simptom → cauză probabilă → fix**, pe
șase categorii, cu referințe la `§5`. Include simptomele reale din istoricul
proiectului: `Device or resource busy`, `_ARRAY_API not found`, `Arm:
Rangefinder 1: No Data`, `NAV_TAKEOFF result=4`, „capac pe obiectiv",
„canalul 3 la 500 PWM de trim", `.bin` cu octeți lipsă.

Un test verifică prezența simptomelor-cheie și a secțiunilor — un checklist
din care lipsește exact ce ți s-a întâmplat nu ajută pe nimeni.

---

## H5 — colectarea de evidență

```bash
python3 tools/collect_session.py --nota "cursa 2, vant lateral"
```

```
data/sessions/20260913-154500/
    manifest.json     versiuni, hash de commit + dirty, parametri, calibrare
    fc/log_42.bin     descărcat de la FC prin LOG_REQUEST_DATA
    logs/             logurile companion-ului
    frames/           ring buffer, cu timestamp de captură în nume
    params.txt        parametrii CITIȚI de pe vehicul
    camera_pi.yaml    calibrarea folosită
```

Trei lucruri care fac diferența între „un director cu fișiere" și evidență:

- **Golurile din `.bin` se raportează.** Protocolul nu retransmite de la
  sine; urmărim ce octeți au sosit. Un `.bin` cu găuri arată ca o dovadă
  până când cineva îl deschide — mai bine se știe acum decât la
  scrutineering. Testat: 2 blocuri pierdute → `180 octeți lipsă`, semnalat.
- **`dirty` contează mai mult decât hash-ul.** Un hash curat spune exact ce
  cod a zburat; un arbore murdar spune că **nu știm**. Ascuns, ar fi o
  minciună liniștitoare — deci se afișează ca avertisment.
- **Parametrii sunt cei CITIȚI de la FC**, nu cei din fișier. Un parametru
  care nu există pe firmware se notează `-`, nu se omite (§5.10).

Cadrele din ring buffer poartă **timestamp-ul de captură în milisecunde** în
nume, nu un index: un `0001.png` nu se poate pune în relație cu `.bin`-ul,
care e tot rostul lui 6.2.1.30. Se scriu la `SIGUSR1`:

```bash
kill -USR1 $(systemctl show -p MainPID --value nova-monitor)
```

FC-ul inaccesibil **nu** anulează colectarea: restul evidenței se adună, iar
eroarea intră în manifest.

---

## Ce NU s-a putut testa fără teren

| Ce | De ce |
|---|---|
| **Reconectarea pe un cablu real** | Toate testele H1 folosesc o legătură mavutil falsă. Comportamentul unui port serial care se mișcă fizic — buffere pe jumătate scrise, octeți corupți, `termios` în stare ciudată — nu e reprodus. Backoff-ul și non-blocarea sunt verificate; **repornirea efectivă a comunicației nu**. |
| **Pragul `LINK_MAX_AGE_S = 1.0 s`** | Ales prin raționament (dublul pragului de detecție, o treime din timeout-ul de reconectare), nu măsurat. Pe teren poate fi prea sensibil dacă Pi-ul are vârfuri de planificare. |
| **`setup_pi.sh` pe Trixie** | Gărzile și alegerea căii sunt testate cu fixture-uri; instalarea efectivă a pachetelor pe aarch64 **nu**. |
| **Serviciul sub systemd** | Unitatea validează cu `systemd-analyze verify`, dar `ProtectSystem=strict` + noul `frames-dir` n-au fost exercitate la runtime. Dacă scrierea cadrelor e blocată, se vede abia acolo. |
| **Descărcarea unui `.bin` real** | Testat pe 3 KB sintetici. Un log de 10 MB pe serial la 921600 durează minute și poate expune probleme de flux pe care un FC fals nu le are. |
| **Ecranul în soare** | Contrastul e proiectat pentru asta, dar „citibil de la un metru" se verifică doar cu ochii, afară. |
| **Python 3.13** | Venv-ul de paritate rulează pe 3.10. Acoperă versiunea de OpenCV, nu versiunea de Python. |

---

## Decizii deschise

**1. `LINK_MAX_AGE_S = 1.0 s`** — provizoriu, marcat ca atare în cod. De
recalibrat după primele zboruri: dacă apar BRAKE-uri pe `link_age` fără o
cauză reală, pragul e prea strâns.

**2. Ring buffer la `SIGUSR1`, nu automat.** Serviciul nu scrie cadre de la
sine — ar umple cardul. Grupul C (8.3.3) va trebui să decidă declanșatorul
automat; până atunci, comanda e în checklist.

**3. Serviciul tot nu poate comanda vehiculul** (`ReadOnlyVehicle`, §5.25).
Am adăugat `check_link` în lista albă, cu motivul scris: e singura intrare
care trimite ceva pe fir (`SET_MESSAGE_INTERVAL` după reconectare) și e o
cerere de **date**, nu o comandă de zbor.

**4. `race_mode` cu `--no-mavlink`** pornește preflight-ul fără FC, dar
verificările sărite tot opresc pornirea. Intenționat: pe banc rulezi
`preflight_check.py` direct, nu modul de cursă.

---

## Validare

**173/173 teste**, rulate pe ambele stive: cea de dezvoltare (OpenCV 5.0 +
numpy 2.2.6) și venv-ul de paritate reconstruit pentru Trixie (OpenCV 4.10 +
numpy 2.2.4).

| suită | teste |
|---|---|
| `test_state_machine` | 13 |
| `test_safety` | 23 |
| `test_handover` | 16 |
| `test_detector_pi` | 17 |
| `test_make_calib_target` | 13 |
| `test_calibrate_camera` | 14 |
| `test_compare_detectors` | 8 |
| `test_pi_tooling` | 24 |
| **`test_link`** (nou, H1) | **13** |
| **`test_ops`** (nou, H2–H5) | **32** |

45 de teste noi în runda asta, toate cu cel puțin un caz negativ pe unealtă.
Cele care contează cel mai mult:

- port care nu răspunde **niciodată** → reîncearcă la infinit, backoff
  1/2/4/8, cel mai lung apel **0.0 ms** pe 400 de cicluri
- supervizorul **nu** își consumă cele 5 reîncercări de mod cu legătura
  căzută; la revenire trimite din prima
- `.bin` cu 2 blocuri pierdute → `180 octeți lipsă`, raportat
- preflight sărit (nu picat) → modul de concurs **tot** refuză
- `race_mode.py` nu are voie să construiască piese proprii (verificat în sursă)

---

## Descoperiri adăugate la CLAUDE.md §5

| § | Ce |
|---|---|
| 5.24 (rescris) | Stiva reală de pe vehicul; regula apt-vs-pip după versiunea de OpenCV |
| 5.27 | Două procese, un singur `/dev/serial0` |
| 5.28 | `WINDOW_AUTOSIZE` ignoră tăcut fullscreen-ul; trei contexte de previzualizare |
| 5.29 | Un ecran de teren se proiectează invers față de un dashboard |
