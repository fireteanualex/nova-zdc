# Test rapid de aterizare autonomă

Scop: **să vezi dacă aterizarea merge.** Atât. Nu e configurația de concurs
și nu pretinde să fie — vezi „Ce lipsește față de concurs" la final.

---

## Întâi, răspunsul la întrebarea care contează

**Modul LAND NU pornește procedura autonomă.**

Verificat în cod: în `nova/state_machine.py`, `MODE_LAND` apare doar în două
feluri — companion-ul îl **comandă** după ce poarta acceptă, și verifică apoi
că vehiculul e **încă** în LAND. Nimic nu se declanșează când pilotul alege
LAND de pe emițător.

Dacă pui LAND manual: ArduPilot face o aterizare normală, **fără** precision
landing. `PLND_ENABLED` pornește pe 0 și îl aprinde companion-ul doar pentru
segmentul autonom (§5.8). Nu te va atrage spre marker.

**Intrarea e un comutator AUX, pe frontul crescător.** Implicit canalul 7,
reglabil:

```bash
pi/descent_test.sh --aux-channel=6
```

De ce un canal dedicat și nu modul LAND: poarta trebuie să **valideze**
(altitudine, distanță, marker văzut, manșe în neutru) și să poată **refuza**
cu motiv. Un mod de zbor nu are unde să întoarcă un refuz, iar la
scrutineering nu s-ar putea demonstra în ce condiții s-a activat (§8).

---

## Pasul 0 — trimite codul pe Pi

**De pe desktop**, nu de pe Pi:

```bash
cd ~/nova-zdc
pi/deploy.sh <user>@<ip-pi> --setup      # prima data: trimite + instaleaza stiva
pi/deploy.sh <user>@<ip-pi>              # de fiecare data dupa
pi/deploy.sh <user>@<ip-pi> --dry-run    # arata ce s-ar schimba
```

Adresa se poate ține și în mediu, ca să n-o scrii de fiecare dată:

```bash
export NOVA_PI=<user>@<ip-pi>
pi/deploy.sh
```

**Ce se trimite:** codul, `config/`, `pi/`, `tools/`, `docs/` **și `.git/`**.

`.git/` deliberat: checklistul cere `git status` curat pe Pi înainte de
zbor — fără el nu se poate spune ce cod a zburat, iar 6.2.1.30 se sprijină
exact pe asta. Costă câțiva MB.

**Ce NU se trimite:** `data/` (cadre E2, sesiuni, campanii — sute de MB),
`__pycache__`, și **venv-ul**. Venv-ul se construiește pe Pi, cu
`--system-site-packages`; copiat de pe desktop ar fi legat de alt Python și
alt numpy, și s-ar rupe la primul `import picamera2` (§5.24).

Scriptul îți spune, **înainte** de a trimite, pe ce commit ești și dacă
arborele e murdar. Nu blochează — dar la pistă nu mai ai când să afli.

> **Dacă ai recalibrat camera PE Pi**, fișierul de acolo e mai nou.
> Adu-l întâi, altfel sincronizarea îl suprascrie:
> ```bash
> scp <user>@<ip-pi>:~/nova-zdc/config/camera_pi.yaml config/
> ```

### Dacă ai Tailscale

Adresa `100.x.x.x` merge **de oriunde**, nu doar din aceeași rețea. Pe
teren, cu Pi-ul pe hotspot, e diferența între a avea SSH și a nu avea.
Adresele `10.x` sunt locale și se schimbă de la o rețea la alta.

### SSH fără parolă, o dată

```bash
ssh-copy-id <user>@<ip-pi>
```

Altfel `deploy.sh` cere parola de două-trei ori pe rulare.

---

## Pasul 1 — Mission Planner: parametrii

Conectează-te la FC **pe USB** (nu pe TELEM2, îl configurăm chiar acum).

### 1.0 Verifică ÎNTÂI versiunea de firmware

`Help` → versiunea apare la conectare, sau în bara de jos a HUD-ului.

`config/nova_flight.parm` e scris pentru **4.7.0 sau mai nou**. Pe 4.6 și
mai vechi, trei nume nu există:

| | 4.5.x / 4.6.x | **4.7.0+** |
|---|---|---|
| | `ARMING_CHECK` | `ARMING_SKIPCHK` |
| | `WPNAV_ACCEL` | `WP_ACC` |
| | `WPNAV_RFND_USE` | `WP_RFND_USE` |
| | `RNGFND1_MIN_CM` / `_MAX_CM` | `RNGFND1_MIN` / `_MAX` (metri) |

Dacă Mission Planner îți spune **`No matching Params`** pe exact numele
astea, ești pe 4.6 sau mai vechi.

> **Pe 4.5.x folosește `config/nova_flight_4.5.parm`** — generat din
> fișierul de 4.7 de `tools/make_parm_45.py`, cu numele, unitățile și masca
> de armare traduse, și verificat de un test. `config/nova.json`
> (`flight_parm`) îl indică deja, deci preflight-ul îl verifică pe el.
>
> Dacă urci firmware-ul la 4.7.0 (cel mai apropiat de ce s-a măsurat în
> SITL), schimbă `flight_parm` înapoi pe `config/nova_flight.parm`.

Dacă rămâi pe 4.6, traducerea numelor nu e de ajuns: **masca de armare se
inversează.** `ARMING_SKIPCHK` listează verificările pe care le **sari**;
`ARMING_CHECK` pe cele pe care le **faci**.

| intenție | 4.7+ | 4.5 / 4.6 |
|---|---|---|
| sari doar verificarea de telemetru | `ARMING_SKIPCHK = 32768` | `ARMING_CHECK = 1015294` |

Un `32768` pus în `ARMING_CHECK` pe 4.6 ar însemna „fă **doar**
verificarea de telemetru" — fără busolă, GPS, INS, baterie, RC. Exact
invers, și trece orice audit pe valoare: parametrul există și are numărul
cerut (§5.10).

### 1.1 Scrie parametrii — de pe Pi, pe nume

**Nu prin Mission Planner.** `PLND_*` și `RNGFND1_*` sunt ascunși din
*lista* de parametri cât timp `PLND_ENABLED` / `RNGFND1_TYPE` sunt 0, iar
Mission Planner scrie doar ce găsește în lista descărcată — de aici
„No matching Params". FC-ul îi acceptă însă **pe nume** oricând (verificat
pe Copter-4.5.7: `AP_Param::find()` nu ține cont de flag).

De pe Pi, cu vehiculul **dezarmat**:

```bash
tools/check_params.py --conn /dev/serial0 --baud 921600 \
    --parm config/nova_flight_4.5.parm --write --reboot
# asteapta ~15 s sa reporneasca FC-ul, apoi:
tools/check_params.py --conn /dev/serial0 --baud 921600 \
    --parm config/nova_flight_4.5.parm
```

Prima comandă scrie fiecare parametru pe nume, îl citește înapoi, și
repornește FC-ul. A doua trebuie să iasă cu **cod 0** — citirea după boot e
dovada.

**Repornirea nu e opțională.** Pe 4.5.7, `init_precland()` rulează o
singură dată, la boot, și creează backend-ul de precision landing din
`PLND_TYPE`. Cu `PLND_TYPE` proaspăt scris dar fără repornire, backend-ul
nu există: companion-ul aprinde `PLND_ENABLED` la handover, trimite
`LANDING_TARGET`, iar FC-ul le ignoră — fără niciun mesaj. La fel pentru
driverul de telemetru din `RNGFND1_TYPE`.

Dacă totuși vrei prin Mission Planner: `RNGFND1_TYPE = 10` și
`PLND_ENABLED = 1` → Write → **reboot** → Load `nova_flight_4.5.parm` →
Write → **reboot din nou**. Fișierul pune singur `PLND_ENABLED` înapoi pe 0.

Cei 28 de parametri, cu motivul fiecăruia:

| parametru | valoare | de ce |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink2 pe TELEM2 |
| `SERIAL2_BAUD` | 921 | = 921600 (valoarea e în mii) |
| `BRD_SER2_RTSCTS` | **0** | implicit e 2 (Auto); cu RTS/CTS nelegate, auto-detecția depinde de ce e pe pini |
| `RNGFND1_TYPE` | 10 | telemetru prin MAVLink — camera e telemetrul (§5.3) |
| `RNGFND1_ORIENT` | 25 | în jos |
| `RNGFND1_MIN` / `_MAX` | 0.05 / 30 | **în metri**, nu cm (redenumite în 4.6+) |
| `RNGFND1_GNDCLR` | 0.0745 | camera e la 74.5 mm de sol la contact |
| `PLND_ENABLED` | **0** | îl aprinde companion-ul doar în segment |
| `PLND_TYPE` | 1 | MAVLink LANDING_TARGET |
| `PLND_EST_TYPE` | 1 | Kalman |
| `PLND_STRICT` | 2 | |
| `PLND_ALT_MIN` | 0.35 | |
| `SURFTRAK_MODE` | **0** | altfel telemetrul intermitent dă `Terrain Rangefinder Unhealthy` → RTL |
| `TERRAIN_ENABLE` | 0 | idem |
| `WP_RFND_USE` | **0** | altfel urcarea din GUIDED e interpretată „deasupra terenului" (§5.7) |
| `ARMING_SKIPCHK` | 32768 | bit 15 = sare verificarea de rangefinder |
| `DISARM_DELAY` | 20 | marjă pentru pauza pe sol |
| `WP_ACC` | 1.5 | plafonează înclinarea la 8.7° — e o constrângere a **camerei** (§5.48) |

> Dacă tot nu găsești un `PLND_*` sau `RNGFND1_*` după pasul 1, nu ai
> repornit FC-ul — `param fetch` nu e suficient (§5.4).

### 1.2 Modurile de zbor — abortul

`Config` → `Flight Modes`.

- Alege canalul din **`Flight Mode Ch`** (= `FLTMODE_CH`). Nu-l lăsa pe 0.
- Pune **LOITER** pe o poziție și **STABILIZE** sau **ALT HOLD** pe alta.
  Aia e ieșirea ta.

Ăsta e abortul care contează: merge direct în FC și **funcționează și dacă
Pi-ul e mort**. Manșele și supervizorul sunt straturile 2 și 3, și amândouă
trec prin exact procesul care ar putea fi cel stricat.

### 1.3 Canalul de handover

Pe emițător, mapează un **comutator cu două poziții** pe canalul ales
(RC6 sau RC7).

În Mission Planner, `Config` → `Full Parameter List`:

```
RC6_OPTION = 0        (sau RC7_OPTION, dupa canalul ales)
```

**0 = „Do Nothing"**, deliberat. Companion-ul citește canalul brut din
`RC_CHANNELS`; nu vrem ca ArduPilot să facă și altceva la comutare.

Verifică că se mișcă: `Setup` → `Mandatory Hardware` → `Radio Calibration`
— bara canalului trebuie să sară între ~1000 și ~2000 la comutare. Poarta
cere **peste 1700** pentru „sus".

### 1.4 Citește înapoi

Un parametru nu e setat până nu a fost citit înapoi (§5.10) — ArduPilot
acceptă tăcut scrieri pe nume inexistente.

```bash
tools/check_params.py --conn /dev/serial0 --baud 921600
```

Cod 0 = toate există **și** au valoarea cerută. De rulat după reboot.

---

## Pasul 2 — Pi-ul

Cablajul, 3 fire (TX↔RX încrucișate, GND comun, **fără 5 V**) și UART-ul
sunt în [`pi/README.md`](../pi/README.md), pașii 1–4. Pe scurt:

```bash
ssh <user>@<ip-pi>
cd ~/nova-zdc
tools/setup_pi.sh                      # daca n-ai dat --setup la pasul 0
sudo pi/setup_uart.sh && sudo reboot   # OBLIGATORIU pe Pi 4
# dupa repornire:
pi/setup_uart.sh --check               # trebuie: /dev/serial0 -> ttyAMA0
pi/bringup.sh --check                  # verifica tot, nu porneste nimic
```

---

## Pasul 3 — scriptul de pornire automată

Vrei ca Pi-ul să pornească singur monitorul la fiecare alimentare:

```bash
cd ~/nova-zdc && pi/install.sh
```

Instalează un serviciu **de utilizator** legat de sesiunea grafică. Cere
autologin:

```
sudo raspi-config
  → System Options → Boot / Auto Login → Desktop Autologin
```

De ce de utilizator și nu de sistem: fereastra OpenCV are nevoie de un
ecran. Un serviciu de sistem pornește înaintea oricărei sesiuni, `imshow`
eșuează, iar `systemctl status` arată verde fără nimic pe monitor (§5.28).

```bash
systemctl --user status nova-bringup
journalctl --user-unit nova-bringup -f
systemctl --user stop nova-bringup      # INAINTE de testul de aterizare
```

> **Unde sunt logurile.** Pe Raspberry Pi OS jurnalul stă implicit în
> memorie (`/run/log/journal`), iar în modul ăsta serviciile de utilizator
> nu au fișiere proprii: `journalctl --user` spune „No journal files
> were found" deși logurile există. Sunt în jurnalul de **sistem**:
>
> ```bash
> journalctl --user-unit nova-bringup -f      # filtreaza serviciul nostru
> ```
>
> Dacă dă „Permission denied", pune `sudo` în față. Și, independent de
> jurnal, fiecare rulare scrie în `~/nova-logs/` — ultima:
>
> ```bash
> tail -f "$(ls -t ~/nova-logs/bringup-*.log | head -1)"
> ```
>
> Fișierul apare doar după ce monitorul pornește; un eșec timpuriu (port
> ocupat, calibrare lipsă) e doar în jurnal.

> **Important:** serviciul ține `/dev/serial0` **și camera**. O a doua
> copie pornită peste el primește „Camera __init__ sequence did not
> complete", iar două procese pe același UART își fură octeții: heartbeat-ul
> trece, citirile de parametri se pierd.
>
> - `descent_test.sh` îl **oprește singur**, înainte de preflight, și îți
>   spune cum îl repornești (`systemctl --user start nova-bringup`)
> - `bringup.sh` rulat de mână peste el **refuză** și îți spune să-l oprești
> - pentru orice altceva: `systemctl --user stop nova-bringup`

Autoboot-ul pornește **monitorul**, nu coborârea: detector activ, zero
comenzi. Testul de aterizare se pornește de mână, deliberat.

---

## Pasul 4 — proba pe masă, cu ELICELE DEMONTATE

Cea mai ieftină verificare din tot lanțul. Cinci minute, zero risc.

```bash
nano config/nova.json     # autonomy_enabled: true
pi/descent_test.sh --aux-channel=6
```

Armezi, lași manșele libere ~1 s, ridici comutatorul. Poarta trebuie să
**REFUZE**:

```
!! HANDOVER REFUZAT: altitudine in afara ferestrei: 0.1 m
```

Dintr-o apăsare ai dovedit că: comutatorul ajunge de la emițător la FC și
de acolo la companion, poarta îl vede **pe frontul crescător**, citește
altitudinea, și refuză cu un motiv citibil.

Repetă cu manșele mișcate — motivul trebuie să devină „manșă în afara
neutrului". Dacă nu se întâmplă **nimic** la comutare, nu ai o problemă de
zbor: ai o problemă de cablaj RC, și ai aflat-o pe masă.

---

## Pasul 5 — zborul

Markerul ArUco **ID 26, latura codată 480 mm**, plan, curat, pe iarbă sau
pământ moale.

```bash
systemctl --user stop nova-bringup       # elibereaza portul
pi/descent_test.sh --aux-channel=6       # ramane pornit tot zborul
```

Scriptul verifică precondițiile, cere să scrii `ZBOR`, apoi așteaptă.

La manșe:

| | |
|---|---|
| 1 | **o aterizare manuală întâi**, pe marker. Dacă nu aterizează curat manual, PLND nu are ce repara |
| 2 | decolezi, urci la **6–8 m** deasupra markerului |
| 3 | lateral **sub ~2 m**. Poarta acceptă 6.5 m, geometria nu (§5.48) |
| 4 | **LOITER**, manșe libere **~1 s** — poarta măsoară în fereastra asta |
| 5 | ridici comutatorul de handover |
| 6 | **mâna pe comutatorul de mod** ~30 s. Nu pe ecran |

Ce trebuie să vezi în log:

```
>> IDLE -> HANDOVER_CHECK   (AUX sus, alt 7.2 m)
>> HANDOVER_CHECK -> ACQUIRE
>> ACQUIRE -> DESCEND_TRACK
>> DESCEND_TRACK -> SCORING_CAPTURE   (incadrare 0.63)
>> SCORING_CAPTURE -> FINAL_DESCENT   (incadrare 0.72)
>> FINAL_DESCENT -> TOUCHDOWN_CONFIRM
```

**Se oprește pe sol** și ArduPilot dezarmează. Fără urcare automată —
implicit e `--no-ascent`, fiindcă o urcare imediat după primul touchdown e
exact genul de surpriză care te face să tragi de manșe. După ce coborârea
merge o dată, `--full-sequence` adaugă urcarea la 5 m.

### Când abortezi, fără discuție

- vehiculul face altceva decât „coboară drept spre marker"
- oscilație laterală care crește
- ai pierdut orientarea vehiculului

### Când NU abortezi

- pentru că pare **încet** — coborârea e 0.5 m/s prin proiect
- pentru că s-a **oprit scurt** din coborât — supervizorul a frânat, e
  chiar comportamentul cerut de 15.2.9
- pentru că ecranul arată ceva ce nu înțelegi — ecranul nu zboară vehiculul

> Garanția „planează în loc să aterizeze" la pierderea detecției **nu se
> aplică sub ~1.2 m AGL**: frânarea consumă 1.00 m măsurat la 0.5 m/s. Sub
> prag, pierderea detecției duce la contact, nu la hover.

---

## Dacă poarta refuză

| motiv | ce faci |
|---|---|
| `altitudine in afara ferestrei` | 5–12 m |
| `distanta prea mare` | apropie-te de marker |
| `marker nedetectat` | prea departe lateral pentru altitudinea asta, sau marker în umbră |
| `mansa in afara neutrului` | lasă manșele, ~1 s, reia |
| `autonomy_enabled = false` | E0 închis — pasul 4 |

Un refuz costă câteva secunde. Reia; nu forța.

---

## După

```bash
tools/collect_session.py --nota "test aterizare 1"
```

Adună `.bin`-ul, logurile, cadrele, parametrii **citiți de pe vehicul** și
calibrarea folosită. Cele două imagini sunt în `data/scoring/`.

---

## Ce lipsește față de concurs

Pentru un test de aterizare sunt acceptabile. Pentru o încercare punctată,
nu:

| | stare |
|---|---|
| **15.2.5** — fără GNSS în segment | `nova/ekf_source.py` merge și e măsurat în sim, dar **nu e cablat în `nova_pi.py`**. Pe vehicul, EKF-ul primește GNSS tot segmentul (element 36) |
| **15.2.4** — geofence 10 m în firmware | `nova/fence.py` validat în SITL, **cablat nicăieri**. Rămân monitoarele din supervizor, care rulează pe Pi (element 35) |
| **E2** | nefăcut. Până atunci, cifrele din Gazebo sunt pregătire, nu înlocuitor |
| calibrarea camerei | rms 0.829 — acceptat (prag 0.85, decizia echipei), dar peste 0.2–0.5 cât dă de obicei o calibrare bună. E2 spune dacă ajunge |
| `STICK_DEADBAND_PWM` | 80, provizoriu — măsurat pe un gamepad, nu pe emițătorul real |
