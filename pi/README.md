# Bring-up pe Raspberry Pi 4 + Pixhawk 6C

Scris pentru **primul test pe hardware**, nu pentru cursă. Scopul e să
ajungi la un Pi care, pornit, arată pe ecran ce vede camera, cu markerul
detectat, și care citește telemetria de la Pixhawk — fără să comande nimic.

> **Calibrarea e deja în repo.** `config/camera_pi.yaml` — ChArUco, 60 de
> poze, `fy = 1038.7 px`, HFOV 96.0° / VFOV 63.9°.
>
> **Are `rms = 0.829 px`, peste pragul de zbor de 0.5.** Detectorul o
> refuză în configurația de zbor; bring-up-ul o acceptă cu `--max-rms`, o
> rulare pe rând, și o spune de fiecare dată. **De refăcut înainte de E2.**
> O cameră bine calibrată stă la 0.2–0.5; 0.83 sugerează țintă neplană,
> poze mișcate sau colțuri neacoperite (§5.34).
>
> Pragul din cod **nu** a fost ridicat: e citit de garda care decide dacă se
> zboară, iar un test verifică faptul că nici `start_flight.sh`, nici modul
> de cursă nu primesc `--max-rms`.

---

## Cablajul — 3 fire, nu 5

| Pixhawk 6C TELEM2 | | Raspberry Pi 4 |
|---|---|---|
| pin 2 — TX | → | pin 10 — GPIO 15 (RXD) |
| pin 3 — RX | → | pin 8 — GPIO 14 (TXD) |
| pin 6 — GND | → | pin 6 — GND |

**TX și RX se încrucișează.** Ambele capete sunt 3.3 V, deci nu trebuie
convertor de nivel.

**Pinul de 5 V al lui TELEM2 NU se leagă la Pi.** Pi-ul are alimentarea lui;
două surse legate împreună se bat, iar în cel mai bun caz repornește Pi-ul
când motoarele trag curent.

RTS și CTS rămân nelegate — de aceea `BRD_SER2_RTSCTS` se pune pe 0 la
pasul 4.

---

## Pașii, în ordine

Ordinea nu e arbitrară: fiecare pas verifică ce a făcut precedentul.

### 1. Copiază repo-ul pe Pi

De pe desktop:

```bash
# inlocuieste <ip-pi> cu adresa Pi-ului
rsync -av --exclude data/ --exclude .git/ ~/nova-zdc/ pi@<ip-pi>:~/nova-zdc/
ssh pi@<ip-pi>
```

Sau direct pe Pi, dacă ai repo-ul pe git:

```bash
ssh pi@<ip-pi>
git clone <url> ~/nova-zdc
```

### 2. Instalează stiva Python

```bash
cd ~/nova-zdc
tools/setup_pi.sh
```

Durează câteva minute. **numpy, OpenCV și picamera2 vin din apt**, nu din
pip, iar venv-ul se creează cu `--system-site-packages` — altfel picamera2
se rupe cu `_ARRAY_API not found`, departe de cauză (§5.24).

Verifică ce a ieșit, nu presupune:

```bash
tools/preflight_check.py --no-mavlink --no-camera
```

Raportează distribuția, Python, numpy, OpenCV **și calea fiecăruia**. Calea
e cea care arată dacă pip a pus o copie peste sistem.

### 3. Pregătește UART-ul — pasul care se uită

```bash
sudo pi/setup_uart.sh --check    # ce e acum
sudo pi/setup_uart.sh            # configureaza
sudo reboot
```

După repornire:

```bash
ssh pi@<ip-pi> 'cd ~/nova-zdc && pi/setup_uart.sh --check'
```

Trebuie să vezi `/dev/serial0 -> /dev/ttyAMA0`.

> **De ce contează.** Pe Pi 4, implicit `/dev/serial0` arată spre
> **miniUART** (`ttyS0`), al cărui tact vine din ceasul miezului VPU — care
> se scalează cu încărcarea și temperatura. La 921600 baud legătura merge
> câteva minute și apoi începe să dea caractere greșite. Simptomul arată ca
> un cablu prost sau un Pixhawk defect.
>
> `dtoverlay=disable-bt` mută PL011 (`ttyAMA0`, ceas propriu) înapoi pe
> GPIO 14/15. Scriptul face și asta, și scoate consola serială — altfel două
> programe stau pe același port și pymavlink primește pachete tăiate fără
> să se plângă.

### 4. Parametrii pe Pixhawk

Din Mission Planner / QGroundControl, sau prin MAVProxy pe USB:

```
SERIAL2_PROTOCOL 2      MAVLink2
SERIAL2_BAUD     921    921600
BRD_SER2_RTSCTS  0      fara control de flux (RTS/CTS nelegate)
```

`TELEM2 = SERIAL2` pe 6C — verificat în sursă, nu din memorie:
`hwdef.dat:36` dă `SERIAL_ORDER OTG1 UART7 UART5 ...`, iar `# telem2` e
deasupra lui `UART5`, care e al treilea, adică indexul 2.

`BRD_SER2_RTSCTS` implicit e **2 (Auto)** pe plăcile ChibiOS. Auto-detecția
testează dacă bufferul de ieșire se umple la pornire — cu RTS/CTS nelegate,
rezultatul depinde de ce se întâmplă să fie pe pini. Se pune 0 explicit.

Apoi citește-i înapoi, de pe Pi (§5.10 — un parametru nu e setat până nu a
fost citit înapoi):

```bash
tools/check_params.py --conn /dev/serial0 --baud 921600
```

### 5. Calibrarea camerei

Deja în repo, deci nu ai ce copia. Verifică doar că a ajuns:

```bash
grep -c . config/camera_pi.yaml && pi/bringup.sh --check | grep -A3 calibr
```

Când o refaci (rms sub 0.5), pe Pi, cu o tablă ChArUco tipărită:

```bash
tools/make_calib_target.py --help     # genereaza ținta de tipărit
tools/calibrate_camera.py --help      # ~24 poze, apoi scrie config/camera_pi.yaml
```

Fereastra e **fullscreen** la uneltele de banc — ai nevoie de detaliu, iar
ecranul Pi-ului e mic (§5.28). Ieșire pe `q` sau `Escape`.

> **Nu ridica pragul de RMS ca să treacă.** `MAX_REPROJ_ERR_PX = 0.5` e citit
> de detectorul de bord; ridicat global, slăbește tăcut exact garda care
> decide dacă se zboară. Pentru o rulare anume există `--max-rms` (§5.34).
> Și RMS-ul nu e criteriu de valabilitate: 24 de poze identice dau RMS 0.061
> cu `fx` greșit cu +754% (§5.22).

### 6. Verifică tot, fără să pornești nimic

```bash
cd ~/nova-zdc && pi/bringup.sh --check
```

Trece prin: venv și versiuni, UART (`ttyAMA0` sau nu), cine ține portul,
calibrarea, apoi preflight-ul întreg.

### 7. Pornește monitorul

```bash
cd ~/nova-zdc && pi/bringup.sh
```

Din sesiunea grafică a Pi-ului: fereastră pe tot ecranul. Prin SSH fără X,
pornește fără fereastră și spune de ce. Cu `ssh -X pi@<ip-pi>` merge și
remote, dar încet.

### 8. Pornire automată la fiecare boot

```bash
cd ~/nova-zdc && pi/install.sh
```

Serviciu **de utilizator**, legat de sesiunea grafică — un serviciu de
sistem pornește înainte să existe un ecran, deci `imshow` ar eșua și
fereastra nu ar apărea niciodată, în timp ce `systemctl status` ar arăta
verde.

Cere autologin: `sudo raspi-config` → System Options → Boot / Auto Login →
**Desktop Autologin**.

```bash
systemctl --user status nova-bringup      # starea
journalctl --user -u nova-bringup -f      # logul, in timp real
systemctl --user restart nova-bringup
systemctl --user stop nova-bringup
ls -t ~/nova-logs/ | head                 # logurile de rulare
```

> Nu porni în același timp `nova-monitor.service` (cel de sistem, fără
> ecran): se bat pe `/dev/serial0` (§5.27).

---

## Override pilot și comutatorul de handover

**În bring-up sunt cablate, dar dorm** — și e corect.

`nova_pi.py` construiește `OverrideMonitor` și `SafetySupervisor` și le dă
lui `run_loop`. Dar supervizorul se armează **din fază**
(`AUTONOMOUS_PHASES`, §5.14), iar cu E0 închis secvența rămâne în `IDLE`,
deci `update()` iese pe `if not self.armed`. Nu e o scăpare: companion-ul
nu comandă nimic în monitor, deci nu există de la ce să preia pilotul — el
are oricum controlul integral prin FC.

> Tabelul din §8 spune „Override pilot … **toate** fazele". Mai exact:
> toate fazele **autonome**. În afara segmentului nu e nimic de întrerupt.

**Înainte de proba de coborâre**, două măsurători cu emițătorul REAL, cu
Pi-ul legat la FC:

```bash
tools/calibrate_sticks.py --conn /dev/serial0 --baud 921600
```

Zgomotul manșelor în repaus → `STICK_DEADBAND_PWM` (elementul deschis 13).
**Cifra de pe gamepad nu se transferă**: acolo s-a măsurat σ = 0.00 și
amplitudine 0 PWM, dar aia e o ieșire cuantizată, nu un gimbal analogic cu
link RC. Lasă manșele libere pe toată durata; orice atingere strică
statistica.

```bash
tools/check_rc_override.py --conn /dev/serial0 --baud 921600
```

Verifică precondiția întregului lanț: FC-ul chiar raportează înapoi în
`RC_CHANNELS` ce primește de la emițător? Dacă **nu**, poarta nu vede
comutatorul AUX 7 și monitorul de override nu funcționează — iar orice test
cu pilot în buclă n-ar însemna nimic.

Mai trebuie, pe emițător (elementul deschis 20, lipsesc deliberat din
`nova_flight.parm` pentru că depind de transmițător):

- **AUX 7** pe un comutator cu două/trei poziții — e singura cale de intrare
  în segmentul autonom, pe **frontul crescător**, cu ≥1700 PWM sus
- **`FLTMODE_CH` + `FLTMODE1..6`** — abortul robust e un mod de zbor mapat
  direct pe FC, care nu trece prin Pi deloc (16.2.3). Detecția pe manșe e
  necesară pentru 15.1.7, dar nu e singurul strat.

---

## Ce te uiți să vezi

| ce | unde | prag |
|---|---|---|
| FPS cameră cu detectorul pornit | linia de stare | ~30 fps |
| rata de detecție pe marker tipărit | linia de stare | cât mai aproape de 100% |
| latență captură → publicare, p99 | linia de stare | **sub 150 ms** (E1.4) |
| `range_m` față de ruletă | comparat manual | sub 5% (E2) |
| HEARTBEAT de la FC | linia de stare | fără întreruperi |

Latența e singura cifră pe care simularea **nu** o poate da: desktopul nu e
Pi 4, iar în Gazebo se măsoară întârzierea de coadă, nu timpul de calcul
(§5.56). Asta e prima măsurătoare care valorează ceva și se face numai aici.

Lasă-l să meargă 10–15 minute și uită-te dacă FPS-ul scade — pe Pi 4 fără
răcire, throttling-ul termic apare după câteva minute, nu imediat.

---

## Proba de coborâre autonomă

```bash
pi/descent_test.sh --check          # doar precondițiile, nu zboară nimic
pi/descent_test.sh                  # briefing + confirmare + rulare
pi/descent_test.sh --full-sequence  # și urcarea la 5 m (15.2.7)
```

Verifică, în ordine, și **refuză** dacă ceva lipsește: E0, calibrarea,
`FLTMODE_CH`, `RC7_OPTION`, heartbeat-ul, preflight-ul întreg. Apoi arată
un briefing și cere să scrii `ZBOR` — un `y` se apasă din reflex.

**Implicit nu urcă după contact.** Secvența se încheie pe sol și ArduPilot
dezarmează singur. O urcare automată imediat după primul touchdown e exact
genul de surpriză care te face să tragi de manșe. `--full-sequence` o
pornește, după ce coborârea a mers o dată.

Ce face vehiculul:

```
pilotul aduce la 5–12 m deasupra markerului, LOITER, manșe libere ~1 s
pilotul ridică AUX 7   → poarta ACCEPTĂ sau REFUZĂ, cu motiv
companion-ul cere LAND → așteaptă confirmarea FC
coborâre cu PLND       → LANDING_TARGET la 20 Hz
încadrarea la 0.72     → coborâre verticală
contact                → pauză pe sol → STOP
```

### Abort, în ordinea încrederii

| | cale | depinde de Pi? |
|---|---|---|
| 1 | **comutatorul de mod** (`FLTMODE_CH`) | **nu** — merge direct în FC |
| 2 | manșele → companion comandă LOITER (150 ms în SITL) | da |
| 3 | Safety Supervisor → BRAKE / RTL | da |

Prima e singura care funcționează dacă Pi-ul e mort, blocat sau
deconectat. Scriptul refuză să pornească fără ea. **Mâna pe comutator tot
segmentul** — durează ~30 s, nu e un moment în care să te uiți la ecran.

> **Geofence-ul din firmware nu e activ.** `nova/fence.py` e validat în
> SITL dar nu e cablat în nicio aplicație (element deschis 35). Rămân
> monitoarele de rază și plafon din supervizor — care depind de Pi. Pentru
> o probă de test, într-un spațiu deschis, cu pilot pe comutator, e
> acceptabil; pentru cursă nu.

---

## De la monitor la coborâre

Monitorul nu comandă nimic: `config/nova.json: autonomy_enabled` e **false**,
iar poarta de handover refuză orice cerere înainte de orice altă condiție
(§5.16). Nu există activare din linia de comandă sau din mediu — deliberat.

Ca să ajungi la coborârea autonomă, ordinea e:

1. **E2 pe masă**, fără să zboare nimic — marker tipărit, ruletă, trei
   condiții de lumină:
   ```bash
   tools/run_e2.py --help
   ```
   Măsoară eroarea de distanță față de ruletă și latența pe Pi. Fără ea,
   cifrele din simulare sunt pregătire, nu înlocuitor.

2. **Verifică că nimic nu atârnă în conul camerei.** Zona liniștită a
   markerului e 60 mm — sub un modul ArUco — și nu iartă umbre, murdărie
   sau o piesă care intră puțin în cadru. În Gazebo, corpul unui gimbal
   tăia un colț al markerului și rupea detecția sub 1 m, cu centrarea
   perfectă (§5.46).

3. **Abia apoi** ridici garda, în fișierul versionat, cu commit care citează
   raportul E2:
   ```bash
   # dupa ce E2 trece
   nano config/nova.json          # autonomy_enabled: true
   git commit -am "E0 ridicat: raport E2 <data>"
   tools/start_flight.sh          # cere confirmare
   ```

Pasul 3 editează un fișier și nu exportă o variabilă, pentru că o gardă
care se poate ridica dintr-un `export` se ridică din greșeală. Un test
verifică explicit că fișierul din repo e în starea închisă.

---

## Când ceva nu merge

| simptom | cauză probabilă | ce faci |
|---|---|---|
| niciun HEARTBEAT | TX/RX neîncrucișate, sau `SERIAL2_PROTOCOL` ≠ 2 | verifică firele; `check_params.py` |
| HEARTBEAT apoi tăcere / caractere greșite | `/dev/serial0` → `ttyS0` (miniUART) | `sudo pi/setup_uart.sh && sudo reboot` |
| legătura merge într-un sens | `BRD_SER2_RTSCTS` pe Auto cu RTS/CTS nelegate | pune-l pe 0 |
| `Permission denied` pe `/dev/serial0` | nu ești în `dialout` | `sudo usermod -aG dialout $USER`, apoi relogare |
| `Errno 16 Device or resource busy` | `nova-monitor` ține portul | `sudo systemctl stop nova-monitor`, sau `--stop-service` |
| pachete MAVLink tăiate, fără eroare | consola serială e încă pe port | `sudo pi/setup_uart.sh` |
| `Can't initialize GUI backend` | fără sesiune grafică | rulează din desktop, sau `ssh -X`, sau `--no-window` |
| fereastra nu devine fullscreen | — | e deja tratat: `WINDOW_NORMAL` înainte de `WND_PROP_FULLSCREEN` (§5.28) |
| detectorul refuză să pornească | `config/camera_pi.yaml` lipsă sau nu e reală | pasul 5 |
| `import picamera2` → `_ARRAY_API not found` | numpy din pip peste cel din apt | recreează venv-ul cu `--system-site-packages` |
| FPS scade după câteva minute | throttling termic | răcire; `vcgencmd get_throttled` |
| serviciul pornește și moare imediat | repo-ul nu e la `~/nova-zdc` | editează `ExecStart` în `~/.config/systemd/user/nova-bringup.service` |

Pentru diagnostic în timp real:

```bash
journalctl --user -u nova-bringup -f
```
