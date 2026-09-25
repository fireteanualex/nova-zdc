# Inventarul scripturilor NOVA — pregătire pentru consolidare

Stare la 25.09.2026, commit `dca5528`. Scop: o hartă completă a ce există,
ce rulează efectiv pe dronă, ce e redundant și ce e mort, ca echipa să
poată decide consolidarea pe fapte. Raportul nu ia decizii; secțiunea 9
propune, secțiunile 1–8 descriu.

Cifre brute:

| categorie | fișiere | linii |
|---|---|---|
| `nova/` (biblioteca companion) | 17 | 6 520 |
| `tools/*.py` fără teste | 33 | 9 431 |
| `tools/test_*.py` | 18 | 11 874 |
| shell (`pi/`, `tools/*.sh`, `start_sim.sh`) | 9 | 2 037 |

Testele sunt mai mari decât codul pe care îl testează. Multe verifică
**textul sursă** al scripturilor (grep pe `bringup.sh`, `nova_pi.py` etc.),
deci orice redenumire sau mutare le rupe — de luat în calcul la
consolidare (§8.9).

---

## 1. Ce rulează efectiv pe dronă — calea de zbor

Lanțul de pornire, de la boot la comenzi către FC:

```
boot Pi
 └─ ~/.config/autostart/nova-bringup.desktop        (sesiunea grafică)
     └─ systemctl --user start nova-bringup.service
         └─ pi/bringup.sh                             (verificări + decizie)
             ├─ nova/config.py: autostart_mode()  → "monitor" | "zbor"
             ├─ zbor:    pi/descent_test.sh --auto
             │             ├─ tools/preflight_check.py
             │             │     └─ importă check_calibration din tools/nova_service.py
             │             │     └─ rulează tools/check_params.py ca subproces
             │             └─ tools/nova_pi.py  (--no-authority --no-ascent --aux-channel N)
             └─ monitor: tools/nova_pi.py --monitor --monitor-motiv "..."
```

`tools/nova_pi.py` e aplicația de bord. Ce cablează:

| modul | rol în zbor | stare |
|---|---|---|
| `nova/config.py` | citește `config/nova.json` (E0, autostart, canal AUX, rotație cameră, expunere, ROI) | activ |
| `nova/vehicle.py` | MAVLink: telemetrie, comenzi de mod, parametri, `LANDING_TARGET`, `DISTANCE_SENSOR` | activ |
| `nova/detector_pi.py` | cameră → ArUco → solvePnP → `Detection` | activ |
| `nova/detection.py` | contractul `Detection` + `CameraModel` | activ |
| `nova/handover.py` | poarta de intrare (E0, altitudine, distanță, marker, manșe) | activ |
| `nova/rc.py` | monitor de manșe: neutru, deadband, override | activ |
| `nova/safety.py` | supervizor: detecție, link, rază, plafon, coborâre, înclinare, override | activ |
| `nova/state_machine.py` | secvența IDLE → … → HANDBACK + `run_loop` | activ |
| `nova/authority.py` | modulare parametri FC pe praguri de altitudine | cablat, dar **oprit** în proba de coborâre (`--no-authority`) |
| `nova/scoring.py` + `nova/frame_ring.py` | imaginile 8.3.3 | activ (ring implicit 30 cadre) |
| `nova/serial_guard.py` | cine ține `/dev/serial0` și camera; oprirea serviciilor | activ |
| `nova/preview.py` | fereastra OpenCV (fullscreen / scalată / oprită) | activ doar în monitor cu ecran |
| `nova/race_screen.py` | ecranul de concurs în terminal | doar cu `--race` (nefolosit în probe) |
| `nova/ekf_source.py` | 15.2.5: comutarea surselor EKF fără GNSS | **necablat** în `nova_pi.py` (element 36) |
| `nova/fence.py` | 15.2.4: geofence de 10 m prin protocolul de misiune | **necablat** nicăieri (element 35) |
| `nova/sim_truth.py` | adevărul din Gazebo | doar simulare |

Concluzie pentru consolidare: pe dronă rulează **13 module din 17**, dintre
care două (`ekf_source`, `fence`) sunt implementate, testate și
nefolosite. Ele sunt cerințe de regulament, nu cod mort — trebuie cablate,
nu șterse.

---

## 2. Modulele `nova/`

| fișier | linii | responsabilitate | folosit de | observații |
|---|---|---|---|---|
| `detector_pi.py` | 1 327 | **patru lucruri**: `CameraCalibration` (YAML), `ArucoMarkerDetector` (OpenCV), sursele de cadre (`PiCameraSource`, `GazeboFrameSource`, `ImageDirSource`, `ArraySource`), `PiDetector` (fir + instrumentare) + `build_pi_detector` | nova_pi, nova_sim, service, run_e2, calibrate, compare, gz_frames, check_handover_fov | cel mai mare fișier; candidat evident la spargere în 3–4 module. `GazeboFrameSource` e cod de simulare într-un modul de bord |
| `state_machine.py` | 773 | mașina de stări + `SequenceConfig` + `run_loop` + emisia MAVLink | nova_pi, nova_sim, fake_detector | `run_loop` e duplicat funcțional în `nova_sim.py` (ceas de simulare) |
| `authority.py` | 600 | modularea `WP_ACC`, `PSC_*`, `WP_SPD_DN`, `LAND_SPD_MS` cu salvare/restaurare | nova_pi, nova_sim | oprit în probe; păstrat pentru cursă |
| `safety.py` | 580 | supervizorul + zăvor + confirmare de mod | nova_pi, nova_sim | pragurile în capul fișierului = Safety Case |
| `vehicle.py` | 542 | legătura MAVLink | toate aplicațiile | `nova_sim.py` îl derivă (`SimVehicle`) doar pentru ceas |
| `sim_truth.py` | 384 | adevăr Gazebo (ENU → NED, cadrul corpului) | nova_sim, batch_sim | nu are ce căuta pe dronă; candidat la `sim/` |
| `fence.py` | 294 | geofence prin misiune | **nimeni** | de cablat |
| `serial_guard.py` | 291 | port + cameră: cine le ține, oprirea serviciilor | nova_pi, bringup, descent_test | operațional, nu control |
| `rc.py` | 259 | `OverrideMonitor` | handover, safety, calibrate_sticks | |
| `race_screen.py` | 252 | ecranul de concurs (terminal) | nova_pi `--race` | nefolosit în probe |
| `ekf_source.py` | 233 | seturi EKF3 | nova_sim (doar `--no-gnss`) | de cablat pe bord |
| `handover.py` | 211 | poarta | nova_pi, nova_sim, fake_detector | |
| `detection.py` | 197 | `Detection`, `CameraModel`, `fill_from_corners` | tot | contractul central; bine izolat |
| `preview.py` | 197 | fereastra OpenCV | nova_pi, calibrate_camera, run_e2 | UI |
| `scoring.py` | 160 | `ScoringRecorder` | nova_pi, nova_sim | |
| `config.py` | 140 | `DEFAULTS`, `load`, E0, `autostart_mode` | tot | |
| `frame_ring.py` | 65 | ring de cadre | scoring, service | |

Observație: `nova/` amestecă trei straturi — **control** (state_machine,
safety, handover, rc, authority, vehicle), **viziune** (detection,
detector_pi, scoring, frame_ring) și **operațional/UI** (config,
serial_guard, preview, race_screen) — plus **simulare** (sim_truth,
GazeboFrameSource). Separarea pe directoare ar face lizibilă harta de mai
sus fără să schimbe codul.

---

## 3. Unelte pentru dronă (banc, instalare, întreținere)

Toate rulează **pe Pi** dacă nu scrie altfel.

| script | linii | ce face | când |
|---|---|---|---|
| `pi/setup_uart.sh` | 171 | PL011 pe GPIO 14/15, scoate consola serială, grupul `dialout` | o dată, la instalare (`sudo`) |
| `tools/setup_pi.sh` | 301 | venv `--system-site-packages`, pip, verificarea importurilor | o dată, la instalare |
| `pi/install.sh` | 147 | serviciul de utilizator + autostart-ul | o dată; din nou doar dacă se schimbă unitatea/.desktop |
| `pi/deploy.sh` | 119 | **de pe desktop**: rsync repo → Pi, chmod, opțional setup/check | la fiecare schimbare de cod |
| `pi/poza.sh` | 54 | **de pe desktop**: poză de pe cameră, rotită ca în detector | verificarea orientării |
| `pi/bringup.sh` | 304 | verificări la boot + decizia monitor/zbor + log de boot | automat, la fiecare boot |
| `pi/descent_test.sh` | 352 | verificări de zbor (E0, calibrare, abort, preflight) + `nova_pi.py` | automat (`--auto`) sau de mână |
| `tools/preflight_check.py` | 490 | stivă, calibrare, cameră (fps, contrast, controale), MAVLink, parametri | din descent_test și bringup |
| `tools/check_params.py` | 284 | citire înapoi a parametrilor FC; `--write --reboot` | preflight + de mână |
| `tools/make_parm_45.py` | 142 | **desktop**: `nova_flight.parm` (4.7+) → `nova_flight_4.5.parm` | la fiecare schimbare de parametri |
| `tools/calibrate_camera.py` | 570 | ChArUco/tablă → `config/camera_pi.yaml` | la schimbarea camerei/obiectivului |
| `tools/make_calib_target.py` | 312 | **desktop**: PNG de tipărit pentru calibrare | o dată |
| `tools/calibrate_sticks.py` | 163 | zgomotul manșelor → deadband recomandat | **încă nerulat** pe emițătorul real |
| `tools/check_rc_override.py` | 101 | FC-ul raportează înapoi `RC_CHANNELS`? | o dată per emițător |
| `tools/run_e2.py` | 491 | colectarea E2: stații de distanță, condiții de lumină | **încă nerulat** |
| `tools/verify_detection.py` | 286 | detector ArUco **independent** (fără `nova/`), pentru verificare încrucișată | E2 |
| `tools/compare_detectors.py` | 225 | rulează ambele detectoare pe același director | E2 |
| `tools/collect_session.py` | 393 | strânge `.bin`, loguri, cadre, manifest | după zbor |

Redundanțe vizibile aici:

- **Contorul de boot** e implementat de două ori, în `bringup.sh` și în
  `descent_test.sh` (același cod bash, copiat).
- `preflight_check.py` **importă dintr-un alt script** (`nova_service.py`)
  funcția `check_calibration`. Un script care importă din alt script e
  semnul că funcția aparține bibliotecii (`nova/`).
- Verificarea calibrării apare în **trei** locuri: `descent_test.sh`
  (bloc Python inline), `bringup.sh` (bloc Python inline) și
  `preflight_check.py`. Toate citesc același YAML cu același prag.

---

## 4. Căi alternative de pornire — aici e cea mai mare confuzie

Există **cinci** moduri de a porni companion-ul pe Pi, scrise în runde
diferite:

| cale | ce pornește | stare |
|---|---|---|
| `pi/bringup.sh` (automat) | `nova_pi.py --monitor` sau `descent_test.sh --auto` | **curentă** |
| `pi/descent_test.sh` (de mână) | `nova_pi.py` cu verificări și `ZBOR` tastat | **curentă** |
| `tools/start_flight.sh` | venv, port, **modifică E0 în `nova.json`**, apoi `race_mode.py` | runda 5; nefolosit de la `pi/` încoace; contrazice regula „E0 doar din fișier, cu commit” |
| `tools/race_mode.py` | `nova_pi.py --race` (ecranul de concurs) | 42 de linii, lansator subțire; nefolosit în probe |
| `tools/nova_service.py` + `systemd/nova-monitor.service` | RACE_MONITOR ca serviciu **de sistem**, cu `ReadOnlyVehicle` | runda 5; înlocuit funcțional de `nova_pi.py --monitor`; **nu poate rula simultan** cu `nova-bringup` (același port) |

Adică două implementări de „monitor fără comenzi” (`nova_service.py`, 555
de linii, și `nova_pi.py --monitor`) și două de „pornire de zbor cu
verificări” (`start_flight.sh`, `descent_test.sh`). Documentația
(`pi/README.md`, `docs/ZBOR_TEST_ATERIZARE.md`, `CLAUDE.md`) le
menționează pe toate, ceea ce e exact ce face codebase-ul greu de înțeles
independent.

Propunere concretă (§9): **un singur lansator**, `pi/nova.sh`, cu moduri
`monitor | zbor | cursa`, și un singur set de verificări. `nova_service.py`
și `start_flight.sh` se retrag după ce `ReadOnlyVehicle` (bariera de
comandă, cu testul ei) e mutată în `nova/`.

---

## 5. Simulare — doar desktop

Nu ajung pe dronă și nu trebuie să ajungă. Astăzi stau amestecate în
`tools/` cu uneltele de Pi.

| script | linii | ce face |
|---|---|---|
| `start_sim.sh` | 251 | Gazebo + SITL + detector, cu `--gamepad`, `--wipe`, `check_params` automat |
| `tools/fake_detector.py` | 366 | detector sintetic din geometrie + aplicația de sim (ocolește E0 explicit) |
| `tools/nova_sim.py` | 865 | aplicația cu cadre din Gazebo: buclă proprie pe ceas de simulare, diagnostic de pierdere, CSV |
| `tools/sim_fly_to.py` | 267 | decolare + poziționare, partea „manuală” |
| `tools/sim_handover.py` | 287 | AUX prin `RC_CHANNELS_OVERRIDE`, fără gamepad |
| `tools/batch_sim.py` | 825 | campanii randomizate, rezumat p50/p95 |
| `tools/gamepad_rc.py` | 398 | DualSense → RC, cu calibrare după intenție |
| `tools/make_marker_model.py` | 418 | modelul Gazebo al markerului |
| `tools/make_camera_model.py` | 473 | senzorul de cameră din calibrare, fără gimbal |
| `tools/measure_rtf.py` | 359 | factorul de timp real |
| `tools/gz_frames.py` | 134 | verifică sursa de cadre Gazebo |
| `tools/check_handover_fov.py` | 113 | încape markerul în cadru la handover? (sintetic) |
| `tools/setup_sim_venv.sh` | 140 | al treilea venv (gz-transport + OpenCV 4.10) |
| `tools/synthetic.py` | 201 | randare sintetică cu antialiasing pentru teste |
| `nova/sim_truth.py` | 384 | adevărul Gazebo |

`nova_sim.py` reimplementează bucla din `state_machine.run_loop` pentru că
ceasul trebuie să fie cel de simulare. Un `run_loop(clock=...)` ar elimina
al doilea cablaj și testul care compară ordinea apelurilor între ele.

---

## 6. Cod mort sau orfan

| ce | de ce |
|---|---|
| `tools/script_cristi.py` (150 linii, fără docstring) | originea lui `verify_detection.py`; nu e referit de nimic în afară de acel docstring |
| `tools/measure_detection.py` | **referit** în docstring-ul lui `detector_pi.py`, dar **nu există** |
| `tools/start_flight.sh` | vezi §4; modifică E0 din script, ceea ce regula proiectului interzice |
| `systemd/nova-monitor.service` + `nova_service.py` | vezi §4 |
| `config/nova_sitl.parm` vs `nova_flight.parm` vs `nova_flight_4.5.parm` | trei fișiere de parametri; al treilea e generat, primele două diferă printr-o singură linie (`FS_THR_ENABLE`) |
| `teste2209.txt` | fișier gol, urmărit de git |

---

## 7. Testele

18 suite, ~396 de teste, toate verzi la `dca5528`, rulate cu
`~/nova-sim-venv/bin/python`.

| suită | teste | ce acoperă |
|---|---|---|
| `test_state_machine` | 20 | secvența, poarta, cablajul aplicației |
| `test_safety` | 28 | supervizorul, zăvorul, 5 ratări |
| `test_handover` | 18 | poarta, E0, monitor |
| `test_detector_pi` | 25 | detectorul pe cadre sintetice, ROI, căutare redusă, expunere |
| `test_pi_tooling` | 53 | uneltele de Pi, **inclusiv textul sursă al scripturilor** |
| `test_ops` | 39 | port serial, previzualizare, `start_flight.sh` |
| `test_sim_loop` | 70 | `nova_sim.py`, `batch_sim.py`, adevărul |
| `test_authority` | 19 | modularea autorității |
| `test_link` | 17 | reconectare, monitor de link |
| `test_calibrate_camera` | 14 | calibrarea pe randări |
| `test_camera_model` | 17 | senzorul Gazebo |
| `test_marker_model` | 12 | modelul markerului |
| `test_make_calib_target` | 13 | ținta de calibrare |
| `test_compare_detectors` | 8 | verificarea încrucișată |
| `test_ekf_source` | 11 | seturi EKF |
| `test_gz_source` | 11 | sursa Gazebo |
| `test_scoring` | 10 | imaginile 8.3.3 |
| `test_sim_handover` | 11 | AUX în SITL |

Nu există un runner; suita se rulează cu un `for` peste `tools/test_*.py`.
Un `tools/run_tests.py` (sau `pytest`) ar fi primul pas ieftin.

---

## 8. Constatări pentru consolidare, în ordinea impactului

1. **Cinci căi de pornire, două monitoare** (§4). Cea mai mare sursă de
   confuzie și de bug-uri de cablaj (§5.14 din CLAUDE.md s-a repetat de
   trei ori tocmai din cauza asta).
2. **`detector_pi.py` face patru lucruri** (§2). Sursele de cadre, în
   special cea din Gazebo, nu au ce căuta în modulul care zboară.
3. **Simularea locuiește în `tools/` alături de uneltele de Pi.** Un
   `deploy.sh` trimite totul pe dronă, inclusiv 4 000 de linii de cod
   Gazebo care nu rulează acolo.
4. **Verificări duplicate**: calibrarea în trei locuri, contorul de boot în
   două, bucla principală în două (`run_loop` / `nova_sim`).
5. **Scripturi care importă din scripturi**: `preflight_check` ←
   `nova_service`; `test_*` ← aproape toate. Ce e importat e bibliotecă și
   trebuie să stea în `nova/`.
6. **Două module de regulament necablate** (`fence`, `ekf_source`).
   Consolidarea e momentul să fie cablate, nu mutate.
7. **Trei fișiere de parametri** pentru două firmware-uri; `make_parm_45.py`
   e singurul lucru care le ține sincronizate.
8. **Cod mort** (§6): trei fișiere de șters, o referință de reparat.
9. **Testele pe text sursă** (`test_pi_tooling`, `test_ops`): ~30 de teste
   fac `grep` pe scripturi. Sunt utile ca gărzi de cablaj, dar orice
   redenumire le rupe. La consolidare, fiecare mutare cere actualizarea
   lor în același commit.
10. **Limba comentariilor**: CLAUDE.md cere de acum comentarii în engleză.
    Tot codul existent e în română. Regula practică: fiecare fișier atins
    de consolidare se traduce integral atunci; nu se lasă fișiere bilingve.

---

## 9. O țintă posibilă (propunere, nu decizie)

```
nova/
  vision/     detection.py, aruco.py (din detector_pi), calibration.py,
              sources/pi_camera.py, sources/image_dir.py, scoring.py, frame_ring.py
  control/    state_machine.py, safety.py, handover.py, rc.py, authority.py,
              vehicle.py, fence.py, ekf_source.py
  ops/        config.py, serial_guard.py, preview.py, race_screen.py,
              checks.py (calibrare, cameră, parametri — un singur loc)
sim/
  truth.py, gazebo_source.py, fake_detector.py, nova_sim.py, batch_sim.py,
  sim_fly_to.py, sim_handover.py, gamepad_rc.py, make_*_model.py, measure_rtf.py
pi/
  nova.sh     un lansator: monitor | zbor | cursa
  setup_uart.sh, install.sh, deploy.sh, poza.sh, nova-bringup.{service,desktop}
tools/
  calibrate_camera.py, calibrate_sticks.py, check_params.py, check_rc_override.py,
  run_e2.py, verify_detection.py, compare_detectors.py, collect_session.py,
  make_parm_45.py, make_calib_target.py, run_tests.py
tests/
  test_*.py
```

Principii care ar rezolva punctele din §8 fără să atingă logica validată:

- **un singur lansator** pe Pi, cu moduri, și un singur modul de verificări;
- **`deploy.sh` trimite doar `nova/`, `pi/`, `tools/`, `config/`** — nu `sim/` și nu `tests/`;
- **nimic din `tools/` nu e importat de altceva** decât de teste;
- **`run_loop(clock=...)`** — o singură buclă, două ceasuri;
- fiecare mutare vine cu testele ei actualizate în același commit, iar
  suita rămâne verde la fiecare pas.

Ordinea recomandată, ca riscul să rămână mic: întâi codul mort (§6), apoi
lansatorul unic (§4), apoi spargerea lui `detector_pi.py`, apoi mutarea
simulării. Cablarea lui `fence` și `ekf_source` e un pas separat, cu
propriile zboruri de validare.
