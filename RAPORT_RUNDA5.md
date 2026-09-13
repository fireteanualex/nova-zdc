# Raport — runda 5: pregătirea Pi-ului pentru E2

Stare: **G1 ✅ · G2 ✅ · G3 ✅ · G4 ✅**

`autonomy_enabled` a rămas `false`. Nimic din ce s-a construit nu poate
comanda vehiculul — vezi „Bariera de comandă" mai jos, care e ceva mai tare
decât „nu comandă".

> **Cel mai important lucru din raport, dacă citești doar unul:** Pi-ul **nu
> poate** rula aceeași versiune de OpenCV ca desktopul. E un conflict dur de
> dependențe, nu o preferință. Am măsurat ce înseamnă asta: pe markerul de
> misiune, nimic (0.023 px); pe ținta de calibrare, o diferență reală care
> invalida un test din runda 4. Detalii în §5.24 și mai jos.

---

## Ce s-a construit

| Fișier | Ce face |
|---|---|
| `requirements-pi.txt` | dependențe pip fixate; **fără** picamera2 și **fără** numpy, ambele din apt |
| `tools/setup_pi.sh` | instalare completă, cu gardă de platformă și verificare finală a importurilor |
| `tools/nova_service.py` | serviciul RACE_MONITOR + generarea unității systemd |
| `systemd/nova-monitor.service` | unitatea, generată cu căile mașinii curente |
| `tools/run_e2.py` | colectarea interactivă a datelor E2 |
| `tools/preflight_check.py` | verificare de banc, cod de ieșire ca poartă |
| `config/nova_flight.parm` | parametri ArduPilot pentru vehiculul real |
| `tools/test_pi_tooling.py` | 22 de teste offline pentru toate cele de mai sus |

---

## Conflictul care a decis G1

`opencv-contrib-python 5.0.0.93` — versiunea de pe desktop — declară
`numpy>=2; python_version >= "3.9"`. Raspberry Pi OS Bookworm are Python 3.11
și **numpy 1.24.2 din apt**, ca dependență a lui `python3-picamera2`.

Deci pe Pi, `pip install opencv-contrib-python==5.0.0.93` într-un venv cu
`--system-site-packages` ar instala numpy 2.x **în venv**, peste cel din apt.
`simplejpeg` și restul stivei picamera2 sunt compilate pentru ABI-ul
numpy 1.x și se rup — iar eroarea apare la `import picamera2`, departe de
cauză.

Nu există variantă în care ambele să meargă. Am ales OpenCV **4.10.0.84**,
care cere `numpy>=1.21` și se mulțumește cu cel din apt.

### Ce costă alegerea, măsurat

Am construit un venv care imită stiva Bookworm (numpy 1.24.2 +
OpenCV 4.10.0.84) și am rulat pe el **toată suita** și o comparație directă
de detecție.

**1. Pe markerul de misiune, diferența e neglijabilă.** Aceleași cadre
sintetice, ambele versiuni:

| distanță | colțuri | distanță | unghiuri |
|---|---|---|---|
| 1.0 m | 0.023 px | 0.0002% | 0.024° |
| 2.5 m | 0.012 px | 0.0050% | 0.001° |
| 5.0 m | 0.005 px | 0.0005% | 0.000° |
| 12.0 m | 0.004 px | 0.0059% | 0.000° |

Praguri E1: 1 px, 2%, 0.3°. Suntem cu **1–2 ordine de mărime** sub. Cifrele
validate pe desktop în rundele 3–4 se transferă pe Pi.

**2. Pe ținta de calibrare, NU e neglijabilă — și a invalidat un test.**
Runda 4 raporta, ca dovadă a separării de dicționare:

> *Dicționar ≠ `DICT_4X4_50`: 0 detecții în `DICT_4X4_50`*

Adevărat pe OpenCV 5.0. Pe OpenCV 4.10, **aceeași țintă produce 5 detecții
false pe 36 de cadre**, toate cu ID 48.

| | OpenCV 5.0 | OpenCV 4.10 |
|---|---|---|
| detecții false în `DICT_4X4_50` | 0 / 36 | **5 / 36** (ID 48) |
| ID 26 (markerul de misiune) | 0 | **0** |

Deci afirmația din runda 4 era adevărată doar pe versiunea de pe desktop —
exact versiunea pe care **nu** o rulează vehiculul. Ce ne protejează în
realitate, pe ambele versiuni, e **filtrul de ID** din `ArucoMarkerDetector`,
nu alegerea dicționarului. Alegerea dicționarului rămâne corectă, dar ca
strat secundar.

Testul e rescris: verifică acum că ID 26 nu apare niciodată (pe 24 de
cadre, distanțe și rotații variate) și **raportează** numărul de detecții
false cu alte ID-uri ca măsurătoare, nu ca eșec. Trece pe ambele versiuni și
spune adevărul pe fiecare.

**Asta e a patra oară când un test „trecea" fără să măsoare ce credeam**
(§5.11, §5.18, §5.20). De data asta cauza e nouă: testul măsura corect, dar
**pe altă platformă decât cea de producție.**

---

## G1 — `requirements-pi.txt` + `tools/setup_pi.sh`

Ordinea pașilor nu e arbitrară și e verificată de un test:

```
1. gardă de platformă     Bookworm; pe Bullseye stiva picamera2 diferă
2. apt                    python3-picamera2, python3-libcamera (+ numpy)
3. venv --system-site-packages    fără flag, pasul 2 devine invizibil
4. pip                    requirements-pi.txt, versiuni fixate
5. verificarea importurilor DIN VENV
```

Pasul 5 există pentru că pașii 1–4 pot toți să „reușească" și sistemul să fie
totuși nefuncțional. E §5.10 aplicat instalării.

### Ce verifică pasul 5, și de ce fiecare

| Verificare | De ce |
|---|---|
| `import picamera2` | dacă venv-ul e greșit, aici se vede |
| `import libcamera` + `AfModeEnum.Manual` | fără el nu se poate bloca focus-ul (§2) |
| `cv2.aruco.ArucoDetector` construibil | pachetul `opencv-python` simplu nu are `aruco` |
| `pymavlink` + `LANDING_TARGET` | mesajul de care depinde tot segmentul autonom |
| **de unde vine numpy** | din venv = capcana de mai sus; din apt = corect |

Ultima e cea care contează. Diagnosticul e scris explicit în script, pentru
că simptomul (`_ARRAY_API not found`) nu seamănă deloc cu cauza.

### Rezultate

| Verificare | Măsurat |
|---|---|
| Bookworm → continuă | exit 0 |
| Bullseye → refuz cu motiv | exit 1, „nume de cod 'bullseye'" |
| Ubuntu → refuz cu motiv | exit 1, „distribuție ubuntu" |
| ordinea apt → venv → pip | verificată în `--dry-run` |
| venv existent fără `--system-site-packages` | **refuzat**, cu comanda de reparare |
| venv corect, existent | refolosit |
| **negativ:** verify pe venv fără picamera2 | exit 1, picamera2 raportat |
| **negativ:** numpy pus în venv | semnalat explicit, exit 1 |

Ultimele două există ca să dovedească faptul că verificarea măsoară ceva. Al
doilea e construit punând un `numpy.py` momeală în `site-packages`-ul unui
venv corect — nu se poate instala numpy real fără rețea, dar detecția
testată e exact aceeași.

---

## G2 — `tools/nova_service.py` + unitatea systemd

RACE_MONITOR din §8: **detector activ, ZERO comenzi, ring buffer.**

### Bariera de comandă — „nu poate", nu „nu o face"

Un serviciu care pornește la boot și are acces la `Vehicle` e la o singură
linie greșită distanță de a comanda un mod de zbor. Deci legătura e
împachetată în `ReadOnlyVehicle`.

**Prima variantă era o listă neagră și am ratat imediat `update_params()`** —
retrimite `PARAM_SET` pentru valorile neconfirmate, nu începe cu niciun
prefix de „comandă" și arată ca o metodă de întreținere. Cu listă neagră, o
metodă nouă în `Vehicle` e permisă până își amintește cineva să o interzică.

Rescris ca **listă albă**: 9 metode de citire permise, tot restul ridică
`PermissionError`. Inclusiv `m`, conexiunea mavutil brută — fără asta bariera
ar fi decorativă, pentru că `vehicle.m.mav.command_long_send(...)` ar ocoli-o
complet.

Un test citește sursa fiecărei metode permise și caută apeluri
`self.m.mav.*_send(` care nu sunt cereri. Rezultat: **0**.

Din același motiv, serviciul **nu instanțiază** mașina de stări, poarta de
handover sau supervizorul. Cu `autonomy_enabled=false` poarta ar refuza
oricum (E0), dar atunci singurul lucru între un serviciu pornit la boot și o
coborâre autonomă ar fi o valoare dintr-un JSON. Două bariere independente.

### Refuzul de pornire fără calibrare

`CameraCalibration.load(require_real=True)` prinde deja fișierul lipsă,
`n_images = 0` și RMS peste prag. Ce îi scapă: un fișier scris de mână care
*arată* ca o calibrare reală și conține focala geometrică.

Nu se poate verifica după **valoarea** focalei — o calibrare reală a acestui
obiectiv dă ~933 px, exact cât dă și formula geometrică. Semnătura care le
deosebește e **distorsiunea**: un obiectiv de 102° are `k1 ≈ −0.05`, iar
`geometric()` pune coeficienți identic zero, ceea ce nu apare niciodată
dintr-o calibrare reală.

| Fișier | Rezultat |
|---|---|
| calibrare reală (25 poze, RMS 0.105) | acceptată |
| lipsă | refuzată |
| sursă `geometric` | refuzată |
| **`n_images=25`, `rms=0.2`, dist toți zero** | **refuzată** ← cazul nou |
| RMS 1.4 px | refuzată |

### Ring buffer

Derivat din sursa de cadre printr-un decorator (`RingTapSource`), nu printr-o
modificare în `nova/detector_pi.py`, care e validat.

Ține și cadrele în care markerul **nu** a fost văzut — verificat de test.
Sub 0.38 m markerul nu mai încape în cadru (§5.2), iar 8.3.3 cere exact
cadrul de la contact, deci un buffer care ar păstra doar detecțiile ar fi gol
fix când e nevoie de el.

Măsurat: 2.99 MB/cadru la 2304×1296, 30 de cadre = 1.0 s = 90 MB.

### Unitatea systemd

`systemd-analyze verify` a prins o greșeală reală: **`StartLimitIntervalSec`
și `StartLimitBurst` sunt chei de `[Unit]`, nu de `[Service]`.** Puse în
`[Service]` sunt ignorate **tăcut** — serviciul pornește și pare configurat.
Încă o instanță din §5.10.

Unitatea validează acum curat. E **implicit dezactivată**; comanda de
activare e scrisă în ea, cu condiția (după E2).

> Testul care verifică asta a avut și el un rezultat fals, prins la timp:
> tăia textul cu `text.split('[Service]')`, iar un **comentariu** din unitate
> conține literalul `[Service]`. Verificarea „nu e în `[Service]`" trecea din
> motivul greșit. Parsează acum pe linii ancorate.

---

## G3 — `tools/run_e2.py`

Rulat prin SSH, de lângă drona. La fiecare stație: cere distanța **măsurată
cu ruleta**, capturează N cadre, rulează detecția, afișează pe loc rata de
detecție, `marker_px` și eroarea.

**De ce pe loc:** o stație cu 30% rată de detecție înseamnă că ceva e greșit
— focus, expunere, marker murdar, ID greșit — și se vede în 5 secunde. Dacă
cifrele apar abia la procesarea de pe desktop, greșeala se descoperă după ce
scara a fost strânsă și lumina s-a schimbat.

### Validare pe adevăr sintetic

4 stații la distanțe cunoscute, 6 cadre fiecare:

| adevăr | detecție | `marker_px` | estimat | eroare |
|---|---|---|---|---|
| 10.0 m | 6/6 | 44 | 10.062 m | **+0.62%** |
| 5.0 m | 6/6 | 89 | 5.016 m | +0.31% |
| 2.0 m | 6/6 | 224 | 1.999 m | −0.03% |
| 1.0 m | 6/6 | 448 | — | < 0.1% |
| **0.30 m** | **0/5** | — | — | corect: sub pragul de încadrare |

`marker_px` 89 la 5 m și 448 la 1 m se potrivesc cu tabelul din §2 (90, 448).
Eroarea crește cu distanța, exact cum prezice §5.23.

### Patru decizii care nu sunt evidente

**Mediana, nu media.** O singură detecție parțial ratată trage media cu
procente întregi. Testat: 9 detecții bune la 5.0 m + una aberantă la 25 m →
mediana dă 5.000 m (eroare 0.00%), media ar da 7.0 m (**+40%**).

**Verdictul se inversează sub 0.38 m.** Acolo absența detecției e rezultatul
corect, iar prezența ei e bug. Testat în ambele sensuri: 0/5 sub prag = OK;
4/5 sub prag = PROBLEMĂ.

**Se scrie pe disc după FIECARE stație.** O sesiune pe teren se întrerupe —
bateria, un Ctrl-C, un cablu. Testat: `KeyboardInterrupt` la a treia stație
lasă primele două complete pe disc, cu manifest și cadre brute.

**ROI oprit.** Introduce dependență de ordinea cadrelor, iar o stație
trebuie să fie reproductibilă independent de cele dinainte.

Cadrele brute se păstrează întotdeauna, **mai ales** la stațiile care ies
prost: pe desktop se poate rula orice alt detector pe exact aceleași imagini.

Temperatura și starea de throttling (`vcgencmd get_throttled`) se
înregistrează la fiecare cadru. Pe Pi 4 fără radiator, throttling-ul la 80 °C
se manifestă ca scădere de FPS, iar fără măsurătoare arată identic cu un
detector lent.

---

## G4 — `tools/preflight_check.py` + `config/nova_flight.parm`

**Cod de ieșire 0 numai dacă toate verificările trec.** O verificare sărită
nu e o verificare trecută: `--no-mavlink` o scoate din listă și spune asta
explicit, dar nu o trece tăcut. Altfel ieșirea uneltei n-ar putea fi folosită
ca poartă.

| Verificare | Ce prinde |
|---|---|
| calibrare | lipsă / geometrică / distorsiune zero / RMS mare |
| rezoluție | calibrare pentru altă rezoluție → distanțe greșite, tăcut |
| cameră | **rata** de cadre, nu doar că se deschide |
| imagine | cadre uniforme — capac pe obiectiv |
| controale | §5.10 aplicat camerei: citite înapoi din metadate |
| mavlink | heartbeat |
| parametri | `check_params.py` pe `nova_flight.parm` |

Cele două cazuri negative merită subliniate, pentru că **trec orice test de
tip „camera se deschide"**: o cameră care dă 16 fps în loc de 30 (prinsă), și
un obiectiv acoperit care produce cadre perfect valide, la rată perfectă,
complet negre (prins prin deviația standard).

### `config/nova_flight.parm`

Nu exista; era referit din `nova_sitl.parm` ca perechea de zbor. Derivat din
fișierul validat în SITL, cu **o singură diferență funcțională**, verificată
de test:

```
FS_THR_ENABLE    SITL 0  →  zbor 1
```

În SITL e 0 pentru că puntea de gamepad ține canalele vii prin
`RC_CHANNELS_OVERRIDE` și orice pauză ar arăta ca pierdere de emițător. Pe
vehiculul real, pierderea emițătorului e exact evenimentul pentru care există
failsafe-ul.

19 parametri. **Neverificat pe hardware** — scris ca atare în fișier.

---

## Ce NU s-a putut testa fără Pi

Spun explicit, în loc să inventez teste care trec:

| Ce | De ce |
|---|---|
| **`setup_pi.sh` rulat efectiv** | `apt-get install python3-picamera2` cere Raspberry Pi OS. Am testat gărzile, ordinea și verificarea finală cu fixture-uri; **nu** am testat că pachetele chiar se instalează și că versiunile din requirements se rezolvă pe arm64. |
| **Camera reală** | FPS efectiv, controale chiar aplicate de driver, durata comutării de mod (§5.15), temperatura sub sarcină. Toate cifrele G3/G4 vin de la surse sintetice. |
| **Serviciul sub systemd** | unitatea validează cu `systemd-analyze verify`, dar nu a pornit niciodată un proces real. Restart la eșec, `ProtectSystem=strict` și `SupplementaryGroups` sunt **neverificate în execuție** — în special `ProtectSystem=strict`, care ar putea bloca scrieri pe care nu le-am anticipat. |
| **`/dev/serial0`** | existența, permisiunile (grupul `dialout`), baud 921600 stabil pe cablu real. |
| **`check_params.py` pe FC real** | prima rulare va fi și prima verificare a lui `nova_flight.parm`. |
| **Termica** | `vcgencmd` nu există pe desktop; funcția întoarce `None` și codul merge mai departe. Calea cu date reale e neexecutată. |
| **OpenCV 4.10 pe arm64** | verificarea de paritate a rulat pe x86-64, Python 3.10. Pi-ul e aarch64, Python 3.11. API-ul e același; codul generat nu. |

---

## Decizii deschise

**1. Serviciul nu poate prelua segmentul autonom** (marcat în cod).
Dacă vrei asta după E2, calea nu e să scoatem `ReadOnlyVehicle`, ci să
adăugăm un al doilea mod explicit (`--mode race`) care construiește cablajul
din `nova_pi.py`. Așa rămâne evident din linia de comandă ce rulează.

**2. Ring buffer: 30 de cadre (1.0 s, ~90 MB)** (marcat în cod).
8.3.3 cere cadrul de la contact, iar §4 arată derivă sub 1.2 cm între captură
și contact, deci 1 s ajunge cu marjă. Grupul C poate cere mai mult;
`--buffer-frames`.

**3. `FLTMODE_CH` și `FLTMODE1..6` lipsesc din `nova_flight.parm`,
deliberat.** Depind de emițătorul de concurs. Un parametru inventat care
trece `check_params` e mai rău decât unul lipsă, pentru că arată ca o
afirmație verificată în Compliance Matrix (§5.10). La fel `BATT_*`,
`MOT_SPOOL_TIM_DN`, reglajul PID și `SERIALn_*` — toate listate în fișier ca
decizii deschise, cu motivul.

**4. Pragul de FPS la preflight: 80% din nominal** (24 fps din 30).
Ales ca sub el bucla de detecție să nu mai țină pasul cu coborârea. Nemăsurat
pe Pi; de recalibrat după primele rulări reale.

**5. Utilizatorul din unitatea systemd** e cel de pe mașina unde se rulează
`--install-unit`. Bookworm nu mai are „pi" garantat.

---

## Ce trebuie să faci tu manual

```bash
# pe Pi, prima dată
git clone <repo> ~/nova-zdc && cd ~/nova-zdc
tools/setup_pi.sh                 # se oprește singur dacă nu e Bookworm

# calibrarea camerei (E1.2) — obligatorie înainte de orice
python3 tools/calibrate_camera.py --target charuco --square-mm <MĂSURAT>

# verificare de banc, înainte de fiecare sesiune
python3 tools/preflight_check.py            # cod 0 = se poate

# colectare E2
python3 tools/run_e2.py --lumina soare
rsync -av pi@nova:~/nova-zdc/data/e2/ ~/nova-zdc/data/e2/

# serviciul: instalat, dar NU activat
python3 tools/nova_service.py --install-unit
sudo cp systemd/nova-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl start nova-monitor
journalctl -u nova-monitor -f            # verifică că pornește curat
```

`systemctl enable` se dă **după** ce E2 trece criteriile, împreună cu
commit-ul care pune `autonomy_enabled` pe `true`.

---

## Descoperiri adăugate la CLAUDE.md §5

| § | Ce |
|---|---|
| 5.24 | Pi-ul și desktopul nu pot rula aceeași versiune de OpenCV; ce diferă, măsurat |
| 5.25 | O barieră de siguranță se scrie ca listă albă; `update_params` era gaura |
| 5.26 | `systemd-analyze verify` prinde chei puse în secțiunea greșită — ignorate tăcut |
