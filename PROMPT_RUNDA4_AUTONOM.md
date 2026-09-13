# Sarcini NOVA — rundă 4 (rulare nesupravegheată)

Citește `CLAUDE.md` și `SARCINI_RUNDA4.md`. Execută F1, F2, F3 în ordine,
**fără să te oprești pentru confirmare**. Nu voi fi disponibil.

Fiecare unealtă are un test sintetic de tip dus-întors care o validează
fără hardware. Rulează-l, interpretează rezultatul, continuă. Dacă un
prag nu e atins, încearcă să diagnostichezi și să repari; dacă nu
reușești în două încercări, notează în raport și treci mai departe.

---

## Reguli pentru lucrul nesupravegheat

- `autonomy_enabled` rămâne **fals**. Testul E0 care verifică starea
  fișierului din repo trebuie să treacă în continuare.
- Nu atinge `nova/state_machine.py`, `nova/safety.py`, `nova/handover.py`,
  `nova/detector_pi.py`. Sunt validate, iar eu nu pot verifica o
  regresie.
- Commit după fiecare unealtă, cu mesaj descriptiv. Dacă ceva se strică,
  vreau să pot izola.
- Rulează suita completă (`tools/test_*.py`) după fiecare commit. Dacă un
  test care trecea începe să pice, **oprește-te** și scrie în raport ce
  ai schimbat ultima dată.
- Nu instala dependențe noi. Ai `opencv-contrib-python`, `numpy`,
  `pymavlink`, `pyyaml`.
- Scrie `RAPORT_RUNDA4.md` pe măsură ce lucrezi, nu la final. Dacă
  sesiunea se întrerupe, vreau să văd unde ai ajuns.

---

## Testele sintetice — cum arată validarea

Principiul: construiești o scenă cu adevăr cunoscut, o randezi prin
intrinseci cunoscuți, rulezi unealta pe imaginea rezultată, și compari cu
adevărul de la care ai plecat.

Ai aplicat deja tiparul ăsta la testele detectorului din E1. **Atenție la
capcana pe care ai găsit-o atunci** (§5.11): renderul prin homografie e
exact doar pentru colțurile pe care le-ai impus, nu pentru cele
interioare. Dacă randezi o tablă de șah, verifică întâi că renderul e
corect geometric — altfel testezi renderul, nu unealta.

### F1 — generator de ținte

| Verificare | Prag |
|---|---|
| Dimensiunile în px corespund cu mm × 300/25.4 | eroare 0 px |
| Marginea albă ≥ o lățime de pătrat pe toate laturile | măsurat din imagine |
| Grila implicită e asimetrică | 9×6 pătrate |
| ChArUco: markerii se decodează cu `CharucoDetector` | toți, ID-uri corecte |
| Dicționarul ChArUco nu e `DICT_4X4_50` | verificare explicită |
| Ținta prea mare pentru hârtie → eroare clară | nu produce fișier |

Test suplimentar: randează ținta generată prin intrinseci cunoscuți, la
o pose cunoscută, și confirmă că `findChessboardCornersSB` (respectiv
`CharucoDetector`) găsește **toate** colțurile interioare. Dacă
generatorul produce o țintă pe care detectorul n-o vede, e inutilă.

### F2 — calibrare

Ăsta e testul principal al rundei. Fără el, calibrarea e o cutie neagră.

1. Alege intrinseci de referință apropiați de camera reală: 2304×1296,
   `fx = fy = 933`, `cx = 1152`, `cy = 648`, `k1 = -0.05`, `k2 = 0.01`,
   restul zero.
2. Generează 24 de pose diverse: înclinări de ±20–40° pe ambele axe,
   rotații în plan, distanțe variate, ținta în toate cele nouă zone ale
   cadrului.
3. Randează ținta prin acei intrinseci, cu distorsiune aplicată.
4. Rulează calibrarea pe imaginile rezultate.
5. Compară cu adevărul:

| Parametru | Prag |
|---|---|
| `fx`, `fy` | sub 1% |
| `cx`, `cy` | sub 1% din lățime |
| `k1` | sub 10% |
| RMS raportat | sub 0.3 px |

**Caz negativ obligatoriu:** injectează într-o poză un colț deplasat cu
8 px și confirmă că filtrarea pe mediană o respinge, iar parametrii
rămân în praguri. Ăsta e exact bug-ul din §5.11; testul trebuie să-l
prindă.

**Al doilea caz negativ:** rulează cu doar 12 poze și confirmă refuzul.
Și cu 24 de poze toate din aceeași pose și confirmă fie refuzul, fie un
RMS mic cu parametri în afara pragurilor — calibrare degenerată. Dacă
trece cu parametri greșiți și RMS mic, e un mod de eșec periculos și
merită notat în raport.

### F3 — verificare încrucișată

Randează markerul ID 26 de 480 mm prin aceiași intrinseci, la 15 pose
cunoscute acoperind 2–15 m și toate zonele cadrului. Rulează ambele
implementări.

| Verificare | Prag |
|---|---|
| Diferență pe colțuri, între implementări | max 1 px |
| Diferență relativă pe distanță | sub 1% |
| Fiecare față de adevărul sintetic: distanță | sub 2% |
| Fiecare față de adevăr: unghiuri | sub 0.3° |
| Acord pe detectat/nedetectat | 100% |

Dacă cele două diverg peste prag, **nu presupune care are dreptate.**
Compară fiecare cu adevărul sintetic și raportează care se abate.

---

## Ce scrii în raport

Pentru fiecare unealtă:
- ce ai construit, în două-trei propoziții
- tabelul de rezultate față de praguri, cu cifre reale
- ce ai reparat pe parcurs și de ce
- ce nu s-a putut testa fără hardware

La final, o secțiune scurtă:
- ce rămâne deschis
- ce trebuie să fac eu manual când mă întorc (tipar, măsurători, poze)
- orice decizie pe care ai luat-o și ai fi vrut s-o confirmi cu mine

Dacă găsești ceva care contrazice `CLAUDE.md`, scrie-l explicit în raport
și actualizează §5. Nu-l lăsa doar în istoricul conversației.
