# Formatul capturii de touchdown (8.3.3, 6.2.1.30)

Scris de Pi (nova/touchdown_capture.py), citit de PC (tools/fetch_scoring.py).
O captură e **completă** doar dacă are `MANIFEST.sha256` — manifestul se
scrie ultimul, atomic (fișier temporar, fsync, redenumire), după toate
celelalte fișiere.

## Structura pe Pi

    ~/nova-zdc/scoring/Handoff<n>-touchdown/
        Handoff<n>-touchdown.png            cadrul de touchdown: nerotit, nedecupat,
                                            nemodificat, rezoluția fluxului; color
                                            (BGR) sau gri
        Handoff<n>-touchdown_annotated.png  copie cu o cruce fină în centrul imaginii
        burst/<seq>_<t_ms>.jpg              seria din ring buffer, JPEG calitate 95
        meta.json
        MANIFEST.sha256                     ultimul

- `<n>`: 1, 2, 3... pe Pi, **peste porniri** (din 28.09.2026): continuă de la
  cel mai mare `Handoff<k>-touchdown` existent în `scoring/`, inclusiv unul
  incomplet. Cât timp `scoring/` e păstrat, un număr nu se refolosește.
- Data și sesiunea (`s<YYYYmmdd-HHMMSS>`, de la pornirea aplicației) nu mai
  sunt în cale, ci în `meta.json`.
- Formatul vechi (`<YYYYMMDD>/<sesiune>/attempt_<n>/`, `nova-scoring-1`) nu mai
  e scris și nu e citit de PC.

## MANIFEST.sha256

Format `sha256sum`: o linie pe fișier, `<hex sha256><2 spații><cale relativă>`,
căi cu `/` (ex. `burst/0007_123456789.jpg`), sortate. Conține **toate**
fișierele din `Handoff<n>-touchdown/` în afară de manifestul însuși (deci și
`meta.json`). Se verifică cu `sha256sum -c MANIFEST.sha256` din director.

## meta.json (`"format": "nova-scoring-2"`)

| Cheie | Tip | Ce e |
|---|---|---|
| `format` | str | `"nova-scoring-2"` |
| `handoff`, `name` | int, str | `<n>` și `Handoff<n>-touchdown`, ca în cale |
| `date`, `session`, `attempt` | str, str, int | data, sesiunea Pi și a câta captură din sesiune |
| `sync.t_capture` | float | timpul capturii cadrului principal, ceasul Pi (time.monotonic, s) |
| `sync.t_on_ground_rx` | float | când a primit Pi-ul `ON_GROUND` (același ceas) |
| `sync.fc_time_boot_ms_contact` | int/null | `time_boot_ms` al FC-ului la `ON_GROUND` (se aliniază cu logul DataFlash) |
| `sync.fc_time_unix_usec` | int/null | ora GPS/UTC de la FC (`SYSTEM_TIME`) la contact — **doar etichetare** |
| `sync.pi_time_utc` | str | ora Pi-ului la contact, ISO 8601 |
| `h_ref_m` | float | altitudinea barometrică (față de home) la contact |
| `last_detection.age_s` | float/null | vârsta ultimei detecții valide la contact |
| `last_detection.lateral_m` | float/null | offsetul ei lateral |
| `camera.preset`, `camera.sensor_mode`, `camera.scaler_crop`, `camera.calibration` | str, [w,h], [x,y,w,h], str | geometria |
| `camera.ExposureTime`, `camera.AnalogueGain`, `camera.LensPosition` | num/null | metadatele cadrului principal |
| `image.file`, `image.annotated`, `image.size`, `image.color` | str, str, [w,h], bool | |
| `center.class` | str | `"negru"` / `"alb"` / `"ambiguu"` — **doar informativ** |
| `center.mean`, `center.min`, `center.max`, `center.patch_px` | float, int, int, int | pe planul de gri, fereastra 5×5 din centru |
| `burst.files`, `burst.t_from`, `burst.t_to` | int, float, float | |

## Clasificarea centrului (informativă)

Fereastra 5×5 px centrată în `(w // 2, h // 2)`, pe gri (luminanța):
`negru` dacă media < 80 și max − min < 60; `alb` dacă media > 170 și
max − min < 60; altfel `ambiguu`. Decizia finală e a omului.

## Pe PC: `tools/fetch_scoring.py`

`list` / `fetch [--watch N] [--cursa NUME] [--fc-log X.BIN]` / `verify DIR`.
Copia verificată a fiecărei capturi stă în `data/scoring_pc/Handoff<n>-touchdown/`
(imaginea: `data/scoring_pc/Handoff<n>-touchdown/Handoff<n>-touchdown.png`),
identică octet cu octet cu cea de pe Pi (manifestul ei rămâne verificabil).
Dacă Pi-ul reîncepe numerotarea (`scoring/` golit), PC-ul compară sha256-ul
manifestului de pe Pi cu cel local și raportează conflictul, fără să
suprascrie. Pachetul de predare, câte unul pe sesiune Pi, e în
`data/scoring_pc/handover/<YYYYMMDD>_<cursa>/`, cu `README.txt` și
`MANIFEST_HANDOVER.sha256` peste tot (inclusiv logul FC, în `fc/`). `verify`
înlocuiește `sha256sum -c` pe Windows. O captură cu hash greșit merge în
`carantina/` și nu intră în pachet.
