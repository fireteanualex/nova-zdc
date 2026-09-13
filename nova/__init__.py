"""
NOVA - ZDC 2026, cod companion.

Separarea pe module urmareste paritatea sim <-> hardware (sectiunea 8 din
CLAUDE.md): acelasi cod pe desktop si pe Raspberry Pi, cu o singura
diferenta - de unde vin detectiile si pe ce link se vorbeste cu FC-ul.

    detection.py     ce ESTE o detectie (contractul intre viziune si control)
                     + modelul de camera comun detectorului sintetic si celui real
    vehicle.py       legatura cu ArduPilot: telemetrie + comenzi
    state_machine.py masina de stari a segmentului autonom (15.2.7)

Detectorul (sintetic in tools/fake_detector.py, ArUco real in Faza 2)
PUBLICA doar detectii. Nu comanda vehiculul si nu cunoaste starile.
"""
