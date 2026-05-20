# CLAUDE.md — WIS Digital Twin (BEP)

## Project overview

Bachelor Eindproject (BEP): een MATLAB-gebaseerde digitale tweeling voor een WIS (Water Informatie Systeem) laboratoriumopstelling met drie verbonden waterbassins. Het systeem combineert een Kalman-filter, MPC-regelaar en een op Cantoni gebaseerde gedistribueerde H∞-controller.

## Mapstructuur

```
Digital Twin/
├── WIS-twin/          # Digitale tweeling (hoofdcode)
│   ├── digital_twin.m          # Hoofdlus (Kalman + MPC, 1 Hz)
│   ├── twin_config.m           # Alle configuratieparameters
│   ├── twin_kalman_update.m    # Discrete Kalman-filterupdate
│   ├── twin_mpc_solve.m        # MPC QP-oplossing (N=10)
│   ├── kalman_smoother.m       # Rauch-Tung-Striebel smoother
│   ├── schat_Q_R.m             # EM-algoritme voor Q/R schatting
│   ├── twin_update_hardware.m  # Firefly hardware interface
│   ├── twin_plot_init.m        # Live plot initialisatie
│   ├── twin_plot_update.m      # Live plot update
│   ├── twin_log_write.m        # CSV logging (14 kolommen)
│   ├── STARTEN.m               # Opstartscript
│   ├── data/
│   │   ├── Q_R_estimated.mat   # EM-geschatte Kalman covarianties
│   │   └── twin_log.csv        # Meest recente log (symlink-achtig)
│   ├── technische_documentatie.html      # Hoofddocumentatie (NL)
│   └── ruis_covariantie_schatting.html  # Q/R schatting documentatie (NL)
├── WIS-sim/           # Simulatietools en controllerontwerp
│   ├── simulation/
│   │   ├── cantoni_LMI.m       # H∞ LMI-ontwerp via CVX → distributed_workspace.mat
│   │   ├── model_hil.m         # Hardware-in-the-loop model
│   │   ├── lab_setup_values.m  # Cantoni gewichten voor 3-basin opstelling
│   │   └── distributed_workspace.mat  # Gecombineerde plant + controller matrices
│   ├── pstc/                   # PSTC (Periodic/Sensor-based Triggering Control)
│   ├── identification/         # Systeemidentificatie
│   └── functions/              # Hulpfuncties
└── WIS-com/           # Python scripts voor hardware
    ├── main.py                 # Hoofdscript seriële communicatie
    ├── log_terminal.py         # Serieel loggen Firefly-nodes
    └── wis_simulation.py       # Hardware-in-the-loop Python-kant
```

## Systeemarchitectuur

- **Plant**: 3 waterbassins in serie, elk 4 toestanden → **12 toestanden totaal**
  - Toestanden per bassin: waterstand, debietdynamica, sluispositie, sluisaktuator
  - 3 ingangen: sluisopeningen (0–255 servo-eenheden)
  - 3 uitgangen: waterstandmeting per bassin [m]
- **Controller**: Cantoni gedistribueerde H∞-controller (Li & Cantoni 2008), LMI via CVX, γ²=50
- **Kalman Q-matrix**: 12×12 (niet 3×3 — alleen posities 1,5,9 op de diagonaal zijn waterstandtoestanden)
- **Kalman R-matrix**: 3×3 (meetruis per sensor)
- **MPC-horizon**: N=10, Q_mpc=10·I₃, R_mpc=0.1·I₃
- **Bemonsteringsfrequentie**: 1 Hz (Cantoni Tustin-discretisatie op 1/60 s is apart)
- **Setpoints**: y_ref = [0.25; 0.20; 0.15] m

## Afhankelijkheden

- **MATLAB** + Optimization Toolbox (voor `quadprog` in MPC)
- **CVX** (alleen nodig bij herberekening Cantoni-controller via `cantoni_LMI.m`)
- **Python 3** + `http.server` (voor HTML-dashboard op localhost)

## Opstarten

```matlab
% Vanuit WIS-twin/ in MATLAB:
run('STARTEN.m')      % of
run('digital_twin.m')
```

Voor het dashboard: start een Python HTTP-server in `WIS-twin/`:
```bash
python -m http.server 8000
```
Daarna open `http://localhost:8000/twin_dashboard.html`.

## Configuratie (twin_config.m)

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `USE_HARDWARE` | `false` | `true` = Firefly serieel, `false` = interne simulatie |
| `USE_ESTIMATED_QR` | auto | Laadt `Q_R_estimated.mat` als die bestaat |
| `H_LOOP` | `0` | `0` = zo snel mogelijk, `1` = real-time 1 Hz |
| `MAX_STEPS` | `60` | Aantal iteraties hoofdlus |
| `PLOT_LIVE` | `true` | Live MATLAB-plot |
| `WEB_DASH` | `true` | CSV-output voor HTML-dashboard |

## Werkstroom Q/R schatting

1. Draai `digital_twin.m` om een log te genereren in `WIS-twin/data/`
2. Draai `schat_Q_R.m` — dit leest de log en voert het EM-algoritme uit
3. Resultaat wordt opgeslagen in `data/Q_R_estimated.mat`
4. Volgende run van `digital_twin.m` laadt automatisch de geschatte waarden

## Sparse Q-structuur

Q is geforceerd spaars: alleen de diagonaalelementen op posities {1, 5, 9} (waterstandtoestanden) worden geschat via het EM-algoritme. De overige elementen blijven nul. Dit is bewust — controllertoestanden zijn niet direct observeerbaar via waterstands­sensoren.

## Belangrijke conventies

- Alle MATLAB-functies in `WIS-twin/` zijn scripts of functies met overeenkomende bestandsnamen
- `distributed_workspace.mat` wordt gegenereerd door `cantoni_LMI.m` en **niet** handmatig aanpassen
- Log-CSV heeft 14 kolommen: `epoch, y1, y2, y3, yp1, yp2, yp3, innov1, innov2, innov3, u1, u2, u3, triggered`
- Documentatie is Nederlandstalig (HTML-bestanden)
- Alarmadrempel dashboard: 0.02 m afwijking van setpoint

## Tests

```matlab
% Vanuit WIS-twin/:
run('test_twin_kalman.m')   % Kalman-filter unit tests
run('test_twin_mpc.m')      % MPC solver tests
run('test_twin_log.m')      % Logging tests
```
