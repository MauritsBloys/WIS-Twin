# Digital Twin WIS — Ontwerpspecificatie

**Datum:** 2026-04-29
**Project:** BEP Bas Boot — Water Infrastructure System (WIS)
**Doel:** Real-time monitoring digital twin met Kalman filter observer én MPC-gebaseerde predictieve regeling

---

## 1. Doel & scope

Een digital twin die het fysieke WIS-watersysteem (3 bassins, 3 poorten) live monitort én predictief bijstuurt. Het model loopt parallel aan de hardware of simulator. Een Kalman filter schat continu de systeemtoestand; een Model Predictive Controller (MPC) optimaliseert stuuringrepen over een tijdshorizon op basis van die schatting en stuurt corrigerende signalen terug naar de hardware.

**In scope:** monitoring, Kalman observer, MPC-gebaseerde predictieve regeling, MATLAB live plots, web dashboard.

**Buiten scope:** cloud-integratie, leren/adapteren van modelparameters online.

---

## 2. Architectuur

Vijf lagen:

```
① Data-laag         hardware (Firefly serial) of simulator (wis_simulation.py)
        ↓ y(k), u(k) @ 1 Hz
② Observer-laag     Kalman filter — predict + update elke tijdstap
        ↓ x̂(k), innovatie(k), P(k)
③ Regelaar-laag     MPC — optimaliseert u over horizon N op basis van x̂(k)
        ↓ u_mpc(k) → terug naar hardware/simulator
④ Log-laag          twin_log.csv — brug tussen MATLAB en web
        ↓
⑤ Visualisatie      MATLAB live plots  +  statische web dashboard
```

**Schakelaar:** `USE_HARDWARE` in `twin_config.m` bepaalt de databron.

---

## 3. Bestandsstructuur

```
Bas Boot/
├── WIS-com/                    ← ongewijzigd
├── WIS-sim/                    ← ongewijzigd
└── WIS-twin/                   ← nieuw
    ├── README.md
    ├── twin_config.m           ← configuratie (USE_HARDWARE, Q, R, N, setpoints, paden)
    ├── digital_twin.m          ← main loop: roept Kalman + MPC aan per tijdstap
    ├── twin_kalman.m           ← Kalman filter kern (predict + update)
    ├── twin_mpc.m              ← MPC solver (quadprog)
    ├── twin_log.m              ← CSV wegschrijven
    ├── twin_plot.m             ← MATLAB live plots
    ├── twin_dashboard.html     ← web dashboard (Chart.js, pollt CSV)
    └── data/
        └── twin_log.csv        ← symlink naar meest recente sessie
```

---

## 4. Bestaande code hergebruik

| Bestand | Aanpassing |
|---|---|
| `wis_simulation.py` | Geen — simulator-modus databron |
| `log_terminal.py` | Geen — hardware logging |
| `cantoni_LMI.m` | Geen — levert `comb_Pool_disc` matrices |
| `parse_serial_log.py` | Geen — optioneel voor post-hoc analyse |
| `FireflyCommunicationPSTC.m` | Klein: `twin_update()` aanroepen in callback, `u_mpc` terugsturen |

---

## 5. Kalman filter (`twin_kalman.m`)

**Toestandsmodel:** gebaseerd op `comb_Pool_disc` uit `cantoni_LMI.m`

```
A = comb_Pool_disc.A    % 6×6 toestandsovergangsmatrix
B = comb_Pool_disc.B    % 6×3 ingangsmatrix
C = comb_Pool_disc.C    % 3×6 uitgangsmatrix
```

**Predict-stap:**
```
x̂⁻(k) = A · x̂(k-1) + B · u(k)
P⁻(k)  = A · P · Aᵀ + Q_kal
```

**Update-stap:**
```
innovatie = y(k) - C · x̂⁻(k)
K         = P⁻ · Cᵀ · (C · P⁻ · Cᵀ + R_kal)⁻¹
x̂(k)     = x̂⁻(k) + K · innovatie
P(k)      = (I - K·C) · P⁻(k)
```

**Parameters:**
- `Q_kal = 1e-4 · I₆` — procesruis (afstembaar in `twin_config.m`)
- `R_kal = 1e-3 · I₃` — sensorruis (initieel uit kalibratiedata)
- Initieel: `x̂(0) = 0`, `P(0) = I₆`

---

## 6. Model Predictive Control (`twin_mpc.m`)

Op basis van `x̂(k)` optimaliseert de MPC een reeks stuuringrepen over N stappen zodat de waterpeilen naar de setpoints bewegen met minimale inspanning.

**Kostfunctie:**
```
min  Σᵢ₌₀ᴺ⁻¹ [ (C·x(k+i) - y_ref)ᵀ · Q_mpc · (C·x(k+i) - y_ref)  +  u(k+i)ᵀ · R_mpc · u(k+i) ]
 u

subject to:
  x(k+i+1) = A·x(k+i) + B·u(k+i)
  0 ≤ u(k+i) ≤ 255           % poortopening servo-instelling
  |Δu(k+i)| ≤ du_max         % maximale stapgrootte per tijdstap
  x(k|k) = x̂(k)              % beginconditie uit Kalman
```

**Solver:** `quadprog` (MATLAB Optimization Toolbox) — omzetten naar standaard QP-vorm.

**Output:** `u_mpc(k)` — eerste element van de geoptimaliseerde reeks (receding horizon principe).

---

## 7. Configuratie (`twin_config.m`)

```matlab
USE_HARDWARE = false;           % true = Firefly serial, false = simulator CSV

% Kalman
Q_kal = 1e-4 * eye(6);         % procesruis covariantie
R_kal = 1e-3 * eye(3);         % sensorruis covariantie

% MPC
N      = 10;                   % voorspellingshorizon [tijdstappen]
Q_mpc  = 10  * eye(3);         % gewicht op setpuntafwijking
R_mpc  = 0.1 * eye(3);         % gewicht op stuurinspanning
du_max = 20;                   % max. poortwijziging per tijdstap

y_ref      = [0.25; 0.20; 0.15];   % setpoints waterpeilen [m]
LOG_FILE   = 'data/twin_log.csv';
PLOT_LIVE  = true;
WEB_DASH   = true;

addpath('../WIS-sim/simulation');
addpath('../WIS-sim/functions');
addpath('../WIS-sim/functions_jacob');
```

---

## 8. Log-formaat (`twin_log.csv`)

```
epoch, y1_meas, y2_meas, y3_meas, y1_pred, y2_pred, y3_pred,
innov1, innov2, innov3, u_mpc1, u_mpc2, u_mpc3, triggered
```

Elke rij = één tijdstap (1 Hz). Bij elke nieuwe run wordt een nieuw bestand aangemaakt met sessie-timestamp (bijv. `twin_log_20260429_143021.csv`). Een symlink `twin_log.csv` wijst altijd naar de meest recente sessie zodat het dashboard geen aanpassing nodig heeft.

---

## 9. MATLAB live plots (`twin_plot.m`)

Vijf figuurvensters, live bijgewerkt via `drawnow` elke stap:

1. **Waterpeilen** — meting (blauw) vs. Kalman-voorspelling (oranje) per bassin
2. **Stuuringrepen** — u_mpc(k) per poort over tijd
3. **Innovatie** — afwijking vóór Kalman-correctie per bassin
4. **Kalman gain** — K diagonaal over tijd (convergentie zichtbaar)
5. **MPC horizon** — voorspelde trajectory over N stappen op huidige tijdstap

---

## 10. Web dashboard (`twin_dashboard.html`)

Geserveerd via een lokale Python HTTP-server (vereist vanwege CORS op `file://`):

```bash
# Uitvoeren vanuit WIS-twin/
python -m http.server 8080
# Open: http://localhost:8080/twin_dashboard.html
```

- Pollt `data/twin_log.csv` elke 2 seconden via JavaScript `fetch()`
- Grafieken via **Chart.js** (CDN, geen installatie)
- Toont: waterpeilen, MPC-stuuringrepen, innovatie, modus-badge (Hardware/Simulator)
- Afwijkingsalert wanneer `|innovatie| > drempelwaarde` (instelbaar in HTML)

---

## 11. Dataflow per tijdstap

```
Hardware/Simulator
    → y(k): waterpeilen [m]
    → twin_kalman.m: predict + update → x̂(k), innovatie(k)
    → twin_mpc.m:    optimaliseer over N stappen → u_mpc(k)
    → u_mpc(k) → FireflyCommunicationPSTC / simulator
    → twin_log.m:    wegschrijven naar data/twin_log_<timestamp>.csv
    → twin_plot.m:   MATLAB figuren updaten (drawnow)
    ← twin_dashboard.html: leest twin_log.csv elke 2s (browser)
```

---

## 12. Testplan

1. **Kalman convergentie** — simulator-modus, stabiel systeem. Controleer dat innovatie naar nul convergeert.
2. **Verstoring injecteren** — stap-verstoring in simulator (t=20s). Innovatie slaat uit, MPC corrigeert, systeem keert terug naar setpoint.
3. **Q/R afstemmen** — varieer `Q_kal` en `R_kal`, observeer effect op convergentiesnelheid vs. gladheid.
4. **MPC horizon** — varieer `N` (5, 10, 20), observeer trade-off tussen reactiesnelheid en rekentijd.
5. **MPC begrenzing** — test of `u_mpc` binnen `[0, 255]` blijft bij grote setpointafwijking.
6. **Hardware-modus** — `USE_HARDWARE = true`, echte Firefly-nodes. Controleer 1 Hz timing en geen dropouts.
7. **Web dashboard** — open naast lopende simulatie, controleer live update en afwijkingsalert.
