%% STARTEN.m — Startcommando's voor de WIS Digital Twin
%
% Dit bestand bevat alle commando's om de digital twin en het dashboard
% op te starten. Kopieer de relevante regels naar de MATLAB Command Window.

%% ── 0. EENMALIGE SETUP (vóór eerste gebruik) ────────────────────────────
%
%  0a. Q/R ruis-covariantie schatten uit echte sensordata.
%      Vereist: data/data.csv (kolommen: t_s, s1_cm..s7_cm, gates gesloten).
%      Resultaat → data/Q_R_estimated.mat (auto-geladen door twin_config.m).

cd(fileparts(which('digital_twin')))
schat_Q_R

%  0b. AEMF-filtermatrices bouwen voor lekkagedetectie.
%      Vereist: WIS-sim/simulation/distributed_workspace.mat.
%      Resultaat → data/wis_aemf_filter.mat (auto-geladen door digital_twin.m).
%      N.B. digital_twin.m genereert dit ook automatisch als het ontbreekt.

cd(fileparts(which('digital_twin')))
wis_aemf_filter_setup

%% ── 1. DIGITAL TWIN (simulator-modus) ───────────────────────────────────
%
%  Draait de twin volledig in software: geen hardware nodig.
%  Instellingen (horizon, setpoints, looptijd) staan in twin_config.m.
%  Logs worden opgeslagen in WIS-twin/data/.

cd(fileparts(which('digital_twin')))   % zorg dat je in de WIS-twin map zit
digital_twin

%% ── 2. DASHBOARD (web-interface) ────────────────────────────────────────
%
%  Open een terminal (bijv. Windows Terminal of de MATLAB Terminal) en
%  voer de onderstaande twee regels uit:
%
%    cd(fileparts(which('digital_twin')))
%    python -m http.server 8080
%
%  Open daarna in de browser:
%    http://localhost:8080/twin_dashboard.html
%
%  Het dashboard ververst automatisch elke 2 seconden en leest
%  data/twin_log.csv.

%% ── 3. HARDWARE-MODUS (Flask API) ───────────────────────────────────────
%
%  Zorg dat de Flask SCADA-app draait (leest Firefly-sensoren via USB):
%    cd /home/bep2026/Bep2026
%    python app.py         # poort 5000
%
%  Stel in twin_config.m in:
%    USE_HARDWARE  = true;
%    USE_FLASK_API = true;
%
%  Start daarna de digital twin vanuit MATLAB:
%    cd(fileparts(which('digital_twin')))
%    digital_twin
%
%  Sensordata: bassin 1 = sensor 2, bassin 2 = sensor 4, bassin 3 = sensor 6
%  Actuatoren: servo 0–255 (nodes 201/202/203) → Cantoni 0–0.5
%
%% ── 3b. HARDWARE-MODUS (directe seriële COM-poort, legacy) ─────────────
%
%  Activeer via FireflyCommunicationPSTC (PSTC-protocol, 115200 baud):
%    USE_HARDWARE = true; USE_FLASK_API = false;
%    fc = FireflyCommunicationPSTC(...);
%    fc.twin_active = true;
%    main_pstc
%
%  Let op: app.py en serial-modus kunnen NIET tegelijkertijd de Firefly-poort
%  gebruiken. Gebruik Flask API (3) of serieel (3b), niet beide.

%% ── 4. TESTS UITVOEREN ──────────────────────────────────────────────────
%
%  Verifieer de losse componenten:

cd(fileparts(which('digital_twin')))
test_twin_kalman
test_twin_mpc
test_twin_log
