# Lekkagemodel WIS-sim — Implementatieplan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Voeg een empirisch lekkagemodel toe aan de WIS-sim simulatie zodat de bypass-flow door gesloten sluizen correct wordt meegenomen.

**Architecture:** Een herbruikbare functie `wis_leakage.m` berekent de lekkageflow op basis van het hoogteverschil tussen twee bassins. De parameters worden in `wis_properties.m` opgeslagen. De functie wordt aangeroepen vanuit zowel `gate_simulation.m` (Simulink-pad) als `FireflySimulationPSTC.runSimulation()` (PSTC-pad).

**Tech Stack:** MATLAB — geen extra toolboxes vereist.

---

## Bestandsoverzicht

| Bestand | Actie |
|---------|-------|
| `WIS-sim/functions/wis_leakage.m` | Nieuw aanmaken |
| `WIS-sim/functions/test_wis_leakage.m` | Nieuw aanmaken |
| `WIS-sim/simulation/wis_properties.m` | Uitbreiden met leakage-parameters |
| `WIS-sim/simulation/gate_simulation.m` | Signatuur uitbreiden + lekkage toevoegen |
| `WIS-sim/simulation/test_gate_leakage.m` | Nieuw aanmaken |
| `WIS-sim/pstc/FireflySimulationPSTC.m` | Lekkagecorrectie toevoegen in `runSimulation()` |

---

## Task 1: `wis_leakage.m` — functie + unit tests

**Files:**
- Create: `WIS-sim/functions/wis_leakage.m`
- Create: `WIS-sim/functions/test_wis_leakage.m`

- [ ] **Stap 1: Schrijf de falende test**

Maak `WIS-sim/functions/test_wis_leakage.m` aan:

```matlab
% test_wis_leakage.m
% Run vanuit WIS-sim/functions/

% Test 1: normale stroming downstream (h1 > h2)
% dh = 5 cm, alpha=39.617, beta=0.328
% verwacht: q = (39.617*sqrt(5) + 0.328*5^1.5) / 1e6
%         = (88.547 + 3.669) / 1e6 = 9.2216e-5 m3/s
q = wis_leakage(0.25, 0.20, 39.617, 0.328);
assert(abs(q - 9.2216e-5) < 1e-8, 'Test 1 mislukt: verwacht 9.2216e-5');

% Test 2: gelijke peilen -> geen lekkage
q = wis_leakage(0.20, 0.20, 39.617, 0.328);
assert(q == 0, 'Test 2 mislukt: verwacht 0 bij gelijke peilen');

% Test 3: terugwaartse richting -> geen lekkage
q = wis_leakage(0.15, 0.20, 39.617, 0.328);
assert(q == 0, 'Test 3 mislukt: verwacht 0 bij h1 < h2');

% Test 4: sluis 2 parameters
% dh = 5 cm, alpha=9.402, beta=0.162
% q = (9.402*sqrt(5) + 0.162*5^1.5) / 1e6
%   = (21.015 + 1.812) / 1e6 = 2.2827e-5 m3/s
q = wis_leakage(0.25, 0.20, 9.402, 0.162);
assert(abs(q - 2.2827e-5) < 1e-8, 'Test 4 mislukt: sluis 2 waarden');

disp('Alle tests voor wis_leakage geslaagd.');
```

- [ ] **Stap 2: Voer de test uit — verwacht FOUT**

Open MATLAB, navigeer naar `WIS-sim/functions/`, voer uit:
```matlab
run('test_wis_leakage.m')
```
Verwacht: foutmelding `Undefined function 'wis_leakage'`.

- [ ] **Stap 3: Implementeer `wis_leakage.m`**

Maak `WIS-sim/functions/wis_leakage.m` aan:

```matlab
function q_m3s = wis_leakage(h1_m, h2_m, alpha, beta)
% wis_leakage  Bereken lekkageflow door een gesloten sluis.
%
%   q_m3s = wis_leakage(h1_m, h2_m, alpha, beta)
%
%   h1_m  : waterpeil upstream [m]
%   h2_m  : waterpeil downstream [m]
%   alpha : empirische constante (eenheden ingebakken)
%   beta  : empirische constante (eenheden ingebakken)
%   q_m3s : lekkageflow [m3/s], altijd >= 0

    dh_cm = (h1_m - h2_m) * 100;
    if dh_cm <= 0
        q_m3s = 0;
    else
        q_lek_cm3s = alpha * sqrt(dh_cm) + beta * dh_cm^(3/2);
        q_m3s = q_lek_cm3s / 1e6;
    end
end
```

- [ ] **Stap 4: Voer de test opnieuw uit — verwacht GESLAAGD**

```matlab
run('test_wis_leakage.m')
```
Verwacht: `Alle tests voor wis_leakage geslaagd.`

- [ ] **Stap 5: Commit**

```bash
git add WIS-sim/functions/wis_leakage.m WIS-sim/functions/test_wis_leakage.m
git commit -m "feat: add wis_leakage function with unit tests"
```

---

## Task 2: Leakage-parameters toevoegen aan `wis_properties.m`

**Files:**
- Modify: `WIS-sim/simulation/wis_properties.m`

- [ ] **Stap 1: Voeg de parameters toe**

Voeg onderaan `WIS-sim/simulation/wis_properties.m` toe:

```matlab
% Lekkageparameters per sluis (empirisch bepaald)
% Formule: q_lek [cm3/s] = alpha*sqrt(dh [cm]) + beta*dh^(3/2)
Wis.leak_alpha = [39.617, 9.402, 40.310]; % sluis 1, 2, 3
Wis.leak_beta  = [0.328,  0.162,  0.559];
Wis.h0         = 0.30;  % aanname pool 0 peil [m]
```

- [ ] **Stap 2: Verifieer in MATLAB**

```matlab
run('wis_properties.m')
disp(Wis.leak_alpha)   % verwacht: [39.617, 9.402, 40.310]
disp(Wis.leak_beta)    % verwacht: [0.328, 0.162, 0.559]
disp(Wis.h0)           % verwacht: 0.30
```

- [ ] **Stap 3: Commit**

```bash
git add WIS-sim/simulation/wis_properties.m
git commit -m "feat: add leakage parameters to wis_properties"
```

---

## Task 3: Lekkage integreren in `gate_simulation.m`

**Files:**
- Modify: `WIS-sim/simulation/gate_simulation.m`
- Create: `WIS-sim/simulation/test_gate_leakage.m`

- [ ] **Stap 1: Schrijf de falende test**

Maak `WIS-sim/simulation/test_gate_leakage.m` aan:

```matlab
% test_gate_leakage.m
% Run vanuit WIS-sim/simulation/
addpath('../functions');

% Test 1: gesloten sluis (flow_request=0, gate=0) zonder lekkage -> flow=0
[flow_no_leak, ~] = gate_simulation(0, 0.25, 0.20, 0, 0, 0);
assert(abs(flow_no_leak) < 1e-10, 'Test 1 mislukt: flow zonder lekkage moet 0 zijn');

% Test 2: gesloten sluis met lekkage -> flow > 0
% verwacht: flow = wis_leakage(0.25, 0.20, 39.617, 0.328) = 9.2216e-5 m3/s
[flow_with_leak, ~] = gate_simulation(0, 0.25, 0.20, 0, 39.617, 0.328);
assert(abs(flow_with_leak - 9.2216e-5) < 1e-8, ...
    sprintf('Test 2 mislukt: verwacht 9.2216e-5, kreeg %e', flow_with_leak));

% Test 3: achterwaartse compatibiliteit — aanroep zonder alpha/beta
[flow_default, ~] = gate_simulation(0, 0.25, 0.20, 0);
assert(abs(flow_default) < 1e-10, 'Test 3 mislukt: zonder alpha/beta moet flow 0 zijn');

disp('Alle tests voor gate_simulation lekkage geslaagd.');
```

- [ ] **Stap 2: Voer de test uit — verwacht FOUT**

```matlab
run('test_gate_leakage.m')
```
Verwacht: foutmelding bij Test 2 (lekkage nog niet geïmplementeerd).

- [ ] **Stap 3: Pas `gate_simulation.m` aan**

Vervang de functiedeclaratie en het einde van `WIS-sim/simulation/gate_simulation.m`:

```matlab
function [flow, next_gate] = gate_simulation(flow_request, h1, h2, current_gate, alpha, beta)
% flow_request requested flow (m^3/s)
% h1, h2 water level before and after gate (m)
% current_gate setting (0-255)
% alpha, beta leakage parameters (optional, default 0)
%
% flow actual flow (m^3/s)
% next_gate (0-255)

if nargin < 6, alpha = 0; beta = 0; end
```

En vervang de commentaarregel `% later: lekkage toevoegen` (huidige regel 71) door:

```matlab
flow = flow + wis_leakage(h1, h2, alpha, beta);
```

Zodat het einde van de functie er als volgt uitziet:

```matlab
    % calculate flow with actual gate setting
    flow = (temp_servo + 0) * (K * 1)  * sqrt(abs(delta_level)) * sign(delta_level);
    next_gate = temp_servo;

    flow = flow + wis_leakage(h1, h2, alpha, beta);

end
```

- [ ] **Stap 4: Voer de test opnieuw uit — verwacht GESLAAGD**

```matlab
run('test_gate_leakage.m')
```
Verwacht: `Alle tests voor gate_simulation lekkage geslaagd.`

- [ ] **Stap 5: Commit**

```bash
git add WIS-sim/simulation/gate_simulation.m WIS-sim/simulation/test_gate_leakage.m
git commit -m "feat: add leakage to gate_simulation via wis_leakage"
```

---

## Task 4: Lekkagecorrectie in `FireflySimulationPSTC.runSimulation()`

**Files:**
- Modify: `WIS-sim/pstc/FireflySimulationPSTC.m` (methode `runSimulation`, rond regel 468)

- [ ] **Stap 1: Schrijf een smoke-test script**

Maak `WIS-sim/pstc/test_simulation_leakage.m` aan:

```matlab
% test_simulation_leakage.m
% Verifieer dat lekkage de waterpeilen in de verwachte richting verandert.
% Run vanuit WIS-sim/pstc/
addpath('../functions');

% Stel Wis struct op met lekkageparameters
Wis.area1      = 0.1853;
Wis.area2      = 0.1187;
Wis.area3      = 0.2279;
Wis.h0         = 0.30;
Wis.leak_alpha = [39.617, 9.402, 40.310];
Wis.leak_beta  = [0.328,  0.162,  0.559];

% Begintoestand: [h1, pade1, h2, pade2, h3, pade3]
xp = [0.25; 0; 0.20; 0; 0.15; 0];

SPS    = 8;
h_step = 1;        % seconden per hoofdstap
dt_sub = h_step / SPS;

% Pas lekkagecorrectie toe voor 1 sub-stap (geen Apd/Bpd nodig voor deze test)
h0 = Wis.h0;
h1 = xp(1); h2 = xp(3); h3 = xp(5);

q1 = wis_leakage(h0, h1, Wis.leak_alpha(1), Wis.leak_beta(1));
q2 = wis_leakage(h1, h2, Wis.leak_alpha(2), Wis.leak_beta(2));
q3 = wis_leakage(h2, h3, Wis.leak_alpha(3), Wis.leak_beta(3));

xp(1) = xp(1) + (q1 - q2) * dt_sub / Wis.area1;
xp(3) = xp(3) + (q2 - q3) * dt_sub / Wis.area2;
xp(5) = xp(5) + q3        * dt_sub / Wis.area3;

% h1 ontvangt meer van pool 0 (q1) dan het verliest naar h2 (q2)
% q1=9.2216e-5, q2=2.2827e-5 -> netto instroom -> h1 moet stijgen
assert(xp(1) > 0.25, 'Test mislukt: h1 moet stijgen (q1 > q2)');

% h2 verliest meer naar h3 (q3) dan het ontvangt van h1 (q2)
% q2=2.2827e-5, q3=9.6396e-5 -> netto uitstroom -> h2 moet dalen
assert(xp(3) < 0.20, 'Test mislukt: h2 moet dalen (q3 > q2)');

% h3 ontvangt lekkage van h2 (q3 > 0) -> h3 moet stijgen
assert(xp(5) > 0.15, 'Test mislukt: h3 moet stijgen');

disp('Smoke-test lekkagecorrectie geslaagd.');
```

- [ ] **Stap 2: Voer de smoke-test uit — verwacht GESLAAGD**

(Test valideert de formule los van de klasse, zodat we zeker zijn van de logica.)

```matlab
run('test_simulation_leakage.m')
```
Verwacht: `Smoke-test lekkagecorrectie geslaagd.`

- [ ] **Stap 3: Voeg lekkagecorrectie toe aan `runSimulation()`**

Zoek in `WIS-sim/pstc/FireflySimulationPSTC.m` de lus `for j = 1:SPS+timingProblem` (rond regel 468). De huidige code:

```matlab
for j = 1:SPS+timingProblem
    xp = Apd*xp + Bpd*uhat/1000 + Epd * (kk*obj.h >= 20);
end
```

Vervang door:

```matlab
for j = 1:SPS+timingProblem
    xp = Apd*xp + Bpd*uhat/1000 + Epd * (kk*obj.h >= 20);

    % Lekkagecorrectie (niet-lineair, per sub-stap)
    dt_sub = obj.h / SPS;
    h0_val = obj.Wis.h0;
    q1 = wis_leakage(h0_val,  xp(1), obj.Wis.leak_alpha(1), obj.Wis.leak_beta(1));
    q2 = wis_leakage(xp(1),   xp(3), obj.Wis.leak_alpha(2), obj.Wis.leak_beta(2));
    q3 = wis_leakage(xp(3),   xp(5), obj.Wis.leak_alpha(3), obj.Wis.leak_beta(3));
    xp(1) = xp(1) + (q1 - q2) * dt_sub / obj.Wis.area1;
    xp(3) = xp(3) + (q2 - q3) * dt_sub / obj.Wis.area2;
    xp(5) = xp(5) + q3        * dt_sub / obj.Wis.area3;
end
```

**Let op:** `wis_leakage` moet op het MATLAB-pad staan. Voeg aan het begin van `main_pstc.m` toe (als dat nog niet bestaat):

```matlab
addpath(fullfile(fileparts(mfilename('fullpath')), '../functions'));
```

- [ ] **Stap 4: Voer de smoke-test opnieuw uit — verwacht GESLAAGD**

```matlab
run('test_simulation_leakage.m')
```
Verwacht: `Smoke-test lekkagecorrectie geslaagd.`

- [ ] **Stap 5: Commit**

```bash
git add WIS-sim/pstc/FireflySimulationPSTC.m WIS-sim/pstc/test_simulation_leakage.m WIS-sim/pstc/main_pstc.m
git commit -m "feat: add leakage correction to FireflySimulationPSTC.runSimulation"
```

---

## Eindcheck

Na alle taken: verifieer dat de drie testsuite's allemaal slagen:

```matlab
% Vanuit WIS-sim/functions/
run('test_wis_leakage.m')

% Vanuit WIS-sim/simulation/
run('test_gate_leakage.m')

% Vanuit WIS-sim/pstc/
run('test_simulation_leakage.m')
```
