# Lekkagemodel WIS-sim — Ontwerpdocument

**Datum:** 2026-05-18  
**Project:** WIS Digital Twin (BEP)  
**Status:** Goedgekeurd

---

## Probleemstelling

De WIS-sim simulatie bevat geen lekkagemodel voor de sluizen. In de fysieke opstelling stroomt er altijd een kleine hoeveelheid water door de sluisafdichtingen, zelfs als de sluizen gesloten zijn. Dit veroorzaakt een afwijking tussen simulatie en realiteit. In `gate_simulation.m` staat al een placeholder: `% later: lekkage toevoegen`.

## Lekkageformule

Empirisch vastgesteld model:

```
q_lek [cm³/s] = α · √(h₁ - h₂ [cm]) + β · (h₁ - h₂ [cm])^(3/2)
```

- h₁, h₂: waterpeilen van aangrenzende bassins in **cm**
- q_lek: lekkageflow in **cm³/s**
- α, β: dimensieloze empirische constanten (impliciet eenheden ingebakken)
- Lekkage is nul als h₁ ≤ h₂ (geen terugwaartse stroom)

### Parameters per sluis

| Sluis | Verbinding           | α      | β     |
|-------|----------------------|--------|-------|
| 1     | Pool 0 → Bassin 1    | 39.617 | 0.328 |
| 2     | Bassin 1 → Bassin 2  | 9.402  | 0.162 |
| 3     | Bassin 2 → Bassin 3  | 40.310 | 0.559 |

## Topologie

```
Pool 0 --[Sluis 1]--> Bassin 1 --[Sluis 2]--> Bassin 2 --[Sluis 3]--> Bassin 3 --> downstream
```

Pool 0 wordt gemeten maar niet gesimuleerd; peil wordt als bekende invoer behandeld (standaard 0.30 m).

## Gekozen aanpak

**Optie C: aparte functie + integratie in beide simulatiepaden**

### Bestanden die worden aangepast/aangemaakt

| Bestand | Actie | Reden |
|---------|-------|-------|
| `WIS-sim/functions/wis_leakage.m` | Nieuw | Herbruikbare lekkagefunctie |
| `WIS-sim/simulation/wis_properties.m` | Update | Parameters toevoegen |
| `WIS-sim/simulation/gate_simulation.m` | Update | Bestaande placeholder invullen |
| `WIS-sim/pstc/FireflySimulationPSTC.m` | Update | Discrete simulatiepad |

---

## Componentontwerp

### 1. `wis_leakage.m`

```matlab
function q_m3s = wis_leakage(h1_m, h2_m, alpha, beta)
% h1_m, h2_m: waterpeilen [m]
% alpha, beta: empirische constanten
% q_m3s: lekkageflow [m³/s], altijd >= 0
    dh_cm = (h1_m - h2_m) * 100;
    if dh_cm <= 0
        q_m3s = 0;
    else
        q_lek_cm3s = alpha * sqrt(dh_cm) + beta * dh_cm^(3/2);
        q_m3s = q_lek_cm3s / 1e6;
    end
end
```

**Eenhedenconversie:** h [m] × 100 → [cm]; q [cm³/s] ÷ 10⁶ → [m³/s]

### 2. `wis_properties.m` — toe te voegen

```matlab
Wis.leak_alpha = [39.617, 9.402, 40.310]; % sluis 1, 2, 3
Wis.leak_beta  = [0.328,  0.162,  0.559];
Wis.h0         = 0.30;  % pool 0 peil [m] (was hardcoded op lijn 214 FireflySimulationPSTC)
```

### 3. `gate_simulation.m` — signatuurwijziging

Optionele parameters voor achterwaartse compatibiliteit:

```matlab
function [flow, next_gate] = gate_simulation(flow_request, h1, h2, current_gate, alpha, beta)
    if nargin < 6, alpha = 0; beta = 0; end
    % ... bestaande code ongewijzigd ...
    flow = flow + wis_leakage(h1, h2, alpha, beta);
end
```

### 4. `FireflySimulationPSTC.runSimulation()` — correctie per sub-stap

Na de discrete toestandsupdate `xp = Apd*xp + Bpd*...` binnen de `for j = 1:SPS` lus:

```matlab
dt_sub = obj.h / SPS;
h0 = Wis.h0;
h1 = xp(1); h2 = xp(3); h3 = xp(5);

q1 = wis_leakage(h0, h1, Wis.leak_alpha(1), Wis.leak_beta(1));
q2 = wis_leakage(h1, h2, Wis.leak_alpha(2), Wis.leak_beta(2));
q3 = wis_leakage(h2, h3, Wis.leak_alpha(3), Wis.leak_beta(3));

xp(1) = xp(1) + (q1 - q2) * dt_sub / Wis.area1;
xp(3) = xp(3) + (q2 - q3) * dt_sub / Wis.area2;
xp(5) = xp(5) + q3        * dt_sub / Wis.area3;
```

**Toestandsvector:** [h1, pade1, h2, pade2, h3, pade3] — lekkage raakt alleen indices 1, 3, 5.  
**Pade-toestanden** (indices 2, 4, 6) worden niet aangepast.

## Randgevallen

- `h1 <= h2`: q_lek = 0 (geen negatieve lekkage)
- Pool 0 onbekend: vaste waarde `Wis.h0 = 0.30 m` als standaard
- Bestaande aanroepen van `gate_simulation` zonder α/β: werken ongewijzigd (nargin-check)

## Wat dit niet doet

- Pool 0 dynamica simuleren (peil blijft vast)
- Lekkage toevoegen aan de continue ODE-plantsimulatie (`odeplant` in `runSimulationGabriel`)
- Lekkage terugkoppelen naar de Kalman-filter in de digitale tweeling (apart project)
