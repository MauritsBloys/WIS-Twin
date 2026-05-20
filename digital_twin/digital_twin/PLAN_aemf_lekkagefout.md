# Plan: AEMF-gebaseerde multiplicatieve lekkagefoutschatting

## Context

De `aemf/` map bevat een gepubliceerde TU Delft codebase voor *Active Estimation of Multiplicative Faults* (Gleizer, Mohajerin Esfahani, Keviczky 2024). Het WIS-digitale-tweeling gebruikt een fysisch lekkagemodel met vaste coëfficiënten (α, β per kanaal). Bij slijtage, verstopping of afdichting kan de werkelijke lekkage structureel afwijken van het model: dit is een **multiplicatieve fout** `c_j`, waarbij `q_j_actual = (1 + c_j) * q_j_nom(h)`. Doel: dit schatbaar maken met de AEMF-ideeën, geïntegreerd in de bestaande Kalman-lus.

---

## Wiskundige verbinding AEMF ↔ WIS lekkage

**Lekkagemodel** (`WIS-sim/functions/wis_leakage.m`):
```
q_j(h) = α_j * sqrt(Δh_j * 100) + β_j * (Δh_j * 100)^(3/2)
```

**Multiplicatieve fout** op lekkagekanaal j:
```
q_j_actual = (1 + c_j) * q_j(h_est)
```

**Effect op Kalman-innovatie** (residual):
```
innov_extra[k] = C * c_j * Δd_j(h_est[k])
```
waarbij `Δd_j(h)` de lekkagecorrectievector is van alleen kanaal j (zelfde structuur als `twin_compute_leakage` maar voor één kanaal).

**AEMF schatformule** (uit `aemf/run_and_process_simulation.m`):
```
E[k, j] = (C * Δd_j(h_est[k]))(relevant_sensor)  ← regressor
c_hat    = pinv(E) * innov_vec                      ← kleinste-kwadraten schatting
σ_min(E) = observeerbaarheidsmaat
```

Dit is de directe analoog van `fhat = pinv(Ei) * Ri` uit de AEMF-code.

---

## Wat hergebruiken uit `aemf/`

| AEMF-bestand | Hergebruik |
|---|---|
| `run_and_process_simulation.m` | Structuur: sliding window, E bouwen, `pinv(E)*r` |
| `generate_filter_parameters.m` | Optioneel: optimaal residu-filter N(z) voor betere observeerbaarheid |
| `grad_descent_loop.m` | Optioneel: optimale ingangsontwerp om σ_min(E) te maximaliseren |
| `MatrixPolynomial.m` | Optioneel: als de lineaire DAE-representatie gebouwd wordt |

Voor BEP-scope: alleen de **kern-schattingstrategie** hergebruiken, niet de volledige filter-ontwerp-pipeline (die vereist Simulink Real-Time toolbox).

---

## Implementatieplan

### Stap 1 — Maak `WIS-twin/twin_estimate_leakage_faults.m` (nieuw bestand)

```matlab
function [c_hat, sigma_min] = twin_estimate_leakage_faults(innov_hist, h_est_hist, Wis, wl_idx, C, n_states)
%   innov_hist  — [3×K] innovatiegeschiedenis
%   h_est_hist  — [3×K] waterstandschatting (absoluut)
%   Wis         — lekkage-struct (h0, leak_alpha, leak_beta, area1/2/3)
%   wl_idx      — waterstandindices in toestandsvector
%   C           — meetmatrix
%   n_states    — totaal aantal toestanden
%
%   c_hat       — [3×1] multiplicatieve foutschattingen per lekkagekanaal
%   sigma_min   — observeerbaarheidsmaat (σ_min(E)²)
```

**Algoritme** (gebaseerd op AEMF `run_and_process_simulation.m` lijnen 11-15):
1. Voor elke tijdstap k en elk kanaal j: bereken `Δd_j(h_est[k])` — lekkagecorrectievector van kanaal j alleen
2. Bouw regressor E ∈ ℝ^(3K × 3): rij per (tijdstap, sensor), kolom per kanaal
   `E((k-1)*3 + 1 : k*3, j) = C * Δd_j(h_est_hist(:,k))`
3. Vectoriseer innovaties: `r_vec = innov_hist(:)` (3K × 1)
4. `c_hat = pinv(E) * r_vec`
5. `sigma_min = min(svd(E))^2`

**Hulpfunctie `compute_delta_d_j`** (inline): zelfde als `twin_compute_leakage` maar voor één kanaal j.

---

### Stap 2 — Breid `WIS-twin/digital_twin.m` uit

**Toevoegen na initialisatie (na regel ~52)**:
```matlab
FAULT_WINDOW = 20;
innov_buf    = nan(3, FAULT_WINDOW);
hest_buf     = nan(3, FAULT_WINDOW);
c_leak_hat   = zeros(3, 1);
```

**In de lus na Kalman-update** (na regel ~151):
```matlab
h_est_now = C * x_hat + y_ref;
innov_buf  = [innov_buf(:,2:end), innov];
hest_buf   = [hest_buf(:,2:end), h_est_now];

if mod(step, FAULT_WINDOW) == 0 && step >= FAULT_WINDOW
    [c_leak_hat, sig_min] = twin_estimate_leakage_faults( ...
        innov_buf, hest_buf, Wis, wl_idx, C, size(A,1));
    fprintf('Lekkagefout c = [%.3f %.3f %.3f], σ_min²=%.2e\n', c_leak_hat, sig_min);
end
```

---

## Observeerbaarheidswaarschuwing (AEMF-idee)

Als `sigma_min(E)^2 < 1e-6`: E is bijna singulier, de fout is niet onderscheidbaar van meetruis. Dit treedt op als waterpeilen dicht bij setpoint blijven (Δh ≈ constant → alle kolommen van E zijn bijna evenredig). Dan: print waarschuwing, zet `c_hat = NaN`.

---

## Verificatie

1. In `digital_twin.m` (simulator-modus), voor de hoofdlus: `Wis.leak_alpha(1) = Wis.leak_alpha(1) * 1.2;`
   → injecteer bekende fout c₁ = 0.2
2. Draai 60 stappen
3. Verwacht: `c_leak_hat(1) ≈ 0.2`, `c_leak_hat(2,3) ≈ 0`
4. Controleer `sigma_min² > 1e-6` voor minstens één kanaal
