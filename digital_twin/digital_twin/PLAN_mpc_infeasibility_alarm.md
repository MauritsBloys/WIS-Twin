# Plan: MPC Infeasibility as Fault Indicator

## Context

Add **terminal set constraints** to the MPC formulation so that infeasibility of the underlying QP becomes a meaningful fault indicator. When leakage or a disturbance is too severe, the MPC can no longer guarantee that the output error at step N is below a threshold ε — the optimisation becomes infeasible. This triggers an alarm and a fault accommodation loop that relaxes ε until either the MPC regains a solution (tolerable fault) or fails entirely (terminal failure).

The scheme:
1. Solve MPC with tight ε_terminal → if infeasible → alarm
2. Double ε and retry, up to EPSILON_MAX → if still infeasible → terminal failure
3. Log both the alarm level and ε used each step
4. Show this in the MATLAB live plot and the HTML dashboard

---

## Mathematical Formulation

The existing MPC builds Y = Sx·x̂ + Su·U. The terminal output at step N is:

```
y(N) = Sx_N · x̂ + Su_N · U         (last ny rows of Sx, Su)
```

The terminal box constraint ||y(N)||_∞ ≤ ε (in deviation coords, y_ref=0) adds 6 QP rows:

```
A_terminal = [ Su_N;  -Su_N ]
b_terminal = [ ε·1 − Sx_N·x̂;  ε·1 + Sx_N·x̂ ]
```

appended to A_ineq / b_ineq.

---

## Files to Change

| File | What changes |
|------|-------------|
| `twin_mpc_solve.m` | New signature, terminal constraint rows, capture exitflag, return `[u_mpc, exitflag, epsilon_used]` |
| `twin_config.m` | Add `EPSILON_TERMINAL = 0.005`, `EPSILON_MAX = 0.05` |
| `digital_twin.m` | Accommodation loop, `alarm_mpc` flag, pass epsilon to logger/plotter |
| `twin_log_write.m` | Add 2 columns: `alarm_mpc`, `epsilon_used` (CSV cols 18–19) |
| `twin_dashboard.html` | New MPC infeasibility alarm + epsilon chart (7th chart panel) |
| `twin_plot_init.m` | New Figure 8: two subplots — ε history + alarm flag |
| `twin_plot_update.m` | Feed Figure 8 with epsilon_hist and alarm_hist |

---

## Step-by-step Implementation

### 1. `twin_mpc_solve.m` — new signature and terminal constraint

Change signature:
```matlab
function [u_mpc, exitflag, epsilon_used] = twin_mpc_solve(..., u_prev, epsilon_terminal)
% epsilon_terminal: optional (default inf = disabled, full backward compatibility)
if nargin < 13; epsilon_terminal = inf; end
```

After building A_ineq / b_ineq (rate constraints), append terminal rows when finite:
```matlab
if isfinite(epsilon_terminal)
    ny = size(C,1);
    Sx_N = Sx(end-ny+1:end, :);          % last ny rows of prediction matrix
    Su_N = Su(end-ny+1:end, :);
    fr   = Sx_N * x_hat;                 % free response at horizon end
    A_ineq = [A_ineq;  Su_N; -Su_N];
    b_ineq = [b_ineq;  epsilon_terminal*ones(ny,1)-fr; ...
                       epsilon_terminal*ones(ny,1)+fr];
end
```

Capture exitflag and return:
```matlab
[U_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts);
epsilon_used = epsilon_terminal;
if isempty(U_opt) || exitflag < 0
    exitflag = -2;
    u_mpc    = min(max(u_prev, u_min), u_max);
else
    u_mpc = U_opt(1:nu);
end
```

### 2. `twin_config.m` — new parameters

```matlab
EPSILON_TERMINAL = 0.005;   % Initial terminal set radius [m] (5 mm)
EPSILON_MAX      = 0.05;    % Max relaxation before terminal failure [m]
```

### 3. `digital_twin.m` — accommodation loop

Replace the single `twin_mpc_solve` call (line 154) with:
```matlab
epsilon_k = EPSILON_TERMINAL;
[u_mpc, exitflag, epsilon_used] = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), ...
    Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev, epsilon_k);

alarm_mpc = 0;  % 0=OK, 1=relaxed (fault tolerable), 2=terminal failure
while exitflag < 0 && epsilon_k < EPSILON_MAX
    alarm_mpc = 1;
    epsilon_k = min(epsilon_k * 2, EPSILON_MAX);
    [u_mpc, exitflag, epsilon_used] = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), ...
        Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev, epsilon_k);
end
if exitflag < 0
    alarm_mpc    = 2;       % terminal failure
    epsilon_used = inf;
    u_mpc        = u_prev;
    warning('MPC: TERMINAL FAILURE — fault too severe to regulate!');
end
u_prev = u_mpc;
```

Also pass `alarm_mpc` and `epsilon_used` to `twin_log_write` and `twin_plot_update`.
Add history buffers: `alarm_hist` and `epsilon_hist` (pre-allocated to MAX_STEPS).

### 4. `twin_log_write.m` — 2 new columns

- Signature: add `alarm_mpc, epsilon_used` parameters
- Header: append `, alarm_mpc, epsilon_used`
- Row vector: append `alarm_mpc, epsilon_used_log` (use `999` as sentinel for `inf`)
- Format string: append `, %d, %.6f`

### 5. `twin_dashboard.html` — new alarm + chart

Column mapping additions (0-indexed):
```javascript
const col_alarm = row[17];     // alarm_mpc  (col 18 in CSV)
const col_eps   = row[18];     // epsilon_used (col 19 in CSV)
```

**New alarm indicator** (insert above existing innovation alarm):
- `alarm_mpc == 1`: orange bar "⚠ MPC noodoplossing: terminale set vergroot naar X mm"
- `alarm_mpc == 2`: red blinking bar "🔴 TERMINALE FOUT: MPC kan systeem niet meer stabiliseren"

**New chart panel** (7th): "MPC Terminale Set ε [m]"
- Line chart of `epsilon_used` over time
- Horizontal reference lines for `EPSILON_TERMINAL` (green dashed) and `EPSILON_MAX` (red dashed)
- y-axis log scale recommended (values span 0.005 → 0.05 → inf)

### 6. `twin_plot_init.m` — Figure 8

```matlab
handles.fig8      = figure(8); clf;
set(handles.fig8, 'Name', 'MPC Uitvoerbaarheid');
handles.ax_eps    = subplot(2,1,1);   % epsilon over time
title(handles.ax_eps, 'Terminale set radius \epsilon_k');
ylabel(handles.ax_eps, '\epsilon [m]');
handles.ax_alarm  = subplot(2,1,2);   % alarm flag (0/1/2)
title(handles.ax_alarm, 'MPC alarm niveau');
ylabel(handles.ax_alarm, '0=OK  1=ontsp.  2=fataal');
```

### 7. `twin_plot_update.m` — feed Figure 8

```matlab
% In figure 8
plot(handles.ax_eps,   t_vec, epsilon_hist, 'b-', 'LineWidth', 1.5);
yline(handles.ax_eps, EPSILON_TERMINAL, 'g--', '\epsilon_0');
yline(handles.ax_eps, EPSILON_MAX,      'r--', '\epsilon_{max}');

stairs(handles.ax_alarm, t_vec, alarm_hist, 'k-', 'LineWidth', 1.5);
ylim(handles.ax_alarm, [-0.5, 2.5]);
yticks(handles.ax_alarm, [0 1 2]);
```

---

## Simulation Demo Scenarios

| Scenario | `disturbance` | Expected result |
|----------|--------------|----------------|
| Normal | `[-0.015; 0; 0]` | MPC stays feasible (alarm_mpc = 0) |
| Moderate fault | `[-0.04; 0; 0]` | ε relaxed 1–2 times (alarm_mpc = 1) |
| Severe fault | `[-0.08; 0; 0]` | Terminal failure (alarm_mpc = 2) |

Set `DISTURBANCE_EPOCH = 20` and choose the scenario in `twin_config.m`.

---

## Verification

1. Run `digital_twin.m` (`USE_HARDWARE=false`):
   - Figure 8 should show ε stepping up at epoch 20 for moderate/severe fault
   - Console prints warning for alarm level ≥ 1
2. Open `twin_dashboard.html` (Python HTTP server on port 8000):
   - Orange/red alarm bar appears after epoch 20
   - Chart 7 shows ε history with reference lines
3. Set `disturbance = [-0.08; 0; 0]` — confirm terminal failure path (alarm_mpc = 2, red blink)
4. Set `EPSILON_TERMINAL = inf` — confirm full backward compatibility (no terminal constraint, alarm_mpc = 0 always)
