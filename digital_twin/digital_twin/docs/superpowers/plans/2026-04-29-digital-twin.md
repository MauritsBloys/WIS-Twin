# Digital Twin WIS — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a real-time digital twin of the WIS 3-pool water system with a Kalman filter observer and MPC controller, MATLAB live plots, and a web dashboard.

**Architecture:** `digital_twin.m` runs the main loop: each second it reads y(k) and u(k) from either the internal plant simulator or Firefly hardware, calls `twin_kalman.m` to update the state estimate, calls `twin_mpc.m` to compute the optimal control input, then logs and plots. A static HTML dashboard polls the CSV log file every 2 seconds.

**Tech Stack:** MATLAB R2025b, `quadprog` (Optimization Toolbox), Chart.js (CDN), Python 3 (HTTP server for dashboard), existing `comb_Pool_disc` from `cantoni_LMI.m`.

**Prerequisite:** `distributed_workspace.mat` must exist in `WIS-sim/simulation/`. Run `cantoni_LMI.m` first if it does not.

---

### Task 1: Project setup — folder structure and configuration

**Files:**
- Create: `WIS-twin/twin_config.m`
- Create: `WIS-twin/data/.gitkeep`

- [ ] **Step 1: Create folder structure**

```bash
mkdir -p "WIS-twin/data"
```

- [ ] **Step 2: Verify prerequisite workspace exists**

In MATLAB:
```matlab
assert(exist('../WIS-sim/simulation/distributed_workspace.mat', 'file') == 2, ...
    'Run cantoni_LMI.m first to generate distributed_workspace.mat');
```

- [ ] **Step 3: Create `WIS-twin/twin_config.m`**

```matlab
%% twin_config.m — Digital twin configuration

% Data source
USE_HARDWARE = false;  % true = Firefly serial, false = internal plant simulator

% Kalman filter noise covariances
Q_kal = 1e-4 * eye(6);   % process noise
R_kal = 1e-3 * eye(3);   % measurement noise

% MPC parameters
N      = 10;             % prediction horizon [time steps]
Q_mpc  = 10  * eye(3);   % weight on setpoint deviation
R_mpc  = 0.1 * eye(3);   % weight on control effort
du_max = 20;             % max gate change per time step [servo units]
u_min  = zeros(3,1);     % lower bound on gate opening
u_max  = 255 * ones(3,1); % upper bound on gate opening

% Setpoints [m]
y_ref = [0.25; 0.20; 0.15];

% Logging and display
LOG_DIR  = fullfile(fileparts(mfilename('fullpath')), 'data');
PLOT_LIVE = true;
WEB_DASH  = true;

% Paths to other modules
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/simulation'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/functions'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/functions_jacob'));
```

- [ ] **Step 4: Commit**

```bash
git add WIS-twin/
git commit -m "feat: add WIS-twin project structure and configuration"
```

---

### Task 2: Kalman filter (`twin_kalman.m`)

**Files:**
- Create: `WIS-twin/twin_kalman.m`
- Create: `WIS-twin/test_twin_kalman.m`

- [ ] **Step 1: Write the failing test**

Create `WIS-twin/test_twin_kalman.m`:

```matlab
%% test_twin_kalman.m — Tests for the Kalman filter

% Simple 1D system: x(k+1) = x(k), y(k) = x(k)
A = 1; B = 0; C = 1;
Q_kal = 0.01; R_kal = 0.1;
x0 = 0; P0 = 1;

% After 20 steps with y=1, x_hat should converge close to 1
x_hat = x0; P = P0;
for k = 1:20
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, 1, 0);
end
assert(abs(x_hat - 1) < 0.05, 'Kalman did not converge to measurement');
assert(isscalar(innov), 'Innovation should be scalar for 1D system');
disp('test_twin_kalman: PASSED');
```

- [ ] **Step 2: Run test to confirm it fails**

```matlab
cd('C:\Users\mauri\Downloads\BEP\Digital Twin\WIS-twin')
test_twin_kalman
```
Expected: error "Undefined function 'twin_kalman_update'"

- [ ] **Step 3: Create `WIS-twin/twin_kalman.m`**

```matlab
function [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_meas, u)
%TWIN_KALMAN_UPDATE  One predict+update step of the discrete Kalman filter.
%
%   Inputs:
%     A, B, C  — state-space matrices (from comb_Pool_disc)
%     Q_kal    — process noise covariance
%     R_kal    — measurement noise covariance
%     x_hat    — prior state estimate (column vector)
%     P        — prior error covariance matrix
%     y_meas   — current measurement (column vector)
%     u        — current control input (column vector)
%
%   Outputs:
%     x_hat    — updated state estimate
%     P        — updated error covariance
%     innov    — innovation (pre-correction residual)

% Predict
x_prior = A * x_hat + B * u;
P_prior = A * P * A' + Q_kal;

% Innovation
innov = y_meas - C * x_prior;

% Kalman gain
S = C * P_prior * C' + R_kal;
K = P_prior * C' / S;

% Update
x_hat = x_prior + K * innov;
P     = (eye(size(P,1)) - K * C) * P_prior;
end
```

- [ ] **Step 4: Run test to confirm it passes**

```matlab
test_twin_kalman
```
Expected output: `test_twin_kalman: PASSED`

- [ ] **Step 5: Commit**

```bash
git add WIS-twin/twin_kalman.m WIS-twin/test_twin_kalman.m
git commit -m "feat: add Kalman filter observer (twin_kalman)"
```

---

### Task 3: MPC solver (`twin_mpc.m`)

**Files:**
- Create: `WIS-twin/twin_mpc.m`
- Create: `WIS-twin/test_twin_mpc.m`

- [ ] **Step 1: Write the failing test**

Create `WIS-twin/test_twin_mpc.m`:

```matlab
%% test_twin_mpc.m — Tests for the MPC solver

% Integrator plant: x(k+1) = x(k) + u(k), y(k) = x(k)
A = 1; B = 1; C = 1;
x_hat = -0.1;             % 0.1 below setpoint
y_ref = 0;
Q_mpc = 10; R_mpc = 0.1;
N = 5; du_max = 1;
u_min = -5; u_max = 5;
u_prev = 0;

u_mpc = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);

% Control input should be positive (push state toward 0)
assert(u_mpc > 0, 'MPC should command positive input to correct negative deviation');
% Must respect bounds
assert(u_mpc <= 5 && u_mpc >= -5, 'MPC output violates input bounds');
disp('test_twin_mpc: PASSED');
```

- [ ] **Step 2: Run test to confirm it fails**

```matlab
test_twin_mpc
```
Expected: error "Undefined function 'twin_mpc_solve'"

- [ ] **Step 3: Create `WIS-twin/twin_mpc.m`**

```matlab
function u_mpc = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev)
%TWIN_MPC_SOLVE  Solve one MPC step using quadprog (receding horizon).
%
%   Builds prediction matrices Sx and Su, formulates a QP, solves with
%   quadprog, and returns only the first control input u*(k).
%
%   Inputs:
%     A, B, C  — state-space matrices (from comb_Pool_disc)
%     x_hat    — current state estimate from Kalman filter
%     y_ref    — setpoint (column vector, length = size(C,1))
%     Q_mpc    — output weight matrix (ny × ny)
%     R_mpc    — input weight matrix (nu × nu)
%     N        — prediction horizon
%     du_max   — max control increment per step (scalar)
%     u_min    — lower bound on u (nu × 1)
%     u_max    — upper bound on u (nu × 1)
%     u_prev   — control input at previous time step (nu × 1)
%
%   Output:
%     u_mpc    — optimal first control input (nu × 1)

nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

% Build prediction matrices: Y = Sx*x_hat + Su*U
% Y is (N*ny × 1), U is (N*nu × 1)
Sx = zeros(N*ny, nx);
Su = zeros(N*ny, N*nu);

Ak = eye(nx);
for i = 1:N
    Ak = A * Ak;
    Sx((i-1)*ny+1:i*ny, :) = C * Ak;
    for j = 1:i
        Su((i-1)*ny+1:i*ny, (j-1)*nu+1:j*nu) = C * A^(i-j) * B;
    end
end

% Block diagonal weight matrices
Q_bar = kron(eye(N), Q_mpc);
R_bar = kron(eye(N), R_mpc);

% QP: min (1/2)*U'*H_qp*U + f_qp'*U
Y_ref_stack = repmat(y_ref, N, 1);
H_qp = 2 * (Su' * Q_bar * Su + R_bar);
f_qp = 2 * Su' * Q_bar * (Sx * x_hat - Y_ref_stack);

% Input bounds: u_min <= u(k+i) <= u_max for all i
lb = repmat(u_min, N, 1);
ub = repmat(u_max, N, 1);

% Rate constraint: |u(k+i) - u(k+i-1)| <= du_max
% Du = [I; -I; 0 I; 0 -I; ...] * U - [u_prev; -u_prev; 0; 0; ...]
D = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1), -1), eye(nu));
A_ineq = [D; -D];
b_ineq = [du_max*ones(N*nu,1) + [u_prev; zeros((N-1)*nu,1)]; ...
          du_max*ones(N*nu,1) - [-u_prev; zeros((N-1)*nu,1)]];

opts = optimoptions('quadprog', 'Display', 'off');
U_opt = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts);

if isempty(U_opt)
    warning('MPC: quadprog returned no solution, using u_prev');
    u_mpc = u_prev;
else
    u_mpc = U_opt(1:nu);
end
end
```

- [ ] **Step 4: Run test to confirm it passes**

```matlab
test_twin_mpc
```
Expected output: `test_twin_mpc: PASSED`

- [ ] **Step 5: Commit**

```bash
git add WIS-twin/twin_mpc.m WIS-twin/test_twin_mpc.m
git commit -m "feat: add MPC solver using quadprog (twin_mpc)"
```

---

### Task 4: Logging (`twin_log.m`)

**Files:**
- Create: `WIS-twin/twin_log.m`
- Create: `WIS-twin/test_twin_log.m`

- [ ] **Step 1: Write the failing test**

Create `WIS-twin/test_twin_log.m`:

```matlab
%% test_twin_log.m

tmp = tempname;
mkdir(tmp);
log_file = fullfile(tmp, 'test_log.csv');

% Write two rows
twin_log_write(log_file, 1, [0.24;0.19;0.14], [0.25;0.20;0.15], [0.01;0.01;0.01], [10;12;9], 1);
twin_log_write(log_file, 2, [0.245;0.195;0.145], [0.25;0.20;0.15], [0.005;0.005;0.005], [10;12;9], 1);

data = readmatrix(log_file);
assert(size(data, 1) == 2, 'Expected 2 data rows');
assert(data(1,1) == 1, 'First epoch should be 1');
assert(data(2,1) == 2, 'Second epoch should be 2');
assert(size(data, 2) == 14, 'Expected 14 columns');

rmdir(tmp, 's');
disp('test_twin_log: PASSED');
```

- [ ] **Step 2: Run test to confirm it fails**

```matlab
test_twin_log
```
Expected: error "Undefined function 'twin_log_write'"

- [ ] **Step 3: Create `WIS-twin/twin_log.m`**

```matlab
function twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, triggered)
%TWIN_LOG_WRITE  Append one row to the twin log CSV.
%
%   Columns: epoch, y1_meas, y2_meas, y3_meas, y1_pred, y2_pred, y3_pred,
%            innov1, innov2, innov3, u_mpc1, u_mpc2, u_mpc3, triggered

header = 'epoch,y1_meas,y2_meas,y3_meas,y1_pred,y2_pred,y3_pred,innov1,innov2,innov3,u_mpc1,u_mpc2,u_mpc3,triggered';

write_header = ~isfile(log_file);

fid = fopen(log_file, 'a');
if fid == -1
    error('twin_log_write: cannot open %s', log_file);
end

if write_header
    fprintf(fid, '%s\n', header);
end

row = [epoch, y_meas(:)', y_pred(:)', innov(:)', u_mpc(:)', triggered];
fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%d\n', row);
fclose(fid);
end
```

- [ ] **Step 4: Run test to confirm it passes**

```matlab
test_twin_log
```
Expected output: `test_twin_log: PASSED`

- [ ] **Step 5: Commit**

```bash
git add WIS-twin/twin_log.m WIS-twin/test_twin_log.m
git commit -m "feat: add CSV logging (twin_log)"
```

---

### Task 5: MATLAB live plots (`twin_plot.m`)

**Files:**
- Create: `WIS-twin/twin_plot.m`

- [ ] **Step 1: Create `WIS-twin/twin_plot.m`**

```matlab
function handles = twin_plot_init(y_ref)
%TWIN_PLOT_INIT  Create 5 live-updating figure windows. Call once before the loop.
%   Returns a struct of figure and axes handles for use in twin_plot_update.

handles.fig = figure('Name', 'WIS Digital Twin', 'NumberTitle', 'off', ...
    'Position', [50 50 1400 900]);

handles.ax_levels  = subplot(3,2,1); title('Water levels'); ylabel('m'); grid on; hold on;
handles.ax_u       = subplot(3,2,2); title('MPC control input'); ylabel('servo'); grid on; hold on;
handles.ax_innov   = subplot(3,2,3); title('Innovation (pre-correction residual)'); ylabel('m'); grid on; hold on;
handles.ax_kgain   = subplot(3,2,4); title('Kalman gain (diagonal)'); grid on; hold on;
handles.ax_horizon = subplot(3,2,[5 6]); title('MPC predicted trajectory (current step)'); ylabel('m'); grid on; hold on;

colors = {'b','r','g'};
for i = 1:3
    handles.h_meas(i)  = plot(handles.ax_levels,  NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('Pool %d meas', i));
    handles.h_pred(i)  = plot(handles.ax_levels,  NaN, NaN, [colors{i} '--'], 'DisplayName', sprintf('Pool %d pred', i));
    handles.h_ref(i)   = yline(handles.ax_levels, y_ref(i), [colors{i} ':']);
    handles.h_u(i)     = plot(handles.ax_u,        NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('u%d', i));
    handles.h_innov(i) = plot(handles.ax_innov,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('innov%d', i));
    handles.h_kg(i)    = plot(handles.ax_kgain,    NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('K%d', i));
end
legend(handles.ax_levels, 'Location', 'best');
yline(handles.ax_innov, 0, 'k:');
end

function twin_plot_update(handles, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref)
%TWIN_PLOT_UPDATE  Refresh all live plot windows with current history.
%   Call every time step inside the main loop, followed by drawnow.

for i = 1:3
    set(handles.h_meas(i),  'XData', t_vec, 'YData', y_hist(i,:));
    set(handles.h_pred(i),  'XData', t_vec, 'YData', y_pred_hist(i,:));
    set(handles.h_u(i),     'XData', t_vec, 'YData', u_hist(i,:));
    set(handles.h_innov(i), 'XData', t_vec, 'YData', innov_hist(i,:));
    set(handles.h_kg(i),    'XData', t_vec, 'YData', K_diag_hist(i,:));
end

% MPC horizon preview
cla(handles.ax_horizon);
t_hor = t_vec(end) + (0:size(mpc_traj,2)-1);
colors = {'b','r','g'};
for i = 1:3
    plot(handles.ax_horizon, t_hor, mpc_traj(i,:), [colors{i} '-o'], ...
        'DisplayName', sprintf('Pool %d predicted', i));
    yline(handles.ax_horizon, y_ref(i), [colors{i} ':']);
end
legend(handles.ax_horizon, 'Location', 'best');

drawnow;
end
```

- [ ] **Step 2: Smoke test (visual check)**

```matlab
cd('C:\Users\mauri\Downloads\BEP\Digital Twin\WIS-twin')
handles = twin_plot_init([0.25; 0.20; 0.15]);
% Should open a figure with 5 subplots, no errors
disp('twin_plot_init: OK (visual check)');
close all;
```

- [ ] **Step 3: Commit**

```bash
git add WIS-twin/twin_plot.m
git commit -m "feat: add MATLAB live plot windows (twin_plot)"
```

---

### Task 6: Main loop — simulator mode (`digital_twin.m`)

**Files:**
- Create: `WIS-twin/digital_twin.m`

This task wires all components together and runs in simulator mode (internal plant model, no hardware required).

- [ ] **Step 1: Create `WIS-twin/digital_twin.m`**

```matlab
%% digital_twin.m — Main loop for the WIS digital twin
%
% Runs simulator mode by default (USE_HARDWARE = false in twin_config.m).
% In simulator mode, advances comb_Pool_disc internally and injects a
% step disturbance at epoch 20.

addpath(fileparts(mfilename('fullpath')));
twin_config;

%% Load plant matrices from pre-computed workspace
load(fullfile('../WIS-sim/simulation/distributed_workspace.mat'), ...
    'comb_Pool_disc', 'h');
A = comb_Pool_disc.A;
B = comb_Pool_disc.B;
C = comb_Pool_disc.C;

%% Initialise Kalman state
x_hat = zeros(size(A,1), 1);
P     = eye(size(A,1));

%% Initialise MPC
u_prev = zeros(size(B,2), 1);

%% Initialise simulator plant state (only used when USE_HARDWARE = false)
x_plant = zeros(size(A,1), 1);
DISTURBANCE_EPOCH = 20;
disturbance = [-0.015; 0; 0];   % outflow from pool 1 [m³/min equivalent]

%% Initialise logging
timestamp  = datestr(now, 'yyyymmdd_HHMMSS');
log_file   = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
symlink_target = fullfile(LOG_DIR, 'twin_log.csv');

%% Initialise plots
if PLOT_LIVE
    plt = twin_plot_init(y_ref);
end

%% History buffers
MAX_STEPS = 1800;
t_vec        = [];
y_hist       = zeros(3, 0);
y_pred_hist  = zeros(3, 0);
innov_hist   = zeros(3, 0);
u_hist       = zeros(3, 0);
K_diag_hist  = zeros(3, 0);

fprintf('WIS Digital Twin starting (%s mode)...\n', ...
    string(ternary(USE_HARDWARE, 'HARDWARE', 'SIMULATOR')));

%% Main loop
for epoch = 1:MAX_STEPS

    %% 1. Get measurement and previous control input
    if USE_HARDWARE
        % Hardware mode: block here until FireflyCommunicationPSTC
        % delivers a measurement via twin_update(). Not implemented in
        % this loop — see Task 7 for hardware integration.
        error('Hardware mode not yet wired. See Task 7.');
    else
        % Simulator mode: advance internal plant
        d = zeros(size(A,1), 1);
        if epoch >= DISTURBANCE_EPOCH
            % inject disturbance: affects first pool state
            d(1) = disturbance(1);
        end
        x_plant = A * x_plant + B * u_prev + d;
        y_meas  = C * x_plant + y_ref;   % absolute water levels
        triggered = 1;
    end

    %% 2. Kalman filter update
    y_dev = y_meas - y_ref;   % work in deviation from setpoint
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_prev);

    %% 3. MPC
    u_mpc = twin_mpc_solve(A, B, C, x_hat, zeros(3,1), Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);
    u_prev = u_mpc;

    %% 4. Compute MPC predicted trajectory for plotting
    mpc_traj = zeros(3, N);
    x_tmp = x_hat;
    for i = 1:N
        x_tmp = A * x_tmp + B * u_mpc;
        mpc_traj(:,i) = C * x_tmp + y_ref;
    end

    %% 5. Log
    y_pred = C * x_hat + y_ref;
    twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, triggered);

    %% 6. Plot
    t_vec       = [t_vec, epoch];
    y_hist      = [y_hist,      y_meas];
    y_pred_hist = [y_pred_hist, y_pred];
    innov_hist  = [innov_hist,  innov];
    u_hist      = [u_hist,      u_mpc];
    K_gain      = (P * C') / (C * P * C' + R_kal);
    K_diag_hist = [K_diag_hist, diag(K_gain(1:3,:))];

    if PLOT_LIVE
        twin_plot_update(plt, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref);
    end

    pause(h);   % simulate real-time 1Hz
end

fprintf('Digital twin finished. Log: %s\n', log_file);
```

Helper function used above — add to bottom of `digital_twin.m` or as separate file:

```matlab
function result = ternary(cond, a, b)
    if cond; result = a; else; result = b; end
end
```

- [ ] **Step 2: Run the simulator for 30 steps, verify no errors**

```matlab
cd('C:\Users\mauri\Downloads\BEP\Digital Twin\WIS-twin')
MAX_STEPS_override = 30;   % edit digital_twin.m temporarily
digital_twin
```
Expected: MATLAB live plot opens, 30 rows written to `data/twin_log_<timestamp>.csv`, no errors.

- [ ] **Step 3: Verify CSV output**

```matlab
data = readmatrix(dir(fullfile('data','twin_log_*.csv')).name);
assert(size(data,1) == 30, 'Expected 30 rows');
assert(size(data,2) == 14, 'Expected 14 columns');
disp('CSV output: OK');
```

- [ ] **Step 4: Commit**

```bash
git add WIS-twin/digital_twin.m
git commit -m "feat: add digital twin main loop with simulator mode"
```

---

### Task 7: Hardware integration — modify `FireflyCommunicationPSTC.m`

**Files:**
- Modify: `WIS-sim/pstc/FireflyCommunicationPSTC.m` — add `twin_update()` call in callback

- [ ] **Step 1: Read the callback method**

Open `WIS-sim/pstc/FireflyCommunicationPSTC.m` and locate the `callbackMessage` method (around line 130). Find the block where `yhat` is set (around line 151–156).

- [ ] **Step 2: Add twin_update call after measurement parsing**

Find this block in `callbackMessage` (lines ~151–171):

```matlab
yhat = [s7, s8, s9]/1e6;           % absolute pressures → water levels [m]
yhat = yhat - [0.25, 0.20, 0.15];  % subtract reference
yhat = yhat * 1000;                 % mm
```

Add immediately after (before the `sleepcontroller` call):

```matlab
% Digital twin update (only if twin is active)
if exist('twin_active', 'var') && twin_active
    y_abs = [s7, s8, s9]' / 1e6;          % absolute [m], column vector
    u_col = [s10, s11, s12]' / 1000;       % control flows, column vector
    addpath(fullfile(fileparts(mfilename('fullpath')), '../../WIS-twin'));
    twin_update_hardware(y_abs, u_col, epoch);
end
```

- [ ] **Step 3: Create `WIS-twin/twin_update_hardware.m`**

```matlab
function twin_update_hardware(y_meas, u_prev, epoch)
%TWIN_UPDATE_HARDWARE  Called from FireflyCommunicationPSTC callback.
%   Runs one Kalman+MPC step and logs/plots.

persistent x_hat P u_mpc_prev log_file plt t_vec y_hist y_pred_hist innov_hist u_hist K_diag_hist

% Lazy initialisation on first call
if isempty(x_hat)
    twin_config;
    load(fullfile(fileparts(mfilename('fullpath')), ...
        '../WIS-sim/simulation/distributed_workspace.mat'), 'comb_Pool_disc');
    x_hat     = zeros(size(comb_Pool_disc.A, 1), 1);
    P         = eye(size(comb_Pool_disc.A, 1));
    u_mpc_prev = zeros(size(comb_Pool_disc.B, 2), 1);
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    log_file  = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
    t_vec = []; y_hist = zeros(3,0); y_pred_hist = zeros(3,0);
    innov_hist = zeros(3,0); u_hist = zeros(3,0); K_diag_hist = zeros(3,0);
    if PLOT_LIVE; plt = twin_plot_init(y_ref); end
end

twin_config;  % reload config (in case parameters changed)
A = comb_Pool_disc.A; B = comb_Pool_disc.B; C = comb_Pool_disc.C;

y_dev = y_meas - y_ref;
[x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_mpc_prev);

u_mpc = twin_mpc_solve(A, B, C, x_hat, zeros(3,1), Q_mpc, R_mpc, N, du_max, u_min, u_max, u_mpc_prev);
u_mpc_prev = u_mpc;

y_pred = C * x_hat + y_ref;
twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, 1);

t_vec = [t_vec, epoch];
y_hist = [y_hist, y_meas]; y_pred_hist = [y_pred_hist, y_pred];
innov_hist = [innov_hist, innov]; u_hist = [u_hist, u_mpc];
K_gain = (P * C') / (C * P * C' + R_kal);
K_diag_hist = [K_diag_hist, diag(K_gain(1:3,:))];

mpc_traj = zeros(3, N);
x_tmp = x_hat;
for i = 1:N
    x_tmp = A * x_tmp + B * u_mpc;
    mpc_traj(:,i) = C * x_tmp + y_ref;
end

if PLOT_LIVE
    twin_plot_update(plt, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, mpc_traj, y_ref);
end
end
```

- [ ] **Step 4: Test hardware integration (requires lab hardware or simulator running)**

In MATLAB (with `wis_simulation.py` running):
```matlab
twin_active = true;
main_pstc   % runs the existing PSTC loop with twin callbacks active
```
Verify that `data/twin_log_<timestamp>.csv` is written and plot windows open.

- [ ] **Step 5: Commit**

```bash
git add WIS-twin/twin_update_hardware.m WIS-sim/pstc/FireflyCommunicationPSTC.m
git commit -m "feat: integrate digital twin into hardware callback"
```

---

### Task 8: Web dashboard (`twin_dashboard.html`)

**Files:**
- Create: `WIS-twin/twin_dashboard.html`

- [ ] **Step 1: Create `WIS-twin/twin_dashboard.html`**

```html
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="UTF-8">
  <title>WIS Digital Twin Dashboard</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
  <style>
    body { font-family: monospace; background: #0d1117; color: #c9d1d9; margin: 16px; }
    h1   { color: #58a6ff; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    .card { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 16px; }
    .badge { display: inline-block; padding: 2px 10px; border-radius: 4px; font-size: 12px; }
    .hw   { background: #238636; }
    .sim  { background: #9e6a03; }
    .alert { color: #f85149; font-weight: bold; }
    canvas { max-height: 220px; }
  </style>
</head>
<body>
  <h1>WIS Digital Twin <span id="mode-badge" class="badge sim">SIMULATOR</span></h1>
  <p id="last-update">Last update: —</p>
  <p id="alert-box"></p>

  <div class="grid">
    <div class="card"><canvas id="ch-levels"></canvas></div>
    <div class="card"><canvas id="ch-innov"></canvas></div>
    <div class="card"><canvas id="ch-umpc"></canvas></div>
    <div class="card"><canvas id="ch-innov-pool"></canvas></div>
  </div>

<script>
const ALERT_THRESHOLD = 0.02;   // [m] — flag if any innovation exceeds this
const POLL_MS = 2000;
const MAX_POINTS = 120;

function makeChart(id, label, datasets) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: [], datasets },
    options: {
      animation: false,
      plugins: { legend: { labels: { color: '#c9d1d9' } } },
      scales: {
        x: { ticks: { color: '#8b949e' }, grid: { color: '#21262d' } },
        y: { ticks: { color: '#8b949e' }, grid: { color: '#21262d' } }
      }
    }
  });
}

const colors = ['#58a6ff', '#f85149', '#3fb950'];

const chartLevels = makeChart('ch-levels', 'Water levels', [
  ...colors.map((c,i) => ({ label: `Pool ${i+1} meas`,  borderColor: c, data: [], pointRadius: 0 })),
  ...colors.map((c,i) => ({ label: `Pool ${i+1} pred`, borderColor: c, borderDash: [5,3], data: [], pointRadius: 0 })),
]);
const chartInnov = makeChart('ch-innov', 'Innovation', colors.map((c,i) =>
  ({ label: `innov${i+1}`, borderColor: c, data: [], pointRadius: 0 })));
const chartU = makeChart('ch-umpc', 'MPC control input', colors.map((c,i) =>
  ({ label: `u${i+1}`, borderColor: c, data: [], pointRadius: 0 })));

function parseCSV(text) {
  const lines = text.trim().split('\n');
  if (lines.length < 2) return [];
  return lines.slice(1).map(l => l.split(',').map(Number));
}

function push(chart, label, values) {
  chart.data.labels.push(label);
  values.forEach((v,i) => chart.data.datasets[i].data.push(v));
  if (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(d => d.data.shift());
  }
}

async function poll() {
  try {
    const resp = await fetch('data/twin_log.csv?t=' + Date.now());
    const text = await resp.text();
    const rows = parseCSV(text);
    if (rows.length === 0) return;

    // Clear and rebuild from full CSV (simpler than diffing)
    chartLevels.data.labels = []; chartLevels.data.datasets.forEach(d => d.data = []);
    chartInnov.data.labels  = []; chartInnov.data.datasets.forEach(d => d.data = []);
    chartU.data.labels      = []; chartU.data.datasets.forEach(d => d.data = []);

    const recent = rows.slice(-MAX_POINTS);
    recent.forEach(r => {
      const epoch = r[0];
      push(chartLevels, epoch, [r[1],r[2],r[3],r[4],r[5],r[6]]);
      push(chartInnov,  epoch, [r[7],r[8],r[9]]);
      push(chartU,      epoch, [r[10],r[11],r[12]]);
    });

    chartLevels.update(); chartInnov.update(); chartU.update();

    // Alert check on latest row
    const last = rows[rows.length-1];
    const maxInnov = Math.max(Math.abs(last[7]), Math.abs(last[8]), Math.abs(last[9]));
    const alertBox = document.getElementById('alert-box');
    alertBox.textContent = maxInnov > ALERT_THRESHOLD
      ? `⚠ High innovation: ${maxInnov.toFixed(4)} m — model deviates from measurement`
      : '';
    alertBox.className = maxInnov > ALERT_THRESHOLD ? 'alert' : '';

    document.getElementById('last-update').textContent = 'Last update: epoch ' + last[0];
  } catch(e) {
    console.warn('Poll failed:', e);
  }
}

setInterval(poll, POLL_MS);
poll();
</script>
</body>
</html>
```

- [ ] **Step 2: Start dashboard server and verify**

```bash
cd "C:\Users\mauri\Downloads\BEP\Digital Twin\WIS-twin"
python -m http.server 8080
```

Open `http://localhost:8080/twin_dashboard.html` in a browser. Run `digital_twin.m` in MATLAB. After a few seconds, the dashboard should show live-updating charts.

- [ ] **Step 3: Verify alert fires**

Wait for the simulated disturbance (epoch 20). The innovation chart should spike and the alert banner should appear.

- [ ] **Step 4: Commit**

```bash
git add WIS-twin/twin_dashboard.html
git commit -m "feat: add web dashboard polling twin_log.csv (Chart.js)"
```

---

## Spec Coverage Check

| Spec requirement | Covered by |
|---|---|
| USE_HARDWARE switch | Task 1 (`twin_config.m`) + Task 7 |
| Kalman filter predict+update | Task 2 (`twin_kalman.m`) |
| Kalman Q/R tunable | Task 1 config + Task 2 |
| MPC with quadprog | Task 3 (`twin_mpc.m`) |
| MPC horizon N, Qy, Ru, du_max | Task 1 config + Task 3 |
| u_mpc output bounds 0–255 | Task 3 |
| CSV logging with timestamp filenames | Task 4 (`twin_log.m`) + Task 6 |
| MATLAB live plots (5 windows) | Task 5 (`twin_plot.m`) |
| Hardware callback integration | Task 7 (`twin_update_hardware.m`) |
| Web dashboard Chart.js | Task 8 (`twin_dashboard.html`) |
| Dashboard CORS fix (Python server) | Task 8 Step 2 |
| Innovation alert | Task 8 |
| Disturbance injection test | Task 6 Step 2 + Task 8 Step 3 |
