%% digital_twin.m — Main loop for the WIS digital twin
%
% USE_HARDWARE = false (twin_config.m): simulator mode, internal plant model.
% USE_HARDWARE = true:  hardware mode, reads sensor data from Firefly via serial.

addpath(fileparts(mfilename('fullpath')));
twin_config;

%% Opstartdialog — setpoints en beginsluisposities
antw = inputdlg( ...
    {'Pool 1 setpoint [m]:', ...
     'Pool 2 setpoint [m]:', ...
     'Pool 3 setpoint [m]:', ...
     'Sluis 1 beginpositie [servo 0–255]:', ...
     'Sluis 2 beginpositie [servo 0–255]:', ...
     'Sluis 3 beginpositie [servo 0–255]:', ...
     'Sluis 4 (overloopsluis) positie [servo 0–255]:'}, ...
    'WIS Digital Twin — Instellingen', 1, ...
    {num2str(y_ref(1)), num2str(y_ref(2)), num2str(y_ref(3)), '0', '0', '0', '0'});
if isempty(antw)
    fprintf('Geen instellingen ingevoerd — simulatie afgebroken.\n');
    return
end
vals = cellfun(@str2double, antw);
if any(isnan(vals(1:3))) || any(vals(1:3) <= 0) || any(vals(1:3) > 0.50)
    warning('digital_twin: ongeldige setpoints — standaard [%.2f %.2f %.2f] m gebruikt.', ...
        y_ref(1), y_ref(2), y_ref(3));
else
    y_ref = reshape(vals(1:3), 3, 1);
end
servo_init = round(vals(4:6));
if any(isnan(servo_init)) || any(servo_init < 0) || any(servo_init > 255)
    warning('digital_twin: ongeldige sluisposities — start met gesloten sluizen (0).');
    u_init = zeros(3,1);
else
    u_init = servo_init(:) / 255 * 0.5;  % servo [0–255] → Cantoni [0–0.5]
end
servo_g4 = round(vals(7));
if isnan(servo_g4) || servo_g4 < 0 || servo_g4 > 255
    warning('digital_twin: ongeldige overloopsluis positie — gebruik 0.');
    servo_g4 = 0;
end
h_overflow_g4 = servo_g4 * h_overflow_per_servo;   % [m] — afgeleid uit sluispositie
fprintf('Setpoints:      [%.3f  %.3f  %.3f] m\n',          y_ref(1),      y_ref(2),      y_ref(3));
fprintf('Beginposities:  [%3d  %3d  %3d] servo  →  [%.3f  %.3f  %.3f] Cantoni\n', ...
        servo_init(1), servo_init(2), servo_init(3), u_init(1), u_init(2), u_init(3));
fprintf('Overloopsluis:  sluis 4 = %d servo  →  h_overflow = %.3f m\n', servo_g4, h_overflow_g4);

%% Load plant matrices from pre-computed workspace
% comb_plant_cont is the continuous-time Cantoni plant (Ap, Bp, Cp).
% We discretize it at 1 Hz here so the Kalman/MPC match the twin loop rate.
ws_file = fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/simulation/distributed_workspace.mat');
if isfile(ws_file)
    load(ws_file, 'comb_plant_cont');
else
    warning('digital_twin: distributed_workspace.mat ontbreekt. Run generate_plant_workspace.m eenmalig in WIS-sim/simulation/.');
    alpha_ = [1/62.269085474698, 1/180.5271392466, 1/43.8788942518649];
    tau_   = [2/92.3076923076923, 2/171.428571428571, 2/80];
    kappa_ = [0.3, 0.5, 0.3]; phi_ = [10,10,10]; rho_ = [0.1,0.1,0.1];
    Ap_ = zeros(12); Bp_ = zeros(12,3); Cp_ = zeros(3,12);
    for ii_ = 1:3
        r_ = (1+(ii_-1)*4):(4+(ii_-1)*4);
        Ap_(r_,r_) = [0,1/alpha_(ii_),-1/alpha_(ii_),0; 0,-2/tau_(ii_),4/tau_(ii_),0; 0,0,0,1; 0,0,0,-1/rho_(ii_)];
        Bp_(r_,ii_) = [0;0;kappa_(ii_)*phi_(ii_)/rho_(ii_);kappa_(ii_)*(rho_(ii_)-phi_(ii_))/rho_(ii_)^2];
        Cp_(ii_,r_) = [1 0 0 0];
        if ii_ < 3
            Ap_(r_, 1+(ii_-1)*4+6) = [-1/alpha_(ii_);0;0;0];
        end
    end
    comb_plant_cont = ss(Ap_, Bp_, Cp_, zeros(3));
end
plant_disc = c2d(comb_plant_cont, 1, 'zoh');
A = plant_disc.A;
B = plant_disc.B;
C = plant_disc.C;

% Bepaal welke toestanden de waterpeilen zijn (via C-matrix)
wl_idx = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
fprintf('B-matrix koppeling waterstand→u: max(|B([1,5,9],:)|) = %.6f\n', max(max(abs(B(wl_idx,:)))));
if USE_ESTIMATED_QR
    Q_kal = Q_kal_final;
    R_kal = R_kal_final;
    % Normaliseer Q: zorg dat alle drie waterstandstoestanden dezelfde Q/R
    % verhouding krijgen (geometrisch gemiddelde). Voorkomt dat Kalman gain
    % voor pool 3 naar nul daalt terwijl pool 1 gain hoog blijft.
    ratios  = [Q_kal(1,1)/R_kal(1,1), Q_kal(5,5)/R_kal(2,2), Q_kal(9,9)/R_kal(3,3)];
    q_r_gem = (prod(ratios))^(1/3);   % geometrisch gemiddelde
    Q_kal(1,1) = q_r_gem * R_kal(1,1);
    Q_kal(5,5) = q_r_gem * R_kal(2,2);
    Q_kal(9,9) = q_r_gem * R_kal(3,3);
else
    Q_kal = Q_kal_scale * eye(size(A,1));
    R_kal = R_kal_scale * eye(size(C,1));
end

%% Nominale lekkage bij setpoints
% Bij x_plant=0 geldt y_meas=y_ref. De lekkage op dat punt is niet nul,
% waardoor pool 3 langzaam vult (geen uitstroom in het model). Door de
% nominale lekkage af te trekken behandelen we x=0 als het ware evenwicht.
d_leak_nom = twin_compute_leakage(y_ref, Wis, wl_idx, size(A,1));

%% Laad AEMF-filter (eenmalig berekend via wis_aemf_filter_setup.m)
aemf_file = fullfile(fileparts(mfilename('fullpath')), 'data', 'wis_aemf_filter.mat');
if isfile(aemf_file)
    tmp = load(aemf_file, 'aemf');
    aemf_filt = tmp.aemf;
    fprintf('AEMF-filter geladen (orde %d, m=%d).\n', aemf_filt.N_degree, aemf_filt.m);
else
    aemf_filt = [];
    fprintf('Geen AEMF-filter — terugvalmodus (run wis_aemf_filter_setup.m).\n');
end

%% Initialise Kalman state
x_hat = zeros(size(A,1), 1);
P     = eye(size(A,1));

%% Initialise MPC
u_prev         = u_init;
mpc_fail_count = 0;
mpc_alarm      = false;

%% Initialise simulator plant state (only used when USE_HARDWARE = false)
x_plant           = zeros(size(A,1), 1);
x_plant_nompc     = zeros(size(A,1), 1);   % parallel simulatie zonder regeling
DISTURBANCE_EPOCH = 20;
disturbance       = [-0.015; 0; 0];

%% Run duration — lower for quick tests, 1800 = 30 min full run
MAX_STEPS = 600;

%% Initialise logging
timestamp       = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
log_file        = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
log_file_latest = fullfile(LOG_DIR, 'twin_log.csv');
if ~isfolder(LOG_DIR)
    mkdir(LOG_DIR);
end
try; delete(log_file_latest); catch; end

%% Initialise plots
if PLOT_LIVE
    if USE_HARDWARE
        plt = twin_plot_init(y_ref, N, [], Wis);
    else
        plt = twin_plot_init(y_ref, N, DISTURBANCE_EPOCH, Wis);
    end
end

%% History buffers (preallocated to MAX_STEPS)
t_vec          = zeros(1, MAX_STEPS);
y_hist         = zeros(3, MAX_STEPS);
y_pred_hist    = zeros(3, MAX_STEPS);
innov_hist     = zeros(3, MAX_STEPS);
u_hist         = zeros(3, MAX_STEPS);
K_diag_hist    = zeros(3, MAX_STEPS);
y_nompc_hist    = nan(3, MAX_STEPS);
q_leak_est_hist = nan(3, MAX_STEPS);   % geschatte q_leak per kanaal [cm³/s]
q_leak_nom_hist = nan(3, MAX_STEPS);   % nominale  q_leak per kanaal [cm³/s]

%% AEMF lekkagefout-schatting buffers  (H(q)x + (L(q)+aL1+bL2)z formulering)
FAULT_WINDOW   = 20;
innov_buf      = nan(3, FAULT_WINDOW);
hest_buf       = nan(3, FAULT_WINDOW);
alpha_hat_hist = nan(3, MAX_STEPS);   % geschatte alpha per kanaal [cm^0.5]
beta_hat_hist  = nan(3, MAX_STEPS);   % geschatte beta  per kanaal [cm^1.5]
xhat_buf       = nan(size(A,1), FAULT_WINDOW);
u_aemf_buf     = nan(size(B,2), FAULT_WINDOW);

%% Open connection for hardware mode
if USE_HARDWARE
    if USE_FLASK_API
        fprintf('WIS Digital Twin starting (HARDWARE mode, Flask API op %s)...\n', FLASK_URL);
        webwrite([FLASK_URL '/api/firefly/mode/manual'], struct(), weboptions('MediaType','application/json'));
        fprintf('Firefly geschakeld naar MANUAL mode — MPC neemt over.\n');
    else
        device = serialport(COM_PORT, 115200, 'Timeout', 2);
        configureTerminator(device, 'LF');
        fprintf('WIS Digital Twin starting (HARDWARE mode, serieel %s)...\n', COM_PORT);
    end
else
    fprintf('WIS Digital Twin starting (SIMULATOR mode)...\n');
end

%% Main loop
epoch = 0;
step  = 0;
while step < MAX_STEPS

    %% 1. Data acquisition
    if USE_HARDWARE
        if USE_FLASK_API
            try
                resp = webread([FLASK_URL '/api/firefly/status']);
            catch e
                fprintf('Flask API fout (%s) — stap overgeslagen.\n', e.message);
                continue;
            end
            if ~resp.connected
                fprintf('Waarschuwing: Firefly seriële verbinding verbroken — gebruik laatste bekende sensordata.\n');
            end
            s2 = resp.sensors.x2.cm; s4 = resp.sensors.x4.cm; s6 = resp.sensors.x6.cm;
            a201 = resp.actuators.x201; a202 = resp.actuators.x202; a203 = resp.actuators.x203;
            if isempty(s2) || isempty(s4) || isempty(s6) || isempty(a201) || isempty(a202) || isempty(a203)
                fprintf('Sensor- of actuatordata nog niet beschikbaar — stap overgeslagen.\n');
                continue;
            end
            epoch    = step + 1;
            y_meas   = [s2; s4; s6] / 100;          % cm → m
            u_actual = [a201; a202; a203] / 255 * 0.5; % servo 0–255 → Cantoni 0–0.5
            triggered = 1;
            y_nompc   = nan(3,1);
        else
            try
                serial_line = readline(device);
            catch
                continue;
            end
            parts = split(strtrim(serial_line), ',');
            if numel(parts) ~= 13
                continue;
            end
            epoch    = str2double(parts(1));
            u_actual = [str2double(parts(3)); str2double(parts(4)); str2double(parts(5))] / 1000;
            y_meas   = [str2double(parts(7)); str2double(parts(8)); str2double(parts(9))] / 1e6;
            triggered = str2double(parts(13));
            y_nompc   = nan(3,1);
        end
    else
        epoch      = epoch + 1;
        h_sim      = C * x_plant + y_ref;
        d_leak_sim = twin_compute_leakage(h_sim, Wis, wl_idx, size(A,1)) - d_leak_nom;
        d_ext      = zeros(size(A,1), 1);
        if epoch >= DISTURBANCE_EPOCH
            d_ext(wl_idx(1)) = disturbance(1);
        end
        x_plant   = A * x_plant + B * u_prev + d_leak_sim + d_ext;
        % Waterpeil kan fysisch niet negatief worden
        for ii = 1:3
            x_plant(wl_idx(ii)) = max(x_plant(wl_idx(ii)), -y_ref(ii));
        end
        y_meas    = C * x_plant + y_ref;
        triggered = 1;

        % Parallelle simulatie zonder regeling (u=0), zelfde stoornis
        h_nompc        = C * x_plant_nompc + y_ref;
        d_leak_nompc   = twin_compute_leakage(h_nompc, Wis, wl_idx, size(A,1)) - d_leak_nom;
        x_plant_nompc  = A * x_plant_nompc + d_leak_nompc + d_ext;
        % Waterpeil kan fysisch niet negatief worden
        for ii = 1:3
            x_plant_nompc(wl_idx(ii)) = max(x_plant_nompc(wl_idx(ii)), -y_ref(ii));
        end
        y_nompc        = C * x_plant_nompc + y_ref;
    end
    step = step + 1;

    %% 2. Kalman filter update
    % In hardware-modus: gebruik de échte Cantoni-output die de Firefly heeft
    % toegepast (u_actual); in simulator-modus: gebruik de MPC-output (u_prev).
    if USE_HARDWARE
        u_kal = u_actual;
    else
        u_kal = u_prev;
    end
    y_dev   = y_meas - y_ref;
    h_est   = C * x_hat + y_ref;
    d_leak  = twin_compute_leakage(h_est, Wis, wl_idx, size(A,1)) - d_leak_nom;
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_kal, d_leak);

    %% 2b. AEMF: schat lekkageparameters alpha en beta
    % Formulering: H(q)x + (L(q) + alpha.*L1 + beta.*L2) * z = 0
    % z(k) = [y2; y3] = [sqrt(Dh*100); (Dh*100)^1.5]  (tijdsvariabel)
    h_est_abs = C * x_hat + y_ref;
    innov_buf  = [innov_buf(:,  2:end), innov];
    hest_buf   = [hest_buf(:,   2:end), h_est_abs];
    xhat_buf   = [xhat_buf(:,   2:end), x_hat];
    u_aemf_buf = [u_aemf_buf(:, 2:end), u_kal];

    if step >= FAULT_WINDOW
        [alpha_now, beta_now, sigma_min_ab] = twin_estimate_leakage_alphabeta( ...
            innov_buf, hest_buf, Wis, wl_idx, C, size(A,1), xhat_buf, u_aemf_buf, aemf_filt);
        alpha_hat_hist(:, step) = alpha_now;
        beta_hat_hist(:,  step) = beta_now;
        if sigma_min_ab < 1e-6
            fprintf('Stap %d: lekkage niet observeerbaar (sigma_min^2=%.2e)\n', step, sigma_min_ab);
        end
    end

    %% 2c. Lekkageflow berekenen (voor live plot en SCADA)
    h_abs_now = C * x_hat + y_ref;
    Dh_cm_now = max(0, [Wis.h0 - h_abs_now(1); ...
                        h_abs_now(1) - h_abs_now(2); ...
                        h_abs_now(2) - h_abs_now(3)]) * 100;
    q_nom_now = Wis.leak_alpha(:) .* sqrt(Dh_cm_now) + Wis.leak_beta(:) .* Dh_cm_now.^1.5;
    q_leak_nom_hist(:, step) = q_nom_now;

    lk_available = step >= FAULT_WINDOW && ~any(isnan(alpha_hat_hist(:, step)));
    if lk_available
        alpha_k   = alpha_hat_hist(:, step);
        beta_k    = beta_hat_hist(:,  step);
        q_est_now = max(0, alpha_k .* sqrt(Dh_cm_now) + beta_k .* Dh_cm_now.^1.5);
        q_leak_est_hist(:, step) = q_est_now;
    else
        q_est_now = zeros(3, 1);
    end

    if USE_HARDWARE && USE_FLASK_API
        try
            lk_payload = struct( ...
                'epoch',   epoch, ...
                'available', double(lk_available), ...
                'q_est_1', q_est_now(1), 'q_est_2', q_est_now(2), 'q_est_3', q_est_now(3), ...
                'q_nom_1', q_nom_now(1), 'q_nom_2', q_nom_now(2), 'q_nom_3', q_nom_now(3));
            webwrite([FLASK_URL '/api/twin/leakage'], lk_payload, ...
                weboptions('MediaType', 'application/json', 'Timeout', 1));
        catch
        end
    end

    %% 3. MPC — overflow-bewuste gewichten + meting-gebaseerde waterstandcorrectie
    % Als pool 3 op/boven het overflow-niveau van sluis 4 zit, verwijder de
    % tracking-strafterm voor pool 3. De MPC kan dan sluis 2 vrijuit openen
    % om pool 2 te draineren: extra water in pool 3 loopt weg via sluis 4.
    Q_mpc_eff = Q_mpc;
    if y_meas(3) >= h_overflow_g4 - 0.005
        Q_mpc_eff(3,3) = 0;
    end

    % Vervang de waterstandtoestanden in x_hat door de directe meting.
    % De Kalman-gain convergeert na ~20 stappen naar een kleine waarde,
    % waarna x_hat(wl_idx) kan driften naar nul terwijl y_meas nog fout is.
    % Door de ruwe afwijking te injecteren reageert de MPC altijd op de
    % werkelijke fout, ongeacht modelafwijkingen of Kalman-convergentie.
    x_hat_mpc = x_hat;
    x_hat_mpc(wl_idx) = y_meas - y_ref;

    [u_mpc, mpc_infeasible] = twin_mpc_solve(A, B, C, x_hat_mpc, zeros(size(C,1),1), Q_mpc_eff, R_mpc, N, du_max, u_min, u_max, u_prev);
    if mpc_infeasible
        mpc_fail_count = mpc_fail_count + 1;
        if mpc_fail_count >= 3 && ~mpc_alarm
            mpc_alarm = true;
            fprintf('*** MPC ALARM: QP niet oplosbaar voor %d opeenvolgende stappen (stap %d) ***\n', mpc_fail_count, step);
        end
    else
        if mpc_alarm
            fprintf('MPC alarm opgeheven na stap %d — QP weer oplosbaar.\n', step);
            mpc_alarm = false;
        end
        mpc_fail_count = 0;
    end
    if USE_HARDWARE && USE_FLASK_API
        try
            webwrite([FLASK_URL '/api/twin/mpc-alarm'], ...
                struct('available', 1, 'alarm', double(mpc_alarm), ...
                       'fail_count', mpc_fail_count, 'step', step), ...
                weboptions('MediaType', 'application/json', 'Timeout', 1));
        catch
        end
    end
    u_prev = u_mpc;

    % Diagnostiek elke 10 stappen: toon verschil Kalman vs. meting
    if mod(step, 10) == 0
        fprintf(['Stap %3d | Kalman wl=[%+.4f %+.4f %+.4f] | ' ...
                 'Meting wl=[%+.4f %+.4f %+.4f] | ' ...
                 'u=[%.3f %.3f %.3f] servo=[%3d %3d %3d]\n'], ...
            step, ...
            x_hat(wl_idx(1)), x_hat(wl_idx(2)), x_hat(wl_idx(3)), ...
            y_meas(1)-y_ref(1), y_meas(2)-y_ref(2), y_meas(3)-y_ref(3), ...
            u_mpc(1), u_mpc(2), u_mpc(3), ...
            round(u_mpc(1)*510), round(u_mpc(2)*510), round(u_mpc(3)*510));
    end

    %% 3b. Stuur MPC-commando naar hardware
    if USE_HARDWARE && USE_FLASK_API
        servo_cmd = min(255, max(0, round(u_mpc * 510)));
        nodes = [201, 202, 203];
        for ii = 1:3
            try
                webwrite(sprintf('%s/api/firefly/gate/%d/%d', FLASK_URL, nodes(ii), servo_cmd(ii)), ...
                         struct(), weboptions('MediaType','application/json'));
            catch e
                fprintf('Waarschuwing: sluis %d commando mislukt (%s)\n', ii, e.message);
            end
        end
        try
            webwrite(sprintf('%s/api/firefly/gate/204/%d', FLASK_URL, servo_g4), ...
                     struct(), weboptions('MediaType','application/json'));
        catch e
            fprintf('Waarschuwing: overloopsluis (sluis 4) commando mislukt (%s)\n', e.message);
        end
    end

    %% 4. MPC predicted trajectory for plotting (inclusief lekkage)
    mpc_traj = zeros(3, N);
    x_tmp = x_hat;
    for i = 1:N
        h_tmp         = C * x_tmp + y_ref;
        d_mpc         = twin_compute_leakage(h_tmp, Wis, wl_idx, size(A,1)) - d_leak_nom;
        x_tmp         = A * x_tmp + B * u_mpc + d_mpc;
        mpc_traj(:,i) = C * x_tmp + y_ref;
    end

    %% 5. Log
    y_pred = C * x_hat + y_ref;
    twin_log_write(log_file,        epoch, y_meas, y_pred, innov, u_mpc, triggered, y_nompc);
    twin_log_write(log_file_latest, epoch, y_meas, y_pred, innov, u_mpc, triggered, y_nompc);

    %% 6. Update history and plot
    t_vec(:,step)          = epoch;
    y_hist(:,step)         = y_meas;
    y_pred_hist(:,step)    = y_pred;
    innov_hist(:,step)     = innov;
    u_hist(:,step)         = u_mpc;
    y_nompc_hist(:,step)   = y_nompc;
    K_gain              = (P * C') / (C * P * C' + R_kal);
    K_diag_hist(:,step) = [K_gain(wl_idx(1),1); K_gain(wl_idx(2),2); K_gain(wl_idx(3),3)];

    if PLOT_LIVE
        twin_plot_update(plt, t_vec(:,1:step), y_hist(:,1:step), y_pred_hist(:,1:step), ...
                         innov_hist(:,1:step), u_hist(:,1:step), K_diag_hist(:,1:step), ...
                         mpc_traj, y_ref, y_nompc_hist(:,1:step), ...
                         alpha_hat_hist(:,1:step), beta_hat_hist(:,1:step), ...
                         q_leak_est_hist(:,1:step), q_leak_nom_hist(:,1:step));
    end

    pause(H_LOOP);
end

%% Cleanup
if USE_HARDWARE && USE_FLASK_API
    webwrite([FLASK_URL '/api/firefly/mode/auto'], struct(), weboptions('MediaType','application/json'));
    fprintf('Firefly teruggeschakeld naar AUTO mode (Cantoni).\n');
elseif USE_HARDWARE
    delete(device);
end

fprintf('Digital twin finished. Log: %s\n', log_file);

%% Post-run: toon geschatte lekkagecurves
twin_plot_leakage_curves(alpha_hat_hist(:,1:step), beta_hat_hist(:,1:step), Wis);
