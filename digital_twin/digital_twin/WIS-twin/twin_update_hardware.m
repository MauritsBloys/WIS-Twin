function twin_update_hardware(y_meas, u_actual, epoch)
%TWIN_UPDATE_HARDWARE  Kalman+MPC stap voor hardware-modus (callback vanuit PSTC).
%
%   y_meas   : waterstandmetingen [m], 3×1
%   u_actual : Cantoni regelaaroutput die de Firefly heeft toegepast, 3×1
%              (serial parts 3-5 gedeeld door 1000)
%   epoch    : huidige tijdstap

persistent x_hat P u_mpc_prev log_file log_file_latest plt ...
           t_vec y_hist y_pred_hist innov_hist u_hist K_diag_hist ...
           A B C wl_idx d_leak_nom ...
           innov_buf hest_buf c_hat_hist q_nom_hist q_hat_hist

% Lazy initialisatie bij eerste aanroep
if isempty(x_hat)
    twin_config;   % laadt ook wis_properties → Wis struct
    load(fullfile(fileparts(mfilename('fullpath')), ...
        '../WIS-sim/simulation/distributed_workspace.mat'), 'comb_plant_cont');
    disc = c2d(comb_plant_cont, 1, 'zoh');
    A = disc.A; B = disc.B; C = disc.C;
    wl_idx     = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
    d_leak_nom = twin_compute_leakage(y_ref, Wis, wl_idx, size(A,1));
    x_hat      = zeros(size(A, 1), 1);
    P          = eye(size(A, 1));
    u_mpc_prev = zeros(size(B, 2), 1);
    timestamp  = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    log_file   = fullfile(LOG_DIR, sprintf('twin_log_%s.csv', timestamp));
    log_file_latest = fullfile(LOG_DIR, 'twin_log.csv');
    if ~isfolder(LOG_DIR); mkdir(LOG_DIR); end
    if isfile(log_file_latest); delete(log_file_latest); end
    t_vec = []; y_hist = zeros(3,0); y_pred_hist = zeros(3,0);
    innov_hist = zeros(3,0); u_hist = zeros(3,0); K_diag_hist = zeros(3,0);
    FAULT_WINDOW = 20;
    innov_buf  = nan(3, FAULT_WINDOW);
    hest_buf   = nan(3, FAULT_WINDOW);
    c_hat_hist = zeros(3,0);
    q_nom_hist = zeros(3,0);
    q_hat_hist = nan(3,0);
    if PLOT_LIVE; plt = twin_plot_init(y_ref, N, [], Wis); end
end

twin_config;   % herlaad elke aanroep zodat tuningwijzigingen doorwerken
if USE_ESTIMATED_QR
    Q_kal = Q_kal_final; R_kal = R_kal_final;
else
    Q_kal = Q_kal_scale * eye(size(A,1));
    R_kal = R_kal_scale * eye(size(C,1));
end

% Kalman update met echte Cantoni-output en lekkagecorrectie
y_dev  = y_meas - y_ref;
h_est  = C * x_hat + y_ref;
d_leak = twin_compute_leakage(h_est, Wis, wl_idx, size(A,1)) - d_leak_nom;
[x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_dev, u_actual, d_leak);

% AEMF: schat multiplicatieve lekkagefouten
h_est_abs  = C * x_hat + y_ref;
innov_buf  = [innov_buf(:,2:end), innov];
hest_buf   = [hest_buf(:,2:end),  h_est_abs];

q_nom_now = [
    wis_leakage(Wis.h0,       h_est_abs(1), Wis.leak_alpha(1), Wis.leak_beta(1));
    wis_leakage(h_est_abs(1), h_est_abs(2), Wis.leak_alpha(2), Wis.leak_beta(2));
    wis_leakage(h_est_abs(2), h_est_abs(3), Wis.leak_alpha(3), Wis.leak_beta(3))
] * 1e6;  % m3/s -> cm3/s
q_nom_hist = [q_nom_hist, q_nom_now];

if ~any(isnan(innov_buf(:)))   % buffer vol na FAULT_WINDOW stappen
    [c_hat_now, sigma_min] = twin_estimate_leakage_faults( ...
        innov_buf, hest_buf, Wis, wl_idx, C, size(A,1));
    c_hat_hist = [c_hat_hist, c_hat_now];
    if ~any(isnan(c_hat_now))
        q_hat_hist = [q_hat_hist, q_nom_now .* (1 + c_hat_now)];
    else
        q_hat_hist = [q_hat_hist, nan(3,1)];
    end
    if sigma_min < 1e-6
        fprintf('Epoch %d: lekkage niet observeerbaar (sigma_min^2=%.2e)\n', epoch, sigma_min);
    end
else
    c_hat_hist = [c_hat_hist, nan(3,1)];
    q_hat_hist = [q_hat_hist, nan(3,1)];
end

% MPC (berekent optimale actie; kan niet verstuurd worden via huidig PSTC-protocol)
u_mpc = twin_mpc_solve(A, B, C, x_hat, zeros(size(C,1),1), Q_mpc, R_mpc, N, ...
                        du_max, u_min, u_max, u_mpc_prev);
u_mpc_prev = u_mpc;

% MPC-trajectvoorspelling inclusief lekkage
mpc_traj = zeros(3, N);
x_tmp = x_hat;
for i = 1:N
    h_tmp         = C * x_tmp + y_ref;
    d_mpc         = twin_compute_leakage(h_tmp, Wis, wl_idx, size(A,1)) - d_leak_nom;
    x_tmp         = A * x_tmp + B * u_mpc + d_mpc;
    mpc_traj(:,i) = C * x_tmp + y_ref;
end

y_pred = C * x_hat + y_ref;
twin_log_write(log_file,        epoch, y_meas, y_pred, innov, u_mpc, 1);
twin_log_write(log_file_latest, epoch, y_meas, y_pred, innov, u_mpc, 1);

t_vec       = [t_vec, epoch];
y_hist      = [y_hist,      y_meas];
y_pred_hist = [y_pred_hist, y_pred];
innov_hist  = [innov_hist,  innov];
u_hist      = [u_hist,      u_mpc];
K_gain      = (P * C') / (C * P * C' + R_kal);
K_diag_hist = [K_diag_hist, [K_gain(wl_idx(1),1); K_gain(wl_idx(2),2); K_gain(wl_idx(3),3)]];

if PLOT_LIVE
    twin_plot_update(plt, t_vec, y_hist, y_pred_hist, innov_hist, u_hist, K_diag_hist, ...
        mpc_traj, y_ref, [], q_nom_hist, q_hat_hist, c_hat_hist);
end
end
