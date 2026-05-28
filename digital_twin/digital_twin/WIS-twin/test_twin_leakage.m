%% test_twin_leakage.m — Tests voor AEMF lekkageparameter schatting
%
%   Verifies twin_estimate_leakage_alphabeta and twin_plot_leakage_curves.
%   Run vanuit WIS-twin/ in MATLAB.

fprintf('\n=== test_twin_leakage.m ===\n\n');
n_pass = 0; n_fail = 0;

%% Testopstelling: minimale WIS-configuratie (3 toestanden = waterpeilen)
Wis.h0         = 0.28;
Wis.leak_alpha = [10.0; 8.0; 6.0];
Wis.leak_beta  = [1.0;  0.8; 0.6];
Wis.area1      = 0.07;
Wis.area2      = 0.07;
Wis.area3      = 0.07;

n_states = 3;
wl_idx   = [1; 2; 3];
C        = eye(3);
y_ref    = [0.25; 0.20; 0.15];
K        = 40;

% Waterpeilen die variëren zodat E voldoende rang heeft
t = (1:K);
h_est_hist = y_ref + 0.03 * [sin(2*pi/K * t); cos(2*pi/K * t); sin(4*pi/K * t)];

%% Test 1: nul innovaties → nominale parameters terugkrijgen
innov_zero = zeros(3, K);
[alpha_eff, beta_eff, sigma_min] = twin_estimate_leakage_alphabeta( ...
    innov_zero, h_est_hist, Wis, wl_idx, C, n_states);

if ~any(isnan(alpha_eff)) && ...
   norm(alpha_eff - Wis.leak_alpha) < 1e-8 && ...
   norm(beta_eff  - Wis.leak_beta)  < 1e-8 && ...
   sigma_min > 1e-6
    fprintf('PASS: nul innovaties geven nominale parameters (sigma_min=%.2e)\n', sigma_min);
    n_pass = n_pass + 1;
else
    fprintf('FAIL: nul innovaties geven verkeerde parameters\n');
    fprintf('      alpha_err=%.2e, beta_err=%.2e\n', ...
        norm(alpha_eff - Wis.leak_alpha), norm(beta_eff - Wis.leak_beta));
    n_fail = n_fail + 1;
end

%% Test 2: synthetische fout → correcte parameters terugkrijgen
% Bouw de regressormatrix E op dezelfde manier als de schatter,
% dan geldt: als r = E * params, moet pinv(E)*r = params.
Dalpha_true = [0.5; -0.3; 0.2];
Dbeta_true  = [0.1; -0.05; 0.03];
params_true = [Dalpha_true; Dbeta_true];

B_alpha = zeros(n_states, 3);
B_alpha(wl_idx(1), 1) = +1 / (1e6 * Wis.area1);
B_alpha(wl_idx(1), 2) = -1 / (1e6 * Wis.area1);
B_alpha(wl_idx(2), 2) = +1 / (1e6 * Wis.area2);
B_alpha(wl_idx(2), 3) = -1 / (1e6 * Wis.area2);
B_alpha(wl_idx(3), 3) = +1 / (1e6 * Wis.area3);
B_beta = B_alpha;

CB_alpha = C * B_alpha;
CB_beta  = C * B_beta;

E_syn = zeros(3*K, 6);
for k = 1:K
    h = h_est_hist(:, k);
    y1 = [max(0, (Wis.h0 - h(1)) * 100);
          max(0, (h(1)   - h(2)) * 100);
          max(0, (h(2)   - h(3)) * 100)];
    y2 = sqrt(y1);
    y3 = y1.^(3/2);
    row = (k-1)*3 + (1:3);
    for j = 1:3
        E_syn(row, j)   = CB_alpha(:, j) * y2(j);
        E_syn(row, 3+j) = CB_beta(:, j)  * y3(j);
    end
end

innov_syn = reshape(E_syn * params_true, 3, K);

[alpha_eff2, beta_eff2, ~] = twin_estimate_leakage_alphabeta( ...
    innov_syn, h_est_hist, Wis, wl_idx, C, n_states);

alpha_exp = Wis.leak_alpha + Dalpha_true;
beta_exp  = Wis.leak_beta  + Dbeta_true;

if ~any(isnan(alpha_eff2)) && ...
   norm(alpha_eff2 - alpha_exp) < 1e-6 && ...
   norm(beta_eff2  - beta_exp)  < 1e-6
    fprintf('PASS: synthetische fout correct teruggekregen\n');
    n_pass = n_pass + 1;
else
    fprintf('FAIL: parameterherstel niet correct\n');
    fprintf('      alpha_err=%.2e, beta_err=%.2e\n', ...
        norm(alpha_eff2 - alpha_exp), norm(beta_eff2 - beta_exp));
    n_fail = n_fail + 1;
end

%% Test 3: onvoldoende data (alles NaN) → NaN terugkrijgen
innov_nan = nan(3, K);
[alpha_nan, beta_nan, sig_nan] = twin_estimate_leakage_alphabeta( ...
    innov_nan, h_est_hist, Wis, wl_idx, C, n_states);

if all(isnan(alpha_nan)) && all(isnan(beta_nan)) && sig_nan == 0
    fprintf('PASS: NaN innovaties geven NaN terug\n');
    n_pass = n_pass + 1;
else
    fprintf('FAIL: NaN invoer gaf geen NaN uitvoer\n');
    n_fail = n_fail + 1;
end

%% Test 4: twin_plot_leakage_curves runt zonder crash
try
    alpha_hist = repmat(alpha_exp, 1, K);
    beta_hist  = repmat(beta_exp,  1, K);
    twin_plot_leakage_curves(alpha_hist, beta_hist, Wis);
    close all;
    fprintf('PASS: twin_plot_leakage_curves runt zonder fout\n');
    n_pass = n_pass + 1;
catch e
    fprintf('FAIL: twin_plot_leakage_curves geeft fout: %s\n', e.message);
    n_fail = n_fail + 1;
end

%% Test 5: twin_plot_leakage_curves met allemaal NaN → geen crash
try
    twin_plot_leakage_curves(nan(3, K), nan(3, K), Wis);
    fprintf('PASS: twin_plot_leakage_curves met NaN geeft geen crash\n');
    n_pass = n_pass + 1;
catch e
    fprintf('FAIL: twin_plot_leakage_curves met NaN geeft fout: %s\n', e.message);
    n_fail = n_fail + 1;
end

%% Resultaat
fprintf('\nResultaat: %d/%d tests geslaagd.\n', n_pass, n_pass + n_fail);
if n_fail == 0
    fprintf('Alle tests GESLAAGD.\n\n');
else
    error('%d test(s) MISLUKT.', n_fail);
end
