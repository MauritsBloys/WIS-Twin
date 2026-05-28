function [alpha_eff, beta_eff, sigma_min] = twin_estimate_leakage_alphabeta( ...
    innov_hist, h_est_hist, Wis, wl_idx, C, n_states, xhat_hist, u_hist, aemf_filt)
%TWIN_ESTIMATE_LEAKAGE_ALPHABETA  Schat lekkageparameters alpha en beta.
%
%   Formulering (Gleizer, Mohajerin Esfahani, Keviczky 2024):
%
%     H(q)*x + (L(q) + sum_j c_j * Lf_j) * z = 0
%
%   met z = [u; y; y2; y3],  y1_j = hoogteverschil kanaal j [cm],
%        y2_j = sqrt(y1_j)   [cm^0.5],
%        y3_j = y1_j^(3/2)  [cm^1.5],
%        H(q) = nominale lineaire dynamica (geen lekkage),
%        Lf_j = foutkanaalmatrix voor alpha_j (j=1..3) / beta_j (j=4..6).
%
%   Twee modi:
%
%   AEMF-filtermodus (9 inputs):
%     Berekent gefilterd residu r[k] = N(q)*e[k] en bouwt regressormatrix
%     op basis van M_j = N * Lf_j.  Laad filter met wis_aemf_filter_setup.m.
%
%   Terugvalmodus (6 inputs of leeg aemf_filt):
%     Direct least-squares op Kalman-innovaties met CB_alpha/CB_beta.
%
%   Inputs:
%     innov_hist  [3xK]  Kalman-innovatiegeschiedenis
%     h_est_hist  [3xK]  absolute waterpeilen [m]
%     Wis         struct: h0, leak_alpha, leak_beta, area1/2/3
%     wl_idx      [3x1]  waterstandindices in toestandsvector
%     C           [3xn]  meetmatrix
%     n_states    totaal aantal toestanden
%     xhat_hist   [nxK]  (optioneel) Kalman-toestandsschattingen
%     u_hist      [3xK]  (optioneel) regelaaringangen
%     aemf_filt   (optioneel) struct uit wis_aemf_filter.mat
%
%   Outputs:
%     alpha_eff   [3x1]  geschatte alpha [cm^0.5]
%     beta_eff    [3x1]  geschatte beta  [cm^1.5]
%     sigma_min   observeerbaarheidsmaat min(svd(E))^2

use_aemf = nargin >= 9 && ~isempty(aemf_filt) && ...
           ~any(isnan(xhat_hist(:))) && ~any(isnan(u_hist(:)));

if use_aemf
    [alpha_eff, beta_eff, sigma_min] = estimate_aemf( ...
        innov_hist, h_est_hist, xhat_hist, u_hist, Wis, aemf_filt);
else
    [alpha_eff, beta_eff, sigma_min] = estimate_fallback( ...
        innov_hist, h_est_hist, Wis, wl_idx, C, n_states);
end
end

% =========================================================================
function [alpha_eff, beta_eff, sigma_min] = estimate_aemf( ...
    innov_hist, h_est_hist, xhat_hist, u_hist, Wis, af)
% AEMF-filtermodus: r[k] = N*e[k], regressor via M_j*z[k].

K  = size(xhat_hist, 2);
nx = af.nx;  ny = af.ny;  nu = af.nu;  nz = af.nz;
A  = af.A;   B  = af.B;
p  = af.N_degree;
m  = af.m;
nn = nx + ny;

% ---- e[k] = [x_hat[k+1]-A*x_hat[k]-B*u[k]; innov[k]], k=1..K-1 ----
e = zeros(nn, K-1);
for k = 1:K-1
    e(1:nx,     k) = xhat_hist(:,k+1) - A*xhat_hist(:,k) - B*u_hist(:,k);
    e(nx+1:end, k) = innov_hist(:, k);
end

% ---- z[k] = [u[k]; y[k]; y2[k]; y3[k]], k=1..K-1 ----
z = zeros(nz, K-1);
for k = 1:K-1
    h  = h_est_hist(:, k);
    y1 = compute_y1(h, Wis.h0);
    z(1:nu,            k) = u_hist(:, k);
    z(nu+1:nu+ny,      k) = h;
    z(nu+ny+1:nu+2*ny, k) = sqrt(y1);
    z(nu+2*ny+1:end,   k) = y1 .^ (3/2);
end

% ---- Gefilterd residu r[k] = N * [e[k]..e[k+p]], k=1..K-1-p ----
n_r = K - 1 - p;
if n_r < 1
    [alpha_eff, beta_eff, sigma_min] = nan_result(Wis);
    return
end

N_rf   = af.N_rows;
r_filt = zeros(m, n_r);
for k = 1:n_r
    ek_stack = reshape(e(:, k:k+p), [], 1);
    r_filt(:, k) = N_rf * ek_stack;
end

% ---- Regressormatrix E via M_j graad-0 coefficient ----
n_eqs = n_r * m;
E_mat = zeros(n_eqs, 6);
r_vec = reshape(r_filt, [], 1);

for k = 1:n_r
    zk   = z(:, k);
    rows = (k-1)*m + (1:m);
    for j = 1:3
        Ma0 = af.Ma_rows{j}(:, 1:nz);
        Mb0 = af.Mb_rows{j}(:, 1:nz);
        E_mat(rows, j)   = Ma0 * zk;
        E_mat(rows, j+3) = Mb0 * zk;
    end
end

valid = ~any(isnan(E_mat), 2) & ~isnan(r_vec);
if sum(valid) < 6
    [alpha_eff, beta_eff, sigma_min] = nan_result(Wis);
    return
end

params    = pinv(E_mat(valid,:)) * r_vec(valid);
sigma_min = min(svd(E_mat(valid,:)))^2;
alpha_eff = Wis.leak_alpha(:) + params(1:3);
beta_eff  = Wis.leak_beta(:)  + params(4:6);
if sigma_min < 1e-10
    [alpha_eff, beta_eff, sigma_min] = nan_result(Wis);
end
end

% =========================================================================
function [alpha_eff, beta_eff, sigma_min] = estimate_fallback( ...
    innov_hist, h_est_hist, Wis, wl_idx, C, n_states)
% Terugvalmodus: least-squares op Kalman-innovaties.

K = size(innov_hist, 2);

B_alpha = zeros(n_states, 3);
B_alpha(wl_idx(1), 1) = +1/(1e6*Wis.area1);
B_alpha(wl_idx(1), 2) = -1/(1e6*Wis.area1);
B_alpha(wl_idx(2), 2) = +1/(1e6*Wis.area2);
B_alpha(wl_idx(2), 3) = -1/(1e6*Wis.area2);
B_alpha(wl_idx(3), 3) = +1/(1e6*Wis.area3);
B_beta = B_alpha;

CB_alpha = C * B_alpha;
CB_beta  = C * B_beta;

E     = zeros(3*K, 6);
r_vec = innov_hist(:);

for k = 1:K
    h = h_est_hist(:, k);
    if any(isnan(h)); continue; end
    y1 = compute_y1(h, Wis.h0);
    y2 = sqrt(y1);
    y3 = y1 .^ (3/2);
    row = (k-1)*3 + (1:3);
    for j = 1:3
        E(row, j)   = CB_alpha(:, j) * y2(j);
        E(row, 3+j) = CB_beta(:, j)  * y3(j);
    end
end

valid = ~any(isnan(E), 2) & ~isnan(r_vec);
if sum(valid) < 6
    [alpha_eff, beta_eff, sigma_min] = nan_result(Wis);
    return
end

params    = pinv(E(valid,:)) * r_vec(valid);
sigma_min = min(svd(E(valid,:)))^2;
alpha_eff = Wis.leak_alpha(:) + params(1:3);
beta_eff  = Wis.leak_beta(:)  + params(4:6);
if sigma_min < 1e-6
    [alpha_eff, beta_eff, sigma_min] = nan_result(Wis);
end
end

% =========================================================================
function y1 = compute_y1(h, h0)
% y1_j = hoogteverschil kanaal j [cm], ondergrensbegrensd op 0
y1 = max(0, [h0 - h(1); h(1) - h(2); h(2) - h(3)] * 100);
end

function [a, b, s] = nan_result(Wis)
a = nan(3,1);  b = nan(3,1);  s = 0;
end
