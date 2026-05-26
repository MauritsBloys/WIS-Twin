function [c_hat, sigma_min] = twin_estimate_leakage_faults(innov_hist, h_est_hist, Wis, wl_idx, C, n_states)
%TWIN_ESTIMATE_LEAKAGE_FAULTS  AEMF-gebaseerde schatting van multiplicatieve lekkagefouten.
%
%   Gebaseerd op: Gleizer, Mohajerin Esfahani, Keviczky (2024).
%   Analogon van fhat = pinv(Ei) * Ri uit aemf/run_and_process_simulation.m.
%
%   De innovatie van de Kalman-filter bevat een systematische component als
%   de werkelijke lekkage afwijkt van het nominale model:
%     innov_extra[k] = C * sum_j( c_j * delta_d_j(h_est[k]) )
%   waarbij delta_d_j de verandering in lekkagecorrectievector is per
%   eenheid fout op kanaal j.
%
%   Inputs:
%     innov_hist  — [3xK] innovatiegeschiedenis (sliding window)
%     h_est_hist  — [3xK] geschatte absolute waterpeilen [m]
%     Wis         — lekkage-struct (h0, leak_alpha, leak_beta, area1/2/3)
%     wl_idx      — waterstandindices in toestandsvector, 3x1
%     C           — meetmatrix
%     n_states    — totaal aantal toestanden
%
%   Outputs:
%     c_hat       — [3x1] multiplicatieve foutschatting per kanaal
%                   (NaN als observeerbaarheid onvoldoende)
%     sigma_min   — observeerbaarheidsmaat sigma_min(E)^2

K     = size(innov_hist, 2);
E     = zeros(3*K, 3);
r_vec = innov_hist(:);   % 3K x 1

for k = 1:K
    h = h_est_hist(:, k);
    if any(isnan(h)); continue; end

    q1 = wis_leakage(Wis.h0, h(1), Wis.leak_alpha(1), Wis.leak_beta(1));
    q2 = wis_leakage(h(1),   h(2), Wis.leak_alpha(2), Wis.leak_beta(2));
    q3 = wis_leakage(h(2),   h(3), Wis.leak_alpha(3), Wis.leak_beta(3));

    % Per-eenheid effect van fout c_j op lekkagecorrectievector [m/stap]
    dd1 = zeros(n_states, 1);
    dd1(wl_idx(1)) =  q1 / Wis.area1;

    dd2 = zeros(n_states, 1);
    dd2(wl_idx(1)) = -q2 / Wis.area1;
    dd2(wl_idx(2)) =  q2 / Wis.area2;

    dd3 = zeros(n_states, 1);
    dd3(wl_idx(2)) = -q3 / Wis.area2;
    dd3(wl_idx(3)) =  q3 / Wis.area3;

    row = (k-1)*3 + 1 : k*3;
    E(row, 1) = C * dd1;
    E(row, 2) = C * dd2;
    E(row, 3) = C * dd3;
end

% Verwijder rijen met NaN (niet-gevulde bufferposities)
valid = ~any(isnan(E), 2) & ~isnan(r_vec);
if sum(valid) < 3
    c_hat     = nan(3, 1);
    sigma_min = 0;
    return;
end

E_v = E(valid, :);
r_v = r_vec(valid);

c_hat     = pinv(E_v) * r_v;
sv        = svd(E_v);
sigma_min = min(sv)^2;

% Onvoldoende observeerbaarheid: waterpeilen te dicht bij setpoint
if sigma_min < 1e-6
    c_hat = nan(3, 1);
end
end
