function d = twin_compute_leakage(h_abs, Wis, wl_idx, n_states)
%TWIN_COMPUTE_LEAKAGE  Lekkagecorrectie voor één tijdstap (dt = 1 s).
%
%   Lekkagestromen (zie wis_leakage.m):
%     q1 : pool 0  → pool 1  (instroming vanuit bovenstroomse bak)
%     q2 : pool 1  → pool 2
%     q3 : pool 2  → pool 3
%
%   Inputs:
%     h_abs    — absolute waterpeilen [m], 3×1  [h1; h2; h3]
%     Wis      — struct met velden:
%                  h0          : peil bak 0 [m]
%                  leak_alpha  : [alpha1, alpha2, alpha3]
%                  leak_beta   : [beta1,  beta2,  beta3]
%                  area1/2/3   : oppervlak bassins [m²]
%     wl_idx   — indices van waterstandtoestanden in toestandsvector, 3×1
%     n_states — totaal aantal toestanden
%
%   Output:
%     d        — correctievector [m per tijdstap], n_states×1

    q1 = wis_leakage(Wis.h0,   h_abs(1), Wis.leak_alpha(1), Wis.leak_beta(1));
    q2 = wis_leakage(h_abs(1), h_abs(2), Wis.leak_alpha(2), Wis.leak_beta(2));
    q3 = wis_leakage(h_abs(2), h_abs(3), Wis.leak_alpha(3), Wis.leak_beta(3));

    d = zeros(n_states, 1);
    d(wl_idx(1)) = (q1 - q2) / Wis.area1;   % [m/s] × 1 s = [m] per stap
    d(wl_idx(2)) = (q2 - q3) / Wis.area2;
    d(wl_idx(3)) =  q3       / Wis.area3;
end
