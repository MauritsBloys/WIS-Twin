function q_m3s = wis_leakage(h1_m, h2_m, alpha, beta)
% wis_leakage  Bereken lekkageflow door een gesloten sluis.
%   q_m3s = wis_leakage(h1_m, h2_m, alpha, beta)
%   h1_m, h2_m : waterpeilen [m]; alpha, beta : empirische constanten
%   q_m3s : lekkageflow [m3/s], altijd >= 0

    dh_cm = (h1_m - h2_m) * 100;
    if dh_cm <= 0
        q_m3s = 0;
    else
        q_lek_cm3s = alpha * sqrt(dh_cm) + beta * dh_cm^(3/2);
        q_m3s = q_lek_cm3s / 1e6;
    end
end
