%% sim_convergentie_test.m
% Simulatietest: bak 1 start op 60 cm, bak 2 en 3 op 10 cm.
% Setpoints: bak1=0.25m, bak2=0.20m, bak3=0.16m.
%
% Plant-model: Cantoni gescaald met B_mpc_scale (consistent met de MPC).
% Reden: B[wl,u]=39 m/Cantoni (onstabiel bij vol B); de geschaalde B=0.039 m/Cantoni
% is realistisch voor het lab (gemeten fill-rate ~1-3 mm/stap bij servo 100-150).

addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/simulation'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/functions'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/identification'));

%% Configuratie
N            = 10;
Q_mpc        = 1000 * eye(3);
R_mpc        = 0.001 * eye(3);
du_max       = 50/510;
u_min        = zeros(3,1);
u_max        = 0.5 * ones(3,1);
B_mpc_scale  = 1e-3;
Q_kal_scale  = 1e-4;
R_kal_scale  = 1e-3;
h_overflow_per_servo = 0.20 / 55;
MAX_STEPS    = 100;

y_ref        = [0.25; 0.20; 0.20];   % bak3=0.20m=overflow hoogte (fysiek haalbaar)
servo_g4     = 55;
h_overflow_g4 = servo_g4 * h_overflow_per_servo;

wis_properties;

%% Plant matrices
ws_file = fullfile(fileparts(mfilename('fullpath')), '../WIS-sim/simulation/distributed_workspace.mat');
if isfile(ws_file)
    load(ws_file, 'comb_plant_cont');
else
    alpha_ = [1/62.269085474698, 1/180.5271392466, 1/43.8788942518649];
    tau_   = [2/92.3076923076923, 2/171.428571428571, 2/80];
    kappa_ = [0.3, 0.5, 0.3]; phi_ = [10,10,10]; rho_ = [0.1,0.1,0.1];
    Ap_ = zeros(12); Bp_ = zeros(12,3); Cp_ = zeros(3,12);
    for ii_ = 1:3
        r_ = (1+(ii_-1)*4):(4+(ii_-1)*4);
        Ap_(r_,r_) = [0,1/alpha_(ii_),-1/alpha_(ii_),0; 0,-2/tau_(ii_),4/tau_(ii_),0; 0,0,0,1; 0,0,0,-1/rho_(ii_)];
        Bp_(r_,ii_) = [0;0;kappa_(ii_)*phi_(ii_)/rho_(ii_);kappa_(ii_)*(rho_(ii_)-phi_(ii_))/rho_(ii_)^2];
        Cp_(ii_,r_) = [1 0 0 0];
        if ii_ < 3; Ap_(r_, 1+(ii_-1)*4+6) = [-1/alpha_(ii_);0;0;0]; end
    end
    comb_plant_cont = ss(Ap_, Bp_, Cp_, zeros(3));
end
plant_disc = c2d(comb_plant_cont, 1, 'zoh');
A = plant_disc.A; B = plant_disc.B; C = plant_disc.C;
wl_idx = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
B_scaled = B * B_mpc_scale;   % plant gebruikt geschaalde B (consistent met MPC)

fprintf('B[wl,u] origineel: %.4f m/Cantoni  →  geschaald: %.6f m/Cantoni\n', ...
    max(max(abs(B(wl_idx,:)))), max(max(abs(B_scaled(wl_idx,:)))));

Q_kal = Q_kal_scale * eye(size(A,1));
R_kal = R_kal_scale * eye(size(C,1));
d_leak_nom = twin_compute_leakage(y_ref, Wis, wl_idx, size(A,1));

%% Begincondities: bak 1 = 60 cm, bak 2/3 = 10 cm
h_start = [0.60; 0.10; 0.10];
x_plant = zeros(size(A,1), 1);
x_plant(wl_idx) = h_start - y_ref;

fprintf('\nStartwaterpeilen: [%.2f  %.2f  %.2f] m\n', h_start(1), h_start(2), h_start(3));
fprintf('Setpoints:        [%.2f  %.2f  %.2f] m\n', y_ref(1), y_ref(2), y_ref(3));
fprintf('Beginafwijkingen: [%+.2f  %+.2f  %+.2f] m\n\n', ...
    h_start(1)-y_ref(1), h_start(2)-y_ref(2), h_start(3)-y_ref(3));

%% Init Kalman + MPC
x_hat  = zeros(size(A,1), 1);
P      = eye(size(A,1));
u_prev = zeros(3,1);

%% Logbuffers
y_hist = zeros(3, MAX_STEPS);
u_hist = zeros(3, MAX_STEPS);
conv_epoch = NaN;

fprintf('%-5s  %-20s  %-20s  %-18s\n', 'Stap', 'Waterpeilen [m]', 'Afwijkingen [m]', 'Servo [0-255]');
fprintf('%s\n', repmat('-',1,70));

for step = 1:MAX_STEPS
    %% Plant (geschaalde B, realistisch)
    h_sim      = C * x_plant + y_ref;
    d_leak_sim = twin_compute_leakage(h_sim, Wis, wl_idx, size(A,1)) - d_leak_nom;
    x_plant    = A * x_plant + B_scaled * u_prev + d_leak_sim;
    for ii = 1:3
        x_plant(wl_idx(ii)) = max(x_plant(wl_idx(ii)), -y_ref(ii));
    end
    y_meas = C * x_plant + y_ref;
    % Overflow gate sluis 4: klamp pool 3 op h_overflow_g4
    if y_meas(3) > h_overflow_g4
        x_plant(wl_idx(3)) = x_plant(wl_idx(3)) - (y_meas(3) - h_overflow_g4);
        y_meas(3) = h_overflow_g4;
    end

    %% Kalman
    y_dev  = y_meas - y_ref;
    h_est  = C * x_hat + y_ref;
    d_leak = twin_compute_leakage(h_est, Wis, wl_idx, size(A,1)) - d_leak_nom;
    [x_hat, P, ~] = twin_kalman_update(A, B_scaled, C, Q_kal, R_kal, x_hat, P, y_dev, u_prev, d_leak);

    %% MPC
    Q_mpc_eff = Q_mpc;
    if y_meas(3) >= h_overflow_g4 - 0.005
        Q_mpc_eff(3,3) = 0;
    end
    x_hat_mpc = zeros(size(x_hat));
    x_hat_mpc(wl_idx) = y_meas - y_ref;
    [u_mpc, ~] = twin_mpc_solve(A, B_scaled, C, x_hat_mpc, zeros(3,1), ...
        Q_mpc_eff, R_mpc, N, du_max, u_min, u_max, u_prev, d_leak);
    u_prev = u_mpc;

    %% Log
    y_hist(:,step) = y_meas;
    u_hist(:,step) = u_mpc;
    servo = round(u_mpc * 510);
    err   = y_meas - y_ref;

    if step <= 10 || mod(step,10)==0
        fprintf('%-5d  [%.3f  %.3f  %.3f]  [%+.3f %+.3f %+.3f]  [%3d %3d %3d]\n', ...
            step, y_meas(1),y_meas(2),y_meas(3), err(1),err(2),err(3), servo(1),servo(2),servo(3));
    end

    if isnan(conv_epoch) && all(abs(err) < 0.01)
        conv_epoch = step;
        fprintf('\n  *** GECONVERGEERD op stap %d (alle bakken binnen 1 cm) ***\n\n', step);
    end
end

%% Eindrapport
fprintf('\n%s\nSIMULATIERESULTAAT\n%s\n', repmat('=',1,55), repmat('=',1,55));
fprintf('Eindwaterpeilen: [%.3f  %.3f  %.3f] m\n', y_hist(1,end),y_hist(2,end),y_hist(3,end));
fprintf('Setpoints:       [%.3f  %.3f  %.3f] m\n', y_ref(1),y_ref(2),y_ref(3));
final_err = y_hist(:,end) - y_ref;
fprintf('Eindafwijkingen: [%+.4f  %+.4f  %+.4f] m\n', final_err(1),final_err(2),final_err(3));
if isnan(conv_epoch)
    fprintf('Convergentie:    NIET bereikt binnen %d stappen\n', MAX_STEPS);
    fprintf('  Max restfout:  %.1f mm\n', max(abs(final_err))*1000);
else
    fprintf('Convergentie:    stap %d ', conv_epoch);
    if conv_epoch <= 50
        fprintf('✓ GESLAAGD (< 50 epoch)\n');
    else
        fprintf('✗ TE LANGZAAM (> 50 epoch)\n');
    end
end

%% Plot
fig = figure('Name','Sim convergentietest','Visible','off','Position',[100 100 900 550]);
subplot(2,1,1); hold on; grid on;
cols = {'b','r','g'};
for i=1:3
    plot(1:MAX_STEPS, y_hist(i,:), cols{i}, 'LineWidth',1.5, 'DisplayName', sprintf('Bak %d',i));
    yline(y_ref(i), [cols{i} '--'], 'HandleVisibility','off');
end
if ~isnan(conv_epoch); xline(conv_epoch,'k--',sprintf('Conv. stap %d',conv_epoch),'LabelVerticalAlignment','bottom'); end
xline(50,'m:','Doel: stap 50','LabelVerticalAlignment','bottom');
ylabel('Waterstand [m]');
title(sprintf('Simulatietest: bak1=60cm, bak2/3=10cm → setpoints [%.2f %.2f %.2f]m', y_ref(1),y_ref(2),y_ref(3)));
legend('Location','best');

subplot(2,1,2); hold on; grid on;
for i=1:3
    stairs(1:MAX_STEPS, u_hist(i,:)*510, cols{i}, 'LineWidth',1.2, 'DisplayName', sprintf('Sluis %d',i));
end
ylabel('Servo [0-255]'); xlabel('Epoch (stap)'); title('MPC stuurcommandos'); legend('Location','best');

saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'sim_convergentie_test.png'));
fprintf('Plot opgeslagen: sim_convergentie_test.png\n');
