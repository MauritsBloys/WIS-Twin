function handles = twin_plot_init(y_ref, N, disturbance_epoch, Wis)
%TWIN_PLOT_INIT  Maak afzonderlijke figuurvensters per plot.
%   handles = twin_plot_init(y_ref, N, disturbance_epoch, Wis)
%   y_ref             : 3x1 referentiewaterpeilen
%   N                 : MPC-horizonlengte
%   disturbance_epoch : tijdstap waarop verstoring begint (optioneel)
%   Wis               : lekkage-struct voor nominale alpha-lijnen (optioneel)

if nargin < 3 || isempty(disturbance_epoch)
    disturbance_epoch = [];
end
if nargin < 4
    Wis = [];
end

colors = {'b','r','g'};

%% Figuur 1: Waterpeilen
handles.fig_levels = figure('Name', 'WIS — Waterpeilen', 'NumberTitle', 'off');
handles.ax_levels  = axes('Parent', handles.fig_levels);
title(handles.ax_levels, 'Water levels');
xlabel(handles.ax_levels, 'Epoch (stap)'); ylabel(handles.ax_levels, 'm');
grid(handles.ax_levels, 'on'); hold(handles.ax_levels, 'on');
for i = 1:3
    handles.h_meas(i) = plot(handles.ax_levels, NaN, NaN, [colors{i} '-'],  'DisplayName', sprintf('Pool %d meas', i));
    handles.h_pred(i) = plot(handles.ax_levels, NaN, NaN, [colors{i} '--'], 'DisplayName', sprintf('Pool %d pred', i));
    handles.h_ref(i)  = yline(handles.ax_levels, y_ref(i), [colors{i} ':'], 'HandleVisibility', 'off');
end
legend(handles.ax_levels, 'Location', 'best');

%% Figuur 2: MPC stuurcommando
handles.fig_u = figure('Name', 'WIS — MPC stuurcommando', 'NumberTitle', 'off');
handles.ax_u  = axes('Parent', handles.fig_u);
title(handles.ax_u, 'MPC control input');
xlabel(handles.ax_u, 'Epoch (stap)'); ylabel(handles.ax_u, 'Cantoni');
grid(handles.ax_u, 'on'); hold(handles.ax_u, 'on');
for i = 1:3
    handles.h_u(i) = plot(handles.ax_u, NaN, NaN, [colors{i} '-'], 'DisplayName', sprintf('u%d', i));
end
legend(handles.ax_u, 'Location', 'best');

%% Figuur 3: Innovatie
handles.fig_innov = figure('Name', 'WIS — Innovatie', 'NumberTitle', 'off');
handles.ax_innov  = axes('Parent', handles.fig_innov);
title(handles.ax_innov, 'Innovation (pre-correction residual)');
xlabel(handles.ax_innov, 'Epoch (stap)'); ylabel(handles.ax_innov, 'm');
grid(handles.ax_innov, 'on'); hold(handles.ax_innov, 'on');
yline(handles.ax_innov, 0, 'k:', 'HandleVisibility', 'off');
for i = 1:3
    handles.h_innov(i) = plot(handles.ax_innov, NaN, NaN, [colors{i} '-'], 'DisplayName', sprintf('innov%d', i));
end
legend(handles.ax_innov, 'Location', 'best');

%% Figuur 4: Kalman gain
handles.fig_kgain = figure('Name', 'WIS — Kalman gain', 'NumberTitle', 'off');
handles.ax_kgain  = axes('Parent', handles.fig_kgain);
title(handles.ax_kgain, 'Kalman gain (diagonal, waterstandtoestanden)');
xlabel(handles.ax_kgain, 'Epoch (stap)');
grid(handles.ax_kgain, 'on'); hold(handles.ax_kgain, 'on');
for i = 1:3
    handles.h_kg(i) = plot(handles.ax_kgain, NaN, NaN, [colors{i} '-'], 'DisplayName', sprintf('K%d', i));
end
legend(handles.ax_kgain, 'Location', 'best');

%% Figuur 5: MPC horizontraject
handles.fig_horizon = figure('Name', 'WIS — MPC trajectorie', 'NumberTitle', 'off');
handles.ax_horizon  = axes('Parent', handles.fig_horizon);
title(handles.ax_horizon, 'MPC predicted trajectory (current step)');
xlabel(handles.ax_horizon, 'Epoch (stap)'); ylabel(handles.ax_horizon, 'm');
grid(handles.ax_horizon, 'on'); hold(handles.ax_horizon, 'on');
for i = 1:3
    handles.h_hor(i)     = plot(handles.ax_horizon, NaN(1,N), NaN(1,N), [colors{i} '-o'], 'DisplayName', sprintf('Pool %d predicted', i));
    handles.h_ref_hor(i) = yline(handles.ax_horizon, y_ref(i), [colors{i} ':'], 'HandleVisibility', 'off');
end
legend(handles.ax_horizon, 'Location', 'best');

%% Figuur 6: Vergelijking MPC vs. geen regeling
handles.fig_compare = figure('Name', 'WIS — MPC vs. geen regeling', 'NumberTitle', 'off');
handles.ax_compare  = axes('Parent', handles.fig_compare);
title(handles.ax_compare, 'Waterpeilen: MPC vs. geen regeling (open lus, u=0)');
xlabel(handles.ax_compare, 'Epoch (stap)'); ylabel(handles.ax_compare, 'm');
grid(handles.ax_compare, 'on'); hold(handles.ax_compare, 'on');
for i = 1:3
    handles.h_cmp_mpc(i) = plot(handles.ax_compare, NaN, NaN, [colors{i} '-'],  'LineWidth', 1.5, 'DisplayName', sprintf('Pool %d MPC', i));
    handles.h_cmp_nom(i) = plot(handles.ax_compare, NaN, NaN, [colors{i} '--'], 'LineWidth', 1,   'DisplayName', sprintf('Pool %d geen reg.', i));
    handles.h_ref_cmp(i) = yline(handles.ax_compare, y_ref(i), [colors{i} ':'], 'HandleVisibility', 'off');
end
legend(handles.ax_compare, 'Location', 'best');

%% Figuur 7: Sluisbewegingen
handles.fig_gates = figure('Name', 'WIS — Sluisbewegingen', 'NumberTitle', 'off');
handles.ax_gates  = axes('Parent', handles.fig_gates);
title(handles.ax_gates, 'Sluisbewegingen (MPC stuurcommando)');
xlabel(handles.ax_gates, 'Epoch (stap)'); ylabel(handles.ax_gates, 'Servo [0–255]');
grid(handles.ax_gates, 'on'); hold(handles.ax_gates, 'on');
yline(handles.ax_gates, 0, 'k:', 'DisplayName', 'geen regeling (u=0)');
for i = 1:3
    handles.h_gates(i) = plot(handles.ax_gates, NaN, NaN, [colors{i} '-'], 'DisplayName', sprintf('Sluis %d (u_mpc)', i));
end
legend(handles.ax_gates, 'Location', 'best');

%% Verstoringsmarkering — na aanmaken alle assen
if ~isempty(disturbance_epoch)
    dist_axes = [handles.ax_levels, handles.ax_innov, handles.ax_compare, handles.ax_gates];
    for ax = dist_axes
        xline(ax, disturbance_epoch, 'k--', 'HandleVisibility', 'off', 'LineWidth', 1);
    end
end

%% Figuur 8: Geschatte alpha en beta per kanaal (AEMF H(q)x + (L(q)+aL1+bL2)z)
handles.fig_leak = figure('Name', 'WIS — Lekkageparameters (AEMF)', 'NumberTitle', 'off');

handles.ax_alpha = subplot(2,1,1, 'Parent', handles.fig_leak);
title(handles.ax_alpha, 'Geschatte \alpha per kanaal  (z: y2 = \surd\Deltah)');
xlabel(handles.ax_alpha, 'Epoch (stap)'); ylabel(handles.ax_alpha, '\alpha_{eff}  [cm^{1/2}]');
grid(handles.ax_alpha, 'on'); hold(handles.ax_alpha, 'on');
if ~isempty(Wis)
    for i = 1:3
        yline(handles.ax_alpha, Wis.leak_alpha(i), [colors{i} ':'], ...
            'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
end
for i = 1:3
    handles.h_alpha(i) = plot(handles.ax_alpha, NaN, NaN, [colors{i} '-'], ...
        'LineWidth', 1.5, 'DisplayName', sprintf('\\alpha_%d', i));
end
legend(handles.ax_alpha, 'Location', 'best');

handles.ax_beta = subplot(2,1,2, 'Parent', handles.fig_leak);
title(handles.ax_beta, 'Geschatte \beta per kanaal  (z: y3 = \Deltah^{3/2})');
xlabel(handles.ax_beta, 'Epoch (stap)'); ylabel(handles.ax_beta, '\beta_{eff}  [cm^{3/2}]');
grid(handles.ax_beta, 'on'); hold(handles.ax_beta, 'on');
if ~isempty(Wis)
    for i = 1:3
        yline(handles.ax_beta, Wis.leak_beta(i), [colors{i} ':'], ...
            'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
end
for i = 1:3
    handles.h_beta(i) = plot(handles.ax_beta, NaN, NaN, [colors{i} '-'], ...
        'LineWidth', 1.5, 'DisplayName', sprintf('\\beta_%d', i));
end
legend(handles.ax_beta, 'Location', 'best');

%% Figuur 9: Lekkageflow per kanaal [cm³/s]
handles.fig_qflow = figure('Name', 'WIS — Lekkageflow (AEMF)', 'NumberTitle', 'off', ...
    'Color', 'white', 'Position', [200 200 860 480]);

handles.ax_qflow = subplot(2, 1, 1, 'Parent', handles.fig_qflow);
title(handles.ax_qflow, 'Geschatte lekkageflow q_{leak}  vs. nominaal');
xlabel(handles.ax_qflow, 'Epoch (stap)');
ylabel(handles.ax_qflow, 'q  [cm^3/s]');
grid(handles.ax_qflow, 'on'); hold(handles.ax_qflow, 'on');
for i = 1:3
    handles.h_qest(i) = plot(handles.ax_qflow, NaN, NaN, [colors{i} '-'],  ...
        'LineWidth', 2,   'DisplayName', sprintf('q_{est,%d}', i));
    handles.h_qnom(i) = plot(handles.ax_qflow, NaN, NaN, [colors{i} ':'],  ...
        'LineWidth', 1.2, 'DisplayName', sprintf('q_{nom,%d}', i));
end
legend(handles.ax_qflow, 'Location', 'northwest');

handles.ax_qover = subplot(2, 1, 2, 'Parent', handles.fig_qflow);
title(handles.ax_qover, 'Lekkage-overschot  \Deltaq = q_{est} - q_{nom}');
xlabel(handles.ax_qover, 'Epoch (stap)');
ylabel(handles.ax_qover, '\Deltaq  [cm^3/s]');
grid(handles.ax_qover, 'on'); hold(handles.ax_qover, 'on');
yline(handles.ax_qover, 0, 'k:', 'HandleVisibility', 'off');
for i = 1:3
    handles.h_qover(i) = plot(handles.ax_qover, NaN, NaN, [colors{i} '-'], ...
        'LineWidth', 1.5, 'DisplayName', sprintf('\\Deltaq_%d', i));
end
legend(handles.ax_qover, 'Location', 'northwest');

if ~isempty(disturbance_epoch)
    xline(handles.ax_qflow,  disturbance_epoch, 'k--', 'HandleVisibility', 'off', 'LineWidth', 1);
    xline(handles.ax_qover,  disturbance_epoch, 'k--', 'HandleVisibility', 'off', 'LineWidth', 1);
end

end
