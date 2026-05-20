%% schat_Q_R.m — Schatting van Kalman ruis-covarianties Q en R
%
% Methode: EM-algoritme (Shumway & Stoffer, 1982) met RTS-smoother
%   R: initieel geschat uit hoog-frequente sensorresiduen in stationair segment
%   Q: geschat via EM-algoritme (30 iteraties), sparse diagonaalstructuur
%   R: verfijnd in elke EM-iteratie
%
% Invoer: METING_FILE hieronder (kolommen: t_s, s1..s7 in cm)
% Uitvoer: data/Q_R_estimated.mat

base = fileparts(mfilename('fullpath'));
addpath(base);

%% Configuratie — pas aan bij nieuwe meting
METING_FILE = fullfile(base, 'data', 'hoofdmeting.csv');

%% Stap 1: Data laden
% Waterstandsensoren: kolommen 3, 5, 7 (s2, s4, s6) → omzetten naar [m]
raw   = readmatrix(METING_FILE);
t_raw = raw(:, 1);
y_raw = raw(:, [3, 5, 7]) / 100;

t_1hz = (0 : floor(t_raw(end)))';
y     = interp1(t_raw, y_raw, t_1hz, 'linear')';   % 3 x N
N     = size(y, 2);

fprintf('Data geladen: %d tijdstappen (%.0f seconden)\n', N, t_1hz(end));

%% Stap 2: Plantmodel laden en discretiseren op 1 Hz
load(fullfile(base, '..', 'WIS-sim', 'simulation', 'distributed_workspace.mat'), ...
     'comb_plant_cont');
plant = c2d(comb_plant_cont, 1, 'zoh');
A  = plant.A;
C  = plant.C;
nx = size(A, 1);
ny = size(C, 1);

fprintf('Plant geladen: nx = %d, ny = %d\n', nx, ny);

%% Stap 3: R beginschatting uit hoog-frequente sensorresiduen (stationair segment)
% Verwijder langzame trend met voortschrijdend gemiddelde
% Gebruik alleen het stabiele segment (t >= 500 s) voor nauwkeurigere R-schatting
win   = 51;
trend = movmean(y, win, 2);
resid = y - trend;

idx_ss = find(t_1hz >= 500);
if numel(idx_ss) < 10
    idx_ss = 1:N;   % terugval als dataset te kort is
    fprintf('Waarschuwing: dataset te kort — gebruik alle data voor R-initialisatie\n');
end
R_est = diag(var(resid(:, idx_ss), 0, 2));

fprintf('\nR beginschatting uit stationair segment (m^2):\n');
fprintf('  y1: %.3e\n  y2: %.3e\n  y3: %.3e\n', R_est(1,1), R_est(2,2), R_est(3,3));

%% Stap 4: Q en R verfijnen via EM-algoritme (Shumway & Stoffer, 1982)
% Sparse beginwaarden: waterpeilen 1e-6, interne toestanden 1e-8
Q = 1e-8 * eye(nx);
Q(1,1) = 1e-6;  Q(5,5) = 1e-6;  Q(9,9) = 1e-6;
R  = R_est;
x0 = pinv(C) * y(:,1);   % schat begintoestand uit eerste meting
P0 = eye(nx);

water_states = [1, 5, 9];

fprintf('\nEM-iteraties:\n');
for iter = 1:30
    % E-stap: Kalman-smoother geeft x(k|N) en P(k|N) voor alle k
    [xs, Ps, Pcs] = kalman_smoother(A, C, Q, R, y, x0, P0);

    % M-stap: bereken tweede-orde statistieken
    S11 = zeros(nx);  S10 = zeros(nx);  S00 = zeros(nx);
    for k = 2:N
        S11 = S11 + Ps(:,:,k)    + xs(:,k)   * xs(:,k)';
        S10 = S10 + Pcs(:,:,k-1) + xs(:,k)   * xs(:,k-1)';
        S00 = S00 + Ps(:,:,k-1)  + xs(:,k-1) * xs(:,k-1)';
    end
    Q_full = (S11 - S10*A' - A*S10' + A*S00*A') / (N-1);

    % Sparse structuur: alleen waterstandstoestanden vrijlaten, rest op 1e-8
    Q = 1e-8 * eye(nx);
    for i = water_states
        Q(i,i) = max(Q_full(i,i), 1e-8);
    end

    % M-stap: R updaten (diagonaal houden — sensoren zijn onafhankelijk)
    R_new = zeros(ny);
    for k = 1:N
        e     = y(:,k) - C * xs(:,k);
        R_new = R_new + e*e' + C*Ps(:,:,k)*C';
    end
    R = diag(diag(R_new / N));

    fprintf('  it %2d: Q(1,1)=%.2e  Q(5,5)=%.2e  Q(9,9)=%.2e\n', ...
            iter, Q(1,1), Q(5,5), Q(9,9));
end

fprintf('\nQ waterstandsdiagonaal na EM (m^2):\n');
fprintf('  Q(1,1): %.3e\n  Q(5,5): %.3e\n  Q(9,9): %.3e\n', Q(1,1), Q(5,5), Q(9,9));
fprintf('\nR na EM (m^2):\n');
fprintf('  y1: %.3e\n  y2: %.3e\n  y3: %.3e\n', R(1,1), R(2,2), R(3,3));

%% Stap 5: Opslaan
Q_kal_final = Q;
R_kal_final = R;
out_file = fullfile(base, 'data', 'Q_R_estimated.mat');
save(out_file, 'Q_kal_final', 'R_kal_final');
fprintf('\nOpgeslagen in %s\n', out_file);

%% Stap 6: Validatie — witheidstoets op innovaties
innov = zeros(ny, N);
x_hat = x0;
P     = P0;
for k = 1:N
    x_prior    = A * x_hat;
    P_prior    = A * P * A' + Q_kal_final;
    innov(:,k) = y(:,k) - C * x_prior;
    S          = C * P_prior * C' + R_kal_final;
    K          = P_prior * C' / S;
    x_hat      = x_prior + K * innov(:,k);
    IKC        = eye(nx) - K * C;
    P          = IKC * P_prior * IKC' + K * R_kal_final * K';
end

figure('Name', 'Innovatie witheidstoets');
for i = 1:3
    subplot(3,1,i)
    autocorr(innov(i,:), 'NumLags', min(30, floor(N/4)));
    title(sprintf('Innovatie autocorrelatie — sensor y%d', i))
    grid on
end
fprintf('\nValidatie: ACF-pieken bij lag >= 1 moeten binnen de blauwe band vallen.\n');
