%% twin_config.m — Digital twin configuration

% Guard: mfilename returns empty when run interactively from Editor in wrong directory
if isempty(mfilename('fullpath'))
    error('twin_config must be run as a script (e.g. via digital_twin), not interactively from the Editor in a different directory.');
end

% Data source
USE_HARDWARE  = true;   % true = hardware data, false = internal plant simulator
USE_FLASK_API = true;   % true = lees via Flask REST API, false = directe seriële COM-poort
COM_PORT      = 'COM3'; % seriële poort Firefly (alleen bij USE_HARDWARE=true, USE_FLASK_API=false)
FLASK_URL     = 'http://localhost:5000'; % Flask SCADA base URL (bij USE_FLASK_API=true)

% Kalman filter noise covariances — laad data-gedreven schatting indien beschikbaar
qr_file = fullfile(fileparts(mfilename('fullpath')), 'data', 'Q_R_estimated.mat');
USE_ESTIMATED_QR = isfile(qr_file);
if USE_ESTIMATED_QR
    load(qr_file, 'Q_kal_final', 'R_kal_final');
else
    % standaardwaarden totdat schat_Q_R.m is uitgevoerd
    Q_kal_scale = 1e-4;
    R_kal_scale = 1e-3;
end

% MPC parameters
% Noot: de plant-input u is het Cantoni regelaarsignaal, NIET servo-eenheden.
% Sluispositietoestand_ss = kappa*phi*u = 3*u, dus u=0.33 → volledig open.
% Bounds zijn bepaald op basis van Cantoni actuatordynamica (kappa=0.3, phi=10).
N      = 10;             % prediction horizon [tijdstappen]
Q_mpc  = 1000 * eye(3);  % weging op setpuntafwijking — groot vanwege zwakke B-koppeling op 1 Hz
R_mpc  = 0.001 * eye(3); % weging op regelmoeite — klein zodat MPC actief bijstuurt
du_max = 25/510;         % max regelaarverandering per stap [Cantoni] = 25 servo-stappen/s
u_min  = zeros(3,1);     % ondergrens (sluis dicht)
u_max  = 0.5 * ones(3,1); % bovengrens (~volledig open in Cantoni signaalruimte)

% Setpoints [m]
y_ref = [0.25; 0.20; 0.15];

% Loop timing: 0 = zo snel mogelijk (testen), 1 = real-time 1 Hz
if USE_HARDWARE
    H_LOOP = 1;   % real-time 1 Hz — één Kalman+MPC stap per seconde
else
    H_LOOP = 0;   % zo snel mogelijk (simulator)
end

% Logging and display
LOG_DIR  = fullfile(fileparts(mfilename('fullpath')), 'data');
PLOT_LIVE = true;
WEB_DASH  = true;

% Add WIS-sim modules to path (guarded against duplicates)
for sub_dir = {'../WIS-sim/simulation', '../WIS-sim/functions', '../WIS-sim/functions_jacob', '../WIS-sim/identification'}
    p = fullfile(fileparts(mfilename('fullpath')), sub_dir{1});
    if ~contains(path, p)
        addpath(p);
    end
end

% Laad lab-eigenschappen (bassindimensies en lekkageparameters → Wis struct)
wis_properties;
