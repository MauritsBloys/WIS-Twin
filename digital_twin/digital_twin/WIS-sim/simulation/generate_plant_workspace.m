%% generate_plant_workspace.m
% Constructs the 12-state Cantoni distributed plant model analytically
% (no CVX required) and saves it as distributed_workspace.mat.
%
% The digital_twin.m script loads comb_plant_cont from that file.
%
% Model reference:
%   Li & Cantoni (2008), "Distributed controller design for open water channels"
%
% Plant structure: 3 pools in series, each with 4 states:
%   state 1: water level y_i          [m]
%   state 2: flow dynamics (Pade)     [m/min]
%   state 3: gate position u_i        [m]
%   state 4: gate actuator omega_i    [m/min]

%% Parameters
% Extracted from lab_setup_values.m and init_plant.m (TU Delft testbed).
% alpha_i = pool cross-sectional area [m^2] (from identification)
% tau_i   = flow delay [min] (from identification, converted: tau = 2/omega_n)
% kappa, phi, rho = loop-shaping weight parameters (tuning 'exceeds_phi_wave')

nPool = 3;

alpha = [1/62.269085474698, 1/180.5271392466, 1/43.8788942518649];
tau   = [2/92.3076923076923, 2/171.428571428571, 2/80];  % [min]
kappa = [0.3, 0.5, 0.3];
phi   = [10, 10, 10];
rho   = [0.1, 0.1, 0.1];

%% Per-pool matrices
% These correspond to eq. (4) of Li & Cantoni (2008).
% Att_i: local dynamics (4x4)
% Ats_i: coupling — inflow from the gate of pool i affects pool i's level
%         (the upstream gate's position state drives this pool's level rate)
% Btu_i: direct control input (4x1)

for i = 1:nPool
    Att{i} = [0,  1/alpha(i), -1/alpha(i), 0;
              0, -2/tau(i),    4/tau(i),    0;
              0,  0,            0,           1;
              0,  0,            0,          -1/rho(i)];

    % Ats couples the gate position of the upstream pool (pool i) into
    % the level dynamics of the downstream pool (pool i+1). Placed in
    % the downstream pool's block row, at the upstream gate's column.
    Ats{i} = [-1/alpha(i); 0; 0; 0];

    Btu{i} = [0;
              0;
              kappa(i)*phi(i)/rho(i);
              kappa(i)*(rho(i)-phi(i))/rho(i)^2];
end

%% Assemble Ap (12x12)
% Block-diagonal entries = Att_i for pool i in rows/cols (1+(i-1)*4):(4+(i-1)*4).
% Off-diagonal coupling: for i < nPool, Ats{i} is placed in pool i's block row
% at column (1 + (i-1)*4 + 6). The offset +6 skips two full 4-state blocks
% relative to pool i's start, landing on the gate position state (state 3)
% of pool i+1. This implements the upstream gate -> downstream level coupling.

Ap = zeros(4*nPool, 4*nPool);

for i = 1:nPool
    rows_i = (1 + (i-1)*4):(4 + (i-1)*4);

    % Diagonal block: local pool dynamics
    Ap(rows_i, rows_i) = Att{i};

    % Coupling block: pool i's gate (state 3 of pool i+1) drives pool i's level
    % Column index: 1 + (i-1)*4 + 6 = gate position of pool i+1
    if i < nPool
        col_gate_next = 1 + (i-1)*4 + 6;
        Ap(rows_i, col_gate_next) = Ats{i};
    end
end

%% Assemble Bp (12x3)
% Block-diagonal: Btu_i in column i, rows of pool i.

Bp = zeros(4*nPool, nPool);

for i = 1:nPool
    rows_i = (1 + (i-1)*4):(4 + (i-1)*4);
    Bp(rows_i, i) = Btu{i};
end

%% Assemble Cp (3x12)
% Each row i picks out state 1 (water level) of pool i.

Cp = zeros(nPool, 4*nPool);

for i = 1:nPool
    Cp(i, 1 + (i-1)*4) = 1;
end

%% Direct feedthrough
Dp = zeros(nPool, nPool);

%% Create continuous-time state-space model
comb_plant_cont = ss(Ap, Bp, Cp, Dp);

%% Save to distributed_workspace.mat (same directory as this script)
save_path = fullfile(fileparts(mfilename('fullpath')), 'distributed_workspace.mat');
save(save_path, 'comb_plant_cont');

fprintf('generate_plant_workspace: saved comb_plant_cont (%dx%d plant) to:\n  %s\n', ...
    size(Ap,1), size(Ap,2), save_path);
