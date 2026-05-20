%% WIS (combined plant + local control, cont)

% states of the plant 
%   y_1, water level pool 1
%   delta_1, pade approximation of the delay of pool 1
%   y_2,
%   delta_2,
%   y_3,
%   delta_3,


Ap = [0 62.269085474698 0 0 0 0     0;
      0 -92.3076923076923 0 0 0 0   0;
      0 0 0 180.5271392466 0 0      0;
      0 0 0 -171.428571428571 0 0   0;
      0 0 0 0 0 43.8788942518649    0;
      0 0 0 0 0 -80                 0;
      0 0 0 0 0 0                   0];  % disturbance

Bp = [-5.39665407447383 -5.39665407447383 0;
      16 0 0;
      0 -8.424599831508 -8.424599831508;
      0 16 0;
      0 0 -4.38788942518649;
      0 0 16;
      0 0 0]; % disturbance


% Do we need to output more states
Cp = [1 0 0 0 0 0   0;
      0 0 1 0 0 0   0;
      0 0 0 0 1 0   0];
      
Dp = 0*Cp*Bp;

% noise on pool 3 outflow which is not in the plant 
% results in a dropping water level
Wis.area3 = 0.2279; %m2
% disturbance 0.015 m^3/min
dist = 1 / Wis.area3; % m/min 
E = dist * [0; 0; 0; 0; 1; 0;   0]; 
% This will be an extra, small fluctuation about the steady state
% disturbance.

Ap_no_disturbance = Ap(1:end-1, 1:end-1);  % Remove the state
Bp_no_disturbance = Bp(1:end-1, :);
E_no_disturbance = E(1:end-1, :);

% Gabriel: is this necessary? I don;t want initial disturbance
Ap(5,end) = dist;  % Now in the A matrix.
% FIXED: must be the same as E (A(:,end) = E).

% WIS controller (P control only)

% TODO: do these states (created by Matlab) have a physical meaning?

Ac = [-10 0 0 0 0 0;
      1 0 0 0 0 0;
      0 0 -10 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -10 0;
      0 0 0 0 1 0];

Bc = [1 0 0;
      0 0 0;
      0 1 0;
      0 0 0;
      0 0 1;
      0 0 0];
  
Cc = [-2.37153069171695 -0.237153069171695 0 0 0 0;
      0 0 -6.99590261662633 -0.699590261662633 0 0;
      0 0 0 0 -12.0648635755688 -1.20648635755688];

Dc = [0 0 0;0 0 0;0 0 0];

% Discretize the controller. Period = 1s, but model
% is in minutes, so use 1/60
contContr = ss(Ac, Bc, Cc, Dc);
discContr = c2d(contContr, 1/60, 'tustin');

Ac = discContr.A;
Bc = discContr.B;
Cc = discContr.C;
Dc = discContr.D;

% alternative, created exactly like on FF
Ac = [1.84615384615385 -0.846153846153846 0 0 0 0;1 0 0 0 0 0;0 0 1.84615384615385 -0.846153846153846 0 0;0 0 1 0 0 0;0 0 0 0 1.84615384615385 -0.846153846153846;0 0 0 0 1 0];
Bc = [-0.0790510230572318 0 0;0 0 0;0 -0.139918052332527 0;0 0 0;0 0 -0.402162119185627;0 0 0];
Cc = [0.426775147928994 -0.426005917159763 0 0 0 0;0 0 0.71129191321499 -0.710009861932939 0 0;0 0 0 0 0.426775147928994 -0.426005917159763];
Dc = [-0.0182577459022568 0 0;0 -0.053859481042104 0;0 0 -0.092883981758065];

% Lineariseer lekkage rond setpoints y_ref = [0.25; 0.20; 0.15] m
% en voeg koppelingstermen toe aan Ap (nog in eenheden van 1/min).
%
% Lekkagestromen: q1: pool0→1 (alpha=39.617), q2: 1→2 (alpha=9.402), q3: 2→3 (alpha=40.310)
% Hoeveelheidsafgeleiden: dq/d(dh_cm) = alpha/(2*sqrt(dh_cm)) + beta*(3/2)*sqrt(dh_cm)
% Omrekening naar SI: k_SI [m^2/s] = k [cm^2/s] / 1e4
% Effect op Ap [1/min] = k_SI / area * 60
h_ref = [0.25; 0.20; 0.15];  h0 = 0.30;
dh = [(h0-h_ref(1)); (h_ref(1)-h_ref(2)); (h_ref(2)-h_ref(3))] * 100;  % [cm]
lk_a = [39.617, 9.402, 40.310];
lk_b = [0.328,  0.162,  0.559];
k_SI = zeros(3,1);
for ii = 1:3
    slope = lk_a(ii)/(2*sqrt(dh(ii))) + lk_b(ii)*(3/2)*sqrt(dh(ii));  % [cm^2/s]
    k_SI(ii) = slope / 1e4;  % [m^2/s]
end
A1 = 0.1853; A2 = 0.1187; A3 = 0.2279;  % bassindimensies [m^2]
% Toestanden: y1=1, delta1=2, y2=3, delta2=4, y3=5, delta3=6
Ap(1,1) = Ap(1,1) + (-k_SI(1) - k_SI(2)) / A1 * 60;
Ap(1,3) = Ap(1,3) +   k_SI(2)             / A1 * 60;
Ap(3,1) = Ap(3,1) +   k_SI(2)             / A2 * 60;
Ap(3,3) = Ap(3,3) + (-k_SI(2) - k_SI(3)) / A2 * 60;
Ap(3,5) = Ap(3,5) +   k_SI(3)             / A2 * 60;
Ap(5,3) = Ap(5,3) +   k_SI(3)             / A3 * 60;
Ap(5,5) = Ap(5,5) +  -k_SI(3)             / A3 * 60;
clear h_ref h0 dh lk_a lk_b k_SI A1 A2 A3 ii slope

% Put time scale to seconds (for conditioning)
Ap = Ap/60;
Bp = Bp/60;
E = E/60;

% Epoch length
h = 1;  % seconds now!


%% Dimensions

np = size(Ap,1);  % states of the plant
nc = size(Ac,1);  % states of the controller
pp = size(Cp,1);  % measured plant outputs
mp = size(Bp,2);  % number of control inputs
nw = size(E,2);   % number of disturbances
ppt = 3;  % For WIS: number of outputs to use in triggering


%% Assumption 4: Bound on disturbance
% negative disturbance => outflow
W_MAG = -0.015;
W_MAG = W_MAG*1000;  % Units to mm and the like

% Keep W_MAG at 100% because now there is only 'fluctuation'
%W_MAG = W_MAG/100;  % 1% fluctuation about real deal.

% Bound on noise
if exist('V_GLOBAL','var')
    V_EACH_ELEMENT = V_GLOBAL;
    disp('Using externally set noise value (V_GLOBAL)');
else
    V_EACH_ELEMENT = 0.1; % in mm
end

YFACTOR = 1.1;  % We never know the noise levels that precisely.
V = V_EACH_ELEMENT^2*YFACTOR^2*eye(pp)*pp;