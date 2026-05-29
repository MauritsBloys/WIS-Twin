%% wis_properties.m

% Global settings for the lab setup 

Wis.area0 = 0.041;  %m2
Wis.area1 = 0.1853; %m2
Wis.area2 = 0.1187; %m2
Wis.area3 = 0.2279; %m2

Wis.delays = [1.3, 0.7, 1.5];

% Lekkageparameters per sluis (empirisch bepaald)
% Formule: q_lek [cm3/s] = alpha*sqrt(dh [cm]) + beta*dh^(3/2)
Wis.leak_alpha = [39.617, 9.402, 40.310]; % sluis 1, 2, 3
Wis.leak_beta  = [0.328,  0.162,  0.559];
Wis.h0         = 0.37;  % gemeten bovenstrooms peil pool 0 [m] (was: 0.30 aanname)
