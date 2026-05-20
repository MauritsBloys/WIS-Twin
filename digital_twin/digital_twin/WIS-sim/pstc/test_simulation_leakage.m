% test_simulation_leakage.m
% Run vanuit WIS-sim/pstc/
addpath('../functions');

Wis.area1      = 0.1853;
Wis.area2      = 0.1187;
Wis.area3      = 0.2279;
Wis.h0         = 0.30;
Wis.leak_alpha = [39.617, 9.402, 40.310];
Wis.leak_beta  = [0.328,  0.162,  0.559];

xp = [0.25; 0; 0.20; 0; 0.15; 0];

SPS    = 8;
h_step = 1;
dt_sub = h_step / SPS;

h0 = Wis.h0;
h1 = xp(1); h2 = xp(3); h3 = xp(5);

q1 = wis_leakage(h0, h1, Wis.leak_alpha(1), Wis.leak_beta(1));
q2 = wis_leakage(h1, h2, Wis.leak_alpha(2), Wis.leak_beta(2));
q3 = wis_leakage(h2, h3, Wis.leak_alpha(3), Wis.leak_beta(3));

xp(1) = xp(1) + (q1 - q2) * dt_sub / Wis.area1;
xp(3) = xp(3) + (q2 - q3) * dt_sub / Wis.area2;
xp(5) = xp(5) + q3        * dt_sub / Wis.area3;

assert(xp(1) > 0.25, 'Test mislukt: h1 moet stijgen (q1 > q2)');
assert(xp(3) < 0.20, 'Test mislukt: h2 moet dalen (q3 > q2)');
assert(xp(5) > 0.15, 'Test mislukt: h3 moet stijgen');

disp('Smoke-test lekkagecorrectie geslaagd.');
