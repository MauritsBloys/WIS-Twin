% test_gate_leakage.m
% Run vanuit WIS-sim/simulation/
addpath('../functions');

% Test 1: gesloten sluis zonder lekkage -> flow=0
[flow_no_leak, ~] = gate_simulation(0, 0.25, 0.20, 0, 0, 0);
assert(abs(flow_no_leak) < 1e-10, 'Test 1 mislukt: flow zonder lekkage moet 0 zijn');

% Test 2: gesloten sluis met lekkage -> flow ~9.2253e-5 m3/s
[flow_with_leak, ~] = gate_simulation(0, 0.25, 0.20, 0, 39.617, 0.328);
assert(abs(flow_with_leak - 9.2253e-5) < 1e-7, ...
    sprintf('Test 2 mislukt: verwacht ~9.2253e-5, kreeg %e', flow_with_leak));

% Test 3: achterwaartse compatibiliteit - aanroep zonder alpha/beta
[flow_default, ~] = gate_simulation(0, 0.25, 0.20, 0);
assert(abs(flow_default) < 1e-10, 'Test 3 mislukt: zonder alpha/beta moet flow 0 zijn');

disp('Alle tests voor gate_simulation lekkage geslaagd.');
