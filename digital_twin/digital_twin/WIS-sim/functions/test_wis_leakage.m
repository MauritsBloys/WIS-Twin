% test_wis_leakage.m
% Run vanuit WIS-sim/functions/

q = wis_leakage(0.25, 0.20, 39.617, 0.328);
assert(abs(q - 9.2253e-5) < 1e-7, 'Test 1 mislukt: verwacht ~9.2253e-5');

q = wis_leakage(0.20, 0.20, 39.617, 0.328);
assert(q == 0, 'Test 2 mislukt: verwacht 0 bij gelijke peilen');

q = wis_leakage(0.15, 0.20, 39.617, 0.328);
assert(q == 0, 'Test 3 mislukt: verwacht 0 bij h1 < h2');

q = wis_leakage(0.25, 0.20, 9.402, 0.162);
assert(abs(q - 2.2827e-5) < 1e-7, 'Test 4 mislukt: sluis 2 waarden');

disp('Alle tests voor wis_leakage geslaagd.');
