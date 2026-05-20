%% sensor_parameters_c.m

% print c code for sensor calibration

fprintf("// START - Calibration values calculated in MATLAB\n")

fprintf("static double sensor1_offsets[NUM_PRIMARY_SENSORS] = {%d, %d, %d, %d};\n", ...
    Wis.b(1), Wis.b(3), Wis.b(5), Wis.b(7));
fprintf("static double sensor2_offsets[NUM_SECONDARY_SENSORS] = {%d, %d, %d};\n", ...
    Wis.b(2), Wis.b(4), Wis.b(6));

fprintf("static double sensor1_scalings[NUM_PRIMARY_SENSORS] = {%d, %d, %d, %d};\n", ...
    Wis.a(1), Wis.a(3), Wis.a(5), Wis.a(7));
fprintf("static double sensor2_scalings[NUM_SECONDARY_SENSORS] = {%d, %d, %d};\n", ...
    Wis.a(2), Wis.a(4), Wis.a(6));

fprintf("// END - Calibration values calculated in MATLAB\n")
