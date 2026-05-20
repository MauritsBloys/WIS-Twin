% write c code for the PI compensator parameters


%y = ku0 * u + ku1 * u * z^-1 + ku2 * u * z^-2 - ky1 * y * z^-1 - ky2 * y * z^-2

fprintf("// START - Compensator values calculated in MATLAB\n");
fprintf("static double ku0[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Numerator{1}(1), Wd{2}.Numerator{1}(1), Wd{3}.Numerator{1}(1));
fprintf("static double ku1[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Numerator{1}(2), Wd{2}.Numerator{1}(2), Wd{3}.Numerator{1}(2));
fprintf("static double ku2[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Numerator{1}(3), Wd{2}.Numerator{1}(3), Wd{3}.Numerator{1}(3));

%fprintf("static double ky0[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Denominator{1}(1), Wd{2}.Denominator{1}(1), Wd{3}.Denominator{1}(1));
fprintf("static double ky1[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Denominator{1}(2), Wd{2}.Denominator{1}(2), Wd{3}.Denominator{1}(2));
fprintf("static double ky2[NUM_ACTUATORS] = {%d, %d, %d};\n", Wd{1}.Denominator{1}(3), Wd{2}.Denominator{1}(3), Wd{3}.Denominator{1}(3));
fprintf("// END - Compensator values calculated in MATLAB\n");

