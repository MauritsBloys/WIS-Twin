%% test_twin_log.m

tmp = tempname;
mkdir(tmp);
log_file = fullfile(tmp, 'test_log.csv');

% Write two rows
twin_log_write(log_file, 1, [0.24;0.19;0.14], [0.25;0.20;0.15], [0.01;0.01;0.01], [10;12;9], 1);
twin_log_write(log_file, 2, [0.245;0.195;0.145], [0.25;0.20;0.15], [0.005;0.005;0.005], [10;12;9], 1);

data = readmatrix(log_file);
assert(size(data, 1) == 2, 'Expected 2 data rows');
assert(data(1,1) == 1, 'First epoch should be 1');
assert(data(2,1) == 2, 'Second epoch should be 2');
assert(size(data, 2) == 14, 'Expected 14 columns');
assert(abs(data(1, 2) - 0.24) < 1e-5, 'y1_meas row 1 value wrong');
assert(data(1, 14) == 1, 'triggered col row 1 wrong');

rmdir(tmp, 's');
disp('test_twin_log: PASSED');
