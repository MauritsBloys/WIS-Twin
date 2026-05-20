%% load_pool_data.m

% Load data sets for identification

% All data sets labeled with type "experiment" will be merged for
% identification.
% All data sets labeled "validation" will be used for validation of the
% identification.
% Other types will be ignored.

% 20210202, 128SPS
PoolData(1) = createWisData("20210202_step_gate1_2_s25_no_intake.csv", Wis, 1, "experiment", "step 25 pool 1", 1/128);
PoolData(2) = createWisData("20210202_step_gate1_2_s100_no_intake.csv", Wis, 1, "experiment", "step 100 pool 1", 1/128);
PoolData(3) = createWisData("20210202_step_gate1_2_s255_no_intake.csv", Wis, 1, "experiment", "step 255 pool 1", 1/128);

PoolData(4) = createWisData("20210202_step_gate2_3_s25_no_intake.csv", Wis, 2, "experiment", "step 25 pool 2", 1/128);
PoolData(5) = createWisData("20210202_step_gate2_3_s100_no_intake.csv", Wis, 2, "experiment", "step 100 pool 2", 1/128);
PoolData(6) = createWisData("20210202_step_gate2_3_s255_no_intake.csv", Wis, 2, "experiment", "step 255 pool 2", 1/128);

PoolData(7) = createWisData("20210202_step_gate3_4_s25_no_intake.csv", Wis, 3, "nexperiment", "step 25 pool 3", 1/128);
PoolData(8) = createWisData("20210202_step_gate3_4_s100_no_intake.csv", Wis, 3, "experiment", "step 100 pool 3", 1/128);
PoolData(9) = createWisData("20210202_step_gate3_4_s255_no_intake.csv", Wis, 3, "experiment", "step 255 pool 3", 1/128);
PoolData(10) = createWisData("20210202_step_gate3_4_s50_no_intake.csv", Wis, 3, "experiment", "step 50 pool 3", 1/128);

PoolData(11) = createWisData("20210202_step_gate3_4_s25_no_intake2.csv", Wis, 3, "nvalidation", "step 25 pool 3", 1/128);
PoolData(12) = createWisData("20210202_step_gate3_4_s100_no_intake2.csv", Wis, 3, "nvalidation", "step 100 pool 3", 1/128);
PoolData(13) = createWisData("20210202_step_gate3_4_s255_no_intake2.csv", Wis, 3, "nvalidation", "step 255 pool 3", 1/128);
PoolData(14) = createWisData("20210202_step_gate3_4_s50_no_intake2.csv", Wis, 3, "nvalidation", "step 50 pool 3", 1/128);

PoolData(15) = createWisData("20210202_step_gate3_4_s50_no_intake3.csv", Wis, 3, "nvalidation", "step 50 pool 3 b", 1/128);

% TODO: Create validation at 128SPS data for pool 1 and 2

PoolData(16) = createWisData("20210202_step_gate1_2_s25_no_intake.csv", Wis, 1, "nvalidation", "step 25 pool 1", 1/128);
PoolData(17) = createWisData("20210202_step_gate1_2_s100_no_intake.csv", Wis, 1, "nvalidation", "step 100 pool 1", 1/128);
PoolData(18) = createWisData("20210202_step_gate1_2_s255_no_intake.csv", Wis, 1, "nvalidation", "step 255 pool 1", 1/128);

PoolData(19) = createWisData("20210202_step_gate2_3_s25_no_intake.csv", Wis, 2, "nvalidation", "step 25 pool 2", 1/128);
PoolData(20) = createWisData("20210202_step_gate2_3_s100_no_intake.csv", Wis, 2, "nvalidation", "step 100 pool 2", 1/128);
PoolData(21) = createWisData("20210202_step_gate2_3_s255_no_intake.csv", Wis, 2, "nvalidation", "step 255 pool 2", 1/128);

% 20210126, 16SPS

PoolData(22) = createWisData("20210126_step_gate1_2_s25_no_intake.csv", Wis, 1, "slow", "step 25 pool 1 1/16", 1/16);
PoolData(23) = createWisData("20210126_step_gate1_2_s100_no_intake.csv", Wis, 1, "slow", "step 100 pool 1 1/16", 1/16);
PoolData(24) = createWisData("20210126_step_gate1_2_s255_no_intake.csv", Wis, 1, "validation", "step 255 pool 1 1/16", 1/16);

PoolData(25) = createWisData("20210126_step_gate2_3_s25_no_intake.csv", Wis, 2, "slow", "step 25 pool 1 1/16", 1/16);
PoolData(26) = createWisData("20210126_step_gate2_3_s100_no_intake.csv", Wis, 2, "slow", "step 100 pool 1 1/16", 1/16);
PoolData(27) = createWisData("20210126_step_gate2_3_s255_no_intake.csv", Wis, 2, "validation", "step 255 pool 1 1/16", 1/16);

PoolData(28) = createWisData("20210126_step_gate3_4_s25_no_intake.csv", Wis, 3, "slow", "step 25 pool 1 1/16", 1/16);
PoolData(29) = createWisData("20210126_step_gate3_4_s100_no_intake.csv", Wis, 3, "slow", "step 100 pool 1 1/16", 1/16);
PoolData(30) = createWisData("20210126_step_gate3_4_s255_no_intake.csv", Wis, 3, "validation", "step 255 pool 1 1/16", 1/16);

