%% test_twin_mpc.m — Tests for the MPC solver

% Integrator plant: x(k+1) = x(k) + u(k), y(k) = x(k)
A = 1; B = 1; C = 1;
x_hat = -0.1;             % 0.1 below setpoint
y_ref = 0;
Q_mpc = 10; R_mpc = 0.1;
N = 5; du_max = 1;
u_min = -5; u_max = 5;
u_prev = 0;

%% Test 1: normal feasible solve
[u_mpc, inf_flag] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);
assert(u_mpc > 0,               'MPC should command positive input to correct negative deviation');
assert(u_mpc <= 5 && u_mpc >= -5, 'MPC output violates input bounds');
assert(~inf_flag,               'infeasible flag should be false for a feasible problem');
disp('test_twin_mpc: Test 1 (normal solve) PASSED');

%% Test 2: infeasibility flag when bounds conflict (u_min > u_max)
[u_mpc_inf, inf_flag2] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, 10, 5, u_prev);
assert(inf_flag2 == true, 'infeasible flag should be true when u_min > u_max');
% Fallback output must still be numeric
assert(isnumeric(u_mpc_inf), 'twin_mpc_solve must return a numeric fallback on infeasibility');
disp('test_twin_mpc: Test 2 (infeasibility flag) PASSED');

%% Test 3: alarm counter logic triggers after 3 consecutive failures
fail_count = 0;
alarm_on   = false;
for i = 1:5
    [~, inf_i] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, 10, 5, u_prev);
    if inf_i
        fail_count = fail_count + 1;
        if fail_count >= 3 && ~alarm_on
            alarm_on = true;
        end
    else
        fail_count = 0;
        alarm_on   = false;
    end
end
assert(alarm_on == true, 'alarm should be active after 3 consecutive infeasible steps');
assert(fail_count == 5,  'fail_count should equal number of infeasible steps');
disp('test_twin_mpc: Test 3 (alarm trigger) PASSED');

%% Test 4: alarm resets when MPC becomes feasible again
fail_count2 = 0;
alarm_on2   = false;
% Force 3 infeasible
for i = 1:3
    [~, inf_i] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, 10, 5, u_prev);
    if inf_i
        fail_count2 = fail_count2 + 1;
        if fail_count2 >= 3 && ~alarm_on2
            alarm_on2 = true;
        end
    end
end
assert(alarm_on2 == true, 'alarm should be on after 3 infeasible steps');
% One feasible step — alarm should clear
[~, inf_j] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);
if ~inf_j
    alarm_on2   = false;
    fail_count2 = 0;
end
assert(alarm_on2 == false, 'alarm should be cleared after a feasible step');
assert(fail_count2 == 0,   'fail_count should reset after a feasible step');
disp('test_twin_mpc: Test 4 (alarm reset) PASSED');
