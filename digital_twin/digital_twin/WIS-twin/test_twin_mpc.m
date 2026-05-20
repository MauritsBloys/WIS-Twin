%% test_twin_mpc.m — Tests for the MPC solver

% Integrator plant: x(k+1) = x(k) + u(k), y(k) = x(k)
A = 1; B = 1; C = 1;
x_hat = -0.1;             % 0.1 below setpoint
y_ref = 0;
Q_mpc = 10; R_mpc = 0.1;
N = 5; du_max = 1;
u_min = -5; u_max = 5;
u_prev = 0;

u_mpc = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev);

% Control input should be positive (push state toward 0)
assert(u_mpc > 0, 'MPC should command positive input to correct negative deviation');
% Must respect bounds
assert(u_mpc <= 5 && u_mpc >= -5, 'MPC output violates input bounds');
disp('test_twin_mpc: PASSED');
