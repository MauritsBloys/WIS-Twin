%% test_twin_kalman.m — Tests for the Kalman filter
clear

%% Test 1: 1D system convergence
% Simple 1D system: x(k+1) = x(k), y(k) = x(k)
A = 1; B = 0; C = 1;
Q_kal = 0.01; R_kal = 0.1;
x_hat = 0; P = 1;

% After 20 steps with y=1, x_hat should converge close to 1
for k = 1:20
    [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, 1, 0);
end
assert(abs(x_hat - 1) < 0.05, 'Kalman did not converge to measurement');
assert(isscalar(innov), 'Innovation should be scalar for 1D system');
assert(P > 0, 'Error covariance must remain positive after 1D update');

%% Test 2: 2-state system, P remains symmetric
A2 = [1 0.1; 0 1]; B2 = zeros(2,1); C2 = [1 0];
Q2 = 1e-4 * eye(2); R2 = 1e-3;
x2 = zeros(2,1); P2 = eye(2);

for k = 1:30
    [x2, P2, innov2] = twin_kalman_update(A2, B2, C2, Q2, R2, x2, P2, 0.5, 0);
end
assert(norm(P2 - P2', 'fro') < 1e-10, 'P must remain symmetric');
assert(all(eig(P2) >= 0), 'P must remain positive semi-definite');

disp('test_twin_kalman: PASSED');
