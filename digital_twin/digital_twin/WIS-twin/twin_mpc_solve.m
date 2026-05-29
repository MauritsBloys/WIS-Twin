function [u_mpc, infeasible] = twin_mpc_solve(A, B, C, x_hat, y_ref, Q_mpc, R_mpc, N, du_max, u_min, u_max, u_prev, d_feed)
%TWIN_MPC_SOLVE  Solve one MPC step using quadprog (receding horizon).
%
%   Builds prediction matrices Sx and Su, formulates a QP, solves with
%   quadprog, and returns only the first control input u*(k).
%
%   Inputs:
%     A, B, C  — state-space matrices (from comb_Pool_disc)
%     x_hat    — current state estimate (alleen waterstandtoestanden niet-nul)
%     y_ref    — setpoint (column vector, length = size(C,1))
%     Q_mpc    — output weight matrix (ny × ny)
%     R_mpc    — input weight matrix (nu × nu)
%     N        — prediction horizon
%     du_max   — max control increment per step (scalar)
%     u_min    — lower bound on u (nu × 1)
%     u_max    — upper bound on u (nu × 1)
%     u_prev   — control input at previous time step (nu × 1)
%     d_feed   — (optional) constante storingsvector [nx×1], bijv. d_leak
%                Wordt als feedforward meegenomen in de QP-gradiënt.
%
%   Output:
%     u_mpc    — optimal first control input (nu × 1)
%     infeasible — true als quadprog geen oplossing vond

nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

% Build prediction matrices: Y = Sx*x_hat + Su*U
% Y is (N*ny × 1), U is (N*nu × 1)
Sx = zeros(N*ny, nx);
Su = zeros(N*ny, N*nu);

% Precompute A^0 ... A^N to avoid repeated matrix powers in the inner loop
A_pow = cell(N+1, 1);
A_pow{1} = eye(nx);
for k = 1:N
    A_pow{k+1} = A * A_pow{k};
end

for i = 1:N
    Sx((i-1)*ny+1:i*ny, :) = C * A_pow{i+1};
    for j = 1:i
        Su((i-1)*ny+1:i*ny, (j-1)*nu+1:j*nu) = C * A_pow{i-j+1} * B;
    end
end

% Block diagonal weight matrices
Q_bar = kron(eye(N), Q_mpc);
R_bar = kron(eye(N), R_mpc);

% Disturbance feedforward: d_feed constant over horizon
% Bijdrage aan voorspelde output: Sd(i) = C * (A^0 + A^1 + ... + A^(i-1)) * d_feed
Sd = zeros(N*ny, 1);
if nargin >= 13 && ~isempty(d_feed)
    A_sum = zeros(nx);
    for i = 1:N
        A_sum = A_sum + A_pow{i};   % = I + A + ... + A^(i-1)
        Sd((i-1)*ny+1:i*ny) = C * A_sum * d_feed;
    end
end

% QP: min (1/2)*U'*H_qp*U + f_qp'*U
Y_ref_stack = repmat(y_ref, N, 1);
H_qp = 2 * (Su' * Q_bar * Su + R_bar);
H_qp = (H_qp + H_qp') / 2;
f_qp = 2 * Su' * Q_bar * (Sx * x_hat + Sd - Y_ref_stack);

% Input bounds: u_min <= u(k+i) <= u_max for all i
lb = repmat(u_min, N, 1);
ub = repmat(u_max, N, 1);

% Rate constraint: |u(k+i) - u(k+i-1)| <= du_max
D = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1), -1), eye(nu));
A_ineq = [D; -D];
b_ineq = [du_max*ones(N*nu,1) + [u_prev; zeros((N-1)*nu,1)]; ...
          du_max*ones(N*nu,1) + [-u_prev; zeros((N-1)*nu,1)]];

opts = optimoptions('quadprog', 'Display', 'off');
U_opt = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], opts);

if isempty(U_opt)
    infeasible = true;
    warning('MPC: quadprog returned no solution, using u_prev');
    u_mpc = min(max(u_prev, u_min), u_max);
else
    infeasible = false;
    u_mpc = U_opt(1:nu);
end
end
