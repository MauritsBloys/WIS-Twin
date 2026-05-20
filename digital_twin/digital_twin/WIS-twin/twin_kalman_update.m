function [x_hat, P, innov] = twin_kalman_update(A, B, C, Q_kal, R_kal, x_hat, P, y_meas, u, d_leak)
%TWIN_KALMAN_UPDATE  One predict+update step of the discrete Kalman filter.
%
%   Inputs:
%     A, B, C  — state-space matrices (from comb_Pool_disc)
%     Q_kal    — process noise covariance
%     R_kal    — measurement noise covariance
%     x_hat    — prior state estimate (column vector)
%     P        — prior error covariance matrix
%     y_meas   — current measurement (column vector)
%     u        — current control input (column vector)
%     d_leak   — (optional) leakage correction vector [m per tijdstap]
%
%   Outputs:
%     x_hat    — updated state estimate
%     P        — updated error covariance
%     innov    — innovation (pre-correction residual)

if nargin < 10
    d_leak = zeros(size(A, 1), 1);
end

% Predict
x_prior = A * x_hat + B * u + d_leak;
P_prior = A * P * A' + Q_kal;

% Innovation
innov = y_meas - C * x_prior;

% Kalman gain
S = C * P_prior * C' + R_kal;
K = P_prior * C' / S;

% Update
x_hat = x_prior + K * innov;
IKC = eye(size(P,1)) - K * C;
P   = IKC * P_prior * IKC' + K * R_kal * K';
end
