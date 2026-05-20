function [xs, Ps, Pcs] = kalman_smoother(A, C, Q, R, y, x0, P0)
%KALMAN_SMOOTHER  Rauch-Tung-Striebel smoother.
%   xs  : nx x N         gesmoothe toestandsschatting x(k|N)
%   Ps  : nx x nx x N    gesmoothe covariantie P(k|N)
%   Pcs : nx x nx x N-1  kruiscovariantie P(k,k-1|N)

nx = size(A, 1);
N  = size(y, 2);

xf = zeros(nx, N);    Pf = zeros(nx, nx, N);
xp = zeros(nx, N);    Pp = zeros(nx, nx, N);
xf(:,1) = x0;  Pf(:,:,1) = P0;

% Voorwaartse Kalman pass
for k = 2:N
    xp(:,k)   = A * xf(:,k-1);
    Pp(:,:,k) = A * Pf(:,:,k-1) * A' + Q;
    S         = C * Pp(:,:,k) * C' + R;
    K         = Pp(:,:,k) * C' / S;
    xf(:,k)   = xp(:,k) + K * (y(:,k) - C * xp(:,k));
    IKC       = eye(nx) - K * C;
    Pf(:,:,k) = IKC * Pp(:,:,k) * IKC' + K * R * K';
end

% Achterwaartse RTS smoothing pass
xs = xf;  Ps = Pf;  Pcs = zeros(nx, nx, N-1);
G_prev = zeros(nx);
for k = N-1 : -1 : 1
    G         = Pf(:,:,k) * A' / Pp(:,:,k+1);
    xs(:,k)   = xf(:,k) + G * (xs(:,k+1) - xp(:,k+1));
    Ps(:,:,k) = Pf(:,:,k) + G * (Ps(:,:,k+1) - Pp(:,:,k+1)) * G';
    if k < N-1
        Pcs(:,:,k) = Pf(:,:,k)*G_prev' + G*(Pcs(:,:,k+1) - A*Pf(:,:,k))*G_prev';
    else
        S_N        = C * Pp(:,:,N) * C' + R;
        K_N        = Pp(:,:,N) * C' / S_N;
        Pcs(:,:,k) = (eye(nx) - K_N*C) * A * Pf(:,:,N-1);
    end
    G_prev = G;
end
end
