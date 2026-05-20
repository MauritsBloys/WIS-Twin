function [status, Ph, mu] = checkheemelslmiOF(Ac, Bc, Cc, Dc, Ap, Bp, Bw, Cp, Q, h, rho, gamma)
%CHECKHEEMELSLMI Verifies if output feedback Periodic Event-Triggered
%Control for LTI plant <Ap,Bp,E> with and triggering
%mechanism xi'*Q*xi > 0 renders the system GES when sampling time h is
%used, with rho decay rate and gamma L2-gain from disturbance to output.
%   
%   [status, Ph, mu] = CHECKHEEMELSLMI(Ap, Bp, E, K, Q, h, rho, gamma) <---
%
%   We assume a system of the form described in Section V of the paper
%   mentioned below.
%
%   Outputs:
%       - status: whether the correspoding LMIs are feasible
%       - Ph: the matrix solution to the LMIs, corresponding to the
%       Lyapunov matrix P(tau), tau = t - t_k, at tau = h.
%       - mu: (vector of) auxiliary variables.
%
% CVX IS REQUIRED: DOWNLOAD IT AT http://cvxr.com/cvx/
%
% This is an implementation of part of the work at Heemels et al (2013):
% Heemels, W., Donkers, M., and Teel, A.R. (2013). Periodic event-triggered
%     control for linear systems. Automatic Control, IEEE Transactions on,
%     58(4), 847-861.
%
% Author of the code: Gabriel de Albuquerque Gleizer (Apr-2018)
% Modified to use for the output feedback case by Jacob Lont (jul-2020)

% heemels LMIs
SMALL_NUMBER = 0;%1e-6;  % To ensure strict inequalities in the LMIs.
nc = size(Ac,1); 
nDc = size(Dc, 2); % width of the matrix Dc
np = size(Ap,1);
nw = size(Bw,2); % width of the matrix Bw
nBp = size(Bp,2); % width of the matrix Bp
mCp = size(Cp,1); % height of the matrix Cp
mCc = size(Cc,1); % height of the matrix Cc
nQ = size(Q,1); % width (and height) of the matrix Q


%% Build matrices
m_Abar = np+nc+mCp+nBp; % height of the matrix Abar (it has to be square)
Abar = [Ap,        zeros(np,nc), zeros(np,mCp), Bp;
        zeros(m_Abar-np,np+nc+mCp+nBp)];

Bbar = [Bw; 
        zeros(m_Abar-np, nw)];

J1 = [eye(np),  zeros(np, nc),  zeros(np,nDc),  zeros(np,mCc);
      Bc*Cp,    Ac,             Bc*0,           zeros(nc,mCc);
      Cp,       zeros(mCp,nc),  zeros(mCp,nDc), zeros(mCp,mCc);
      zeros(mCc,np), Cc,        Dc,             zeros(mCc,mCc)];  
% Jacob: J1 should be square based on the equations

% J0 = ... % Should be defined for the empty set <--------
% J0 is defined with Gamma_u = 0 and Gamma_y = 0
J0 = [eye(np),          zeros(np, nc),  zeros(np,nDc),   zeros(np,mCc);
      zeros(nc,np),     Ac,             Bc,              zeros(nc,mCc);
      zeros(mCp,np),    zeros(mCp,nc),  eye(mCp),        zeros(mCp,mCc);
      zeros(mCc,np),    Cc*0,           Dc*0,            eye(mCc)];  

% J1 = [eye(np), zeros(np); eye(np), zeros(np)]; % Gabriels version
% J2 = [eye(np), zeros(np); zeros(np), eye(np)]; % Gabriels version

% Gabriel: Use Cpxp istead of y_hat in the performance output <----
Cbar_yi = [1 0 0 0];
Cbar_y = blkdiag(Cbar_yi, Cbar_yi, Cbar_yi, Cbar_yi, Cbar_yi);
Cbar_u_hat = eye(5);
Cbar = [Cbar_y,     zeros(mCp, m_Abar-size(Cbar_y,2));
        zeros(mCc,  m_Abar-size(Cbar_u_hat,2)), Cbar_u_hat];

nz = size(Cbar,1);
Dbar = zeros(nz,nw);

% Build more matrices
M = inv(eye(nw) - 1/(gamma*gamma) * (Dbar'*Dbar));
L = inv(gamma*gamma*eye(nz) - (Dbar*Dbar'));
% H11 = Abar + rho*eye(2*np) + 1/(gamma*gamma) * Bbar*M*Dbar'*Cbar;
H11 = Abar + rho*eye(m_Abar) + 1/(gamma*gamma) * Bbar*M*Dbar'*Cbar;
H12 = Bbar*M*Bbar';
H21 = -Cbar'*L*Cbar;
H = [H11, H12; H21, -H11'];

% We should check if the first 2*np block of expm(-H*t) is invertible for
% all t in [0,h]. It looks like, for this case, it is. (For h ~< 1)

Fh = expm(-H*h); % h = \tau in the paper.
% F11 = Fh(1:2*np, 1:2*np); % F11bar
F11 = Fh(1:m_Abar, 1:m_Abar);
% F12 = Fh(1:2*np, (2*np+1):end);
F12 = Fh(1:m_Abar, (m_Abar+1):end);
% F21 = Fh((2*np+1):end, 1:2*np); % F21bar 
F21 = Fh((m_Abar+1):end, 1:m_Abar); % F21bar 
% F22 = Fh((2*np+1):end, (2*np+1):end);
F22 = Fh((m_Abar+1):end, (m_Abar+1):end);
iF11 = inv(F11);
SS = -F11\F12;

try
    Sbar = chol(SS);
catch
    warning('Due to numerical reasons, -F11\F12 is not symmetric positive definite');
    SS = (SS + SS')/2;
    mineig = min(eig(SS));
    while mineig < eps
        SS = SS - (mineig-eps)*eye(size(SS,1));
        mineig = min(eig(SS));
    end
    Sbar = chol(SS);
end
% Sbar = Sbar'; % Does nothing?

[U,S,V] = svd(-F11\F12);
Sbar = U*sqrt(S)*V';

% Qh = Q(1:2*np, 1:2*np);
% Ih = eye(2*np);
% Zh = zeros(2*np);
RF = F21*iF11;
RF = (RF+RF')/2;
mineig = min(eig(RF));
while mineig < 0
    RF = RF - mineig*eye(size(RF,1));
    mineig = min(eig(RF));
end

% Dimensions of Ph: = dim(Q)


% Build LMIs!
cvx_begin sdp
  cvx_precision low
  variable mu1 nonnegative
  variable mu2 nonnegative
%   variable Ph(2*np,2*np) symmetric
  variable Ph(nQ, nQ) semidefinite %symmetric
  [Ph - mu1*Q + zeros(nQ), J1'*iF11'*Ph*Sbar, J1'*(iF11'*Ph*iF11 + RF);
  Sbar'*Ph'*iF11*J1, eye(size(Sbar,1))-Sbar'*Ph*Sbar, zeros(size(Sbar,1),size(RF,2));
  (iF11'*Ph*iF11 + RF)'*J1, zeros(size(Sbar,1),size(RF,2))', (iF11'*Ph*iF11 + RF)] >= SMALL_NUMBER*eye(3*m_Abar);
% The LMI for J0 (the empty set)
  [Ph + mu2*Q , J0'*iF11'*Ph*Sbar, J0'*(iF11'*Ph*iF11 + RF);
  Sbar'*Ph'*iF11*J0, eye(size(Sbar,1))-Sbar'*Ph*Sbar, zeros(size(Sbar,1),size(RF,2));
  (iF11'*Ph*iF11 + RF)'*J0, zeros(size(Sbar,1),size(RF,2))', (iF11'*Ph*iF11 + RF)] >= SMALL_NUMBER*eye(3*m_Abar);
%
%
%   Ph >= SMALL_NUMBER*eye(m_Abar);
%   mu1 >= 0;
%   mu2 >= 0;
cvx_end

mu = [mu1; mu2];
% mu = mu1;

% Extract solution
if strcmp(cvx_status,'Infeasible')
    status = false;
elseif strcmp(cvx_status,'Solved')
    status = true;
else
    error('I wasn''t expecting %s status', cvx_status);
end

