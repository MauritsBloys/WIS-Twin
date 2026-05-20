%% Parameters
PIECEWISE_CONTINUOUS = true;

%% Ellipsoidal reachability for computation of \mathcal{X}_w

fluctuation = 5/100;  % 1% about steady state disturbance
Wel = W_MAG*fluctuation*ell_unitball(nw);  % Ellipsoidal Toolbox command for a ball.
if PIECEWISE_CONTINUOUS
    [Ad1, Ed1] = c2d(Ap, E, h);
    sys = linsys(Ad1, Ed1, Wel, [], [], [], [], 'd');
else
    sys = linsys(Ap,E,Wel);  % Ellipsoidal Toolbox command
end

X0 = 0.0001*ell_unitball(np);  % Initial state set, should be the origin.
TINTV = [0,kfinal*h];  % Time interval to compute reachability
L0 = eye(np);  % Support vectors for tight approximation

reachoptions.approximation = 0;  % external (outer) approximation

% Alternative approach: update X0 every h time units
Wk = zeros(np,np,kfinal);
for kk = 1:kfinal
    if PIECEWISE_CONTINUOUS        
        Xn = Ad1*X0;
        Wn = Ed1*Wel;
        pstar = sqrt(trace(Xn))/sqrt(trace(Wn));
        if pstar > 0 && ~isinf(pstar)
            Xn = (1 + 1/pstar)*parameters(Xn)...
                + (1 + pstar)*parameters(Wn);
        end
        X0 = ellipsoid(zeros(np,1), Xn);
    else
        RS = reach(sys, X0, L0, [0, h], reachoptions);  % Ellipsoidal Toolbox
        RC = cut(RS, h);  % Gets slice of the tube at instant h
        EAC = get_ea(RC);  % Get the array of ellipsoids
        traces = zeros(np,1);
        for ii = 1:np
            traces(ii) = trace(EAC(ii));
        end
        [~,imin] = min(traces);
        %X0 = EI;  % Update initial ellipsoid for the next iteration
        X0 = EAC(imin);
    end
    Wk(:,:,kk) = parameters(X0);  % Extract W_\kappa
end

%% Transition matrices
% Compute transition matrices:
%   M(\kappa) such that \xi_p(k+\kappa) = M(\kappa)[xp;xc;y]
%   N(\kappa) such that \zeta(k+\kappa) = M(\kappa)[xp;xc;y]

% First the more obvious CE: [y;u] = CE[xp;xc;y]
CE = [zeros(nz,np), [zeros(pp,nc), eye(pp); Cc, Dc]];

% Fundamental transition matrices
Abar = [Ap, Bp; zeros(mp,np+mp)];
Phibar = expm(Abar*h);
Phip = Phibar(1:np,1:np);
Gammap = Phibar(1:np,np+1:end);

% Auxiliary matrices
I0 = [eye(np),zeros(np,nc)];
OI = [zeros(nc,np), eye(nc)];

% Loop to compute Mks
Phipk = Phip;
Gammapk = Gammap;
Ack = Ac;
Bck = Bc;
KMAX = kfinal;

try
    clear M N;
end
for k = 1:KMAX
    M1 = [Phipk, Gammapk*Cc, Gammapk*Dc];
    M2 = [zeros(nc,np), Ack, Bck];
    N1 = Cp*[Phipk, Gammapk*Cc, Gammapk*Dc];
    N2 = [zeros(mp,np), Cc*Ack, Cc*Bck + Dc];
    M{k} = [M1; M2];
    N{k} = [N1; N2];

    Phipk = Phip*Phipk;
    Ack = Ac*Ack;
    
    Gammapk = Gammap + Phip*Gammapk;
    Bck = Bc + Ac*Bck;
end

%% Conic equation matrices (Q_\kappa)
try
    clear Qq
end

for k = 1:KMAX
   Qq{k} = [N{k}', CE']*Q*[N{k}; CE];
   lambda = eig(Qq{k});
   maxeig(k) = max(real(lambda));
   mineig(k) = min(real(lambda));
   % If Qk > 0, every state will have triggered up to here. Can stop.
   if mineig(k) > 0  
       break;
   end
end

kbeg = find(maxeig > 0, 1, 'first');
kend = find(mineig > 0, 1, 'first');
if isempty(kend)
    kend = KMAX;
end

dkmax = min(kend,KMAX);

% Turn cell array into a 3-array
nh = np+nc+pp;
QQ = reshape(cell2mat(Qq),nh,nh,dkmax);
%MM = reshape(cell2mat(M),np,nh,dkmax);
% TODO: check with Gabriel if this is correct
MM = reshape(cell2mat(M),np+nc,nh,dkmax);
%% Get auxiliary matrices and offline computable vectors
% Eq. (21) in the paper

Cw = [Cp; zeros(nz+mp,np)];
Cv = [eye(pp); zeros(nz+mp,pp)];

% Pre-allocate kappa-dependent variables
Fv = zeros(nh,pp,kfinal);
Fw = zeros(nh,np,kfinal);
cvw = zeros(kfinal,1);
wQw = zeros(kfinal,1);
Rw = zeros(nh,nh,kfinal);  % Fw W Fw'
Rv = zeros(nh,nh,kfinal);  % Fv V Fv'

% Compute kappa-independent variables
Qv = Cv'*Q*Cv;
Qw = Cw'*Q*Cw;
cv = eigs(V*Qv,1);

% Auxiliary matrix
Qwv = Cw'*Q*Cv;

% Compute kappa-dependent variables
for kk = 1:kfinal
    Fv(:,:,kk) = [N{kk}; CE]'*Q*Cv;
    Fw(:,:,kk) = [N{kk}; CE]'*Q*Cw;  
    Rw(:,:,kk) = Fw(:,:,kk)*Wk(:,:,kk)*Fw(:,:,kk)';
    Rv(:,:,kk) = Fv(:,:,kk)*V*Fv(:,:,kk)'; 
    wQw(kk) = eigs(Wk(:,:,kk)*Qw,1);
    cvw(kk) = sqrt(eigs(V*(Qwv'*Wk(:,:,kk)*Qwv), 1));
end

%% Initialization phase (Appendix C)

% Change to include controller states
Phifull = [Phip + Gammap*Dc*Cp, Gammap*Cc;
           Bc*Cp,               Ac];
Cfull = [Cp, zeros(pp,nc); zeros(nc, np), eye(nc)];

% Initialize intersection of ellyptical cylinders
% Determine kbar
Obs = [];
kbar = 0;
while true
    Obs = [Cp; Obs];
    if rank(Obs) >= np
        break;
    end
    Obs = Obs*Phip;
    kbar = kbar + 1;
end

Obsbar = Obs*Phip^(-kbar);

% Data gathered for initialization. Compute ellipsoid now
% -- this is an offline phase, could have been done above --
% 1. Build Vtilde(k) (Lemma 4 + minkowski sum)
tic;
for ii = 0:kbar-1  % No disturbance for kbar
    Vtildek = V;
    CC = Cp*Phip^(kbar-ii);
    Wkk = Wk(:,:,kbar-ii);
    CW = CC*Wkk*CC';
    
    % Minkowski sum outer-approximation
    pstar = sqrt(trace(Vtildek))/sqrt(trace(CW));
    if pstar == 0
        Vtildek = CW;
    elseif ~isinf(pstar)
        Vtildek = (1 + 1/pstar)*Vtildek + (1 + pstar)*CW;
    end

    Vtildek = (Vtildek+Vtildek')/2;
    Vtilde(ii*pp+1:(ii+1)*pp,:) = Vtildek;
end

Vtilde(kbar*pp+1:(kbar+1)*pp,:) = V;

%2. Compute Vbar (Theorem 4)
Vbar = zeros(pp*(kbar+1));
for ii = 0:kbar
    Vbar(ii*pp+1:(ii+1)*pp,ii*pp+1:(ii+1)*pp)...
        = (kbar+1)*Vtilde(ii*pp+1:(ii+1)*pp,:);
end