function [dk, k, xc, xptilde, X, initialized, psibar] = ...
    sleepcontroller(...
        y, triggered, uhat, ...  % Plant output and triggered flag
        k, xc, xptilde, X, initialized, psibar, ...
        kfinal, kbar, TRIG_LEVEL, ...
        np, nc, pp, mp, nw, ppt, ...
        Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
        Obsbar, Vbar, V, ...
        MM, Wk, QQ, Rw, Rv, wQw, cv, cvw)
%SLEEPCONTROLLER computes the control action, initializes / updates the 
%state observer, and computes a sleeping time to the sensors.
%
%   [u, dk, k, xc, xptilde, X, initialized, psibar] = ...
%     SLEEPCONTROLLER(...
%         y, triggered, ...  % Plant output and triggered flag
%         k, xc, xptilde, X, initialized, psibar, ...
%         kfinal, kbar, TRIG_LEVEL, ...
%         np, nc, pp, mp, nw, ppt, ...
%         Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
%         Obsbar, Vbar, V, ...
%         MM, Wk, QQ, Rw, Rv, wQw, cv, cvw)
%
%   Inputs:
%       external inputs
%           y in R^pp  (sensor readings)
%           triggered (boolean), true if a sensor has triggered
%       states:
%           k (int), an internal clock 
%           xc in R^nc, the controller states
%           xptilde in R^np, the center of the plant's state estimate
%           X in R^(np x np), the state estimate ellipsoid shape matrix
%           initialized (bool), true if the observer has initialized
%           psibar in R^(np*kbar), vector of outputs for the observer
%               initialization
%       parameters:
%           all the rest (see init_ files for the definition)
%
%   Outputs:
%       u in R^mp, the control command
%       dk (int, >= 0) the scheduled sleep time. 0 if none was calculated.
%       the states (as in "Inputs")
%
% 
%   Author: Gabriel de A. Gleizer, Jun 2021 (g.gleizer@tudelft.nl)


% Controller
u = Cc*xc + Dc*y;
xc = Ac*xc + Bc*y;
% FIXME: This will only work because this controller is stateless
% Otherwise, we would need a uhat as well

if ~triggered && initialized  % No update on output
    dk = 0;
end

% Main "PSTC" algorithm here

if ~initialized
    dk = 1;  % Trivial sleep time
    % TODO: Adapt this because now we're triggering with ETC
    
    % Initialization routine
    
    % Collect outputs
    psibar(1 + k*pp : (k+1)*pp) = y;
    
    % Iterate clock
    k = k + 1;
    
    % Stopping criteria: Obs is full rank at this point
    if k == kbar+1
        % Compute first state estimate
        xptilde = Obsbar\psibar;
        X = Obsbar\Vbar/Obsbar';
        X = (X+X')/2;
        X = ell_regularize(X);
        
        initialized = true;
        k = 0;  % Reset clock
    else
        % Update psitilde values to next time (\tilde{\psi}(0,k))
        for ii = 0:k-1
            psibar(ii*pp+1:(ii+1)*pp) = psibar(ii*pp+1:(ii+1)*pp) +...
                Cp*Phip^(ii-k)*Gammap*u;
        end
    end
elseif triggered   
    % protection against exceeding the heartbeat
    if (k > kfinal)
        % This happens when etc does not trigger within the heartbeat
        disp('Re-initializing (k > kfinal)')
        dk = 1;
        k = 0;
        initialized = false;
        return
    end
    
    % Get transition matrix
    Mk = MM(:,:,k);  % Current triggering time is k
    Wkk = Wk(:,:,k);     
    Mknn = Mk(1:np,1:np);  % the same as Phip(dk)
    
    % Algorithm 1, line 11
    % xptilde = Mkn*ptilde  % Already updated at previous iteration
    % (instead, do everything here)
    for ii = 1:k
        xptilde = Phip*xptilde + Gammap*uhat;
    end
    X = Mknn*X*Mknn';
    X = (X + X')/2;
    X = ell_regularize(X);
    
    % Algorithm 1, line 12
    pstar = sqrt(trace(X))/sqrt(trace(Wkk));
    if pstar == 0
        X = Wkk;
    elseif ~isinf(pstar)
        X = (1 + 1/pstar)*X + (1 + pstar)*Wkk;
    end
    X = (X+X')/2;
    
    % Algorithm 1, line 2: fusion
    [xptilde,X] = ellobserverintersection(xptilde, y, X, V, Cp);
    X = (X+X')/2;
    
    if any(eig(X) < 0)
        % This happens when the disturbance is bigger than we thought.
        % In this case, reset to non-initialized and sleep = 0.
        disp('Re-initializing')
        dk = 1;
        k = 0;
        initialized = false;
        return
    end
    
    % PSTC triggering mechanism
    ptilde = [xptilde; xc; y];  % Algorithm 1, line 3
    
    for dk = 1:kfinal  % Algorithm 1, lines 4--10        
        % Matrices indexing
        Qk = QQ(:,:,dk);
        Qkn = Qk(:,1:np);
        Qknn = Qk(1:np,1:np);
        
        Rwdk = Rw(:,:,dk);
        Rvdk = Rv(:,:,dk);
        
        % Compute etabar (Theorem 3)
        xQx = ptilde'*Qk*ptilde;
        Qp = Qkn'*ptilde;
        xQe = sqrt(Qp'*X*Qp);
        eQe = eigs(X*Qknn,1,'largestreal');
        xQw = sqrt(ptilde'*Rwdk*ptilde);
        eQw = sqrt(eigs(Rwdk(1:np,1:np)*X,1));
        xQv = sqrt(ptilde'*Rvdk*ptilde);
        eQv = sqrt(eigs(Rvdk(1:np,1:np)*X,1));
        
        etabar = xQx + 2*xQe + eQe + 2*xQw + wQw(dk) + 2*eQw + cv...
            + 2*cvw(dk) + 2*xQv + 2*eQv;         

        if etabar > TRIG_LEVEL  % epsilon^2 in the paper
            break;
        end
    end
    
    % Reset clock
    k = 0;
end
    
if initialized
    k = k + 1;
end







