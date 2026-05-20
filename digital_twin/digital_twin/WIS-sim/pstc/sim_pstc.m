% SIM init pstc

%% init
clear all; % TODO: only clear controller?
init_plant;
init_etc;
init_pstc;


init_simulation;


%% create controller communication object
controller = FireflyCommunicationPSTC("/dev/cu.SLAB_USBtoUART10", ...
                k, xc, xptilde, X, initialized, psibar, ...
                kfinal, kbar, TRIG_LEVEL, ...
                np, nc, pp, mp, nw, ppt, ...
                Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
                Obsbar, Vbar, V, ...
                MM, Wk, QQ, Rw, Rv, wQw, cv, cvw, ...
                h, xp);
            
            
controller.runSimulation(noises, TEND, yhat, triggered, uhat, odeplant, omega, sigma, Ap, Bp, Cp, Dp);

%%
%controller.deactivate();


% w = 
% 
%   struct with fields:
% 
%     identifier: 'MATLAB:callback:error'
%          state: 'on'