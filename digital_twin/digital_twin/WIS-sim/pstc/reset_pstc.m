% init pstc

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
                MM, Wk, QQ, Rw, Rv, wQw, cv, cvw);
            
            
controller.connect();

% activate auto control
controller.sendMessage("205 0"); 

%%
controller.deactivate();

clear controller;
% w = 
% 
%   struct with fields:
% 
%     identifier: 'MATLAB:callback:error'
%          state: 'on'