% init pstc

%% init
clear all; % TODO: only clear controller?
init_plant;
init_etc;
init_pstc;
init_simulation;

%% Load identification results
try
    load('../identification/identification.mat');
catch
    assert(false, "File 'identification.mat' does not exist. Run identification first.");
end


%% create controller communication object
simulation = FireflySimulationPSTC("/dev/cu.SLAB_USBtoUART", "/dev/cu.SLAB_USBtoUART4", "/dev/cu.SLAB_USBtoUART6", "/dev/cu.SLAB_USBtoUART8", ...
                k, xc, xptilde, X, initialized, psibar, ...
                kfinal, kbar, TRIG_LEVEL, ...
                np, nc, pp, mp, nw, ppt, ...
                Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
                Obsbar, Vbar, V, ...
                MM, Wk, QQ, Rw, Rv, wQw, cv, cvw, ...
                h, xp, Wis);
            
%%        
simulation.connect();

%%
%controller.deactivate();


% w = 
% 
%   struct with fields:
% 
%     identifier: 'MATLAB:callback:error'
%          state: 'on'