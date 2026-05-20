function [SiA, SiB, SiC, SiD, ssi] = splitup_S_matrix(Si)
% Author: Jacob Lont
% Date: 05-06-2020
% Most recent revision: 05-06-2020

% Function to extract A,B,C,D matrices from the constructed S-matrices
% Instead of doing this manually in simulink

    SiA = Si(1:4, 1:4);
    SiB = Si(1:4, 5:6);
    SiC = Si(5:6, 1:4);
    SiD = Si(5:6, 5:6);
    
    ssi = ss(SiA, SiB, SiC, SiD); % Also create a state-space model


end