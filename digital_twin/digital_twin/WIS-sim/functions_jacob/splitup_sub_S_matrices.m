function [S1BV, S1BY, S1CW, S1CU, S1DVW, S1DVU, S1DYU, S1DYW] = splitup_sub_S_matrices(S1A, S1B, S1C, S1D)
% Function used to splitup the discrete-time state-space matrices of the
% controller. This is done to add wi to the state vector in the end, which
% enables us to meet the format used in Heemels 2013 output feedback case.

% Author: Jacob Lont
% Date: 21-07-2020

    S1BV = S1B(:,1); 
    S1BY = S1B(:,2);

    S1CW = S1C(1,:); 
    S1CU = S1C(2,:);

%     S1DV = S1D(:,1); 
%     S1DY = S1D(:,2);
    S1DVW = S1D(1,1); % SiDVW (from input v to w)
    S1DVU = S1D(2,1); % From input v to u 
    S1DYU = S1D(2,2); % From input y to u
    S1DYW = S1D(1,2); % From input y to w


end