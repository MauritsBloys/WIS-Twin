function [Aci, Bci, Cci, Dci] = create_combined_control_sub_matrices(ssid, i, nPool)
% Small function to create the sub-A-matrices of the combined controller
% (distributed controllers in centralized form)
% Author: Jacob Lont
% Date: 21-07-2020
% Make sure to use the discrete-time version of the controller for all
% matrices.
% The constructed matrices all form blocks used as rows in the combined 
% controller matrices. The cause of this is the offset of 5 states needed
% for the definition of wi in the state vector, which prevents us from
% using blkdiag for example.

    % use 5 as default so Jacob's scripts still work without the nPool arg
    if nargin < 3
        nPool = 5;
    end

    [SiBV, SiBY, SiCW, SiCU, SiDVW, SiDVU, SiDYU, SiDYW] = splitup_sub_S_matrices(ssid.A, ssid.B, ssid.C, ssid.D);
    
    nc = size(ssid.A,1); % # states of the control state matrix
    nB = size(SiBV,2); % width of this matrix and thus of D (should be 1)
    hc = size(SiCU,1); % height of the c-matrices
    
    offset = 5; % the offset from wi to wi+1 for i = [1,..,4]
    
    
    n_tot = nPool * offset; % total number of states: 5*4*xi + 5 * wi = 25

    % because w2 is a state used for w1, we need to skip 5 states using 0's
    if (i == 1)
        rest_width = n_tot-nc-offset-nB;
        SiCW = 0*SiCW; S1DVW = 0*SiDVW; % As w1K=w6K=0 always.
        Aci = [ssid.A, zeros(nc,offset), SiBV, zeros(nc, rest_width); 
               SiCW, zeros(hc,offset), SiDVW, zeros(hc, rest_width)];
        Cci = [SiCU, zeros(hc,offset), SiDVU, zeros(hc, rest_width)];
    elseif i<nPool
        rest_width = n_tot-offset*(i-1)-nc-offset-nB; % should be >= 0
        Aci = [zeros(nc, offset*(i-1)), ssid.A, zeros(nc,offset), SiBV, zeros(nc, rest_width); 
               zeros(hc, offset*(i-1)), SiCW, zeros(hc,offset), SiDVW, zeros(hc, rest_width)];
        Cci = [zeros(hc, offset*(i-1)), SiCU, zeros(hc,offset), SiDVU, zeros(hc, rest_width)];
    elseif (i == nPool) % NOT DONE YET
        % For w5, we need w6, but chosen to set w1=w6=0.
        rest_width = n_tot-nc-nc-nB-1;

        Aci = [zeros(nc,nc), SiBV, zeros(nc, rest_width), ssid.A, zeros(nc,1);
               zeros(hc,nc), SiDVW, zeros(hc, rest_width), SiCW, zeros(hc,1)];
        Cci = [zeros(hc,nc), SiDVU, zeros(hc, rest_width), SiCU, zeros(hc,1)];
    end
    
    
    % NOT IN BLOCK-ROW FORM:
    Bci = [SiBY; SiDYW];
    Dci = SiDYU;
    
end

























