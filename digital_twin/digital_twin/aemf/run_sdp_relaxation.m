%% This is the SDP relaxation of the unit infinity norm constained problem


%% Optimize

cvx_begin sdp
    variable U(N*m, N*m) semidefinite;
    variable lbd;
    
    subject to

    for ientry = 1:m
        trace(U(N*(ientry-1)+1:N*ientry,N*(ientry-1)+1:N*ientry)) <= 4*N*bound^2;
    end

    M = zeros(p,p);
    for j = 1:N
        Pj = P_i(j);
        M = M + Pj*U*Pj';
    end

    M >= lbd*eye(p);

    maximize(lbd);
cvx_end
