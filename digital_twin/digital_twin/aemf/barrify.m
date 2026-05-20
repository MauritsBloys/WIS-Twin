function Hb = barrify(H, m)
%BARRIFY Creates \bar{H(q)} of row order m
    n = length(H);
    if nargin < 2
        m = n-1;
    end
    H{n+1} = 0*H{n};
    vc = [1, (n+1)*ones(1,m)];
    vr = [1:n, (n+1)*ones(1,m)];
    Hb = cell2mat(H(toeplitz(vc,vr)));
end