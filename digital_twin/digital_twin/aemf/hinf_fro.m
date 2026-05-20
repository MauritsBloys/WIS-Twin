function nu = hinf_fro(T)

TT = T'*T;
n = length(TT);
trTT = TT(1,1);
for i = 2:n
    trTT = trTT + TT(i,i);
end
nu = sqrt(norm(trTT,"inf"));

