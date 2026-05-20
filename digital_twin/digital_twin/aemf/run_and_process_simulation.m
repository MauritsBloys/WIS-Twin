cc = c;
out = sim('simulation.slx');
E = out.e(:,2:end);
t = out.z(:,1);
r = out.r(:,2:end);

%%
s1 = zeros(length(t)-N*NN,1);
fhat = zeros(length(t)-N*NN,nf);
for i = 1:length(s1)
    Ei = E(i:i+N*NN-1,:);
    Ri = r(i:i+N*NN-1,:);
    s1(i) = min(svd(Ei))^2;
    fhat(i,:) = pinv(Ei)*Ri;
end

