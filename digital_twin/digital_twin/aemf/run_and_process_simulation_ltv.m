%% sim
cc = c;
out = sim(system_name);
E = out.e(:,2:end);
t = out.r(:,1);
r = out.r(:,2:end);

%%
s1 = zeros(length(t)-N,1);
fhat = zeros(length(t)-N,nf);
for i = 1:length(s1)
    Ei = E(i:i+N-1,:);
    Ri = r(i:i+N-1,:);
    s1(i) = min(svd(Ei))^2;
    fhat(i,:) = pinv(Ei)*Ri;
end

