function A = max_peak_norm(J, den)

[n, m] = size(J);

A = 0;

for i = 1:n
    for j = 1:m
        Tj = tf(J{i,j},tf('s'))/tf(den,1);
        Aij = 0;
        for k = 1:length(Tj)
            if Tj(k).Numerator{1} == 0
                continue;
            else
                Aij = Aij + l1norm(Tj(k));
            end
        end
        A = max(A, Aij);
    end
end
