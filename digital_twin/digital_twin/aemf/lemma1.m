% lemma1
function [J, F] = lemma1(H, L, Hf, Lf, W, N, Hdagger)
    m = length(Hf);
    J = cell(m);
    F = cell(1,m);
    for i = 1:m
        NHF = clean(-N*Hf(i));
        NHFHd = NHF*Hdagger;
        F{i} = clean(NHFHd*W);
        for j = 1:m
            JH = clean(NHFHd*Hf(j));
            JL = clean(NHFHd*Lf(j));
            J{i,j} = [JH, JL];
        end
    end
end
        