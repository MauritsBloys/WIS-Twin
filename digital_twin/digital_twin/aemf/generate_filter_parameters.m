%load('now.mat')
function [Hbar, G, Hdagger, degree] = generate_filter_parameters(H, L, Hf, Lf, order)
%GENERATE_FILTER_PARAMETERS Create main matrices for fault estimation
%   [Hbar, G, Hdagger, degree] = GENERATE_FILTER_PARAMETERS(H, L, Hf, Lf,
%   order) produces Hbar, a Toeplitz matrix encoding H(q), Hdagger, a left
%   inverse of H, and a polynomial matrix G as in Lemma 3.1 of the paper.

%   Author: Gabriel de Albuquerque Gleizer, 2024

nn = H.Size(1);
nx = H.Size(2);
nz = L.Size(2);

%% Build standard bar matrices
Hb = H.barrify(order-1); 
IO = [eye(nx), zeros(nx, size(Hb,2)-nx)];
Hdagger = lsqminnorm(Hb',IO')';
Hdagger(abs(Hdagger) < eps) = 0; 
order_Hdagger = size(Hdagger,2)/nn - 1;  % TODO: order of H^dagger can be specified independetly
Hdagger = MatrixPolynomial(Hdagger, order_Hdagger);

% Build matrix that we want to anihilate
Hbar = H.barrify(order);

%% Linear generators of H
G = {};
for j = 1:length(Hf)
    Hj = Hf(j);
    Lj = Lf(j);
    HdHbj = Hj*Hdagger;
    Gj = Lj - HdHbj*L;
    G{j} = Gj;
end

degree = order;

