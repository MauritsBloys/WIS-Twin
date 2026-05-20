% -------------------------------------------------------------------------
% Transcription and adaptation from Python by Segher de Reij (14-02-2023)
%
% Modified by Gabriel Gleizer (06-2023, 06-2024)

close all

%% Build Bar matrices:

% Set the order of the filter
order = 0;

% Get H_bar, F_bar and L_bar:
Hb = H.barrify(order);
Lb = L.barrify(order);
IO = [eye(nx), zeros(nx, size(Hb,2)-nx)];
Hdagger = lsqminnorm(Hb',IO')';

while norm(Hdagger*Hb - IO,"fro") > 1e-10 || isempty(null(Hb'))
    order = order + 1;
    if order >= 10          
        error('Observability criterion failed');
    end  
    Hb = H.barrify(order);
    Lb = L.barrify(order);
    IO = [eye(nx), zeros(nx, size(Hb,2)-nx)];
    Hdagger = lsqminnorm(Hb',IO')';
end

%% Build linear filters:

[R,G,Hdagger,~] = generate_filter_parameters(H, L, Hf, Lf, order);
NR = null(R')';
% mmm = [NR(1,:)*Mr{1}; NR(1,:)*Mr{2}; NR(1,:)*Mr{3}]
% assert that:
% min(svd(mmm)) big enough
Nb = NR(1,:);


%% Generate M(s)
N = MatrixPolynomial(Nb, order);
M = cellfun(@(x) clean(N*x), G);
degree = max([M.Degree]);

%% Get Residual Filter:
% -------------------------------------------------------------------------
% R(s) = a^-1(s)*N(s)*L(s) 
% L(s) contains all the measurements and the inputs. (known signals)
% N(s) is the designed linear filter.
% a(s) is designed to make the filter proper and stable.
% -------------------------------------------------------------------------

% N(s)*L(s):
NL = clean(N*L);  % clean removes least significant zeros

% Design a(s) so the TF's are proper and stable:
den = get_denominator(degree+1);

% R(s) = a(s)^1*N(s)*L(s):
R = tf(NL,tf('s'))/tf(den,1);

%% Build pre-filter
Ms = [];
for i = 1:length(M)
    Ms = [Ms; tf(M(i),tf('s'))];
end
Ms = Ms/tf(den,1);

save('inv_pendulum.mat','-mat','-append');


%% 
function den = get_denominator(npoles)

    % Choose the desired pole locations in the left-half plane
    poles = -10 * (1:npoles);

    % Create the denominator polynomial
    den = poly(poles);

    % Normalize the denominator coefficients
    den = den / den(end);
end
