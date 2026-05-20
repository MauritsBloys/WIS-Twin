% main_bounds.m

load_cart_pendulum_model;
run_generate_filters;
load('optimal_input.mat')
W = MatrixPolynomial([zeros(nz-ny,ny); eye(ny)]);
NW = N*W;
% Cx - y + w = 0 -> y = Cx + w

%% Lemma 1
[J, F] = lemma1(H, L, Hf, Lf, W, N, Hdagger);

%% Theorem 2
% System-based metrics
addpath("l1norm/");
peak_norm = max_peak_norm(J,den);

% Signal-based metrics
NN = 10;
N = 40;
h = 0.05;
nf = 3;
CYCLES = 10;
NOISE = 0;

simulink_seed = 22321;
load_system('simulation.slx');
set_param('simulation/Manual Switch','sw','1');
run_and_process_simulation;

xiz = [out.z(:,2:end), out.d(:,2:end), out.x(:,2:end)];
xiz_inf = norm(xiz,"inf");
sigma_minv = 1/sqrt(s1(end));
fnorm = norm(c,2)^2;
B = peak_norm*sqrt(N*NN)*nf*fnorm*sigma_minv*xiz_inf;
fprintf('Bias bound: %g\n', B);

%% Theorem 3
% System-based metrics
TF = [];
for i = 1:length(F)
    TF = [TF; tf(F{i},tf('s'))];
end
TF = TF/tf(den,1);
TFd = c2d(TF,h);
etaF = hinf_fro(TFd);

TW = tf(NW,tf('s'))/tf(den,1);
TWd = c2d(TW,h);
etaW = hinf_fro(TWd);

eta2 = etaF^2 + etaW^2;

% Signal-based metrics
NOISE = 1;

load_system('simulation.slx');
set_param('simulation/Manual Switch','sw','1');
run_and_process_simulation;

sig = 1./svd(Ei).^2;

term1 = 2*(dot(c,c)+1)*eta2*sum(sig);
term2 = B^2*etaF^2*(2*sum(sig)+sig(end));
trace_var_bound = NOISE^2*(term1 + term2);
fprintf('TraceVar bound: %g\n', trace_var_bound);

%% Squared error
exp_err = var(out.p(end-N*NN*5:end,2:end)) + (mean(out.p(end-N*NN*5:end,2:end)) - c).^2;
total_err = sum(exp_err);
fprintf('Observed summed squared error: %g\n', total_err);

%% First-order approximation conditions

gammaF = norm(TFd, 2);
noise_gamma = gammaF^2*NOISE^2;
noise_eta = etaF^2*NOISE^2;
inv_effective_richness = sig(end)*N*NN;
inv_richness = sig(end);
c1 = inv_effective_richness*noise_gamma;
c2 = inv_richness*noise_eta;
c3 = 2*c1;

fprintf('SNR: %.2f\n', c3);