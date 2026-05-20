%% main.m

load_cart_pendulum_model;
run_generate_filters;

%%
run_dt_grad_ascent;
save('optimal_input.mat','U');

% Add SDP
Ugood = U;
run_sdp_relaxation;
U = Ugood;

%% Multiplier for horizon length
NN = 10;
nf = 3;
CYCLES = 10;

%% Now generate data. First, noiseless case, sine input
simulink_seed = 22321;
NOISE = 0;
load_system('simulation.slx');
set_param('simulation/Manual Switch','sw','0');
run_and_process_simulation;
fprintf('Relative error, noiseless: %.3f\n', norm(out.p(end,2:end)-c,2)/norm(c));  % To report
plots;
%matlab2tikz('fest_bad_u_noiseless.tex')

% With optimal input
set_param('simulation/Manual Switch','sw','1');
run_and_process_simulation;
fprintf('Relative error, noiseless, optimal input: %.3f\n', norm(out.p(end,2:end)-c,2)/norm(c));  % To report

NOISE = 1;
set_param('simulation/Manual Switch','sw','0');
run_and_process_simulation;
plots;
%matlab2tikz('fest_bad_u_noise.tex')

% save history of min_svs
s1_bad = s1;

% Now the good u
set_param('simulation/Manual Switch','sw','1');
run_and_process_simulation;
fprintf('Relative error, noisy, optimal input: %.3f\n', norm(mean(out.p(N*NN+1,2:end)-c))/norm(c));
plots;
%matlab2tikz('fest_good_u_noise.tex')

%% Plot comparison between svs
figure;
semilogy(t(N*NN+1:end),s1_bad,t(N*NN+1:end),s1);
xlabel('Time [s]');
ylabel('\sigma_{\min}(\boldsymbol{E})','Interpreter','latex');
legend('sine','optimal')
%matlab2tikz('svs.tex')



