%% main.m

load_cart_pendulum_model;
run_generate_filters;
load('inv_pendulum.mat','R','Ms','sys_nom','sys_fault')
%%
run_dt_grad_ascent;

%% Multiplier for horizon length
CYCLES = 10;
NN = 10;
nf = 3;

%% Now generate data. First, noiseless case, sine input
freq = 1;
simulink_seed = 22321;
system_name = 'simulation_ltv';
NOISE = 1;

% With optimal input
USE_SINE = 0;
load_system([system_name, '.slx']);
set_param([system_name, '/Manual Switch'],'sw','1');
run_and_process_simulation_ltv;

% plot
figure; 

subplot(121)
plot(out.p(N+1:end,1), out.p(N+1:end,2:end)); 
hold on;  set(gca,'ColorOrderIndex', 1);
plot([0 out.p(end,1)], [cc(1:2); cc(1:2)], '--'); 
sine_fault = 0.05*sin(freq*out.p(N+1:end,1)).*(out.p(N+1:end,1)>50);
plot(out.p(N+1:end,1), sine_fault,'--');
ylabel('Fault estimate'); 
ylim([-0.2, 0.2]); 
legend({'$\hat{f}_1$', '$\hat{f}_2$', '$\hat{f}_3$'  '$f_1$', '$f_2$', '$f_3$'}, 'Interpreter','latex','NumColumns',2); 
xlabel('Time [s]');

%% USING CORRECT REGRESSOR
USE_SINE = 1;
run_and_process_simulation_ltv;
out.p(:,end) = sin(freq*out.p(:,1)).*out.p(:,end);  % f = p*sin(wt)

subplot(122)
plot(out.p(N+1:end,1), out.p(N+1:end,2:end)); 
hold on; set(gca,'ColorOrderIndex', 1);
plot([0 out.p(end,1)], [cc(1:2); cc(1:2)], '--'); 
sine_fault = 0.05*sin(freq*out.p(N+1:end,1)).*(out.p(N+1:end,1)>50);
plot(out.p(N+1:end,1), sine_fault,'--');
ylabel('Fault estimate'); 
ylim([-0.2, 0.2]); 
legend({'$\hat{f}_1$', '$\hat{f}_2$', '$\hat{f}_3$'  '$f_1$', '$f_2$', '$f_3$'}, 'Interpreter','latex','NumColumns',2); 
xlabel('Time [s]');
%title('Response of the "best" input (square wave @ 0.9 rad/s)')

%
set(gcf(),'Position', [680 458 4*200 200])

%matlab2tikz('fest_ltv.tex')
