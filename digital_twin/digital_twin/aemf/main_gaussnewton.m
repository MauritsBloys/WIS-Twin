%% main.m

% Step 1. Initialize filters and compute optimal input
load_cart_pendulum_model_very_weak_motor;
run_generate_filters;
load('inv_pendulum.mat','R','Ms','sys_nom')
load('inv_pendulum_very_weak_motor.mat','sys_fault')

run_dt_grad_ascent;

% Multiplier for horizon length
NN = 10;
nf = 3;

% Loop through simulations
Nsim = 10;
ffinal = zeros(1,nf);
simulink_seed = 22322;

fhathist = zeros(Nsim+1,nf);
CYCLES = 2;
for isim = 1:Nsim
    %% Now generate data. First, noiseless case, sine input
    NOISE = 1;
    load_system('simulation.slx');
    set_param('simulation/Manual Switch','sw','1');
    run_and_process_simulation;

    % Step 2. obtain and display fault estimates
    fhat = out.p(end, 2:end);

    % Step 3. Update H, L based on fault estimates
    for i = 1:nf
        H = H + fhat(i)*Hf(i);
        L = L + fhat(i)*Lf(i);
    end

    % Step 4. Update residual and regressor generators
    Nhorizon = N;
    run_generate_filters;
    N = Nhorizon;

    % Step 5. Update fault
    ffinal = ffinal + fhat;
    fhathist(isim+1,:) = ffinal;
    fprintf('Error at step %d: %.4f\n', isim, norm(c - ffinal));

    % "Step 6." Change noise rng seeds
    simulink_seed = simulink_seed+2;
end

%%

figure;
subplot(121);
plot(0:isim, fhathist); hold on; set(gca,'ColorOrderIndex', 1); plot([0;isim],[c; c],'--')
legend({'$\hat{f}_1$', '$\hat{f}_2$', '$\hat{f}_3$'  '$f_1$', '$f_2$', '$f_3$'}, 'Interpreter','latex','NumColumns',2); 
xlim([0,isim]);
xlabel('Iteration')
ylabel('Fault estimate')
grid;

subplot(122);
plot(0:isim, vecnorm(fhathist-c,2,2))
ylabel('$|\hat{\boldsymbol{f}}-\boldsymbol{f}|$')
xlim([0,isim]);
xlabel('Iteration')
grid;

set(gcf(),'Position', [680 458 4*200 200])
%matlab2tikz('fest_gn.tex')


