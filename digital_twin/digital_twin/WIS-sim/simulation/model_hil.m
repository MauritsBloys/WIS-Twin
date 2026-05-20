%% model_hil.m

% Create controllers using cantoni_LMI.m first
% Then resample them for HIL simulation


%% load pool parameters for lab setup
cantoni_LMI;

SPS = 8;

for i = 1:nPool

    % Now construct discrete-time versions of the dynamic control matrices
    h = 1/60 * 1/SPS; % Sampling time - STILL NEEDS TO BE PROPERLY SELECTED <------------
 
    ddelay(i) = round(tau(i)/h); % Discrete delay
    
    % first order
    P{i} = tf([1],[alpha(i) 0 ]);

    % Discretize the plant models
    Pd{i} = c2d(P{i}, h, 'zoh');

end

%% pool 0 %%
P0 = tf([1],[Wis.area0 0 ]);
Pd0 = c2d(P0, h, 'zoh');

%% Create models for use with PSTC

%% Controller (global + local)

% Create K_i from hat{K_i} i and W_i
% Combine all K_i into one controller K

% Because hat(K_i) is just a P-controller we can just 
% aff it in the numerator of W_i

KA = zeros(nPool * 2, nPool * 2);
KB = zeros(nPool * 2, nPool);
KC = zeros(nPool, nPool * 2);
KD = zeros(nPool, nPool);

for i = 1:nPool
    tempKi =  comb_contr.D(i,i) * W{i};
    [tempA, tempB, tempC, tempD] = tf2ss(tempKi.Numerator{1}, tempKi.Denominator{1});
    KA(2*i-1:2*i, 2*i-1:2*i) = tempA;
    KB(2*i-1:2*i, i) = tempB;
    KC(i, 2*i-1:2*i) = tempC;
    KD(i,i) = tempD;
end

% create continuous and discrete ss models
comb_K_cont = ss(KA, KB, KC, KD);
comb_K_disc = c2d(comb_K_cont,h,'tustin');

%% Pools

% init combined pool model with matrices of correct sizes
PA = zeros(nPool * 2, nPool * 2);
PB = zeros(nPool * 2, nPool);
PC = zeros(nPool, nPool * 2);
PD = zeros(nPool, nPool);

for i = 1:nPool

    % create first order ss model without delay
    tempA = [0];
    tempB = [1/alpha(i)];
    tempC = [1];
    tempD = [0];

    tempPoolNoDelay = ss(tempA, tempB, tempC, tempD);

    % add Pade approximation of delay for pool i
    tempPool = pade(tempPoolNoDelay * exp(-s*tau(i)));
    
    % build combined ss
    PA(2*i-1:2*i, 2*i-1:2*i) = tempPool.A;
    PB(2*i-1:2*i, i) = tempPool.B;
    PC(i, 2*i-1:2*i) = tempPool.C; % 1 0 1 0 1 0
    PD(i,i) = tempPool.D; % is zero anyway
end

% connect pools 1->2, 2->3 
% inflow pool 2,3 = outflow pool 1,2 without delay

PB(1, 2) = -1/alpha(1); 
PB(3, 3) = -1/alpha(2); 

% create continuous and discrete ss models
comb_Pool_cont = ss(PA, PB, PC, PD);
comb_Pool_disc = c2d(comb_Pool_cont,h,'zoh');
