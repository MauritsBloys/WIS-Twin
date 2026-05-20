% load_cart_pendulum_model_DAE.m

M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.81;
l = 0.3;

nx = 3;
nz = 4;
ny = 2;

% Linearized model
H = {[b 0 (M+m)*g; 0 m*g*l m*g*l; 1 0 0; 0 180/pi 0],...
     [M+m 0 0; m*l 0 0; 0 0 0; 0 0 0],...
     [0 m*l 0; 0 I+m*l^2 0; 0 0 0; 0 0 0]};  % H^+(q) of order 1
L = {[0 0 -1 0; 0 0 0 -0.1; -1 0 0 0; 0 -1 0 0]};  
% Making second input 10x weaker so that effects are in the same order
% of magnitude

%% Faulty system
% Friction is 5% smaller
H1 = {[b 0 0; 0 0 0; 0 0 0; 0 0 0],...
     zeros(4,3),zeros(4,3)};
L1 = {0*L{1}};

c(1) = -0.2;

% Second mode: mass is higher
H2 = {[0 0 m*g; 0 m*g*l m*g*l; 0 0 0; 0 0 0],...
     [m 0 0; m*l 0 0; 0 0 0; 0 0 0],...
     [0 m*l 0; 0 m*l^2 0; 0 0 0; 0 0 0]};  
% H2 = {[0 0 m*g; 0 m*g*l m*g*l; 0 0 0; 0 0 0],...
%      [m 0 0; m*l 0 0; 0 0 0; 0 0 0],...
%      [0 m*l m*l; 0 m*l^2 m*l^2; 0 0 0; 0 0 0]};
L2 = L1(1);

c(2) = 0.2;

% Third mode: weak motor
H3 = {zeros(4,3),zeros(4,3),zeros(4,3)};
L3 = {[L{1}(1:2,:); 0 0 0 0; 0 0 0 0]};

c(3) = -0.5;

Hf = {H1, H2, H3};
Lf = {L1, L2, L3};

%% Build MatrixPolynomials
H = MatrixPolynomial(H{:});
L = MatrixPolynomial(L{:});
Hf = cellfun(@(x) MatrixPolynomial(x{:}), Hf);
Lf = cellfun(@(x) MatrixPolynomial(x{:}), Lf);

%% Build nominal and faulty systems
sys = make_ss_from_dae(H,L);

% For debugging
% c = 0*c;

% Build final faulty system
HF = H;
LF = L;
for i = 1:numel(c)
    HF = HF + c(i)*Hf(i);
    LF = LF + c(i)*Lf(i);
end

sys_nom = sys(:,1:2);  % sys_nom cannot have the disturbance as input
sys_fault = make_ss_from_dae(HF, LF);
%%
save('inv_pendulum_very_weak_motor.mat','sys_nom','sys_fault');

%%
function sys = make_ss_from_dae(H, L)
    % State is v, theta, theta_dot. First write as Edx = Ax + Bu
    % Step one: move last column to Bw, build B, C and D
    Bw = H{1}(1:2,3);
    B = -L{1}(1:2,3:4);
    C = [H{1}(3:4,1:2), zeros(2,1)];
    D = zeros(2,3);
    % Step 2: qtheta = theta_dot
    H0 = H{1}(1:2,1:2);
    H1 = H{2}(1:2,1:2);
    % 2.1: Add column to relate old states to new state
    H0(:,3) = 0;
    H1(:,3) = H{3}(1:2,2);
    % 2.2 Add row to describe that theta_dot - qtheta = 0
    H1(3,2) = -1;
    H0(3,3) = 1;
    B = [B Bw; zeros(1,3)];
    sys = ss(dss(-H0, B, C, D, H1),'explicit');
end
