%% ETC / STC parameters

% triggering function parameters
TRIG_LEVEL = 1;%1;%0.1; %2;
sigma = 0.1;%0.1;%0.0025; %0.1; % 0.01
kfinal = 30;  % Heartbeat

%% Q Matrix (\bar{Q} in the paper)
nz = pp+mp;  % z / zeta variable

Q1 = (1-sigma^2)*eye(nz);
Q1(ppt+1:end, ppt+1:end) = eye(nz-ppt);  % No triggering based on control input
Q2 = -eye(nz);
Q3 = eye(nz);
Q = [Q1, Q2; Q2', Q3];