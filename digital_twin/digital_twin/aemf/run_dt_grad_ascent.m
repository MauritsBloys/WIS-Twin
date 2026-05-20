
%% Plant

G = sys_nom;
My = Ms(:,1:ny);
Mu = Ms(:,ny+1:end);
MG = My*G + Mu;
    
%     w = 1000;  %rad/s
%     f = w/(2*pi);
%     h = 1/f/40;
h = 0.05;  % About 1/10 of Nyquist frequency for the peak amplitude of G 
sysd = c2d(minreal(ss(MG)),h);

%% Make P
N = 40;
AP = sysd.A;
BP = sysd.B;
CP = sysd.C;
DP = sysd.D;

n = length(AP);
m = size(BP,2);
p = size(CP,1);

A0 = eye(n);
As = zeros(n*N,n);
Pc = {};
Pc{1} = DP;
Pxc = {};
Pyc = {};
for i = 1:N
    As(1:n,1:n) = A0;
    if i < N
        Pc{i+1} = CP*A0*BP;
    end
    Pxc{i} = A0*BP;
    Pyc{i} = CP*A0;
    A0 = A0*AP;
end
Pc{N+1} = CP*BP*0;
Pxc{N+1} = BP*0;

P = cell2mat(Pc(toeplitz(1:N,[1, (N+1)*ones(1,N-1)])));
Px = cell2mat(Pxc(toeplitz(1:N,[1, (N+1)*ones(1,N-1)])));

%% Effect of initial (recurring) state
x0FromU = (eye(n) - A0)\Px(end-n+1:end,:);

%% Make Pi function

P_i = @(i) P(p*(i-1)+1:p*i,:) + Pyc{i}*x0FromU;

pArray = zeros([size(P_i(1)), N]);
for i = 1:N
    pArray(:,:,i) = P_i(i);
end

%% Optimize
Ncases = 1;  % In case we want to check multiple initial conditions
uall = zeros(N*m,Ncases);
lall = zeros(Ncases);
rng(1907);

bound = 1/sqrt(2);  % MSE of 5*sin(t)

for icase = 1:Ncases
    %% Initialize u

    u = 100*(rand(N*m,1) - 0.5);
    u = my_project(u,2,bound,m,false);
    u0 = u;
    
    %% Start gradient ascent procedure

    step = 10;
    Ntries = 10000;  %was 100000
    lvec = nan(Ntries,1);
    uold = u;
    lold = 0;
    tau = 100;

    tic;
    grad_descent_loop;
    
    toc
    l
    
    U = reshape(u,m,[])';

    uall(:,icase) = u;
    lall(icase) = l;
end


%% Plot u's
figure;
us = uall(:,1:icase);
u = us(:,1);
us(:,1) = [];  %pop
U = reshape(u,m,[])';
plot(U);
while ~isempty(us)
    pause(0.2);
    [~, iclose] = min(diag((us-u)'*(us-u)),[],1);
    u = us(:,iclose);
    us(:,iclose) = [];
    cla;
    U = reshape(u,m,[])';
    plot(U);
    ylim([-1.1,1.1]);
end
