% l1normtest.m
%
% test  L1-norm calculation (Rutland & Lane's algorithm)
% 
disp('-----------------------------------------------------------')
disp('Example 1')

zeta=0.1;
wn=1;
tau=-1;

A = [-2*zeta*wn, -wn^2; 1,  0];
B = [1; 0];
C = wn^2*[tau, 1];
G=ss(A,B,C,0)
[L1norm,err,U,L,tol,niter]=l1norm(G,1e-12,24)

% calculate exact value
beta=zeta/(sqrt(1-zeta^2));
phi=atan(wn*tau*sqrt(1-zeta^2)/(zeta*wn*tau-1));
gamma= beta-wn*tau/(sqrt(1-zeta^2));
if tau/(zeta*wn*tau -1) >=0
    L1norm_calc=exp(-beta*phi)*abs(cos(phi)+ gamma*sin(phi))...
        *(1+exp(-beta*pi))/(1-exp(-beta*pi))...
        +abs(1-exp(-beta*phi)*(cos(phi)+ gamma*sin(phi)))
else
    L1norm_calc=exp(-beta*phi)*abs(cos(phi)+ gamma*sin(phi))...
        *(1+exp(-beta*pi))/(1-exp(-beta*pi))*exp(-beta*pi)...
        +abs(1+exp(-beta*(phi+pi))*(cos(phi)+ gamma*sin(phi)))
end

disp(['Actual error    = ',num2str(L1norm_calc-L1norm)])
disp(['Estimated error = ',num2str(err)])
disp(' ')

disp('Rutland & Lane reported ||G||_1=9.1759')


disp(' ')
disp('-----------------------------------------------------------')
disp('Example 2')
A = [-0.2, -1, 0;
    1, 0, 0;
    -1, 1, -1];
B = [1; 0; 0];
C = [0, 0, 1];
G=ss(A,B,C,0)
[L1norm,err,U,L,tol,niter]=l1norm(ss(A,B,C,0),1e-6) 
disp('Rutland & Lane reported ||G||_1=6.4886')
disp('-----------------------------------------------------------')
