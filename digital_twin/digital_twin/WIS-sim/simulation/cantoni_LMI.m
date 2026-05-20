%% Construction of a set of distributed controllers for a string of pools
% Source: Distributed controller design for open water channels (2008) [1]
% Yuping Li & Michael Cantoni

% Author: Jacob Lont with help from Gabriel A. Gleizer
% Date: 12-11-2019
% Last modified: 18-06-2020
% Status: Controllers work, but the control may be improved based on 
% comparison with a PI-controller from literature.

% Adapted by Bas Boot:
%   - number of pools variable
%   - use TU Delft testbed values
%   - create models for PSTC


%% Parameters
clear all; % for debug purposes, can be removed later


%% load pool parameters for lab setup
lab_setup_values;


%% Define the matrices

for i=1:nPool
    % Create the pool specific matrices for pool i
    % Corresponding to eq. (4) of the paper.
    Att{i} = [0, 1/alpha(i), -1/alpha(i), 0;
            0, -2/tau(i), 4/tau(i), 0;
            0, 0, 0, 1;
            0, 0, 0, -1/rho(i) ];
    Ats{i} = [-1/alpha(i); 0; 0; 0];
    Ast{i} = [0, 0, 1, 0];
    Ass{i} = [0];
    
    Btn{i} = [0, -1/alpha(i), 0;
            0, 0, 0;
            0, 0, kappa(i)*phi(i)/rho(i);
            0, 0, kappa(i)*(rho(i)-phi(i))/rho(i)^2];
    Btu{i} = [0; 0; kappa(i)*phi(i)/rho(i); kappa(i)*(rho(i)-phi(i))/rho(i)^2];


    Bsn{i} = [0, 0, 0];
    Bsu{i} = [0];
    Ctz{i} = [-1 , 0, 0, 0;
             0, 0, 0, 0];
    Csz{i} = [0; 0];
    Cty{i} = [-1, 0, 0, 0];
    Csy{i} = [0];

    Dzn{i} = [1, 0, 0;
            0, 0, 0];
    Dzu{i} = [0; 1];
    Dyn{i} = [1, 0, 0];
    Dyu{i} = [0];    

    % Construct \Pi_Xi
    PiX_partial = [eye(4),      zeros(4,1),   zeros(4,3);
            Att{i},        Ats{i},         Btn{i};
            Ast{i},        Ass{i},         Bsn{i};
            zeros(1,4),     eye(1),            zeros(1,3);
            Ctz{i},        Csz{i},         Dzn{i};
            zeros(3,4),     zeros(3,1),   eye(3)];
        
    

    % Construct \Pi_Yi
    PiY_partial = [Att{i}',       Ast{i}',        Ctz{i}';
            -eye(4),     zeros(4,1),   zeros(4,2);
            zeros(1,4),  -eye(1),      zeros(1,2);       % Comment this line only for pool 1
            Ats{i}',       Ass{i}',        Csz{i}';
            zeros(2,4),  zeros(2,1),   -eye(2);
            Btn{i}',       Bsn{i}',        Dzn{i}'];
          
    % Construct NXi and NYi after emptying blocks
    % Construct NYi 
    % "NYi with columns that span the null-space of (Btu)T (Bsu)T (Dzu)T"
    BBD = [Btu{i}', Bsu{i}', Dzu{i}'];
    NY{i} = null(BBD);
    
    % Construct NXi 
    % "NXi is a matrix with columns that span the null-space of Cty Csy
    % Dyn"

    CCD = [Cty{i}, Csy{i}, Dyn{i}]; 
    NX{i} = null(CCD);
    
    % Calvulate \Pi_Xi and \Pi_Yi
    PiX{i} = PiX_partial * NX{i};    
    PiY{i} = PiY_partial * NY{i}; 


    % TODO: cleanup! and check! There is something strange in the paper
    % here
    switch i
        case 1          % Pool 1 specific matrices
            % Modify N1Y to comply with the empty (5th) column of PiY1
            NY{1} = [NY{1}(1:4,:);  NY{1}(6:7,:)]; % Only for pool 1

            Ast{1} = zeros(0,4); % Only for pool 1
            Ass{1} = zeros(0,1); % Only for pool 1
            Bsn{1} = zeros(0,3); % Only for pool 1
%             Bsu1 = zeros(0,1); % Only for pool 1  % Initial mistake?

            % PiX1, PiY1 computation differs from rest of the pools
            PiX{1} = [eye(4),     zeros(4,1),  zeros(4,3);
                    Att{1},       Ats{1},        Btn{1};
                    Ast{1},       Ass{1},        Bsn{1};
                    zeros(1,4), 1,           zeros(1,3);
                    Ctz{1},       Csz{1},        Dzn{1};
                    zeros(3,4), zeros(3,1),  eye(3)]      * NX{1};
            
            PiY{1} = [Att{1}',      Ast{1}',        Ctz{1}';
                    -eye(4),    zeros(4,0),   zeros(4,2);
                    %0,  -zeros(1,0),  0;       % Only comment for pool 1
                    Ats{1}',      Ass{1}',        Csz{1}';
                    zeros(2,4), zeros(2,0),   -eye(2);
                    Btn{1}',      Bsn{1}',        Dzn{1}']      * NY{1};

        case nPool          % Pool 5 specific matrices
            % Modify N5X to comply with the empty (5th) column of PiX5
            % Remove row 5 of N5X
            NX{nPool} = [NX{nPool}(1:4,:);  NX{nPool}(6:8,:)]; % Only for pool 5

            Ats{nPool} = zeros(4,0); % Only for pool N
            Ass{nPool} = zeros(1,0); % Only for pool N
%             Ass5 = []; % This is what we use later on to Create S5, so
%             maybe we should also implement this here?
            Csz{nPool} = zeros(2,0); % Only for pool N

            % PiX5, PiY5 computation differs from rest of the pools
            PiX{nPool} = [eye(4),     zeros(4,0),  zeros(4,3);
                    Att{nPool},       Ats{nPool},        Btn{nPool};
                    Ast{nPool},       Ass{nPool},        Bsn{nPool};
                    %zeros(1,4), 1,           zeros(1,3); % Only pool 5
                    Ctz{nPool},       Csz{nPool},        Dzn{nPool};
                    zeros(3,4), zeros(3,0),  eye(3)]      * NX{nPool};
            
            PiY{nPool} = [Att{nPool}',      Ast{nPool}',        Ctz{nPool}';
                    -eye(4),    zeros(4,1),   zeros(4,2);
                    zeros(1,4)  -eye(1),      zeros(1,2);
                    Ats{nPool}',      Ass{nPool}',        Csz{nPool}';
                    zeros(2,4), zeros(2,1),   -eye(2);
                    Btn{nPool}',      Bsn{nPool}',        Dzn{nPool}']      * NY{nPool};
    end
end %for






%% Define the optimization problem for the string of pools

gamma_sqr = 50; % small enough to solve both valid + exceed controller


disp(strcat('solving for \gamma = ', num2str(sqrt(gamma_sqr))));


cvx_begin sdp  % semi-definite programming
    variable Xtt(4,4,nPool) semidefinite % implies pos def
    variable Ytt(4,4,nPool) semidefinite;

    % BB: It's hard to come up with a good variable names for X_{i, i-1}^tt, Y_{i, i-1}^tt
    % using suffix minus one to indicate i-1 so so 1 is actually 1,0 
    variable X_minusone(1,1,nPool+1) symmetric; % symmetric, but not pos def
    variable Y_minusone(1,1,nPool+1) symmetric;

%     variable gamma_sqr nonnegative
    %
    minimize 0  
%     minimize gamma_sqr
    subject to
        % Did not include: Xitt > 0, Yitt > 0, not sure if this is already
        % implicitly declared or not. Check this later on.
        
        % BB: Set to zero for now, but the value will not be used during
        % optimization (although they might be used during creation of the
        % controllers
%         X_minusone(:,:,1) == 0;
%         Y_minusone(:,:,nPool+1) == 0;
%         X_minusone(:,:,1) == 0;
%         Y_minusone(:,:,nPool+1) == 0;
        
        
        for i = 1:nPool
            % skip first round for variables X_{i, i-1}^tt, Y_{i, i-1}^tt,
            % because boundary has already been declared
            if i > 1
                X_minusone(:,:,i) <= 0;
                Y_minusone(:,:,i) <= 0;
                
                [X_minusone(:,:,i), -1; -1, Y_minusone(:,:,i)] <= 0;
            end
                
        % 
            [Xtt(:,:,i), eye(4); eye(4), Ytt(:,:,i)] >= 0;
            
            % Create intermediate result, so we can remove the empty rows
            % at the boundaries
            
            % Calculate partial results first so we can remove rows if
            % necessary
            X_partial = [zeros(4)    Xtt(:,:,i)        zeros(4,1)   zeros(4,1) zeros(4,2)  zeros(4,3);
             Xtt(:,:,i)        zeros(4)    zeros(4,1)   zeros(4,1) zeros(4,2)  zeros(4,3);
             zeros(1,4)  zeros(1,4)  -X_minusone(:,:,i)        0          zeros(1,2)  zeros(1,3);
             zeros(1,4)  zeros(1,4)  0            X_minusone(:,:,i+1)        zeros(1,2)  zeros(1,3);
             zeros(2,4)  zeros(2,4)  zeros(2,1)   zeros(2,1) eye(2)      zeros(2,3);
             zeros(3,4)  zeros(3,4)  zeros(3,1)   zeros(3,1) zeros(3,2)  -gamma_sqr*eye(3)];
         
            Y_partial = [zeros(4)    Ytt(:,:,i)        zeros(4,1)   zeros(4,1) zeros(4,2)  zeros(4,3);
             Ytt(:,:,i)        zeros(4)    zeros(4,1)   zeros(4,1) zeros(4,2)  zeros(4,3);
             zeros(1,4)  zeros(1,4)  -Y_minusone(:,:,i)         0          zeros(1,2)  zeros(1,3);
             zeros(1,4)  zeros(1,4)  0            Y_minusone(:,:,i+1)        zeros(1,2)  zeros(1,3);
             zeros(2,4)  zeros(2,4)  zeros(2,1)   zeros(2,1) eye(2)      zeros(2,3);
             zeros(3,4)  zeros(3,4)  zeros(3,1)   zeros(3,1) zeros(3,2)  -(1/gamma_sqr)*eye(3)];
         
            % TODO remove empty rows/columns according to paper Cantoni
            if i == 1
                X_partial(:, 9) = [];
                X_partial(9, :) = [];
                Y_partial(:, 9) = [];
                Y_partial(9, :) = [];
            end
            if i == nPool
                X_partial(:, 10) = [];
                X_partial(10, :) = [];
                Y_partial(:, 10) = [];
                Y_partial(10, :) = [];
            end    
            disp(i)
            PiX{i}' * X_partial * PiX{i} <= 0;
            PiY{i}' * Y_partial * PiY{i} >= 0;
         
        end
cvx_end 

%% Stop if cvx has failed
if strcmp(cvx_status, 'Failed')
    disp('CVX is not able to solve the problem for this value of gamma.');
    return;
end

%% Check the by cvx computed values:
smallestXtti = inf;
smallestYtti = inf;

for i = 1:nPool
    smallestXtti = min(smallestXtti, min(eig(Xtt(:,:,i))));
    smallestYtti = min(smallestYtti, min(eig(Ytt(:,:,i))));
end
display('The smallest Xtti eigenvalue is:' + " " + num2str(smallestXtti));
display('The smallest Ytti eigenvalue is:' + " " + num2str(smallestYtti));


%% Create the controllers
% Just as before; the 1st and Nth pool have some specialties because of the
% interconnection properties. 
%N = 5; % The number of pools. To show the 'createsi' function which pool is the Nth pool.
% X10 = []; Y10 = []; This does not work out as Xiim1C gets all empty
% entries, which makes it not possible to create Gi as it looks like.
%X10 = 0; Y10 = 0; 
% BB: already set by cvx, but just to be safe
X_minusone(:,:,1) = 0;
Y_minusone(:,:,1) = 0;


for i = 1:nPool
    % clc;
    % Function to create the dynamic control matrix of pool 1: S1
    % BB: Because Jacob used zero values instead of empty matrices for X
    % and Y (see comment above) pool 1 is not an exception anymore
    
    if i == nPool
%         X65 = []; % Set to zero. Should be a zero dimension entry. It does not exist.
%         Y65 = [];
%         Ass5 = [];
        %% The Nth pool:
        % The Nth pool has some pool specific properties in the calculation.
        
        %S{} = createsi(Ass5, Ast5, Att5, Ats5, Bsn5, Bsu5, Btn5, Btu5, Csy5, Csz5, Cty5, Ctz5, Dyn5, Dzn5, Dzu5, X54, X65, Xtt5, Y54, Y65, Ytt5, gamma_sqr, 5, N);
        S{i} = createsi([], Ast{i}, Att{i}, Ats{i}, Bsn{i}, Bsu{i}, Btn{i}, Btu{i}, Csy{i}, Csz{i}, Cty{i}, Ctz{i}, Dyn{i}, Dzn{i}, Dzu{i}, X_minusone(:,:,i), [], Xtt(:,:,i), Y_minusone(:,:,i), [], Ytt(:,:,i), gamma_sqr, i, nPool);
    else
        S{i} = createsi(Ass{i}, Ast{i}, Att{i}, Ats{i}, Bsn{i}, Bsu{i}, Btn{i}, Btu{i}, Csy{i}, Csz{i}, Cty{i}, Ctz{i}, Dyn{i}, Dzn{i}, Dzu{i}, X_minusone(:,:,i), X_minusone(:,:,i+1), Xtt(:,:,i), Y_minusone(:,:,i), Y_minusone(:,:,i+1), Ytt(:,:,i), gamma_sqr, i, nPool);
    end
    
    disp('Si-matrices created');

    %% Simulink matrices and models
    % Matrices used in the simulink file to split the output of the dynamic
    % controller into wiK and uiK
    C1 = [1 0]; % w_i^K
    C2 = [0 1]; % u_i^K

    % Create the state-space A,B,C,D matrices from the computed S matrices
    [SA{i}, SB{i}, SC{i}, SD{i}, SS{i}] = splitup_S_matrix(S{i});


    % Now construct discrete-time versions of the dynamic control matrices
    h = 1/(60); % Sampling time - STILL NEEDS TO BE PROPERLY SELECTED <------------
    % I use the 'tustin' method for phase property preservation of the contr.
    [ssd{i}, ssd_map{i}] = c2d(SS{i}, h, 'tustin'); 

    % TODO: BB: Check this. Tau is in minutes, but h in seconds.
    ddelay(i) = round(tau(i)/h); % Discrete delay
    
    % Continuous-time shaping weights
    W{i} = tf([kappa(i)*phi(i) kappa(i)], [rho(i) 1 0]);

    % Discretize the shaping weights
    Wd{i} = c2d(W{i},h,'tustin');

    % Define the continuous-time plant models (third order)
    %P{i} = tf([1],[alpha(i)/w_n(i)^2 2*alpha(i)*zeta(i)/w_n(i) alpha(i) 0 ]);
 
    % first order
    P{i} = tf([1],[alpha(i) 0 ]);
    
    % Discretize the plant models 
    Pd{i} = c2d(P{i}, h, 'zoh');

end

%% pool 0 %%
P0 = tf([1],[Wis.area0 0 ]);
Pd0 = c2d(P0, h, 'zoh');

%% Save the workspace for use in the simulation (moved to later on)
% This way, we don't use this script as initialization script, but instead
% we can use a script that simply loads the set of precomputed variables.
% save('distributed_workspace.mat'); 
disp(strcat('Problem solved for \gamma = ', num2str(sqrt(gamma_sqr))));
% disp('Workspace saved to file for use in simulation.');

%% TODO: Look at Heemels and adapt scripts

%% Check stability using the LMI script based on Heemels(2013) PETC for ..

gamma = sqrt(gamma_sqr);

% System matrices using the generalized plant defined in eq. 4 in Cantoni
% et al. (2008) - Distributed controller design for open water channels

% Bsu1 = zeros(0,1); % This is different from before but this is right!!
% But, it gives problems, as we now do not have a square A matrix
% Adjust the matrices of the wi row of G1 to be 1 row, 0-valued entries
% Bsu1 = zeros(1,1);
% Bsn1 = zeros(1,3);
% Ast1 = zeros(1,4);
% Ass1 = zeros(1,1);
% Same problems for the number N(=5) case, but now for the Atsi column.
% Ats5 = zeros(4,1);
% Ass5 = zeros(1,1);
% Csz5 = zeros(2,1);
% Csy5 = zeros(1,1);
% --> Chosen to define A in a different way -> A = Att; instead of saying 
% A = [Att, Ats; Ast, Ass]; 

% TODO: Adjust CreateSi to handle this and check!
% Csy5 = zeros(1,0); % This is different from before, but this is right!


% Try to build the interconnection using append
% G = append(G1, G2, G3, G4, G5); % Does give Ap, but the interconnection is not reflected now..

% Now define the combined A,B,C,D plant-matrices
Ap = zeros(4*nPool, 4*nPool);
Bp = zeros(4*nPool, 1*nPool);
Bw = zeros(4*nPool, 2*nPool);
Cp = zeros(1*nPool, 4*nPool);
for i = 1:nPool
    % Put pool behaviour in Ap
    Ap(1 + (i-1) * 4:1 + (i-1) * 4 + 3, 1 + (i-1) * 4:1 + (i-1) * 4 + 3) = Att{i};
    % Put outflow behaviour in Ap (for last pool outflow is added disturbance)
    if i <  nPool
        Ap(1 + (i-1) * 4:1 + (i-1) * 4 + 3, 1 + (i-1) * 4 + 6:1 + (i-1) * 4 + 6) = Ats{i};
    end
    
    % Put input / control in Bp
    Bp(1 + (i-1) * 4:1 + (i-1) * 4 + 3, 1 + (i-1) * 1:1 + (i-1) * 1) = Btu{i};
    
%Bp = blkdiag(Btu1, Btu2, Btu3, Btu4, Btu5);

    % Put input / disturbance in Bw
    Bw(1 + (i-1) * 4:1 + (i-1) * 4 + 3, 1 + (i-1) * 2:1 + (i-1) * 2 + 1) = Btn{i}(:,2:3); 

    % Put output in Cp
    Cp(1 + (i-1) * 1:1 + (i-1) * 1, 1 + (i-1) * 4:1 + (i-1) * 4 + 3) = [1 0 0 0];
    
end


% Define the controller by combining the individual controller matrices
% I have the discrete time state-space description of the controller
% Now extend it to have w_i^K as a state.
% Create the submatrices I need, to redefine my states
Ac = zeros(5 * nPool, 5 * nPool);
Bc = zeros(5 * nPool, 1 * nPool);
Cc = zeros(1 * nPool, 5 * nPool);
Dc = zeros(1 * nPool, 1 * nPool);

for i = 1:nPool   
    [Aci{i}, Bci{i}, Cci{i}, Dci{i}] = create_combined_control_sub_matrices(ssd{i}, i, nPool);
    % TODO: BB: check why this has to be inverted for the first pool
    if i == 1
        Bci{i} = -Bci{i};
        Dci{i} = -Dci{i};
    end
    
    Ac(1 + (i-1) * 5:1 + (i-1) * 5 + 4, :) = Aci{i};
    Bc(1 + (i-1) * 5:1 + (i-1) * 5 + 4, i) = Bci{i};
    Cc(i, :) = Cci{i}; 
    Dc(i, i) = Dci{i};
end

nDc = size(Dc,1);
D = [zeros(nDc), zeros(nDc); Dc, zeros(nDc)];
nD = size(D,1); % D should be square (10x10), check eq. 43 of Heemels(2013)
C = blkdiag(Cp,Cc);

%% Create state-space models from the matrices for simulation checks
% Create a simulation to check if the matrix operations are right
% All discrete-time!
Dp = zeros(size(Cp,1),size(Bp,2)); % Because we dont have a Dp
comb_plant_cont = ss(Ap, Bp, Cp, Dp); % combined plant model cont.-time
comb_plant_disc = c2d(comb_plant_cont,h,'zoh'); % combined plant model disc.-time
comb_contr = ss(Ac, Bc, Cc, Dc, h); % combined controller model (discrete-time)

% Simulation works!

%% Check nominal stability using feedback and eigenvalues within unit circle
% They are all inside the unit circle! => Nominal stability
CPcomb = feedback(comb_plant_disc*comb_contr,eye(nPool),+1); % Combined system, pos. feedback
poles = eig(CPcomb);
fprintf('The eigenvalues of the CL-system are between %f and %f \n',min(real(poles)), max(poles));

if max(abs(poles))>1
    fprintf('NO nominal stability!\n Check the closed-loop poles.\n');
else
    fprintf('Nominal stability: all poles inside the unit circle.\n');
end

% Plot the eigenvalues with the unit circle for visual inspection
% figure(1); clf;
% plot(poles,'*')
% hold on
% ezplot(@(x,y) (x).^2 + (y).^2 -1^2)
% xlim([-1.1,1.1]); ylim([-1.1,1.1]);
% title('Closed loop eigenvalues and the unit circle');
% xlabel('Re'); ylabel('Im');
% hold off

% Compute a reference for rho I put into the LMI:
a = max(abs(poles));
rho_reference = -log(abs(a))/h; % 0.0044

%% Do the Heemels LMI check for GES and L2-gain
if gamma < sqrt(max(eig(D'*D))) %check of Heemels's Theorem V.2, eq 48
    fprintf('gamma is too small according to Heemels for this matrix D.\n');
    % Then M is not invertible see eq. 20 of Heemels 2013
end

% TODO: renamed rho to rho_lmi because it shadows rho used in the
%       simulation => CHECK WHERE RHO IS USED!

% Triggering parameters
rho_lmi = 0.0000000000001; %Tuned manually % rho > 0 , (lower bound on) the decay rate
% sigma = 0.15; % Tuned manually together with rho and the L2gain: 0.15 works nice in simulation
sigma = 0.10;
% sigma = 0.05; % 0.05 Very nice performance and also a great reduction of communication!
% sigma = 0.01; % sigma for LMI: 0.01 looks great, and still less than 25% of samples used
lambda = 20000000; % called gamma in Heemels 2013. L2 upper bound.

% Create Q
% Gamma = eye(10); % In our case of 1 artificial node, based: on 2*y, 2*u
% ng = size(Gamma, 1);
Q11 = (1-sigma)*C'*C;
Q12 = (1-sigma)*C'*D-C';
Q21 = (1-sigma)* D'*C - C;
Q22 = (D-eye(nD))'*(D-eye(nD)) - sigma*D'*D;
Q = [Q11, Q12; Q21, Q22];


%% Save the workspace for use in the simulation
% This way, we don't use this script as initialization script, but instead
% we can use a script that simply loads the set of precomputed variables.
save('distributed_workspace.mat'); 
disp(strcat('Problem solved for \gamma = ', num2str(sqrt(gamma_sqr))));
disp('Workspace saved to file for use in simulation.');



%% Run the LMI solver to check GES and get the L2-gain <= gamma
% Uncomment the next 3 lines to run the Heemels LMI check for output feeedback
% clc;
% fprintf('Calling the LMI-check-function.\n');
% [status, Ph, mu] = checkheemelslmiOF(comb_contr.A, comb_contr.B, comb_contr.C, comb_contr.D, Ap, Bp, Bw, Cp, Q, h, rho_lmi, lambda);
% 





% Without the second LMI added for J0, it solved for 
% sigma = 0.01;  rho = 0.0000000000001; lambda = 2000000000000;
% Also with the second LMI added, but the matrix of the second LMI has a
% complex value for min(eig) and even a negative real eig -5.2003e-05
% this is also achieved for smaller lambda (around 200(0) or so -> check)
% and rho around 0.000001

% Gabriel: Be very careful in the wording about stability. 'numerical
% issues', 'it is likely that we have stability, although because of the
% very numerical instability' we have not been able to get an actual
% positive definite solution.
% Mention the large size of the LMIs, and the ill conditioned matrices:
% very large and very small matrices

% You could try to use cvx_solver sedumi as an alternative -> fails all the
% time

% mu1 and mu2 are independent now. At first we only had 1 mu

% rho = 0.0000001; sigma = 0.01; lambda = 200000; FAILED 
% with: precision high, and the >= 0 conditions

% rho = 0.0000000000001; sigma = 0.01; lambda = 2000000000000; FAILED
% with: precision high, and the >= 0 conditions

% rho = 0.0000000000001; sigma = 0.01; lambda = 2000000000000; FAILED
% with: precision standard, and the >= 0 conditions

% rho = 0.0000000000001; sigma = 0.01; lambda = 2000000000000; FAILED
% with: precision standard, and NOT the >= 0 conditions for Ph (choosing semidefinite),
% but using the >= 0 conditions for mu1 and mu2

% rho = 0.0000000000001; sigma = 0.01; lambda = 2000000000000; FAILED
% with: precision standard, and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% rho = 0.0000000000001; sigma = 0.01; lambda = 2000000000000; FAILED but
% it was close. It looked like it was converging.
% with: precision standard, USING ONLY mu1 (in both LMIs)
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% Idea: Maybe I should go back to 1 single mu instad of mu1 and mu2,
% however I did get some results at some point using mu1 and mu2 ??
% OUTCOME: It doesnt directly solve my problem.

% rho = 0.0000001; sigma = 0.01; lambda = 2000;  FAILED
% with: precision standard, USING ONLY mu1 (in both LMIs)
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% rho = 0.0000001; sigma = 0.01; lambda = 2000; FAILED
% with: precision HIGH, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% rho = 0.0000001; sigma = 0.01; lambda = 20000;   FAILED
% with: precision HIGH, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% Idea: Change the performance output matrix Cbar as Gabriel suggested
% -> Changed the matrix Cbar to use the actual outputs instad of y_hat

% rho = 0.0000001; sigma = 0.01; lambda = 200000;   FAILED??
% with: precision HIGH, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2.

% rho = 0.0000001; sigma = 0.01; lambda = 200000;   FAILED
% with: precision DEFAULT, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2

% rho = 0.0000000000001; sigma = 0.01; lambda = 200000;   FAILED
% with: precision DEFAULT, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2

% rho = 0.0000000000001; sigma = 0.001; lambda = 200000;   FAILED
% with: precision DEFAULT, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2

% rho = 0.0000000000001; sigma = 0.0001; lambda = 20000000;   FAILED
% with: precision LOW, using both mu1 and mu2
% and NOT the >= 0 conditions for Ph (choosing semidefinite),
% and NOT using the >= 0 conditions for mu1 and mu2, but choosing
% nonnegative instead for both mu1 and mu2





%% extra
C_level = zeros(5,20);
C_level(1,1) = 1;
C_level(2,1+4) = 1;
C_level(3,1+8) = 1;
C_level(4,1+12) = 1;
C_level(5,1+16) = 1;

C_delta_i = zeros(5,20);
C_delta_i(1,2) = 1;
C_delta_i(2,2+4) = 1;
C_delta_i(3,2+8) = 1;
C_delta_i(4,2+12) = 1;
C_delta_i(5,2+16) = 1;

C_u = zeros(5,20);
C_u(1,3) = 1;
C_u(2,3+4) = 1;
C_u(3,3+8) = 1;
C_u(4,3+12) = 1;
C_u(5,3+16) = 1;

C_omega = zeros(5,20);
C_omega(1,4) = 1;
C_omega(2,4+4) = 1;
C_omega(3,4+8) = 1;
C_omega(4,4+12) = 1;
C_omega(5,4+16) = 1;


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

KAd = zeros(nPool * 2, nPool * 2);
KBd = zeros(nPool * 2, nPool);
KCd = zeros(nPool, nPool * 2);
KDd = zeros(nPool, nPool);

for i = 1:nPool
    tempKi =  comb_contr.D(i,i) * W{i};
    [tempA, tempB, tempC, tempD] = tf2ss(tempKi.Numerator{1}, tempKi.Denominator{1});
    KA(2*i-1:2*i, 2*i-1:2*i) = tempA;
    KB(2*i-1:2*i, i) = tempB;
    KC(i, 2*i-1:2*i) = tempC;
    KD(i,i) = tempD;
    
    % Alternative approch (direct digital) to avoid discretizing problems
    [tA ,tB ,tC ,tD ] = tf2ss( Wd{i}.Numerator{1} , Wd{i}.Denominator{1} )
    % multiply B with K_i (P controller)
    tB = comb_contr.D(i,i) *tB;
    tD = comb_contr.D(i,i) *tD;
    
    KAd(2*i-1:2*i, 2*i-1:2*i) = tA;
    KBd(2*i-1:2*i, i) = tB;
    KCd(i, 2*i-1:2*i) = tC;
    KDd(i,i) = tD;
    
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

%% Update saved workspace so digital twin can load comb_Pool_cont/disc
save('distributed_workspace.mat');
disp('Workspace updated with pool models (comb_Pool_cont, comb_Pool_disc).');









