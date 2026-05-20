function Si = createsi(Assi, Asti, Atti, Atsi, Bsni, Bsui, Btni, Btui, Csyi, Cszi, Ctyi, Ctzi, Dyni, Dzni, Dzui, Xiim1, Xip1i, Xtti, Yiim1, Yip1i, Ytti, gamma_sqr, poolnum, N)
% Parameters:
% N is the number of the last pool. 

T11i = [Atti,        zeros(4); 
        zeros(4),    zeros(4)]; %  H=8 W=8

if poolnum == 1
    T22i = [Assi,  zeros(0,1),  Bsni;       % empty row
            0,     0,           zeros(1,3);
            Cszi,  zeros(2,1),  Dzni]; 
    
    T21i = [Asti,        zeros(0,4);
            zeros(1,4),  zeros(1,4);    % Chosen: Height 1
            Ctzi,        zeros(2,4)];   % W=8

elseif poolnum == N
    T12i = [Atsi,        zeros(4,1),   Btni;
            zeros(4,0),  zeros(4,1),   zeros(4,3)];   %H=8 W=W-1   Modified for Nth pool
    
    T22i = [Assi,  0,           Bsni;
            [],     0,          zeros(1,3);
            Cszi,  zeros(2,1),  Dzni];      % Modified for the Nth pool
else
    T22i = [Assi,  0,           Bsni;
            0,     0,           zeros(1,3);
            Cszi,  zeros(2,1),  Dzni]; 
end

if poolnum ~= N
    T12i = [Atsi,        zeros(4,1),   Btni;
            zeros(4,1),  zeros(4,1),   zeros(4,3)];   %H=8
end

if poolnum ~= 1
    T21i = [Asti,        zeros(1,4);
            zeros(1,4),  zeros(1,4);    % Chosen: Height 1
            Ctzi,        zeros(2,4)];   % W=8
end



Tui1 = [zeros(4,4),  zeros(4,1), Btui;
        eye(4),      zeros(4,1), zeros(4,1)];   %zeros dimensions based on compatibility for Ui calculation
Tui2 = [zeros(1,2),  zeros(1,3),   Bsui;
        zeros(3,2),  eye(3),       zeros(3,1);        
        zeros(2,2),  zeros(2,3),   Dzui];    %zeros dimensions based on compatibility for Ui calculation 
Tyi = [zeros(4,4),  eye(4),       zeros(4,1),   zeros(4,1),  zeros(4,3);   % Chosen height 4
       zeros(1,4),  zeros(1,4),   zeros(1,1),   eye(1),      zeros(1,3);   % Chosen height 1 (first 3)
       Ctyi,        zeros(1,4),   Csyi,         zeros(1,1),  Dyni];        %zeros dimensions trial error

   
Ztti = sqrtm(Xtti - inv(Ytti));    % eq: Ztti * Ztti' = Xtti - inv(Ytti) 
Zip1i = inv(Xip1i - inv(Yip1i));


XttiC = [Xtti,      Ztti;       % dimensions done
         zeros(4),  eye(4)];    % calligraphic Xtti

if poolnum == 1
    Ziim1 = 0;   % eq: inv(Ziim1) = Xiim1 - inv(Yiim1)
    Xiim1C = [Xiim1, eye(1);        % dimensions done 
              eye(1), Ziim1];       % C for calligraphic    
else
    Ziim1 = inv(Xiim1 - inv(Yiim1));   % eq: inv(Ziim1) = Xiim1 - inv(Yiim1)
    Xiim1C = [Xiim1, eye(1);        % dimensions done
              eye(1), Ziim1];       % C for calligraphic    
end
    
if poolnum ~= N
    Xip1iC = [Xip1i,  eye(1);
              eye(1), Zip1i];
end

if poolnum == 1
    Ui = [Tui1' * XttiC, zeros(6,2), Tui2'];
elseif poolnum == N
%     Xip1iC = []; %[Xip1i, eye(1); eye(1), Zip1i];  Both X and Z are empty then
    Xip1iC = [[],[]; [], []]; % Made it 2x2 empty. That is the usual dimension of this matrix.
    Ui = [Tui1' * XttiC, zeros(6,2), Tui2'];
else
	Ui = [Tui1' * XttiC, zeros(6,3), Tui2'];
end
      



if poolnum == N
    Gi = [T11i' * XttiC + XttiC' * T11i,  XttiC' * T12i,                       T21i'; % 8 x 16 (17 normally)
          zeros(4,8),    [Xip1iC, zeros(0,4); zeros(4,0), -gamma_sqr*eye(4)],   T22i'; % 4x16 (5 x 17 normally)
          zeros(4,8),    zeros(4,4),            [inv(Xiim1C), zeros(2,2); zeros(2,2), -eye(2)] ]; % 4 x 16  (17 normally)  
elseif poolnum == 1
    Gi = [T11i' * XttiC + XttiC' * T11i,  XttiC' * T12i,                       T21i'; % 8 x 16
          zeros(5,8),    [Xip1iC, zeros(2,3); zeros(3,2), -gamma_sqr*eye(3)],   T22i'; % 3 x 16 (5 x 17)
          zeros(3,8),    zeros(3,4),            [inv(Xiim1C), zeros(2,2); zeros(1,3), -eye(1)] ]; % 4 x 16
else
    Gi = [T11i' * XttiC + XttiC' * T11i,  XttiC' * T12i,                       T21i'; % 8 x 17
          zeros(5,8),    [Xip1iC, zeros(2,3); zeros(3,2), -gamma_sqr*eye(3)],   T22i'; % 5 x 17
          zeros(4,8),    zeros(4,5),            [inv(Xiim1C), zeros(2,2); zeros(2,2), -eye(2)] ]; % 4 x1 17
end


% epsi is a positive scalar such epsi << 1/mui, and
Uiplus = pinv(Ui); % plus denotes the Moore-Penrose inverse
Uiort = null(Ui)'; % This should be it according to Gabriel
% The orthogonal completing is defined in the beginning of p.1310 of
% Iwasaki's 1994 paper.
% Utest = [Ui; null(Ui)']; % should be square and invertible. It is.

% Watch out! I modified the following equation according to eq.17 of
% Iwasaki's 1994 paper. The transposes are different than in Cantoni's.
% TODO: BB: Why has this been changed? Was this a typo in Cantoni?
mui = max(eig( Uiplus' * (Gi - Gi*Uiort' * inv(Uiort*Gi*Uiort') *Uiort*Gi) * Uiplus )); %lambda_max as max(eig())

epsi_scaling = 0.1; % Seems like I have to choose it myself in the end, such epsi << 1/mui
% epsi_scaling = 0.0001; % Changing the epsi_scaling value doesnt change
% simulation outcome, so it has no significant effect.
epsi = epsi_scaling * (1/mui);  

Phii = inv(1/epsi * (Ui' * Ui) - Gi);

if (poolnum == N || poolnum == 1)
    Vi = [Tyi, zeros(6,3)]; % Extend with zeros to get .. x 16 dimensions for the poolN or Pool1 case
else
    Vi = [Tyi, zeros(6,4)]; % Extend with zeros to get .. x 17 dimensions
end

Si = -1/epsi * Ui * Phii * Vi' * inv(Vi*Phii*Vi'); % for i=1, .., N
% Si should be 6x6 I think at this point
% To get Si = 6x6, I adjusted the size of Tyi, which is not clearly given
% in the original paper. This can be a source of faults.



end