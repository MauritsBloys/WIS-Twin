function Gi = create_gi(Atti, Atsi, Btni, Btui, Asti, Assi, Bsni, Bsui, ...
    Ctzi, Cszi, Dzni, Dzui, Ctyi, Csyi, Dyni, Dyui, poolnum)

% Used to create the individual generalized plants Gi defined in eq. 4 in
% Cantoni et al. (2008) - Distributed controller design for open water 
% channels

% Author: Jacob Lont - 17-07-2020
% Last modified: 17-07-2020

% A = [Atti, Atsi;
%      Asti, Assi];
% B = [Btni, Btui;
%      Bsni, Bsui];
% C = [Ctzi, Cszi;
%      Ctyi, Csyi];
% D = [Dzni, Dzui;
%      Dyni, Dyui];

A = Atti;
B = [Atsi, Btni, Btui];
C = [Asti; Ctzi; Ctyi];
D = [Assi, Bsni, Bsui;
     Cszi, Dzni, Dzui;
     Csyi, Dyni, Dyui];


Gi = ss(A, B, C, D);

i = num2str(poolnum);
if poolnum == 5
    % ri, di, qi, uiK
    Gi.InputName = {strcat('r', i), strcat('d', i), strcat('q', i), strcat('u', i, 'K')}; %4x1 cell
else
    % vi, ri, di, qi, uiK
    Gi.InputName = {strcat('v', i), strcat('r', i), strcat('d', i), strcat('q', i), strcat('u', i, 'K')}; %5x1 cell
end

if poolnum == 1
    % zi1, zi2, yiK
    Gi.OutputName = {strcat('z', i, '1 = e', i, '. = y', i, 'K'), strcat('z', i, '2 = u', i, 'K'), strcat('y', i, 'K')};
else
    % wi, zi1, zi2, yiK
    Gi.OutputName = { strcat('w', i), strcat('z', i, '1 = e', i, '. = y', i, 'K'), strcat('z', i, '2 = u', i, 'K'), strcat('y', i, 'K')};
end

% Gi = [Atti, Atsi, Btni, Btui;
%       Asti, Assi, Bsni, Bsui;
%       Ctzi, Cszi, Dzni, Dzui;
%       Ctyi, Csyi, Dyni, Dyui];
  
  end