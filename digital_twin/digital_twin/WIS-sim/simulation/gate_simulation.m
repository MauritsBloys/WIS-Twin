% Used by simulation (simulate_3pools_slx) to calculate actual flow
% over an undershot gate from a desired flow and the water levels
% and rateb limit of the gate

function [flow, next_gate] = gate_simulation(flow_request, h1, h2, current_gate, alpha, beta)
% flow_request requested flow (m^3/s)
% h1, h2 water level before and after gate (m)
% current_gate setting (0-255)
% alpha, beta leakage parameters (optional, default 0)
%
% flow actual flow (m^3/s)
% next_gate (0-255)

if nargin < 6, alpha = 0; beta = 0; end

% Increased this to effectively disble the rate limit 
max_step = 20000*255/(7*1); % rate limit based on sample time 

% flow in m^3/sec

K = 6.5000e-05*60; % controller asks for flow per minute so gate constant must be multiplied
delta_level = h1 - h2;

% bepaal richting adhv h1, h2?

% calculate desired gate setting

% flow only possible downstream, close gate if not possible
if (sign(flow_request) ~= sign(delta_level))
    temp_servo = 0;
else
    temp_servo = flow_request / (K * sqrt(abs(delta_level)));
end

% no negative flow implemented on FF yet
if flow_request < 0
    temp_servo = 0;
end


% rate limit the gate
if (temp_servo > current_gate + max_step) 
    temp_servo = current_gate + max_step;
else
    if (temp_servo < current_gate - max_step)
        temp_servo = current_gate - max_step;
    end
end

% respect limits of the gate
if (temp_servo > 255)
    temp_servo = 255;
end

% if (temp_servo < 5)
%     temp_servo = 5;
% end

%temp_servo = temp_servo + rand(1) - 0.5;

% calculate flow with actual gate setting
flow = (temp_servo + 0) * (K * 1)  * sqrt(abs(delta_level)) * sign(delta_level); 
next_gate = temp_servo;


%1


%Sflow = 4.43 * 10^-4;

% later: rate limit gate

flow = flow + wis_leakage(h1, h2, alpha, beta);

%flow = 4.43 * 10^-4;
%flow = 0.2279; % this flow will bring last pool to 1m in 60 steps


end

