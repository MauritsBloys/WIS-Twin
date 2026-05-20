% Used by simulation (simulate_3pools_hil.slx) to calculate 
% the flow over an undershot gate from the water levels
% and gate setting

function [flow] = gate2flow(servo, h1, h2)
% servo     gate setting (0-255)
% h1, h2    water levels before and after the gate (m)
%
% flow      flow m^3/sec

K = 6.5000e-05 *60; % controller asks for flow per minute so gate constant must be multiplied
delta_level = h1 - h2;

% calculate flow with actual gate setting
flow = (servo + 0) * (K )  * sqrt(abs(delta_level)) * sign(delta_level); 

