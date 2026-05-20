function [flow] = height_to_flow(h1,h2)
%HEIGHT_TO_FLOW Summary of this function goes here
%   Detailed explanation goes here

% TODO: check negative values + scale values

delta_h = h1 - h2;

neg_values = (delta_h < 0);

delta_h(neg_values) = 0;

flow = sqrt(delta_h);

end

