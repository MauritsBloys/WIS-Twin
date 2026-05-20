%% Select in and out flow for a pool from experiment data

function [in, out] = selectFlowsForPool(f_in,f_out, pool, showPlot)
% f_in      all in flows 
% f_out     all out flows
% pool      pool number, note: pool 0-3
% showPlot  show plot of the flows
%
% in, out   return in and out flow

in = f_in(:,pool+1);
out = f_out(:,pool);

%% show plot
if showPlot
    plot(in)
    hold on;
    plot(out)
    title("Water flows");
    legend("q_{in}", "q_{out}")
end

end

