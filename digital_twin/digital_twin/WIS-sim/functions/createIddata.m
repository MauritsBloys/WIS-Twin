%% Create iddata for experiment data

function [ze] = createIddata(ExperimentData, Wis, useGateModel, showPlot)
% ExperimentData    data structure of the experiment created by createWisData
% Wis               data struicture with lab setup properties
% useGateModel      use gate model for input (true) or experiment data (false)
% showPlot          show plot of the experiment
%
% ze                return iddata

pool = ExperimentData.pool;

%% show time plot of the experiment
if showPlot
    figure();
    title("Water levels");
    plot(ExperimentData.timing/1000, ExperimentData.water_levels);
    legend("s1", "s2", "s3", "s4", "s5", "s6", "s7")
    xlabel("time [s]");
    ylabel("water level [m]");
    saveFigureEps("water_levels_identification");
end

%% show plot of the input
if showPlot
    figure(2);
end
[input1, ~] = selectFlowsForPool(ExperimentData.flow_in, ExperimentData.flow_out, pool, showPlot);

% optional: don't use flow estimated from direct measurements but from the 
% model of the gates
if useGateModel
    k = (ExperimentData.actuators(:,pool) .* Wis.a_gate + Wis.b_gate);
    f_est_exp = k .* ExperimentData.actuators(:,pool) .* sign(ExperimentData.delta_height(:,pool)) .* sqrt(abs(ExperimentData.delta_height(:,pool)));

    if showPlot
        plot(f_est_exp)
    end
    input1 = f_est_exp;
end

if showPlot
    saveFigureEps("flows_identification");
end

output1 = ExperimentData.water_levels(:, pool*2+1);
dt1 = ExperimentData.dt;

% crop data (unused at the moment)
input1 = input1(1:end);
output1 = output1(1:end);


% remove 'datum' to start from zero
output1 = output1 - output1(1);

%% create iddata

ze = iddata(output1,input1,dt1); 
if showPlot
    figure(4)
    plot(ze)
end

% %% bode plot from raw data
% 
% Ge = spa(ze);
% figure(5)
% bode(Ge)

%%
% plot impulse estimation, raw data
Mimp = impulseest(ze,60); 
if showPlot
    figure(6)
    % step response
    step(Mimp)
end

% estimate delay
disp('delay')
delayest(ze)

% add experiment properties to iddata
ze.ExperimentName = sprintf("%s-%s", ExperimentData.description, ExperimentData.type);
ze.Name = sprintf("%s-%s", ExperimentData.filename, ExperimentData.type);
ze.InputName = "flow";
ze.InputUnit = "m^3/sec";
ze.OutputName = "water level";
ze.OutputUnit = "m";

end

