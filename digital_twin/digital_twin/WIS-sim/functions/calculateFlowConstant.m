%% Use least squares optimization to find a flow constant (K) for an experiment.
%
% flow = sign(dh) * gate_opening * K * sqrt(abs(dh)

function [k] = calculateFlowConstant(GateData, gateToIdentify, gateSetting, skipOpening, flowToUse, showPlot)
% GateData          data structure of the experiment from createWisData
% gateToIdentify    gate number to identify
% gateSetting       servo setting of the gate 
% skipOpening       true|false skip the opening period
% flowToUse         flow to use for identification 1 = in, 2 = out, 3 = both
% showPlot          show plots of experiment and result
%
% k                 return flow constant

% hold if no flow has been selected
assert(flowToUse > 0, "WARNING: in or outflow must be used to identify the gate.");

% show message as feedback that the script is still doing something
fprintf("Calculating flow constant for gate %d with setting %d.\n", gateToIdentify, gateSetting);

firstMeasurementToUse = 1;
% Remove opening of the gate
if skipOpening
    firstMeasurementToUse = find(GateData.actuators(:, gateToIdentify) == gateSetting, 1, 'first');
end

% TODO: remove hardcoded 500
lastMeasurementToUse = size(GateData.actuators(:, gateToIdentify),1)-500;


%% show time plot
if showPlot
    figure();
    plot(GateData.timing/1000, GateData.water_levels);
    title("Water levels");
    legend("s1", "s2", "s3", "s4", "s5", "s6", "s7")
    xlabel("time [s]");
    ylabel("water level [m]");
end

[f_in, f_out] = selectFlowsForPool(GateData.flow_in, GateData.flow_out, gateToIdentify, false);

%% show flows calculated from derivative
if showPlot
    figure(2);
    plot(f_in);
    hold on;
    plot(f_out);
    title("Flows calculated using the water levels")
    legend("q_{in}", "q_{out}");
end


%% show filtered flows, and flow estimated from difference in water height
% Filtered flows are for illustrative purposes only

if showPlot
    figure();
    plot(lowpass(f_in,1,1/GateData.dt))
    hold on
    plot(lowpass(f_out,1,1/GateData.dt))
end


%% use least squares to find k analytically (k = (A'b) / (A'A))
A = GateData.actuators(firstMeasurementToUse:lastMeasurementToUse, gateToIdentify) ...
    .* sign(GateData.delta_height(firstMeasurementToUse:lastMeasurementToUse, gateToIdentify)) ...
    .* sqrt(abs(GateData.delta_height(firstMeasurementToUse:lastMeasurementToUse, gateToIdentify)));

AtA = 0;
Atb = 0;

if bitand(flowToUse, 1)
    AtA = AtA + A' * A;
    Atb = Atb + A' * f_in(firstMeasurementToUse:lastMeasurementToUse);
end

if bitand(flowToUse, 2)
    AtA = AtA + A' * A;
    Atb = Atb + A' * f_out(firstMeasurementToUse:lastMeasurementToUse);
end

k = Atb / AtA;

%% Show plot of the result
if showPlot
    plot(k * GateData.actuators(:,gateToIdentify) .* sign(GateData.delta_height(:,gateToIdentify)) .* sqrt(abs(GateData.delta_height(:,gateToIdentify))));

    title("Filtered flows and model comparision")
    legend("q_{in}", "q_{out}", "q_{est}");
end

end

