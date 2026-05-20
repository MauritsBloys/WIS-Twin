%% gate_identification.m

% Identify gate parameters

% simplified model:
% flow = sign(dh) * gate_opening * K * sqrt(abs(dh))
% K not constant, probably due to leakage so a better approximation will
% be:
% flow = sign(dh) * gate_opening * K(gate_opening) * sqrt(abs(dh))


% Identify 3 gates
% Identify 3 settings
%   - use only part where gate is fully open
% Plot results

Gate(1).data = [PoolData(1), PoolData(2), PoolData(3)];
Gate(1).setting = [25, 100, 255];
Gate(1).flow_to_use = [2, 3, 3];
Gate(1).k = zeros(1, size(Gate(1).setting, 2));

Gate(2).data = [PoolData(4), PoolData(5), PoolData(6)];
Gate(2).setting = [25, 100, 255];
Gate(2).flow_to_use = [2, 3, 3];
Gate(2).k = zeros(1, size(Gate(2).setting, 2));

Gate(3).data = [PoolData(7), PoolData(8), PoolData(9)];
Gate(3).setting = [25, 100, 255];
Gate(3).flow_to_use = [2, 3, 3];
Gate(3).k = zeros(1, size(Gate(3).setting, 2));

for iGate = 1: 3
    for i = 1:size(Gate(iGate).setting, 2)
        Gate(iGate).k(i) = calculateFlowConstant(Gate(iGate).data(i), iGate, Gate(iGate).setting(i), true, Gate(iGate).flow_to_use(i), show_gate_figures);
    end
    
end

coefficients = polyfit([Gate(1).setting Gate(2).setting Gate(3).setting],...
    [Gate(1).k Gate(2).k Gate(3).k], 1);

a_gate = coefficients (1);
b_gate = coefficients (2);

% store results in wis data
Wis.a_gate = a_gate;
Wis.b_gate = b_gate;

if show_gate_figures
    %% Plot results
    figure();
    hold on;
    for iGate = 1: 3
        scatter(Gate(iGate).setting, Gate(iGate).k);
    end
    legend("gate1", "gate2", "gate3");
    xlabel("servo setting");
    ylabel("\gamma_i");
    saveFigureEps(sprintf("gates_gamma"));

    %% Plot typical flows for linear approx
    gate_settings = [1, 5, 10, 25, 50, 100, 255];
    figure();
    for i = 1:size(gate_settings, 2)
        k = i * Wis.a_gate + Wis.b_gate;
        delta_height = 0:0.01:0.30;
        flow = i * k * sqrt(delta_height);
        plot(delta_height, flow);
        hold on;
    end
    xlabel("h1-h2 (m)");
    ylabel("flow m^3/s");
    legend(string(gate_settings));
    saveFigureEps(sprintf("estimated_possible_flows"));
    title("estimated flow over a gate for different servo settings");
end


