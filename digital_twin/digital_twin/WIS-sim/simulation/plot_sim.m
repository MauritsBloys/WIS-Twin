%% plot_sim.m

% TODO: load model params, operate switches and start sim 
% (cantoni_combined_matrices_3pools_gates) from here instead
% of preparing the data by hand

useCachedData = true;

experimentName = "full_sim_with_gates_valid";
%experimentName = "full_sim_valid";
%experimentName = "full_sim_exceed";
%experimentName = "full_sim_with_gates_exceed";

fileName = sprintf('mat/sim_%s.mat', experimentName);

%% Load cached, or save new data
if useCachedData
    try
        load(fileName);
    catch
        assert(false, "Data file does not exist");
    end
else
    save(fileName,'levels','u_global','u_local','u_local_restricted');    
end

    
figure();

plot(levels.Time, levels.Data);
hold on;

xlabel('time (s)')
ylabel('level (m)')
legend('pool1', 'pool2', 'pool3');

yline(0.25,'-','reference 1', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
yline(0.20,'-','reference 2', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
yline(0.15,'-','reference 3', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');

saveFigureEps(sprintf("sim-%s-levels", experimentName));

title("Water levels");

%saveas(gcf,'../Latex/images/pool123', 'epsc')

figure();

plot(u_global.Time, u_global.Data * 10000);
hold on;

plot(u_local.Time, u_local.Data * 10000);
hold on;

xlabel('time (s)')
ylabel('signal (x 10^{-4})'); 
legend('global1', 'local1', 'global2', 'local2', 'global3', 'local3');

saveFigureEps(sprintf("sim-%s-control", experimentName));

title("Control signals");

%figure();
% plot(pool_data(:,1)/1000, pool_data(:,5));
% hold on;
% 
% plot(pool_data(:,1)/1000, pool_data(:,5+6));
% hold on;
% 
% plot(pool_data(:,1)/1000, pool_data(:,5+12));
% hold on;
% 
% xlabel('time (s)')
% ylabel('servo')
% legend('gate1', 'gate2', 'gate3');
% 
% saveFigureEps(sprintf("exp_real_gates_%s", csvFile));
% 
% title("Gates");
% saveas(gcf,'../Latex/images/gates', 'epsc')
                  