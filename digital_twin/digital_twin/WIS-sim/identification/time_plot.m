%% time_plot.m

% Show a graph to show time evaluation of an experiment

PlotData = createWisData("20210202_step_gate3_4_s255_no_intake.csv", Wis, 3, "experiment", "step 255 pool 3", 1/128);

figure();
plot(PlotData.timing/1000, PlotData.water_levels);
legend("s1", "s2", "s3", "s4", "s5", "s6", "s7")
xlabel('time (s)')
ylabel('level (m)')
saveFigureEps("water_levels_identification");

title("Water levels");
             