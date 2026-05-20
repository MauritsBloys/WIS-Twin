%% 

% Time series from PSTC and Python sim, with Data for 3 pools / FFs
% flow, level, servo, epoch, radio_on

target = [0.25 0.20 0.15];
showPlots = false;
showFigures = true; %true = Latex code for figures, false = Latex code for table


useCachedData = true;


nExperiments = 6;

% for report
experiments{1} = ["20210818temp_controllerlogging_python", "ETC+", "(0.1 1 demo re-init)", "20210818temp_controllerlogging_controller"];
experiments{2} = ["20210817etc_no_trigger_heartbeat30_python", "ETC", "(no trigger)", "20210817etc_no_trigger_heartbeat30_controller"];
experiments{3} = ["20210815etc0-1_1_python", "ETC", "(0.1 1)", "20210815etc0-1_1_controller"];
% has an error, but late in the experiment, so probably still usefull
experiments{4} = ["20210815test0-1_1_python", "ETC+", "(0.1 1)", "20210815test0-1_1_controller"];
experiments{5} = ["20210813etc0-2_2_python", "ETC", "(0.2 2)", "20210813etc0-2_2_controller"];
experiments{6} = ["20210813test0-2_2_python", "ETC+", "(0.2 2)", "20210813test0-2_2_controller"];


for iExperiments = 1: nExperiments

    experimentNameSim = experiments{iExperiments}(1);
    experimentNameCtrl = experiments{iExperiments}(4);
    
    fileName1 = sprintf('%s.mat', experimentNameSim);
    fileName2 = sprintf('%s.mat', experimentNameCtrl);
    fileName3 = sprintf('%s.mat', "sim_0-01-1");

    
    %% Load cached data
    try
        sensors = load(fileName1);
        controller = load(fileName2);
        sim = load(fileName3);
    catch
        assert(false, "Data file does not exist");
    end


    if showPlots   
        figure();
        plot(sensors.y_log');
        hold on;

        xlabel('time (s)')
        ylabel('level (m)')
        legend('pool1', 'pool2', 'pool3');

        yline(0.25,'-','reference 1', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
        yline(0.20,'-','reference 2', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');
        yline(0.15,'-','reference 3', 'LabelHorizontalAlignment', 'left', 'HandleVisibility','off');

        saveFigureEps(sprintf("%s-levels", experimentNameSim));

        title("Water levels");
    end


    % NOTE: there is a lag of 1 epoch in the timing of the radio
    if showPlots
        figure();
        yyaxis left
        plot(sensors.radio_log');
        ylabel('radio on (ms)')
        ylim([0 80]) % fix scale for comparison and crop to avoid scaling caused by single outliers

        hold on;
        yyaxis right
        plot(cumsum(sensors.radio_log')/1000);

        xlabel('time (s)')
        ylabel('total radio on (s)')
        ylim([0 50]) % fix scale for comparison
        legend('FF1', 'FF2', 'FF3');

        saveFigureEps(sprintf("%s-radio", experimentNameSim));

        title("Radio on");
    end

    % take average of 3 FF for more accurate result
    total_on = (sum(sum(sensors.radio_log'))/3)/1000; %s

    samples = size(sensors.y_log, 2);

    triggers = sum(sum(((sensors.radio_log')) > 50))/3;
    

    trigger_ratio = triggers / samples;

    error = (sensors.y_log' - target) * 1000; % mm
    mse = sum(sum((error .^2))) / (size(error, 1) );

    ise = 0;
    iae = 0;
    itse = 0;
    iate = 0;

    for t = 1:samples
        ise = ise + sum(error(t,:) .^2);
        iae = iae + sum(abs(error(t,:)));
        itse = ise + sum(error(t,:) .^2) * t;
        iate = iae + sum(abs(error(t,:))) * t;
    end
    
    mse = ise / samples;
    mtse = itse / samples;

    %disp(experimentName);
%     sum(controller.t_log')
    % trigger data from controller seems more reliable
    triggers = sum(controller.t_log');
%     sum(controller.initialized_log')

    if showFigures
        
disp(sprintf("        \\begin{figure}[H]"));
disp(sprintf("    \\begin{subfigure}{0.5\\linewidth}"));
disp(sprintf("        \\centering"));
disp(sprintf("        \\includegraphics[trim={0.0cm 0.0cm 0.0cm 0.0cm},clip,width=1\\textwidth]{images/%s-levels.pdf}", experiments{iExperiments}(1)));
disp(sprintf("        \\caption{Water levels}"));
disp(sprintf("        \\label{fig:%s-levels}", experiments{iExperiments}(1)));
disp(sprintf("    \\end{subfigure}"));
disp(sprintf("    \\begin{subfigure}{0.5\\linewidth}"));
disp(sprintf("        \\centering"));
disp(sprintf("        \\includegraphics[trim={0.0cm 0.0cm 0.0cm 0.0cm},clip,width=1\\textwidth]{images/%s-radio.pdf}", experiments{iExperiments}(1)));
disp(sprintf("        \\caption{Radio}"));
disp(sprintf("        \\label{fig:%s-radio}", experiments{iExperiments}(1)));
disp(sprintf("    \\end{subfigure}"));


disp(sprintf("    \\begin{subfigure}{0.5\\linewidth}"));
disp(sprintf("        \\centering"));
disp(sprintf("        \\includegraphics[trim={0.0cm 0.0cm 0.0cm 0.0cm},clip,width=1\\textwidth]{images/%s-controller_sleeping.pdf}", experiments{iExperiments}(1)));
disp(sprintf("        \\caption{Sleeping periods}"));
disp(sprintf("        \\label{fig:%s-controller_sleeping}", experiments{iExperiments}(1)));
disp(sprintf("    \\end{subfigure}"));
disp(sprintf("    \\begin{subfigure}{0.5\\linewidth}"));
disp(sprintf("        \\centering"));
disp(sprintf("        \\includegraphics[trim={0.0cm 0.0cm 0.0cm 0.0cm},clip,width=1\\textwidth]{images/%s-controller_init.pdf}", experiments{iExperiments}(1)));
disp(sprintf("        \\caption{Initialisation state}"));
disp(sprintf("        \\label{fig:%s-controller_init}", experiments{iExperiments}(1)));
disp(sprintf("    \\end{subfigure}"));

disp(sprintf("    \\caption{Time evolution of water levels, radio on time, calculated sleeping periods and initialisation state after a step disturbance on the HIL simulation using ETC+ with controller from Section\\,\\ref{sec:ctrl_local_controllers_exceed} ($\\sigma=%s, \\epsilon=%s$).}",experiments{iExperiments}(2), experiments{iExperiments}(3)));
disp(sprintf("    \\label{fig:%s}", experiments{iExperiments}(1)));
disp(sprintf("    \\end{figure}"));
    else
    disp(sprintf("%s & %s & %.1f & %d & %.0f & %.0f  \\\\", experiments{iExperiments}(2), experiments{iExperiments}(3), total_on, triggers, mse, mtse));
    end

    if showPlots 
        figure()
        plot(sensors.u_log');
        title('Sim - Control signal');

        % PSTC plots
        figure()
        plot(controller.u_log');
        ylim([0 20]);
        title('PSTC - Control signal');

    %     figure()
    %     plot(controller.dk_log');
    %     title('PSTC - Calculated sleeping times');

        figure();
        plot(controller.dk_log');
        xlabel('time (s)')
        ylabel('conservative sleeping periods')   
        saveFigureEps(sprintf("%s-controller_sleeping", experimentNameSim)); 
        title('PSTC - sleeping');

        figure()
        plot(controller.t_log');
        title('PSTC - Triggers');

    %     figure()
    %     plot(controller.initialized_log');
    %     title('PSTC - Initialisation state');

        figure();
        plot(controller.initialized_log');
        xlabel('time (s)')
        ylabel('initialisation state')

        saveFigureEps(sprintf("%s-controller_init", experimentNameSim)); 
        title('PSTC - Initialisation state');

        figure()
        plot(controller.y_log');
        title('PSTC - Water levels');

    %     figure()
    %     plot();
    %     title('PSTC - Radio');

        figure();
    %     yyaxis left
        plot(controller.radio_log');
        xlabel('time (s)')
        ylabel('radio on (ms)')
        ylim([0 100]) % fix scale for comparison and crop to avoid scaling caused by single outliers

        saveFigureEps(sprintf("%s-controller_radio", experimentNameSim));

        title("Radio on - Controller");
    end
    
end



