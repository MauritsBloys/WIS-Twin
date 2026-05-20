classdef FireflySimulationPSTC < handle
    %FIREFLYCOMMUNICATIONPSTC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        port                                % comm port
        baudrate = 115200                   % comm speed
        device                              % serial port connection
        n_values = 13                       % num of values (doubles) per message
        log_data = []                       % log of all received data
        DEBUG = 1                           % show debug messages
        
        flows = [0, 0, 0, 0]
        radios = [0, 0, 0, 0]
        sleep = [1, 1, 1, 1]
        pressure = [0, 0, 0, 0, 0, 0, 0, 0]
        simulationHasRun = true;
        
        % PSTC
        k, xc, xptilde, X, initialized, psibar,
        kfinal, kbar, TRIG_LEVEL,
        np, nc, pp, mp, nw, ppt,
        Ac, Bc, Cc, Dc, Cp, Phip, Gammap,
        Obsbar, Vbar, V,
        MM, Wk, QQ, Rw, Rv, wQw, cv, cvw
        
        % TODO: add to constructor
        h = 1
        xp
        
        replay_data = []
        
        % logging
        y_log = [];
        yd_log = [];
        k_log = [];
        sleeplog = [];
        dk_log = [];
        u_log = [];
        t_log = [];
        epoch_log = [];
        radio_log = [];
        
        initialized_log = [];
        
        SPS
        Apd
        Bpd
        Cpd
        Dpd
        Epd
        xpd
        kk = 1
        
        Wis
        yd
        
        end_epoch = -1;
        error_log = [];
        
    end
    
    methods
        function obj = FireflySimulationPSTC(port1, port2, port3, port4, ...
                k, xc, xptilde, X, initialized, psibar, ...
                kfinal, kbar, TRIG_LEVEL, ...
                np, nc, pp, mp, nw, ppt, ...
                Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
                Obsbar, Vbar, V, ...
                MM, Wk, QQ, Rw, Rv, wQw, cv, cvw, ...
                h, xp, Wis)
            %FIREFLYCOMMUNICATIONPSTC Construct an instance of this class
            %   Detailed explanation goes here
            obj.port{1} = port1;
            obj.port{2} = port2;
            obj.port{3} = port3;
            obj.port{4} = port4;
            
            obj.k = k;
            obj.xc = xc;
            obj.xptilde = xptilde;
            obj.X = X;
            obj.initialized = initialized;
            obj.psibar = psibar;
            obj.kfinal = kfinal;
            obj.kbar = kbar;
            obj.TRIG_LEVEL = TRIG_LEVEL;
            obj.np = np;
            obj.nc = nc;
            obj.pp = pp;
            obj.mp = mp;
            obj.nw = nw;
            obj.ppt = ppt;
            obj.Ac = Ac;
            obj.Bc = Bc;
            obj.Cc = Cc;
            obj.Dc = Dc;
            obj.Cp = Cp;
            obj.Phip = Phip;
            obj.Gammap = Gammap;
            obj.Obsbar = Obsbar;
            obj.Vbar = Vbar;
            obj.V = V;
            obj.MM = MM;
            obj.Wk = Wk;
            obj.QQ = QQ;
            obj.Rw = Rw;
            obj.Rv = Rv;
            obj.wQw = wQw;
            obj.cv = cv;
            obj.cvw = cvw;
            
            obj.h = h;
            obj.xp = xp;
            
            
            % Discrete plant model, copied from HIL (zoh)
            obj.SPS = 1;
            
            obj.Apd = [1 0.529741518927619 0 0 0 0;0 0.214711172341697 0 0 0 0;0 0 1 0.992594124612871 0 0;0 0 0 0.0574326192676173 0 0;0 0 0 0 1 0.403906791292383;0 0 0 0 0 0.263597138115727];
            obj.Bpd = [-0.00187762870622354 -0.0899442345745638 0;0.136116730127439 0 0;0 0.0477678788945988 -0.1404099971918;0 0.0879729555350224 0;0 0 -0.00764986783870191;0 0 0.147280572376855];
            obj.Cpd = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
            obj.Dpd = [0 0 0;0 0 0;0 0 0];
            
            obj.Epd = -0.015/0.2279*1/60* 1/obj.SPS * [0; 0; 0; 0; 1; 0];
            
             
            obj.xpd =[0.25; 0; 0.20; 0; 0.15; 0]; % digital model does not have the disturbance state
            
           obj.Wis = Wis;
            
            
        end
        
        function connect(obj)
            % connect
            % suppress warnings about timeout, because we need this small
            % timeout to avoid blocking write
            warning ('on','all');
            for i = 1:4
                obj.device{i} = serialport(obj.port{i}, obj.baudrate, "Timeout",0.1);
                configureTerminator(obj.device{i},"LF")

            end
            
            % activate callback
            obj.activate();
        end
        
        function activate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            for i = 1:4
                configureCallback(obj.device{i},"terminator",@obj.callbackMessage)

            end
            
        end
        
        function deactivate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            for i = 1:4
                configureCallback(obj.device{i}, "off");
            end
            
        end
        
        
        function handleMessage(obj, command, id, epoch, data)
            if (id == 1)
                disp(sprintf("%d - %s",epoch,command));
            end
            obj.error_log = [obj.error_log [epoch; 0]];
            if (command == "r")
                %disp("sensor update request");
                % run sim on first request
                % store radio on time
                obj.radios(id) = data;
                if (~obj.simulationHasRun)
                    disp("run sim");
                    % Discrete simulation of the plant

                    % Update plant state, disturbance starts after 20 secs
                    
                    u = [obj.flows(1); obj.flows(2); obj.flows(3)];
                    for i = 1:obj.sleep(id) % use sleep time from the node that woke me
                        for j = 1:obj.SPS
                            obj.xpd = obj.Apd*obj.xpd + obj.Bpd*u + obj.Epd * (obj.kk*obj.h >= 20);
                            obj.kk = obj.kk +1;
                        end

                        obj.yd = obj.Cpd*obj.xpd; % HIL sim has no noise + noises(:, kk+1)*2/1000;
           
                        obj.u_log = [obj.u_log u];
                        obj.yd_log = [obj.yd_log obj.yd];
                        obj.y_log = [obj.yd_log obj.yd];
                        obj.epoch_log = [obj.epoch_log epoch];
                        obj.k_log = [obj.k_log obj.kk];
                        
                        %  add extra entries in radio log to show radio is
                        %  off. Note that the radio normally lags 1 period
                        %  so when doing extra sleep it lags multiple
                        if (i > 1) 
                            obj.radio_log = [obj.radio_log [0 0 0 0]];
                        end
                        
                    end
                   
                    obj.simulationHasRun = true;
                    
                    % convert water levels to pressure
                    levels = [0.30 obj.yd(1) obj.yd(1) obj.yd(2) obj.yd(2) obj.yd(3) obj.yd(3)];
                    obj.pressure(:, 1:7) = ((levels*100) - obj.Wis.b) ./ obj.Wis.a;
                    
                end
                
            
                % send pressure (node_id s1 s2\n)
                msg = sprintf("%d %d %d", (id + 200), round(obj.pressure(id*2-1)), round(obj.pressure(id*2)));
                obj.sendMessage(id, msg);
            end
            if (command == "f")
                %disp("flow update request");
                % save requested flow (scale back to m^3/s)
                obj.flows(id) = data/1000000;
            end
            
            if (command == "s")
                % update radios now we all have them (only once!)
                if (id == 1)
                    obj.radio_log = [obj.radio_log obj.radios'];
                    if obj.end_epoch == -1
                        obj.end_epoch = epoch + 1800;
                    else
                        if epoch > obj.end_epoch
                            obj.deactivate(); % stop after 1800 epochs (==30 mins with no extra sleep)
                        end
                    end
                end
                %disp("sleep update request");
                obj.sleep(id) = data;
                % reset sim
                obj.simulationHasRun = false;
            end            
        end
        
        function callbackMessage(obj, device, ~)

            data = readline(device);
            %disp(data)
            
            command = split(data, ",");
            
            %assert(size(command,1) == 4, "Received garbled data.");
            if size(command,1) == 4
                obj.handleMessage(command(1), double(command(2))-200, double(command(3)), double(command(4)))    
            else
                % error, skip... (log although we don't know the epoch)
                obj.error_log = [obj.error_log [0; 1]];
            end
        end
        
        function [dk] = sleepcontroller(obj, yhat, triggered, uhat)
            % calculate sleep (using old uhat)
            
            [dk, obj.k, obj.xc, obj.xptilde, obj.X, obj.initialized, obj.psibar] = sleepcontroller(...
                yhat, triggered, uhat, ...
                obj.k, obj.xc, obj.xptilde, obj.X, obj.initialized, obj.psibar, ...
                obj.kfinal, obj.kbar, obj.TRIG_LEVEL, ...
                obj.np, obj.nc, obj.pp, obj.mp, obj.nw, obj.ppt, ...
                obj.Ac, obj.Bc, obj.Cc, obj.Dc, obj.Cp, obj.Phip, obj.Gammap, ...
                obj.Obsbar, obj.Vbar, obj.V, ...
                obj.MM, obj.Wk, obj.QQ, obj.Rw, obj.Rv, obj.wQw, obj.cv, obj.cvw);
        end
        
        function [] = runSimulationGabriel(obj, noises, TEND, yhat, triggered, uhat, odeplant, omega, sigma, Ap, Bp, Cp, Dp)
            
            % init timestep
            kk = 1;
            prevKK = 0;
            
            % simulation loop
            while kk <= TEND/obj.h
                
                % calculate sleep (using old uhat)
                
                dk = obj.sleepcontroller(yhat, triggered, uhat);
                
                % Controller
                u = obj.Cc*obj.xc + obj.Dc*yhat;
                obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                
                % Control is sent with limited precision from Firefly to HIL
                scaledU = u ./ 1000;
                for i = 1:3
                    if (scaledU(i) < -0.01)
                        scaledU(i) = 0;
                    else
                        if (scaledU(i) > 0.05)
                            scaledU(i) = 65535;
                        else
                            scaledU(i) = round(((scaledU(i) + 0.01) * 1092250));
                        end
                    end
                    
                    u(i) = ((scaledU(i) / 1092250) - 0.01) * 1000;
                end
                
                % log u (flow), trigger, calculated sleeping periods
                obj.u_log = [obj.u_log u];
                obj.t_log = [obj.t_log triggered];
                obj.dk_log = [obj.dk_log dk];
                
                
                if dk > 0  % There was a sleep command, so there was a trigger
                    sleep = dk;
                    uhat = u;
                    % Ask Gabriel what this is for
                    if obj.initialized
                        xpw = obj.xp;
                        xpw(end) = omega(obj.h*kk-1);
                        %disp(h*kk); disp((xpw - xptilde)'*(X\(xpw - xptilde)))
                    end
                end
                % Run plant forward
                
                % create a timing problem of max +- 1/8 period
                currKK = kk + (rand() - 0.5) * 0.25;
                [tt,xpode] = ode45(@(t,x) odeplant(t, x, uhat), obj.h*[prevKK currKK], obj.xp);
                prevKK = currKK;
                
                obj.xp = xpode(end,:)';
                y = obj.Cp*obj.xp + noises(:, kk+1)*2;
                
                % Trigger?  (I'm not using the sleep time here, but checking always)
                if obj.initialized
                    yh = y(1:obj.ppt);  % heights
                    yhhat = yhat(1:obj.ppt);
                    eh = yh - yhhat;
                    
                    % Use this for the real deal
                    if any(sum(eh.^2,2) - sigma^2*sum(yh.^2,2) > obj.TRIG_LEVEL) ||...
                            obj.k >= obj.kfinal
                        triggered = true;
                        yhat = y;
                        
                        obj.k_log = [obj.k_log, obj.k];
                        obj.sleeplog = [obj.sleeplog, sleep];
                    else
                        triggered = false;
                    end
                else
                    yhat = y;
                    triggered = false;
                end
                
                obj.y_log = [obj.y_log, y];
                
                % Iterate
                kk = kk + 1;
                
            end
            
        end
        
        
        function [] = runSimulation(obj, noises, TEND, yhat, triggered, uhat, odeplant, omega, sigma, Ap, Bp, Cp, Dp)
            % Discrete plant model, copied from HIL (zoh)
            SPS = 1;
            
            Apd = [1 0.529741518927619 0 0 0 0;0 0.214711172341697 0 0 0 0;0 0 1 0.992594124612871 0 0;0 0 0 0.0574326192676173 0 0;0 0 0 0 1 0.403906791292383;0 0 0 0 0 0.263597138115727];
            Bpd = [-0.00187762870622354 -0.0899442345745638 0;0.136116730127439 0 0;0 0.0477678788945988 -0.1404099971918;0 0.0879729555350224 0;0 0 -0.00764986783870191;0 0 0.147280572376855];
            Cpd = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
            Dpd = [0 0 0;0 0 0;0 0 0];
            
            SPS = 8;
            
            Apd = [1 0.1180160773202 0 0 0 0;0 0.825052966980536 0 0 0 0;0 0 1 0.316267336378338 0 0;0 0 0 0.69967253737513 0 0;0 0 0 0 1 0.084202651990668;0 0 0 0 0 0.846481724890614];
            Bpd = [-0.00921309074701414 -0.0112430293218205 0;0.0303241523900404 0 0;0 -0.0119670350796699 -0.017551249648975;0 0.0280305631783212 0;0 0 -0.00769909409566176;0 0 0.0307036550218772];
            
            Epd = -0.015/0.2279*1/60* 1/SPS * [0; 0; 0; 0; 1; 0];
            
            xp =[0; 0; 0; 0; 0; 0]; % digital model does not have the disturbance state
            
            % init timestep
            kk = 1;
            prevKK = 0;
            
            % simulation loop
            while kk <= TEND/obj.h
                
                % calculate sleep (using old uhat)
                
                dk = obj.sleepcontroller(yhat, triggered, uhat);
                
                % Controller
                u = obj.Cc*obj.xc + obj.Dc*yhat;
                obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                
                % Control is sent with limited precision from Firefly to HIL
                scaledU = u ./ 1000;
                for i = 1:3
                    if (scaledU(i) < -0.01)
                        scaledU(i) = 0;
                    else
                        if (scaledU(i) > 0.05)
                            scaledU(i) = 65535;
                        else
                            scaledU(i) = round(((scaledU(i) + 0.01) * 1092250));
                        end
                    end
                    
                    u(i) = ((scaledU(i) / 1092250) - 0.01) * 1000;
                end
                
                % log u (flow), trigger, calculated sleeping periods
                obj.u_log = [obj.u_log u];
                obj.t_log = [obj.t_log triggered];
                obj.dk_log = [obj.dk_log dk];
                
                if dk > 0  % There was a sleep command, so there was a trigger
                    sleep = dk;
                    uhat = u;
                    triggered = false;
                end
                
                % --- START HERE ---
                % Run plant forward
                
                % trivial sleep when we wait for a trigger
                if dk == 0
                    sleep = 1;
                end
                
                % No updates on yhat and uhat during sleep, but
                % sleepcontroller and plant controller must be propagated
                % based on these kept values
                for i = 1:sleep
                    
                    % forward pstc and controller for extra sleeping
                    if i > 1
                        [~] = obj.sleepcontroller(yhat, triggered, uhat);
                        
                        % Controller
                        obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                    end
                    
                    % Continuous simulation of the plant
                    % create a timing problem of max +- 1/8 period
                    %                     currKK = kk + (rand() - 0.5) * 0.25;
                    %                     [tt,xpode] = ode45(@(t,x) odeplant(t, x, uhat), obj.h*[prevKK currKK], obj.xp);
                    %                     prevKK = currKK;
                    %                     obj.xp = xpode(end,:)';
                    %
                    %                     y = obj.Cp*obj.xp + noises(:, kk+1)*2;
                    
                    % Discrete simulation of the plant
                    % Create timing problem of a few samples
                    if SPS > 1
                        timingProblem = round((rand() - 0.5) * 0);
                    else
                        timingProblem = 0; % cannot create timing problem for testcase period=1
                    end
                    
                    % Update plant state, disturbance starts after 20 secs
                    for j = 1:SPS+timingProblem
                        xp = Apd*xp + Bpd*uhat/1000 + Epd * (kk*obj.h >= 20);

                        dt_sub = obj.h / SPS;
                        h0_val = obj.Wis.h0;
                        q1 = wis_leakage(h0_val,  xp(1), obj.Wis.leak_alpha(1), obj.Wis.leak_beta(1));
                        q2 = wis_leakage(xp(1),   xp(3), obj.Wis.leak_alpha(2), obj.Wis.leak_beta(2));
                        q3 = wis_leakage(xp(3),   xp(5), obj.Wis.leak_alpha(3), obj.Wis.leak_beta(3));
                        xp(1) = xp(1) + (q1 - q2) * dt_sub / obj.Wis.area1;
                        xp(3) = xp(3) + (q2 - q3) * dt_sub / obj.Wis.area2;
                        xp(5) = xp(5) + q3        * dt_sub / obj.Wis.area3;
                    end
                    
                    yd = Cpd*xp; % HIL sim has no noise + noises(:, kk+1)*2/1000;
                    
                    % limit accuracy
                    factor = 1000000;
                    yd = (round(yd * factor))/factor;
                    
                    
                    obj.yd_log = [obj.yd_log, yd];
                    
                    % use digital y
                    y = yd*1000;
                    
                    
                    obj.y_log = [obj.y_log, y];
                    
                    % Iterate
                    kk = kk + 1;
                end
                
                % Trigger?  (I'm not using the sleep time here, but checking always)
                if obj.initialized
                    yh = y(1:obj.ppt);  % heights
                    yhhat = yhat(1:obj.ppt);
                    eh = yh - yhhat;
                    
                    % Use this for the real deal
                    if any(sum(eh.^2,2) - sigma^2*sum(yh.^2,2) > obj.TRIG_LEVEL) ||...
                            obj.k >= obj.kfinal
                        triggered = true;
                        yhat = y;
                        
                        obj.k_log = [obj.k_log, obj.k];
                        obj.sleeplog = [obj.sleeplog, sleep];
                    else
                        triggered = false;
                    end
                else
                    yhat = y;
                    triggered = false;
                end
                
            end
            
        end
        
        
        
        function [] = runReplay(obj)
            
            
            obj.xc = obj.xc * 0; % reset controller
            for i = 1:size(obj.u_log,2)
                i
                yhat = obj.y_log(:, i);
                triggered = obj.t_log(:, i);
                uhat = obj.u_log(:, i);
                
                % calculate sleep (using old uhat)
                
                dk = obj.sleepcontroller(yhat, triggered, uhat);
                
                % Controller
                obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                
                if dk > 0  % There is a sleep command, so there was a trigger
                    sleep = dk;
                    %uhat = u;
                    
                    
                    % update for trivial sleep already done, and sending is not
                    % necessary
                    if sleep > 1
                        % notify network controller about extra sleep
                        msg = sprintf("0 %s", string(dk-1));
                        %obj.sendMessage(msg);
                        disp("Extra sleep possible!");
                        disp(msg);
                        
                        % Update sleepcontroller and plant controller state
                        %                     for i = 2:sleep
                        %
                        %                         % forward pstc and controller for extra sleeping
                        %                         [~] = obj.sleepcontroller(yhat, false, uhat);
                        %
                        %                         % Controller
                        %                         obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                        %                     end
                    end
                end
                
                %fprintf("# k=%d u =(%d, %d, %d) trigger=%d, dk=%d \n", obj.k, uhat(1), uhat(2), uhat(3), triggered, dk);
                
                
            end
            
            
            
            
            
            
            
            
            
        end
        
        
        function dk = updatePstc(obj, u, yhat, triggered)
            [dk, obj.k, obj.xc, obj.xptilde, obj.X, obj.initialized, obj.psibar] = sleepcontroller(...
                u, yhat, triggered, ...
                obj.k, obj.xc, obj.xptilde, obj.X, obj.initialized, obj.psibar, ...
                obj.kfinal, obj.kbar, obj.TRIG_LEVEL, ...
                obj.np, obj.nc, obj.pp, obj.mp, obj.nw, obj.ppt, ...
                obj.Ac, obj.Bc, obj.Cc, obj.Dc, obj.Cp, obj.Phip, obj.Gammap, ...
                obj.Obsbar, obj.Vbar, obj.V, ...
                obj.MM, obj.Wk, obj.QQ, obj.Rw, obj.Rv, obj.wQw, obj.cv, obj.cvw);
        end
        
        function sendMessage(obj, id, message)
%             if obj.DEBUG
%                 fprintf("Send: %s\n", message);
%             end
            writeline(obj.device{id}, message)
        end
        
        function saveReplayData(obj, filename)
            replayData = obj.replay_data;
            
            u_log = obj.u_log;
            y_log = obj.y_log;
            

            epoch_log = obj.epoch_log;
            k_log = obj.k_log;
            
            radio_log = obj.radio_log;
                        
            error_log = obj.error_log;
            save(filename, 'replayData', 'u_log', 'y_log', 'epoch_log', 'k_log', 'radio_log', 'error_log');
        end
        
        function loadReplayData(obj, filename)
            load(filename, 'replayData');
            
            obj.replay_data = replayData;
        end
        
    end
end

