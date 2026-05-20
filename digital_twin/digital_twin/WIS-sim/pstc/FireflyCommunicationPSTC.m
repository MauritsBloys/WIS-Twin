classdef FireflyCommunicationPSTC < handle
    %FIREFLYCOMMUNICATIONPSTC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        port                                % comm port
        baudrate = 115200                   % comm speed
        device                              % serial port connection
        n_values = 13                       % num of values (doubles) per message
        log_data = []                       % log of all received data
        DEBUG = 1                           % show debug messages
        
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

        % Digital twin
        twin_active = false

        % logging
        y_log = [];
        yd_log = [];
        k_log = [];
        sleeplog = [];
        dk_log = [];
        u_log = [];
        t_log = [];
        xc_log = [];
        initialized_log = [];
        epoch_log = [];
        radio_log = [];   
        
        end_epoch = -1;
        error_log = [];
        current_epoch = -1;
    end
    
    methods
        function obj = FireflyCommunicationPSTC(port, ...
                k, xc, xptilde, X, initialized, psibar, ...
                kfinal, kbar, TRIG_LEVEL, ...
                np, nc, pp, mp, nw, ppt, ...
                Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
                Obsbar, Vbar, V, ...
                MM, Wk, QQ, Rw, Rv, wQw, cv, cvw, ...
                h, xp)
            %FIREFLYCOMMUNICATIONPSTC Construct an instance of this class
            %   Detailed explanation goes here
            obj.port = port;
            
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
            
            
        end
        
        function connect(obj)
            % connect
            % suppress warnings about timeout, because we need this small
            % timeout to avoid blocking write
            warning ('on','all');
            obj.device = serialport(obj.port, obj.baudrate, "Timeout",0.1);
            configureTerminator(obj.device,"LF")
            
            % activate callback
            obj.activate();
        end
        
        function activate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %configureCallback(obj.device, "byte", 4*obj.n_values, @obj.callbackMessage);
            configureCallback(obj.device,"terminator",@obj.callbackMessage)

        end
        
        function deactivate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            configureCallback(obj.device, "off");
        end
        
        function callbackMessage(obj, device, ~)
            %         // epoch (1)
            %         // radio on (2)
            %         // send global u (3, 4, 5)
            %         // send water levels (6, 7, 8, 9)
            %         // send water flows (10, 11, 12)
            %         // event (13)
%             data = read(device, obj.n_values, "double");
%             if obj.DEBUG
%                 disp(data(1));
%             end

            serial_data = readline(device);
            disp(serial_data)
            
            data = split(serial_data, ",");
            
            %assert(size(command,1) == 4, "Received garbled data.");
            if size(data,1) == obj.n_values
                % only handle correctly received data
                
                obj.log_data = [obj.log_data; data];
           
                % handle data and send extra sleeping

                % multiply values by 1000 for pstc scripts

                % level pool 1, 2, 3 and flow 1, 2, 3
                yhat = [double(data(7))/1000000; double(data(8))/1000000; double(data(9))/1000000]; %data(10); data(11); data(12)];
                yref = [0.25; 0.20; 0.15];% 0; 0; 0];

                yhat = yhat - yref;
                yhat = yhat * 1000; % to mm

                % event detected?
                triggered = double(data(13));
                
                if triggered == 2
                    % test: force re-init on bad reception
                    disp("COMMUNICATION ERROR WITH ONE OF THE FIREFLIES, REINIT");
                    obj.initialized = false;
                end

                %             % global control gate 1, 2, 3
                %             u = [data(3); data(4); data(5)] * 1000;

                % local control flow 1, 2, 3
                uhat = [double(data(10))/1000; double(data(11))/1000; double(data(12))/1000]; % /1000000 * 1000 = /1000;

                if any(abs(uhat) > 100) || any(abs(yhat) > 100)
                    disp("ERROR: communication error, skip (should only happen at at start)")
                    return
                end
                % radio
                radio_on = double(data(2));
                epoch = double(data(1));

                % Digital twin update (only when twin_active = true)
                if obj.twin_active
                    y_abs = [double(data(7)); double(data(8)); double(data(9))] / 1e6;
                    u_col = [double(data(3)); double(data(4)); double(data(5))] / 1000;
                    addpath(fullfile(fileparts(mfilename('fullpath')), '../../WIS-twin'));
                    twin_update_hardware(y_abs, u_col, epoch);
                end

                if obj.current_epoch > -1
                    assert(epoch == obj.current_epoch +1, "Missed an epoch!");
                    obj.current_epoch = epoch;    
                end

                if obj.end_epoch == -1
                    obj.end_epoch = epoch + 1800;
                else
                    if epoch > obj.end_epoch
                        obj.deactivate(); % stop after 1800 epochs (==30 mins with no extra sleep)
                    end
                end

                replay = [uhat; yhat; triggered];
                obj.replay_data = [obj.replay_data replay];



                % we now have uhat and yhat (==y if there is a trigger)

                % calculate sleep (using old uhat)
                
                if ~obj.initialized
                    triggered = false;
                end

                dk = obj.sleepcontroller(yhat, triggered, uhat);

                obj.u_log = [obj.u_log uhat];
                obj.t_log = [obj.t_log triggered];
                obj.dk_log = [obj.dk_log dk];
                obj.initialized_log = [obj.initialized_log obj.initialized];
                obj.y_log = [obj.y_log yhat];

                obj.epoch_log = [obj.epoch_log epoch];
                obj.radio_log = [obj.radio_log radio_on];

                % Controller
                obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                obj.xc_log = [obj.xc_log obj.xc];

%                 % never extra sleep = normal ETC
%                 dk = 1;
                
                if dk > 0  % There is a sleep command, so there was a trigger
                    sleep = dk;

                    % update for trivial sleep already done, and sending is not
                    % necessary
                    
                    if ~obj.initialized
                        % not initialised yet, so force an update
                        obj.sendMessage("0 0");
                        disp("Force update!");
                    end

                    if sleep > 2

                        % notify network controller about extra sleep
                        msg = sprintf("0 %s", string(dk-1));
                        obj.sendMessage(msg);
                        disp("Extra sleep possible!");
                        disp(sleep);
                        disp(msg);

                        % Update sleepcontroller and plant controller state
                        for i = 2:sleep

                            % forward pstc and controller for extra sleeping
                            [~] = obj.sleepcontroller(yhat, false, uhat);

                            % Controller
                            obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                            obj.xc_log = [obj.xc_log obj.xc];

                            obj.u_log = [obj.u_log uhat];
                            obj.t_log = [obj.t_log 0]; % no trigger in sleep
                            obj.dk_log = [obj.dk_log 0]; % no prediction in sleep
                            obj.initialized_log = [obj.initialized_log 1]; % initialized in sleep
                            obj.y_log = [obj.y_log yhat];

                            obj.epoch_log = [obj.epoch_log epoch];
                            obj.radio_log = [obj.radio_log 0];

                        end
                    end
                end

                fprintf("# k=%d u =(%d, %d, %d) y = (%d, %d, %d) trigger=%d, dk=%d \n", obj.k, uhat(1), uhat(2), uhat(3), yhat(1), yhat(2), yhat(3), triggered, dk);
                
                obj.error_log = [obj.error_log [epoch; 0]];
            else
                % error, skip... (log although we don't know the epoch)
                disp('ERROR: communication error');
                disp(data);
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
            
%             SPS = 8;
%             
%             Apd = [1 0.1180160773202 0 0 0 0;0 0.825052966980536 0 0 0 0;0 0 1 0.316267336378338 0 0;0 0 0 0.69967253737513 0 0;0 0 0 0 1 0.084202651990668;0 0 0 0 0 0.846481724890614];
%             Bpd = [-0.00921309074701414 -0.0112430293218205 0;0.0303241523900404 0 0;0 -0.0119670350796699 -0.017551249648975;0 0.0280305631783212 0;0 0 -0.00769909409566176;0 0 0.0307036550218772];
            
            Epd = -0.015/0.2279*1/60* 1/SPS * [0; 0; 0; 0; 1; 0];
            
            xp =[0.25; 0; 0.20; 0; 0.15; 0]; % digital model does not have the disturbance state
            yref = [0.25; 0.20; 0.15];
            
            
            % init timestep
            kk = 1;
            prevKK = 0;
            
            Wis = load("../identification/identification.mat");
            
            % simulation loop
            while kk <= TEND/obj.h
                
                % calculate sleep (using old uhat)
                
                % limit precision send to pstc
                uhat = (round(uhat * 1000))/1000;
                yhat = (round(yhat * 1000))/1000;
                
                dk = obj.sleepcontroller(yhat, triggered, uhat);
                
                % Controller
                u = obj.Cc*obj.xc + obj.Dc*yhat;
                obj.xc = obj.Ac*obj.xc + obj.Bc*yhat;
                
                
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
                    
                    % Update plant state, disturbance starts after 20 secs
                    xp = Apd*xp + Bpd*uhat/1000 + Epd * (kk*obj.h >= 20);
                    
                    yd = Cpd*xp; % HIL sim has no noise + noises(:, kk+1)*2/1000;
                    

                    
                    obj.yd_log = [obj.yd_log, yd];
                    
                    % limit sensor accuracy, based on pressure sensors
                    % convert water levels to pressure
                    levels = [0.30, yd(1), yd(1), yd(2), yd(2), yd(3), yd(3)];
                    pressures = ((levels * 100) - Wis.Wis.b) ./ Wis.Wis.a;
                    pressures = round(pressures);
                    levels = pressures .* Wis.Wis.a + Wis.Wis.b;
                    yd = ([levels(3); levels(5); levels(7)])/100;
                    
                    % remove ref
                    yd = yd - yref;
                    
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
        
        function sendMessage(obj, message)
            if obj.DEBUG
                fprintf("Send: %s\n", message);
            end
            writeline(obj.device, message)
        end
        
        function saveReplayData(obj, filename)
            replayData = obj.replay_data;
            
            u_log = obj.u_log;
            dk_log = obj.dk_log;
            
            t_log = obj.t_log;
            initialized_log = obj.initialized_log;
            y_log = obj.y_log;
            
            epoch_log = obj.epoch_log;
            radio_log = obj.radio_log;
            data_log = obj.log_data;
            
            error_log = obj.error_log;
            
            xc_log = obj.xc_log;
            
            save(filename, 'replayData', 'u_log', 'dk_log', 't_log', 'initialized_log', 'y_log', 'radio_log', 'epoch_log', 'data_log', 'error_log', 'xc_log');
        end
        
        function loadReplayData(obj, filename)
            load(filename, 'replayData');
            
            obj.replay_data = replayData;
        end
        
    end
end

