% Test sleep_controller.m
warning ('on','all');

%% init
clear all;
disp("init plant");
init_plant;
%Copy_of_init_plant;
disp("init etc");
init_etc;
disp("init pstc");
init_pstc;
disp("init sim");
init_simulation;

y_log = [];
% Main simulation loop
kk = 1;

k_log = [];
sleeplog = [];

dk_log = [];

%% sim
yref = [0.25; 0.20; 0.15; 0; 0; 0] * 1000;

u_log = [];
t_log = [];

prevKK = 0;

while kk <= TEND/h
    %disp(kk)

    % calculate sleep (using old uhat)
    
    [dk, k, xc, xptilde, X, initialized, psibar] = sleepcontroller(...
        yhat, triggered, uhat, ...
        k, xc, xptilde, X, initialized, psibar, ...
        kfinal, kbar, TRIG_LEVEL, ...
        np, nc, pp, mp, nw, ppt, ...
        Ac, Bc, Cc, Dc, Cp, Phip, Gammap, ...
        Obsbar, Vbar, V, ...
        MM, Wk, QQ, Rw, Rv, wQw, cv, cvw);
    
    % Controller
    u = Cc*xc + Dc*yhat;
    xc = Ac*xc + Bc*yhat;
    
    % Control is sent with limited precision from Firefly to HIL
    scaledU = u ./ 1000; % CHECK!!!
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
    
    u_log = [u_log u];
    t_log = [t_log triggered];
    dk_log = [dk_log dk];
    
    if dk > 0  % There was a sleep command
        sleep = dk;
        uhat = u;
        if initialized
            xpw = xp;
            xpw(end) = omega(h*kk-1);
            %disp(h*kk); disp((xpw - xptilde)'*(X\(xpw - xptilde)))
        end
    end
    
    % Run plant forward
    
    % create a timing problem of max +- 1/8 period
    currKK = kk + (rand() - 0.5) * 0.25;
    [tt,xpode] = ode45(@(t,x) odeplant(t, x, uhat), h*[prevKK currKK], xp);
    
    prevKK = currKK;
    
    xp = xpode(end,:)';
    y = Cp*xp + noises(:, kk+1)*2;
    
    %initialized = false;
    
    % Trigger?  (I'm not using the sleep time here, but checking always)
    if initialized
        yh = y(1:ppt);  % heights
        yhhat = yhat(1:ppt);
        eh = yh - yhhat;
        % Use this to test prediction
%         if sum(sum(eh.^2,2) - sigma^2*sum(yh.^2,2)) > TRIG_LEVEL ||...
%                 k >= kfinal
%             triggered = true;
%             yhat = y;
%             
%             klog = [klog, k];
%             sleeplog = [sleeplog, sleep];
%         else
%             triggered = false;
%         end
        
        % Use this for the real deal
        if any(sum(eh.^2,2) - sigma^2*sum(yh.^2,2) > TRIG_LEVEL) ||...
                k >= kfinal
            triggered = true;
            yhat = y;
            
            k_log = [k_log, k];
            sleeplog = [sleeplog, sleep];
        else
            triggered = false;
        end
    else
        yhat = y;
        triggered = false;
    end
    
    y_log = [y_log, y];
            
    % Iterate
    kk = kk + 1;    
    
end

plot(y_log(1:3,:)' + yref(1:3)')