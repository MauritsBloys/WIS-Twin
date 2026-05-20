%% identification.m

% Identify all pools

nPoolData = size(PoolData, 2); % total number of data files

% identify all pools
for iPool = 1:3
    
    %% collect experiment data
    clear ze

    % keep track of first so we know if we need to merge
    firstIddata = true;

    % search all data, for data labeled 'experiment' 
    for i = 1:nPoolData
        if PoolData(i).pool == iPool
            if PoolData(i).type == "experiment"
                if firstIddata
                    if (iPool == 3)
                        % only plot first dataset of pool 3 for
                        % illustration of the data in report
                        ze = createIddata(PoolData(i), Wis, false, show_identification_figures);
                    else
                        ze = createIddata(PoolData(i), Wis, false, false);
                    end
                    firstIddata = false;
                else
                    ze = merge(ze, createIddata(PoolData(i), Wis, false, false));
                end
            end
        end
    end


    %% collect validation data
    clear zv

    % keep track of first so we know if we need to merge
    firstIddata = true;

    % search all data, for data labeled 'validation' 
    for i = 1:nPoolData
        if PoolData(i).pool == iPool
            if PoolData(i).type == "validation"
                if firstIddata
                    zv = createIddata(PoolData(i), Wis, false, false);
                    firstIddata = false;
                else
                    zv = merge(zv, createIddata(PoolData(i), Wis, false, false));
                end
            end
        end
    end

    %% identify

    % create third order model with pole at origin (type 1) with a transport
    % delay for identification
    init_sys = idtf(NaN(1,1),[1,NaN(1,2),0],'IODelay',Wis.delays(iPool));

    % fix last coefficient, to force tfest to put an integrator 1/s in the
    % transferfunction
    init_sys.Structure.Denominator.Free = [0 1 1 0];

    Opt = tfestOptions('Display','on');
    
    clear mtf;

    mtf = tfest(ze,init_sys,Opt);
    
    %% validate
    if show_identification_figures
        figure();
        [y,fit,x0] = compare(zv,mtf);
        compare(zv,mtf); % for plotting
        saveFigureEps(sprintf("validation_pool_%d", iPool));
    else
        [~,fit,~] = compare(zv,mtf);
    end
    
    PoolModel(iPool).experiment = ze;
    PoolModel(iPool).validation = zv;
    PoolModel(iPool).tf = mtf;
    PoolModel(iPool).validation_fit = fit;
    PoolModel(iPool).tau = Wis.delays(iPool);

end

%% Bode plot of the results
if show_identification_figures
    figure();
    bode(PoolModel(1).tf, PoolModel(2).tf, PoolModel(3).tf)
    legend("pool1", "pool2", "pool3");
    hold on;
    saveFigureEps("dominant_wave_freq");
end

% structure of the tf is a delayed second order system combined 
% with an integrator

%                              omega_n^2                      1
%  G(s) =   e^-st * ----------------------------------  *   -----
%                   s^2 + 2 zeta omega_n s + omega_n^2        s

%% Calculate dominant wave frequency for each pool
for iPool = 1:3
    [omega_n,zeta,p] = damp(PoolModel(iPool).tf);
    % first pole is the integrator which has damping ratio -1
    assert(zeta(1) == -1, 'WARNING: something went wrong, first pole was not the integrator');

    % second and third poles are complex conjugates, just use the second
    % pole for zeta and omega)n
    PoolModel(iPool).zeta = zeta(2);
    PoolModel(iPool).omega_n = omega_n(2);
    
    % Literature survey Jacob sec. 3-3-1.
    PoolModel(iPool).phi_wave = omega_n(2) * sqrt(1-zeta(2)^2);
end

%% Add alpha from WIS properties to the pools
 PoolModel(1).alpha = Wis.area1;
 PoolModel(2).alpha = Wis.area2;
 PoolModel(3).alpha = Wis.area3;
 
