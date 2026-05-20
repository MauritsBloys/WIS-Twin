%% Frequency analysis of the pools

% show comparison of filtered and unfiltered data on the fireflies
% show FFT of unfiltered data


%% Load experiment data
pool_data = readmatrix(sprintf("../data/%s", "20210709_filtering_off_on_offtake_on_off.csv"));

% extract and convert sensor data
water_levels = pool_data(:, [3,4,9,10,15,16,21]) .* Wis.a + Wis.b; % cm
water_levels_filtered = pool_data(:, [5,6,11,12,17,18,23]) .* Wis.a + Wis.b; % cm

X = water_levels(:,1)';
X_filtered = water_levels_filtered(:,1)';


% extract timing
timing = pool_data(:, 1)'; % ms 
    
%% show time plot
if show_fft_figures
    % Plot unfiltered
    figure();
    plot(timing/1000, X/100); % cm => m
    xlabel('time (s)')
    ylabel('level (m)')
    ylim([0 0.25])
    saveFigureEps("sensor_data_unfiltered_s1");
    title("Unfiltered sensor data");

    % Plot filtered
    figure();
    plot(timing/1000, X_filtered/100); % cm => m
    xlabel('time (s)')
    ylabel('level (m)')
    ylim([0 0.25])
    saveFigureEps("sensor_data_filtered_s1");
    title("Filtered sensor data");
end

%% FFT

Fs = 128;        % Sampling frequency
T = 1/128;       % Sampling period
L = size(X,2);    % Length of signal
t = timing;        % Time vector

Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

if show_fft_figures
    figure();
    f = Fs*(0:(L/2))/L;
    plot(f,P1)
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
    saveFigureEps("fft_s1");
    title('Single-Sided Amplitude Spectrum of X(t)')
end