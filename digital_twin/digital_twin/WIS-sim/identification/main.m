%% main.m

% Runs all scripts needed to identify the lab setup from data and saves
% the result as a Matlab workspace file.

clear all;

%% Figure settings - set to false to skip figures for that section
show_time_plot          = false;  % 1 figure: water levels during identification
show_fft_figures        = false;  % 3 figures: unfiltered/filtered sensor data + FFT
show_gate_figures       = false;  % ~19 figures: water levels, flows and model fit per gate measurement
show_identification_figures = false;  % 9 figures: createIddata (pool 3), validation per pool + bode plot

%% Add common functions to path
addpath ../functions/

%% set global properties
wis_properties;

%% perform calibration
calibration;

%% visual validation of calibration
if show_time_plot
    time_plot;
end

%% frequency analysis
wis_fft;

%% load data sets
load_pool_data;
%load_pool_data20210302;


%% identify flow over the gates
gate_identification;

%% identify transfer function for the pools
identification;

%% save results for later use
save("identification.mat", 'Wis', 'PoolModel')

%% print c code for sensor calibration
sensor_parameters_c

