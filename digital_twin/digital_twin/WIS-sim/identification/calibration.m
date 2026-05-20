%% calibration.m

% Calibrate the presure sensors (needed to convert pressure to water level)

%% calibration data for lowest value
CalibrationData = readmatrix("../data/20210202_calib_5cm.csv");

% figure(1);
% plot(CalibrationData(:, [3,4,7,8,11,12, 15]));
% title("calibration data raw (5cm)");

lowValue = mean(CalibrationData(:, [3,4,7,8,11,12, 15]));
low = 5;

%% calibration data for highest value
CalibrationData = readmatrix("../data/20210202_calib_31cm.csv");

% figure(2);
% plot(CalibrationData(:, [3,4,7,8,11,12, 15]));
% title("calibration data raw (31cm)");

highValue = mean(CalibrationData(:, [3,4,7,8,11,12, 15]));
high = 31;

%% fit linear data for all sensors

% TODO: can this be vectorized?
[~, nSensors] = size(lowValue);
a = zeros(1,nSensors);
b = zeros(1,nSensors);
for i = 1:nSensors
    coefficients = polyfit([lowValue(i), highValue(i)], [low, high], 1);
    a(i) = coefficients (1);
    b(i) = coefficients (2);
end

Wis.a = a;
Wis.b = b;

