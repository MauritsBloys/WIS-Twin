function [pressure_sensor] = m2pressure(water_level, a, b)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    
    pressure_sensor = round((water_level * 100 - b) / a); 

end

