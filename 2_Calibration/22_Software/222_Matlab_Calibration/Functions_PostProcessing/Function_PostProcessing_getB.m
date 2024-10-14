function [Br] = Function_PostProcessing_getB(Bs,ar,aTC1,aTC12,aTC2,OS,dT)
% Function for calculating the calibrated measured magnetic field from the raw value
% Parameters:
%   * Bs: vector, raw measurement value from the sensor
%   * ar: gain real
%   * aTC1: temp. coeff. 1
%   * aTC12: temp. coeff. 2
%   * aTC2: temp. coeff. 3
%   * OS: offset
%   * dT: temperature difference to the reference value
% 
% Return:
%   * Br: Vector, calibrated Output


    gain_error_T = aTC1.*dT + aTC12.*(dT.^2);
    Br = (ar+gain_error_T) .* (Bs+OS) + aTC2.*dT;
end