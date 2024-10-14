function [Bs,Br,T] = function_optimizer_maskData(B_in,LogFile,Offset,i_sensor)
% Function to mask the data
% Goal: 
%   * Delete all NaN data that could cause problems in the calibration run
%   * determine the magnetic field of the calibration coil
%   * Subtract the already determined offset from the data
% Parameter:
%   * B_in: Matrix, magnetic field of the sensors to be evaluated
%   * Logfile: Struct,  log file of the calibration measurement
%   * Offset: Number, determined offset of the sensor in the corresponding direction x/y/z
%   * i_sensor: Number, indicates which of the active sensors is currently being calculated
% Returned:
%   * Bs: Matrix, masked magnetic field data sensor
%   * Bs: Matrix, masked magnetic field data coil ("real")
%   * T: Matrix, masked temperature data sensor

B = cell2mat(B_in);
T = cell2mat(LogFile.HallSens.Temp);
T = T(i_sensor,:);

%% Step 1) Offset Compensation
Bs = B(i_sensor,:) + Offset; % Offset Comp


%% Step 2) Calculate Magnetic Field Calibration Coil
try
    % Linearity Measurement
    Br = Function_PostProcessing_GetBField(LogFile,'Lin');
catch
    %Temperature Measurement
    Br = Function_PostProcessing_GetBField(LogFile,'Temp');
end

%% Step 3) Mask Data

mask = (isnan(Br) == true).* (isnan(Bs) == true);
Bs = Bs(mask==0);
Br = Br(mask==0);
T = T(mask==0);


end

