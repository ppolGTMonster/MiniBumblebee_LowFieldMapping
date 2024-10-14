function [B] = Function_PostProcessing_GetBField(LogFile,mode)
% Function to calculate the magnetic field of the calibration coil from the measured current
% Parameters
% * LogFile: struct, file with stored variables
% * mode: 
%    * 'Lin': measurement of the linearity was carried out (sign of current can change at any time)
%    * 'Temp': measurement at constant field (sign stable) with increasing temperature
% 
% Returned value:
% * B: applied magnetic field in the calibration coil

global CalCoil_Conversion;

% Calculate Magnetic Field of Calibration Coil using the Current Measurement
Current = cell2mat(LogFile.Multi.Current);
B = CalCoil_Conversion * Current;


% Check the sign of the B-Field
% Reason:
% In the system presented, the measurement of the current (which is used to operate the calibration cylinder) can only be carried out in one direction
% and is therefore independent of the sign of the current flow in the coil itself
B_sign = 1;

if contains(mode, 'Temp') % Temperature measurement. The sign of Breal (which is B Coil) is stable over all measuring points
    if strcmp(LogFile.SetUp.CalibratedDirection,'x')== 1
        Hall_Data = LogFile.HallSens.Bx;
    elseif strcmp(LogFile.SetUp.CalibratedDirection,'y')== 1
        Hall_Data = LogFile.HallSens.By;
    elseif strcmp(LogFile.SetUp.CalibratedDirection,'z')== 1
        Hall_Data = LogFile.HallSens.Bz;
    end

elseif contains(mode, 'Lin') % Linearity Measurement. Depending on the set-up, the sign changes.
    Hall_Data = LogFile.HallSens.B;
else
    Hall_Data=[];
end

B_sign = ones(1,length(Hall_Data));

for i_2 = 1:length(Hall_Data)
    B_sign(i_2) = sign(sum(Hall_Data{i_2}));
end

B = abs(B).*B_sign;


end