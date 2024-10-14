function [B_error] = function_PostProcessing_getBerror(B)
% Function for calculating the error caused by the calibration itself (mainly due to the tolerance of the multimeter and the shunt).
%
% !!!For your measurement, the tolerances must be adjusted - depending on which multimeter you use and which shunt!!!
% 
% Used Multimeter: GW Instek GDM-8255A
% Used Shunt: Isabellenhütte Heusler RUG-Z-R020-0.1-TK1 20mΩ
%
% Parameters:
%   * B: Vector, applied measured magnetic field in the calibration coil
% Return
%   * B_error: Vector, calculated error of the magnetic field


global CalCoil_Conversion;

%% Step 1) Define Tolerances Multimeter

multi.Tolerance = 0.12/100;         % Overall Tolerance 
multi.Range = [100e-3,1];           % The set range chagnes depending on measured values and therefore also the resolution & offset
multi.Resolution = [1e-6, 10e-6];   % Resolution Limit at different Ranges
multi.Offset = [8,5];               % Offset Errorat different Ranges

%% Step 2) Define Tolerances Shunt/Resistor
shunt.Ohm = 20e-3;                  %Ohm, nominal resistance
shunt.Tolerance = 0.1/100;          % Maunfactoring Tolerance 

%% Step 3) Start Calculation
% Calculate Votlage at Multimeter
I = B/CalCoil_Conversion;  
U = abs(I*shunt.Ohm);

% Calcualte B Error
B_error = zeros(size(U));

for i=1:length(U)
    if U(i)<100e-3
        index = 1;
    else
        index = 2;
    end

    Multi_Res = multi.Resolution(index);
    Multi_OS  = multi.Offset(index) * Multi_Res;

    Multi_GainError = multi.Tolerance  * U(i);
    Multi_ResError = U(i)/10000;                    % Slow Modus, Defined in the multi data sheet

    Multi_Error = Multi_GainError + Multi_OS + Multi_ResError;

    Shunt_Error =  shunt.Ohm*shunt.Tolerance;

    Bmin = CalCoil_Conversion*(U(i)+Multi_Error)/(shunt.Ohm-Shunt_Error);
    Bmax = CalCoil_Conversion*(U(i)-Multi_Error)/(shunt.Ohm+Shunt_Error);
    B_error(i) = abs(Bmax-Bmin);

end


end