function [Volt_Meas, Ampere_Meas] = Function_GetCurrent()
% Readout Multimeter to get current flow

global Multi;
global Multi_R_Shunt;
global CalCoil_Sens;


% 1) Get current Voltage from Multimeter
writeline(Multi,"val1?");
data = readline(Multi);
Volt_Meas = str2double(data); %Volt

% 2) Convert to Current. A shunt is used because of to high currents for the Multi
Ampere_Meas = Volt_Meas/Multi_R_Shunt; %Ampere

disp(['   Multi: Volt = ' num2str(Volt_Meas) 'V | Current = ' num2str(Ampere_Meas) 'A']);

end