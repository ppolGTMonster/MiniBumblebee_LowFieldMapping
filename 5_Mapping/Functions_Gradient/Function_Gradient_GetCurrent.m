function [Volt_Meas, Ampere_Meas] = Function_Gradient_GetCurrent()
% Function to serial read out the Multimeter, which is used to measure the flowing current
%
% Parameter: none
% Return:
%  * Volt_Meas: measured Voltage
%  * Ampere_Meas: measured Current

global Multi;
global Multi_R_Shunt;


% 1) Get current Voltage
writeline(Multi,"val1?");
data = readline(Multi);

Volt_Meas = str2double(data); %Volt


% 2) Convert to Current. A shunt is used because of to high currents for the Multi
Ampere_Meas = Volt_Meas/Multi_R_Shunt; %Ampere

disp(['   Multi: Volt = ' num2str(Volt_Meas) 'V | Current = ' num2str(Ampere_Meas) 'A']);


end