function  [] = Function_Gradient_SwitchOnCurrent()
% Function, which controls the Relais, which turns on the current flow through the gradient
%
% Parameter: none
% Return: none

global Relais_Control;
global SetFile_System


time_on = SetFile_System.WaitOnPos_BSens - 2*1; % Sec
time_on = time_on *1e6;     %Microseconds

flush(Relais_Control);
pause(0.001);

writeline(Relais_Control,num2str(time_on));
flush(Relais_Control);
pause(1);

end