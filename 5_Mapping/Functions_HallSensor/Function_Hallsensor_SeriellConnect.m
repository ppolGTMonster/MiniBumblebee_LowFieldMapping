function [] = Function_Hallsensor_SeriellConnect(connected_ports)
% Function, to connect the Hall sensor with serialport
%
% Parameter:
%   * connected ports: String List of all current connected ports

global Hallsensor;
global SetFile_Comm;


COM = SetFile_Comm.HallSensor.COM;
Baud = SetFile_Comm.HallSensor.Baud;

% Check if the sensor was already connected (if so do not try to connect
% again)
if any(contains(connected_ports,COM))
    Hallsensor = serialport(COM,Baud);
    disp('...Hallsensor connected');
else
    disp('...Hallsensor was already connected');
end


end