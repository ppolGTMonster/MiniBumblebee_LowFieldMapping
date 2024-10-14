function [] = Function_Robot_SeriellConnect(connected_ports)
% Function, to connect the Robot Control with serialport
%
% Parameter:
%   * connected ports: String List of all current connected ports


global Robot;
global SetFile_Comm;

COM = SetFile_Comm.Robot.COM;
Baud = SetFile_Comm.Robot.Baud;


% Check if the robot was already connected (if so do not try to connect
% again)
if any(contains(connected_ports,COM))
    Robot = serialport(COM,Baud,"Timeout",10); %10 sec timeout should be more than enough
    disp('...Robot connected');
else
    disp('...Robot was already connected');
end

pause(10e-3);

% Wait till robot is booted (Arduino restarts when serial com is
% established)
while 1==1
    serialtext = readline(Robot);
    try
    if contains(serialtext,"Complete") == true
        disp('...Robot booted');
        break;
    end
    catch
    end
end
end