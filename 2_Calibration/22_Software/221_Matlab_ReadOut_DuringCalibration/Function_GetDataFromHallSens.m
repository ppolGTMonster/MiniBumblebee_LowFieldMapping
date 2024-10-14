function [Bx,By,Bz,Temp,newData] = Function_GetDataFromHallSens ()
% Function to Reat Out the Hallsensor

global HallSens;
data = "";

flush(HallSens);    % Clear Serial Buffer
pause(1e-3);        % Wait is sensefull to have a more stable script

try
    data = readline(HallSens);
    data = erase(data,newline);
    disp(['   ' convertStringsToChars(data)]);

    [Bx,By,Bz,Temp,newData] = Function_Decode_Values(data);
catch
    Bx = zeros(8,1)*NaN;
    By = zeros(8,1)*NaN;
    Bz = zeros(8,1)*NaN;
    Temp = zeros(8,1)*NaN;
    newData = zeros(8,1)*NaN;
end
end