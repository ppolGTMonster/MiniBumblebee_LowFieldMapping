function [Bx,By,Bz,Temp] = Function_Hallsensor_getData()
% Function to read the Data from the Hall sensor via serial
%
% Prameter:
%   no
% Return:
%   * Bx: Vektor 8x1, x-Values
%   * By: -''-, y
%   * Bz: -''- z
%   * Temp: °C, Temperture of the sensor

global Hallsensor;
data = "";

flush(Hallsensor); % Empty Buffers (which may be filled during the movement)
pause(1e-3);

counter = 0;


try
    % Run through 2 iterations, because the first measurement can be
    % corrupted due to the movement (1 sensing takes around 7 sec)
    for i=1:2 

        data = readline(Hallsensor); % Can take up to 7 seconds

        if strlength(data)~=0 % some Data could be read
            if i>1 % 2. iteration -> Analyze Data
                data = erase(data,newline);
                [Bx,By,Bz,Temp,newData] =  help_function_DecodeValues(data);
            end
        end
    end


catch
    Bx = zeros(8,1)*NaN;
    By = zeros(8,1)*NaN;
    Bz = zeros(8,1)*NaN;
    Temp = zeros(8,1)*NaN;
    newData = zeros(8,1)*NaN;
end
end




%% Local Functions

function [Bx,By,Bz,Temp,newData] =  help_function_DecodeValues(COM_line)
% Deode one Line of Text from Serial into Values for every sensor

% get individual sting
H1 = extractBetween(COM_line, "H1;","|");
H2 = extractBetween(COM_line, "H2;","|");
H3 = extractBetween(COM_line, "H3;","|");
H4 = extractBetween(COM_line, "H4;","|");
H5 = extractBetween(COM_line, "H5;","|");
H6 = extractBetween(COM_line, "H6;","|");
H7 = extractBetween(COM_line, "H7;","|");
H8 = extractAfter(COM_line, "H8;");

% Measured B-Data (Here SENSOR COOR SYSTEM!)
Bx = zeros(8,1);
By = zeros(8,1);
Bz = zeros(8,1);
Temp = zeros(8,1);      % Temperature of the Sensor, important for temperature-dependant drift
newData = zeros(8,1);   % newData: Value important for debugging (see Hall sensor documentation itself)

%Convert String to Double
help_function_getValues(H1,1);
help_function_getValues(H2,2);
help_function_getValues(H3,3);
help_function_getValues(H4,4);
help_function_getValues(H5,5);
help_function_getValues(H6,6);
help_function_getValues(H7,7);
help_function_getValues(H8,8);


    function [] = help_function_getValues(text, index)
        Bx(index) = str2double(extractBetween(text,"Bx=","("));
        By(index) = str2double(extractBetween(text,"By=","("));
        Bz(index) = str2double(extractBetween(text,"Bz=","("));
        Temp(index) = str2double(extractBetween(text,"Temp=",";"));
        newData(index) = str2double(extractAfter(text,"ND="));
    end



end

