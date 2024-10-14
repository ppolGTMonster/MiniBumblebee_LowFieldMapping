function [ConvMat] = Function_HallSensor_getConvertMatrix_MountingPos()
% Function to build the coordinate transformation matrix based on the the SetFile
%
% Paramaters:
%  * none
% Return:
%  * ConvMat: 3x3-Matrix, with the conversion matrix

global SetFile_Mechanical;

% Extract the mountign position out of the set file
mount_x = extractBetween(SetFile_Mechanical.HallSensor.MountingPosition,'x_s=','|');
mount_y = extractBetween(SetFile_Mechanical.HallSensor.MountingPosition,'y_s=','|');
mount_z = extractAfter(SetFile_Mechanical.HallSensor.MountingPosition,'z_s=');

% Calculate Conversion Matrix
x = getRow(mount_x);
y = getRow(mount_y);
z = getRow(mount_z);

ConvMat = [x;y;z];

end

%% Local Functions

function [ret] = getRow(txt)
% convert string-command into real unit-vector
if contains(txt,'-')
    setsign = -1;
else
    setsign = 1;
end

if contains(txt,'x')
    ret = [1 0 0];
elseif contains(txt,'y')
    ret = [0 1 0];
elseif contains(txt,'z')
    ret = [0 0 1];
end

ret = ret * setsign;
end