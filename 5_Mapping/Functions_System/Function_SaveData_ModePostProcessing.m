function [] = Function_SaveData_ModePostProcessing(alpha_set,alpha_Robot,alpha_base_meas,pos_set,Bx, By,Bz,Temp,...
    i,i_max,FOV)
% Function to handle the composition of the save-variable (struct) & safe
% it as .mat
%
% Parameter:
%   * alpha_set: Vector, Angles that should be set
%   * alpha_Robot: Vector, real Angles the robot can set
%   * alpha_base_meas: num, measured base angle sensor
%   * pos_set: Vector, FOV position set
%   * Bx: Vector, x-Values
%   * By: Vector, y-Values
%   * Bz: Vector, z-Values
%   * Temp: Vector, Temperature Sensor
%   * i: current iteration
%   * i_max: max iterations (to check if current iteration is the last one)
%   * FOV: struct, info about the FOV
% Return
%   no

global Save_Data;

B_meas = [Bx'; By';Bz';Temp']; % 8x4 Matrix 


% Matrix Sizes
% B_meas = 8x4 Matrix
% pos_FOVsystem = 8x3 Matrix
% B_FOVsystem = 8x3 Matrix
% alpha_Robot_real = 4x1 Matrix

Save_Data.time{end+1} = datetime('now');
Save_Data.alpha_set{end+1} = alpha_set;
Save_Data.alpha_Robot{end+1} = alpha_Robot;
Save_Data.alpha_Base_Meas{end+1} = alpha_base_meas;
Save_Data.position_set{end+1} = pos_set;

Save_Data.B_meas{end+1} = B_meas;
Save_Data.FOV_info = FOV;


% Save Save_Data
% After every iteration the file is saved (overrites old file) to be on the
% safe side. At the end the current time is added to the name 

folder = 'Save_Folder/';

if i == i_max % Last Position
    save_name = strrep(datestr(Save_Data.time{end}),':','_');
    save_name = strrep(save_name,' ','_');
else
    save_name = 'Save_Data.mat';
end

save_name = ['Measurement_' save_name];

save_name = [folder save_name];

save(save_name,'Save_Data');


end