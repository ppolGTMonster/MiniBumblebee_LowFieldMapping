function [camera_cycle] = Function_System_CheckCamera(i,camera_cycle)
% Function to check after which iteration the camera needs to be restarted
% to take care on the recording limit
%
% Parameter
%   * i: current iteration of measurement for loop
%   * camera_cycle: current camera cycle
% Return
%   * camera_cycle: new value of camera cycle (+1, if a newstart was needed)

global SetFile_System;

if SetFile_System.Camera_RecTime_Limit == true
    % get max iteration
    index_record_limit = floor(SetFile_System.Max_Recording_Length/(SetFile_System.WaitOnPos_BSens + SetFile_System.WaitOnPos_Move));

    i_to_check = i-camera_cycle*index_record_limit;
    if i_to_check>index_record_limit
        beep; pause (0.5); beep; pause (0.5); beep; % Make the system beep to get attention of user
       
        input("------- CAMERA reached the recording limit. Reset the camera. Then press 'ENTER' to continue");
        pause(0.1);
        camera_cycle = camera_cycle+1;
    else
        return;
    end
else
    return;
end



end
