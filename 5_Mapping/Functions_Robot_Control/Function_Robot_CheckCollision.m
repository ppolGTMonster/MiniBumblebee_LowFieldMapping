function [] = Function_Robot_CheckCollision(save_file,flag_check_corner)
% Function which moves the robot to the extrema positions, so the user can
% check if something is colliding
%
% Parameter:
%   * save_file: struct, file where the precalculated trajectory is stored
%   * flag_check_corner: true -> check the corners, false -> do nothing
% Return:
%   no

if flag_check_corner == true
    alpha_corner = save_file.alpha_corner;

for i=1:length(alpha_corner)

    %% 1) Lade Koordinate aus den berechneten FOV
    alpha_step=alpha_corner(:,i);
     disp(['Corner ' num2str(i) ' of ' num2str(length(alpha_corner))]);

    disp(['    ' 'Move Robot, set angles ' num2str(Function_ConvAlphaMatlab2Robot(alpha_step)) ' °']);

    [~] = Function_Robot_Move(alpha_step);
    pause(3);  % Warte etwas, damit sich die Position beruhigt hat

end

else
    return
end
end