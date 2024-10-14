function [alpha_Robot,alpha_base_meas] = Function_Robot_Move(alpha_in)
% Function to move the robot to a new position
%
% Parameter:
%   * alpha_in: Vektor, Angles from kinematics in radiants
% Return:
%   * alpha_Robot: Vektor, Angles for Robot Control in degree
%   * alpha_base_meas: Angle, which was measured with the extra sensor

global SetFile_Kinematics;

alpha_Robot = Function_ConvAlphaMatlab2Robot(alpha_in); % Convert Angles from Kinematic to Robot
text = help_function_ConvAlpha2Text(alpha_Robot);       % Convert Number-Angles to String
alpha_base_meas = help_function_Send2Robot(text);       % Send new Angles & Get Base Angle

end





%% Local Functions

function text = help_function_ConvAlpha2Text(alpha_in)
% Convert numerical Angles to text

global SetFile_Kinematics;

alpha_round = round(alpha_in); % round values, Robot can set only integers

% Check on Limits
alpha_round(1) = help_function_checkLimits(alpha_round(1),SetFile_Kinematics.alpha_1_limit); %Base
alpha_round(2) = help_function_checkLimits(alpha_round(2),SetFile_Kinematics.alpha_2_limit); %Shoulder
alpha_round(3) = help_function_checkLimits(alpha_round(3),SetFile_Kinematics.alpha_3_limit); % Ellbow
alpha_round(4) = help_function_checkLimits(alpha_round(4),SetFile_Kinematics.alpha_4_limit); % Wrist

alpha1 = help_function_getChars(alpha_round(1)); % Base
alpha2 = help_function_getChars(alpha_round(2)); % Shoulder
alpha3 = help_function_getChars(alpha_round(3)); % Elbow
alpha4 = help_function_getChars(alpha_round(4)); % Wrist

text = alpha1 +"," +alpha2 +"," +alpha3 +"," +alpha4;

end

function retval =  help_function_checkLimits(val,limit)
% checks if the wanted new angle is inside the limits

    lb = limit(1);
    ub = limit(2);
    if val < lb
        retval =  lb;
        return;
    elseif val > ub
        retval =  ub;
        return;    
    else
        retval = val;
        return;
    end
end

function alpha_base = help_function_Send2Robot(text)
% Sends the text-Line via serial to the Robot & waits on the returned Base-Angle

global Robot;

flush(Robot); % Empty the robot buffers - always a good idea in serial usage
pause(0.001); % pause, an even better idea with this crappy serial objects :D

writeline(Robot,text); % Send Data

data = readline(Robot); % Can take up to 1.5 sec
alpha_base = str2double(extractBetween(data,"Base = ","|")); % Read Base Angle
disp(data);
flush(Robot);

end

function str_out = help_function_getChars(num_in)
% Add enough zeros infront of the wanted angle to get 4 digits 
str_out ="";
str_start="";

if abs(num_in)<10       % 1 digit angle
    str_start="000";
elseif abs(num_in)<100  % 2 digit angle
    str_start="00";
else                    % 3 digit angle
    str_start="0";
end

str_out = str_start+ num2str(num_in);
end