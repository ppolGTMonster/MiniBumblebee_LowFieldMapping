function cost = Function_Kinematics_CostFunction_HomogTrafo(alpha_in,tvec_goal)
% Cost Function of fmincon optimizer
% Calculates the cost as difference between the target position and the achieved position by kinematics
%
% Parameter:
%  * alpha_in: Vector, Angles of Robot
%  * tvec_goal: Vector, Target Position
% Return: 
%  * cost: Distance in mm


global SetFile_Mechanical;



[t_P5_RobS,~] = Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha_in);

% Distance between set point and goal
% dist_points = norm(tvec_goal - t_P5_RobS);

dist_points =  0.5*(tvec_goal-t_P5_RobS)'*(tvec_goal-t_P5_RobS); %in Meter

cost = dist_points*1000; % Convert to Milimeter




end


