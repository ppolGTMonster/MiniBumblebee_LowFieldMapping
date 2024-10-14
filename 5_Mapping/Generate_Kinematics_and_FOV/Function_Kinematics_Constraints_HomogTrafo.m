function c_result = Function_Kinematics_Constraints_HomogTrafo(alpha)
% Function to calculate the inequality constraints for the fmincon-optimizer
% The boundary condition is that the last element of the robot (where sensor is attached to) is positioned as horizontally as possible.
%
% Parameter:
%  * alpha: Vector, Angles of the robot
% Return:
%  * c_result: Constraint



global SetFile_Kinematics;

beta_tol = deg2rad(SetFile_Kinematics.tol_horizontality); %Tolerace of max allowed horizontality 



[t_ax4, t_ax3] = Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha);


% Calculation of horizontality of last element as angle between vector and horizontal plane 
dir_l4 = t_ax4-t_ax3;
n_plane = [0;0;1];
asin_val = (dot(n_plane,dir_l4))/(norm(dir_l4)*norm(n_plane));

angle = asin(asin_val);

% Inequality: abs(angle) < tolerance -> c = abs(angle)- tolerance < 0
c_result = abs(angle)-beta_tol;





end