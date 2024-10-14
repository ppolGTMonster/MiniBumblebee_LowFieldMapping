function alpha_Matlab = Function_Robot_ConvAlphaRobot2Matlab(alpha_in)
% Function to convert the angels used in the Robot Control to the ones
% used in the kinematics (and therefore Matlab)
% Basecally the inversion of Function_ConvAlphaMatlab2Robot()
%
% Parameters:
%   * alpha_in: Vector, Angles from robot controls in degree
% Return:
%   * alpha_Matlab: Vektor, Angles for Kinematics in radiants


global SetFile_Kinematics;

% Avoid all other offsets (due to nonperfection) here -> ideal system

alpha_offset = SetFile_Kinematics.alpha_offset_RobotControl;
alpha_base =     -1*alpha_in(1);
alpha_shoulder = alpha_in(2) - alpha_offset;
alpha_ellbow =   alpha_in(3) - alpha_offset;
alpha_wrist =    alpha_in(4) - alpha_offset;


alpha_Matlab = [alpha_base alpha_shoulder alpha_ellbow alpha_wrist];
alpha_Matlab = deg2rad(alpha_Matlab);


end