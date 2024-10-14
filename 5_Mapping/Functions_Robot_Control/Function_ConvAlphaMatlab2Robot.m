function alpha_Robot = Function_ConvAlphaMatlab2Robot(alpha_in)
% Function to convert the calculated angles (got from the
% kinematic/mathematical desicription) to angles, that can be set on the
% robot (used in the drive of the servo motors)
%
% Parameters:
%   * alpha_in: Vektor, Angles from kinematics in radiants
% Return:
%   * alpha_Robot: Vektor, Angles for Robot Control in degree

global SetFile_Kinematics;

alpha_offset = SetFile_Kinematics.alpha_offset_RobotControl;
alpha_in = rad2deg(alpha_in);

alpha_base =     -1*alpha_in(1);
alpha_shoulder = alpha_offset+alpha_in(2);
alpha_ellbow =   alpha_offset+alpha_in(3);
alpha_wrist =    alpha_offset+alpha_in(4);

% Use Calibration if necessairy
alpha_base =  alpha_base        - SetFile_Kinematics.alpha_base_offset;
alpha_shoulder = alpha_shoulder - SetFile_Kinematics.alpha_shoulder_offset;
alpha_ellbow =  alpha_ellbow    - SetFile_Kinematics.alpha_ellbow_offset;
alpha_wrist =   alpha_wrist     - SetFile_Kinematics.alpha_wrist_offset;

alpha_Robot= [alpha_base,alpha_shoulder,alpha_ellbow,alpha_wrist];

end