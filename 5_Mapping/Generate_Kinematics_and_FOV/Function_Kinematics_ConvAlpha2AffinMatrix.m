function [A_new] = Function_Kinematics_ConvAlpha2AffinMatrix(alpha_in_MT, alpha_in_Sensor)
% Function to calculate the homogeneous coordinate of the sensor out of the tracked angles
%
% Parameter:
%  * alpha_in_MT: Vector, motion tracked 4 angles of robot (base/shoulder/elbow/wrist) in Radiant
%  * alpha_in_Sensor: angle from Sensor in degree
% Return:
%  * A_new: 4x4-Matrix, homogenous coordinates of Senor


global SetFile_Mechanical;


%% 1) Prepare angles & constants
% angles from Motion Tracking
alpha1 = alpha_in_MT(2);
alpha2 = alpha_in_MT(3);
alpha3 = alpha_in_MT(4);
% angles from extra Sensor
alpha0 = deg2rad(alpha_in_Sensor);
% extra: Point5 (sensor front) needs to be rotated too due to mounting position of the sensor.
% by mechnical definition: Rotation around z-axis with -90°
alpha_p5 = deg2rad(-90);

% Convert to affine matrices
R_alpha0_sensor =  help_function_getAffin(help_function_RotY(alpha0));
R_alpha1 =  help_function_getAffin(help_function_RotZ(alpha1));
R_alpha2 =  help_function_getAffin(help_function_RotZ(alpha2));
R_alpha3 =  help_function_getAffin(help_function_RotZ(alpha3));
P_alpha_P5 = help_function_getAffin(help_function_RotZ(alpha_p5));

% Translation due to mechanical size of the robot
% Definition: Motion Tracking all angles 0° -> arm is vertical (defined by Aruco marker)
l0_t = SetFile_Mechanical.Robot.lB * [0;1;0];
l1_t = SetFile_Mechanical.Robot.lT * [0;1;0];
l2_t = SetFile_Mechanical.Robot.lS * [0;1;0];
l3_t = SetFile_Mechanical.Robot.lH * [0;1;0];

% Convert to affine matrices
l0 =        help_function_getAffin(l0_t);
l1 =        help_function_getAffin(l1_t);
l2 =        help_function_getAffin(l2_t);
l3 =        help_function_getAffin(l3_t);


%% 2) Combine everything together 

H_RCS_P1 = help_function_getAffin(SetFile_Mechanical.Env.t_ORB_To_RobotBase);
H_P1_P2 = R_alpha0_sensor*l0;
H_P2_P3 = R_alpha1*l1;
H_P3_P4 = R_alpha2*l2;
H_P4_P5r = R_alpha3*l3;
H_P5r_P5 = P_alpha_P5;

H_RCS_P5 = H_RCS_P1 * H_P1_P2 * H_P2_P3 * H_P3_P4 * H_P4_P5r*H_P5r_P5;
A_new = H_RCS_P5;

end


function [retval] = help_function_getAffin(in)


    if isvector(in)
        tvec = in; 
        tvec_affin = [eye(3) tvec; 0 0 0 1];
        retval = tvec_affin;
    else
        R_in = in;
        R = [R_in [0;0;0]; 0 0 0 1];
        retval = R;
    end


end

function [R] = help_function_RotY(alpha)
R = [cos(alpha)             ,0  ,sin(alpha);...
     0                      ,1  ,0         ;...
     -1*sin(alpha)          ,0  ,cos(alpha)];
end

function [R] = help_function_RotZ(alpha)
R = [cos(alpha)         ,-1*sin(alpha)  ,0;...
     sin(alpha)         ,cos(alpha)     ,0;... 
     0                  ,0              ,1];
end
