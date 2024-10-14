function [SetFile_Mechanical,SetFile_Kinematics,SetFile_Comm,SetFile_System] = get_SetFile()

%% 1) Mechanical Constants
SetFile_Mechanical.Robot.lB = 0.0715; % distance from Base to Shoulder. 
SetFile_Mechanical.Robot.lT = 0.125; % distance from Shoulder to Elbow
SetFile_Mechanical.Robot.lS = 0.125; % distance from Elbow to Wrist
SetFile_Mechanical.Robot.lH_part1_Robot = 60.95;
SetFile_Mechanical.Robot.lH_part2_Holder = 220;
SetFile_Mechanical.Robot.lH = (SetFile_Mechanical.Robot.lH_part1_Robot+ SetFile_Mechanical.Robot.lH_part2_Holder)*1e-3; % distance from Wrist to Sensor 

% Distances Hallsensor, Coor System analog to openCV (Base CS)
deltaX = 20;
deltaY = 10;
deltaZ = 10;

SetFile_Mechanical.HallSensor.deltaX = deltaX * 1e-3;
SetFile_Mechanical.HallSensor.deltaY = deltaY * 1e-3;
SetFile_Mechanical.HallSensor.deltaZ = deltaZ * 1e-3;

SetFile_Mechanical.HallSensor.t_P5_To_H1 = [10        12.75         5  ]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H2 = [0;        0;        -deltaZ]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H3 = [deltaX;   0;            0  ]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H4 = [deltaX;   0;        -deltaZ]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H5 = [0;       -deltaY;         0]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H6 = [0;       -deltaY;   -deltaZ]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H7 = [deltaX;  -deltaY;         0]*1e-3;
SetFile_Mechanical.HallSensor.t_H1_To_H8 = [deltaX;  -deltaY;   -deltaZ]*1e-3;
SetFile_Mechanical.HallSensor.MountingPosition = "x_s=-x|y_s=z|z_s=y";  % How are the axis of the Hallsensor aligned with the axis of the Base CoorSys


% Environment (SetUp of robot/reference board)
SetFile_Mechanical.Env.Dist_Corner_robot = 90;% mm
SetFile_Mechanical.Env.t_ORB_To_MagnetFOV = [-180;78;200] *1e-3;
SetFile_Mechanical.Env.t_ORB_To_RobotBase = [340-SetFile_Mechanical.Env.Dist_Corner_robot;-96;200] *1e-3;


%% 2) Kinematics
lim_offset = [2 -2];            %Degrees,  Backup, in order to not drive into the limits
SetFile_Kinematics.tol_horizontality = 10; %Degree, maximum allowed tolerance (+-) of the sensor horizontality, used as contraint in kinematik
SetFile_Kinematics.alpha_offset_RobotControl = 90;    % Degree, Offset: Robot sets values in the range 0-180°. 
SetFile_Kinematics.alpha_offset_RobotControl_optimzer = 90;    %Degree, Offset: Robot sets values in the range 0-180°. 
SetFile_Kinematics.alpha_1_limit= [0,180] + lim_offset;  %Degree, Limit of possible angles for BASE
SetFile_Kinematics.alpha_2_limit= [15,165] + lim_offset; %Degree, Limit of possible angles for SHOULDER
SetFile_Kinematics.alpha_3_limit = [0,180] + lim_offset; % Degree, Limit of possible angles for ELLBOW
SetFile_Kinematics.alpha_4_limit = [0,180] + lim_offset; % Degree, Limit of possible angles for WRIST

% Calibration to adjust tolerances of the mechanical robot setup (e.g. if
% its not perpendicular mounted)
shoulder_off_tolerance = 0;     %°, used to possibliy preload the Shoulder-motors if the weight is to heavy
ellbow_off_tolerance = 0;       %°, same for ellbow
SetFile_Kinematics.alpha_base_offset = 90-85;           %°, Offset = ideal-real. Here = 90°-85° for the base
SetFile_Kinematics.alpha_shoulder_offset = 90-85 + shoulder_off_tolerance;   %°, -''- for shoulder
SetFile_Kinematics.alpha_ellbow_offset = 90-89 + ellbow_off_tolerance;      %°, -''- for ellbow
SetFile_Kinematics.alpha_wrist_offset = 90-89;	%°, -''- for Wrist


%% 3) Communication Setup
% HALL SENSOR
SetFile_Comm.HallSensor.COM = "COM7"; % COM-Port for serial Communication (check Device Manager)
SetFile_Comm.HallSensor.Baud = 115200; % Baud rate of COM-Port
SetFile_Comm.HallSensor.timeout =10;    %sec, 10s are enough, Data should be sent every ~7 seconds (tested to be reliable)

% ROBOT
SetFile_Comm.Robot.COM = "COM11";   % COM-Port for serial Communication 
SetFile_Comm.Robot.Baud = 115200;   % Baud rate


%% 3) Setup for Camera Based Measurement (Motion Tracking done in Post Processing)
SetFile_System.WaitOnPos_BSens =16; 
%sec, Time to wait on one position to measure the magnetic field.
% The measurement is normally done faster, but wait a constant time to make
% the sync with the video file in post processing easier.
SetFile_System.WaitOnPos_Move = 4;   %sec, tbd
SetFile_System.Camera_RecTime_Limit = true; 
% true -> Cam has limited recording time (like most cams). false -> cam can record unlimited (professional stuff)
SetFile_System.Max_Recording_Length = 28*60; %sec, max duration your cam can record a single file. Here: Sony HX60 with max length 28mins 



end