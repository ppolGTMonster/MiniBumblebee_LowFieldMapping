%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script that is running the measurement, controls the robot and
%       readsout the base angle sensor
%       Evaluation of the Motion Tracking together with the Hallsensor is done in Post Processing    
%       after the Measurement
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clc;
pause(1);

Timer_Script = tic;

%% 0) Global Variables
% Predefined Settings 
global SetFile_Mechanical;
global SetFile_Kinematics;
global SetFile_Comm;
global Hallsensor_CalFile;
global SetFile_System;

% Communication Objects
global Robot;
global Hallsensor;
global openCV;
global MeasuredData;
global Save_Data;

%% 1) SetUp Variables
flag_check_corner = true; % If true -> First the corners of FOV are checked (check for collision)
camera_cycle = 0; % In case your camera is limited regarding recording time, this value helps to stop the script to restart the camera (with new storage) 

%% 2) Load Settings
[SetFile_Mechanical,SetFile_Kinematics,SetFile_Comm,SetFile_System] = get_SetFile();
% all constants, physical properties etc can be adjusted by the user in
% this function (its more like a set-file)

%% 3) Start Connections
disp('Communication: Setting Up')
connected_ports = serialportlist('available');

% Robot
Function_Robot_SeriellConnect(connected_ports);
% HallSensor
Function_Hallsensor_SeriellConnect(connected_ports);
pause(5);

%% 4) Start Save_File
Save_Data.time = {};        % Time (can help with sync with video)
Save_Data.alpha_set = {};   % robot angle, which should be set (used as backup)
Save_Data.alpha_Robot = {}; % robot angle, which are then set really 
Save_Data.alpha_Base_Meas={}; % Base angle, measured with extra sensor
Save_Data.position_set= {}; % position of the FOV, which should be reached
Save_Data.B_meas = {};      % measured B-field (raw data)
%Save_Data.B_correct = {};   % B-Werte nach Einsatz der Kalibrierung


%% 5) Load Trajectory of FOV, which was generated prior (using Generate_Trajectory_FOV_Coordinates.m)
load('Save_Folder\FOV_Trajectory_alphas.mat');
alpha_save = save_file.alpha;
pos_save = save_file.pos;

%% 6) Check possible collisiion of corner points (in case of square fov)
disp('-COLLISION CHECK START-');
Function_Robot_CheckCollision(save_file,flag_check_corner);
disp('    ...finished')
input(' -- Press >>Enter<< to Start the MEASUREMENT -- ');

%% 7) START MEASUREMENT
for i=1:length(alpha_save)
    disp(['Step ' num2str(i) ' of ' num2str(length(alpha_save))]);
    
    % 7.1) After half measurement you could install the springs to help the
    % shoulder motor handle the high weight
    if i== floor(length(alpha_save)/2)
        beep; pause (0.5); beep; pause (0.5); beep; % Sound Alarm
        input("...................... Install new Springs... ");
    end

    % 7.2) Check if Camera is still running
    camera_cycle = Function_System_CheckCamera(i,camera_cycle); 

    % 7.3) Get next coordinates from precalculated FOV
    alpha_step=alpha_save(:,i);

    % 7.4) Start Timer, so that one measured poits takes the same amout of time
    Timer = Function_TimerForSyncing('Start','Move',0);

    % 7.5) Move the robot to new position
    disp(['    ' 'Move Robot, set angles ' num2str(Function_ConvAlphaMatlab2Robot(alpha_step)) ' °']);
    disp(['    ' '  Pos (Robot Coor Sys) ' num2str(pos_save(:,i)'*1000) ' mm']);
    
    [alpha_Robot,alpha_base_meas] = Function_Robot_Move(alpha_step);
    pause(0.5);  % Wait a bit, so the shaking is over

    % 7.6) Stop Timer
    [~] = Function_TimerForSyncing('Stopp','Move',Timer);

    % 7.7) Start new Timer for B-Field Measurement
    Timer = Function_TimerForSyncing('Start','Meas',0);

    % 7.8) Measure the Magnetic Field
    disp(['    ' 'Measuring B-Field: Start']);
    [Bx,By,Bz,Temp] = Function_Hallsensor_getData();
    disp(['    ' 'Measuring B-Field: Stop']);
    
    % 7.9) Stop the running timer and wait the remaining duration
    [~] = Function_TimerForSyncing('Stopp','Meas',Timer);

    % 7.10) Save the current iteration
    disp(['    ' 'Save Data']);
    Function_SaveData_ModePostProcessing(alpha_step,alpha_Robot,alpha_base_meas,pos_save(:,i),Bx, By,Bz,Temp,...
        i,length(alpha_save),save_file.FOV);

    
end

%% 8) Finish the Measurement
beep; pause (0.5); beep; pause (0.5); beep; % Make the system beep to get attention of user
disp(['Measurement finished. Needed time: ' num2str(toc(Timer_Script)) ' Seconds']);
