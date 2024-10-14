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

% Gradient related
global Multi_R_Shunt;
global Multi;
global Relais_Control;

%% 1) SetUp Variables
flag_check_corner = true; % If true -> First the corners of FOV are checked (check for collision)

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

% Gradient related
Relais_COM_Port = "COM9";
Relais_SRL_Baud = 9600;
% SetUp GW Instek 8255A-Multimeter
Multi_COM_Port = "COM10";
Multi_SRL_Baud = 115200;
Multi_R_Shunt = 0.02; % Ohm, needs to be measured exactly

if sum(contains(connected_ports,Multi_COM_Port))>=1
    Multi = serialport(Multi_COM_Port,Multi_SRL_Baud);
    disp('...Multimeter connected');
end
configureTerminator(Multi,"LF");

if sum(contains(connected_ports,Relais_COM_Port))>=1
    Relais_Control = serialport(Relais_COM_Port,Relais_SRL_Baud);
    disp('...Relais connected');
end
pause(5);




%% 4) Start Save_File
Save_Data.time = {};        % Time (can help with sync with video)
Save_Data.alpha_set = {};   % robot angle, which should be set (used as backup)
Save_Data.alpha_Robot = {}; % robot angle, which are then set really 
Save_Data.alpha_Base_Meas={}; % Base angle, measured with extra sensor
Save_Data.position_set= {}; % position of the FOV, which should be reached
Save_Data.B_meas = {};      % measured B-field (raw data)
Save_Data.Multi.Current = [];     % current that has flown through the gradient
Save_Data.Multi.Voltage =[];      % measured voltage (important for error calculation)
Save_Data.Multi.Shunt = Multi_R_Shunt;

%% 5) Load Trajectory of FOV, which was generated prior (using Generate_Trajectory_FOV_Coordinates.m)
load('FOV_Trajectory_alphas.mat');
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
    
    % 7.1) Cooling phase
    % The gradient is cooled by convection and can quickly become warm (since the power is on permanently)
    % After 6 measurements, allow to cool and restart the camera.

    if mod(i,6) == 0
        beep; pause (0.5); beep; pause (0.5); beep;
        input("......................Cool down System & Restart Camera... ");
    end

    % 7.2) Get next coordinates from precalculated FOV
    alpha_step=alpha_save(:,i);

    % 7.3) Start Timer, so that one measured poits takes the same amout of time
    Timer = Function_TimerForSyncing('Start','Move',0);

    % 7.4) Move the robot to new position
    disp(['    ' 'Move Robot, set angles ' num2str(Function_ConvAlphaMatlab2Robot(alpha_step)) ' °']);
    disp(['    ' '  Pos (Robot Coor Sys) ' num2str(pos_save(:,i)'*1000) ' mm']);
    
    [alpha_Robot,alpha_base_meas] = Function_Robot_Move(alpha_step);
    pause(0.5);  % Wait a bit, so the shaking is over

    % 7.5) Stop Timer
    [~] = Function_TimerForSyncing('Stopp','Move',Timer);

    % 7.6) Start new Timer for B-Field Measurement
    Timer = Function_TimerForSyncing('Start','Meas',0);

    % 7.7) Measure the Magnetic Field
    disp(['    ' 'Measuring B-Field: Start']);
    Function_Gradient_SwitchOnCurrent();
    [Volt_temp1, Ampere_temp1] = Function_Gradient_GetCurrent(); % Current shortly before measurement
    [Bx,By,Bz,Temp] = Function_Hallsensor_getData();
    [Volt_temp2, Ampere_temp2] = Function_Gradient_GetCurrent(); % Current shortly after the measurement
    disp(['    ' 'Measuring B-Field: Stop']);
   
    % 7.8) Stop the running timer and wait the remaining duration
    [~] = Function_TimerForSyncing('Stopp','Meas',Timer);

    % 7.9) Save the current iteration
    disp(['    ' 'Save Data']);
    Save_Data.Multi.Voltage(end+1,:) = [Volt_temp1 Volt_temp2];
    Save_Data.Multi.Current(end+1,:) = [Ampere_temp1 Ampere_temp2];

    Function_SaveData_ModePostProcessing(alpha_step,alpha_Robot,alpha_base_meas,pos_save(:,i),...
        Bx, By,Bz,Temp,i,length(alpha_save),save_file.FOV);  
    
end

%% 8) Finish the Measurement
beep; pause (0.5); beep; pause (0.5); beep; % Make the system beep to get attention of user
disp(['Measurement finished. Needed time: ' num2str(toc(Timer_Script)) ' Seconds']);
