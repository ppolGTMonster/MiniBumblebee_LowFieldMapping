%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script that calculates the calibration coefficients using the measurement data from the measured values themselves              %
%       Goal: determine calibration coeff of all 8 sensors                                                                              %
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
addpath('Functions_PostProcessing');

% Global Variables
global active_HallSens;
global show_plots;
global Temp_Ref;
global T_limit;

global Lin_error;
global Lin_error_xy;
global Lin_error_z;
global Temp_Coeff_gain_error;
global Temp_Coeff_gain_error2;
global TC_OS_error;
global OS_Earth;
global x0;
global ub;
global lb;
global ub_xy;
global lb_xy;
global ub_z;
global lb_z;
global opt_weight_MeasCal;
global opt_weight_TempCal;
global check_clear_OverTemp;



%% Limits of Optimizsation from the data sheet
% maximum linearity error according to data sheet
Lin_error_xy = 2.5/100;                             % in x/y axis
Lin_error_z = 4.9/100;                              % in z axis

%maximum temperature coefficient of linearity according to data sheet
Temp_Coeff_gain_error = 0.24/100;                   %1/K, linear part
Temp_Coeff_gain_error2 = Temp_Coeff_gain_error/2;   % square part
%maximum temperature coefficient of the offset
TC_OS_error = 50*0.07;                              %mT/°C, from Datasheet A1342 Application Info
% Offset during linearity measurements (caused by earth's magnetic field + stray field of devices in the laboratory)
OS_Earth = 50 * 1.5 *1e-3; % mT, Earth Magnetic Field + stray field

% Weighting of the measured values between the linearity measurement and the temperature sweep measurement:
opt_weight_MeasCal = 2; % In the Lin measurement, the sensors were individually perfectly in the center of the cylinder (therefore double weighted). 
opt_weight_TempCal = 1; % This was not so good in the temperature measurement (therefore individually weighted).


% Variables that are optimized: [Gain TC1(Gain) TC12(Gain) TC2(OS)]x, ... ,[Gain TC1(Gain) TC12(Gain) TC2(OS)]z OS_Env 
% Where OS_Env stands for earth's magnetic field + stray field

x0 = [1.01,0.001,0,40e-3];                                                              % Start Value
ub_xy = [1 + Lin_error_xy, Temp_Coeff_gain_error, Temp_Coeff_gain_error2 ,TC_OS_error]; % upper 
lb_xy = [1 - Lin_error_xy, 0,                    -Temp_Coeff_gain_error2,-TC_OS_error]; % lower Limit for x/y axis
ub_z =  [1 + Lin_error_z,  Temp_Coeff_gain_error, Temp_Coeff_gain_error2 ,TC_OS_error]; % upper
lb_z =  [1 - Lin_error_z,  0,                    -Temp_Coeff_gain_error2,-TC_OS_error]; % lower Limit for z axis

%% Select Dataset
% It may be useful to measure two neighboring sensors at the same time (e.g. H1 and H2), 
% since the calibration coil has a sufficiently large homogeneous magnetic field in the center for this. 
% If so -> write down the mapping here.
% For example, if sensor 2 has been measured at the same time as sensor 1, then: Dataset{2} = 'H1';
Dataset{1} = 'H1'; 
Dataset{2} = 'H2';
Dataset{3} = 'H3';
Dataset{4} = 'H4';
Dataset{5} = 'H5';
Dataset{6} = 'H6';
Dataset{7} = 'H7';
Dataset{8} = 'H8';


%% SetUp
Folder = 'Log_Folder';

% 0) Hardware
active_HallSens = 8; % How much individual Hall Sensors are active?
error_CoilConversion = -4.2/100; % How much differs the real Conversion Factor from the simulation?
T_limit = 40; % Degree Celsius, Maximum allowed temperature of the sensor for accepted calibration point

show_plots = false; % true = show plots during post processing (Will slow down the processing time)
check_clear_OverTemp = true; % true = delete Datapoint, if Temperature of Sensor > T_limit

% 1) Zero Offset
limit_std_temperature = 1; % Maximum standard deviation of temperature during zero offset determination. The smaller the temperature change, the better the calibration will run.

% 2) Temperature Dependence
Temp_Ref = 25; % °C, Reference temperature (no temperature correction is needed at this temperature, arbitrarily set at room temperature)


%% Calibration File
CalFile.Zero_Offset = {};
CalFile.TempDependence = cell(active_HallSens,1);
CalFile.Linearity = cell(active_HallSens,1);


%% Perform Calibration
[CalFile, alpha, std_ret,CountMeasPoints,LogFile,~]= Function_PostProcessing_Calibration(...
                Folder, CalFile, limit_std_temperature, error_CoilConversion, Dataset,...
                false);


disp('------------FINSIHED CALIBRATION-----------------------');

%% Finish CalFile and save it
CalFile.SetUp.Date = datetime;
CalFile.SetUp.MeasSetUp = LogFile.SetUp;
CalFile.SetUp.CountMeasPoints = CountMeasPoints;
CalFile.SetUp.DatasetMatch = Dataset;

name = datestr(now);
name = strrep(name,':','_');
name = strrep(name,' ','_');
name = ['Calibration_' LogFile.SetUp.HallSens_Type '_' name];

save(name, 'CalFile');





