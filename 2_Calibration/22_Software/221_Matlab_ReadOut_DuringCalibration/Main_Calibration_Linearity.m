%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
%       Script to read out the Hall sensor during calibration.
%       Goal: determine linearity using a varying magnetic field
% 
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
%       MIT LICENSED
%       Have fun guys!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc;
clear;
close all;
pause(0.5);

%% SetUp Measurement Informations
num_AQ = 10;    %  Number of measured Points per Magnetic Field Step. Each Point is equivalent to an Arith.Average of "num_Averages_HallSens" Points
num_BField_Steps = 10; % Number of different magnetic fields that are set (i.e. number of measuring points used for linearization)

HallSensor_Type = '50mT';
measured_Sensor = "H1";  % Can be H1 ... H8
measurend_Direction = "x"; % Select Direction of SetUp/measured field

% SetUp Communication with Multimeter
Multi_COM_Port = "COM13";
Multi_SRL_Baud = 115200;

% SetUp Communication with Hallsensor
HallSens_COM_Port = "COM12";
HallSens_SRL_Baud = 115200;

%% Objects and Variables
tic;
% Global Variables
global HallSens;
global Multi;

global Multi_R_Shunt;
Multi_R_Shunt = 0.154; % Ohm, needs to be measured exactly
Shunt_Type = "Isabellenhuette_DX";

%% LogFile
global LogFile;

LogFile.HallSens.Bx = {};
LogFile.HallSens.By = {};
LogFile.HallSens.Bz = {};
LogFile.HallSens.Temp = {};
LogFile.HallSens.newData = {};
LogFile.Multi.Voltage = {};
LogFile.Multi.Current = {};
LogFile.SetUp.Shunt = Multi_R_Shunt;
LogFile.SetUp.ShuntType = Shunt_Type;
LogFile.SetUp.Time = datestr(datetime('now'));
LogFile.SetUp.CalibratedSensor = measured_Sensor;
LogFile.SetUp.CalibratedDirection = measurend_Direction;
LogFile.SetUp.HallSens_Type = HallSensor_Type;
LogFile.SetUp.Range = 50; %mT


%% Comunication Objetcs
disp('Setting Up Communication');
connected_ports = serialportlist('available');

% HallSens COM Port
if sum(contains(connected_ports,SRL_COM_Port))>=1
    HallSens = serialport(HallSens_COM_Port,HallSens_SRL_Baud);
    disp('...Hallsensor connected');
else
    disp('...Hallsensor was already connected');
end

% SetUp GW Instek 8255A-Multimeter
if sum(contains(connected_ports,Multi_COM_Port))>=1
    Multi = serialport(Multi_COM_Port,Multi_SRL_Baud);
    disp('...Multimeter connected');
else
    disp('...Multimeter was already connected');
end
configureTerminator(Multi,"CR");


%% Start Measurement
for i_B = 1:num_BField_Steps

    disp(['Step of B-Field-Sweep: ' num2str(i_B) ' of ' num2str(num_BField_Steps) ': Measuring...']);


    % Readout of Hallsensor Values at one external Magnetfield
    for i=1:num_AQ
        disp(['   Acquire Step: ' num2str(i) ' of ' num2str(num_AQ)]);
        [Bx_temp,By_temp,Bz_temp,Temp_temp,newData_temp] = Function_GetDataFromHallSens();
        [Volt_temp, Ampere_temp] = Function_GetCurrent();

        LogFile.HallSens.Bx{end+1} = Bx_temp;
        LogFile.HallSens.By{end+1} = By_temp;
        LogFile.HallSens.Bz{end+1} = Bz_temp;
        LogFile.HallSens.Temp{end+1} = Temp_temp;
        LogFile.HallSens.newData{end+1} = newData_temp;
        LogFile.Multi.Voltage{end+1} = Volt_temp;
        LogFile.Multi.Current{end+1} = Ampere_temp;
        disp("   ---");

    end

    % Wait here until the user has increased the current (setting on the power supply, for example).
    % If the new current is set: the user presses Enter and the script continues.

    if i_B<num_BField_Steps
        input(" ----------- NEXT B-Field Step: Set new Current with your Power Supply. Then press 'ENTER' --------- ");
        pause(1); % Wait a bit, so the Hallsens measured the new field
    end

end



%% Close Serial Port
clear HallSens;
clear Multi;

%% Saving the calculated Offsets as a LogFile


save_name = strrep(LogFile.SetUp.Time,':','_');
save_name = strrep(save_name,' ','_');
save_name = ['Linearity_Cal_' HallSensor_Type '_Date_' save_name '.mat'];
save(save_name,'LogFile');

toc;
