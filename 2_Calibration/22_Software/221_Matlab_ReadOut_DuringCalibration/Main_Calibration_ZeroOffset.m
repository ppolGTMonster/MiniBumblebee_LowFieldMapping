
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
%       Script to read out the Hall sensor during Zero Offset Measurement.
%       Goal: zero offset
%       Makes just sense, when sensor can be placed in a magnetic shielding chamber 
% 
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
%       MIT LICENSED
%       Have fun guys!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



clc;
clear;
close all;
pause(0.5);

%% Objects and Variables

tic;

% SetUp HallSensor
num_Averages_HallSens = 1000; % Programmed Arith. Averages in HallSensor
SRL_Baud = 115200;
HallSensor_Type = '50mT';
HallSensor_numActiveSensors = 8; % 8 Sensors on the Frontend Probe
HallSensor_TempDrift = -0.12 / 100; % Temperatur Coefficient of Hallsensor is -0.12%/°C 
HallSensor_ENOB = 10; %Bit

% HallSens COM Port
SRL_COM_Port = "COM12";
global HallSens;
HallSens = serialport(SRL_COM_Port,SRL_Baud);


% Calibration SetUp
num_AQ = 1000;    %  Number of measured Points. Each Point is equivalent to an Arith.Average of "num_Averages_HallSens" Points

% Variables
Bx = zeros(8,num_AQ);
By = zeros(8,num_AQ);
Bz = zeros(8,num_AQ);
Temp = zeros(8,num_AQ);
newData = zeros(8,num_AQ);

for i=1:num_AQ
    [Bx_temp,By_temp,Bz_temp,Temp_temp,newData_temp] = Function_GetDataFromHallSens();
    Bx(1:end,i) = Bx_temp;
    By(1:end,i) = By_temp;
    Bz(1:end,i) = Bz_temp;
    Temp(1:end,i) = Temp_temp;
    newData(1:end,i) = newData_temp;
end


%% Getting the averaged Offset (Assuming Temperature stays stable during Measurement of all Points), but individual for eacht independant Hallsensor
Bx_Offset = mean(Bx,2);
By_Offset = mean(By,2);
Bz_Offset = mean(Bz,2);
Temp = mean(Temp,2);
newData = mean(newData,2);

B_Offset = sqrt(Bx_Offset.^2+By_Offset.^2+Bz_Offset.^2);


%% Close Serial Port
clear HallSens;

%% Saving the calculated Offsets as a LogFile
Logfile.Bx_Offset = Bx_Offset;
Logfile.By_Offset = By_Offset;
Logfile.Bz_Offset = Bz_Offset;
Logfile.B_Offset = B_Offset;
Logfile.Temp = Temp;
Logfile.newData = mean(newData,2);
Logfile.Range = 50;

Logfile.Resolution = (Logfile.Range/2^HallSensor_ENOB)/sqrt(num_AQ*num_Averages_HallSens) ; %in mT
Logfile.ENOB_real = floor(log2((Logfile.Range/Logfile.Resolution)));
Logfile.Time = datestr(datetime('now'));

save_name = strrep(Logfile.Time,':','_');
save_name = strrep(save_name,' ','_');
save_name = ["Offset_Cal_" HallSensor_Type "Date_" save_name '.mat'];

save(save_name,'Logfile');


toc;
