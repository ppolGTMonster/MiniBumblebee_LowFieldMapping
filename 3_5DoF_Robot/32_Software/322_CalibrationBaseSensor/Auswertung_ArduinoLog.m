%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script that analyzes the precission of the Base Angle Sensor
%       Goal: determine linearization coeff of sensor                                                                              %
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;

% Names of the Logfiles
name_Dataset_Optical = 'MeasPoints_Optical6.mat'; % Calcuated Angles

% phys Parameter Sensing System
l = 77.698;         %mm, Length Lever
Ucc= 5.023;         %V, Voltage Supply of Potentiometer/Voltage Divider

% phys Parameter Calibration Setup
dist_board = 595; %mm, Distance between rotational axis and board (where the aruco board was attached)

% Load Files
MeasPoint_Opt = load(name_Dataset_Optical);
MeasPoint_Opt = MeasPoint_Opt.MeasPoints_Optical6;
MeasPoint_Opt = table2array(MeasPoint_Opt);

dist_opt_P1 = MeasPoint_Opt(:,2);                       %mm, relative Distance to first point
angles_opt_P1 = rad2deg(atan(dist_opt_P1/dist_board))'; %°, Convert to Degree


%% Linear Fit

% Load Meas Data
adc_V = MeasPoint_Opt(:,3)/1000;

%Linear Fit
p = polyfit(adc_V,angles_opt_P1,1);
deg_ideal = polyval(p,adc_V);

%Calculate further Parameter (used in the derivation of the sensor)
k_real = 180/(p(1)*pi*l);
lpoti_real = Ucc/k_real;
x0_real = -pi*l/180*p(2);


%% Output
disp('--------- RESULT ----------');
t1 = ['1) Linearization Result: \n alpha_base = alpha_gain*U + alpha_offset [°] \n with alpha_gain = ' num2str(p(1)) ' °/V, alpha_offset = ' num2str(p(2)) '°'];
disp(sprintf(t1));

%% Plot
figure;
fig = tiledlayout(2,1);
title(fig, 'Result Calibration');
ax = nexttile;
plot(adc_V*1000);
title(ax,'Voltage Measured');
ylabel('mV');

ax=nexttile;
hold on;
plot(angles_opt_P1);
plot(deg_ideal);
ylabel('Angle (°)')
legend('Meas Optical','Linear Fit');
title(ax,'Angles');
