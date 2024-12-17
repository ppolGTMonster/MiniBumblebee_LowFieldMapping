%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script to determine the error of the calibration coil.                                                                          %
%       The measurement data is used for this and the calibration is run through.                                                       %
%       At the end, the error of the coil is evaluated using the specified data sheet limits of the sensor                              %
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
addpath('Functions_PostProcessing');
addpath('Functions_CoilError_Estimation');



% Global Variables
global active_HallSens;
global show_plots;
global CalCoil_Conversion;
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
Lin_error = 1.7 /100; % From Datasheet
Lin_error_xy = 2.5/100;
Lin_error_z = 4.9/100;
Temp_Coeff_gain_error = 0.24/100; %1/K From Datasheet
Temp_Coeff_gain_error2 = Temp_Coeff_gain_error/2;
TC_OS_error = 50*0.07; %mT/°C, fomr Application Info for A1342
OS_Earth = 50 * 1.5 *1e-3; % mT, Earth Magnetic Field + stray field

% Weighting of the measured values between the linearity measurement and the temperature sweep measurement:
opt_weight_MeasCal = 2; % In the Lin measurement, the sensors were individually perfectly in the center of the cylinder (therefore double weighted).
opt_weight_TempCal = 1; % This was not so good in the temperature measurement (therefore individually weighted).

% Variables that are optimized: [Gain TC1(Gain) TC12(Gain) TC2(OS)]x, ... ,[Gain TC1(Gain) TC12(Gain) TC2(OS)]z OS_Env
% Where OS_Env stands for earth's magnetic field + stray field
x0 = [1.01,0.001,0,40e-3];% Start Value
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




%% Result Coil Optimization

result_offset = [];
result_offset_list_x=[];
result_offset_list_y=[];
result_offset_list_z=[];
RMS_Ret=[];
offset_list = linspace(0,6,31);   %%%%%%%%%%%%%%%%%%%%%% OFFSET LIST in %
alphaList_Offset=cell(size(offset_list));

std_overview = [];



% Sweep thorugh all prepared offset errors
for i_offset = 1:length(offset_list)
    disp(['Step ' num2str(i_offset)]);

    error_CoilConversion = -1*offset_list(i_offset)/100; % How much differs the real Conversion Factor from the simulation?

    %% Perform a  Calibration using the selected Coil Error
    [CalFile, alpha, std_ret,~,LogFile,CoilError]= Function_PostProcessing_Calibration(...
        Folder, CalFile, limit_std_temperature, error_CoilConversion, Dataset,...
        true);


    RMS_Ret = [RMS_Ret,CoilError.RMS];
    std_overview(:,end+1) = CoilError.std';


    %% Evaluate the result according to alpha (how many comply with the datasheet-sensitivity)
    % If the deviation of the sensors from the data sheet value is 0.5% greater than the maximum deviation allowed in the data sheet,
    % this is count as an error and increases the index

    error_nonlin = 0;
    error_ninlin_z = 0;
    error_list_x=ones(active_HallSens,1)*0;
    error_list_y=ones(active_HallSens,1)*0;
    error_list_z=ones(active_HallSens,1)*0;

    alphaList_Offset{i_offset} = alpha;

    for i = 1:active_HallSens
        at = alpha{i};

        x_lin = at(1,1);
        y_lin = at(2,1);
        z_lin = at(3,1);


        if abs(x_lin-1) < 99.5/100*Lin_error_xy
        else
            error_nonlin = error_nonlin+1;
            error_list_x(i)=i;
        end

        if abs(y_lin-1) < 99.5/100*Lin_error_xy
        else
            error_nonlin = error_nonlin+1;
            error_list_y(i)=i;
        end

        if abs(z_lin-1) < 99.5/100*Lin_error_z
        else
            error_ninlin_z = error_ninlin_z+1;
            error_list_z(i)=i;
        end

    end
    result_offset = [result_offset; offset_list(i_offset) error_nonlin error_ninlin_z];
    result_offset_list_x = [result_offset_list_x error_list_x];
    result_offset_list_y = [result_offset_list_y error_list_y];
    result_offset_list_z = [result_offset_list_z error_list_z];


end

%% Post Processing & Evaluation
disp('----- CALCULATION FINISHED -----');


%% Figure 1:
% Plot the STD for every sensor individually
% This information is well suited to get a first insight into the overall performance of the Hall probe.
% However, the axes perform differently. In particular, the z-axis of the sensor is intrinsically worse.
% This plot only shows the summed total performance of an individual sensor. Caution is necessary!
figure;
fig = tiledlayout(1,active_HallSens+1);
title(fig, 'STD Sum for every Sensor');
for i=1:active_HallSens

    ax = nexttile;
    plot(offset_list,std_overview(i,:));
    hold on;
    title(ax,['Sensor ' num2str(i)]);
    [~,ind]  = min(std_overview(i,:));
    xlabel('Coil error (%)');
    ylabel('STD Sum');

    xline(offset_list(ind),'-.','Minima');
end
ax = nexttile;
plot(offset_list,sum(std_overview,1));
hold on;
title(ax,'Sum over all Sensors');
[~,ind]  = min(sum(std_overview,1));
xlabel('Coil error (%)');
ylabel('STD Sum');
xline(offset_list(ind),'-.','Minima');


%% Figure 2:
% Plot how many axes of the sensors run out of the limits according to the data sheet.
% This information is well suited to determine the final error of the calibration coil.

num_error = zeros(size(result_offset_list_x));
num_error(result_offset_list_x>0.1)=1;
% In result_offset_list_x/y/z, the indices indicate for each sensor which has crossed the boundary.
%The >0.1 is just a threshold (only integers are in the matrix).
num_error_x = sum(num_error,1);

num_error = zeros(size(result_offset_list_y));
num_error(result_offset_list_y>0.1)=1;
num_error_y = sum(num_error,1);

num_error = zeros(size(result_offset_list_z));
num_error(result_offset_list_z>0.1)=1;
num_error_z = sum(num_error,1);


figure;
fig = tiledlayout(2,1);
ax = nexttile;
title(fig,'Calibration Error Analysis')
plot(offset_list,num_error_x,'-x');
hold on;
plot(offset_list,num_error_y,'-x');
plot(offset_list,num_error_z,'-x');
plot(offset_list,num_error_x+num_error_y+num_error_z,'-x');

yline(active_HallSens*3);
yline(active_HallSens*1,'--');
title(ax,'Total number of sensors that have run into the limit');
xlabel(ax,'Assumed Coil Error - negative (%)');
ylabel(ax,'Number of sensors (per axis) above the limit');
ylim([0,28]);
xlim([offset_list(1) offset_list(end)+0.5]);
xline(offset_list,'--');
legend(ax,'X-Axis','Y-Axis','Z-Axis','Sum', 'Total number of sensors', 'Number of z-axis only');


ax =nexttile;
hold on;
steps_offset = offset_list(2)-offset_list(1);
offset_x = 0.2*steps_offset;
offset_y = 0.5*steps_offset;
offset_z = 0.7*steps_offset;

for i=1:8
    plot(offset_list+offset_x,result_offset_list_x(i,:),'o','MarkerSize',8,'Color','b','MarkerFaceColor','b');
    plot(offset_list+offset_y,result_offset_list_y(i,:),'o','MarkerSize',8,'Color','k','MarkerFaceColor','k');
    plot(offset_list+offset_z,result_offset_list_z(i,:),'o','MarkerSize',8,'Color','r','MarkerFaceColor','r');
    yline(i,'--');
end
ylim([0.1,8.8]);
xlim([offset_list(1) offset_list(end)+0.5]);
title(ax,'Calibration error (X=blue, Y=black, Z=red)');
ylabel(ax,'Sensor #');
xlabel(ax,'Assumed Coil Error - negative (%)');
xline(offset_list,'--');



%% Figure 3:
% Only the linearity factors of the Hall sensors themselves
% (which are most decisive for the evaluation) are analyzed.
Sens_x = [];
Sens_y = [];
Sens_z = [];

for i1=1:length(alphaList_Offset)

    a_t = alphaList_Offset{i1};

    x_temp =[];
    y_temp=[];
    z_temp=[];

    for i2=1:active_HallSens

        a_t2 = a_t{i2};
        x_temp = [x_temp; a_t2(1,1)];
        y_temp = [y_temp; a_t2(2,1)];
        z_temp = [z_temp; a_t2(3,1)];
    end

    Sens_x = [Sens_x x_temp];
    Sens_y = [Sens_y y_temp];
    Sens_z = [Sens_z z_temp];
end

figure;
fig = tiledlayout(3,1);
title(fig, 'Linearity analysis: deviation in % from ideal linearity (=1)');


ax=nexttile;
plot(offset_list,(Sens_x-1)*100);
hold on;
yline(Lin_error_xy*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_xy*100);
legend('H1','H2','H3','H4','H5','H6','H7','H8','Location','southwest');
title(ax,'Axis X');
xlabel(ax,'Assumed Coil Error - negative (%)');
ylabel(ax,'deviation from ideal linearity (%)');



ax=nexttile;
plot(offset_list,(Sens_y-1)*100);
hold on;
yline(Lin_error_xy*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_xy*100);
title(ax,'Axis Y');
xlabel(ax,'Assumed Coil Error - negative (%)');
ylabel(ax,'deviation from ideal linearity (%)');



ax=nexttile;
plot(offset_list,(Sens_z-1)*100);
hold on;
yline(Lin_error_z*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_z*100);
title(ax,'Axis Z');
xlabel(ax,'Assumed Coil Error - negative (%)');
ylabel(ax,'deviation from ideal linearity (%)');




