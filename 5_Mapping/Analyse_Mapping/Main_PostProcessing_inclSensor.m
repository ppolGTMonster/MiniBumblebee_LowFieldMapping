% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script that analyzes the mapped magnetic field data and plots them
%       in post processing
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



close all;
clear;
clc;


%% 0) Global Variables
% Predefined Settings
global SetFile_Mechanical;
global SetFile_Kinematics;
global SetFile_Comm;
global CalFile;
global HallData;
global active_Hallsens;
global Tref;

global show_plot;
global use_CalFile;
global show_plot_Hallsens;

%% 1) User Settings
show_plot = true;           % true = show plot of every motion tracked position
show_plot_Hallsens=false;   % true = show trajectory, how the Sensor head moved inside the FOV (careful, takes really long!)
use_CalFile = true;         % true = use Calibration File & apply the calibration on the measured Hall sensor data

active_Hallsens=8;  % amount of active sensors
Tref = 23;          %°C, Reference Temperature. Here set to room temperature, which was pretty constant during the measurement
step_goal = 5e-3;   %m, Step size of the interpolated grid


% Measured Hall Data
file_HallData = '\Halbach\Measurement_17-Apr-2024_20_11_23.mat';
%file_HallData = '\GradX\Measurement_17-Apr-2024_21_10_23.mat';

% Motion Tracking Data
file_MT = '\Halbach\MotionTracking_Log.csv';
%file_MT = '\GradX\MotionTracking_Log.csv';

% FOV Points
file_FOV='\Halbach\FOV_Trajectory_alphas.mat';
%file_FOV='\GradX\FOV_Trajectory_alphas.mat';

%Calibration Data
file_CalHallData = '\Calibration_50mT_27-May-2024_MessungFeb_Coilm4.2_mitOffset_6z.mat';


%% 1) Load Files
data_folder = [pwd '\Analyse_Mapping\Data_for_Evaluation'];
file_MT = [data_folder file_MT];
file_HallData = [data_folder file_HallData];
file_CalHallData = [data_folder file_CalHallData];
file_FOV = [data_folder file_FOV];

% FOV Points
load(file_FOV);
Points_FOV = save_file.pos';
% Measured Magnetic field data
load(file_HallData);
HallData = Save_Data;
%Calibration Parameters
load(file_CalHallData);
MTData = readtable(file_MT);
% alpha Base (measured with extra sensor) is stored in the Hallsensor Data
alpha0_Sensor = cell2mat(HallData.alpha_Base_Meas');

% Load Settings
[SetFile_Mechanical,SetFile_Kinematics,SetFile_Comm] = get_SetFile();

clear save_file Save_Data file_HallData file_MT file_CalHallData alpha0_MT

%% 2) Use the calibration to correct the measured magnetic field values.
if use_CalFile== true
    Function_Hallsensor_CorrectData()
end


%% 3) Extract RotMat from MotionTracking
A_kin_Sensor = cell(height(MTData),1);
alpha_MT = [MTData.Robot_alpha0, MTData.Robot_alpha1, MTData.Robot_alpha2, MTData.Robot_alpha3]; % Data from Motion Tracking

for i=1:height(MTData)
    alpha0_temp  = alpha0_Sensor(i); % Data from extra sensor
    alpha = alpha_MT(i,:);
    %get affine Matrix
    [A_new_Sensor] = Function_Kinematics_ConvAlpha2AffinMatrix(alpha,alpha0_temp);
    A_kin_Sensor{i} = A_new_Sensor;
end


%% 4) Calculate the affine Matrices of all 8 Hallsensors
[H_Ref_Hi_kin] = Function_Hallsensor_CalculateAffineMatrices(A_kin_Sensor);


%% 5) Correct Rotation of all 8 Hallsensors

HallData_cor = cell(length(HallData.B_meas),1); % Corrected Magnet Field Data (Rotatet to FOV CoorSys)
HallData_pos = cell(length(HallData.B_meas),1); % Position of every individual sensor

for i=1:length(HallData.B_meas)
    B_Data = HallData.B_correct{i};
    H_Ref_Hi_kin_tmp = H_Ref_Hi_kin(i,:);

    [HallSens_rot,HallSens_pos] = Function_Hallsensor_Rotate2FOVsystem(B_Data,H_Ref_Hi_kin_tmp);
    HallData_cor{i} = HallSens_rot;
    HallData_pos{i} = HallSens_pos;

end

%% 6) If measured File is a Gradient -> Norm the measured Values, to avoid a fluctuating current source
[HallData_cor] = help_function_checkGradient(HallData_cor);


%% 7) Interpolation
%Due to the kinematics and rotations, the mapped points are not equally distributed.
% Therefore, interpolate the values for an even grid.

% Generate one big Dataset of all measured points
HSens_pos_kin = [];
HSens_kin = [];

for i=1:length(HallData_cor)

    c_x = HallData_pos{i,1}(1,:);
    c_y = HallData_pos{i,1}(2,:);
    c_z = HallData_pos{i,1}(3,:);
    c = [c_x' c_y' c_z'];
    HSens_pos_kin= [HSens_pos_kin; c];


    c_x = HallData_cor{i,1}(1,:);
    c_y = HallData_cor{i,1}(2,:);
    c_z = HallData_cor{i,1}(3,:);
    c = [c_x' c_y' c_z'];
    HSens_kin=[HSens_kin;c];
end

% Split Position % Magnetic Field vector
x_vec = HSens_pos_kin(:,1);
y_vec = HSens_pos_kin(:,2);
z_vec = HSens_pos_kin(:,3);

Bx = HSens_kin(:,1);
By = HSens_kin(:,2);
Bz = HSens_kin(:,3);

% Calculate ideal interpolated grid
x_ip = min(x_vec):step_goal: max(x_vec);
y_ip = min(y_vec):step_goal: max(y_vec);
z_ip = min(z_vec):step_goal: max(z_vec);
[x_kin_q,y_kin_q,z_kin_q] = meshgrid(x_ip,y_ip,z_ip);

% Some measured points are nan (no connection to sensor during measurement) -> Delete this points for a good interpolation
mask = isnan(Bx);
Bx=Bx(mask==0);
By=By(mask==0);
Bz=Bz(mask==0);
x_vec=x_vec(mask==0);
y_vec=y_vec(mask==0);
z_vec=z_vec(mask==0);

% Interpolate
Bx_kin_ip = griddata(x_vec,y_vec,z_vec, Bx ,x_kin_q,y_kin_q,z_kin_q,'natural');
By_kin_ip = griddata(x_vec,y_vec,z_vec, By ,x_kin_q,y_kin_q,z_kin_q,'natural');
Bz_kin_ip = griddata(x_vec,y_vec,z_vec, Bz ,x_kin_q,y_kin_q,z_kin_q,'natural');
B_kin_ip =  sqrt(Bx_kin_ip.^2 + By_kin_ip.^2+Bz_kin_ip.^2);


%% 8) Finalize Analysis
% Generate FOV-mask
FOV_Radius = 10/1000; %m, radius of spherical FOV for calculation of Homogeneity
mask_FOV = zeros(size(x_kin_q));
mask_FOV(x_kin_q.^2+y_kin_q.^2 +z_kin_q.^2 <FOV_Radius^2) = 1;
mask_FOV(mask_FOV==0)=NaN;

% Mask Data
B_FOV = B_kin_ip.*mask_FOV;

% Calculate Min/Max/Mean/Homog
mean_mF = mean(B_FOV,'all','omitnan');
min_mF = min(B_FOV,[],'all','omitnan');
max_mF = max(B_FOV,[],'all','omitnan');
B0_val = mean_mF; %mT
delta_B0  = (max_mF-min_mF);  %mT
B0_ppm = delta_B0/B0_val*1e6;

disp(['Homogeneity: mean=' num2str(mean_mF) ' mT, min=' num2str(min_mF) ...
    ' mT, max=' num2str(max_mF) ' mT, delta=' num2str(delta_B0) ' mT, ppm = ' num2str(B0_ppm)]);


%% 9) Save Results
name = datestr(now);
name = strrep(name,':','_');
name = strrep(name,' ','_');
input_files.Points_FOV = Points_FOV;
input_files.HallData = HallData;
input_files.MTData = MTData;
input_files.alpha0_Sensor = alpha0_Sensor;
input_files.CalFile = CalFile;

save(['Analyse_Mapping\Result_Mapping\Result_Analysis_' name '.mat',''],'z_kin_q' , 'y_kin_q' ,'x_kin_q' , 'Bz_kin_ip' , 'By_kin_ip' , 'Bx_kin_ip' , 'B_kin_ip','z_vec', 'y_vec', 'x_vec', 'Bz', 'By','Bx','input_files');


%% 10) Generate Plots

% % % 10.1)  Comparison of the two measured values of alpha0 (with Motion Tracking & with extra sensor)
alpha0_MT=table2array(MTData(:,9:12));
alpha0_MT=alpha0_MT(:,1);
alpha0_MT = rad2deg(alpha0_MT);

figure;
hold on;
plot(alpha0_MT);
plot(alpha0_Sensor);
ylabel('Angle (°)');
xlabel('Step');
title('Comparison of base angle measurements alpha0');
legend('MotionTracking','Sensor');
hold ofF;

% % % 10.2) Plot of all motion-tracked points in the FOV
% This is a good way to check whether the coordinate systems have been rotated correctly.

% Convert points from robot coordinates to openCV coordinates (only translation)
fov_x = -1*Points_FOV(:,1);
fov_y = Points_FOV(:,3);
fov_z = Points_FOV(:,2);
Points_FOV = [fov_x fov_y fov_z];
Points_FOV = Points_FOV + SetFile_Mechanical.Env.t_ORB_To_RobotBase';

% Get Points of a cylinder (inside wall of Hallbach Magnet)
[cyl_x,cyl_y,cyl_z] = genFOVCylinder();

% Plot
if show_plot==true
    figure('WindowState','maximized');
    fig = tiledlayout(1,2);
    title(fig,'Result Motion Tracking')

    % show only every 10th point, otherwise it takes forever
    for i=1:10:length(A_kin_Sensor)

        t_A_kin = A_kin_Sensor{i}(1:3,end);
        R_A_kin = A_kin_Sensor{i}(1:3,1:3);
        fov = Points_FOV(1:i,:);

        % Target FOV Points
        ax1 = nexttile(1);
        grid on;
        hold on;
        plot3D_help(fov,'k','-o');                  % Plot Points
        plot3D_help('Cylinder',cyl_x,cyl_y,cyl_z);  % Plot Cylinder
        plot3D_help('FOV Target',ax1);              % Adjust Plot (Title etc)
        hold off;

        % Mapped Points
        ax1 = nexttile(2);
        grid on;
        hold on;
        plot3D_help(t_A_kin','b','o');
        plot_coorsys(t_A_kin,R_A_kin,'--'); % Plot small Coor-Sys at every mapped point
        plot3D_help('Cylinder',cyl_x,cyl_y,cyl_z);
        plot3D_help('FOV Mapped',ax1);
        hold off;

    end

end


% % % 10.3) Plot a simulation of the measured trajectory of the hall sensors
% (good for debugging whether all coordinate systems are rotated correctly)

pos_sens_kin=[];
pos_P5_kin =[];
txt = {};

for i=1:length(HallData_pos)

    for i_2=1:active_Hallsens
        pos_sens_kin(end+1,:) = HallData_pos{i,1}(:,i_2)';
        txt{end+1} = ['H' num2str(i_2)];
    end
end

pos_sens_kin = pos_sens_kin*1000;
pos_P5_kin = pos_P5_kin * 1000;


if show_plot_Hallsens
    figure('WindowState','maximized');
    fig = tiledlayout(1,2);

    for q=1:length(HallData_pos)

        ax = nexttile(1);
        grid on;
        for i_2 = 1:active_Hallsens
            index = i_2 + active_Hallsens*(q-1);
            scatter3(pos_sens_kin(index,1),pos_sens_kin(index,3),pos_sens_kin(index,2), 'r','filled');
            hold on;
            text(pos_sens_kin(index,1),pos_sens_kin(index,3),pos_sens_kin(index,2), txt(index));
        end

        title(ax,'Position Kinematik');
        xlabel('X_{BCS} (-x_{ML})');
        ylabel('Z_{BCS} (y_{ML})');
        zlabel('Y_{BCS} (z_{ML})');
        ax.YDir = 'reverse';
        view([29.838652765645847,28.706626513317985]);
        xlim([-50 50]);
        zlim([-30 70]);
        ylim([-50 50]);
        hold ofF;

        drawnow;
        pause(0.3);

    end
end


% % % 10.4) Plot of the magnetic field from motion tracking

B_kin=[];
pos_kin=[];
pos_P5_kin =[];

for i=1:length(HallData_pos)

    % kin
    for i_2=1:active_Hallsens
        B_kin(end+1,:) = HallData_cor{i,1}(:,i_2)';


        pos_kin(end+1,:) = HallData_pos{i,1}(:,i_2)';
    end

    pos_P5_kin(end+1,:)=(A_kin_Sensor{i}(1:3,end)-SetFile_Mechanical.Env.t_ORB_To_MagnetFOV)';
end



figure('WindowState','maximized');
fig = tiledlayout(1,2);

% Vector Plot of Magnetic Field
ax = nexttile;
quiver3( pos_kin(:,1),pos_kin(:,3),pos_kin(:,2), ...
    B_kin(:,1), B_kin(:,3),B_kin(:,2));
hold on;
scatter3( pos_kin(:,1),pos_kin(:,3),pos_kin(:,2), 'r','filled');
scatter3(pos_P5_kin(:,1),pos_P5_kin(:,3),pos_P5_kin(:,2),'+','k');
title(ax,'Vector Plot');
xlabel('X_{BCS} (-x_{MatLab})');
ylabel('Z_{BCS} (y_{MatLab})');
zlabel('Y_{BCS} (z_{MatLab})');
axis equal;
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);


% Color Plot of B_abs

ax = nexttile;
B_kin_mag = sqrt(sum(B_kin.^2,2));
scatter3( pos_kin(:,1),pos_kin(:,3),pos_kin(:,2), [], B_kin_mag ,'filled');
c = colorbar;
c.Label.String = 'mT';
hold on;
scatter3(pos_P5_kin(:,1),pos_P5_kin(:,3),pos_P5_kin(:,2),'+','k');
title(ax,'Magnitude Plot');
xlabel('X_{BCS} (-x_{MatLab})');
ylabel('Z_{BCS} (y_{MatLab})');
zlabel('Y_{BCS} (z_{MatLab})');
axis equal;
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);
hold off;

% % % 10.5) Plot of the interpolated magnetic field

pos_fov = ConvMesh2Vector(x_kin_q,y_kin_q,z_kin_q);
B_fov = ConvMesh2Vector(Bx_kin_ip,By_kin_ip,Bz_kin_ip);

pos_fov = 1000*pos_fov';
B_fov = B_fov';

figure('WindowState','maximized');
fig = tiledlayout(2,2);
title(fig, 'Plot of interpolated B, all axes in mm');

ax = nexttile;
scatter3(pos_fov(1,:),pos_fov(3,:),pos_fov(2,:), [],B_fov(1,:),'filled');
c = colorbar;
c.Label.String = 'mT';
title(ax,'Bx');
axis equal;
xlabel('X_{BCS} (-x_{ML})');
ylabel('Z_{BCS} (y_{ML})');
zlabel('Y_{BCS} (z_{ML})');
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);

ax = nexttile;
scatter3(pos_fov(1,:),pos_fov(3,:),pos_fov(2,:), [],B_fov(2,:),'filled');
c = colorbar;
c.Label.String = 'mT';
title(ax,'By');
axis equal;
xlabel('X_{BCS} (-x_{ML})');
ylabel('Z_{BCS} (y_{ML})');
zlabel('Y_{BCS} (z_{ML})');
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);

ax = nexttile;
scatter3(pos_fov(1,:),pos_fov(3,:),pos_fov(2,:), [],B_fov(3,:),'filled');
c = colorbar;
c.Label.String = 'mT';
title(ax,'Bz');
axis equal;
xlabel('X_{BCS} (-x_{ML})');
ylabel('Z_{BCS} (y_{ML})');
zlabel('Y_{BCS} (z_{ML})');
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);


ax = nexttile;
scatter3(pos_fov(1,:),pos_fov(3,:),pos_fov(2,:), [],vecnorm(B_fov),'filled');
c = colorbar;
c.Label.String = 'mT';
title(ax,'B');
axis equal;

xlabel('X_{BCS} (-x_{ML})');
ylabel('Z_{BCS} (y_{ML})');
zlabel('Y_{BCS} (z_{ML})');
ax.YDir = 'reverse';
view([29.838652765645847,28.706626513317985]);









%% Local Functions


function [HallData_cor] = help_function_checkGradient(HallData_cor)
%Reads the magnetic field and normalizes current fluctuations

global HallData;
% Check if Gradient Field was measured
try % Strange code, because in this Matlab version it seems to be not possible to check if a struct has a corresponding field
    test = HallData.Multi;
    flag_isGradient = true;
catch
    flag_isGradient = false;
end

if flag_isGradient==true

    I_tol = 30/100; % maximum tolerance allowed between the two measured values, if it is greater, it is highly likely that one of the two values has been measured incorrectly.
    I_meas = HallData.Multi.Current;

    for i=1:length(I_meas(:,1))

        I1 = I_meas(i,1);
        I2 = I_meas(i,2);

        if abs(1-I1/I2) > I_tol
            if abs(I1)>abs(I2)
                I_max = I1;
            else
                I_max=I2;
            end
            I_avg = I_max;
        else
            I_avg = (I1+I2)/2;
        end
        I_meas(i,3) = I_avg;
    end
    I_avg = mean(I_meas(:,3));
    scale_fac = I_avg./I_meas(:,3);

    for i=1:length(HallData_cor)

        HallData_cor{i,1} = scale_fac(i) * HallData_cor{i,1};

    end

end

end

function [cyl_x,cyl_y,cyl_z] = genFOVCylinder()
% Calculate Points of Cylinder for Plot (symbolizes the inner wall of the examined Hallbach magnet)
% -Y easy check if mapped coordinates are sensefull and not touching the wall
global SetFile_Mechanical;

% FOV Cylinder
r = 80e-3; %radius
h = 300e-3;% heights
[cyl_x,cyl_y,cyl_z] = cylinder(r);
cyl_z = h*cyl_z;
trafo = SetFile_Mechanical.Env.t_ORB_To_MagnetFOV;

rotMat = help_function_RotY(deg2rad(90));

ring_o1 = [cyl_x(1,:)',cyl_y(1,:)',cyl_z(1,:)'];
ring_o2 = [cyl_x(2,:)',cyl_y(2,:)',cyl_z(2,:)'];
p_new1 = ring_o1*rotMat;
p_new2 = ring_o2*rotMat;

% reassemble the two sets of points into X Y Z format:
cyl_x = [p_new1(:,1),p_new2(:,1)];
cyl_y = [p_new1(:,2),p_new2(:,2)];
cyl_z = [p_new1(:,3),p_new2(:,3)];

%FOV Center in BCS coordinates
dx = trafo(1);
dy = trafo(2);
dz = trafo(3);

cyl_x = cyl_x+dx+300/2*1e-3;
cyl_y = cyl_y+dz;
cyl_z = cyl_z+dy;
end

function [] = plot3D_help(in1,in2,in3,in4)
% Helper for 3D plotting

if nargin==3    % Plot Data
    data_3d = in1;
    col = in2;
    style = in3;
    val_x = data_3d(:,1);
    val_y = data_3d(:,3); 
    val_z = data_3d(:,2);
    plot3(val_x,val_y,val_z,style,'Color',col);

elseif nargin==2    % Set Title and axis
    tit_string = in1;
    ax = in2;
    title(ax,tit_string);
    view([-30,37.5]);
    xlabel('X_{BCS} (-x_{MatLab})');
    ylabel('Z_{BCS} (y_{MatLab})');
    zlabel('Y_{BCS} (z_{MatLab})');
    xlim([-0.4,0.25  ]);
    ylim([   0,0.3]);
    axis equal;
    ax.YDir = 'reverse';
else % Cylinder Plot
    cyl_x = in2;
    cyl_y = in3;
    cyl_z = in4;
    s = surf(cyl_x,cyl_y,cyl_z,'FaceColor','#80B3FF', 'EdgeColor','none');alpha(s,.2);      % Cylinder
end

end

function [] = plot_coorsys(t0,R,linstyl)
% Plot small coordinate systems at the mapped points

scale = 0.01;
xline = t0 + [[0;0;0] , R*[1;0;0]*scale];
yline = t0 + [[0;0;0] , R*[0;1;0]*scale];
zline = t0 + [[0;0;0] , R*[0;0;1]*scale];
% plot coor sys
plot3(xline(1,:),xline(3,:),xline(2,:),['r' linstyl]); %xline
plot3(yline(1,:),yline(3,:),yline(2,:),['g' linstyl]); %xline
plot3(zline(1,:),zline(3,:),zline(2,:),['b' linstyl]); %xline

end

function [ret] = ConvMesh2Vector(x,y,z)
% Convert 3x3 Mesh into single vector
ret = [x(:),y(:),z(:)];
end

function [R] = help_function_RotY(alpha)
% Rotational Matrix around y-axis
R = [cos(alpha)             ,0  ,sin(alpha);...
    0                      ,1  ,0         ;...
    -1*sin(alpha)          ,0  ,cos(alpha)];
end