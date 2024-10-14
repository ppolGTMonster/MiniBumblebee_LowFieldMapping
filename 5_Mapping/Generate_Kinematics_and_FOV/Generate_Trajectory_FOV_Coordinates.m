%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Script to calculate the  positions of the FOV points 
%       at the moment only cubic FOVs are supporte, but this could be
%       easily adapted to other styles here
%                                                                                                                                       %
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany                                                                            %
%       MIT LICENSED                                                                                                                    %
%       Have fun guys!                                                                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;
close all;


%% 0) Global Variables
% Predefined Settings
global SetFile_Mechanical;
global SetFile_Kinematics;

%% 1) Load Settings
[SetFile_Mechanical,SetFile_Kinematics,~,~] = get_SetFile();

%% 2)  Set Variables & configure FOV
save_name = 'Save_Folder\FOV_Trajectory_alphas.mat';
plot_flag = true;  % true = plot FOV points

FOV_Style = "Cube";
%Take care about the rotated coor systems of robot and FOV (cylinder axis magnet = x-axis robot)
cube_FOVSize = [75 75 75]; %mm, side length x/y/z
% You should use an odd multiple of the step size here to gain the max
% benefit from the 8Ch-Hallsensor

% Stepsize in mm
% Should be a common factor/or common multiple of the distances on the
% hallsensor to gain the max benefit out of the 8Ch 
% Distances Hallsensor: in x = 20mm, in y = 19mm, in z = 10mm
% only sensefull stepsizes: 5 | 10 | 20 | 40 mm
cube_StepSize = 5; %mm


%% 3) Generate FOV Points
[Points_FOV, Points_FOV_corners] = Function_TrajectoryFOV_GenerateCube(cube_FOVSize, cube_StepSize);


%% 4) Convert to Angles for Robot
alpha_save = [];
pos_save = [];
alpha_save_corner = [];
pos_save_corner = [];

% Normal points of FOV
for i=1:length(Points_FOV)
    xF = Points_FOV(i,:); % get point
    alpha_in = Function_Kinematics_ConvCoodinates2Alpha_HomogTrafo(xF'); % convert point
    alpha_save(:,end+1) = alpha_in; % save point
    pos_save(:,end+1) = xF';
end

% Corner (Extrema) points of FOV
for i=1:length(Points_FOV_corners)
    xF = Points_FOV_corners(i,:);
    alpha_in = Function_Kinematics_ConvCoodinates2Alpha_HomogTrafo(xF');

    alpha_save_corner(:,end+1) = alpha_in;

    x_t = -xF(1);
    y_t = xF(3);
    z_t = xF(2);
    pos_save_corner(:,end+1) = [x_t;y_t;z_t];
end

save_file.alpha = alpha_save;
save_file.pos = pos_save;
save_file.alpha_corner = alpha_save_corner;
save_file.pos_corner = pos_save_corner;
save_file.FOV.size = cube_FOVSize;
save_file.FOV.Style = FOV_Style;
save_file.FOV.stepsize = cube_StepSize;


%% 5) Save calculated Angels 
save(save_name,'save_file');


%% 6) Plots

% Calculate Points, which should be plotted out of the calculated angles at every step
Points_Kinematik = [];
for i=1:length(alpha_save(1,:))
    alpha_temp = alpha_save(:,i);
    [t_kin_temp,~] = Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha_temp);
  
    Points_Kinematik = [Points_Kinematik; t_kin_temp'];

end

Points_Kinematik_corner = [];
for i=1:length(alpha_save_corner(1,:))
    alpha_temp = alpha_save_corner(:,i);
    [t_kin_temp,~]  = Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha_temp);
  
    Points_Kinematik_corner = [Points_Kinematik_corner; t_kin_temp'];

end



% Plot Section

% Plot 1)  Draw all points of FOV-target & the achieved by kinematics
if plot_flag==true
    figure;
    tiledlayout(1,2);
    ax = nexttile;
    hold on
    grid(ax,'on')
    plot3(Points_FOV(:,1), Points_FOV(:,2),Points_FOV(:,3),'-o');
    plot3(Points_Kinematik(:,1),Points_Kinematik(:,2),Points_Kinematik(:,3),'+');
   
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view([-30,37.5]);
    %axis equal;
    title(ax,'FOV Points');
    legend(ax,'FOV Goal','set Point');
   
 
    ax = nexttile;
    hold on
    grid(ax,'on')    ;
    hold on;
    
    plot3(Points_FOV_corners(:,1), Points_FOV_corners(:,2),Points_FOV_corners(:,3),'-o');
    plot3(Points_Kinematik_corner(:,1),Points_Kinematik_corner(:,2),Points_Kinematik_corner(:,3),'+');
    legend(ax,'FOV Goal','set Point');
   
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view([-30,37.5]);
    axis equal;
    title('FOV Corner Points');

end

alpha_Robot=[];


% Plot 2) Simulation of Trajectory
fig_Conv = figure;

for i=1:length(alpha_save(1,:))
    alpha_temp = alpha_save(:,i);
    [~,~]=Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha_temp,true,fig_Conv,['Simulated Trajectory, Step: ' num2str(i) ' of ' num2str(length(alpha_save(1,:)))]);
    drawnow;
    pause(0.1);
end



