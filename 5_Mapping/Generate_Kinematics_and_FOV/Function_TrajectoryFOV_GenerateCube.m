function [Points_FOV, Points_FOV_corners] = Function_TrajectoryFOV_GenerateCube(FOVsize, FOV_stepsize)
% Function to calculate all the FOV points
% Care is given on speed up measurement, therefore the 8 individual sensors
% are used and double positions skipped
%
% Parameter
%   * FOVsize: Vector, 3 sidelengths of the FOV cube
%   * FOV_stepsize: selected side length

global SetFile_Mechanical;

% sensefull stepsizes with the used hall sensor
FOV_stepsize_list = [5,10,20,40];

%% 1) Check if the selected stepsize is a "sensefull" one, if not -> used the
% closest one
if ismember(FOV_stepsize,FOV_stepsize_list) == false
    [minVal,index] = min(abs(FOV_stepsize_list-FOV_stepsize));
    FOV_stepsize = FOV_stepsize_list(index);
    disp(['  FOV Stepsize was changed to = ' num2str(FOV_stepsize) ' mm']);
else
    disp(['  FOV Stepsize was set to = ' num2str(FOV_stepsize) ' mm']);
end

%% 2) CHeck if the FOV sidelength is a multiple of the stepsize now, if not change it slightly
% set X
if mod(FOVsize(1),FOV_stepsize)~=0
    FOVsize(1) = floor(FOVsize(1)/FOV_stepsize) * FOV_stepsize;
end
% Set y
if mod(FOVsize(2),FOV_stepsize)~=0
    FOVsize(2) = floor(FOVsize(2)/FOV_stepsize) * FOV_stepsize;
end
% Set z
if mod(FOVsize(3),FOV_stepsize)~=0
    FOVsize(3) = floor(FOVsize(3)/FOV_stepsize) * FOV_stepsize;
end

x = -FOVsize(1)/2:FOV_stepsize:FOVsize(1)/2;
y = -FOVsize(2)/2:FOV_stepsize:FOVsize(2)/2;
z = -FOVsize(3)/2:FOV_stepsize:FOVsize(3)/2;

%% 3) Change sign
% Reason: In the used setup (Robot arm is elongated in negative x-direction
% + position of the hall sensor) + Base CS
x = +1*x;
y = -1*y;  
z = -1*z; 

%% 4) Delete all points, that are already sensed through other sensor (FOV is calculated for Sensor H1)
Hallsens_dx = SetFile_Mechanical.HallSensor.deltaX *1000; %mm
Hallsens_dy = SetFile_Mechanical.HallSensor.deltaY *1000;
Hallsens_dz = SetFile_Mechanical.HallSensor.deltaZ *1000;

% x-axis
for i = 2:length(x)
    for i_2 = 1:i-1
        if abs(x(i)-x(i_2)) == Hallsens_dx 
            x(i) = NaN;
        end
    end
end

%y-axis
for i = 2:length(y)
    for i_2 = 1:i-1
        if abs(y(i)-y(i_2)) == Hallsens_dy
            y(i) = NaN;
        end
    end
end

%z-axis
for i = 2:length(z)
    for i_2 = 1:i-1
        if abs(z(i)-z(i_2)) == Hallsens_dz
            z(i) = NaN;
        end
    end
end


x(isnan(x)) = [];
y(isnan(y)) = [];
z(isnan(z)) = [];

% Convert from mm to Meter
x=x*1e-3;
y=y*1e-3;
z=z*1e-3;

%% 5) Change reference system from FOV center to Robot reference system
% Coordinate Systems:
% (openCV) x/y/z: Reference Centre FOV (Center Magnet) 

% 5.1) Add Offset from Hallsensor (Translation)
% Robot is controling the axial position (front screw Sensor Head in real)
tvec_H1 = -1*SetFile_Mechanical.HallSensor.t_P5_To_H1;
x = x + tvec_H1(1);
y = y + tvec_H1(2);
z = z + tvec_H1(3);

% 5.2) FOV CoorSys -> Base (Refrence) CoorSys (Translation tvec)
tvec_ORB = SetFile_Mechanical.Env.t_ORB_To_MagnetFOV;
x_ref = x+tvec_ORB(1);
y_ref = y+tvec_ORB(2);
z_ref = z+tvec_ORB(3);

% 5.3) Base CoorSys -> Robot CoorSys (but axis oriented like in Base
% CoorSys) -> here just translation (tvec)
tvecRobB = -1*SetFile_Mechanical.Env.t_ORB_To_RobotBase;
x_ref = x_ref+tvecRobB(1);
y_ref = y_ref+tvecRobB(2);
z_ref = z_ref+tvecRobB(3);

% 5.4) Transform CoorSys into Robot CoorSys
% Quick & dirty 
x_rob = -x_ref;
y_rob = z_ref;
z_rob = y_ref;

%% 6) Create Mesh
% Could be somehow done with meshgrid too, i think
Points_FOV = [];
count_y_odd = mod(length(y_rob),2); % 1 because odd number of points

for i_x = 1:length(x_rob)

    for i_y = 1:length(y_rob)

        % Goal: Move alternating up & down (minimal path)
        if count_y_odd ==1  
            if mod(i_x+i_y,2)==0 % Move from bottom to top (up)
                slct_z_pth_down = true;
            else % Move from top to bottom (down)
                slct_z_pth_down = false;
            end
        else
            if mod(i_y,2)==0 % Move from bottom to top (up)
                slct_z_pth_down = false;
            else % Move from top to bottom (down)
                slct_z_pth_down = true;
            end
        end

        if slct_z_pth_down
            for i_z =length(z_rob):-1:1
                z_rob_set = z_rob(i_z);
                Points_FOV(end+1,:) = [x_rob(i_x), y_rob(i_y), z_rob_set];
            end

        else
            for i_z = 1:length(z_rob)
                z_rob_set = z_rob(i_z);
                Points_FOV(end+1,:) = [x_rob(i_x), y_rob(i_y), z_rob_set];
            end
        end
    end
end


%% 7) Find Extremas (to check Collision of the FOV)
x_ext = [min(x_rob) max(x_rob)];
y_ext = [min(y_rob) max(y_rob)];
z_ext = [min(z_rob) max(z_rob)];

plane_low = [x_ext(1),y_ext(1),z_ext(1);
             x_ext(1),y_ext(2),z_ext(1);
             x_ext(2),y_ext(2),z_ext(1);
             x_ext(2),y_ext(1),z_ext(1)];
plane_up = [ x_ext(1),y_ext(1),z_ext(2);
             x_ext(1),y_ext(2),z_ext(2);
             x_ext(2),y_ext(2),z_ext(2);
             x_ext(2),y_ext(1),z_ext(2)];


Points_FOV_corners = [plane_low; plane_up];

disp(['  generated Points = ' num2str(length(Points_FOV)) ]);



end