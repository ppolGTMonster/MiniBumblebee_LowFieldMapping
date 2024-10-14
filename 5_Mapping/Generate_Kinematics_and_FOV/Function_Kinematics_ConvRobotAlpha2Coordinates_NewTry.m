function [t_ax4, t_ax3] = Function_Kinematics_ConvRobotAlpha2Coordinates_NewTry(alpha,showfig,figure_Obj,figure_title)
% Function to calculate the position (coordinate) of the beginning & end of the last element
% Used to calculate the horizontality of the sensor & help to plot the simulation of the moving roboter
% Parameter:
%  * alpha: Vector, Angles of Robot
%  * showfig: bool, true = show Figure
%  * figure_Obj: Figure Object
%  * figure_title: String, Title of Figure
% Return:
%  * t_ax4: Vector, Coordinate End
%  * t_ax3: Vector, Coordinate Start




global SetFile_Mechanical;

if nargin<2
    showfig=false;
end

%% Position vectors in the individual coordinate systems
l0_t = SetFile_Mechanical.Robot.lB * [0;0;1];  % Per Mechanik nur entlang z-Achse (in RobotCoorSys)
l1_t = SetFile_Mechanical.Robot.lT * [0;0;1];  
l2_t = SetFile_Mechanical.Robot.lS * [0;0;1];
l3_t = SetFile_Mechanical.Robot.lH * [0;0;1];  



%% Create transformation matrices
R_alpha0 = RotZ(alpha(1));
R_alpha1 = RotX(alpha(2));
R_alpha2 = RotX(alpha(3));
R_alpha3 = RotX(alpha(4));


t_ax0 = [0;0;0];
t_ax1 = [0;0;0] + R_alpha0*(l0_t);
t_ax2 = [0;0;0] + R_alpha0*(l0_t+R_alpha1*(l1_t));
t_ax3 = [0;0;0] + R_alpha0*(l0_t+R_alpha1*(l1_t+R_alpha2*(l2_t)));
t_ax4 = [0;0;0] + R_alpha0*(l0_t+R_alpha1*(l1_t+R_alpha2*(l2_t+R_alpha3*l3_t)));


t_ax = [t_ax0 t_ax1 t_ax2 t_ax3 t_ax4];



%% Plot Part

if showfig

if nargin>2
    figure(figure_Obj);
else
    figure;
end

tiledlayout(1,1);
ax = nexttile;
view([-30,37.5]);
plot3(t_ax(1,:),t_ax(2,:),t_ax(3,:),'-o');
grid(ax,'on');
xlabel('x');
ylabel('y');
zlabel('z');
ax.XDir = 'reverse';
xlim([-0.05 0.5]);
zlim([0 0.25]);
ylim([-0.1 0.1]);
% axis equal

if nargin>2
    title(figure_title);
end

end


end

%% Help Functions

function [R] = RotX(alpha)
R = [1      ,0          ,0;
     0      ,cos(alpha) ,-sin(alpha);
     0      , sin(alpha),cos(alpha)];
end

function [R] = RotZ(alpha)
    R = [cos(alpha)         ,-1*sin(alpha)          ,0;...
         sin(alpha)         ,cos(alpha)             ,0;... 
         0                  ,0                      ,1];
end
