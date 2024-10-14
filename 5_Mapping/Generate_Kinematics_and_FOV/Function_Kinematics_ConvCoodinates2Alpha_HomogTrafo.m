function alpha = Function_Kinematics_ConvCoodinates2Alpha_HomogTrafo(xF)
% Function to Convert Coordinates into Robot Angles (using fmincon)
% Parameter:
%  * xF: Vector, Target xyz-Point
% Return:
%  * alpha: Vector, calculated Angle



global SetFile_Kinematics;

%% Get Limits of Robot & Reorder them for Optimizer
% The Angles are given the way the robot control uses them -> reorder them
lim1  = SetFile_Kinematics.alpha_1_limit; %Base
lim2  = SetFile_Kinematics.alpha_2_limit; %Shoulder
lim3  = SetFile_Kinematics.alpha_3_limit; %Ellbow
lim4  = SetFile_Kinematics.alpha_4_limit; %Wrist

lb_R = [lim1(1) lim2(1) lim3(1) lim4(1)];
ub_R = [lim1(2) lim2(2) lim3(2) lim4(2)];

% Convert to kinematics
lb_M = Function_Robot_ConvAlphaRobot2Matlab(lb_R);
ub_M = Function_Robot_ConvAlphaRobot2Matlab(ub_R);

% Extract lb & ub
lims = [lb_M;ub_M];
lb = min(lims);
ub = max(lims);


%% Start Value
alpha0 =[-1.570796326794897; 0; -0.043633231299858; 0];

%% Nonlinear Constraints
ceq = @(alpha) []; % no Equality - Constraints
c_result = @(alpha)Function_Kinematics_Constraints_HomogTrafo(alpha); % Inequality



nonlinfcn = @(alpha) deal( c_result(alpha), ceq(alpha));

options = optimoptions("fmincon",...
    'Algorithm','interior-point',...
    "Display","iter", ...
    "PlotFcn","optimplotfval");


rng(26);
[alpha,fval,exitflag,output] = fmincon(@(alpha)Function_Kinematics_CostFunction_HomogTrafo(alpha,xF),alpha0,[],[],[],[],lb,ub,nonlinfcn);




end