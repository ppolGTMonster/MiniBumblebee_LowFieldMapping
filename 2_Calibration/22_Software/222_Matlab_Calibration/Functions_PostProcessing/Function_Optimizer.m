function [alpha_opt_comb] = Function_Optimizer(Breal_x,Bsens_x, Temp_x, ...
    Breal_y,Bsens_y, Temp_y,...
    Breal_z,Bsens_z, Temp_z,mask_weight)
% Function to calculate the calibration parameters using an optimization (fmincon)
% 
% Parameters
%   * Breal_x: vector, magnetic field of calibration coil when measuring X linearity
%   * Bsens_x: vector, magnetic field of one sensor when measuring X linearity
%   * Temp_x: vector, temperature of sensor when measuring X linearity
%   * ... Analogous for direction Y/Z
%   * mask_weight: matrix, mask for deleting the test dataset
% Return:
%   * alpha_opt: calculated calibration parameters


global Temp_Ref;
global OS_Earth;
global x0;

global ub_xy;
global lb_xy;
global ub_z;
global lb_z;

% Calculate Temperature Difference to Reference
dT_x =  Temp_x -  Temp_Ref;
dT_y =  Temp_y -  Temp_Ref;
dT_z =  Temp_z -  Temp_Ref;

%% Step 0) Setting Up Optimization Parameter

% Constraint Tolerance (here set to minimum, so it runs thorugh all Iterations
con_val = 1e-20; 

% Lower & Upper Boundries
ub_set = [ub_xy,ub_xy,ub_z,    OS_Earth];
lb_set = [lb_xy, lb_xy, lb_z, -OS_Earth];

% Start values
x0_set = [x0 ,x0,x0,0];

% options
options = optimoptions("fmincon",...
    'Algorithm','interior-point',...
    "Display","iter", ...
    "PlotFcn","optimplotfval", ...
    "MaxIterations",500, ...
    "MaxFunctionEvaluations", 5000, ...
    "ConstraintTolerance",con_val, ...
    "OptimalityTolerance",con_val, ...
    "StepTolerance",con_val);

% Loss Function
objfun = @(x) function_optimizer_loss(x, Breal_x,Bsens_x, dT_x,...
    Breal_y,Bsens_y, dT_y,...
    Breal_z,Bsens_z, dT_z,mask_weight);

% Constraints
ceq = @(alpha) [];                                      % Equality - empty
c_result = @(alpha)function_optimizer_const(alpha);     % Inequality
nonlinfcn = @(alpha) deal( c_result(alpha), ceq(alpha));% Combination


%% Step 1) Start fmincon Optimization
alpha_opt = fmincon(objfun,x0_set,[],[],[],[],lb_set,ub_set,nonlinfcn,options);

%% Step 2) Reshape the optimum Solution
alpha_opt_comb= reshape(alpha_opt(1:12),[4,3])';
alpha_opt_comb(5,1) = alpha_opt(end);




end