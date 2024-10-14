function out = function_optimizer_loss(alpha,Breal_x,Bsens_x, dT_x,...
    Breal_y,Bsens_y, dT_y,...
    Breal_z,Bsens_z, dT_z,mask_weight)
% Function to calculate the calibration parameters using an optimization (fmincon)
% 
% Parameters
%   * alpha: Calubration Parameter
%   * Breal_x: vector, magnetic field of calibration coil when measuring X linearity
%   * Bsens_x: vector, magnetic field of one sensor when measuring X linearity
%   * Temp_x: vector, temperature of sensor when measuring X linearity
%   * ... Analogous for direction Y/Z
%   * mask_weight: matrix, mask for deleting the test dataset
% Return:
%   * out: loss value (squared error)


global opt_weight_MeasCal;
global opt_weight_TempCal;


%% Step 0) Split Calibration Parameter in alpha for the indvidual axis & calculate error
OS_env = alpha(end); %%Offset

% x
a_real = alpha(1);
a_TC1 = alpha(2);
a_TC12 = alpha(3);
a_TC2 = alpha(4);
rhs = Function_PostProcessing_getB(Bsens_x,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_x); % calculate Bx from sensor
val_x = Breal_x-rhs; % Error in x

% y
a_real = alpha(5);
a_TC1 =  alpha(6);
a_TC12 = alpha(7);
a_TC2 =  alpha(8);
rhs = Function_PostProcessing_getB(Bsens_y,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_y); % calculate By from sensor
val_y = Breal_y-rhs; % Error in y

% z
a_real = alpha(9);
a_TC1 =  alpha(10);
a_TC12 = alpha(11);
a_TC2 =  alpha(12);
rhs = Function_PostProcessing_getB(Bsens_z,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_z); % calculate Bz from sensor
val_z = Breal_z-rhs; % Error in z


%% Step 1) Mask data again & calc overall loss

m_x = mask_weight.x;
m_y = mask_weight.y;
m_z = mask_weight.z;

m_x(m_x == 1) = opt_weight_MeasCal;
m_x(m_x == 0) = opt_weight_TempCal;
m_y(m_y == 1) = opt_weight_MeasCal;
m_y(m_y == 0) = opt_weight_TempCal;
m_z(m_z == 1) = opt_weight_MeasCal;
m_z(m_z == 0) = opt_weight_TempCal;


val_x = val_x.^2 .*m_x;
val_y = val_y.^2 .*m_y;
val_z = val_z.^2 .*m_z;


out = sum(val_x,'all') + sum(val_y,'all') + sum(val_z.^2,'all'); %Overall loss




end