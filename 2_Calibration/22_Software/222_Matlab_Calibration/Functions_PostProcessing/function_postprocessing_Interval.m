function [std_ret,alpha] = function_postprocessing_Interval(...
    B_sens_x, B_real_x, Temp_x, B_sens_x_T,B_real_x_T,Temp_x_T,...
    B_sens_y, B_real_y, Temp_y, B_sens_y_T,B_real_y_T,Temp_y_T,...
    B_sens_z, B_real_z, Temp_z, B_sens_z_T,B_real_z_T,Temp_z_T,PlotTitle)
% Function to calibrate a sensor. To do this, the “Function_Optimizer” function is called up here, which does the main work. 
% Before that, however, the data is prepared and masked
% Parameters:
%   * B_sens_x: Vector, (training data) raw data sensor in x-axis
%   * B_real_x: Vector, (training data) magnetic field of the calibration cylinder in x-axis
%   * Temp_x: Vector, (training data) temperature of the sensors
%   * B_sens_x_T: Vector, (test data) analogous to above
%   * B_real_x_T: Vector, (test data) analogous to above
%   * Temp_x_T: Vector, (test data) analogous to above
%   * ... analogous for axis y/z
%   * PlotTitle: String, name of the sensor, used for plot at the end
% 
% Return values:
%   * std_ret: Struct, calculated STD for the training and test data set
%   * alpha: Struct, calculated calibration parameters


global check_clear_OverTemp;
global T_limit;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fmincon nach PhDsumacum FelixG %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 0) Combine the linearity measurement data set with the variable temperature data set
if check_clear_OverTemp
    %Check if temperature is above T_Limit

    mask_T_x = ones(size(Temp_x));
    mask_T_x(Temp_x_T>T_limit) = 0;

    mask_T_y = ones(size(Temp_y));
    mask_T_y(Temp_y_T>T_limit) = 0;

    mask_T_z = ones(size(Temp_z));
    mask_T_z(Temp_z_T>T_limit) = 0;
    
    Temp_x_T(mask_T_x==0) = [];
    Temp_y_T(mask_T_y==0) = [];
    Temp_z_T(mask_T_z==0) = [];

    B_sens_x_T(mask_T_x==0) = [];
    B_sens_y_T(mask_T_y==0) = [];
    B_sens_z_T(mask_T_z==0) = [];
    
    B_real_x_T(mask_T_x==0) = [];
    B_real_y_T(mask_T_y==0) = [];
    B_real_z_T(mask_T_z==0) = [];
    
end
B_real_x = [B_real_x B_real_x_T];
B_real_y = [B_real_y B_real_y_T];
B_real_z = [B_real_z B_real_z_T];

B_sens_x = [B_sens_x B_sens_x_T];
B_sens_y = [B_sens_y B_sens_y_T];
B_sens_z = [B_sens_z B_sens_z_T];

Temp_x = [Temp_x Temp_x_T];
Temp_y = [Temp_y Temp_y_T];
Temp_z = [Temp_z Temp_z_T];



%% Step 1) Data Preperation
% The goal is to divide the entire data set into a training data set and a test data set.
% The division is done according to the variable “n”. 
% Every n-th element is taken from the overall set and added toe test data set. 
% The remaining data is the training data set.
% Dividing the Data Sets with masks
n = 3; 

l = max([length(B_real_x),length(B_real_y),length(B_real_z)]);
length_mask = l+(n-mod(l,n));
mask = ones(1,length_mask);
for i=3:3:length_mask
    mask(i)=0;
end
% Calculate Masks
mask_weight_X = ones(size(B_real_x));
mask_weight_X(end-length(B_real_x_T):end) = 0;
mask_weight_Y = ones(size(B_real_y));
mask_weight_Y(end-length(B_real_y_T):end) = 0;
mask_weight_Z = ones(size(B_real_z));
mask_weight_Z(end-length(B_real_z_T):end) = 0;


% Test Data Set ("Ch" for check)
Br_x_ch  = B_real_x(~mask(1:length(B_real_x))==1);
Br_y_ch  = B_real_y(~mask(1:length(B_real_y))==1);
Br_z_ch  = B_real_z(~mask(1:length(B_real_z))==1);

Bs_x_ch  = B_sens_x(~mask(1:length(B_sens_x))==1);
Bs_y_ch  = B_sens_y(~mask(1:length(B_sens_y))==1);
Bs_z_ch  = B_sens_z(~mask(1:length(B_sens_z))==1);

T_x_ch  = Temp_x(~mask(1:length(Temp_x))==1);
T_y_ch  = Temp_y(~mask(1:length(Temp_y))==1);
T_z_ch  = Temp_z(~mask(1:length(Temp_z))==1);

% Training Data Set
Br_x = B_real_x(mask(1:length(B_real_x))==1);
Br_y = B_real_y(mask(1:length(B_real_y))==1);
Br_z = B_real_z(mask(1:length(B_real_z))==1);

Bs_x = B_sens_x(mask(1:length(B_sens_x))==1);%% Step 2) Calculate Calibration Parameters
Bs_y = B_sens_y(mask(1:length(B_sens_y))==1);
Bs_z = B_sens_z(mask(1:length(B_sens_z))==1);

T_x = Temp_x(mask(1:length(Temp_x))==1);
T_y = Temp_y(mask(1:length(Temp_y))==1);
T_z = Temp_z(mask(1:length(Temp_z))==1);

mask_weight_X = mask_weight_X(mask(1:length(mask_weight_X))==1);
mask_weight_Y = mask_weight_Y(mask(1:length(mask_weight_Y))==1);
mask_weight_Z = mask_weight_Z(mask(1:length(mask_weight_Z))==1);

mask_weight.x = mask_weight_X;
mask_weight.y = mask_weight_Y;
mask_weight.z = mask_weight_Z;

clear mask_weight_X mask_weight_Y mask_weight_Z;


%% Step 2) Optimization / Calculate Calibration-Parameter 

% Optimization
[alpha_opt] = Function_Optimizer(Br_x,Bs_x, T_x, ...
    Br_y,Bs_y, T_y, ...
    Br_z,Bs_z, T_z,mask_weight);


%% Step 3) Validate the calculated Result 


[~,~,~,std_training] = function_optimizer_getBopt(alpha_opt,...
    Bs_x,Br_x,T_x,...
    Bs_y,Br_y,T_y, ...
    Bs_z,Br_z,T_z,[PlotTitle ' - Training Data Set']);

[~,~,~,std_test] = function_optimizer_getBopt(alpha_opt,...
    Bs_x_ch,Br_x_ch,T_x_ch,...
    Bs_y_ch,Br_y_ch,T_y_ch, ...
    Bs_z_ch,Br_z_ch,T_z_ch,[PlotTitle ' - Test Data Set']);



std_ret.TrainingSet =  std_training;
std_ret.TestSet = std_test;

alpha = alpha_opt;


end