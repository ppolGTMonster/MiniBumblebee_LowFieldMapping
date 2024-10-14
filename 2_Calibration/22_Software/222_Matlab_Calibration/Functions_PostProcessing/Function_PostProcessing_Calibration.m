function [CalFile, alpha_temp, std_ret_temp,CountMeasPoints,LogFile,CoilError] = ...
    Function_PostProcessing_Calibration(Folder, CalFile, limit_std_temperature, error_CoilConversion, Dataset, performCoilErrorOpt)
% Function for determining the optimal parameters for calibration of the 8Ch Hall sensors. 
% Each sensor is individually calibrated by running an optimization
% 
% Parameters:
%   * Folder: String, folder in which the measurement data is stored
%   * CalFile: Struct, result of the calibration (still empty here)
%   * limit_std_temperature: value, limit how much deviation the temperature may have during the offset measurement
%   * error_CoilConversion, value, error of the calibration coil 
%   * Dataset: String-Cell-Array, mapping of the Hall sensors (When was which measured)
%   * performCoilErrorOpt: Bool, 
%       * True = function in mode: Calculation of the error of the calibration coil 
%       * False = calibration of the Hall sensors
% 
% Return values:
%   * CalFile: Struct, result of the calibration
%   * alpha_temp: Cell-Array, calculated calibration parameters for all 8 sensors
%   * std_ret_temp: Cell-Array, calculated STD for all 8 sensors 
%   * CountMeasPoints: Vector, how many measurement values are available
%   * LogFile: Struct, an exemplary one (used for saving in the CalFile)
%   * CoilError: Struct, RMS/STD used in the analysis of the coil error



if performCoilErrorOpt == true
    CoilError_RMS_Ret=[];
    CoilError_std_val=[];
end


% Global Variables
global active_HallSens;
global CalCoil_Conversion;


%% Step 0) Concatenate Data (multiple Linearitiy Data due to multiple measurements)
Function_PostProcessing_ConcatenateData(Folder);

%% Step 1) get Zero-Offset
[CalFile] = Function_PostProcessing_AnalyzeZeroOffset(Folder,CalFile,limit_std_temperature);

%% Step 2) Calculate Calibration Parameters
% In the following, the nomenclature applies:
% B_real = magnetic field of the calibration coil
% B_sensor = magnetic field of the sensor

CalCoil_Conversion = 0.780737777544529 * (1+error_CoilConversion); % mT/A, Conversion Factor of the calibration Coil from Current to Magnetic Field

% 2.1) get the Filenames of all Linearity-Files, but cumulated Data only
list_lin = {};
for i=1:active_HallSens

    dataset_name = Dataset{i};
    name = ['Linearity*' dataset_name '*CumulatedData.mat'];
    list_lin{end+1} = dir(fullfile(Folder,name));
end

% 2.1) get the Filenames of all Temperature-Files
list_temp_x = dir(fullfile(Folder,'Temperature_Dep*_x_*'));
list_temp_y = dir(fullfile(Folder,'Temperature_Dep*_y_*'));
list_temp_z = dir(fullfile(Folder,'Temperature_Dep*_z_*'));

alpha_temp=cell(8,1);
std_ret_temp = cell(8,1);

% run through all active Sensors
for i=1:active_HallSens
    if isempty(list_lin{i})
        continue;
    end

    % 2.3) Readout calculated Offset
    CalFile_temp = CalFile.Zero_Offset{i};
    Offset_x = CalFile_temp(1);
    Offset_y = CalFile_temp(2);
    Offset_z = CalFile_temp(3);


    for i_2 = 1:length(list_lin{i})
        % 2.4) Load Linearity Data
        file_name = [list_lin{i}(i_2).folder '\' list_lin{i}(i_2).name];
        load(file_name);
        LogFile = LogFile_Sum;

        % 2.5) Start Masking Data, but for all directions independent
        if contains(file_name,'_x_')
            %Linearity
            [B_sens_x,B_real_x,Temp_x] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_x,i);
            %Temperature
            file_name = [list_temp_x(1).folder '\' list_temp_x(1).name];
            load(file_name);
            [B_sens_x_T,B_real_x_T,Temp_x_T] = function_optimizer_maskData(LogFile.HallSens.Bx,LogFile,Offset_x,i);
        elseif contains(file_name,'_y_')
            %Linearity
            [B_sens_y,B_real_y,Temp_y] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_y,i);
            %Temperature
            file_name = [list_temp_y(1).folder '\' list_temp_y(1).name];
            load(file_name);
            [B_sens_y_T,B_real_y_T,Temp_y_T] = function_optimizer_maskData(LogFile.HallSens.By,LogFile,Offset_y,i);
        elseif contains(file_name,'_z_')
            %Linearity
            [B_sens_z,B_real_z,Temp_z] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_z,i);
            %Temperature
            file_name = [list_temp_z(1).folder '\' list_temp_z(1).name];
            load(file_name);
            [B_sens_z_T,B_real_z_T,Temp_z_T] = function_optimizer_maskData(LogFile.HallSens.Bz,LogFile,Offset_z,i);
        end
    end

    Title = ['H' num2str(i)];

    % 2.6) Start calibration with an optimization for all 3 axes simultaneously, but independently for each sensor.
    [std_ret,alpha] = function_postprocessing_Interval(...
        B_sens_x, B_real_x, Temp_x, B_sens_x_T,B_real_x_T,Temp_x_T,...
        B_sens_y, B_real_y, Temp_y, B_sens_y_T,B_real_y_T,Temp_y_T,...
        B_sens_z, B_real_z, Temp_z,B_sens_z_T,B_real_z_T,Temp_z_T,Title);

    %2.7) Save data in Cal-File
    CalFile.Linearity{i} = alpha(1:3,1);
    CalFile.TempDependence{i} =  alpha(1:3,2:4);
    CalFile.SetUp.STD{i} = std_ret;

    alpha_temp{i} = alpha;
    std_ret_temp{i} = std_ret;



    % % % % % The following part is only used if the deviation of the calibration coil from reality is to be calculated. % % % % %
    if performCoilErrorOpt == true
        %x-Wert
        slct_i=1;
        [RMS_Ret_x] = function_CoilError_getRMSError(B_real_x,B_sens_x,alpha(slct_i,1),alpha(slct_i,2),alpha(slct_i,3),alpha(slct_i,4),alpha(5,1),Temp_x);
        %y-Wert
        slct_i=2;
        [RMS_Ret_y] = function_CoilError_getRMSError(B_real_y,B_sens_y,alpha(slct_i,1),alpha(slct_i,2),alpha(slct_i,3),alpha(slct_i,4),alpha(5,1),Temp_y);
        %z-Wert
        slct_i=3;
        [RMS_Ret_z] = function_CoilError_getRMSError(B_real_z,B_sens_z,alpha(slct_i,1),alpha(slct_i,2),alpha(slct_i,3),alpha(slct_i,4),alpha(5,1),Temp_z);

        CoilError_RMS_Ret = [CoilError_RMS_Ret, [RMS_Ret_x;RMS_Ret_y;RMS_Ret_z] ];
        CoilError_std_val(end+1) = sum(std_ret.TrainingSet(:,2));
    end
    % % % % %      % % % % %

end




if performCoilErrorOpt == true
    CoilError.RMS = CoilError_RMS_Ret;
    CoilError.std = CoilError_std_val;
else
    CoilError=[];
end



CountMeasPoints = [length(B_real_x)+length(B_real_x_T);length(B_real_y)+length(B_real_y_T);length(B_real_z)+length(B_real_z_T)];







end