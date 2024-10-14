function [CalFile] = Function_PostProcessing_AnalyzeZeroOffset(Folder,CalFile,limit_std_temperature)
% Function for calculating the zero offset. 
% To do this, the probe was in the shielding chamber and recorded approx. 1000 measuring points.
% Parameters:
% - Folder: String, location where the log files are stored
% - CalFile: Struct, in which the calculated calibration values are stored
% - limit_std_temperature: double, STD limit from which measuring points are removed in case of excess temperature
% Return:
% CalFile: Struct, see above


global active_HallSens;
global show_plots;

list_zero = dir(fullfile(Folder,'Zero*.mat'));
list_temperature = dir(fullfile(Folder,'Temperature_Dep*.mat'));


H_zero_data     = cell(active_HallSens,1);
H_zero_offset   = cell(active_HallSens,1);


%% Step 0) Load Data, For-Loop in case the user created multiple Zero-Offset Files

for i=1:length(list_zero)
    file_name = [list_zero(i).folder '\' list_zero(i).name];
    load(file_name);

    H_zero_data_temp = Function_PostProcessing_ZeroOffset_SplitData(LogFile);

    if i==1
        H_zero_data = H_zero_data_temp;
    else % Concetenate Data
        for i_2 = 1:active_HallSens
            H_zero_data{i_2} = [ H_zero_data{i_2} H_zero_data_temp{i_2}];

        end

    end
end


%% Step 1) Data Analysis + QQ-Plot

% Check if Temperature is stable enough
std_temperature = zeros(active_HallSens,1); % Standard Deviation of Temperature
for i_2=1:active_HallSens
    std_temperature(i_2) = std(H_zero_data{i_2}(end,:));
end

if any(std_temperature>limit_std_temperature)
    % Temperature Differences are too high -> Repeat the Measurement in a temperature stable environment
    CalFile.Zero_Offset = NaN;
else
    % Continue to analyze Data

    for i_2 = 1:active_HallSens     

        H_zero_offset{i_2}(1) = -1 * mean(H_zero_data{i_2}(1,:)); % x
        H_zero_offset{i_2}(2) = -1 * mean(H_zero_data{i_2}(2,:)); % y
        H_zero_offset{i_2}(3) = -1 * mean(H_zero_data{i_2}(3,:)); % z  

    end

    CalFile.Zero_Offset = H_zero_offset;

    % plot normal distribution (qqplot) of sampled data
    if show_plots == true

        figure;
        fig = tiledlayout(2,4);
        fig.Parent.WindowState='maximized';
        for i_2=1:active_HallSens
            ax = nexttile;

            qqplot(ax,H_zero_data{i_2}(1,:)); %x-Axis
            hold on;
            qqplot(ax,H_zero_data{i_2}(2,:)); %y-Axis
            qqplot(ax,H_zero_data{i_2}(3,:)); %z-Axis

            ylabel('Quantiles of Offset Measurements');
            title(['QQ Plot of Sensor H' num2str(i_2)]);
            %legend('x-Axis','y-Axis','z-Axis');
        end
    end
end

end