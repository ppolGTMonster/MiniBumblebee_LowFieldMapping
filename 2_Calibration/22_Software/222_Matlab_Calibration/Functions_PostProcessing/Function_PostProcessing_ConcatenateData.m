function [] = Function_PostProcessing_ConcatenateData(Folder)
% Function to combine the individual measurements (Linearity Measurement only) into a single file
% Parameters:
% - Folder: String, location where the log files are stored
% Return:
% - x


global active_HallSens;


%% Step 0) Create a list of all the files in the folder that can be used
list_dir = ["x", "y", "z"];
list_linearity = {};

for i_1 = 1:active_HallSens

    for i_2 = 1:length(list_dir)

        val = "H" + num2str(i_1) + "_" + list_dir(i_2);
        search_name = ['Linearity*' convertStringsToChars(val) '*.mat'];

        list_linearity{end+1} = dir(fullfile(Folder,search_name));
       
    end
end


%% Step 1) Sweep through the individual files


for i=1:length(list_linearity)

    list_temp = list_linearity{i};
    if isempty(list_temp) %  The stored data does not exist -> cancel
        continue;
    end


    B_value = {};
    Temp = {};
    Voltage = {};
    Current = {};

    % Joining the individual measurement files
    for i_2 = 1:length(list_temp)

        file_name = [list_temp(i_2).folder '\' list_temp(i_2).name];
        load(file_name);

        if contains(file_name,'Cumulated') % Script can be executed repeatedly -> combination that have already been saved should be ignored
            continue;
        end

        if  strcmp(LogFile.SetUp.CalibratedDirection,"x")
            B_value = [B_value LogFile.HallSens.Bx];
        elseif strcmp(LogFile.SetUp.CalibratedDirection,"y")
            B_value = [B_value LogFile.HallSens.By];
        elseif strcmp(LogFile.SetUp.CalibratedDirection,"z")
            B_value = [B_value LogFile.HallSens.Bz];
        end

        Temp = [Temp LogFile.HallSens.Temp];
        Voltage = [Voltage LogFile.Multi.Voltage];
        Current = [Current LogFile.Multi.Current];

    end


    % Building the log file to be stored
    LogFile_Sum.SetUp = LogFile.SetUp;

    LogFile_Sum.Multi.Voltage = Voltage;
    LogFile_Sum.Multi.Current = Current;

    LogFile_Sum.HallSens.Temp = Temp;
    LogFile_Sum.HallSens.B = B_value;

    file_name_new = [extractBefore(file_name, '_Date') '_CumulatedData.mat'];
    save(file_name_new, 'LogFile_Sum'); % Save the new combination


end



end