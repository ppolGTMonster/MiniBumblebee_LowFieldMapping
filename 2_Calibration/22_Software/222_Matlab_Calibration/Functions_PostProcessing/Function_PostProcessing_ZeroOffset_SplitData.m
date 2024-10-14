function [H_temp] = Function_PostProcessing_ZeroOffset_SplitData(LogFile)
% Function to break down the log data so that it can be used afterwards
% Parameters:
% * Logfile: struct, file in which the files are located


global active_HallSens;

H_temp= cell(active_HallSens,1);

for i_2 = 1:length(LogFile.HallSens.Bx)

    for i_3 = 1:active_HallSens
        H_temp{i_3}(:,i_2) = [LogFile.HallSens.Bx{i_2}(i_3); LogFile.HallSens.By{i_2}(i_3); LogFile.HallSens.Bz{i_2}(i_3); LogFile.HallSens.Temp{i_2}(i_3)];
    end
end

% Delete NaNColumns (Transmit did not worked)
for i_2=1:active_HallSens
    H_temp{i_2} = H_temp{i_2}(:,all(~isnan(H_temp{i_2})));
end

end