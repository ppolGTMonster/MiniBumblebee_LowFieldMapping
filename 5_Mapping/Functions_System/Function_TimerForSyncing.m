function Timer = Function_TimerForSyncing(command,mode,Timer_in)
% Function to handle the needed Timers during the measurement
%
% Parameter:
%   * command: Start/Stopp as String... Does exactly that
%   * mode: Move/Meas as String... Timer for Moving, or Timer for measure one point
%   * Timer_in: Timer Value
% Return:
%   * Timer: Time Value when Timer was started

global SetFile_System;

if contains(mode,"Move")
    timer_limit = SetFile_System.WaitOnPos_Move;
elseif contains(mode,"Meas")
    timer_limit = SetFile_System.WaitOnPos_BSens;
end

if contains(command, "Start")

    Timer = tic;

elseif contains(command, "Stopp")

    disp(['    ' 'Timer: Wait until desired delay is reached']);

    time_old = toc(Timer_in);
    fprintf('\n    Timer: Elapsed Time [sec]=  ');
    while true

        time_delay = toc(Timer_in);
        if time_delay>=  timer_limit
            break;
        end

                
        % Show current time as status in command window
        % So the user knows when the procedure continues
        if time_delay-time_old >=1

            time_old = time_delay;

            time_print_old = time_delay;
        
            if time_print_old>=10
                for_stop = log10(time_print_old);
            else
                for_stop = log10(time_print_old-1);
            end

            for j=0:for_stop
                fprintf('\b'); % delete previous counter display
            end
            fprintf('%d',round(time_delay));
        end

    end
    Timer=0;
    disp(['    ' 'Timer: Finished']);



end

end
