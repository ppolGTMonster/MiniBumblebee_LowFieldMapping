function [] = Function_Hallsensor_CorrectData()
% Function to correct the measured magnetic fields by using the determined calibration coeff
% 
% Parameter:
%  * None
% Return
%  * None


global CalFile;
global HallData;
global active_Hallsens;
Temp_Ref=25; %°C

% Sweep through all Meas Points
for i=1:length(HallData.B_meas)
    
    Bsens = HallData.B_meas{i};

    Br = [];

    % Sweep through all channels
    for i_2 =1:active_Hallsens
        % Get Data Point
        Bsens_Hi = Bsens(:,i_2);
        T = Bsens_Hi(4);
        Bsens_Hi = Bsens_Hi(1:3);  

        % Get Calibration Parameter
        OS_Zero = CalFile.Zero_Offset{i_2};
        TempVal = CalFile.TempDependence{i_2};
        Lin = CalFile.Linearity{i_2};
        dT = T-Temp_Ref;

        Br_temp = [];
       
        % Sweep through all 3 axes
        for i_3=1:3

            % Get axis dependent cal parameter
            a_real = Lin(i_3);
            a_TC1 = TempVal(i_3,1);
            a_TC12 = TempVal(i_3,2);
            a_TC2 = TempVal(i_3,3);
            OS = OS_Zero(i_3);

            B = Bsens_Hi(i_3);

            % Use Calibration
            B_opt = help_function_correctB(B,a_real,a_TC1,a_TC12,a_TC2,OS,dT);
            Br_temp = [Br_temp;B_opt];
        end

        Br = [Br Br_temp];
    
    end

    HallData.B_correct{i} = Br;

end


end


%% Local Functions


function [Br] = help_function_correctB(Bs,ar,aTC1,aTC12,aTC2,OS,dT)
% Apply the determined calibration parameters to sensed value
    gain_error_T = aTC1.*dT + aTC12.*(dT.^2);
    Br = (ar+gain_error_T) .* (Bs+OS) + aTC2.*dT;
end



