function [RMS_Ret] = function_CoilError_getRMSError(Br,Bs,ar,aTC1,aTC12,aTC2,OS,Temp)
global Temp_Ref;


Br_opt = Function_PostProcessing_getB(Bs,ar,aTC1,aTC12,aTC2,OS,Temp-Temp_Ref);

RMS_Ret=0;

for i=1:length(Bs)
    
    RMS_Ret = RMS_Ret + (Br(i)-Br_opt(i)).^2;
end

RMS_Ret = sqrt(RMS_Ret/length(Bs));



end