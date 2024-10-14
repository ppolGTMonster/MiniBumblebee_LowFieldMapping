function [Bx_opt,By_opt,Bz_opt,std_out] = function_optimizer_getBopt(alpha,Bx,Brx,Tx,By,Bry,Ty,Bz,Brz,Tz,Title)
% Function for calculating the calibrated magnetic field from the optimizer result 
% (analogous to Function_PostProcessing_getB, except that all 3 axes are calculated simultaneously). 
% As well as calculating the standard deviation of the difference to the magnetic field of the calibration coil
% 
% Parameters:
%   * alpha: Parameter of the calibration
%   * Bx: Vector, raw measurement values of the sensor in x direction
%   * Brx: Vector, magnetic field of the calibration coil at the same time
%   * Tx: Vector: temperature of the sensor
%   * ... analogous for y and z axes
%   * Title: String, name of the plot
% Return values:
%   * Bx_opt: calibrated result in x-direction
%   * By_opt: -''- in y
%   * Bz_opt: -''- in z
%   * std_out: STD of the difference between the coil magnetic field and the raw value or calibrated value


global Temp_Ref;
global show_plots;


%% Step 1) Calculate the actual measured field using the calibration and the raw data 

% Offset (Earth  Field/Stray field machines etc)
OS = alpha(5,1);

% X
a_real = alpha(1,1);
a_TC1 = alpha(1,2);
a_TC12 = alpha(1,3);
a_TC2 = alpha(1,4);
dT = Tx-Temp_Ref;
Bx_opt = Function_PostProcessing_getB(Bx,a_real,a_TC1,a_TC12,a_TC2,OS,dT);

% Y
a_real = alpha(2,1);
a_TC1 = alpha(2,2);
a_TC12 = alpha(2,3);
a_TC2 = alpha(2,4);
dT = Ty-Temp_Ref;
By_opt = Function_PostProcessing_getB(By,a_real,a_TC1,a_TC12,a_TC2,OS,dT);

% Z
a_real = alpha(3,1);
a_TC1 = alpha(3,2);
a_TC12 = alpha(3,3);
a_TC2 = alpha(3,4);
dT = Tz-Temp_Ref;
Bz_opt = Function_PostProcessing_getB(Bz,a_real,a_TC1,a_TC12,a_TC2,OS,dT);


%% Step 2) Calculate STD
std_out = [std(Brx-Bx) std(Brx-Bx_opt);...
    std(Bry-By) std(Bry-By_opt);...
    std(Brz-Bz) std(Brz-Bz_opt)];




%% Step 3) Plots
if show_plots == true
    figure;
    fig = tiledlayout(3,3);
    fig.Parent.WindowState='maximized';
    plothelp(Brx,Bx,Bx_opt,Tx,'Measurement Direction X');
    plothelp(Bry,By,By_opt,Ty,'Measurement Direction Y');
    plothelp(Brz,Bz,Bz_opt,Tz,'Measurement Direction Z');
    title(fig,Title);
end


    function [] = plothelp(Breal,Bsens,B_real_opt,Temp,titleFig)

        B_error = function_PostProcessing_getBerror(Breal);
        x_temp = 1:1:length(Breal);

        nexttile(fig);
        yyaxis left;
        errorbar(x_temp,Breal,B_error);
        hold on;
        plot(Bsens);
        yyaxis right;
        plot(Breal-Bsens);
        title(['Measurement: '  titleFig]);
        legend('real','sens','real-sens');

        nexttile(fig);
        yyaxis left;
        plot(B_real_opt);
        hold on;
        errorbar(x_temp,Breal,B_error);
        yyaxis right;
        plot(Breal-B_real_opt);
        plot(B_error,'-.r');        % Error from Calibration SetUp
        plot(-1*B_error,'-.r');     % Error from Calibration SetUp
        title('Result Calibration');
        legend('Opimization','real','real-opt','MeasError');

        nexttile(fig);
        plot(Temp);
        title('Temperature');


    end



end