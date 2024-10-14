clc;
clear all;
close all;

% GLobal Variables
global active_HallSens;
global show_plots;
global calc_V2;
global calc_V1;
global CalCoil_Conversion;
global Temp_Ref;

global Lin_error;
global Lin_error_xy;
global Lin_error_z;
global Temp_Coeff_gain_error;
global Temp_Coeff_gain_error2;
global TC_OS_error;
global OS_Earth;
global x0;
global ub;
global lb;
global ub_xy;
global lb_xy;
global ub_z;
global lb_z;
global opt_weight_MeasCal;
global opt_weight_TempCal;
global check_clear_OverTemp;





%% Limits of Optimizsation
Lin_error = 4.5/100; % Aus Datenblatt
Lin_error_xy = 2.5/100;
Lin_error_z = 4.9/100;



Temp_Coeff_gain_error = 0.24/100; %1/K Aus Datenblatt 
Temp_Coeff_gain_error2 = Temp_Coeff_gain_error/2;
% TC_OS_error = 0.07; %mT/°C, aus DB für A1342 Application Info
TC_OS_error = 50*0.07; %mT/°C, aus DB für A1342 Application Info
OS_Earth = 80 *1e-3; % µT aus den mT Messungen
%OS_Earth = 0.7; % Offsetfehler laut Datenblatt (max 24LSB) 

opt_weight_MeasCal = 2;
opt_weight_TempCal = 1;


% Variablen, die optimiert werden: [Gain TC1(Gain) TC2(OS)]x ... [Gain TC1(Gain) TC2(OS)]y OS_Env    
% Wobei OS_Env für Erdmagnetfeld + Streufeld 14er stehen (irgendwas um die 50µT. Je nach Einbausituation mal pos/mal negativ. Oder gar nicht
x0 = [1.0,0.001,0,40e-3];% Startwert
ub = [1 + Lin_error, Temp_Coeff_gain_error, Temp_Coeff_gain_error2 ,TC_OS_error]; %oberes Limit 
lb = [1 - Lin_error, 0,                    -Temp_Coeff_gain_error2,-TC_OS_error]; % Unteres Limit
ub_xy = [1 + Lin_error_xy, Temp_Coeff_gain_error, Temp_Coeff_gain_error2 ,TC_OS_error]; %oberes Limit 
lb_xy = [1 - Lin_error_xy, 0,                    -Temp_Coeff_gain_error2,-TC_OS_error]; % Unteres Limit
ub_z =  [1 + Lin_error_z,  Temp_Coeff_gain_error, Temp_Coeff_gain_error2 ,TC_OS_error]; %oberes Limit 
lb_z =  [1 - Lin_error_z,  0,                    -Temp_Coeff_gain_error2,-TC_OS_error]; % Unteres Limit



%% Select Dataset
% während der Kalibrierung konnten manche Hallsensoren nicht positioniert werden (wegen Platz und aus Zeitgründen)
% -> Wurden "mitgemessen" bei anderen
Dataset{1} = 'H1'; % Einstellung Kalibrierung Februar 20s4
Dataset{2} = 'H2';
Dataset{3} = 'H3';
Dataset{4} = 'H4';
Dataset{5} = 'H5';
Dataset{6} = 'H6';
Dataset{7} = 'H7';
Dataset{8} = 'H8';




%% SetUp
% 0) General
active_HallSens = 8;
averages_on_uC = 50; % How many averages are programmed inside the uC
averages_on_uC_goal = 1000; %How many averages on chip are planned in the end

show_plots = false; % true = show plots during post processing
calc_V1 = false;    % Version 1 der Berechnung/Optimierung
calc_V2 = false;    % Version 2 der Berechnung

check_clear_OverTemp = true;

result_offset = [];
result_offset_list_x=[];
result_offset_list_y=[];
result_offset_list_z=[];
RMS_Ret=[];
offset_list = linspace(0,6,31);   %%%%%%%%%%%%%%%%%%%%%% OFFSET LIST
% offset_list = linspace(3.5,5.5,11);   %%%%%%%%%%%%%%%%%%%%%% OFFSET LIST
alphaList_Offset=cell(size(offset_list));


std_overview = [];


for i_offset = 1:length(offset_list)
disp(['Step ' num2str(i_offset)]);

CalCoil_Conversion = 0.780737777544529 * (1-offset_list(i_offset)/100); % mT/A

% 1) Zero Offset
limit_std_temperature = 1; % Maximale Standardabweichung Temperatur bei Zero Offset Bestimmung

% 2) Temperature Dependence

% 3) Linearity
Temp_Ref = 25; % °C, Referenztemperatur (bei dieser Temperatur ist keine Temperaturkorrektur nötig, willkürlich auf Raumtemperatur festgesetzt)

Folder = 'Log_Folder';


%% Calibration File
CalFile.Zero_Offset = {};
CalFile.TempDependence = cell(active_HallSens,1);
CalFile.Linearity = cell(active_HallSens,1);

%% Step 0) Concetenate Data (multiple Linearitiy Data)
% Function_PostProcessing_ConcatenateData(Folder);



%% Step 1.1) get Zero-Offset

[CalFile] = Function_PostProcessing_AnalyzeZeroOffset(Folder,CalFile,limit_std_temperature);

%% Step 2: Use Optimizer

list_lin = {};
for i=1:active_HallSens

    dataset_name = Dataset{i};
    name = ['Linearity*' dataset_name '*CumulatedData.mat'];
    list_lin{end+1} = dir(fullfile(Folder,name));
end

list_temp_x = dir(fullfile(Folder,'Temperature_Dep*_x_*'));
list_temp_y = dir(fullfile(Folder,'Temperature_Dep*_y_*'));
list_temp_z = dir(fullfile(Folder,'Temperature_Dep*_z_*'));

alpha_temp=cell(8,1);
std_val=[];

for i=1:active_HallSens
%for i=1:1
    if isempty(list_lin{i})
        continue;
    end

%     CalFile_temp = CalFile.Zero_Offset{i};
    CalFile_temp = [0 0 0];

    Offset_x = CalFile_temp(1);
    Offset_y = CalFile_temp(2);
    Offset_z = CalFile_temp(3);
  


    for i_2 = 1:length(list_lin{i})
        file_name = [list_lin{i}(i_2).folder '\' list_lin{i}(i_2).name];
        load(file_name);
        LogFile = LogFile_Sum;

        if contains(file_name,'_x_')
            [B_sens_x,B_real_x,Temp_x] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_x,i);
            
            B_sens_x_T=[];
            B_real_x_T=[];
            Temp_x_T = [];
%             file_name = [list_temp_x(1).folder '\' list_temp_x(1).name];
%             load(file_name);
%             [B_sens_x_T,B_real_x_T,Temp_x_T] = function_optimizer_maskData(LogFile.HallSens.Bx,LogFile,Offset_x,i);
        elseif contains(file_name,'_y_')
            [B_sens_y,B_real_y,Temp_y] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_y,i);
            B_sens_y_T=[]; 
            B_real_y_T=[];
            Temp_y_T=[];
%             file_name = [list_temp_y(1).folder '\' list_temp_y(1).name];
%             load(file_name);
%             [B_sens_y_T,B_real_y_T,Temp_y_T] = function_optimizer_maskData(LogFile.HallSens.By,LogFile,Offset_y,i);
        elseif contains(file_name,'_z_')
            [B_sens_z,B_real_z,Temp_z] = function_optimizer_maskData(LogFile.HallSens.B,LogFile,Offset_z,i);
            B_sens_z_T=[];
            B_real_z_T=[];
            Temp_z_T=[];
%             file_name = [list_temp_z(1).folder '\' list_temp_z(1).name];
%             load(file_name);
%             [B_sens_z_T,B_real_z_T,Temp_z_T] = function_optimizer_maskData(LogFile.HallSens.Bz,LogFile,Offset_z,i);
        end
    end

    Title = ['H' num2str(i)];
    [std_ret,alpha] = function_postprocessing_Interval( B_sens_x, B_real_x, Temp_x,B_sens_x_T, B_real_x_T, Temp_x_T, ...
        B_sens_y, B_real_y, Temp_y,B_sens_y_T, B_real_y_T, Temp_y_T, ...
        B_sens_z, B_real_z, Temp_z,B_sens_z_T, B_real_z_T, Temp_z_T,Title);

    %% Speichern im Cal-File -> Nutze nur Version 3 (am sichersten)

    CalFile.Linearity{i} = alpha.Var3(1:3,1);
    CalFile.TempDependence{i} =  alpha.Var3(1:3,2:3);
    CalFile.SetUp.STD{i} = std_ret;

    alpha_temp{i} = alpha.Var3;


    %x-Wert
    slct_i=1;
    [RMS_Ret_x] = function_getRMSError(B_real_x,B_sens_x,alpha.Var3(slct_i,1),alpha.Var3(slct_i,2),alpha.Var3(slct_i,3),alpha.Var3(slct_i,4),alpha.Var3(5,1),Temp_x);
    %y-Wert
    slct_i=2;
    [RMS_Ret_y] = function_getRMSError(B_real_y,B_sens_y,alpha.Var3(slct_i,1),alpha.Var3(slct_i,2),alpha.Var3(slct_i,3),alpha.Var3(slct_i,4),alpha.Var3(5,1),Temp_y);
    %z-Wert
    slct_i=3;
    [RMS_Ret_z] = function_getRMSError(B_real_z,B_sens_z,alpha.Var3(slct_i,1),alpha.Var3(slct_i,2),alpha.Var3(slct_i,3),alpha.Var3(slct_i,4),alpha.Var3(5,1),Temp_z);

%     RMS_Ret = [RMS_Ret_x;RMS_Ret_y;RMS_Ret_z];
    RMS_Ret = [RMS_Ret,[RMS_Ret_x;RMS_Ret_y;RMS_Ret_z] ];

    std_val(end+1) = sum(std_ret.Var3_training(:,2));

end

std_overview(:,end+1) = std_val';

disp('-----------------------------------');
disp('alpha = ');
disp(alpha_temp{1});


CalFile.SetUp.Date = datetime;
CalFile.SetUp.MeasSetUp = LogFile.SetUp;
CalFile.SetUp.CountMeasPoints = [length(B_real_x)+length(B_real_x_T);length(B_real_y)+length(B_real_y_T);length(B_real_z)+length(B_real_z_T)];
CalFile.SetUp.DatasetMatch = Dataset;

name = datestr(now);
name = strrep(name,':','_');
name = strrep(name,' ','_');
name = ['Calibration_' LogFile.SetUp.HallSens_Type '_' name];

%save(name, 'CalFile');


%% Bewerte das Ergebnis nach alpha (wieviele halten die Sensitivität ein?
error_nonlin = 0;
error_ninlin_z = 0;
error_list_x=ones(8,1)*0;
error_list_y=ones(8,1)*0;
error_list_z=ones(8,1)*0;

alphaList_Offset{i_offset} = alpha_temp;

for i = 1:active_HallSens
    at = alpha_temp{i};

    x_lin = at(1,1);
    y_lin = at(2,1);
    z_lin = at(3,1);


    if abs(x_lin-1) < 99.5/100*Lin_error_xy
    else
        error_nonlin = error_nonlin+1;
        error_list_x(i)=i;
    end

    if abs(y_lin-1) < 99.5/100*Lin_error_xy
    else
        error_nonlin = error_nonlin+1;
        error_list_y(i)=i;
    end

    if abs(z_lin-1) < 99.5/100*Lin_error_z
    else
        error_ninlin_z = error_ninlin_z+1;
        error_list_z(i)=i;
    end


%     for i2=1:3
%         at_val = at(i2,1);
% 
% 
%         if abs(at_val-1) < 99.5/100*Lin_error
%                 %do nothing
%         else
%             %disp(['Sensor: ' num2str(i) ', achse: '  num2str(i2)])
%             error_nonlin=error_nonlin+1;
%             if i2 == 3
%                 error_ninlin_z = error_ninlin_z+1;
%             end
%         end
%     end
end

%disp([' ---  Result of limited real sensitivity: ' num2str(error_nonlin)]);

result_offset = [result_offset; offset_list(i_offset) error_nonlin error_ninlin_z];
result_offset_list_x = [result_offset_list_x error_list_x];
result_offset_list_y = [result_offset_list_y error_list_y];
result_offset_list_z = [result_offset_list_z error_list_z];





end
disp('Ergebnis der Optimierung: ');
disp(['offset   std sum']);
%disp([offset_list' ; std_overview'])

figure;
tiledlayout(1,8);
for i=1:8

    ax = nexttile;
    plot(offset_list,std_overview(i,:));
    title(ax,['Sensor ' num2str(i)]);
    [val,ind]  = min(std_overview(i,:));
    disp([num2str(val) '   ' num2str(ind) '    ' num2str(offset_list(ind))]);

end

figure;

plot(offset_list,sum(std_overview,1));
title('Summe Alles Std-Abweichungen');
xlabel('Angenommener Error (immer negativ)');


num_error = zeros(size(result_offset_list_x));
num_error(result_offset_list_x>0.1)=1;
num_error_x = sum(num_error,1);

num_error = zeros(size(result_offset_list_x));
num_error(result_offset_list_y>0.1)=1;
num_error_y = sum(num_error,1);

num_error = zeros(size(result_offset_list_x));
num_error(result_offset_list_z>0.1)=1;
num_error_z = sum(num_error,1);



figure;
tiledlayout(2,1);
ax = nexttile;
plot(offset_list,num_error_x,'-x');
hold on;
plot(offset_list,num_error_y,'-x');
plot(offset_list,num_error_z,'-x');
plot(offset_list,num_error_x+num_error_y+num_error_z,'-x');

yline(8*3);
yline(8*1,'--');
title(ax,'Summe der Anzahl der Sensoren, die in die Begrenzung gelaufen sind');
xlabel(ax,'Angenommener Error (immer negativ)');
ylim([0,28]);
xlim([offset_list(1) offset_list(end)+0.5]);
xline(offset_list,'--');
legend(ax,'X-Ax','Y-Ax','Z-Ax','Sum', 'Anzahl Gesamt Sensoren', 'Anzahl z-Sensoren');


ax =nexttile;
hold on;
steps_offset = offset_list(2)-offset_list(1);
offset_x = 0.2*steps_offset;
offset_y = 0.5*steps_offset;
offset_z = 0.7*steps_offset;

for i=1:8
    plot(offset_list+offset_x,result_offset_list_x(i,:),'o','MarkerSize',8,'Color','b','MarkerFaceColor','b');
    plot(offset_list+offset_y,result_offset_list_y(i,:),'o','MarkerSize',8,'Color','k','MarkerFaceColor','k');
    plot(offset_list+offset_z,result_offset_list_z(i,:),'o','MarkerSize',8,'Color','r','MarkerFaceColor','r');
    yline(i,'--');
end
ylim([0.1,8.8]);
xlim([offset_list(1) offset_list(end)+0.5]);
title(ax,'Kalibrierfehler (X=blau, Y=schwarz, Z=rot)');
ylabel(ax,'Sensor');
xline(offset_list,'--');



%% Analyse Lin-Faktor
Sens_x = [];
Sens_y = [];
Sens_z = [];



for i=1:length(alphaList_Offset)

    a_t = alphaList_Offset{i};

    x_temp =[];
    y_temp=[];
    z_temp=[];

    for ii=1:8

        a_t2 = a_t{ii};
        x_temp = [x_temp; a_t2(1,1)];
        y_temp = [y_temp; a_t2(2,1)];
        z_temp = [z_temp; a_t2(3,1)];
    end

    Sens_x = [Sens_x x_temp];
    Sens_y = [Sens_y y_temp];
    Sens_z = [Sens_z z_temp];


end

figure;
tiledlayout(3,1);
ax=nexttile;
plot(offset_list,(Sens_x-1)*100);
hold on;
yline(Lin_error_xy*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_xy*100);
legend('1','2','3','4','5','6','7','8');
title(ax,'X');
ax=nexttile;
plot(offset_list,(Sens_y-1)*100);
hold on;
yline(Lin_error_xy*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_xy*100);
title(ax,'Y');
ax=nexttile;
plot(offset_list,(Sens_z-1)*100);
hold on;
yline(Lin_error_z*[1 0 -1]*100,'--');
ylim([-1.2 1.2]*Lin_error_z*100);
title(ax,'Z');







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNKTIONEN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



















function [std_ret,alpha] = function_postprocessing_Interval(    B_sens_x, B_real_x, Temp_x,B_sens_x_T, B_real_x_T, Temp_x_T, ...
    B_sens_y, B_real_y, Temp_y,B_sens_y_T, B_real_y_T, Temp_y_T, ...
    B_sens_z, B_real_z, Temp_z,B_sens_z_T, B_real_z_T, Temp_z_T, PlotTitle)

global show_plots;
global calc_V2;
global calc_V1;
global check_clear_OverTemp;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fmincon nach PDsumacumFG %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if calc_V1

% Variante 1: Nur die Strommessung selbst
[alpha_opt_v1] = function_optimizer(B_real_x,B_sens_x, Temp_x, ...
    B_real_y,B_sens_y, Temp_y,...
    B_real_z,B_sens_z, Temp_z);

[Bx_opt,By_opt,Bz_opt,std_v1] = function_optimizer_getBopt(alpha_opt_v1,   B_sens_x,B_real_x,Temp_x,...
    B_sens_y,B_real_y,Temp_y, ...
    B_sens_z,B_real_z,Temp_z,[PlotTitle ' - Variante 1']);
else

std_v1 = zeros(3,2);
alpha_opt_v1 = zeros(1,13);


end


%% Variante 2:
% zusätzliches Einfügen der Temperaturdaten


if check_clear_OverTemp
    T_limit = 40; %Grad Celsius

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

if calc_V2

[alpha_opt_v2] = function_optimizer(B_real_x,B_sens_x, Temp_x, ...
    B_real_y,B_sens_y, Temp_y,...
    B_real_z,B_sens_z, Temp_z);

[Bx_opt,By_opt,Bz_opt,std_v2] = function_optimizer_getBopt(alpha_opt_v2,   B_sens_x,B_real_x,Temp_x,...
    B_sens_y,B_real_y,Temp_y, ...
    B_sens_z,B_real_z,Temp_z,[PlotTitle ' - Variante 2']);
else

 std_v2 = zeros(3,2);
 alpha_opt_v2 = zeros(1,13);
end

%% Variante 3:
% Jedes dritte Element aus Variante 3 herausnehmen: Damit gegencheck der Optimierung




n = 3; %
l = max([length(B_real_x),length(B_real_y),length(B_real_z)]);
length_mask = l+(n-mod(l,n));
mask = ones(1,length_mask);
for i=3:3:length_mask
    mask(i)=0;
end

mask_weight_X = ones(size(B_real_x));
mask_weight_X(end-length(B_real_x_T):end) = 0;
mask_weight_Y = ones(size(B_real_y));
mask_weight_Y(end-length(B_real_y_T):end) = 0;
mask_weight_Z = ones(size(B_real_z));
mask_weight_Z(end-length(B_real_z_T):end) = 0;

% Check Daten
Br_x_ch  = B_real_x(~mask(1:length(B_real_x))==1);
Br_y_ch  = B_real_y(~mask(1:length(B_real_y))==1);
Br_z_ch  = B_real_z(~mask(1:length(B_real_z))==1);

Bs_x_ch  = B_sens_x(~mask(1:length(B_sens_x))==1);
Bs_y_ch  = B_sens_y(~mask(1:length(B_sens_y))==1);
Bs_z_ch  = B_sens_z(~mask(1:length(B_sens_z))==1);

T_x_ch  = Temp_x(~mask(1:length(Temp_x))==1);
T_y_ch  = Temp_y(~mask(1:length(Temp_y))==1);
T_z_ch  = Temp_z(~mask(1:length(Temp_z))==1);

% Training Daten

Br_x = B_real_x(mask(1:length(B_real_x))==1);
Br_y = B_real_y(mask(1:length(B_real_y))==1);
Br_z = B_real_z(mask(1:length(B_real_z))==1);

Bs_x = B_sens_x(mask(1:length(B_sens_x))==1);
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


% Optimierung
[alpha_opt_v3] = function_optimizer(Br_x,Bs_x, T_x, ...
    Br_y,Bs_y, T_y, ...
    Br_z,Bs_z, T_z,mask_weight);

[~,~,~,std_v3_orig] = function_optimizer_getBopt(alpha_opt_v3,...
    Bs_x,Br_x,T_x,...
    Bs_y,Br_y,T_y, ...
    Bs_z,Br_z,T_z,[PlotTitle ' - Variante 3 - Originalteile']);

[~,~,~,std_v3_check] = function_optimizer_getBopt(alpha_opt_v3,...
    Bs_x_ch,Br_x_ch,T_x_ch,...
    Bs_y_ch,Br_y_ch,T_y_ch, ...
    Bs_z_ch,Br_z_ch,T_z_ch,[PlotTitle ' - Variante 3 - Neues Modell']);

% Übersicht Standardabweichung

if show_plots == true
    figure;
    fig = tiledlayout(1,3);
    %fig.Parent.WindowState='maximized';
    title_array = ['x';'y';'z'];
    xlabels = {'Lin Data','+ Temp Data','Originalteile','Neues Modell'};
    xlabels_loc = [1 2 3 4];
    title(fig, [PlotTitle ' - STD']);

    for i=1:3
        ax = nexttile;

        data_orig = [std_v1(i,1) std_v2(i,1) std_v3_check(i,1) std_v3_orig(i,1)];
        data_fit = [std_v1(i,2) std_v2(i,2) std_v3_check(i,2) std_v3_orig(i,2)];
        %yyaxis left;
        plot(data_orig,'-x');
        %yyaxis right;
        hold on;
        plot(data_fit,'-o');
        ax.XTick = xlabels_loc;
        ax.XTickLabel = xlabels;
        legend('Experiment','Fit');
        title(ax,['STD in ' title_array(i)]);

    end
    saveas(fig,['Plots/' PlotTitle ' - STD.png']);
end

std_ret.Var1 =  std_v1;
std_ret.Var2 =  std_v2;
std_ret.Var3_training =  std_v3_orig;
std_ret.Var3_proof = std_v3_check;


alpha.Var1 = alpha_opt_v1;
alpha.Var2 = alpha_opt_v2;
alpha.Var3 = alpha_opt_v3;


end

function [alpha_opt_comb] = function_optimizer(Breal_x,Bsens_x, Temp_x, ...
    Breal_y,Bsens_y, Temp_y,...
    Breal_z,Bsens_z, Temp_z,mask_weight)
global Temp_Ref;
global Lin_error;
global Temp_Coeff_gain_error;
global Temp_Coeff_gain_error2;
global TC_OS_error;
global OS_Earth;
global x0;
global ub;
global lb;
global ub_xy;
global lb_xy;
global ub_z;
global lb_z;


con_val = 1e-20;

%Breal = B_real_x;
%Bsens = B_sens_x;
dT_x = Temp_x -  Temp_Ref;
dT_y =  Temp_y -  Temp_Ref;
dT_z =  Temp_z -  Temp_Ref;
%OS = Offset_x;

x0_set = [x0 ,x0,x0,0];
ub_set = [ub_xy,ub_xy,ub_z,    OS_Earth];
lb_set = [lb_xy, lb_xy, lb_z, -OS_Earth];
%lb_set = [lb, lb, lb, OS_Earth/2];


options = optimoptions("fmincon",...
    'Algorithm','interior-point',...
    "Display","iter", ...
    "PlotFcn","optimplotfval", ...
    "MaxIterations",500, ...
    "MaxFunctionEvaluations", 5000, ...
    "ConstraintTolerance",con_val, ...
    "OptimalityTolerance",con_val, ...
    "StepTolerance",con_val);

objfun = @(x) function_optimizer_loss(x, Breal_x,Bsens_x, dT_x,...
    Breal_y,Bsens_y, dT_y,...
    Breal_z,Bsens_z, dT_z,mask_weight);

ceq = @(alpha) []; % Equality - Constraints -> gibts nicht
c_result = @(alpha)function_optimizer_const(alpha);
nonlinfcn = @(alpha) deal( c_result(alpha), ceq(alpha));



alpha_opt = fmincon(objfun,x0_set,[],[],[],[],lb_set,ub_set,nonlinfcn,options);


alpha_opt_comb= reshape(alpha_opt(1:12),[4,3])';
alpha_opt_comb(5,1) = alpha_opt(end);




end

function [c1,c2,c3] = function_optimizer_const(alpha)
    
    %x
    atc11 = abs(alpha(2));
    atc12 = abs(alpha(3));
    %atc12<atc11 -> atc12-atc11 < 0
    c1 = atc12-atc11;

    %y
    atc11 =  abs(alpha(6));
    atc12 =  abs(alpha(7));
    c2 = atc12-atc11;

    %z
    atc11 = abs(alpha(10));
    atc12 = abs(alpha(11));
    c3 = atc12-atc11;

% 
    c1 = -1;
    c2 = -1; 
    c3 = -1;
end

function out = function_optimizer_loss(alpha,Breal_x,Bsens_x, dT_x,...
    Breal_y,Bsens_y, dT_y,...
    Breal_z,Bsens_z, dT_z,mask_weight)

global opt_weight_MeasCal;
global opt_weight_TempCal;


OS_env = alpha(end);

% x
a_real = alpha(1);
a_TC1 = alpha(2);
a_TC12 = alpha(3);
a_TC2 = alpha(4);

%rhs = a_real .* (1+a_TC1.*dT_x) .* Bsens_x + a_TC2.*dT_x + OS_env;
rhs = function_getB(Bsens_x,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_x);

val_x = Breal_x-rhs;

% y
a_real = alpha(5);
a_TC1 =  alpha(6);
a_TC12 = alpha(7);
a_TC2 =  alpha(8);
rhs = function_getB(Bsens_y,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_y);

val_y = Breal_y-rhs;

% z
a_real = alpha(9);
a_TC1 =  alpha(10);
a_TC12 = alpha(11);
a_TC2 =  alpha(12);
rhs = function_getB(Bsens_z,a_real,a_TC1,a_TC12,a_TC2,OS_env,dT_z);


val_z = Breal_z-rhs;

switch nargin

    case 10
        out = sum(val_x.^2,'all') + sum(val_y.^2,'all') + sum(val_z.^2,'all');
    case 11
      
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


        out = sum(val_x,'all') + sum(val_y,'all') + sum(val_z.^2,'all');


        t= 4;


    otherwise

        out = NaN;


end






end

function [Br] = function_getB(Bs,ar,aTC1,aTC12,aTC2,OS,dT)

    gain_error_T = aTC1.*dT + aTC12.*(dT.^2);

    Br = (ar+gain_error_T) .* Bs + aTC2.*dT + OS;
end

function [Bx_opt,By_opt,Bz_opt,std_out] = function_optimizer_getBopt(alpha,Bx,Brx,Tx,By,Bry,Ty,Bz,Brz,Tz,Title)
global Temp_Ref;
global show_plots;

OS = alpha(5,1);

a_real = alpha(1,1);
a_TC1 = alpha(1,2);
a_TC12 = alpha(1,3);
a_TC2 = alpha(1,4);
dT = Tx-Temp_Ref;
Bx_opt = function_getB(Bx,a_real,a_TC1,a_TC12,a_TC2,OS,dT);

a_real = alpha(2,1);
a_TC1 = alpha(2,2);
a_TC12 = alpha(2,3);
a_TC2 = alpha(2,4);
dT = Ty-Temp_Ref;
By_opt = function_getB(By,a_real,a_TC1,a_TC12,a_TC2,OS,dT);


a_real = alpha(3,1);
a_TC1 = alpha(3,2);
a_TC12 = alpha(3,3);
a_TC2 = alpha(3,4);
dT = Tz-Temp_Ref;
Bz_opt = function_getB(Bz,a_real,a_TC1,a_TC12,a_TC2,OS,dT);


std_out = [std(Brx-Bx) std(Brx-Bx_opt);...
    std(Bry-By) std(Bry-By_opt);...
    std(Brz-Bz) std(Brz-Bz_opt)];




%Plots
if show_plots == true

    figure;
    fig = tiledlayout(3,3);
    fig.Parent.WindowState='maximized';
    plothelp(Brx,Bx,Bx_opt,Tx,'Messung Dir X');
    plothelp(Bry,By,By_opt,Ty,'Messung Dir Y');
    plothelp(Brz,Bz,Bz_opt,Tz,'Messung Dir Z');
    title(fig,Title);
    saveas(fig,['Plots/' Title '.png']);
end


    function [] = plothelp(Breal,Bsens,B_real_opt,Temp,titleFig)

        B_error = function_optimizer_getBerror(Breal);
        x_temp = 1:1:length(Breal);

        nexttile(fig);
        yyaxis left;
        errorbar(x_temp,Breal,B_error);
        %plot(Breal);
        hold on;
        plot(Bsens);
        yyaxis right;
        plot(Breal-Bsens);
        title(['Messung: '  titleFig]);
        legend('real','sens','real-sens');

        nexttile(fig);
        yyaxis left;
        plot(B_real_opt);
        hold on;
        errorbar(x_temp,Breal,B_error);
        yyaxis right;
        plot(Breal-B_real_opt);
        plot(B_error,'-.r'); % Error aus Messung
        plot(-1*B_error,'-.r'); % Error aus Messung
        title('Ergebnis Optimierung');
        legend('Optimierung','real','real-opt','MeasError');

        nexttile(fig);
        plot(Temp);
        title('Temperature');


    end



end

function [B_error] = function_optimizer_getBerror(B)
global CalCoil_Conversion;
multi.Tolerance = 0.12/100;
multi.Resolution = [1e-6, 10e-6];
multi.Offset = [8,5];
multi.Range = [100e-3,1];


shunt.Tolerance = 0.1/100;
shunt.Ohm = 20e-3;

I = B/CalCoil_Conversion;
U = abs(I*shunt.Ohm);
B_error = zeros(size(U));


for i=1:length(U)
    if U(i)<100e-3
        index = 1;
    else
        index = 2;
    end

    Multi_Res = multi.Resolution(index);
    Multi_OS  = multi.Offset(index) * Multi_Res;

    Multi_GainError = multi.Tolerance  * U(i);
    Multi_ResError = U(i)/10000; % Slow Modus

    Multi_Error = Multi_GainError + Multi_OS + Multi_ResError;

    Shunt_Error =  shunt.Ohm*shunt.Tolerance;

    Bmin = CalCoil_Conversion*(U(i)+Multi_Error)/(shunt.Ohm-Shunt_Error);
    Bmax = CalCoil_Conversion*(U(i)-Multi_Error)/(shunt.Ohm+Shunt_Error);
    B_error(i) = abs(Bmax-Bmin);

end


end

function [Bs,Br,T] = function_optimizer_maskData(B_in,LogFile,Offset,i_sensor)

B = cell2mat(B_in);
Bs = B(i_sensor,:) + Offset; % Offset Comp

try
    Br = Function_PostProcessing_GetBField(LogFile,'Lin');

catch
    %Temperaturvariante
    Br = Function_PostProcessing_GetBField(LogFile,'Temp');
end


T = cell2mat(LogFile.HallSens.Temp);
T = T(i_sensor,:);


mask = (isnan(Br) == true).* (isnan(Bs) == true);
Bs = Bs(mask==0);
Br = Br(mask==0);
T = T(mask==0);


end

function [RMS_Ret] = function_getRMSError(Br,Bs,ar,aTC1,aTC12,aTC2,OS,Temp)
global Temp_Ref;


Br_opt = function_getB(Bs,ar,aTC1,aTC12,aTC2,OS,Temp-Temp_Ref);

RMS_Ret=0;

for i=1:length(Bs)
    
    RMS_Ret = RMS_Ret + (Br(i)-Br_opt(i)).^2;
end

RMS_Ret = sqrt(RMS_Ret/length(Bs));



end