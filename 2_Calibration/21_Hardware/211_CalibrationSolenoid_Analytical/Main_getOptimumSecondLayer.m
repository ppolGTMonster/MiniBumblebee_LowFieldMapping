%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
%       Script to optimize the second layer of the solenoid to get best homogeneity
%       
%       Used Source: Simpson J, Lane J, Immer C, Youngquist R (2001) Simple Analytic Expressions for the Magnetic Field of a Circular Current Loop. 
%                   https://ntrs.nasa.gov/citations/20010038494
% 
%       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
%       MIT LICENSED
%       Have fun guys!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



clear all;
close all;
clc;

% Physical constants
gyroProton = 42.577*1e6; % Hz/T, gyromagnetic ratio proton

% Geometry Core
l_Magnet = 452.4 *1e-3; %Length Magnet, Meter

r_Magnet = 80 *1e-3;  %Radius Magnet, Meter

%Second layer
%Used for brute force optimization:
%What is the maximum number of turns for the second layer?
%The entire field is calculated for each number and at the end the user can select the correct number of turns.
n_sec_turn = 40;
result_n = 1:1:n_sec_turn;
result_ppm = zeros(1,length(result_n));

% Flowing Current
I = 1;

% Wire
d_Cu = 1.56 *1e-3; %Meter, diameter of wire + thickness of coat
r_Cu = d_Cu/2; %Meter, radius of wire + thickness of coat


n_Cu_innen = l_Magnet/(d_Cu);
n_Cu = n_Cu_innen+2*n_sec_turn;
l_Cu = 2*pi*r_Magnet*n_Cu;


rho_Cu = 0.0178 ; %Ohm*mm2/m, specific resistance
R_Cu = rho_Cu * l_Cu/(pi*(r_Cu*1e3)^2); %Rough estimate of how high the resistance will be
P_loss = I*I*R_Cu;


for n_opt=1:length(result_n)
n_sec_turn = result_n(n_opt);


% Cylinder Coordinate system
stepsize_rho = 1e-3;    %Meter, Stepsize
stepsize_z = stepsize_rho;
FOV_Max_R = 30e-3;       %Meter, Offset entlang R
FOV_Max_l = 30e-3;       %Meter, Offset entlang Zylinder Höhe

rho = 0: stepsize_rho:FOV_Max_R;
z = -FOV_Max_l: stepsize_z:FOV_Max_l;
[coor_Rho,coor_Z] = meshgrid(rho,z); % Get all possible coordinates inside the cylinder

z_list = -l_Magnet/2:d_Cu:l_Magnet/2;

Brho = zeros(size(coor_Z)); %Superpositioned Field of all individual current loops
Bz=zeros(size(coor_Z));

for z_i=1:length(z_list) % Sweep through all copper loops

    z_pos = z_list(z_i);
    [Brho_temp, Bz_temp] = Calc_BiotSavart_Nasa(coor_Rho,coor_Z,z_pos,r_Magnet); % Get the whole magnetic field for one current loop

    Brho = Brho+I*Brho_temp;
    Bz = Bz+I*Bz_temp;

    if z_i <=n_sec_turn     % Add second layer (Start of Solenoid)
        r_sec_turn = r_Magnet+r_Cu*2;
        [Brho_temp, Bz_temp] = Calc_BiotSavart_Nasa(coor_Rho,coor_Z,z_pos,r_sec_turn);
        Brho = Brho+I*Brho_temp;
        Bz = Bz+I*Bz_temp;


    elseif (length(z_list)-z_i)<n_sec_turn % Add second layer (End of Solenoid)
        r_sec_turn = r_Magnet+r_Cu*2;
        [Brho_temp, Bz_temp] = Calc_BiotSavart_Nasa(coor_Rho,coor_Z,z_pos,r_sec_turn);
        Brho = Brho+I*Brho_temp;
        Bz = Bz+I*Bz_temp;
    end


end

% Conversion to mT
Brho = Brho*1e3;
Bz=Bz*1e3;
B0 = sqrt(Brho.^2 + Bz.^2);

%Calculate magnetic field strength and homogeneity in the FOV. 
%Mask only the FOV sphere and set everything around it to nan
FOV_Radius = 10 *1e-3; %Meter, FOV Radius

mask_FOV = zeros(size(coor_Rho));
mask_FOV(coor_Z.^2+coor_Rho.^2<FOV_Radius^2) = 1;
mask_FOV(mask_FOV==0)=NaN;


B_FOV = B0.*mask_FOV;

mean_mF = mean(mean(B_FOV,'omitnan'),'omitnan');
min_mF = min(min(B_FOV,[],'omitnan'),[],'omitnan');
max_mF = max(max(B_FOV,[],'omitnan'),[],'omitnan');
B0_val = mean_mF; %mT
delta_B0  = (max_mF-min_mF);  %mT
B0_ppm = delta_B0/B0_val*1e6;


Linewidth = gyroProton*delta_B0*1e-3;



result_ppm(n_opt) = B0_ppm;
end


%get Minimum
[val,in] =min(result_ppm);

figure;
plot(result_n,result_ppm);
hold on;
xline(in,'-',{['Min: ' num2str(val) 'ppm,win:' num2str(in)]});
xlabel('# of Windings');
ylabel('Homogenity (ppm) inside FOV');
title('Result Optimization');









