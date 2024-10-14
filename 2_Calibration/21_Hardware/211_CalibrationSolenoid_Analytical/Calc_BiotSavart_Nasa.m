function [Brho, Bz] = Calc_BiotSavart_Nasa(rho,z,z_pos,R)
%CALC_BIOTSAVART_NASA: Calculation of a loop in cylindrical coordinates 
% rho, z: Cylindrical coordinates in meters, can also be passed as a matrix in the form of a mesh grid
% z_pos: Position of the loop
% R: Radius of the coil in meters
%
%Used Source: Simpson J, Lane J, Immer C, Youngquist R (2001) Simple Analytic Expressions for the Magnetic Field of a Circular Current Loop. 
%                   https://ntrs.nasa.gov/citations/20010038494
%
% Calculation for a current of 1A: linear scaling possible
%
% Cartesian coordinate system convention:
%   coil lies in the xy-plane, z-axis is perpendicular to the center point.
%   cylindrical coordinates: z=z, xy-plane = R
%   current flows counterclockwise (right hand rule - thumb along z) through the coil


% physical constants
mu0 = 4*pi*1e-7; %N/A², Magn Feldkonstante

% Constants
I = 1;  %A, Strom



% Determining the correct position of the loop: moving in the z-direction
z_loop = z-z_pos;


%Convert to paper notation
a = R; % Radius im Paper
C = mu0*I/pi;
alpha = sqrt(a.^2+ rho.^2 + z_loop.^2 - 2*a*rho);
beta = sqrt(a.^2+ rho.^2 + z_loop.^2 + 2*a*rho);
k = sqrt(1-(alpha.^2)./(beta.^2));
m = k.^2;
[K,E] = ellipke(m); % elliptical integral


% Start Analytical Calculation
Faktor_Brho = (C.*z_loop)./ (2*alpha.^2.*beta.*rho);
Faktor_Brho(isinf(Faktor_Brho)==true)=0;
Faktor_Brho(isnan(Faktor_Brho)==true)=0;
Faktor_Bz = C            ./ (2*alpha.^2.*beta);

Brho = Faktor_Brho .*   ((a.^2+rho.^2+z_loop.^2) .* E - alpha.^2.*K);
Bz = Faktor_Bz.*        ((a.^2-rho.^2-z_loop.^2) .* E + alpha.^2.*K);



end