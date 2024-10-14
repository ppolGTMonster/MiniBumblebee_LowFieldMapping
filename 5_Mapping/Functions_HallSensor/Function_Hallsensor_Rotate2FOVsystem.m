function [HallSens_rot,HallSens_pos] = Function_Hallsensor_Rotate2FOVsystem(B_Data,H_Ref_Hi_kin)
% Function to calculate the positions & rotations of all 8 Hallsensors inside the FOV
%
% Parameter:
%  * B_Data: Matrix, magnetic field of every sensor at one position
%  * H_Ref_Hi_kin: cell-array, homog Matrix of every sensor
% Return:
%  * HallSens_rot: 3x3-Matrix, Rotationmatrix of all Hallsensor
%  * HallSens_pos: Vector, position of all Hallsensors


global SetFile_Mechanical;
global active_Hallsens;

ConvMat = Function_HallSensor_getConvertMatrix_MountingPos(); % get Coordinate Transformation Matrix

%% 1) Transform Coordinate system of Hallsensor to Basis CoorSys (BCS)
B_conv = zeros(size(B_Data));
for i=1:active_Hallsens
    B_conv(:,i) = ConvMat*B_Data(:,i);
end

%% 2) Rotate Magnetvector
B_rot_kin = zeros(size(B_Data));

for i=1:active_Hallsens
    rotM_kin = H_Ref_Hi_kin{i}(1:3,1:3); % get Rotation Matrix (BCS -> Hall)
    B_rot_kin(:,i) = inv(rotM_kin) * B_conv(:,i); % Rotate Magnetic Field (Hall -> BCS)
end

%% get Translation relative to FOV center
tvec_FOV_Hi_kin = zeros(size(B_Data));
tvec_FOV_Hi_id3 = zeros(size(B_Data));

t_Ref_FOV = SetFile_Mechanical.Env.t_ORB_To_MagnetFOV; % Translation from Origin Base to Origin FOV

for i=1:active_Hallsens
    t_Ref_Hi_kin = H_Ref_Hi_kin{i}(1:3,end); % Translation Vectors

    tvec_FOV_Hi_kin(:,i) = t_Ref_Hi_kin - t_Ref_FOV;
end

% Combine Output Data

HallSens_rot = B_rot_kin;
HallSens_pos = tvec_FOV_Hi_kin;

end

