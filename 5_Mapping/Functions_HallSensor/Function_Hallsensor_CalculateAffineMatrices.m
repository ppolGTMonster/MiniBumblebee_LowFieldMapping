function [H_Ref_Hi_kin] = Function_Hallsensor_CalculateAffineMatrices(A_kin)
% Function to calculate the affine matrices of all 8 Hall sensors out of the already calculated
% affine Matrix A_kin, which is known for one point of the sensor head (front screw)
%
% Parameter:
%  * A_kin: Cell-Array of affine Matrices of every motion tracked point
%
% Return:
%  * H_Ref_Hi_kin: Cell-Array, all affine matrices of all 8 sensors for every position

global SetFile_Mechanical;
global active_Hallsens;


%% 1) Prepare all constants
% Definition:
% Ref = Reference System
% H1 = Origin Hall sensor1-CoorSys 
% H2 = Origin Hall sensor1-CoorSys | ... | 
% P5 = point at the front of the sensor, where the screw is
% Convention H_s1_s2 : H = homogenous matrix, s1 = Origin ("Basis")-CoorSys, s2 = Target CoorSys

H_P5_H1     = help_convT2Affin(SetFile_Mechanical.HallSensor.t_P5_To_H1); % screw (Point5) to H1
H_H1_Hi{1}  = eye(4);
H_H1_Hi{2}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H2); % H1 to H2
H_H1_Hi{3}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H3); % ...
H_H1_Hi{4}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H4);
H_H1_Hi{5}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H5);
H_H1_Hi{6}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H6);
H_H1_Hi{7}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H7);
H_H1_Hi{8}  = help_convT2Affin(SetFile_Mechanical.HallSensor.t_H1_To_H8);


%% 2) Start Conversion
H_Ref_Hi_kin = cell(length(A_kin),active_Hallsens);

for i=1:length(A_kin)
    H_Ref_P5_kin = A_kin{i};
    H_Ref_H1_kin = H_Ref_P5_kin*H_P5_H1;

    H_Ref_Hi_kin{i,1} = H_Ref_H1_kin;

    for i_2 = 1:active_Hallsens
        H_Ref_Hi_kin{i,i_2} = H_Ref_H1_kin * H_H1_Hi{i_2};
    end
end

end


%% Local Functions

function [tvec_affin] = help_convT2Affin(tvec)
% Convert Vector to Affine Matrix
    tv_size = size(tvec); % Check if Transpose needed
    if tv_size(1)==1
        tvec = tvec';
    end
    tvec_affin = [eye(3) tvec; 0 0 0 1];
end