%=========================================================================%
% fixed-wing stability/control augmentation system                        %
%-------------------------------------------------------------------------%
% author(s): minhyun                                                      %
% description: computing trim condition of fixed-wing mode and            %
%              linearize model to enable auto-tuner                       %
%=========================================================================%

%.. clear workspace, command window and close fiugres
clear all;
clc;
close all;

%.. load constants
const       =   load_const();

%.. set up vtol dynamics parameters
uavParams   =   load_vtol_dynamics_7000lb(const);

%.. trim speed
trimspd     =   0*const.kts2mps;                                            % [m/s] trim speed
trimalt     =   100*const.ft2m;                                             % [m] trim altitude

%.. model specific data and parameters
%.. disable wind
Wind        =   0;

%.. initialize landing gear model
load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, ...
%                  'slidingFriction', 0.8, 'rollingFriction', 0.2, ...
%                  'gLimit', 100);

%.. load bus interfaces for plant
load_digital_twin_interface();

trim_data   =   struct('alt_trim_vtol',{},'eas_trim_vtol',{},'tas_trim_vtol',{},...
                       'roll_trim_vtol',{},'pitch_trim_vtol',{},'heading_trim_vtol',{},...
                       'thr_trim_vtol',{},'rot_spd_trim_vtol',{});

lin_model   =   struct('A_vtol',{},'B_vtol',{},'C_vtol',{},'D_vtol',{},...
                       'A_LON_vtol',{},'B_LON_vtol',{},'C_LON_vtol',{},'D_LON_vtol',{},...
                       'A_LAT_vtol',{},'B_LAT_vtol',{},'C_LAT_vtol',{},'D_LAT_vtol',{});

%.. trim batch
for idx1 = 1:size(trimspd,2)
    for idx2 = 1:size(trimalt,2)

        EAS_trim    =   trimspd(idx1);
        H_trim      =   trimalt(idx2);

        trim_validity   =   true;

        vtol_trim_linearization_analysis;

        if trim_validity

            trim_data(idx1,idx2).validity           =   1;
            trim_data(idx1,idx2).alt_trim_vtol      =   H_trim;
            trim_data(idx1,idx2).eas_trim_vtol      =   EAS_trim;
            trim_data(idx1,idx2).tas_trim_vtol      =   x_trim(1);
            trim_data(idx1,idx2).roll_trim_vtol     =   x_trim(4);
            trim_data(idx1,idx2).pitch_trim_vtol    =   x_trim(5);
            trim_data(idx1,idx2).heading_trim_vtol  =   x_trim(6);
            trim_data(idx1,idx2).thr_trim           =   [u_trim(1), u_trim(2), u_trim(3), u_trim(4)];
            trim_data(idx1,idx2).rot_spd_trim       =   [u_trim(1), u_trim(2), u_trim(3), u_trim(4)]*uavParams.motor.RPMMAX;

            lin_model(idx1,idx2).A_vtol             =   A_vtol;
            lin_model(idx1,idx2).B_vtol             =   B_vtol;
            lin_model(idx1,idx2).C_vtol             =   C_vtol;
            lin_model(idx1,idx2).D_vtol             =   D_vtol;

            lin_model(idx1,idx2).A_vtol_lon         =   A_vtol_lon;
            lin_model(idx1,idx2).B_vtol_lon         =   B_vtol_lon;
            lin_model(idx1,idx2).C_vtol_lon         =   C_vtol_lon;
            lin_model(idx1,idx2).D_vtol_lon         =   D_vtol_lon;

            %.. longitudinal SCAS design
            [A_vtol_lon_0, B_vtol_lon_0, C_vtol_lon_0, D_vtol_lon_0] 	=   linmod('vtol_lon_no_ctrl');
            [num_lon_0, den_lon_0]      =   ss2tf(A_vtol_lon_0, B_vtol_lon_0, C_vtol_lon_0, D_vtol_lon_0, 1);
            dthetadpsq0     =   tf(num_lon_0(1,:), den_lon_0);              % [d(pseudo-q) -> d(theta)]
            dqdpsq0         =   tf(num_lon_0(2,:), den_lon_0);              % [d(pseudo-q) -> d(q)]

            Kp_q    =   0.3;
            Ki_q    =   0;
            Kd_q    =   0;

            
            
            [A_vtol_lon_1, B_vtol_lon_1, C_vtol_lon_1, D_vtol_lon_1] 	=   linmod('vtol_lon_inner_q');
            [num_lon_1, den_lon_1]      =   ss2tf(A_vtol_lon_1, B_vtol_lon_1, C_vtol_lon_1, D_vtol_lon_1, 1);
            dthetadq1     =   tf(num_lon_1(1,:), den_lon_1);              % [d(q) -> d(theta)]
            dqdq1         =   tf(num_lon_1(2,:), den_lon_1);               % [d(q) -> d(q)]


        end

    end
end