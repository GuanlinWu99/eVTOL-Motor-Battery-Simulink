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
                       'thr1_trim_vtol',{},'rot_spd_trim_vtol',{});

lin_model   =   struct('A_fw',{},'B_fw',{},'C_fw',{},'D_fw',{},...
                       'A_LON_fw',{},'B_LON_fw',{},'C_LON_fw',{},'D_LON_fw',{},...
                       'A_LAT_fw',{},'B_LAT_fw',{},'C_LAT_fw',{},'D_LAT_fw',{});

%.. trim batch
for idx1 = 1:size(trimspd,2)
    for idx2 = 1:size(trimalt,2)

        EAS_trim    =   trimspd(idx1);
        H_trim      =   trimalt(idx2);

        trim_validity   =   true;

        vtol_trim_linearization_analysis;

        [b,a] = ss2tf(A_vtol_lon,B_vtol_lon,C_vtol_lon,D_vtol_lon);
        
        [minreal(tf(b(3,:),a),1e-4);
         minreal(tf(b(4,:),a),1e-4)]

        % if trim_validity
        %     trim_data(idx1,idx2).alt_trim       =   H_trim;
        %     trim_data(idx1,idx2).eas_trim       =   EAS_trim;
        %     trim_data(idx1,idx2).tas_trim       =   x_trim(1);
        %     trim_data(idx1,idx2).aoa_trim       =   x_trim(2);
        %     trim_data(idx1,idx2).aos_trim       =   x_trim(3);
        %     trim_data(idx1,idx2).roll_trim      =   x_trim(4);
        %     trim_data(idx1,idx2).pitch_trim     =   x_trim(5);
        %     trim_data(idx1,idx2).heading_trim   =   x_trim(6);
        %     trim_data(idx1,idx2).flight_path_trim   =   gamma_trim;
        %     trim_data(idx1,idx2).thr_trim       =   u_trim(1);
        %     trim_data(idx1,idx2).rot_spd_trim   =   u_trim(1)*uavParam.motor.RPMMAX;
        %     trim_data(idx1,idx2).elevator_trim  =   u_trim(2);
        %     trim_data(idx1,idx2).aileron_trim   =   u_trim(3);
        %     trim_data(idx1,idx2).rudder_trim    =   u_trim(4);
        % 
        % end

    end
end