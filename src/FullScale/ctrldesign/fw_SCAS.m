% ========================================================================
% Fixed-wing stability/control augmentation system
% ------------------------------------------------------------------------
% Compute fixed-wing trim conditions over a speed/altitude grid and
% linearize the dynamics to enable controller designer
%
% Inputs:
%
% Outputs (in workspace):
%
% Notes:
% ========================================================================

%.. clear workspace, command window and close fiugres
clear all;
clc;
close all;

%.. load constants
const       =   load_const();

%.. set up vtol dynamics parameters
uavParams   =   load_vtol_dynamics_7000lb(const);

%.. trim speed
trimspd     =   [60 75 90 100]*const.kts2mps;                               % [m/s] trim speed
trimalt     =   [100]*const.ft2m;                                           % [m] trim altitude

%.. level flight conditions
gamma_trim      =   0.0*const.deg2rad;                                      %.. [rad] flight path angle for level-wing trim
turn_rate_trim  =   0.0*const.deg2rad;                                      %.. [rad/s] turning rate for level coordinated-turn trim
heading_trim    =   0.0*const.deg2rad;                                      %.. [rad] initial heading of aircraft
alpha_trim      =   15.0*const.deg2rad;                                     %.. [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

%.. flap condition
flap_trim       =   40.0*const.deg2rad;

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

trim_data_fw    =   struct('alt_trim',{},'eas_trim',{},'tas_trim',{},'aoa_trim',{},'aos_trim',{}, ...
                           'roll_trim',{},'pitch_trim',{},'heading_trim',{},'flight_path_trim',{}, ...
                           'fwd_thr_trim',{},'fwd_rot_spd_trim',{},'rwd_thr_trim',{},'rwd_rot_spd_trim',{}, ...
                           'elevator_trim',{},'aileron_trim',{},'rudder_trim',{},'tilt_trim',{});

lin_model_fw    =   struct('A',{},'B',{},'C',{},'D',{}, ...
                           'A_LON',{},'B_LON',{},'C_LON',{},'D_LON',{}, ...
                           'A_LAT',{},'B_LAT',{},'C_LAT',{},'D_LAT',{});

%.. trim batch
for idx1 = 1:size(trimspd,2)
    for idx2 = 1:size(trimalt,2)

        EAS_trim    =   trimspd(idx1);
        H_trim      =   trimalt(idx2);

        trim_validity   =   true;

        fw_trim_linearization_analysis;

        if trim_validity
            trim_data(idx1,idx2).alt_trim       =   H_trim;
            trim_data(idx1,idx2).eas_trim       =   EAS_trim;
            trim_data(idx1,idx2).tas_trim       =   x_trim(1);
            trim_data(idx1,idx2).aoa_trim       =   x_trim(2);
            trim_data(idx1,idx2).aos_trim       =   x_trim(3);
            trim_data(idx1,idx2).roll_trim      =   x_trim(4);
            trim_data(idx1,idx2).pitch_trim     =   x_trim(5);
            trim_data(idx1,idx2).heading_trim   =   x_trim(6);
            trim_data(idx1,idx2).flight_path_trim   =   gamma_trim;
            trim_data(idx1,idx2).thr_trim       =   u_trim(1);
            trim_data(idx1,idx2).rot_spd_trim   =   u_trim(1)*uavParams.motor.RPMMAX;
            trim_data(idx1,idx2).elevator_trim  =   u_trim(2);
            trim_data(idx1,idx2).aileron_trim   =   u_trim(3);
            trim_data(idx1,idx2).rudder_trim    =   u_trim(4);

        end

    end
end



plot_trimspd        =   trimspd*const.mps2kts;
plot_trimalt        =   trimalt*const.m2ft;
plot_thr_trim       =   zeros(size(plot_trimspd,1),size(plot_trimalt,2));
plot_aoa_trim       =   zeros(size(plot_trimspd,1),size(plot_trimalt,2));
plot_elevator_trim  =   zeros(size(plot_trimspd,1),size(plot_trimalt,2));

for idx1 = 1:size(trim_data,1)
    for idx2 = 1:size(trim_data,2)

        plot_thr_trim(idx1,idx2)        =   trim_data(idx1,idx2).thr_trim*100;
        plot_aoa_trim(idx1,idx2)        =   trim_data(idx1,idx2).aoa_trim*const.rad2deg;
        plot_elevator_trim(idx1,idx2)   =   trim_data(idx1,idx2).elevator_trim*const.rad2deg;

    end
end

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trimspd,plot_thr_trim,'LineWidth',1.5);
ylabel('Throttle (%)');
xlabel('EAS (kts)');
title('Throttle Trim Results')

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trimspd,plot_aoa_trim,'LineWidth',1.5);
ylabel('AoA (deg)');
xlabel('EAS (kts)');
title('AoA Trim Results')

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trimspd,plot_elevator_trim,'LineWidth',1.5);
ylabel('dElevator (deg)');
xlabel('EAS (kts)');
title('Elevator Trim Results')