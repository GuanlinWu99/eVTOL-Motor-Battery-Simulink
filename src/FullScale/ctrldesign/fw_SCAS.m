
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
trimspd     =   [65 75 90 100]*const.kts2mps;                               % [m/s] trim speed
trimalt     =   [10]*const.ft2m;                                          % [m] trim altitude

%.. level flight conditions
gamma_trim      =   0.0*const.deg2rad;                                      %.. [rad] flight path angle for level-wing trim
turn_rate_trim  =   0.0*const.deg2rad;                                      %.. [rad/s] turning rate for level coordinated-turn trim
heading_trim    =   0.0*const.deg2rad;                                      %.. [rad] initial heading of aircraft
alpha_trim      =   15.0*const.deg2rad;                                     %.. [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

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

trim_data   =   struct('alt_trim_fw',{},'eas_trim_fw',{},'tas_trim_fw',{},...
                       'aoa_trim_fw',{},'aos_trim_fw',{},...
                       'roll_trim_fw',{},'pitch_trim_fw',{},'heading_trim_fw',{},'flight_path_trim_fw',{},...
                       'thr_trim_fw',{},'rot_spd_trim_fw',{},'elevator_trim_fw',{},'aileron_trim_fw',{},'rudder_trim_fw',{});

lin_model   =   struct('A_fw',{},'B_fw',{},'C_fw',{},'D_fw',{},...
                       'A_LON_fw',{},'B_LON_fw',{},'C_LON_fw',{},'D_LON_fw',{},...
                       'A_LAT_fw',{},'B_LAT_fw',{},'C_LAT_fw',{},'D_LAT_fw',{});

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
            trim_data(idx1,idx2).rot_spd_trim   =   u_trim(1)*uavParam.motor.RPMMAX;
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