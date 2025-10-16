% ========================================================================
% Backward transition scheduler
% ------------------------------------------------------------------------
% Compute fixed-wing trim conditions over a speed/altitude grid and
% linearize the dynamics to enable an auto-tuner workflow.
%
% Inputs:
%   trimspd (1xN) [kts]     : Equivalent Airspeed grid for trim (converted to m/s)
%   trimalt (1xM) [ft]      : Altitude grid for trim (converted to m)
%   trimpitch (1xN) [deg]   : Scheduled body pitch guess vs. speed
%   gamma_trim [deg]        : Flight-path angle for level/climb/descend trim (0 for level)
%   heading_trim [deg]      : Initial heading for trim
%   alpha_trim [deg]        : AoA constraint/initial guess (used by solver)
%   flap_trim [deg]         : Flap schedule for backward transition
%   Wind [bool]             : 0 disables wind model, 1 enables (if supported)
%
% Outputs (in workspace):
%   trim_data (NxM struct): one entry per (speed, altitude)
%       .alt_trim [m]
%       .eas_trim [m/s]
%       .tas_trim [m/s]
%       .aoa_trim [rad]
%       .aos_trim [rad]
%       .roll_trim           [rad]
%       .pitch_trim          [rad]
%       .heading_trim        [rad]
%       .flight_path_trim    [rad]   (= gamma_trim)
%     .thr_trim            [-]     (0–1)
%     .rot_spd_trim        [RPM]   (= thr_trim * uavParams.motor.RPMMAX)
%     .elevator_trim       [rad]
%     .aileron_trim        [rad]
%     .rudder_trim         [rad]
%
%   lin_model (struct arrays aligned with trim grid):
%     .A_fw, .B_fw, .C_fw, .D_fw              : full-state linear model (if assigned inside called routine)
%     .A_LON_fw, .B_LON_fw, .C_LON_fw, .D_LON_fw  : longitudinal linear model
%     .A_LAT_fw, .B_LAT_fw, .C_LAT_fw, .D_LAT_fw  : lateral-directional linear model
%
% Notes:
%   Ensure 'tr_trim_linearization_analysis' returns consistent x_trim/u_trim and sets 'trim_validity'.
%   If a trim case fails, the corresponding trim_data entry is left at default (zeros/unset).
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
trimspd     =   [60 50 40 30 20 10]*const.kts2mps;                          % [m/s] trim speed
trimalt     =   [100]*const.ft2m;                                           % [m] trim altitude
trimpitch   =   [30 30 20 15 10 5]*const.deg2rad;                           % [rad] trim pitch schedule (w/ trim speed)
trimrwdthr  =   [0.10 0.20 0.30 0.50 0.65 0.70];                            % [-] trim rearward throttle level
trimtilt    =   [1.00 0.90 0.50 0.50 0.10 0.10];

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

trim_data_tr    =   struct('alt_trim',{},'eas_trim',{},'tas_trim',{},'aoa_trim',{},'aos_trim',{}, ...
                           'roll_trim',{},'pitch_trim',{},'heading_trim',{},'flight_path_trim',{}, ...
                           'fwd_thr_trim',{},'fwd_rot_spd_trim',{},'rwd_thr_trim',{},'rwd_rot_spd_trim',{}, ...
                           'elevator_trim',{},'aileron_trim',{},'rudder_trim',{},'tilt_trim',{});

lin_model_tr    =   struct('A',{},'B',{},'C',{},'D',{}, ...
                           'A_LON',{},'B_LON',{},'C_LON',{},'D_LON',{}, ...
                           'A_LAT',{},'B_LAT',{},'C_LAT',{},'D_LAT',{});

%.. trim batch
for idx1 = 1:size(trimspd,2)
    for idx2 = 1:size(trimalt,2)

        %.. get trim condition
        EAS_trim        =   trimspd(idx1);
        H_trim          =   trimalt(idx2);
        pitch_trim      =   trimpitch(idx1);
        rwd_thr_trim    =   trimrwdthr(idx1);
        tilt_trim       =   trimtilt(idx1);

        %.. level flight condition
        gamma_trim      =   0.0*const.deg2rad;                              % [rad] flight path angle for level-wing trim
        turn_rate_trim  =   0.0*const.deg2rad;                              % [rad/s] turning rate for level coordinated-turn trim
        heading_trim    =   0.0*const.deg2rad;                              % [rad] initial heading of aircraft
        alpha_trim      =   trimpitch(idx1);                                % [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

        %.. flap condition
        flap_trim       =   40.0*const.deg2rad;                             % [rad] backward transition flap schedule

        %.. trim validity signal
        trim_validity   =   true;

        tr_trim_linearization_analysis;

        if trim_validity
            trim_data_tr(idx1,idx2).alt_trim            =   H_trim;
            trim_data_tr(idx1,idx2).eas_trim            =   EAS_trim;
            trim_data_tr(idx1,idx2).tas_trim            =   x_trim(1);
            trim_data_tr(idx1,idx2).aoa_trim            =   x_trim(2);
            trim_data_tr(idx1,idx2).aos_trim            =   x_trim(3);
            trim_data_tr(idx1,idx2).roll_trim           =   x_trim(4);
            trim_data_tr(idx1,idx2).pitch_trim          =   x_trim(5);
            trim_data_tr(idx1,idx2).heading_trim        =   x_trim(6);
            trim_data_tr(idx1,idx2).flight_path_trim    =   gamma_trim;
            trim_data_tr(idx1,idx2).fwd_thr_trim        =   u_trim(1);
            trim_data_tr(idx1,idx2).fwd_rot_spd_trim    =   u_trim(1)*uavParams.motor.RPMMAX;
            trim_data_tr(idx1,idx2).rwd_thr_trim        =   u_trim(2);
            trim_data_tr(idx1,idx2).rwd_rot_spd_trim    =   u_trim(2)*uavParams.motor.RPMMAX;
            trim_data_tr(idx1,idx2).rwd_thrust_trim     =   uavParams.rotor.Ct*(u_trim(2)*uavParams.motor.RPMMAX/60*2*pi)^2;
            trim_data_tr(idx1,idx2).tilt_trim           =   u_trim(3);
            trim_data_tr(idx1,idx2).aileron_trim        =   u_trim(4);
            trim_data_tr(idx1,idx2).elevator_trim       =   u_trim(5);
            trim_data_tr(idx1,idx2).rudder_trim         =   u_trim(6);

            lin_model_tr(idx1,idx2).A                   =   A;
            lin_model_tr(idx1,idx2).B                   =   B;
            lin_model_tr(idx1,idx2).C                   =   C;
            lin_model_tr(idx1,idx2).D                   =   D;
            lin_model_tr(idx1,idx2).A_lon               =   A_lon;
            lin_model_tr(idx1,idx2).B_lon               =   B_lon;
            lin_model_tr(idx1,idx2).C_lon               =   C_lon;
            lin_model_tr(idx1,idx2).D_lon               =   D_lon;
            lin_model_tr(idx1,idx2).A_lat               =   A_lat;
            lin_model_tr(idx1,idx2).B_lat               =   B_lat;
            lin_model_tr(idx1,idx2).C_lat               =   C_lat;
            lin_model_tr(idx1,idx2).D_lat               =   D_lat;
        end
    end
end

%.. scheduler plot
plot_trim_spd       =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_alt       =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_fwd_thr   =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_rwd_thr   =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_pitch     =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_tilt      =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_elevator  =   zeros(size(trimspd,2),size(trimalt,2));

plot_trim_ctrl_lon_rotor    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_lon_csurf    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_lat_rotor    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_lat_csurf    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_dir_rotor    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_dir_csurf    =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_latadv_rotor =   zeros(size(trimspd,2),size(trimalt,2));
plot_trim_ctrl_diradv_rotor =   zeros(size(trimspd,2),size(trimalt,2));  

for idx1 = 1:size(trimspd,2)
    for idx2 = 1:size(trimalt,2)

        plot_trim_spd(idx1,idx2)        =   trim_data_tr(idx1,idx2).eas_trim;
        plot_trim_alt(idx1,idx2)        =   trim_data_tr(idx1,idx2).alt_trim;
        plot_trim_fwd_thr(idx1,idx2)    =   trim_data_tr(idx1,idx2).fwd_thr_trim*100;
        plot_trim_rwd_thr(idx1,idx2)    =   trim_data_tr(idx1,idx2).rwd_thr_trim*100;
        plot_trim_pitch(idx1,idx2)      =   trim_data_tr(idx1,idx2).pitch_trim*const.rad2deg;
        plot_trim_tilt(idx1,idx2)       =   trim_data_tr(idx1,idx2).tilt_trim*(pi/2)*const.rad2deg;
        plot_trim_elevator(idx1,idx2)   =   trim_data_tr(idx1,idx2).elevator_trim*const.rad2deg;

        plot_trim_ctrl_lon_rotor(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lon(3,1);
        plot_trim_ctrl_lon_csurf(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lon(3,2);
        plot_trim_ctrl_lat_rotor(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lat(2,1);
        plot_trim_ctrl_lat_csurf(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lat(2,3);
        plot_trim_ctrl_latadv_rotor(idx1,idx2)  =   lin_model_tr(idx1,idx2).B_lat(2,4);
        plot_trim_ctrl_dir_rotor(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lat(3,2);
        plot_trim_ctrl_dir_csurf(idx1,idx2)     =   lin_model_tr(idx1,idx2).B_lat(3,4);
        plot_trim_ctrl_diradv_rotor(idx1,idx2)  =   lin_model_tr(idx1,idx2).B_lat(3,4);
    end
end

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,plot_trim_fwd_thr,'LineWidth',1.5);
plot(plot_trim_spd,plot_trim_rwd_thr,'LineWidth',1.5);
ylabel('Throttle (%)');
xlabel('EAS (kts)');
legend('Forward Throttle', 'Rear Throttle');
title('Throttle Trim Results (Back Transition)');

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,plot_trim_tilt,'LineWidth',1.5);
ylabel('Tilt (deg)');
xlabel('EAS (kts)');
title('Tilt Angle Trim Results (Back Transition)');

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,plot_trim_elevator,'LineWidth',1.5);
ylabel('Elevator (deg)');
xlabel('EAS (kts)');
title('Elevator Trim Results (Back Transition)');

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,abs(plot_trim_ctrl_lon_rotor),'LineWidth',1.5);
plot(plot_trim_spd,abs(plot_trim_ctrl_lon_csurf),'LineWidth',1.5);
ylabel('Control Power (rad/s^2)');
xlabel('EAS (kts)');
legend('Rotor','Control Surface')
title('Longitudinal Control Effectiveness (Back Transition)');

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,abs(plot_trim_ctrl_lat_rotor),'LineWidth',1.5);
plot(plot_trim_spd,abs(plot_trim_ctrl_lat_csurf),'LineWidth',1.5);
ylabel('Control Power (rad/s^2)');
xlabel('EAS (kts)');
legend('Rotor','Control Surface')
title('Lateral Control Effectiveness (Back Transition)');

figure;
set(gcf,'color','w');
hold on;
grid on;
plot(plot_trim_spd,abs(plot_trim_ctrl_dir_rotor),'LineWidth',1.5);
plot(plot_trim_spd,abs(plot_trim_ctrl_dir_csurf),'LineWidth',1.5);
ylabel('Control Power (rad/s^2)');
xlabel('EAS (kts)');
legend('Rotor','Control Surface')
title('Directional Control Effectiveness (Back Transition)');