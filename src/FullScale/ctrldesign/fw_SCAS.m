% (GPT generated, update needed) ========================================================================
% Fixed-wing Trim & Linearization Sweep (Backward Transition Scheduler)
% ------------------------------------------------------------------------
% Computes fixed-wing trim conditions over a speed/altitude grid and (if the
% called routine provides models) collects linearizations to support an
% auto-tuner workflow. Generates summary plots vs. EAS.
%
% Coordinate/units conventions (script-wide):
%   - Angles stored/used internally in [rad]; plotting converts to [deg].
%   - EAS input grid given in [kts] then converted to [m/s].
%   - Altitude input grid given in [ft] then converted to [m].
%
% Key inputs (set in this script):
%   trimspd (1xN) [kts]        : Equivalent Airspeed grid for trim (→ m/s)
%   trimalt (1xM) [ft]         : Altitude grid for trim (→ m)
%   gamma_trim [rad]           : Flight-path angle for level/climb/descend (0 = level)
%   turn_rate_trim [rad/s]     : Turn rate for coordinated-turn trim (may be unused)
%   heading_trim [rad]         : Initial heading
%   alpha_trim [rad]           : AoA constraint / initial guess for solver
%   flap_trim [rad]            : Flap setting/schedule (used if model supports it)
%   Wind [bool]                : 0 disables wind model, 1 enables (if supported)
%
% Model/data dependencies:
%   - load_const()                          : returns unit conversion constants
%   - load_vtol_dynamics_7000lb(const)      : loads aircraft/propulsion params
%   - load_digital_twin_interface()         : loads bus interfaces for plant
%   - "data\contact.mat"                    : landing gear/contact parameters
%   - fw_trim_linearization_analysis        : must set x_trim, u_trim, and
%                                             trim_validity for each case; may
%                                             also expose linear models A,B,C,D
%                                             (full, LON, LAT) in workspace
%
% Primary outputs (to base workspace):
%   trim_data (NxM struct), one entry per (trimspd, trimalt):
%     .alt_trim             [m]
%     .eas_trim             [m/s]
%     .tas_trim             [m/s]
%     .aoa_trim             [rad]
%     .aos_trim             [rad]
%     .roll_trim            [rad]
%     .pitch_trim           [rad]
%     .heading_trim         [rad]
%     .flight_path_trim     [rad]   (= gamma_trim)
%     .thr_trim             [-]     (0–1)
%     .rot_spd_trim         [RPM]   (= thr_trim * uavParams.motor.RPMMAX)
%     .elevator_trim        [rad]
%     .aileron_trim         [rad]
%     .rudder_trim          [rad]
%
% Optional outputs (if produced by fw_trim_linearization_analysis):
%   Linear models aligned with the trim grid available in workspace variables
%   (e.g., A,B,C,D and/or A_LON,B_LON,C_LON,D_LON, A_LAT,B_LAT,C_LAT,D_LAT).
%   If you prefer struct storage, assign them into lin_model_fw inside the loop.
%
% Figures produced:
%   1) Throttle trim (%) vs EAS (kts)
%   2) AoA trim (deg) vs EAS (kts)
%   3) Elevator trim (deg) vs EAS (kts)
%
% Notes/assumptions:
%   - If a trim case fails (trim_validity == false), the corresponding
%     trim_data entry is left at default (zeros/unset).
%   - Angles are provided to the solver in radians; only plotting converts to deg.
%   - Some inputs (e.g., turn_rate_trim, flap_trim, Wind) may be used internally
%     by the model/trim routine depending on configuration.
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
trimflap    =   [0 10 20 30 40]*const.deg2rad;                              % [rad] trim flap

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
        for idx3 = 1:size(trimflap,2)

            %.. trim condition
            EAS_trim        =   trimspd(idx1);
            H_trim          =   trimalt(idx2);
    
            %.. level flight conditions
            gamma_trim      =   0.0*const.deg2rad;                          % [rad] flight path angle for level-wing trim
            turn_rate_trim  =   0.0*const.deg2rad;                          % [rad/s] turning rate for level coordinated-turn trim
            heading_trim    =   0.0*const.deg2rad;                          % [rad] initial heading of aircraft
            alpha_trim      =   15.0*const.deg2rad;                         % [rad] angle of attack constraint (might be used for climb) - initial guess for level trim
    
            %.. flap condition
            flap_trim       =   trimflap(idx3);

            %.. trim validity signal
            trim_validity   =   true;

            fw_trim_linearization_analysis;

            if trim_validity
                
                trim_data_fw(idx1,idx2,idx3).alt_trim           =   H_trim;
                trim_data_fw(idx1,idx2,idx3).eas_trim           =   EAS_trim;
                trim_data_fw(idx1,idx2,idx3).tas_trim           =   x_trim(1);
                trim_data_fw(idx1,idx2,idx3).aoa_trim           =   x_trim(2);
                trim_data_fw(idx1,idx2,idx3).aos_trim           =   x_trim(3);
                trim_data_fw(idx1,idx2,idx3).roll_trim          =   x_trim(4);
                trim_data_fw(idx1,idx2,idx3).pitch_trim         =   x_trim(5);
                trim_data_fw(idx1,idx2,idx3).heading_trim       =   x_trim(6);
                trim_data_fw(idx1,idx2,idx3).flight_path_trim   =   gamma_trim;
                trim_data_fw(idx1,idx2,idx3).fwd_thr_trim       =   u_trim(1);
                trim_data_fw(idx1,idx2,idx3).fwd_rot_spd_trim   =   u_trim(1)*uavParams.motor.RPMMAX;
                trim_data_fw(idx1,idx2,idx3).fwd_thrust_trim    =   uavParams.rotor.Ct*(u_trim(1)*uavParams.motor.RPMMAX/60*2*pi)^2;
                trim_data_fw(idx1,idx2,idx3).rwd_thr_trim       =   0.0;
                trim_data_fw(idx1,idx2,idx3).rwd_rot_spd_trim   =   0.0;
                trim_data_fw(idx1,idx2,idx3).rwd_thrust_trim    =   0.0;
                trim_data_fw(idx1,idx2).tilt_trim               =   u_trim(3);
                trim_data_fw(idx1,idx2).aileron_trim            =   u_trim(4);
                trim_data_fw(idx1,idx2).elevator_trim           =   u_trim(5);
                trim_data_fw(idx1,idx2).rudder_trim             =   u_trim(6);
    
                lin_model_fw(idx1,idx2).A                   =   A;
                lin_model_fw(idx1,idx2).B                   =   B;
                lin_model_fw(idx1,idx2).C                   =   C;
                lin_model_fw(idx1,idx2).D                   =   D;
                lin_model_fw(idx1,idx2).A_lon               =   A_lon;
                lin_model_fw(idx1,idx2).B_lon               =   B_lon;
                lin_model_fw(idx1,idx2).C_lon               =   C_lon;
                lin_model_fw(idx1,idx2).D_lon               =   D_lon;
                lin_model_fw(idx1,idx2).A_lat               =   A_lat;
                lin_model_fw(idx1,idx2).B_lat               =   B_lat;
                lin_model_fw(idx1,idx2).C_lat               =   C_lat;
                lin_model_fw(idx1,idx2).D_lat               =   D_lat;
    
            end

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