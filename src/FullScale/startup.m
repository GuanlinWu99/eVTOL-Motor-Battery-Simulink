%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: start up script for simulator                                   %
% Author(s): minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%.. clear workspace, command window and close fiugres
clear all;
clc;
close all;
cleanup;

%.. load the model
mdl     =   'VTOLTiltrotor';
load_system(mdl);

% Call battery pack parameters
uav_param

motorctrl.p = 0.003; 
motorctrl.i = 0.001;
motorctrl.d = 0.1;
motorctrl.n = 100;

cfgRefTop = getActiveConfigSet('VTOLTiltrotor');         % mdl = 'VTOLTiltrotor'
cfgTop    = getRefConfigSet(cfgRefTop);
set_param(cfgTop, 'SolverType','Variable-step', 'Solver','ode23t');
% save_system(mdl);

mdlSub = 'VTOLDynamics';
load_system(mdlSub);
cfgSubActive = getActiveConfigSet(mdlSub);

if isa(cfgSubActive, 'Simulink.ConfigSetRef')
    cfgSub = getRefConfigSet(cfgSubActive);  
else
    cfgSub = cfgSubActive;                   
end

set_param(cfgSub, 'SolverType','Variable-step', 'Solver','ode23t');

%.. define the aircraft modes of flight
% Simulink.clearIntEnumType('flightState');
Simulink.defineIntEnumType('flightState',{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3],'StorageType','uint8');

%.. initialize simulator: velocity defined later
xGround     =   0;
yGround     =   0;
zGround     =   0;
iniRoll     =   0;
iniYaw      =   0/180*pi;
initPitch   =   0;
iniP        =   0;
iniQ        =   0;
iniR        =   0;

%.. initialize landing gear model
load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, 'slidingFriction', 0.8, 'rollingFriction', 0.2, 'gLimit', 100);

%.. load bus interfaces for controller
load_ctrl_interface();

%.. load bus interfaces for plant
load_digital_twin_interface;

%.. load constants
const           =   load_const();

%.. set up vtol dynamics parameters
uavParams       =   load_vtol_dynamics_7000lb(const);

%.. load controller parameters
controlParams   =   load_controller_parameters(uavParams, const);

% Flag to enable/disable visualization
Visualization   =   0;

% Disable Wind
Wind            =   0;

% Disable Sensors
SensorType      =   0;

% Setup tuning flag
TuningMode      =   0;
Deployment      =   false;

% Initialize Control and Guidance gains for Tiltrotor
exampleHelperInitializeVTOLGains_m;

% Initialize initial velocity
vIni = 0*const.kts2mps;
disp("Initialized VTOL model.")

% Initialize hover configuration
setupHoverConfiguration_mod
% setupFixedWingConfiguration_mod
% setupHoverGuidanceMission_mod

setupTransitionGuidanceMission_mod;
% setupFixedWingGuidanceMission_mod

transition_throttle = 0.2;

% Setup configuration set
configObj = getActiveConfigSet('VTOLAutopilotController');
set_param(configObj, 'SourceName', 'VTOLConfiguration');

%% Run SIMULINK
outTuned = sim(mdl);
keyboard;

%% Plot Simulation Results
% [1] : Battery Powertrain Performance (SOC, Voltage, Current, C-rate)
figure(1);
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 0.7 0.4]);
title('ddddd')

subplot(1,4,1)
plot(outTuned.Battery_Data.Batt.SOC____.Time,outTuned.Battery_Data.Batt.SOC____.Data,'LineWidth',2);
grid on
ylabel('SOC')
xlabel('Time (s)')

subplot(1,4,2)
plot(outTuned.Battery_Data.Batt.Voltage__V_.Time,outTuned.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',2);
grid on
ylabel('Voltage (V)')
xlabel('Time (s)')

subplot(1,4,3)
plot(outTuned.Battery_Data.Batt.Current__A_.Time,outTuned.Battery_Data.Batt.Current__A_.Data,'LineWidth',2);
grid on
ylabel('Current (A)')
xlabel('Time (s)')

subplot(1,4,4)
plot(outTuned.Battery_Data.Batt.C_rate.Time, outTuned.Battery_Data.Batt.C_rate.Data, 'Linewidth', 2)
grid on
ylabel('C-rate')
xlabel('Time (s)')

keyboard

%% [2] : Flight Performance (motor angular velocity, tilting angle, and altitude)
figure(2);
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 0.7 0.4]);
subplot(1,3,1)
plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 2,'LineStyle','-'); hold on;
plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 2,'LineStyle','--'); hold on;
plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 2,'LineStyle','-'); hold on;
plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 2,'LineStyle','--'); hold on;
grid on
ylabel('w (rad/s)')
xlabel('Time (s)')

subplot(1,3,2)
plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 2); hold on;
plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 2, 'LineStyle', '--'); hold on;
grid on
ylabel('Tilt (deg)')
xlabel('Time (sec)')

subplot(1,3,3)
plot(outTuned.UAV_State.Xe.Time, -reshape(outTuned.UAV_State.Xe.Data(3,:,:),[size(outTuned.UAV_State.Xe.Data(2,:,:),3),1]), 'Linewidth', 1.5);
grid on
ylabel('Altitude (m)')
xlabel('Time (sec)')

keyboard;

%% Plot results
% exampleHelperPlotHoverControlTrackingResults(outTuned);
% EVTOL_Plots_update(outTu ned)


% figure
% hold on
% plot3(outTuned.UAV_State.Xe.Data(1,:),-outTuned.UAV_State.Xe.Data(2,:),-outTuned.UAV_State.Xe.Data(3,:),'LineWidth',2)
% plot3([0, 0, 20, 1000, 4000, 8000, 10000, 15000, 17000],-[0, 0, 0, 0, 0, 0, 0, 0, 0],-[0, -100, -100, -150, -1000, -1000, -1000, -100, -100],'LineWidth',1.5,'LineStyle','--')
% hold off
% grid on
% xlabel('North (m)')
% ylabel('West (m)')
% zlabel('Up (m)')
% legend('UAV-3D Trajectory', 'P4-profile (Modified)')
% ylim([-250, 250])
% title('eVTOL trajectory (P4 profile)')
% 
% v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
% v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% 
% figure
% subplot(3,1,1)
% hold on
% plot(outTuned.UAV_State.airspeed.Time, outTuned.UAV_State.airspeed.Data, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Airspeed (m/s)')
% title('eVTOL AirData (P4 profile)')
% subplot(3,1,2)
% plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 1.5)
% hold off
% grid on
% ylim([-5 35])
% ylabel('AoA (deg)')
% subplot(3,1,3)
% plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 1.5)
% hold off
% grid on
% ylim([-20 20])
% ylabel('AoS (deg)')
% xlabel('Time (sec)')
% 
% figure
% subplot(3,1,1)
% hold on
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Roll (deg)')
% ylim([-30 30])
% title('eVTOL Attitude (P4 profile)')
% subplot(3,1,2)
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Pitch (deg)')
% ylim([-40 40])
% subplot(3,1,3)
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Yaw (deg)')
% ylim([-30 30])
% xlabel('Time (sec)')
% 
% figure
% subplot(2,1,1)
% hold on
% plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 1.5,'LineStyle','--')
% hold off
% grid on
% ylabel('Rotor SPD (rad/s)')
% legend('1: front/right','2: front/left')
% title('eVTOL Rotor Speed (P4 profile)')
% subplot(2,1,2)
% hold on
% plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 1.5,'LineStyle','--')
% hold off
% grid on
% ylabel('Rotor SPD (rad/s)')
% legend('3: rear/left','4: rear/right')
% xlabel('Time (sec)')

figure
hold on
plot3(outTuned.UAV_State.Xe.Data(1,:),-outTuned.UAV_State.Xe.Data(2,:),-outTuned.UAV_State.Xe.Data(3,:),'LineWidth',2)
% plot3([0, 0, 20, 1000, 4000, 8000, 10000, 15000, 17000],-[0, 0, 0, 0, 0, 0, 0, 0, 0],-[0, -100, -100, -150, -1000, -1000, -1000, -100, -100],'LineWidth',1.5,'LineStyle','--')
hold off
grid on
xlabel('North (m)')
ylabel('West (m)')
zlabel('Up (m)')
% legend('UAV-3D Trajectory', 'P4-profile (Modified)')
ylim([-250, 250])
% title('eVTOL trajectory (P4 profile)')

figure
ax1(1) = subplot(4,1,1)
hold on
plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 1.5)
plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 1.5, 'LineStyle', '--')
hold off
grid on
ylabel('Tilt (deg)')
ax1(2) = subplot(4,1,2)
hold on
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 1.5)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 1.5)
hold off
grid on
ylabel('Euler (deg)')
ax1(3) = subplot(4,1,3)
hold on
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]), 'Linewidth', 1.5)
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,2,:),3),1]), 'Linewidth', 1.5)
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,3,:),3),1]), 'Linewidth', 1.5)
hold off
grid on
ylabel('Vb (m/s)')
ax1(4) = subplot(4,1,4)
hold on
plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 1.5,'LineStyle','-')
plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 1.5,'LineStyle','--')
plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 1.5,'LineStyle','-')
plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 1.5,'LineStyle','--')
hold off
grid on
ylabel('w (rad/s)')
xlabel('Time (sec)')
linkaxes(ax1,'x')

v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);

figure
ax2(1) = subplot(4,1,1)
hold on
plot(outTuned.UAV_State.airspeed.Time, outTuned.UAV_State.airspeed.Data, 'Linewidth', 1.5)
hold off
grid on
ylabel('Air SPD (m/s)')
ax2(2) = subplot(4,1,2)
hold on
plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 1.5)
plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 1.5)
hold off
grid on
ylabel('AOA/AOS (deg)')
ax2(3) = subplot(4,1,3)
hold on
plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(3,:,:),[size(outTuned.UAV_State.Xe.Data(3,:,:),3),1]), 'Linewidth', 1.5)
hold off
grid on
ylabel('Z (m)')
ax2(4) = subplot(4,1,4)
hold on
plot(outTuned.UAV_State.aileron.Time, outTuned.UAV_State.aileron.Data*const.rad2deg, 'Linewidth', 1.5)
plot(outTuned.UAV_State.elevator.Time, outTuned.UAV_State.elevator.Data*const.rad2deg, 'Linewidth', 1.5)
plot(outTuned.UAV_State.rudder.Time, outTuned.UAV_State.rudder.Data*const.rad2deg, 'Linewidth', 1.5)
hold off
grid on
ylim([-30 30])
legend('AIL','ELE','RUD')
ylabel('CtrlSurf (deg)')
linkaxes(ax2,'x')

figure
ax3(1) = subplot(4,1,1)
hold on
plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(1,:,:),[size(outTuned.UAV_State.Xe.Data(1,:,:),3),1]), 'Linewidth', 1.5)
hold off
grid on
ylabel('X (m)')
ax3(2) = subplot(4,1,2)
hold on
plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(2,:,:),[size(outTuned.UAV_State.Xe.Data(2,:,:),3),1]), 'Linewidth', 1.5)
hold off
grid on
ylabel('Y (m)')
ylim([-5, 5])
ax3(3) = subplot(4,1,3)
hold on
hold off
grid on
ylabel('')
ax3(4) = subplot(4,1,4)
hold on
hold off
grid on
ylabel('')
linkaxes(ax3,'x')

% subplot(3,1,2)
% 
% hold off
% grid on
% ylim([-5 35])
% ylabel('AoA (deg)')
% subplot(3,1,3)
% hold off
% grid on
% ylim([-20 20])
% ylabel('AoS (deg)')
% xlabel('Time (sec)')


% TransitionMission = struct;
% TransitionMission(1).mode = 1;
% TransitionMission(1).position = [0; 0; 0];
% TransitionMission(1).params = [0; 0; 0; 0];
% TransitionMission(2).mode = 2;
% TransitionMission(2).position = [0; 0; -100];
% TransitionMission(2).params = [0; 0; 0; 0];
% TransitionMission(3).mode = 2;
% TransitionMission(3).position = [20; 0; -100];
% TransitionMission(3).params = [0; 0; 0; 0];
% TransitionMission(4).mode = 6;
% TransitionMission(4).position = [1;1;1];
% TransitionMission(4).params = [1; 1; 1; 1];
% TransitionMission(5).mode = 2;
% TransitionMission(5).position = [1000; 0; -150];
% TransitionMission(5).params = [0; 0; 0; 0];
% TransitionMission(6).mode = 2;
% TransitionMission(6).position = [8000; 0; -1000];
% TransitionMission(6).params = [0; 0; 0; 0];
% TransitionMission(7).mode = 2;
% TransitionMission(7).position = [10000; 0; -1000];
% TransitionMission(7).params = [0; 0; 0; 0];
% TransitionMission(8).mode = 2;
% TransitionMission(8).position = [15000; 0; -100];
% TransitionMission(8).params = [0; 0; 0; 0];
% TransitionMission(9).mode = 2;
% TransitionMission(9).position = [17000; 0; -100];
% TransitionMission(9).params = [0; 0; 0; 0];