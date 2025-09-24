%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: start up script for simulator                                   %
% Author(s): Minhyun                                                     %
% Description and updates (09/12/2025):                                  %
% 1. Include drag torque calculation in the powertrain
% 2. Upgrade battery pack in the powertrain                         
% 3. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%... Clear workspace, command window and close fiugres
clear all;
clc;
close all;
cleanup;

%... Load the model
mdl ='VTOLTiltrotor';
load_system(mdl);

%... Call motor speed PID controller gains
motorctrl.p  = 0.4;
motorctrl.i  = 0.001;
motorctrl.d  = 0;
motorctrl.n  = 100;
Limit        = 700;

cfgRefTop    = getActiveConfigSet('VTOLTiltrotor');         % mdl = 'VTOLTiltrotor'
cfgTop       = getRefConfigSet(cfgRefTop);
set_param(cfgTop, 'SolverType','Variable-step', 'Solver','ode23t');
% save_system(mdl);

mdlSub       = 'VTOLDynamics';
load_system(mdlSub);
cfgSubActive = getActiveConfigSet(mdlSub);

if isa(cfgSubActive, 'Simulink.ConfigSetRef')
    cfgSub = getRefConfigSet(cfgSubActive);  
else
    cfgSub = cfgSubActive;                   
end

set_param(cfgSub, 'SolverType','Variable-step', 'Solver','ode23t');

%... Define the aircraft modes of flight
% Simulink.clearIntEnumType('flightState');
Simulink.defineIntEnumType('flightState',{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3],'StorageType','uint8');

% Hover mode : 0
% Transition mode : 1
% Fixedwing mode : 2
% Back Transition mode : 3

%... Initialize simulator: velocity defined later
xGround     =   0;
yGround     =   0;
zGround     =   0;
iniRoll     =   0;
iniYaw      =   0/180*pi;
initPitch   =   0;
iniP        =   0;
iniQ        =   0;
iniR        =   0;

%... Initialize landing gear model
load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, 'slidingFriction', 0.8, 'rollingFriction', 0.2, 'gLimit', 100);

%... Load bus interfaces for controller
load_ctrl_interface();

%... Load bus interfaces for plant
load_digital_twin_interface;

%... Load constants
const                  =   load_const();

%... Set up vtol dynamics parameters
[uavParams, HEV_Param] =   load_vtol_dynamics_7000lb(const);

%% Check CM, CL, CD

figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.35], 'Color','w'); 
subplot(1,3,1)
plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CL, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CL_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CL_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
grid on; xlabel('\alpha [deg]'); ylabel('C_L');
title('Lift Curve');
legend('Location','best');

subplot(1,3,2)
plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CD, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CD_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CD_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
grid on; xlabel('\alpha [deg]'); ylabel('C_D');
title('Drag Curve');
legend('Location','best');

subplot(1,3,3)
plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CM, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CM_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CM_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
grid on; xlabel('\alpha [deg]'); ylabel('C_M');
title('Pitching Moment');
legend('Location','best');

%%
%... Load controllercontrolParams.TiltScheduleRate parameters
controlParams          =   load_controller_parameters(uavParams, const);

%... Flag to enable/disable visualization
Visualization          =   1;

%... Disable Wind
Wind                   =   0;

if Wind == 1
    Wind_Speed         =   5;
end

%... Disable Sensors
SensorType             =   0;

%... Setup tuning flag
TuningMode             =   0;
Deployment             =   false;

%... Initialize Control and Guidance gains for Tiltrotor
exampleHelperInitializeVTOLGains_m;

%... Initialize initial velocity
vIni = 0*const.kts2mps;
disp("Initialized VTOL model.")

%... Initialize hover configuration
setupHoverConfiguration_mod
% setupFixedWingConfiguration_mod
% setupHoverGuidanceMission_mod

setupTransitionGuidanceMission_mod;
% setupFixedWingGuidanceMission_mod

%... Setup configuration set
configObj = getActiveConfigSet('VTOLAutopilotController');
set_param(configObj, 'SourceName', 'VTOLConfiguration');
transition_throttle = 0.2;

Back_Transition_Rate = 0.5*controlParams.TiltScheduleRate;

fprintf('Forward TiltAngle Rate %.2f [deg/s]\n', controlParams.TiltScheduleRate*57.295);
fprintf('Backward TiltAngle Rate %.2f [deg/s]\n', Back_Transition_Rate*57.295);

%exampleHelperAutomatedHoverControlTuning;

keyboard;

Flap_Activate  =  16000;

%%... Run SIMULINK
tic
outTuned = sim(mdl);
toc

%% Read Simulation data
Time                   =   outTuned.Rotor1_RPM_Reference.Time;
Time_motor             =   outTuned.Motor1_Current.Time;
Time_torque            =   outTuned.Rotor1_Drag_Tq.Time;
Time_battery           =   outTuned.Battery_Data.Batt.SOC____.Time;
Time_flight            =   outTuned.UAV_State.Xe.Time;

positionFeedbackData   =   squeeze(outTuned.PositionCmdFdbk.signals.values);

Rotor1_RPM_Reference   =   outTuned.Rotor1_RPM_Reference.Data;
Rotor2_RPM_Reference   =   outTuned.Rotor2_RPM_Reference.Data;
Rotor3_RPM_Reference   =   outTuned.Rotor3_RPM_Reference.Data;
Rotor4_RPM_Reference   =   outTuned.Rotor4_RPM_Reference.Data;

Rotor1_RPM             =   outTuned.Rotor1_RPM.Data;
Rotor2_RPM             =   outTuned.Rotor2_RPM.Data;
Rotor3_RPM             =   outTuned.Rotor3_RPM.Data;
Rotor4_RPM             =   outTuned.Rotor4_RPM.Data;

Motor1_Current         =   outTuned.Motor1_Current.Data;
Motor2_Current         =   outTuned.Motor2_Current.Data;
Motor3_Current         =   outTuned.Motor3_Current.Data;
Motor4_Current         =   outTuned.Motor4_Current.Data;

Motor1_Voltage         =   outTuned.Motor1_Voltage.Data;
Motor2_Voltage         =   outTuned.Motor2_Voltage.Data;
Motor3_Voltage         =   outTuned.Motor3_Voltage.Data;
Motor4_Voltage         =   outTuned.Motor4_Voltage.Data;

Motor1_Power           =   outTuned.Motor1_Power.Data;
Motor2_Power           =   outTuned.Motor2_Power.Data;
Motor3_Power           =   outTuned.Motor3_Power.Data;
Motor4_Power           =   outTuned.Motor4_Power.Data;

Motor1_Drag_Tq         =   outTuned.Rotor1_Drag_Tq.Data;
Motor2_Drag_Tq         =   outTuned.Rotor2_Drag_Tq.Data;
Motor3_Drag_Tq         =   outTuned.Rotor3_Drag_Tq.Data;
Motor4_Drag_Tq         =   outTuned.Rotor4_Drag_Tq.Data;

Battery_SOC            =   outTuned.Battery_Data.Batt.SOC____.Data;
Battery_Crate          =   outTuned.Battery_Data.Batt.C_rate.Data;
Battery_Current        =   outTuned.Battery_Data.Batt.Current__A_.Data;

v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);

keyboard


% % figure
% % subplot(3,1,1)
% % hold on
% % plot(outTuned.UAV_State.airspeed.Time, outTuned.UAV_State.airspeed.Data, 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylabel('Airspeed (m/s)')
% % title('eVTOL AirData (P4 profile)')
% % subplot(3,1,2)
% % plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylim([-5 35])
% % ylabel('AoA (deg)')
% % subplot(3,1,3)
% % plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylim([-20 20])
% % ylabel('AoS (deg)')
% % xlabel('Time (sec)')


%% Plot Simulations (09/12/2025) 

%% [1] 4 Rotor Performance

figure('Units','normalized','OuterPosition',[0.1 0.1 0.65 0.9], 'Color','w');
t = tiledlayout(4,4,'TileSpacing','compact','Padding','compact');
names = ["Motor1","Motor2","Motor3","Motor4"];
Width = 2;
AxisFont = 17;

% -------- Row 1: Speed --------
ax1 = gobjects(1,4);
for k = 1:4
    ax1(k) = nexttile(k);
    plot(Time, eval("Rotor"+k+"_RPM"), 'LineWidth',Width); hold on;
    plot(Time, eval("Rotor"+k+"_RPM_Reference"), '-.', 'LineWidth', Width);
    grid on; box on;
    title(names(k) + " Speed");
    if k==1, ylabel('RPM','FontSize',AxisFont); end
    if k==1, legend({'Actual','Cmd'},'Location','southeast'); else, legend off; end
    ylim([0, max(eval("Rotor"+k+"_RPM_Reference"))])
end
linkaxes(ax1,'y');

% -------- Row 2: Power --------
ax2 = gobjects(1,4);
for k = 1:4
    ax2(k) = nexttile(4+k);
    plot(Time_motor, eval("Motor"+k+"_Power"), 'LineWidth', Width);
    grid on; box on;
    title(names(k) + " Power");
    if k==1, ylabel('Power (kW)','FontSize',AxisFont); end
end
linkaxes(ax2,'y');

% -------- Row 3: Current --------
ax3 = gobjects(1,4);
for k = 1:4
    ax3(k) = nexttile(8+k);
    plot(Time_motor, eval("Motor"+k+"_Current"), 'LineWidth', Width);
    grid on; box on;
    title(names(k) + " Current");
    if k==1, ylabel('Current (A)','FontSize',AxisFont); end
end
linkaxes(ax3,'y');

% -------- Row 4: Voltage --------
ax4 = gobjects(1,4);
for k = 1:4
    ax4(k) = nexttile(12+k);
    plot(Time_motor, eval("Motor"+k+"_Voltage"), 'LineWidth', Width);
    grid on; box on;
    title(names(k) + " Voltage");
    if k==1, ylabel('Voltage (V)','FontSize',AxisFont); end
    xlabel('Time (s)','FontSize',AxisFont);
end
linkaxes(ax4,'y');

sgtitle('Tilted-Rotor eVTOL (Rotors 1 & 2 Tilted, 3 & 4 Fixed)','FontSize',15,'FontWeight','bold');

%% [2] Drag Torque

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.65 0.3], 'Color','w'); 
% subplot(1,4,1)
% plot(Time_torque, Motor1_Drag_Tq, 'LineWidth', Width); grid on;
% xlabel('Time (s)')
% ylabel('(Nm)')
% title('Motor1 Drag Torque')
% 
% subplot(1,4,2)
% plot(Time_torque, Motor2_Drag_Tq, 'LineWidth', Width); grid on;
% xlabel('Time (s)')
% ylabel('(Nm)')
% title('Motor2 Drag Torque')
% 
% subplot(1,4,3)
% plot(Time_torque, Motor3_Drag_Tq, 'LineWidth', Width); grid on;
% xlabel('Time (s)')
% ylabel('(Nm)')
% title('Motor3 Drag Torque')
% 
% subplot(1,4,4)
% plot(Time_torque, Motor4_Drag_Tq, 'LineWidth', Width); grid on;
% xlabel('Time (s)')
% ylabel('(Nm)')
% title('Motor4 Drag Torque')

%% [3] Battery Pack Electrical Performance

L = length(outTuned.Battery_Data.Batt.Voltage__V_.Time);
Pack_Power = outTuned.Battery_Data.Batt.Voltage__V_.Data .* outTuned.Battery_Data.Batt.Current__A_.Data;
Pack_Energy = zeros(L,1);

for i = 2 : length(outTuned.Battery_Data.Batt.Voltage__V_.Time)
    Energy_tmp = Pack_Power(i)*0.001;
    Pack_Energy(i) = Energy_tmp + Pack_Energy(i-1); 
end

figure('Units','normalized','OuterPosition',[0.1 0.1 0.4 0.5], 'Color','w'); 
subplot(2,3,1)
plot(outTuned.Battery_Data.Batt.SOC____.Time, outTuned.Battery_Data.Batt.SOC____.Data,'LineWidth',2); grid on;
title('SOC')
ylabel('(%)')
xlabel('Time (s)')

subplot(2,3,2)
plot(outTuned.Battery_Data.Batt.C_rate.Time, outTuned.Battery_Data.Batt.C_rate.Data,'LineWidth',2); grid on;
title('C-rate')
ylabel('(-)')
xlabel('Time (s)')

subplot(2,3,3)
plot(outTuned.Battery_Data.Batt.Current__A_.Time, outTuned.Battery_Data.Batt.Current__A_.Data,'LineWidth',2); grid on;
title('Current')
ylabel('(A)')
xlabel('Time (s)')

subplot(2,3,4)
plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, outTuned.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',2); grid on;
title('Voltage')
ylabel('(V)')
xlabel('Time (s)')

subplot(2,3,5)
plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, Pack_Power/1000,'LineWidth',2); grid on;
title('Power')
ylabel('(kW)')
xlabel('Time (s)')

subplot(2,3,6)
plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, Pack_Energy/(3.6*10^6),'LineWidth',2); grid on;
title('Energy')
ylabel('(kWh)')
xlabel('Time (s)')

sgtitle('Battery Pack Electrical Performance','FontSize',15,'FontWeight','bold');

%% [4] Flight Dynamics Simulations 

figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.7], 'Color','w'); 
subplot(3,3,1)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
xlabel('Time (s)'); ylabel('(-)'); grid on; title('North & Flight Mode') 

yyaxis left
plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(4,:)/1000,'LineWidth',2)
ylabel('(km)'); legend('North','Mode','Location','best'); xlim([0 Total_sim_time])

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color = 'k';  

subplot(3,3,2)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
xlabel('Time (s)'); ylabel('(-)'); grid on; title('West & Flight Mode') 

yyaxis left
plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(5,:),'LineWidth',2)
ylabel('(km)'); legend('West','Mode','Location','best'); xlim([0 Total_sim_time]); ylim([-50 50]);

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color = 'k';  

subplot(3,3,3)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
xlabel('Time (s)'); ylabel('(-)'); grid on; title('Altitude & Flight Mode') 

yyaxis left
plot(outTuned.PositionCmdFdbk.time,-positionFeedbackData(6,:),'LineWidth',2); 
ylabel('(m)'); legend('Altitude','Mode','Location','best'); xlim([0 Total_sim_time])

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color = 'k';  

subplot(3,3,4)
plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 2); 
grid on; ylim([-5 35]); xlabel('Time (s)'); ylabel('(deg)'); title('AOA Angle'); xlim([0 Total_sim_time])

subplot(3,3,5)
plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 2); hold on;
plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 2, 'LineStyle', '--')
grid on; xlabel('Time (s)'); ylabel('(deg)'); title('Tilt Angles'); xlim([0 Total_sim_time])

subplot(3,3,6)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 2); hold on; grid on;
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 2); hold on;
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 2); hold on;
xlabel('Time (s)'); ylabel('(deg)') ;title('Euler Angles'); legend('Roll','Pitch','Yaw','Location','southwest'); xlim([0 Total_sim_time])

subplot(3,3,7)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
grid on; xlabel('Time (s)'); ylabel('(-)'); title('Body Velocity & Flight Mode')

yyaxis left
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,1,:), [size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]), 'LineWidth', 2);
ylabel('(m/s)'); legend('Vx','Mode','Location','southwest'); xlim([0 Total_sim_time]); 

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color   = 'k';  

subplot(3,3,8)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
xlabel('Time (s)'); ylabel('(-)'); grid on; title('Body Velocity & Flight Mode')

yyaxis left
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,2,:), [size(outTuned.UAV_State.Vb.Data(:,2,:),3),1]), 'LineWidth', 2);
ylabel('(m/s)'); legend('Vy','Mode','Location','southwest'); xlim([0 Total_sim_time]);

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color = 'k';  

subplot(3,3,9)
yyaxis right
plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
xlabel('Time (s)'); ylabel('(-)'); grid on; title('Body Velocity & Flight Mode')

yyaxis left
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,3,:), [size(outTuned.UAV_State.Vb.Data(:,3,:),3),1]), 'LineWidth', 2);
ylabel('(m/s)'); legend('Vz','Mode','Location','southwest'); xlim([0 Total_sim_time]);

ax = gca;
ax.YAxis(1).Color = 'k';  
ax.YAxis(2).Color = 'k';  
ax.XAxis.Color = 'k';  

sgtitle('Flight Dynamics Simulations','FontSize',15,'FontWeight','bold');

% [5] Flight Dynamics Simulations (3D)

figure('Units','normalized','OuterPosition',[0.1 0.1 0.3 0.45], 'Color','w'); 
plot3(positionFeedbackData(4,:),-positionFeedbackData(5,:),-positionFeedbackData(6,:),'LineWidth',2); grid on
xlabel('North (m)')
ylabel('West (m)')
zlabel('Altitude (m)')
ylim([-200 200])

keyboard;

% subplot(2,3,6)
% plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 2); hold on; grid on
% plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 2);
% xlabel('Time (s)')
% ylabel('(deg)')
% title('AOA/AOS Angles')

%% [5] Flight Guidance Simulations
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.7], 'Color','w'); 
% subplot(3,3,1)
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(4,:),'LineWidth',2)
% grid
% xlabel('Time (sec)')
% ylabel('(m)')
% title('North')
% 
% subplot(3,3,2)
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(5,:),'LineWidth',2)
% grid
% xlabel('Time (sec)')
% ylabel('(m)')
% ylim([-100 100])
% title('West')
% 
% subplot(3,3,3)
% plot(outTuned.PositionCmdFdbk.time,-positionFeedbackData(6,:),'LineWidth',2); hold on; grid on;
% ylabel('(m)')
% xlabel('Time (sec)')
% title('Altitude')
% 
% subplot(3,3,4)
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(1,:),'LineWidth',2); hold on;
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(4,:),'LineWidth',2); grid on
% xlabel('Time (sec)')
% ylabel('(m)')
% title('North')
% 
% subplot(3,3,5)
% plot(outTuned.FixedWingCmdFdbk.time, outTuned.FixedWingCmdFdbk.signals.values(:,2),'LineWidth',2); hold on; grid on
% plot(outTuned.FixedWingCmdFdbk.time, outTuned.FixedWingCmdFdbk.signals.values(:,5),'LineWidth',2);
% xlabel('Time (sec)')
% ylabel('(m)')
% title('Fixed-Wing: Altitude')
% 
% subplot(3,3,6)
% plot(outTuned.FixedWingCmdFdbk.time, outTuned.FixedWingCmdFdbk.signals.values(:,1),'LineWidth',2); hold on; grid on
% plot(outTuned.FixedWingCmdFdbk.time, outTuned.FixedWingCmdFdbk.signals.values(:,4),'LineWidth',2);
% xlabel('Time (sec)')
% ylabel('(m/s)')
% title('Fixed-Wing: Body Speed')
% 
% subplot(3,3,7)
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); grid on
% xlabel('Time (sec)')
% ylabel('(-)')
% title('Flight Mode')
% 
% subplot(3,3,8)
% plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 2); hold on; grid on
% plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 2, 'LineStyle', '--')
% xlabel('Time (s)')
% ylabel('(deg)')
% title('Tilt Angles')
% 
% subplot(3,3,9)
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 2); hold on; grid on;
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 2); hold on;
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 2); hold on;
% xlabel('Time (s)')
% ylabel('(deg)')
% title('Euler Angles')
% legend('Roll','Pitch','Yaw','Location','southwest')

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.3 0.45], 'Color','w'); 
% plot3(positionFeedbackData(4,:),-positionFeedbackData(5,:),-positionFeedbackData(6,:),'LineWidth',2); grid on
% xlabel('North (m)')
% ylabel('West (m)')
% zlabel('Up (m)')
% ylim([-200 200])
% 
% keyboard;


% %% Previous code
% plot(MotorRPM.Time, MotorRPM.Data,'LineWidth',2); hold on; grid on;
% plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data,'-.','LineWidth',2); hold on;
% title('Motor Speed (RPM)')
% ylabel('RPM')
% legend('RPM','RPM cmd')
% 
% subplot(2,5,2)
% plot(MotorRPM.Time, MotorRPM.Data*2*pi/60,'LineWidth',2); hold on; grid on;
% plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data*2*pi/60,'-.','LineWidth',2); hold on;
% legend('rad/s','rad/s cmd')
% ylabel('(rad/s)')
% title('Motor Speed (rad/s)')
% 
% subplot(2,5,3)
% plot(Motor_Current_Test.Time, Motor_Current_Test.Data,'LineWidth',2); grid on;
% legend('Motor Current')
% title('Motor Current')
% ylabel('(A)')
% 
% subplot(2,5,4)
% plot(Torque_Command_Test.Time, Torque_Command_Test.Data,'LineWidth',2); hold on; grid on;
% legend('Motor Torque')
% ylabel('(Nm)')
% title('Motor Torque')
% ylim([0 Torque_Command_Test.Data(1)])
% 
% subplot(2,5,5)
% plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,1)/1000,'LineWidth',2); grid on;
% legend('Motor Electrical Power')
% title('Motor Power')
% ylabel('(kW)')
% 
% subplot(2,5,6)
% plot(Battery_Output_Data.Batt.SOC____.Time, Battery_Output_Data.Batt.SOC____.Data,'LineWidth',2);
% grid on
% title('SOC')
% legend('SOC')
% ylabel('(%)')
% xlabel('Time (s)')
% xlim([0 Battery_Output_Data.Batt.SOC____.Time(end)])
% 
% subplot(2,5,7)
% plot(Battery_Output_Data.Batt.Voltage__V_.Time, Battery_Output_Data.Batt.Voltage__V_.Data,'LineWidth',2);
% grid on
% title('Voltage')
% legend('Battery Voltage')
% ylabel('(V)')
% xlabel('Time (s)')
% xlim([Battery_Output_Data.Batt.Voltage__V_.Time(1) Battery_Output_Data.Batt.Voltage__V_.Time(end)])
% 
% subplot(2,5,8)
% plot(Battery_Output_Data.Batt.Current__A_.Time, Battery_Output_Data.Batt.Current__A_.Data,'LineWidth',2);
% grid on
% title('Current')
% legend('Battery Pack')
% ylabel('(A)')
% xlabel('Time (s)')
% xlim([Battery_Output_Data.Batt.Current__A_.Time(1) Battery_Output_Data.Batt.Current__A_.Time(end)])
% 
% subplot(2,5,9)
% plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
% grid on
% title('C-rate')
% legend('Battery C-rate')
% ylabel('(-)')
% xlabel('Time (s)')
% xlim([Battery_Output_Data.Batt.C_rate.Time(1) Battery_Output_Data.Batt.C_rate.Time(end)])
% 
% subplot(2,5,10)
% plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,3)/10^5,'LineWidth',2); hold on; grid on;
% title('Power')
% legend('Battery Pack Power')
% xlabel('Time (s)')
% ylabel('(MW)')
% 
% keyboard;
% 
% % figure(1);
% % subplot(4,1,1)
% % plot(outTuned.Battery_Data.Batt.SOC____.Time,outTuned.Battery_Data.Batt.SOC____.Data,'LineWidth',2);
% % grid on
% % title('SOC')
% % ylabel('(%)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.Batt.SOC____.Time(end)])
% % 
% % subplot(4,1,2)
% % plot(outTuned.Battery_Data.Batt.SOC____.Time,outTuned.Battery_Data.Batt.Batt_Ah.Data,'LineWidth',2);
% % grid on
% % title('Capavity')
% % ylabel('(Ah)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.Batt.SOC____.Time(end)])
% % 
% % subplot(4,1,3)
% % plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, outTuned.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',2);
% % grid on
% % title('Voltage')
% % ylabel('(V)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.Batt.Voltage__V_.Time(end)])
% % 
% % subplot(4,1,4)
% % plot(outTuned.Battery_Data.Batt.Current__A_.Time, outTuned.Battery_Data.Batt.Current__A_.Data,'LineWidth',2);
% % grid on
% % title('Current')
% % ylabel('(A)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.Batt.Current__A_.Time(end)])
% 
% keyboard;
% 
% %%
% 
% figure(1)
% plot(outTuned.UAV_State.Xe.Time, -reshape(outTuned.UAV_State.Xe.Data(3,:,:),[size(outTuned.UAV_State.Xe.Data(2,:,:),3),1]), 'Linewidth', 2);
% grid on
% title('Altitude')
% ylabel('(m)')
% xlabel('Time (s)')
% keyboard;
% 
% figure(2)
% plot(outTuned.Rotor_Force.Time, -outTuned.Rotor_Force.Data(:,1)/1000, 'Linewidth', 2); hold on; grid on;
% plot(outTuned.Rotor_Force.Time, -outTuned.Rotor_Force.Data(:,2)/1000, 'Linewidth', 2); hold on;
% plot(outTuned.Rotor_Force.Time, -outTuned.Rotor_Force.Data(:,3)/1000, 'Linewidth', 2); hold on;
% legend('Fx','Fy','Fz')
% title('Rotor Thrust')
% ylabel('(kN)')
% xlabel('Time (s)')
% keyboard;
% 
% figure(3)
% plot(outTuned.Battery_Data.signal6.Time, outTuned.Battery_Data.signal6.Data(:,1)/1000, 'Linewidth', 2); hold on; grid on;
% plot(outTuned.Battery_Data.signal6.Time, outTuned.Battery_Data.signal6.Data(:,3)/1000, 'Linewidth', 2); hold on;
% legend('Motor Power','Battery Power')
% title('Power (eVTOL Electric Powertrain)')
% ylabel('(kW)')
% xlabel('Time (s)')
% keyboard;
% 
% figure(4)
% plot(outTuned.Motor_Current_Test.Time, outTuned.Motor_Current_Test.Data, 'Linewidth', 2); 
% hold on; 
% grid on;
% 
% figure(5)
% plot(outTuned.Torque_Command_Test.Time, outTuned.Torque_Command_Test.Data, 'Linewidth', 2); 
% hold on; 
% grid on;
% 
% 
% % %% Calculate the required battery energy 
% % Total_Time = outTuned.Battery_Data.signal6.Time(end);
% % dT         = 0.001;
% % t          = 0:dT:Total_Time;
% % Energy     = zeros(length(t),1);
% % 
% % for i = 1 : length(t)
% %     power = outTuned.Battery_Data.signal6.Data(i,3);
% %     e     = power*dT;
% %     if i == 1
% %         Energy(i) = e; 
% %     else
% %         Energy(i) = Energy(i-1) + e;
% %     end
% % end
% 
% 
% %% Plot Simulation Results
% % [1] : Battery Powertrain Performance (SOC, Voltage, Current, C-rate)
% figure(1);
% set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 0.35]);
% 
% subplot(1,6,1)
% plot(outTuned.Battery_Data.Batt.SOC____.Time,outTuned.Battery_Data.Batt.SOC____.Data,'LineWidth',2);
% grid on
% title('SOC')
% ylabel('(%)')
% xlabel('Time (s)')
% xlim([0 outTuned.Battery_Data.Batt.SOC____.Time(end)])
% 
% subplot(1,6,2)
% plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, outTuned.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',2);
% grid on
% title('Voltage')
% ylabel('(V)')
% xlabel('Time (s)')
% xlim([0 outTuned.Battery_Data.Batt.Voltage__V_.Time(end)])
% 
% subplot(1,6,3)
% plot(outTuned.Battery_Data.Batt.Current__A_.Time, outTuned.Battery_Data.Batt.Current__A_.Data,'LineWidth',2);
% grid on
% title('Current')
% ylabel('(A)')
% xlabel('Time (s)')
% xlim([0 outTuned.Battery_Data.Batt.Current__A_.Time(end)])
% 
% subplot(1,6,4)
% plot(outTuned.Battery_Data.Batt.C_rate.Time, outTuned.Battery_Data.Batt.C_rate.Data, 'Linewidth', 2)
% grid on
% title('C-rate')
% ylabel('(-)')
% xlabel('Time (s)')
% xlim([0 outTuned.Battery_Data.Batt.C_rate.Time(end)])
% 
% % subplot(1,6,5)
% % plot(outTuned.Battery_Data.signal6.Time, Energy/3600, 'Linewidth', 2)
% % grid on
% % title('Energy')
% % ylabel('(Wh)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.signal6.Time(end)])
% 
% subplot(1,6,6)
% plot(outTuned.Battery_Data.signal6.Time, outTuned.Battery_Data.signal6.Data(:,3)/1000, 'Linewidth', 2)
% grid on
% title('Power')
% ylabel('(kW)')
% xlabel('Time (s)')
% xlim([0 outTuned.Battery_Data.signal6.Time(end)])
% 
% keyboard
% 
% % figure(3)
% % plot(outTuned.Battery_Data.signal6.Time, outTuned.Battery_Data.signal6.Data(:,1)/1000, 'Linewidth', 2)
% % grid on
% % title('Motor Power')
% % ylabel('(kW)')
% % xlabel('Time (s)')
% % xlim([0 outTuned.Battery_Data.signal6.Time(end)])
% 
% %% [2] : Flight Performance (motor angular velocity, tilting angle, and altitude)
% figure(2);
% set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 0.7 0.4]);
% subplot(1,3,1)
% plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 2,'LineStyle','-'); hold on;
% plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 2,'LineStyle','--'); hold on;
% plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 2,'LineStyle','-'); hold on;
% plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 2,'LineStyle','--'); hold on;
% grid on
% title('Angular velocity')
% ylabel('(rad/s)')
% xlabel('Time (s)')
% 
% subplot(1,3,2)
% plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 2); hold on;
% plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 2, 'LineStyle', '--'); hold on;
% grid on
% title('Tilting angle')
% ylabel('(deg)')
% xlabel('Time (s)')
% 
% subplot(1,3,3)
% plot(outTuned.UAV_State.Xe.Time, -reshape(outTuned.UAV_State.Xe.Data(3,:,:),[size(outTuned.UAV_State.Xe.Data(2,:,:),3),1]), 'Linewidth', 2);
% grid on
% title('Altitude')
% ylabel('(m)')
% xlabel('Time (s)')
% 
% keyboard;
% 
% %% Plot results
% % exampleHelperPlotHoverControlTrackingResults(outTuned);
% % EVTOL_Plots_update(outTu ned)
% 
% 
% % figure
% % hold on
% % plot3(outTuned.UAV_State.Xe.Data(1,:),-outTuned.UAV_State.Xe.Data(2,:),-outTuned.UAV_State.Xe.Data(3,:),'LineWidth',2)
% % plot3([0, 0, 20, 1000, 4000, 8000, 10000, 15000, 17000],-[0, 0, 0, 0, 0, 0, 0, 0, 0],-[0, -100, -100, -150, -1000, -1000, -1000, -100, -100],'LineWidth',1.5,'LineStyle','--')
% % hold off
% % grid on
% % xlabel('North (m)')
% % ylabel('West (m)')
% % zlabel('Up (m)')
% % legend('UAV-3D Trajectory', 'P4-profile (Modified)')
% % ylim([-250, 250])
% % title('eVTOL trajectory (P4 profile)')
% % 
% % v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% % v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
% % v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% % 
% % figure
% % subplot(3,1,1)
% % hold on
% % plot(outTuned.UAV_State.airspeed.Time, outTuned.UAV_State.airspeed.Data, 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylabel('Airspeed (m/s)')
% % title('eVTOL AirData (P4 profile)')
% % subplot(3,1,2)
% % plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylim([-5 35])
% % ylabel('AoA (deg)')
% % subplot(3,1,3)
% % plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylim([-20 20])
% % ylabel('AoS (deg)')
% % xlabel('Time (sec)')
% % 
% % figure
% % subplot(3,1,1)
% % hold on
% % plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylabel('Roll (deg)')
% % ylim([-30 30])
% % title('eVTOL Attitude (P4 profile)')
% % subplot(3,1,2)
% % plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylabel('Pitch (deg)')
% % ylim([-40 40])
% % subplot(3,1,3)
% % plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% % hold off
% % grid on
% % ylabel('Yaw (deg)')
% % ylim([-30 30])
% % xlabel('Time (sec)')
% % 
% % figure
% % subplot(2,1,1)
% % hold on
% % plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 1.5)
% % plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 1.5,'LineStyle','--')
% % hold off
% % grid on
% % ylabel('Rotor SPD (rad/s)')
% % legend('1: front/right','2: front/left')
% % title('eVTOL Rotor Speed (P4 profile)')
% % subplot(2,1,2)
% % hold on
% % plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 1.5)
% % plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 1.5,'LineStyle','--')
% % hold off
% % grid on
% % ylabel('Rotor SPD (rad/s)')
% % legend('3: rear/left','4: rear/right')
% % xlabel('Time (sec)')
% 
% figure
% hold on
% plot3(outTuned.UAV_State.Xe.Data(1,:),-outTuned.UAV_State.Xe.Data(2,:),-outTuned.UAV_State.Xe.Data(3,:),'LineWidth',2)
% % plot3([0, 0, 20, 1000, 4000, 8000, 10000, 15000, 17000],-[0, 0, 0, 0, 0, 0, 0, 0, 0],-[0, -100, -100, -150, -1000, -1000, -1000, -100, -100],'LineWidth',1.5,'LineStyle','--')
% hold off
% grid on
% xlabel('North (m)')
% ylabel('West (m)')
% zlabel('Up (m)')
% % legend('UAV-3D Trajectory', 'P4-profile (Modified)')
% ylim([-250, 250])
% % title('eVTOL trajectory (P4 profile)')
% 
% figure
% ax1(1) = subplot(4,1,1)
% hold on
% plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 1.5, 'LineStyle', '--')
% hold off
% grid on
% ylabel('Tilt (deg)')
% ax1(2) = subplot(4,1,2)
% hold on
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Euler (deg)')
% ax1(3) = subplot(4,1,3)
% hold on
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]), 'Linewidth', 1.5)
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,2,:),3),1]), 'Linewidth', 1.5)
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,3,:),3),1]), 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Vb (m/s)')
% ax1(4) = subplot(4,1,4)
% hold on
% plot(outTuned.UAV_State.RotorParameters.w1.Time, outTuned.UAV_State.RotorParameters.w1.Data, 'Linewidth', 1.5,'LineStyle','-')
% plot(outTuned.UAV_State.RotorParameters.w2.Time, outTuned.UAV_State.RotorParameters.w2.Data, 'Linewidth', 1.5,'LineStyle','--')
% plot(outTuned.UAV_State.RotorParameters.w3.Time, outTuned.UAV_State.RotorParameters.w3.Data, 'Linewidth', 1.5,'LineStyle','-')
% plot(outTuned.UAV_State.RotorParameters.w4.Time, outTuned.UAV_State.RotorParameters.w4.Data, 'Linewidth', 1.5,'LineStyle','--')
% hold off
% grid on
% ylabel('w (rad/s)')
% xlabel('Time (sec)')
% linkaxes(ax1,'x')
% 
% v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
% v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
% 
% figure
% ax2(1) = subplot(4,1,1)
% hold on
% plot(outTuned.UAV_State.airspeed.Time, outTuned.UAV_State.airspeed.Data, 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Air SPD (m/s)')
% ax2(2) = subplot(4,1,2)
% hold on
% plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 1.5)
% plot(outTuned.UAV_State.airspeed.Time, atand(v3), 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('AOA/AOS (deg)')
% ax2(3) = subplot(4,1,3)
% hold on
% plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(3,:,:),[size(outTuned.UAV_State.Xe.Data(3,:,:),3),1]), 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Z (m)')
% ax2(4) = subplot(4,1,4)
% hold on
% plot(outTuned.UAV_State.aileron.Time, outTuned.UAV_State.aileron.Data*const.rad2deg, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.elevator.Time, outTuned.UAV_State.elevator.Data*const.rad2deg, 'Linewidth', 1.5)
% plot(outTuned.UAV_State.rudder.Time, outTuned.UAV_State.rudder.Data*const.rad2deg, 'Linewidth', 1.5)
% hold off
% grid on
% ylim([-30 30])
% legend('AIL','ELE','RUD')
% ylabel('CtrlSurf (deg)')
% linkaxes(ax2,'x')
% 
% figure
% ax3(1) = subplot(4,1,1)
% hold on
% plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(1,:,:),[size(outTuned.UAV_State.Xe.Data(1,:,:),3),1]), 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('X (m)')
% ax3(2) = subplot(4,1,2)
% hold on
% plot(outTuned.UAV_State.Xe.Time, reshape(outTuned.UAV_State.Xe.Data(2,:,:),[size(outTuned.UAV_State.Xe.Data(2,:,:),3),1]), 'Linewidth', 1.5)
% hold off
% grid on
% ylabel('Y (m)')
% ylim([-5, 5])
% ax3(3) = subplot(4,1,3)
% hold on
% hold off
% grid on
% ylabel('')
% ax3(4) = subplot(4,1,4)
% hold on
% hold off
% grid on
% ylabel('')
% linkaxes(ax3,'x')

