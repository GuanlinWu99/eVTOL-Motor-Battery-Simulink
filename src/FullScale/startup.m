
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: start up script for simulator                                   %
% Author(s): Minhyun, Sounghwan, and Guanlin                             %
% Description and updates (09/12/2025):                                  %
% 1. Include drag torque calculation in the powertrain                   %
% 2. Upgrade battery pack in the powertrain                              %
% 3. Changed from Variable-Type to Fixed-Type Ts = 0.001 (09/26/2025)    %
% 4. Included flaps and spoilers (11/13/2025)                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%... Clear workspace, command window and close figures
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
set_param(cfgTop,'SolverType','Fixed-step','Solver','ode4');
Ts           = 0.001;
mdlSub       = 'VTOLDynamics';
load_system(mdlSub);
cfgSubActive = getActiveConfigSet(mdlSub);

% if isa(cfgSubActive, 'Simulink.ConfigSetRef')
%     cfgSub = getRefConfigSet(cfgSubActive);  
% else
%     cfgSub = cfgSubActive;                   
% end

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

%%
%... Load controllercontrolParams.TiltScheduleRate parameters
controlParams          =   load_controller_parameters(uavParams, const);

%... Flag to enable/disable visualization
Visualization          =   1;

%... Disable Wind
Wind                   =   0;

if Wind == 1
    Wind_Speed         =   5;
elseif Wind == 0
    Wind_Speed         =   0;
else
    Wind_Speed         =   0;
end

%... Disable Sensors
SensorType             =   0;

%... Setup tuning flag
TuningMode             =   0;
Deployment             =   false;

%... Initialize Control and Guidance gains for Tiltrotor
load_controller_gains;

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

%Back_Transition_Rate = 0.5*controlParams.FWDTiltScheduleRate;
%fprintf('Forward TiltAngle Rate %.2f [deg/s]\n', controlParams.FWDTiltScheduleRate*57.295);
%fprintf('Backward TiltAngle Rate %.2f [deg/s]\n', controlParams.BWDTiltScheduleRate*57.295);

%exampleHelperAutomatedHoverControlTuning;

Flap_Activate  =  15000;
Fading_time    =  0.1;

%%... Run SIMULINK

disp(['✅ Total Simulation Time: ', num2str(Total_sim_time), ' (s)']);
disp(['✅ Flight Profile: P', num2str(Profile)]);
disp(['✅ Cross-Wind Speed: ', num2str(Wind_Speed), ' (m/s)']);

keyboard;

tic
outTuned = sim(mdl);
toc

%%... Plot figures
% Simulation_Plot();



