%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: start up script for simulator                                   %
% author(s): minhyun                                                     %
% description:                                                           %
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

%.. define the aircraft modes of flight
Simulink.defineIntEnumType('flightState',...
{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3]);

%.. initialize simulator: velocity defined later
xGround     =   0;
yGround     =   0;
zGround     =   0;
iniRoll     =   0;
iniYaw      =   0;
initPitch   =   0;
iniP        =   0;
iniQ        =   0;
iniR        =   0;

%.. initialize landing gear model
load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, ...
%                  'slidingFriction', 0.8, 'rollingFriction', 0.2, ...
%                  'gLimit', 100);

%.. load bus interfaces for controller
define_ctrl_interface;

%.. load bus interfaces for plant
define_digital_twin_interface;

%.. set up vtol dynamics parameters
uavParam    =   load_vtol_dynamics_7000lb;

% Flag to enable/disable visualization
Visualization = 1;
% Disable Wind
Wind=0;
% Disable Sensors
SensorType=0;
% Setup tuning flag
TuningMode = 0;
Deployment = false;
% Initialize Control and Guidance gains for Tiltrotor
exampleHelperInitializeVTOLGains_m;
% Initialize initial velocity
vIni = 0;
disp("Initialized VTOL model.")
% Initialize hover configuration
setupHoverConfiguration
setupHoverGuidanceMission_mod

% Setup configuration set
configObj = getActiveConfigSet('VTOLAutopilotController');
set_param(configObj, 'SourceName', 'VTOLConfiguration');

%% Run SIMULINK
outTuned = sim(mdl);

%% Plot results
exampleHelperPlotHoverControlTrackingResults(outTuned);
EVTOL_Plots_update(outTuned)