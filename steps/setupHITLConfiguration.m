% load contact data and units
load("data\contact.mat");
load("data\SIUnitData.mat");

% Sample time 
SampleTime = 0.01;
%% Environmental effects

%Wind model
Wind=0;

% Enable ground model
Groundon = 1;

% Disable data logging
Deployment = true;
%% Initial Pose
%Set attitude to be 0.
iniRoll=0;
iniYaw=0;
initPitch=0;
%% VTOL Dynamics
%Set VTOL Dynamics:Aerodynamics and Geometry Parameters
uavParam=exampleHelperSetVTOLDynamics;
%%Initialize Control and Guidance gains for Tiltrotor
%% Hover Guidance Controls
R_WAYPOINTTRANSITION=1;
R_LOOKAHEAD=5;

