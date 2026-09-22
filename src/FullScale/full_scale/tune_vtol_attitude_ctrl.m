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

%.. load the model
mdl     =   'vtol_dynamics_trim';
load_system(mdl);

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

%.. load bus interfaces for plant
define_digital_twin_interface;

%.. set up vtol dynamics parameters
uavParam    =   load_vtol_dynamics_7000lb;

%.. initialize initial velocity
vIni        =   0;

%.. disable wind
Wind        =   0;

minPWM=0.1;
% tilt_max=pi/4;

%Get initial velocity and tilt based on flight mode.
tiltIni= 0;