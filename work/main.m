clear all
close all
clc

%%% This simulation code is for EVTOL mass 12kg and 
%%% flight mode is Hover mode (flight mechanism is similar to drone)


%% Setup EVTOL
setupPlant;

%% Load updated EVTOL geometry and its corresponding flight control gains
load('VTOLTuning.mat');
uavParam.geom.mass         =  VTOL_Geometry.Mass;
uavParam.geom.PropDiameter =  VTOL_Geometry.PropDiameter;
CT                         =  VTOL_Geometry.CT;
CQ                         =  VTOL_Geometry.CQ;
number_of_blades           =  VTOL_Geometry.number_of_blades;

%% Setup flight profiles
setupHoverConfiguration;
setupHoverGuidanceMission;

%% Run SIMULINK
outTuned = sim(mdl);

%% Plot results
exampleHelperPlotHoverControlTrackingResults(outTuned);
EVTOL_Plots(outTuned)