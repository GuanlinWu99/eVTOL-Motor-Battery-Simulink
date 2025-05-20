%exampleHelperAutomatedFixedWingTuning tune all control gains for fixed
%wing mode.

% Copyright 2023 The MathWorks, Inc.

% Model Name
mdl = 'VTOL_Tiltrotor';

%Define the aircraft modes of flight. 
Simulink.defineIntEnumType('flightState',...
{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3]);

%Set Manual control mode to fixed wing.
set_param([mdl '/Guidance Test Bench/ControlType/Manual Control/Slider4'],'Value','2');

%Disable QGC
useQGC=0;

%Define mode to enable or disable tuning
TuningMode=0;

%Set Mission to Fixed Wing
guidanceType=0;

%Set attitude to be 0.
iniRoll=0;
iniYaw=0;
initPitch=0;

%Define Bus interfaces for controller
exampleHelperDefineCtrlInterface;

%Define Plant bus interface.
exampleHelperDefineDigitalTwinInterface;

%Set VTOL Dynamics:Aerodynamics and Geometry Parameters
uavParam=exampleHelperSetVTOLDynamics;

%Disable Sensors
SensorType=0;

%Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,100)