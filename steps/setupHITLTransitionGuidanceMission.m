%Tuning Mode: Control Tuning
TuningMode = 0;
Deployment = true;

% Set Test Bench to execute complex guidance mission
% Variant switch for 'VTOLAutopilotController/Guidance Test Bench/ControlType'
TestMode=1;

% Set guidance type to execute transition mission.
% Variant swtich for 'HITL_Controller_top/noQGC/Variant Source'
guidanceType=3;

%Load in Transition Misison
exampleHelperTransitionMissionData;
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '9');

%Open data dictionary
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
%Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,200)
