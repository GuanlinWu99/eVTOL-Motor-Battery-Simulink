% Set Test Bench to execute complex guidance mission
TestMode=1;

% Set guidance type to execute fixed wing mission.
guidanceType=3;

% Enable Visualization
Visualization = 1;

%Disable Sensors
SensorType=0;

%Load in Transition Misison
exampleHelperTransitionMissionData;

disp("Enabled transition guidance mission.")

%Open data dictionary
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
%Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,200)
