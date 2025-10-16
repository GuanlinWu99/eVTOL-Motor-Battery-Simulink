
% Set Test Bench to execute complex guidance mission
TestMode      = 1;

% Set guidance type to execute transition mission 
guidanceType  = 3;

% Enable Visualization
Visualization = 1;

% Disable Sensors
SensorType    = 0;

% Load in Transition Misison
% exampleHelperTransitionMissionData_mod;
P4mission;

disp("Enabled transition guidance mission.")

% Open data dictionary
% myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
% dDataSectObj = getSection(myDictionaryObj,'Design Data');

% Set Time for Simulation
% Total_sim_time  = 450;
Total_sim_time  = 600;
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj    = getSection(myDictionaryObj,'Design Data');
simTimeParam    = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,Total_sim_time)

