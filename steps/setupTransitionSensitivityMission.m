% Setup Hover configuration.
FSState=flightState.Hover;

% Set Test Bench to execute complex guidance mission
TestMode=1;

% Set guidance type to execute fixed wing mission.
guidanceType=3;

% Enable Visualization
Visualization = 1;

%Disable Sensors
SensorType=0;

%Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,30)

%Define a short transition mission
TransitionMission = struct;
TransitionMission(1).mode = 1;
TransitionMission(1).position = [0; 0; 0];
TransitionMission(1).params = [0; 0; 0; 0];
TransitionMission(2).mode = 2;
TransitionMission(2).position = [0; 0; -20];
TransitionMission(2).params = [0; 0; 0; 0];
TransitionMission(3).mode = 2;
TransitionMission(3).position = [20; 0; -20];
TransitionMission(3).params = [0; 0; 0; 0];
TransitionMission(4).mode = 6;
TransitionMission(4).position = [0; 0; 0];
TransitionMission(4).params = [1; 2; 3; 4];
TransitionMission(5).mode = 2;
TransitionMission(5).position = [100; 0; -20];
TransitionMission(5).params = [0; 0; 0; 0];
TransitionMission(6).mode = 3;
TransitionMission(6).position = [100; 200; -50];
TransitionMission(6).params = [50; -1; 3; 0];
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '6')

disp("Enabled transition sensitivity mission.")