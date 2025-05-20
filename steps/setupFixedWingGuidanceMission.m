% Setup Fixed Wing configuration.
FSState=flightState.FixedWing;

% Set Test Bench to execute complex guidance mission
TestMode=1;

% Set guidance type to execute fixed wing mission.
guidanceType=1;

% Enable Visualization
Visualization = 1;

%Disable Sensors
SensorType=0;

% Setup Fixed Wing Mission
FixedWingMission(1)=struct('mode',2,'position',[250,0,-20]','params',[0;0;0;0]);
FixedWingMission(2)=struct('mode',2,'position',[150,-50,-20]','params',[0;3*pi/4;0;0]);
FixedWingMission(3)=struct('mode',2,'position',[50,-100,-40]','params',[0;pi/2;0;0]);
FixedWingMission(4)=struct('mode',2,'position',[150,-150,-40]','params',[0;pi/4;0;0]);
FixedWingMission(5)=struct('mode',2,'position',[250,-200,-20]','params',[0;0;0;0]);
FixedWingMission(6)=struct('mode',3,'position',[250,-250,-20]','params',[50;-1;3;0]);
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '6')

disp("Enabled fixed-wing guidance mission.")