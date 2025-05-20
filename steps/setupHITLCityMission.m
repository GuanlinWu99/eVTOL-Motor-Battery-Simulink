%Set TestMode to guidance
TestMode=1;
% Setup Hover configuration.
FSState=flightState.Hover;

%Set initial position to ground location in local coordinates (NED)
xGround=0;
yGround=0;
zGround=0;

%Set guidance type to execute hover mission.
guidanceType=5;

%City Mission
CityMission = struct;
CityMission(1).mode = 1;
CityMission(1).position = [xGround; yGround; zGround];
CityMission(1).params = [0; 0; 0; zGround];
CityMission(2).mode = 2;
CityMission(2).position = [0; 0; -30];
CityMission(2).params = [0; 0; 0; 0];
CityMission(3).mode = 2;
CityMission(3).position = [140; 0; -30];
CityMission(3).params = [0; 0; 0; 0];
CityMission(4).mode = 2;
CityMission(4).position = [140; 35; -30];
CityMission(4).params = [0; 0; 0; 0];
CityMission(5).mode = 4;
CityMission(5).position = [140; 35;-0.4];
CityMission(5).params = [0; 0; 0; 0];
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '5')

%Set waypoint guidance parameters
R_WAYPOINTTRANSITION=1;
R_LOOKAHEAD=5;

disp("Initialized city mission.")