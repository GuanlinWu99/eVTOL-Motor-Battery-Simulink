% Setup Hover configuration.
FSState=flightState.Hover;
%Get initial velocity and tilt based on flight mode.
[vIni,tiltIni]=exampleHelperGetInitialConfiguration(FSState);
%Load in Hover tuned gains
load tunedHoverGains_BW50.mat;
%Load in Fixed wing tuned gains
load tunedFixedWingGains_BW10.mat

%Set initial position to ground location in local coordinates (NED)
xGround=0;
yGround=0;
zGround=-0.4;
% Setup Hover configuration.
FSState=flightState.Hover;

% Set Test Bench to execute complex guidance mission
TestMode=1;

% Set guidance type to execute city mission.
guidanceType=5;

% Enable Photorealistic Visualization
Visualization = 2;

%Disable Sensors
SensorType=0;

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

disp("Initialized city mission.")