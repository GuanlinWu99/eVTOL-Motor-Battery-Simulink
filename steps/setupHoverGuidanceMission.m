% Setup Hover configuration.
FSState=flightState.Hover;
%Set Visualization on.
Visualization=0;
%Set Test Bench to execute complex guidance mission
TestMode=1;
%Set guidance type to execute hover mission.
guidanceType=2;
%Sensors are disabled
SensorType=0;

% Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,560)

%% Setup Hover Mission (오리지널)
% HoverMission(1)=struct('mode',1,'position',[0,0,0]','params',[0;0;0;0]);
% HoverMission(2)=struct('mode',2,'position',[0,0,-20]','params',[0;0;0;0]);
% HoverMission(3)=struct('mode',2,'position',[0,-40,-20]','params',[0;0;0;0]);
% HoverMission(4)=struct('mode',2,'position',[20,-40,-20]','params',[0;0;0;0]);
% HoverMission(5)=struct('mode',2,'position',[60,-40,-20]','params',[0;0;0;0]);
% HoverMission(6)=struct('mode',2,'position',[60,40,-20]','params',[0;0;0;0]);
% HoverMission(7)=struct('mode',2,'position',[80,40,-20]','params',[0;0;0;0]);  
% HoverMission(8)=struct('mode',2,'position',[80,-40,-20]','params',[0;0;0;0]);  
% HoverMission(9)=struct('mode',2,'position',[100,-40,-20]','params',[0;0;0;0]);  
% HoverMission(10)=struct('mode',2,'position',[100,40,-20]','params',[0;0;0;0]);  
% HoverMission(11)=struct('mode',4,'position',[100,40,0]','params',[0;0;0;0]);

HoverMission = struct;
HoverMission(1).mode = 1;
HoverMission(1).position = [0; 0; 0];
HoverMission(1).params = [0; 0; 0; 0];

HoverMission(2).mode = 2;
HoverMission(2).position = [0; 0; -40];
HoverMission(2).params = [0; 0; 0; 0];

HoverMission(3).mode = 2;
HoverMission(3).position = [20; 0; -40];
HoverMission(3).params = [0; 0; 0; 0];

HoverMission(4).mode = 2;
HoverMission(4).position = [100; 0; -40];
HoverMission(4).params = [0; 0; 0; 0];

HoverMission(5).mode = 2;
HoverMission(5).position = [400; 0; -80];
HoverMission(5).params = [0; 0; 0; 0];

HoverMission(6).mode = 2;
HoverMission(6).position = [1000; 0; -80];
HoverMission(6).params = [0; 0; 0; 0];

HoverMission(7).mode = 2;
HoverMission(7).position = [1500; 0; -80];
HoverMission(7).params = [0; 0; 0; 0];

HoverMission(8).mode = 2;
HoverMission(8).position = [2000; 0; -80];
HoverMission(8).params = [0; 0; 0; 0];

HoverMission(9).mode = 2;
HoverMission(9).position = [2400; 0; -40];
HoverMission(9).params = [0; 0; 0; 0];

HoverMission(10).mode = 4;
HoverMission(10).position = [2400; 0; -20];
HoverMission(10).params = [0; 0; 0; 0];

HoverMission(11).mode = 4;
HoverMission(11).position = [2400; 0; 0];
HoverMission(11).params = [0; 0; 0; 0];

load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', 'length(HoverMission)');

%Set waypoint guidance parameters
R_WAYPOINTTRANSITION=1;
R_LOOKAHEAD=5;
disp("Enabled hover guidance mission.")









