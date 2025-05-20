%Set TestMode to guidance
TestMode=1;
% Setup Hover configuration.
FSState=flightState.Hover;

%Set guidance type to execute hover mission.
guidanceType=2;

%% Setup Hover Mission
HoverMission(1)=struct('mode',1,'position',[0,0,0]','params',[0;0;0;0]);
HoverMission(2)=struct('mode',2,'position',[0,0,-20]','params',[0;0;0;0]);
HoverMission(3)=struct('mode',2,'position',[0,-40,-20]','params',[0;0;0;0]);
HoverMission(4)=struct('mode',2,'position',[20,-40,-20]','params',[0;0;0;0]);
HoverMission(5)=struct('mode',2,'position',[60,-40,-20]','params',[0;0;0;0]);
HoverMission(6)=struct('mode',2,'position',[60,40,-20]','params',[0;0;0;0]);
HoverMission(7)=struct('mode',2,'position',[80,40,-20]','params',[0;0;0;0]);  
HoverMission(8)=struct('mode',2,'position',[80,-40,-20]','params',[0;0;0;0]);  
HoverMission(9)=struct('mode',2,'position',[100,-40,-20]','params',[0;0;0;0]);  
HoverMission(10)=struct('mode',2,'position',[100,40,-20]','params',[0;0;0;0]);  
HoverMission(11)=struct('mode',4,'position',[100,40,0]','params',[0;0;0;0]);
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '11')

%Set waypoint guidance parameters
R_WAYPOINTTRANSITION=1;
R_LOOKAHEAD=5;

