%exampleHelperTransitionMissionData Define a transition mission.

% Copyright 2023 The MathWorks, Inc.

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
TransitionMission(4).position = [1;1;1];
TransitionMission(4).params = [1; 1; 1; 1];
TransitionMission(5).mode = 2;
TransitionMission(5).position = [100; 0; -20];
TransitionMission(5).params = [0; 0; 0; 0];
TransitionMission(6).mode = 3;
TransitionMission(6).position = [100; 200; -20];
TransitionMission(6).params = [50; -1; 0.5; 0];
TransitionMission(7).mode = 6;
TransitionMission(7).position = [-1; -1; -1];
TransitionMission(7).params = [-1; -1; -1;-1];
TransitionMission(8)=struct('mode',2,'position',[300,300,-20]','params',[0;0;0;0]);
TransitionMission(9)=struct('mode',4,'position',[300,300,0]','params',[0;0;0;0]);
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '9')

