%exampleHelperTransitionMissionData Define a transition mission.

% Copyright 2023 The MathWorks, Inc.

TransitionMission = struct;
TransitionMission(1).mode = 1;
TransitionMission(1).position = [0; 0; -50*const.ft2m];
TransitionMission(1).params = [0; 0; 0; 0];
TransitionMission(2).mode = 2;
TransitionMission(2).position = [0; 0; -200];
TransitionMission(2).params = [0; 0; 0; 0];
TransitionMission(3).mode = 2;
TransitionMission(3).position = [20; 0; -200];
TransitionMission(3).params = [0; 0; 0; 0];
TransitionMission(4).mode = 6;
TransitionMission(4).position = [1000; 0; -200];
TransitionMission(4).params = [1; 1; 1; 1];
TransitionMission(5).mode = 2;
TransitionMission(5).position = [2000; 0; -300];
TransitionMission(5).params = [0; 0; 2; 75*const.kts2mps];
TransitionMission(6).mode = 2;
TransitionMission(6).position = [2700; 0; -200];
TransitionMission(6).params = [0; 0; 1; 75*const.kts2mps];
% TransitionMission(7).mode = 2;
% TransitionMission(7).position = [8200; 100; -2200];
% TransitionMission(7).params = [0; 0; 0; 65*const.kts2mps];

TransitionMission(7).mode = 6;
TransitionMission(7).position = [3700; 0; -200];
TransitionMission(7).params = [1; 1; 1; 1];
TransitionMission(8).mode = 2;
TransitionMission(8).position = [4200; 0; -200];
TransitionMission(8).params = [0; 0; 0; 0];
TransitionMission(9).mode = 2;
TransitionMission(9).position = [4200; 0; -30];
TransitionMission(9).params = [0; 0; 0; 0];
TransitionMission(10).mode = 4;
TransitionMission(10).position = [4200; 0; 0];
TransitionMission(10).params = [0; 0; 0; 0];

% TransitionMission(6).mode = 2;
% TransitionMission(6).position = [2000; 0; -300];
% TransitionMission(6).params = [0; 0; 0; 0];
% TransitionMission(7).mode = 2;
% TransitionMission(7).position = [10000; 0; -1000];
% TransitionMission(7).params = [0; 0; 0; 0];
% TransitionMission(8).mode = 2;
% TransitionMission(8).position = [15000; 0; -100];
% TransitionMission(8).params = [0; 0; 0; 0];
% TransitionMission(9).mode = 2;
% TransitionMission(9).position = [17000; 0; -100];
% TransitionMission(9).params = [0; 0; 0; 0];

% TransitionMission(6).mode = 3;
% TransitionMission(6).position = [1000; 200; -100];
% TransitionMission(6).params = [50; -1; 0.5; 0];
% TransitionMission(10).mode = 6;
% TransitionMission(10).position = [-1; -1; -1];
% TransitionMission(10).params = [-1; -1; -1;-1];
% TransitionMission(8)=struct('mode',2,'position',[300,300,-20]','params',[0;0;0;0]);
% TransitionMission(9)=struct('mode',4,'position',[300,300,0]','params',[0;0;0;0]);
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', 'length(TransitionMission)')

