
%% This is a code for P4 mission profile

% mode = 1 : Takeoff,    Supported Flight Mode : Hover
% mode = 2 : Waypoint,   Supported Flight Mode : Hover, Fixed Fing
% mode = 4 : Land,       Supported Flight Mode : Hover
% mode = 6 : Transition, Supported Flight Mode : Hover, Fixed Fing

% For more details, please refer to 
% https://www.mathworks.com/help/uav/ug/visualize-PX4-HITL-with-VTOL-UAV-in-urban-environment.html
% - sign of the altitude means "going up"

% 1 nautical mile (nm) : 1.15 miles

% TransitionMission              = struct;
% TransitionMission(1).mode      = 1;
% TransitionMission(1).position  = [0; 0; -30*const.ft2m];
% TransitionMission(1).params    = [0; 0; 0; 0];
% 
% TransitionMission(2).mode      = 2;
% TransitionMission(2).position  = [0; 0; -300*const.ft2m];
% TransitionMission(2).params    = [0; 0; 0; 0];
% 
% TransitionMission(3).mode      = 2;
% TransitionMission(3).position  = [0.2*const.nm2m; 0; -350*const.ft2m];
% TransitionMission(3).params    = [0; 0; 0; 0];
% 
% TransitionMission(4).mode      = 2;
% TransitionMission(4).position  = [(0.2+0.1)*const.nm2m; 0; -350*const.ft2m];
% TransitionMission(4).params    = [0; 0; 0; 0];
% 
% TransitionMission(5).mode      = 4;
% TransitionMission(5).position  = [(0.2+0.1)*const.nm2m; 0; 0];
% TransitionMission(5).params    = [0; 0; 0; 0];

TransitionMission              = struct;
TransitionMission(1).mode      = 1;
TransitionMission(1).position  = [0; 0; -50*const.ft2m];
TransitionMission(1).params    = [0; 0; 0; 0];

TransitionMission(2).mode      = 2;
TransitionMission(2).position  = [0; 0; -300*const.ft2m];
TransitionMission(2).params    = [0; 0; 0; 0];

TransitionMission(3).mode      = 6;
TransitionMission(3).position  = [0.5*0.62*const.nm2m; 0; -300*const.ft2m];
TransitionMission(3).params    = [1; 1; 1; 1];

TransitionMission(4).mode      = 2;
TransitionMission(4).position  = [0.62*const.nm2m; 0; -500*const.ft2m];
TransitionMission(4).params    = [0; 0; 0; 75*const.kts2mps];

TransitionMission(5).mode      = 2;
TransitionMission(5).position  = [(3.76+0.62)*const.nm2m; 0; -3000*const.ft2m];
TransitionMission(5).params    = [0; 0; 0; 90*const.kts2mps];

TransitionMission(6).mode      = 2;
TransitionMission(6).position  = [(3.76+0.62+1.2)*const.nm2m; 0; -3000*const.ft2m];
TransitionMission(6).params    = [0; 0; 0; 75*const.kts2mps];

TransitionMission(7).mode      = 2;
TransitionMission(7).position  = [(3.76+0.62+1.2+2.61)*const.nm2m; 0; -900*const.ft2m];
TransitionMission(7).params    = [0; 0; 0; 62*const.kts2mps];

TransitionMission(8).mode      = 2;
TransitionMission(8).position  = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m-1100; 0; -900*const.ft2m];
TransitionMission(8).params    = [0; 0; 0; 62*const.kts2mps];

TransitionMission(9).mode      = 6;
TransitionMission(9).position  = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m-550; 1; 1];
TransitionMission(9).params    = [1; 1; 1; 1];

TransitionMission(10).mode     = 2;
TransitionMission(10).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -450*const.ft2m];
TransitionMission(10).params   = [0; 0; 0; 0];

TransitionMission(11).mode     = 2;
TransitionMission(11).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -200*const.ft2m];
TransitionMission(11).params   = [0; 0; 0; 0];

TransitionMission(12).mode     = 4;
TransitionMission(12).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -30*const.ft2m];
TransitionMission(12).params   = [0; 0; 0; 0];

load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', 'length(TransitionMission)')