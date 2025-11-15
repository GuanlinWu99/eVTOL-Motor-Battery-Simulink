
%% This is a code for P4 mission profile

% mode = 1 : Takeoff,    Supported Flight Mode : Hover
% mode = 2 : Waypoint,   Supported Flight Mode : Hover, Fixed Fing
% mode = 4 : Land,       Supported Flight Mode : Hover
% mode = 6 : Transition, Supported Flight Mode : Hover, Fixed Fing

% For more details, please refer to 
% https://www.mathworks.com/help/uav/ug/visualize-PX4-HITL-with-VTOL-UAV-in-urban-environment.html
% - sign of the altitude means "going up"

% 1 nautical mile (nm) : 1.15 miles

% Mode = 1;
% 
% Mode = 0 : Only hover mode
% Mode = 1 : Transition mode
% Mode = 2 : Prototype P4
% 
% switch Mode
%     case 0
%         TransitionMission              = struct;
%         TransitionMission(1).mode      = 1;
%         TransitionMission(1).position  = [0; 0; -30*const.ft2m];
%         TransitionMission(1).params    = [0; 0; 0; 0];
% 
%         TransitionMission(2).mode      = 2;
%         TransitionMission(2).position  = [0; 0; -300*const.ft2m];
%         TransitionMission(2).params    = [0; 0; 0; 0];
% 
%         TransitionMission(3).mode      = 2;
%         TransitionMission(3).position  = [0.2*const.nm2m; 0; -350*const.ft2m];
%         TransitionMission(3).params    = [0; 0; 0; 0];
% 
%         TransitionMission(4).mode      = 2;
%         TransitionMission(4).position  = [(0.2+0.1)*const.nm2m; 0; -350*const.ft2m];
%         TransitionMission(4).params    = [0; 0; 0; 0];
% 
%         TransitionMission(5).mode      = 4;
%         TransitionMission(5).position  = [(0.2+0.1)*const.nm2m; 0; 0];
%         TransitionMission(5).params    = [0; 0; 0; 0];
% 
%     case 1   
%         TransitionMission              = struct;
%         %% Hover Mode
%         TransitionMission(1).mode      = 1;
%         TransitionMission(1).position  = [0; 0; -50*const.ft2m];
%         TransitionMission(1).params    = [0; 0; 0; 0];
% 
%         TransitionMission(2).mode      = 2;
%         TransitionMission(2).position  = [0; 0; -300*const.ft2m];
%         TransitionMission(2).params    = [0; 0; 0; 0];
% 
%         %% Forward transition
%         TransitionMission(3).mode      = 6;
%         TransitionMission(3).position  = [0.5*0.62*const.nm2m; 0; -300*const.ft2m];
%         TransitionMission(3).params    = [1; 1; 1; 1];
% 
%          %% Fixed-Wing Mode       
%         TransitionMission(4).mode      = 2;
%         TransitionMission(4).position  = [0.62*const.nm2m; 0; -500*const.ft2m];
%         TransitionMission(4).params    = [0; 0; 0; 75*const.kts2mps];
% 
%         TransitionMission(5).mode      = 2;
%         TransitionMission(5).position  = [(3.76+0.62)*const.nm2m; 0; -3000*const.ft2m];
%         TransitionMission(5).params    = [0; 0; 0; 90*const.kts2mps];
% 
%         TransitionMission(6).mode      = 2;
%         TransitionMission(6).position  = [(3.76+0.62+1.2)*const.nm2m; 0; -3000*const.ft2m];
%         TransitionMission(6).params    = [0; 0; 0; 75*const.kts2mps];
% 
%         TransitionMission(7).mode      = 2;
%         TransitionMission(7).position  = [(3.76+0.62+1.2+2.61)*const.nm2m; 0; -1500*const.ft2m];
%         TransitionMission(7).params    = [0; 0; 0; 55*const.kts2mps];
% 
%         TransitionMission(8).mode      = 2;
%         TransitionMission(8).position  = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m-1100; 0; -1500*const.ft2m];
%         TransitionMission(8).params    = [0; 0; 0; 55*const.kts2mps];
% 
%         %% Back transition
%         TransitionMission(9).mode      = 6;
%         TransitionMission(9).position  = [(3.76+0.62+1.2+2.61+1.5 + 0.1)*const.nm2m-0*550; 0; -1500*const.ft2m];
%         TransitionMission(9).params    = [1; 1; 1; 1];
% 
%         %% Hover Mode
%         TransitionMission(10).mode     = 2;
%         TransitionMission(10).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -900*const.ft2m];
%         TransitionMission(10).params   = [0; 0; 0; 0];
% 
%         TransitionMission(11).mode     = 2;
%         TransitionMission(11).position = [(3.76+0.62+1.2+2.61+1.5+0.5)*const.nm2m; 0; -450*const.ft2m];
%         TransitionMission(11).params   = [0; 0; 0; 0];
% 
%         TransitionMission(12).mode     = 2;
%         TransitionMission(12).position = [(3.76+0.62+1.2+2.61+1.5+0.5)*const.nm2m; 0; -100*const.ft2m];
%         TransitionMission(12).params   = [0; 0; 0; 0];
% 
%         TransitionMission(13).mode     = 4;
%         TransitionMission(13).position = [(3.76+0.62+1.2+2.61+1.5+0.5)*const.nm2m; 0; 0];
%         TransitionMission(13).params   = [0; 0; 0; 0];
% 
%     case 2
%         %% Original 3 (latest long version)
%         TransitionMission               = struct;
% 
%         % Vertical takeoff
%         TransitionMission(1).mode       = 1;
%         TransitionMission(1).position   = [0; 0; -50*const.ft2m];
%         TransitionMission(1).params     = [0; 0; 0; 0];
% 
%         % Hover
%         TransitionMission(2).mode       = 2;
%         TransitionMission(2).position   = [0; 0; -200];
%         TransitionMission(2).params     = [0; 0; 0; 0];
% 
%         TransitionMission(3).mode       = 2;
%         TransitionMission(3).position   = [20; 0; -200];
%         TransitionMission(3).params     = [0; 0; 0; 0];
% 
%         % Forward transition
%         TransitionMission(4).mode       = 6;
%         TransitionMission(4).position   = [1000; 0; -200];
%         TransitionMission(4).params     = [1; 1; 1; 1];
% 
%         % Fixed-wing (climb)
%         TransitionMission(5).mode       = 2;
%         TransitionMission(5).position   = [8000; 0; -500];
%         TransitionMission(5).params     = [0; 0; 0; 80*const.kts2mps];
% 
%         TransitionMission(6).mode       = 2;
%         TransitionMission(6).position   = [13000; 0; -400];
%         TransitionMission(6).params     = [0; 0; 0; 70*const.kts2mps];
% 
%         TransitionMission(7).mode       = 2;
%         TransitionMission(7).position   = [18000; 0; -400];
%         TransitionMission(7).params     = [0; 0; 0; 70*const.kts2mps];
% 
%         TransitionMission(8).mode       = 2;
%         TransitionMission(8).position   = [22000; 0; -400];
%         TransitionMission(8).params     = [0; 0; 0; 70*const.kts2mps];
% 
%         % Back Transition 
%         TransitionMission(9).mode       = 6;
%         TransitionMission(9).position   = [22000; 0; -400];
%         TransitionMission(9).params     = [1; 1; 1; 1];
% 
%         % Hover mode
%         TransitionMission(10).mode      = 2;
%         TransitionMission(10).position  = [22500; 0; -100];
%         TransitionMission(10).params    = [1; 1; 1; 1];
% 
%         TransitionMission(11).mode      = 2;
%         TransitionMission(11).position  = [22500; 0; -30];
%         TransitionMission(11).params    = [1; 1; 1; 1];
% 
%         TransitionMission(12).mode      = 4;
%         TransitionMission(12).position  = [22500; 0; 0];
%         TransitionMission(12).params    = [1; 1; 1; 1];
% 
%         %% Original 2 (long version)
%         % TransitionMission               = struct;
%         % 
%         % % Vertical takeoff
%         % TransitionMission(1).mode       = 1;
%         % TransitionMission(1).position   = [0; 0; -50*const.ft2m];
%         % TransitionMission(1).params     = [0; 0; 0; 0];
%         % 
%         % % Hover
%         % TransitionMission(2).mode       = 2;
%         % TransitionMission(2).position   = [0; 0; -200];
%         % TransitionMission(2).params     = [0; 0; 0; 0];
%         % 
%         % TransitionMission(3).mode       = 2;
%         % TransitionMission(3).position   = [20; 0; -200];
%         % TransitionMission(3).params     = [0; 0; 0; 0];
%         % 
%         % % Forward transition
%         % TransitionMission(4).mode       = 6;
%         % TransitionMission(4).position   = [1000; 0; -200];
%         % TransitionMission(4).params     = [1; 1; 1; 1];
%         % 
%         % % Fixed-wing
%         % TransitionMission(5).mode       = 2;
%         % TransitionMission(5).position   = [2000; 0; -500];
%         % TransitionMission(5).params     = [0; 0; 0; 70*const.kts2mps];
%         % 
%         % % Fixed-wing
%         % TransitionMission(6).mode       = 2;
%         % TransitionMission(6).position   = [2000+3000; 0; -500];
%         % TransitionMission(6).params     = [0; 0; 0; 70*const.kts2mps];
%         % 
%         % % Fixed-wing 
%         % TransitionMission(7).mode       = 2;
%         % TransitionMission(7).position   = [4700+3000; 0; -500];
%         % TransitionMission(7).params     = [0; 0; 0; 60*const.kts2mps];
%         % 
%         % % Back transition
%         % TransitionMission(8).mode       = 6;
%         % TransitionMission(8).position   = [6000+3000; 0; -500];
%         % TransitionMission(8).params     = [1; 1; 1; 1];
%         % 
%         % % Hover
%         % TransitionMission(9).mode       = 2;
%         % TransitionMission(9).position   = [6000+3500; 0; -400];
%         % TransitionMission(9).params     = [0; 0; 0; 0];
%         % 
%         % % Hover
%         % TransitionMission(10).mode       = 2;
%         % TransitionMission(10).position   = [6000+3500; 0; -150];
%         % TransitionMission(10).params     = [0; 0; 0; 0];
%         % 
%         % % Hover
%         % TransitionMission(11).mode      = 2;
%         % TransitionMission(11).position  = [6000+3500; 0; -30];
%         % TransitionMission(11).params    = [0; 0; 0; 0];
%         % 
%         % % Vertical landing
%         % TransitionMission(12).mode      = 4;
%         % TransitionMission(12).position  = [6000+3500; 0; 0];
%         % TransitionMission(12).params    = [0; 0; 0; 0];
% 
%         %% Original (short version)
%         % TransitionMission              = struct;
%         % 
%         % TransitionMission(1).mode      = 1;
%         % TransitionMission(1).position  = [0; 0; -50*const.ft2m];
%         % TransitionMission(1).params    = [0; 0; 0; 0];
%         % 
%         % TransitionMission(2).mode      = 2;
%         % TransitionMission(2).position  = [0; 0; -200];
%         % TransitionMission(2).params    = [0; 0; 0; 0];
%         % 
%         % TransitionMission(3).mode      = 2;
%         % TransitionMission(3).position  = [20; 0; -200];
%         % TransitionMission(3).params    = [0; 0; 0; 0];
%         % 
%         % TransitionMission(4).mode      = 6;
%         % TransitionMission(4).position  = [1000; 0; -200];
%         % TransitionMission(4).params    = [1; 1; 1; 1];
%         % 
%         % TransitionMission(5).mode      = 2;
%         % TransitionMission(5).position  = [2000; 0; -500];
%         % TransitionMission(5).params    = [0; 0; 0; 70*const.kts2mps];
%         % 
%         % % Based on the inter-distance and command speed, the simulation
%         % % time between these segments can be determined. 
%         % 
%         % TransitionMission(6).mode      = 2;
%         % TransitionMission(6).position  = [4700; 0; -500];
%         % TransitionMission(6).params    = [0; 0; 0; 57*const.kts2mps];
%         % 
%         % TransitionMission(7).mode      = 6;
%         % TransitionMission(7).position  = [6700; 0; -500];
%         % TransitionMission(7).params    = [1; 1; 1; 1];
%         % 
%         % TransitionMission(8).mode      = 2;
%         % TransitionMission(8).position  = [6700; 0; -100];
%         % TransitionMission(8).params    = [0; 0; 0; 0];
%         % 
%         % TransitionMission(9).mode      = 2;
%         % TransitionMission(9).position  = [6700; 0; -30];
%         % TransitionMission(9).params    = [0; 0; 0; 0];
%         % 
%         % TransitionMission(10).mode     = 4;
%         % TransitionMission(10).position = [6700; 0; 0];
%         % TransitionMission(10).params   = [0; 0; 0; 0];
% 
% end