% mode = 1 : Takeoff,    Supported Flight Mode : Hover
% mode = 2 : Waypoint,   Supported Flight Mode : Hover, Fixed Fing
% mode = 4 : Land,       Supported Flight Mode : Hover
% mode = 6 : Transition, Supported Flight Mode : Hover, Fixed Fing

% 1 nautical mile (nm) : 1.15 miles

% Mode = 1;
% 
% Mode = 0 : Only hover mode
% Mode = 1 : Transition mode
% Mode = 2 : Prototype P4
% 
switch Profile
    
    %.. Test profile
    case 0

        TransitionMission               =   struct;
        TransitionMission(1).mode       =   1;                              % [-] takeoff
        TransitionMission(1).position   =   [0; 0; -50*const.ft2m];
        TransitionMission(1).params     =   [0; 0; 0; 0];

        TransitionMission(2).mode       =   2;                              % [-] VTOL waypoint (reaching hovering altitude)
        TransitionMission(2).position   =   [0; 0; -200];
        TransitionMission(2).params     =   [0; 0; 0; 0];

        TransitionMission(3).mode       =   2;                              % [-] VTOL waypoint (moving forward)
        TransitionMission(3).position   =   [20; 0; -200];
        TransitionMission(3).params     =   [0; 0; 0; 0];

        TransitionMission(4).mode       =   6;                              % [-] forward transition
        TransitionMission(4).position   =   [1000; 0; -200];
        TransitionMission(4).params     =   [1; 1; 1; 1];

        TransitionMission(5).mode       =   2;                              % [-] FW waypoint (climb)
        TransitionMission(5).position   =   [2000; 0; -300];
        TransitionMission(5).params     =   [0; 0; 2; 75*const.kts2mps];

        TransitionMission(6).mode       =   2;                              % [-] FW waypoint (descent)
        TransitionMission(6).position   =   [2700; 0; -200];
        TransitionMission(6).params     =   [0; 0; 1; 75*const.kts2mps];

        TransitionMission(7).mode       =   6;                              % [-] backward transition
        TransitionMission(7).position   =   [3700; 0; -200];
        TransitionMission(7).params     =   [1; 1; 1; 1];

        TransitionMission(8).mode       =   2;                              % [-] VTOL waypoint (reaching vertical approach point after back transition)
        TransitionMission(8).position   =   [4200; 0; -200];
        TransitionMission(8).params     =   [0; 0; 0; 0];

        TransitionMission(9).mode       =   2;                              % [-] VTOL waypoint (vertical approach)
        TransitionMission(9).position   =   [4200; 0; -30];
        TransitionMission(9).params     =   [0; 0; 0; 0];

        TransitionMission(10).mode      =   4;                              % [-] landing
        TransitionMission(10).position  =   [4200; 0; 0];
        TransitionMission(10).params    =   [0; 0; 0; 0];


    %.. Profile 4
    case 4

        TransitionMission               =   struct;
        TransitionMission(1).mode       =   1;                              % [-] takeoff
        TransitionMission(1).position   =   [0; 0; -30*const.ft2m];
        TransitionMission(1).params     =   [0; 0; 0; 0];

        TransitionMission(2).mode       =   2;                              % [-] takeoff (VTOL waypoint for reaching hovering altitude)
        TransitionMission(2).position   =   [0; 0; -300*const.ft2m];
        TransitionMission(2).params     =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(3).mode       =   6;                              % [-] initial climb (forward transition before climb)
        TransitionMission(3).position   =   [0.62*const.nm2m-200; 0; -300*const.ft2m]; 
        TransitionMission(3).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(4).mode       =   2;                              % [-] initial climb (altitude hold vertical autopilot)
        TransitionMission(4).position   =   [0.62*const.nm2m; 0; -500*const.ft2m]; 
        TransitionMission(4).params     =   [0; 0; 1; 75*const.kts2mps];

        TransitionMission(5).mode       =   2;                              % [-] second climb (flight path hold vertical autopilot)
        TransitionMission(5).position   =   [(2.35+0.62)*const.nm2m; 0; -2000*const.ft2m];
        TransitionMission(5).params     =   [0; 0; 2; 82*const.kts2mps];

        TransitionMission(6).mode       =   2;                              % [-] cruise
        TransitionMission(6).position   =   [(6.19+2.35+0.62)*const.nm2m; 0; -2000*const.ft2m];
        TransitionMission(6).params     =   [0; 0; 1; 95*const.kts2mps];

        TransitionMission(7).mode       =   2;                              % [-] descent (in FW mode)
        TransitionMission(7).position   =   [(5.34+6.19+2.35+0.62)*const.nm2m; 0; -300*const.ft2m];
        TransitionMission(7).params     =   [0; 0; 2; 75*const.kts2mps];

        TransitionMission(8).mode       =   2;                              % [-] approach pattern (FW, slow cruise)
        TransitionMission(8).position   =   [(1.5+5.34+6.19+2.35+0.62)*const.nm2m-1500; 0; -300*const.ft2m];
        TransitionMission(8).params     =   [0; 0; 1; 75*const.kts2mps];
    
        TransitionMission(9).mode       =   6;                              % [-] approach pattern (back transition)
        TransitionMission(9).position   =   [(1.5+5.34+6.19+2.35+0.62)*const.nm2m-500; 1; -300*const.ft2m];
        TransitionMission(9).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(10).mode      =   2;                              % [-] approach pattern (VTOL, reaching hovering point)
        TransitionMission(10).position  =   [(1.5+5.34+6.19+2.35+0.62)*const.nm2m; 1; -300*const.ft2m];
        TransitionMission(10).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(11).mode      =   2;                              % [-] landing (vertical approach using waypoint)
        TransitionMission(11).position  =   [(1.5+5.34+6.19+2.35+0.62)*const.nm2m; 0; -30*const.ft2m];
        TransitionMission(11).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(12).mode      =   4;                              % [-] landing
        TransitionMission(12).position  =   [(1.5+5.34+6.19+2.35+0.62)*const.nm2m; 0; 0*const.ft2m];
        TransitionMission(12).params    =   [0; 0; 0; 0];


    %.. Profile 1
    case 1

        TransitionMission               =   struct;
        TransitionMission(1).mode       =   1;                              % [-] takeoff
        TransitionMission(1).position   =   [0; 0; -30*const.ft2m];
        TransitionMission(1).params     =   [0; 0; 0; 0];

        TransitionMission(2).mode       =   2;                              % [-] takeoff (VTOL waypoint for reaching hovering altitude)
        TransitionMission(2).position   =   [0; 0; -300*const.ft2m];
        TransitionMission(2).params     =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(3).mode       =   6;                              % [-] initial climb (forward transition before climb)
        TransitionMission(3).position   =   [0.62*const.nm2m-200; 0; -300*const.ft2m]; 
        TransitionMission(3).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(4).mode       =   2;                              % [-] initial climb (altitude hold vertical autopilot)
        TransitionMission(4).position   =   [0.62*const.nm2m; 0; -500*const.ft2m]; 
        TransitionMission(4).params     =   [0; 0; 1; 75*const.kts2mps];

        TransitionMission(5).mode       =   2;                              % [-] second climb (flight path hold vertical autopilot)
        TransitionMission(5).position   =   [(1.17+0.62)*const.nm2m; 0; -1500*const.ft2m];
        TransitionMission(5).params     =   [0; 0; 2; 82*const.kts2mps];

        TransitionMission(6).mode       =   2;                              % [-] cruise
        TransitionMission(6).position   =   [(2.08+1.17+0.62)*const.nm2m; 0; -1500*const.ft2m];
        TransitionMission(6).params     =   [0; 0; 1; 90*const.kts2mps];

        TransitionMission(7).mode       =   2;                              % [-] descent (in FW mode)
        TransitionMission(7).position   =   [(2.82+2.08+1.17+0.62)*const.nm2m; 0; -300*const.ft2m];
        TransitionMission(7).params     =   [0; 0; 2; 75*const.kts2mps];

        TransitionMission(8).mode       =   2;                              % [-] approach pattern (FW, slow cruise)
        TransitionMission(8).position   =   [(1.5+2.82+2.08+1.17+0.62)*const.nm2m-1500; 0; -300*const.ft2m];
        TransitionMission(8).params     =   [0; 0; 1; 75*const.kts2mps];
    
        TransitionMission(9).mode       =   6;                              % [-] approach pattern (back transition)
        TransitionMission(9).position   =   [(1.5+2.82+2.08+1.17+0.62)*const.nm2m-250; 0; -300*const.ft2m];
        TransitionMission(9).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(10).mode      =   2;                              % [-] approach pattern (VTOL, reaching hovering point)
        TransitionMission(10).position  =   [(1.5+2.82+2.08+1.17+0.62)*const.nm2m; 0; -300*const.ft2m];
        TransitionMission(10).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(11).mode      =   2;                              % [-] landing (vertical approach using waypoint)
        TransitionMission(11).position  =   [(1.5+2.82+2.08+1.17+0.62)*const.nm2m; 0; -30*const.ft2m];
        TransitionMission(11).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(12).mode      =   4;                              % [-] landing
        TransitionMission(12).position  =   [(1.5+2.82+2.08+1.17+0.62)*const.nm2m; 0; 0*const.ft2m];
        TransitionMission(12).params    =   [0; 0; 0; 0];


    %.. Profile 6
    case 6

        TransitionMission               =   struct;
        TransitionMission(1).mode       =   1;                              % [-] takeoff
        TransitionMission(1).position   =   [0; 0; -30*const.ft2m];
        TransitionMission(1).params     =   [0; 0; 0; 0];

        TransitionMission(2).mode       =   2;                              % [-] takeoff (VTOL waypoint for reaching hovering altitude)
        TransitionMission(2).position   =   [0; 0; -300*const.ft2m];
        TransitionMission(2).params     =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(3).mode       =   6;                              % [-] initial climb (forward transition before climb)
        TransitionMission(3).position   =   [0.62*const.nm2m-200; 0; -300*const.ft2m]; 
        TransitionMission(3).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(4).mode       =   2;                              % [-] initial climb (altitude hold vertical autopilot)
        TransitionMission(4).position   =   [0.62*const.nm2m; 0; -500*const.ft2m]; 
        TransitionMission(4).params     =   [0; 0; 1; 75*const.kts2mps];

        TransitionMission(5).mode       =   2;                              % [-] second climb (flight path hold vertical autopilot)
        TransitionMission(5).position   =   [(3.53+0.62)*const.nm2m; 0; -2000*const.ft2m];
        TransitionMission(5).params     =   [0; 0; 2; 82*const.kts2mps];

        TransitionMission(6).mode       =   2;                              % [-] cruise
        TransitionMission(6).position   =   [(12.01+3.53+0.62)*const.nm2m; 0; -2000*const.ft2m];
        TransitionMission(6).params     =   [0; 0; 1; 90*const.kts2mps];

        TransitionMission(7).mode       =   2;                              % [-] descent (in FW mode)
        TransitionMission(7).position   =   [(5.34+12.01+3.53+0.62)*const.nm2m; 0; -300*const.ft2m];
        TransitionMission(7).params     =   [0; 0; 2; 75*const.kts2mps];

        TransitionMission(8).mode       =   2;                              % [-] approach pattern (FW, slow cruise)
        TransitionMission(8).position   =   [(1.5+5.34+12.01+3.53+0.62)*const.nm2m-1500; 0; -300*const.ft2m];
        TransitionMission(8).params     =   [0; 0; 1; 75*const.kts2mps];
    
        TransitionMission(9).mode       =   6;                              % [-] approach pattern (back transition)
        TransitionMission(9).position   =   [(1.5+5.34+12.01+3.53+0.62)*const.nm2m-250; 0; -300*const.ft2m];    % note. make y = 1 cause error?
        TransitionMission(9).params     =   [1; 1; 1; 60*const.kts2mps];

        TransitionMission(10).mode      =   2;                              % [-] approach pattern (VTOL, reaching hovering point)
        TransitionMission(10).position  =   [(1.5+5.34+12.01+3.53+0.62)*const.nm2m; 0; -300*const.ft2m];    % note. make y = 1 cause error?
        TransitionMission(10).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(11).mode      =   2;                              % [-] landing (vertical approach using waypoint)
        TransitionMission(11).position  =   [(1.5+5.34+12.01+3.53+0.62)*const.nm2m; 0; -30*const.ft2m];
        TransitionMission(11).params    =   [0; 0; 0; 10*const.kts2mps];

        TransitionMission(12).mode      =   4;                              % [-] landing
        TransitionMission(12).position  =   [(1.5+5.34+12.01+3.53+0.62)*const.nm2m; 0; 0*const.ft2m];
        TransitionMission(12).params    =   [0; 0; 0; 0];
end

load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', 'length(TransitionMission)')