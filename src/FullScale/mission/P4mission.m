%% This is a code for P4 mission profile
TransitionMission             = struct;
profile = 4;
switch profile
    case 4
    TransitionMission(1).mode     = 1;
    TransitionMission(1).position = [0; 0; -30*const.ft2m];
    TransitionMission(1).params   = [0; 0; 0; 0];
    
    % Vertical takeoff
    TransitionMission(2).mode     = 2;
    TransitionMission(2).position = [0; 0; -450*const.ft2m];
    TransitionMission(2).params   = [0; 0; 0; 0];
    
    % Forward transition
    TransitionMission(3).mode     = 6;
    TransitionMission(3).position = [0.62*const.nm2m-100; 0; -450*const.ft2m]; % 1000 정도 되야 lateral error 나쁘지 않음
    TransitionMission(3).params   = [1; 1; 1; 1];
    
    % Initial climb
    TransitionMission(4).mode     = 2;
    TransitionMission(4).position = [0.62*const.nm2m; 0; -500*const.ft2m];
    TransitionMission(4).params   = [0; 0; 0; 75*const.kts2mps];
    
    % Second climb
    TransitionMission(5).mode     = 2;
    TransitionMission(5).position = [(3.76+0.62)*const.nm2m; 0; -2000*const.ft2m];
    TransitionMission(5).params   = [0; 0; 0; 75*const.kts2mps];
    
    % Cruise
    TransitionMission(6).mode     = 2;
    TransitionMission(6).position = [(3.76+0.62+1.2)*const.nm2m; 0; -2000*const.ft2m];
    TransitionMission(6).params   = [0; 0; 0; 75*const.kts2mps];
    
    % Descent Starts
    TransitionMission(7).mode     = 2;
    TransitionMission(7).position = [(3.76+0.62+1.2+2.61)*const.nm2m; 0; -500*const.ft2m];
    TransitionMission(7).params   = [0; 0; 0; 75*const.kts2mps];
    
    TransitionMission(8).mode     = 2;
    TransitionMission(8).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m-1950; 0; -500*const.ft2m];
    TransitionMission(8).params   = [0; 0; 0; 75*const.kts2mps];
    
    % Backword transition starts
    TransitionMission(9).mode     = 6;
    TransitionMission(9).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m-100; 1; -500*const.ft2m];
    TransitionMission(9).params   = [1; 1; 1; 1];
    
    % Hover mode
    TransitionMission(10).mode     = 2;
    TransitionMission(10).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -330*const.ft2m];
    TransitionMission(10).params   = [0; 0; 0; 0];
    
    TransitionMission(11).mode     = 2;
    TransitionMission(11).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -200*const.ft2m];
    TransitionMission(11).params   = [0; 0; 0; 0];
    
    TransitionMission(12).mode     = 4;
    TransitionMission(12).position = [(3.76+0.62+1.2+2.61+1.5)*const.nm2m; 0; -30*const.ft2m];
    TransitionMission(12).params   = [0; 0; 0; 0];
end
load_system('VTOLAutopilotController');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', 'length(TransitionMission)')
 