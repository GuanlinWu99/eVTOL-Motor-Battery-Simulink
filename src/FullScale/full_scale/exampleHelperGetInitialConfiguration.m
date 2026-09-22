function [vIni,iniTilt]=exampleHelperGetInitialConfiguration(FSState)
% exampleHelperGetInitialConfiguration get initial configuration of UAV.

% Copyright 2023 The MathWorks, Inc.

if(isequal(FSState,flightState.Hover))||(isequal(FSState,flightState.Transition))
vIni=0;
iniTilt=0;
else
    vIni=16;
    iniTilt=pi/2;
end
end