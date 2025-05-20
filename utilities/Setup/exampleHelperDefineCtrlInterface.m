function exampleHelperDefineCtrlInterface() 
%exampleHelperDefineCtrlInterface define control system interface for
%VTOL.

% Copyright 2023 The MathWorks, Inc.

% Bus object: AAC 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'airspeed';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'altitude';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'course';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'L1';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'climbrate';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

AAC = Simulink.Bus;
AAC.HeaderFile = '';
AAC.Description = '';
AAC.DataScope = 'Auto';
AAC.Alignment = -1;
AAC.PreserveElementDimensions = 0;
AAC.Elements = elems;
clear elems;
% Bus object: controlMode 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'lateralGuidance';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint8';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'airspeedAltitude';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint8';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'attitude';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint8';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'manual';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint8';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'armed';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'uint8';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'inTransition';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'uint8';
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'TransitionCondition';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint8';
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

controlMode = Simulink.Bus;
controlMode.HeaderFile = '';
controlMode.Description = '';
controlMode.DataScope = 'Auto';
controlMode.Alignment = -1;
controlMode.PreserveElementDimensions = 0;
controlMode.Elements = elems;
clear elems;

%% FixedWingCommands
clear felems;
felems(1) = Simulink.BusElement;
felems(1).Name = 'roll';
felems(2) = Simulink.BusElement;
felems(2).Name = 'pitch';
felems(3) = Simulink.BusElement;
felems(3).Name = 'yaw';
felems(4) = Simulink.BusElement;
felems(4).Name = 'airspeed';
FixedWingCommandBus = Simulink.Bus;
FixedWingCommandBus.Elements = felems;

%% Assign into base workspace
assignin('base',"FixedWingCommandBus",FixedWingCommandBus);
assignin('base','controlMode', controlMode);
assignin('base','AAC', AAC);
end
