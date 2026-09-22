%=========================================================================%
% Define buses for controller module                                      %
% author(s): minhyun                                                      %
% description:                                                            %
%=========================================================================%

function load_ctrl_interface() 

%.. bus object: LAPY
clear elems;
elems(1)                =   Simulink.BusElement;
elems(1).Name           =   'localposition';
elems(1).Dimensions     =   3;
elems(1).DimensionsMode =   'Fixed';
elems(1).DataType       =   'double';
elems(1).Complexity     =   'real';
elems(1).Min            =   [];
elems(1).Max            =   [];
elems(1).DocUnits       =   '';
elems(1).Description    =   '';

elems(2)                =   Simulink.BusElement;
elems(2).Name           =   'yaw';
elems(2).Dimensions     =   1;
elems(2).DimensionsMode =   'Fixed';
elems(2).DataType       =   'double';
elems(2).Complexity     =   'real';
elems(2).Min            =   [];
elems(2).Max            =   [];
elems(2).DocUnits       =   '';
elems(2).Description    =   '';

LAPY                            =   Simulink.Bus;
LAPY.HeaderFile                 =   '';
LAPY.Description                =   '';
LAPY.DataScope                  =   'Auto';
LAPY.Alignment                  =   -1;
LAPY.PreserveElementDimensions  =   0;
LAPY.Elements                   =   elems;

%.. bus object: VTOLCommands
clear elems;
elems(1)                =   Simulink.BusElement;
elems(1).Name           =   'roll';
elems(1).Dimensions     =   1;
elems(1).DimensionsMode =   'Fixed';
elems(1).DataType       =   'double';
elems(1).Complexity     =   'real';
elems(1).Min            =   [];
elems(1).Max            =   [];
elems(1).DocUnits       =   '';
elems(1).Description    =   '';

elems(2)                =   Simulink.BusElement;
elems(2).Name           =   'pitch';
elems(2).Dimensions     =   1;
elems(2).DimensionsMode =   'Fixed';
elems(2).DataType       =   'double';
elems(2).Complexity     =   'real';
elems(2).Min            =   [];
elems(2).Max            =   [];
elems(2).DocUnits       =   '';
elems(2).Description    =   '';

elems(3)                =   Simulink.BusElement;
elems(3).Name           =   'yaw';
elems(3).Dimensions     =   1;
elems(3).DimensionsMode =   'Fixed';
elems(3).DataType       =   'double';
elems(3).Complexity     =   'real';
elems(3).Min            =   [];
elems(3).Max            =   [];
elems(3).DocUnits       =   '';
elems(3).Description    =   '';

VTOLCommandBus                              =   Simulink.Bus;
VTOLCommandBus.HeaderFile                   =   '';
VTOLCommandBus.Description                  =   '';
VTOLCommandBus.DataScope                    =   'Auto';
VTOLCommandBus.Alignment                    =   -1;
VTOLCommandBus.PreserveElementDimensions    =   0;
VTOLCommandBus.Elements                     =   elems;

%.. bus object: AAC 
clear elems;
elems(1)                =   Simulink.BusElement;
elems(1).Name           =   'airspeed';
elems(1).Dimensions     =   1;
elems(1).DimensionsMode =   'Fixed';
elems(1).DataType       =   'double';
elems(1).Complexity     =   'real';
elems(1).Min            =   [];
elems(1).Max            =   [];
elems(1).DocUnits       =   '';
elems(1).Description    =   '';

elems(2)                =   Simulink.BusElement;
elems(2).Name           =   'altitude';
elems(2).Dimensions     =   1;
elems(2).DimensionsMode =   'Fixed';
elems(2).DataType       =   'double';
elems(2).Complexity     =   'real';
elems(2).Min            =   [];
elems(2).Max            =   [];
elems(2).DocUnits       =   '';
elems(2).Description    =   '';

elems(3)                =   Simulink.BusElement;
elems(3).Name           =   'course';
elems(3).Dimensions     =   1;
elems(3).DimensionsMode =   'Fixed';
elems(3).DataType       =   'double';
elems(3).Complexity     =   'real';
elems(3).Min            =   [];
elems(3).Max            =   [];
elems(3).DocUnits       =   '';
elems(3).Description    =   '';

elems(4)                =   Simulink.BusElement;
elems(4).Name           =   'L1';
elems(4).Dimensions     =   1;
elems(4).DimensionsMode =   'Fixed';
elems(4).DataType       =   'double';
elems(4).Complexity     =   'real';
elems(4).Min            =   [];
elems(4).Max            =   [];
elems(4).DocUnits       =   '';
elems(4).Description    =   '';

elems(5)                =   Simulink.BusElement;
elems(5).Name           =   'climbrate';
elems(5).Dimensions     =   1;
elems(5).DimensionsMode =   'Fixed';
elems(5).DataType       =   'double';
elems(5).Complexity     =   'real';
elems(5).Min            =   [];
elems(5).Max            =   [];
elems(5).DocUnits       =   '';
elems(5).Description    =   '';

AAC                             =   Simulink.Bus;
AAC.HeaderFile                  =   '';
AAC.Description                 =   '';
AAC.DataScope                   =   'Auto';
AAC.Alignment                   =   -1;
AAC.PreserveElementDimensions   =   0;
AAC.Elements                    =   elems;

%.. bus object: FixedWingCommands
clear elems;
elems(1)                =   Simulink.BusElement;
elems(1).Name           =   'roll';
elems(1).Dimensions     =   1;
elems(1).DimensionsMode =   'Fixed';
elems(1).DataType       =   'double';
elems(1).Complexity     =   'real';
elems(1).Min            =   [];
elems(1).Max            =   [];
elems(1).DocUnits       =   '';
elems(1).Description    =   '';

elems(2)                =   Simulink.BusElement;
elems(2).Name           =   'pitch';
elems(2).Dimensions     =   1;
elems(2).DimensionsMode =   'Fixed';
elems(2).DataType       =   'double';
elems(2).Complexity     =   'real';
elems(2).Min            =   [];
elems(2).Max            =   [];
elems(2).DocUnits       =   '';
elems(2).Description    =   '';

elems(3)                =   Simulink.BusElement;
elems(3).Name           =   'yaw';
elems(3).Dimensions     =   1;
elems(3).DimensionsMode =   'Fixed';
elems(3).DataType       =   'double';
elems(3).Complexity     =   'real';
elems(3).Min            =   [];
elems(3).Max            =   [];
elems(3).DocUnits       =   '';
elems(3).Description    =   '';

elems(4)                =   Simulink.BusElement;
elems(4).Name           =   'airspeed';
elems(4).Dimensions     =   1;
elems(4).DimensionsMode =   'Fixed';
elems(4).DataType       =   'double';
elems(4).Complexity     =   'real';
elems(4).Min            =   [];
elems(4).Max            =   [];
elems(4).DocUnits       =   '';
elems(4).Description    =   '';

elems(5)                =   Simulink.BusElement;
elems(5).Name           =   'flap';
elems(5).Dimensions     =   1;
elems(5).DimensionsMode =   'Fixed';
elems(5).DataType       =   'double';
elems(5).Complexity     =   'real';
elems(5).Min            =   [];
elems(5).Max            =   [];
elems(5).DocUnits       =   '';
elems(5).Description    =   '';

FixedWingCommandBus                             =   Simulink.Bus;
FixedWingCommandBus.HeaderFile                  =   '';
FixedWingCommandBus.Description                 =   '';
FixedWingCommandBus.DataScope                   =   'Auto';
FixedWingCommandBus.Alignment                   =   -1;
FixedWingCommandBus.PreserveElementDimensions   =   0;
FixedWingCommandBus.Elements                    =   elems;

%.. bus object: controlMode 
clear elems;
elems(1)                =   Simulink.BusElement;
elems(1).Name           =   'lateralguidance';
elems(1).Dimensions     =   1;
elems(1).DimensionsMode =   'Fixed';
elems(1).DataType       =   'uint8';
elems(1).Complexity     =   'real';
elems(1).Min            =   [];
elems(1).Max            =   [];
elems(1).DocUnits       =   '';
elems(1).Description    =   '';

elems(2)                =   Simulink.BusElement;
elems(2).Name           =   'airspeedaltitude';
elems(2).Dimensions     =   1;
elems(2).DimensionsMode =   'Fixed';
elems(2).DataType       =   'uint8';
elems(2).Complexity     =   'real';
elems(2).Min            =   [];
elems(2).Max            =   [];
elems(2).DocUnits       =   '';
elems(2).Description    =   '';

elems(3)                =   Simulink.BusElement;
elems(3).Name           =   'attitude';
elems(3).Dimensions     =   1;
elems(3).DimensionsMode =   'Fixed';
elems(3).DataType       =   'uint8';
elems(3).Complexity     =   'real';
elems(3).Min            =   [];
elems(3).Max            =   [];
elems(3).DocUnits       =   '';
elems(3).Description    =   '';

elems(4)                =   Simulink.BusElement;
elems(4).Name           =   'manual';
elems(4).Dimensions     =   1;
elems(4).DimensionsMode =   'Fixed';
elems(4).DataType       =   'uint8';
elems(4).Complexity     =   'real';
elems(4).Min            =   [];
elems(4).Max            =   [];
elems(4).DocUnits       =   '';
elems(4).Description    =   '';

elems(5)                =   Simulink.BusElement;
elems(5).Name           =   'armed';
elems(5).Dimensions     =   1;
elems(5).DimensionsMode =   'Fixed';
elems(5).DataType       =   'uint8';
elems(5).Complexity     =   'real';
elems(5).Min            =   [];
elems(5).Max            =   [];
elems(5).DocUnits       =   '';
elems(5).Description    =   '';

elems(6)                =   Simulink.BusElement;
elems(6).Name           =   'intransition';
elems(6).Dimensions     =   1;
elems(6).DimensionsMode =   'Fixed';
elems(6).DataType       =   'uint8';
elems(6).Complexity     =   'real';
elems(6).Min            =   [];
elems(6).Max            =   [];
elems(6).DocUnits       =   '';
elems(6).Description    =   '';

elems(7)                =   Simulink.BusElement;
elems(7).Name           =   'transitioncondition';
elems(7).Dimensions     =   1;
elems(7).DimensionsMode =   'Fixed';
elems(7).DataType       =   'uint8';
elems(7).Complexity     =   'real';
elems(7).Min            =   [];
elems(7).Max            =   [];
elems(7).DocUnits       =   '';
elems(7).Description    =   '';

ControlMode                             =   Simulink.Bus;
ControlMode.HeaderFile                  =   '';
ControlMode.Description                 =   '';
ControlMode.DataScope                   =   'Auto';
ControlMode.Alignment                   =   -1;
ControlMode.PreserveElementDimensions   =   0;
ControlMode.Elements                    =   elems;

%.. clear elements
clear elems;

%.. assign into base workspace
assignin('base','LAPY', LAPY);
assignin('base',"VTOLCommandBus",VTOLCommandBus);
assignin('base','AAC', AAC);
assignin('base',"FixedWingCommandBus",FixedWingCommandBus);
assignin('base','ControlMode', ControlMode);

end
