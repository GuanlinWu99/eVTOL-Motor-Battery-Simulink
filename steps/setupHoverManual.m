%Set Slider for Manual Control to Hover.
load_system('VTOLTiltrotor');
set_param('VTOLTiltrotor/Ground Control Station/Get Flight Mission/noQGC/Flight Mode','Value','0');
set_param('VTOLAutopilotController/Mission', 'PortDimensions', '1');
%Set Aircraft to Manual Control Mode
TestMode=0;
%Set Visualization on.
Visualization=1;
%No guidance mission
guidanceType=0;
disp("Enabled hover manual testbench mode.")
