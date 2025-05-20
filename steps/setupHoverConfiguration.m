% Setup Hover configuration.
FSState=flightState.Hover;
%Get initial velocity and tilt based on flight mode.
[vIni,tiltIni]=exampleHelperGetInitialConfiguration(FSState);
%Set Time for Simulation
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,400);
disp("Enabled hover configuration.")