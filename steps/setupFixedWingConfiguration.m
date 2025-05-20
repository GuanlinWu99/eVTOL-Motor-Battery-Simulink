% Setup Fixed Wing configuration.
FSState=flightState.FixedWing;
%Get initial velocity and tilt based on flight mode.
[vIni,tiltIni]=exampleHelperGetInitialConfiguration(FSState);
%Set Time for Simulation -100 seconds
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
simTimeParam = getEntry(dDataSectObj,'simTime');
setValue(simTimeParam,100);

disp("Enabled fixed-wing configuration.")