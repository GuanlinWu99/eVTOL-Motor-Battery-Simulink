%cleanupVTOL clean up VTOL parameters.

% Copyright 2023 The MathWorks, Inc.

clear AAC FSState controlParams FWControlParams FixedWingCommandBus FixedWingCtrlBus ForwardVelocityCutoff HoverMission R_LOOKAHEAD R_WAYPOINTTRANSITION ReferenceFilterDen ReferenceFilterNum RotorCntrlBus SensorAAFiltDen SensorAAFiltNum SensorType TestMode TuningMode Visualization controlMode controlParams dDataSectObj guidanceType iniRoll iniYaw initPitch mdl minPWM myDictionaryObj simTimeParam tiltIni tilt_max uavParam vIni
myDictionaryObj = Simulink.data.dictionary.open('VTOLDynamicsData.sldd');
discardChanges(myDictionaryObj);