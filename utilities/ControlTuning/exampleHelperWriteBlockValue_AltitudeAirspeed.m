function controlParams = exampleHelperWriteBlockValue_AltitudeAirspeed(hTuner,controlParams)
%exampleHelperWriteBlockValue_AltitudeAirspeed Write tuned control gains to control param structure

% Copyright 2023 The MathWorks, Inc.

altitudeController = getBlockValue(hTuner,'Altitude Controller/PID Controller');
controlParams.P_ALT = altitudeController.Kp;

pitchSetPointController = getBlockValue(hTuner,'Pitch SP Controller/PID Controller');
controlParams.P_CLIMBRATE = pitchSetPointController.Kp;

airspeedController = getBlockValue(hTuner,'Airspeed Controller');
controlParams.P_AIRSPD = airspeedController.Kp;
controlParams.I_AIRSPD = airspeedController.Ki;
controlParams.D_AIRSPD = airspeedController.Kd;
controlParams.N_AIRSPD = 1/airspeedController.Tf;

end

