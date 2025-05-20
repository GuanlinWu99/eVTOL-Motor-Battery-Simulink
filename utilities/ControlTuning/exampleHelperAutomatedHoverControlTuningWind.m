%exampleHelperAutomatedHoverControlTuningWind tune all control gains for hover
%mode in steady wind.

% Copyright 2023-2024 The MathWorks, Inc.
%Disable linearization warning
state = warning('off', 'Simulink:blocks:BmathSqrtOfNegativeNumber');
cleanup = onCleanup(@()warning(state));
pidWarningSuppression = warning('off', 'Control:design:pidtune11');
cleanupPIDWarning = onCleanup(@()warning(pidWarningSuppression));

options = linearizeOptions('SampleTime',0.005);
ctrloptions= pidtuneOptions;
ctrloptions.PhaseMargin=60;
ctrloptions.DesignFocus = 'balanced';

%% Enable/Disable Tuning of Loops
TuneXY = true;
TuneVxVy = true;
TunePR = true;
TunePrRr = true;
TuneZ = true;
TuneVZ = true;
%% Roll rate
if TunePrRr
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Roll Rate Controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux2'],1,'openoutput');
    rollRateSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(rollRateSys,'PDF',50,ctrloptions);
    P_ROLL_RATE=C.Kp;
    I_ROLL_RATE=C.Ki; % NO I Gain because of 1/s plant
    D_ROLL_RATE=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_ROLL_RATE=1/C.Tf;
end

%% Pitch rate
if TunePrRr
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Pitch Rate Controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux2'],2,'openoutput');
    pitchRateSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(pitchRateSys,'PDF',50,ctrloptions);
    P_PITCH_RATE=C.Kp;
    I_PITCH_RATE=C.Ki; % NO I Gain because of 1/s plant
    D_PITCH_RATE=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_PITCH_RATE=1/C.Tf;
end

%% Yaw rate
if TunePrRr
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Yaw rate controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux2'],3,'openoutput');
    yawRateSys=linearize(mdl,io,options);
    [C,~]=pidtune(yawRateSys,'PDF',50,ctrloptions);
    P_YAW_RATE=C.Kp;
    I_YAW_RATE=C.Ki;  % NO I Gain because of 1/s plant
    D_YAW_RATE=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_YAW_RATE=1/C.Tf;
end

%% Design outer loop after designing inner loop.
%% Roll
if TunePR
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Roll Controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux3'],1,'openoutput');
    rollSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(rollSys,'P',10,ctrloptions);
    P_ROLL=C.Kp;
    I_ROLL=C.Ki; % NO I Gain because of 1/s plant
    D_ROLL=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_ROLL=1/C.Tf;
end

%% Pitch
if TunePR
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Pitch Controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux3'],2,'openoutput');
    pitchSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(pitchSys,'P',10,ctrloptions);
    P_PITCH=C.Kp;
    I_PITCH=C.Ki; % NO I Gain because of 1/s plant
    D_PITCH=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_PITCH=1/C.Tf;
end

%% Yaw
if TunePR
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Yaw Controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Derivative Gain/Demux3'],3,'openoutput');
    yawSys=linearize(mdl,io,options);
    [C,~]=pidtune(yawSys,'P',10,ctrloptions);
    P_YAW=C.Kp;
    I_YAW=C.Ki; % NO I Gain because of 1/s plant
    D_YAW=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
   N_YAW=1/C.Tf;
end

%% Forward Velocity Controller
% NEEDS TO BE "SNAPSHOT" LINEARIZED
% SYSTEM IS VERY HIGH ORDER WITH NUMEROUS MODES
if TuneVxVy
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Forward velocity controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Forward velocity'],1,'openoutput');
    XRateSys=linearize(mdl,io,options,7);     
    [C,~]=pidtune(XRateSys,'PDF',5,ctrloptions);
    P_VX=C.Kp;
    I_VX = C.Ki;
    D_VX = C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_VX=1/C.Tf;
end

%% Lateral Velocity Controller
if TuneVxVy
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Lateral velocity controller '],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Lateral velocity'],1,'openoutput');
    YRateSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(YRateSys,'PDF',5,ctrloptions);
    P_VY=C.Kp;
    I_VY = C.Ki;
    D_VY = C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_VY=1/C.Tf;
end

%% Forward Position Controller
% NEEDS TO BE "SNAPSHOT" LINEARIZED
% SYSTEM IS VERY HIGH ORDER WITH NUMEROUS MODES
if TuneXY
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Forward position controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Forward position'],1,'openoutput');
    XSys=linearize(mdl,io,options,7);   
    [C,~]=pidtune(XSys,'P',1,ctrloptions);
    P_X=C.Kp;
    I_X=C.Ki;
    D_X=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_X=1/C.Tf;
end

%% Lateral Position Controller
if TuneXY
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Lateral position controller'],1,'openinput');
    io(2) = linio([ 'VTOLAutopilotController/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller/Lateral position'],1,'openoutput');
    YSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(YSys,'P',1,ctrloptions);
    P_Y=C.Kp;
    I_Y=C.Ki;
    D_Y=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_Y=1/C.Tf;
end


%% Z Rate Controller
if TuneVZ
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Altitude Control/Altitude rate controller'],1,'openinput');
    io(2) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Altitude Control/Selector1'],1,'openoutput');
    ZRateSys=linearize(mdl,io,options,7);
    [C,~]=pidtune(ZRateSys,'PDF',5,ctrloptions);
    P_VZ=-C.Kp;
    I_VZ=-C.Ki;
    D_VZ=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_VZ=1/C.Tf;
end

%% Z Controller
if TuneZ
    % Disable VZ Controller for linearization   
    io(1) = linio(['VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Altitude Control/Altitude controller'],1,'openinput');
    io(2) = linio([ 'VTOLAutopilotController/Low level controller/Multicopter Controller/Attitude & Altitude controller/Altitude Control/Alt signal condition'],1,'openoutput');
    ZSys=linearize(mdl,io,options,7);
    UseVZControl = true;
    [C,~]=pidtune(ZSys,'P',1,ctrloptions);
    P_Z=C.Kp;
    I_Z=C.Ki;
    D_Z=C.Kd;
    if C.Tf<eps
        C.Tf = 0.01;
    end
    N_Z=1/C.Tf;
end

% Update attitude controller gains
exampleHelperUpdateAttitudeControllerGains;

% Update forward and lateral controllers
exampleHelperUpdateForwardControllerGains;
exampleHelperUpdateLateralControllerGains;

% Update altiude contoller gains
exampleHelperUpdateAltitudeControllerGains;

controlParams.P_BACK = 0.1;

%% Discard IO points
setlinio(mdl,[]);