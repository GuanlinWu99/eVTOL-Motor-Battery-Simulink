%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [controlParams, simParams] = load_controller_parameters(uavParams, const)

simParams.simTime                   =   500;
simParams.UAVTimeStep               =   0.001;

%% define controller parameters


% sampling time for controller

controlParams.ControlSampleTime     =   0.005;
controlParams.UAVSampleTime=   0.005;

controlParams.maxPWM                =   1.0;
controlParams.minPWM                =   0.1;
controlParams.VTOLthrCoefficient    =   (uavParams.rotor.Ct*const.rho0*pi*(uavParams.geom.PropDiameter/2)^4);   % [N/(rad/s)^2] Thrust coefficients for VTOL mode
controlParams.FWTrimThrottle        =   2800*const.rpm2rps;                 % [rps] trim throttle for fixed-wing mode (~ 62kts)           
controlParams.FWElevatorTrim        =   -11*const.deg2rad;                  % [rad] elevator trim for fixed-wing mode (~ 62kts)
controlParams.TiltScheduleRate      =   3/180*pi;                          % [rad/s] tilt rate for forward transition
controlParams.CriticalTiltAngle     =   60/180*pi;                          % [rad] critical tilt angle for forward transition
controlParams.VtransitionFWD2       =   60*const.kts2mps;                   % [m/s] critical 
controlParams.VtransitionFWD1       =   40*const.kts2mps;
controlParams.PitchtransitionFWD    =   11*const.deg2rad;                     %                 
controlParams.WindDownRate          =   10;                        %                             
controlParams.VTOLmaxPitch          =   20/180*pi;                   %
controlParams.VTOLminPitch          =   -20/180*pi;                   %
controlParams.VTOLmaxPitchSlewrate  =   200/180*pi;                   %
controlParams.VTOLminPitchSlewrate  =   -200/180*pi;                   %
controlParams.VTOLmaxRoll           =   20/180*pi;                   %
controlParams.VTOLminRoll           =   -20/180*pi;                   %
controlParams.VTOLmaxRollSlewrate   =   200/180*pi;                   %
controlParams.VTOLminRollSlewrate   =   -200/180*pi;                   %
controlParams.maxAileron            =   30*const.deg2rad;
controlParams.minAileron            =   -30*const.deg2rad;
controlParams.maxElevator           =   30*const.deg2rad;
controlParams.minElevator           =   -30*const.deg2rad;
controlParams.maxRudder             =   30*const.deg2rad;
controlParams.minRudder             =   -30*const.deg2rad;

controlParams.R_WAYPOINTTRANSITION_VTOL =   1.0;
controlParams.R_LOOKAHEAD_VTOL          =   5.0;

controlParams.Vcruise = 16;
controlParams.L1                    =   300;                                % [m]
controlParams.turningRadius =   60;

controlParams.minThrottle =   0;
controlParams.maxThrottle =   1;
controlParams.slewRateThrottle= 1;
controlParams.slewRateServos= 5;
controlParams.maxPitch= 0.1745;
controlParams.minPitch= -0.1745;
controlParams.maxRoll= 0.6981;
controlParams.minRoll= -0.6981;
controlParams.maxRollRate= 1.2217;
controlParams.minRollRate= -1.2217;
controlParams.maxPitchRate= 1.2217;
controlParams.minPitchRate= -1.2217;
controlParams.maxClimbRate= 3;
controlParams.maxDescendRate= -3;
controlParams.stallSpeed= 9.5000;
controlParams.cruiseSpeed= 20;
controlParams.takeoffSpeed= 12.3500;
controlParams.takeoffNavAlt= 5;
controlParams.climboutAltMin= 10;
controlParams.climboutAlt= 13;
controlParams.landingApproachSpeed= 12.3500;
controlParams.flarePitch= 0.0349;
controlParams.flareAlt= 2;
controlParams.landStopVel= 1;
controlParams.finalLoiterSpeed= 15; 

controlParams.tilt_trim             =   0.0;                                % [rad] tilt angle trim for VTOL mode
controlParams.w_trim = 0;
controlParams.windVel = [0, 0, 0];

controlParams.transitionRadius = 10;

controlParams.MaxTilt = 1.0472;




%% define simulation parameters





end
