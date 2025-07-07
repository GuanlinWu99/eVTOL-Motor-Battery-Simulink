%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function controlParams = load_controller_parameters(uavParam, const)

%% define controller parameters
% sampling time for controller
controlParams.ControlSampleTime=   0.005;
controlParams.UAVSampleTime=   0.005;

controlParams.maxPWM                =   1.0;
controlParams.minPWM                =   0.1;
controlParams.VTOLthrCoefficient    =   (uavParam.rotor.Ct*const.rho0*pi*(uavParam.geom.PropDiameter/2)^4);   % [N/(rad/s)^2] Thrust coefficients for VTOL mode
controlParams.FWTrimThrottle        =   2800*const.rpm2rps;                 % [rps] trim throttle for fixed-wing mode (~ 62kts)           
controlParams.FWElevatorTrim        =   -11*const.deg2rad;                  % [rad] elevator trim for fixed-wing mode (~ 62kts)
controlParams.TiltScheduleRate      =   3/180*pi;                          % [rad/s] tilt rate for forward transition
controlParams.CriticalTiltAngle     =   60/180*pi;                          % [rad] critical tilt angle for forward transition
controlParams.Vtransition           =   50*const.kts2mps;                   % [m/s] critical 
controlParams.WindDownRate          =   10;                                  

controlParams.Vcruise               =   16;

controlParams.turningRadius         =   60;
controlParams.L1                    =   60;
controlParams.minThrottle           =   0;
controlParams.maxThrottle           =   1;
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
simParams.simTime                   =   500;
simParams.UAVTimeStep               =   0.001;



end
