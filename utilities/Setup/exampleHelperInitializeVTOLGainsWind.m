% exampleHelperInitializeVTOLGains initialize control gains for VTOL UAV.

% Copyright 2023 The MathWorks, Inc.

%Hover Default Control Based Gains
%% Roll rate
% controlParams.P_ROLL_RATE=25.89;
controlParams.P_ROLL_RATE = controlParams.P_ROLL_RATE*0.1;

controlParams.I_ROLL_RATE=0;
controlParams.N_ROLL_RATE=100;
controlParams.D_ROLL_RATE=2*sqrt(controlParams.P_ROLL_RATE*uavParam.geom.Ixx*0.1)/controlParams.N_ROLL_RATE;
%% Pitch rate
% controlParams.P_PITCH_RATE=4.4;
controlParams.P_PITCH_RATE=controlParams.P_PITCH_RATE*0.1;
controlParams.I_PITCH_RATE=0;
% controlParams.N_PITCH_RATE=100;
controlParams.D_PITCH_RATE=2*sqrt(controlParams.P_PITCH_RATE*uavParam.geom.Iyy*0.1)/controlParams.N_PITCH_RATE;
%% Yaw rate
% controlParams.P_YAW_RATE=1.77;
controlParams.P_YAW_RATE=controlParams.P_YAW_RATE*0.2;
controlParams.I_YAW_RATE=1.77;
controlParams.D_YAW_RATE=0.00701;
controlParams.N_YAW_RATE=100;
%% Design outer loop after designing inner loop.
%% Roll
controlParams.P_ROLL=18.2;
controlParams.D_ROLL=0.0454;
controlParams.N_ROLL=100;
%% Pitch
controlParams.P_PITCH=8.79;
controlParams.D_PITCH=0.442;
controlParams.N_PITCH=100;
%% Yaw
controlParams.P_YAW=5;
controlParams.D_YAW=0.04;
controlParams.N_YAW=100;
%% X Rate Controller
controlParams.P_VX=1;
controlParams.D_VX=0;
controlParams.I_VX=0;
controlParams.N_VX=0;
%% Y Rate Controller
controlParams.P_VY=0.2;
controlParams.D_VY=0;
controlParams.I_VY=0;
controlParams.N_VY=0;
%% Z Rate Controller
% controlParams.P_VZ=31.9;
controlParams.P_VZ = controlParams.P_VZ*0.1;
controlParams.I_VZ=1.4;
% controlParams.D_VZ=2.53;
controlParams.N_VZ=8.4215;
controlParams.D_VZ = 2*sqrt(controlParams.P_VZ*6.03*0.1)/controlParams.N_VZ;
% X Controller
controlParams.P_X=0.5;
controlParams.I_X=0;
controlParams.D_X=0;
controlParams.N_X=100;
%% Y Controller
% controlParams.P_Y=1.03;
controlParams.P_Y=controlParams.P_Y*0.5;
controlParams.I_Y=0;
controlParams.D_Y=0.266;
controlParams.N_Y=236.430185434264;
%% Z Controller
% controlParams.P_Z=-6.97;
controlParams.P_Z=controlParams.P_Z*0.1;
controlParams.I_Z = 0;
controlParams.D_Z=-11.2; %NOTE: IF USING MEASURED VELOCITY, D SHOULD BE POSITIVE
controlParams.N_Z=100;

%% Hover Guidance Controls
R_WAYPOINTTRANSITION=1;
R_LOOKAHEAD=5;
%% Fixed Wing Default Control Gains
%Altitude
FWControlParams.P_CLIMBRATE = 0.1;
FWControlParams.P_ALT=0.4;
FWControlParams.P_AIRSPD=10;
FWControlParams.I_AIRSPD=0;
FWControlParams.D_AIRSPD=0;
FWControlParams.N_AIRSPD=100;
%% Roll 
FWControlParams.P_FW_ROLL=2;
%Pitch
FWControlParams.P_FW_PITCH=10;
FWControlParams.I_FW_PITCH=0;
FWControlParams.D_FW_PITCH=0;
FWControlParams.N_FW_PITCH=2.18933823147713;
%Roll Rate
FWControlParams.P_FW_ROLLRATE=0.4;
FWControlParams.I_FW_ROLLRATE=0.8;
%% Pitch rate
FWControlParams.P_FW_PITCHRATE=0.15;
%Yaw rate
FWControlParams.P_FW_YAWRATE=0.01;
%% Back Transition Gains
controlParams.P_BACK=0.1;
%Tilt Max
tilt_max=pi/4;
%Minimum Allowed PWM for motors.
minPWM=0.1;
%% Filters
ForwardVelocityCutoff = 3;
SensorAAFiltNum = 4.386e+06;
SensorAAFiltDen = [1 2.96e+03 4.386e+06];
ReferenceFilterNum = 0.04877;
ReferenceFilterDen = [1 -0.9512];
