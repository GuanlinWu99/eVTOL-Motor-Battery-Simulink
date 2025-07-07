% exampleHelperInitializeVTOLGains initialize control gains for VTOL UAV.

% Copyright 2023 The MathWorks, Inc.

%Hover Default Control Based Gains
%% Roll rate
VTOLcontrolGains.P_ROLL_RATE=160;%2.589;
VTOLcontrolGains.D_ROLL_RATE=320;%0.0166;
VTOLcontrolGains.I_ROLL_RATE=0;
VTOLcontrolGains.N_ROLL_RATE=100;
%% Pitch rate
VTOLcontrolGains.P_PITCH_RATE=160;%4.4;
VTOLcontrolGains.D_PITCH_RATE=320;%0.0217;
VTOLcontrolGains.I_PITCH_RATE=0;
VTOLcontrolGains.N_PITCH_RATE=100;
%% Yaw rate
VTOLcontrolGains.P_YAW_RATE=1.77*150;
VTOLcontrolGains.D_YAW_RATE=250;%100;
VTOLcontrolGains.I_YAW_RATE=0;
VTOLcontrolGains.N_YAW_RATE=100;
%% Design outer loop after designing inner loop.
%% Roll
VTOLcontrolGains.P_ROLL=8.79;
%% Pitch
VTOLcontrolGains.P_PITCH=8.79; %100.0;%
%% Yaw
VTOLcontrolGains.P_YAW=1;%1;
%% X Rate Controller
VTOLcontrolGains.P_VX=0.2;%1*2.5;
VTOLcontrolGains.I_VX = 0;
VTOLcontrolGains.D_VX = 0;
VTOLcontrolGains.N_VX = 1.07046911218118;
%% Y Rate Controller
VTOLcontrolGains.P_VY= 0.2;
VTOLcontrolGains.I_VY = 0;
VTOLcontrolGains.D_VY = 0.5518;
VTOLcontrolGains.N_VY = 1.07046911218118;
%% Z Rate Controller
VTOLcontrolGains.P_VZ=15;
VTOLcontrolGains.I_VZ=0;
VTOLcontrolGains.D_VZ=90;
VTOLcontrolGains.N_VZ=8.4215;
%% X Controller
VTOLcontrolGains.P_X=-1.03;%0.5*1.5;
%% Y Controller
VTOLcontrolGains.P_Y=-1.03;
%% Z Controller
VTOLcontrolGains.P_Z= 1.97*7.0;
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
FWControlParams.P_FW_ROLL=2*5;
%Pitch
FWControlParams.P_FW_PITCH=10;
FWControlParams.I_FW_PITCH=0;
FWControlParams.D_FW_PITCH=0;
FWControlParams.N_FW_PITCH=2.18933823147713;
%Roll Rate
FWControlParams.P_FW_ROLLRATE=0.4*10;
FWControlParams.I_FW_ROLLRATE=0.8*4;
%% Pitch rate
FWControlParams.P_FW_PITCHRATE=0.15*3;
%Yaw rate
FWControlParams.P_FW_YAWRATE=0.01*80;

FWControlParams.P_FW_AOS = 0.5;
FWControlParams.I_FW_AOS = 0.0;

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

%% Custom
controlParams.fwd_max_slewrate = 300/180*pi;
controlParams.fwd_max_angle = 30/180*pi;
controlParams.swd_max_slewrate = 300/180*pi;
controlParams.swd_max_angle = 15/180*pi;