% exampleHelperInitializeVTOLGains initialize control gains for VTOL UAV.

% Copyright 2023 The MathWorks, Inc.

%Hover Default Control Based Gains
%% Roll rate
VTOLcontrolGains.P_ROLL_RATE    =   0.5*5.8920e+3;                              %160;%2.589;
VTOLcontrolGains.D_ROLL_RATE    =   1.0*139.4433;%320;%0.0166;
VTOLcontrolGains.I_ROLL_RATE    =   0.01*4.2450e+4;%0;
VTOLcontrolGains.N_ROLL_RATE    =   83.1175;%100;
%% Pitch rate
VTOLcontrolGains.P_PITCH_RATE   =   0.5*5.8936e+3;%160;%4.4;
VTOLcontrolGains.D_PITCH_RATE   =   1.0*139.6983;%320;%0.0217;
VTOLcontrolGains.I_PITCH_RATE   =   0.01*4.2439e+4;%0;
VTOLcontrolGains.N_PITCH_RATE   =   83.1175;%100;
%% Yaw rate
VTOLcontrolGains.P_YAW_RATE     =   0.005*2.4721e+5;%1.77*200;
VTOLcontrolGains.D_YAW_RATE     =   0.5*5.8759e+3;%250;%100;
VTOLcontrolGains.I_YAW_RATE     =   0.00001*1.7784e+6;%0;
VTOLcontrolGains.N_YAW_RATE     =   83.1175;%100;
%% Design outer loop after designing inner loop.
%% Roll
VTOLcontrolGains.P_ROLL         =   1.0*5.9255;%8.79;
%% Pitch
VTOLcontrolGains.P_PITCH        =   1.0*5.9253;%8.79; %100.0;%
%% Yaw
VTOLcontrolGains.P_YAW          =   0.05*180.3320;%8;%1;

%.. VTOL horizontal/lateral velocity control
VTOLcontrolGains.P_VX   =   9.8*0.0958;                                     % [-] original: 0.2, 2.5
VTOLcontrolGains.I_VX   =   1.2*0.0558;                                     % [-] original: 0
VTOLcontrolGains.D_VX   =   9.8*0.0395;                                     % [-] original: 0
VTOLcontrolGains.N_VX   =   88.8873;                                        % [-] original: 1.07046911218118
VTOLcontrolGains.P_VY   =   9.8*0.0953;                                     % [-] original: 0.2
VTOLcontrolGains.I_VY   =   1.2*0.0573;                                     % [-] original: 0
VTOLcontrolGains.D_VY   =   9.8*0.0375;                                     % [-] original: 0.5518
VTOLcontrolGains.N_VY   =   88.8873;                                        % [-] original: 1.07046911218118
%.. VTOL vertical velocity control
VTOLcontrolGains.P_VZ   =   0.7*345.0381;                                   % [-] original: 15
VTOLcontrolGains.I_VZ   =   0.01*526.2230;                                  % [-] original: 0
VTOLcontrolGains.D_VZ   =   0.9*136.1529;                                   % [-] original: 90
VTOLcontrolGains.N_VZ   =   10.2217;                                        % [-] original: 8.4215

%.. VTOL horizontal/lateral position control
VTOLcontrolGains.P_X    =   1.5*0.2001;                                     % [-] original: 1.03, 0.75
VTOLcontrolGains.P_Y    =   1.5*0.2071;                                     % [-] original: 1.03
%.. VTOL altitude control
VTOLcontrolGains.P_Z    =   0.3*1.4317;                                     % [-] original: 13.79

TransitioncontrolGains.zeta     =    0.505;
TransitioncontrolGains.L        =    50;
TransitioncontrolGains.P_YAW    =    1;


%% Hover Guidance Controls
% R_WAYPOINTTRANSITION=1;
% R_LOOKAHEAD=5;
%% Fixed Wing Default Control Gains
%Altitude
FWControlParams.P_CLIMBRATE =   10*0.1; % 0.1
FWControlParams.P_ALT       =   0.3; %0.4
FWControlParams.P_AIRSPD    =   360;%10;
FWControlParams.I_AIRSPD    =   5;%0;
FWControlParams.D_AIRSPD    =   120;%%0;
FWControlParams.N_AIRSPD    =   100;
%% Roll 
FWControlParams.P_FW_ROLL=2*2;%*4
%Pitch
FWControlParams.P_FW_PITCH=6;%10
FWControlParams.I_FW_PITCH=0;
FWControlParams.D_FW_PITCH=0;
FWControlParams.N_FW_PITCH=2.18933823147713;
%Roll Rate
FWControlParams.P_FW_ROLLRATE=0.4*3;%*10
FWControlParams.I_FW_ROLLRATE=0.8*1;%*4
%% Pitch rate
FWControlParams.P_FW_PITCHRATE=0.15*3;
%Yaw rate
FWControlParams.P_FW_YAWRATE=0.01*40;%*80

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
% controlParams.fwd_max_slewrate = 300/180*pi;
% controlParams.fwd_max_angle = 30/180*pi;
% controlParams.swd_max_slewrate = 300/180*pi;
% controlParams.swd_max_angle = 15/180*pi;