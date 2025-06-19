%=========================================================================%
% trim calculation and linearization                                      %
%-------------------------------------------------------------------------%
% author(s): minhyun                                                      %
% description: computing trim condition of fixed-wing mode and            %
%              linearize model to enable auto-tuner                       %
%=========================================================================%

%.. clear workspace, command window and close fiugres
clear all;
clc;
close all;

%.. load constants
kph2mps     =   1/3.6;
mps2kph     =   3.6;
kts2mps     =   0.51445;
mps2kts     =   1/0.51445;
ft2m        =   0.3048;
m2ft        =   1/0.3048;
deg2rad     =   pi/180;
rad2deg     =   180/pi;

%.. set up vtol dynamics parameters
uavParam    =   load_vtol_dynamics_7000lb;

%.. trim condition computation
%.. flight conditions
EAS_trim    =   60*kts2mps;                                                 %.. [m/s] equivalent air speed for trim
H_trim      =	3000*ft2m;                                                  %.. [m] trim altitude
[~, a, ~, rho]      =   atmosisa(H_trim);                                   %.. [kg/m^3] air density at trim altitude
[~, ~, ~, rho0]     =   atmosisa(0);                                        %.. [kg/m^3] air density at sea level
vt_trim     =	EAS_trim/sqrt(rho/rho0);                                    %.. [m/s] conversion to true air speed

%.. level flight conditions
gamma_trim      =   0.0*deg2rad;                                            %.. [rad] flight path angle for level-wing trim
turn_rate_trim  =   0.0*deg2rad;                                            %.. [rad/s] turning rate for level coordinated-turn trim
heading_trim    =   0.0*deg2rad;                                            %.. [rad] initial heading of aircraft
alpha_trim      =   15.0*pi/180;                                            %.. [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

%.. trim options
%.. Maximum Number of Function Evaluations to Find a Trim Point
options(1)  =   0;
options(2)  =   1e-8;
options(3)  =   1e-8;
options(4)  =   1e-8;
options(14) =   1e5;

%.. model specific data and parameters
%.. disable wind
Wind        =   0;

%.. initialize landing gear model
load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, ...
%                  'slidingFriction', 0.8, 'rollingFriction', 0.2, ...
%                  'gLimit', 100);

%.. load bus interfaces for plant
define_digital_twin_interface;

%.. trim
%.. initial guess for inputs
u0  =   [0.6    0   0   0]';

%.. initial guess/constraints for trim states
%.. x = [v(1)       aoa(2)      aos(3)  phi/theta/psi(4:6)  p/q/r(7:9)  speed/roc/ctc]
x0  =   [vt_trim    alpha_trim	0       0 0 heading_trim    0 0 0       0 0 0]';   
dx0 =   [0    	    0           0 	    0 0 0  	            0 0 0       0 0 0]';
y0  =	[vt_trim	alpha_trim 	0       0 0 heading_trim    0 0 0	    0 0 0]';

%.. trim constraints set for
x_const     =   [1 3 4 5 6 7 9 10 11 12];
y_const     =   [1 3 4 5 6 7 9 10 11 12];
dx_const    =   1:12;
u_const     =   [2 4];

[x_trim, u_trim, y_trim, xd_trim, options]  =   trim('fw_trim_VTAOAS', x0, u0, y0, x_const, u_const, y_const);

%.. display trim results
disp('================== Trim Results ==================');
fprintf('    dth_trim                 =  %10.4f  (%%) \n', u_trim(1)*100);
fprintf('    rotor_speed_trim         =  %10.4f  (RPM)  \n', u_trim(1)*uavParam.motor.RPMMAX);
fprintf('    elevator_trim            =  %10.4f  (deg) \n', u_trim(2)*rad2deg);
fprintf('    aileron_trim             =  %10.4f  (deg) \n', u_trim(3)*rad2deg);
fprintf('    rudder_trim              =  %10.4f  (deg) \n', u_trim(4)*rad2deg);
fprintf('    trim_speed (true)        =  %10.4f  (km/h) \n', x_trim(1)*mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', x_trim(1));
fprintf('                             =  %10.4f  (kts) \n', x_trim(1)*mps2kts);
fprintf('               (equivalent)  =  %10.4f  (km/h) \n', EAS_trim*mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', EAS_trim);
fprintf('                             =  %10.4f  (kts) \n', EAS_trim*mps2kts);
fprintf('    trim_altitude            =  %10.4f  (m) \n', H_trim);
fprintf('                             =  %10.4f  (ft) \n', H_trim*m2ft);
fprintf('    roll_trim                =  %10.4f  (deg) \n', x_trim(4)/deg2rad);
fprintf('    pitch_trim               =  %10.4f  (deg) \n', x_trim(5)/deg2rad);
fprintf('    heading_trim             =  %10.4f  (deg) \n', x_trim(6)/deg2rad);
fprintf('    alpha_trim               =  %10.4f  (deg) \n', x_trim(2)/deg2rad);
fprintf('    beta_trim                =  %10.4f  (deg) \n', x_trim(3)/deg2rad);
fprintf('    flight_path_trim         =  %10.4f  (deg) \n\n', gamma_trim/deg2rad);    
disp('--------------------------------------------------');

