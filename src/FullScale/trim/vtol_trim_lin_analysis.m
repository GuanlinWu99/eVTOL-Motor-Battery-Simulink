%=========================================================================%
% trim calculation and linearization                                      %
%-------------------------------------------------------------------------%
% author(s): minhyun                                                      %
% description: computing trim condition of fixed-wing mode and            %
%              linearize model to enable auto-tuner                       %
%=========================================================================%

%.. clear workspace, command window and close fiugres
% clear all;
% clc;
% close all;

%.. load constants
const       =   load_const;

%.. set up vtol dynamics parameters
uavParam    =   load_vtol_dynamics_7000lb;

%.. trim condition computation
%.. flight conditions
% EAS_trim    =   60*const.kts2mps;                                           %.. [m/s] equivalent air speed for trim
% H_trim      =	3000*const.ft2m;                                            %.. [m] trim altitude
[~, a, ~, rho]      =   atmosisa(H_trim);                                   %.. [kg/m^3] air density at trim altitude
[~, ~, ~, rho0]     =   atmosisa(0);                                        %.. [kg/m^3] air density at sea level
vt_trim     =	EAS_trim/sqrt(rho/rho0);                                    %.. [m/s] conversion to true air speed

%.. level flight conditions
gamma_trim      =   0.0*const.deg2rad;                                      %.. [rad] flight path angle for level-wing trim
turn_rate_trim  =   0.0*const.deg2rad;                                      %.. [rad/s] turning rate for level coordinated-turn trim
heading_trim    =   0.0*const.deg2rad;                                      %.. [rad] initial heading of aircraft
alpha_trim      =   15.0*const.deg2rad;                                     %.. [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

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
%.. u = [thr(1) dE(2)   dA(3)   dR(4)]
u0  =   [0.6    0       0       0]';

%.. initial guess/constraints for trim states
%.. x = [vt(1)      aoa(2)      aos(3)  phi/theta/psi(4:6)  p/q/r(7:9)  speed/roc/ctc]
x0  =   [vt_trim    alpha_trim	0       0 0 heading_trim    0 0 0       0 0 0]';   
dx0 =   [0    	    0           0 	    0 0 0  	            0 0 0       0 0 0]';
y0  =	[vt_trim	alpha_trim 	0       0 0 heading_trim    0 0 0	    0 0 0]';

%.. trim constraints set (hard constraints for optimization)
x_const     =   [1 3 4 6 7 8 9 10 11 12];
y_const     =   [1 3 4 6 7 8 9 10 11 12];
dx_const    =   1:12;
u_const     =   [3 4];

[x_trim, u_trim, y_trim, xd_trim, options]  =   trim('fw_trim_VTAOAS', x0, u0, y0, x_const, u_const, y_const, dx0, dx_const, options);

%.. display trim results
disp('================== Trim Results ==================');
fprintf('    dth_trim                 =  %10.4f  (%%) \n', u_trim(1)*100);
fprintf('    rotor_speed_trim         =  %10.4f  (RPM)  \n', u_trim(1)*uavParam.motor.RPMMAX);
fprintf('    elevator_trim            =  %10.4f  (deg) \n', u_trim(2)*const.rad2deg);
fprintf('    aileron_trim             =  %10.4f  (deg) \n', u_trim(3)*const.rad2deg);
fprintf('    rudder_trim              =  %10.4f  (deg) \n', u_trim(4)*const.rad2deg);
fprintf('    trim_speed (true)        =  %10.4f  (km/h) \n', x_trim(1)*const.mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', x_trim(1));
fprintf('                             =  %10.4f  (kts) \n', x_trim(1)*const.mps2kts);
fprintf('               (equivalent)  =  %10.4f  (km/h) \n', EAS_trim*const.mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', EAS_trim);
fprintf('                             =  %10.4f  (kts) \n', EAS_trim*const.mps2kts);
fprintf('    trim_altitude            =  %10.4f  (m) \n', H_trim);
fprintf('                             =  %10.4f  (ft) \n', H_trim*const.m2ft);
fprintf('    roll_trim                =  %10.4f  (deg) \n', x_trim(4)/const.deg2rad);
fprintf('    pitch_trim               =  %10.4f  (deg) \n', x_trim(5)/const.deg2rad);
fprintf('    heading_trim             =  %10.4f  (deg) \n', x_trim(6)/const.deg2rad);
fprintf('    alpha_trim               =  %10.4f  (deg) \n', x_trim(2)/const.deg2rad);
fprintf('    beta_trim                =  %10.4f  (deg) \n', x_trim(3)/const.deg2rad);
fprintf('    flight_path_trim         =  %10.4f  (deg) \n\n', gamma_trim/const.deg2rad);    
disp('--------------------------------------------------');

%.. linearization
[A_fw, B_fw, C_fw, D_fw]    =   linmod('fw_trim_VTAOAS', x_trim, u_trim);

%.. decoupling lateral/longitudinal dynamics
%.. state variables: vt(1), aoa(2), q(8), theta(5), h(12)  
%.. output variables: vt(1), aoa(2), q(8), theta(5), h(12)  
%.. input variables: thr(1), dE(2), dA(3), dR(4)
A_fw_LON    =   A_fw([1, 2, 8, 5, 12], [1, 2, 8, 5, 12]);
B_fw_LON    =   B_fw([1, 2, 8, 5, 12], [1, 2, 3, 4]);    
C_fw_LON    =   C_fw([1, 2, 8, 5, 12], [1, 2, 8, 5, 12]);
D_fw_LON    =   D_fw([1, 2, 8, 5, 12], [1, 2, 3, 4]);

%.. longitudinal dynamics considering elevator
A_LON       =   A_fw_LON;
B_LON       =   B_fw_LON(:,2);
C_LON       =   C_fw_LON;
D_LON       =   D_fw_LON(:,2);

%.. state variables: aos(3), p(7), r(9), phi(4), psi(6)  
%.. output variables: aos(3), p(7), r(9), phi(4), psi(6)  
%.. input variables: thr(1), dE(2), dA(3), dR(4)
A_fw_LAT    =   A_fw([3, 7, 9, 4, 6], [3, 7, 9, 4, 6]);
B_fw_LAT    =   B_fw([3, 7, 9, 4, 6], [1, 2, 3, 4]); 
C_fw_LAT    =   C_fw([3, 7, 9, 4, 6], [3, 7, 9, 4, 6]);
D_fw_LAT    =   D_fw([3, 7, 9, 4, 6], [1, 2, 3, 4]);

%.. Lateral/Directional Dynamics(5th Order) for Control Allocation
A_LAT       =   A_fw_LAT;
B_LAT       =   B_fw_LAT(:,3:4);
C_LAT       =   C_fw_LAT;
D_LAT       =   D_fw_LAT(:,3:4);