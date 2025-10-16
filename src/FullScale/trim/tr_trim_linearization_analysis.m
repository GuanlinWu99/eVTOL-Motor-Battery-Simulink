% ========================================================================
% Trim calculation and linearization 
% ------------------------------------------------------------------------
% Compute transition mode trim condition for a given speed/altitude and
% linearize the dynamics
%
% Inputs:
%
% Outputs (in workspace):
%
% Notes:
% ========================================================================

%.. clear workspace, command window and close fiugres
% clear all;
% clc;
% close all;

%.. load constants
% const       =   load_const();

%.. set up vtol dynamics parameters
% uavParams   =   load_vtol_dynamics_7000lb(const);

%.. trim condition computation
%.. flight conditions
% EAS_trim    =   60*const.kts2mps;                                           %.. [m/s] equivalent air speed for trim
% H_trim      =   3000*const.ft2m;                                            %.. [m] trim altitude
[~, a, ~, rho]      =   atmosisa(H_trim);                                   %.. [kg/m^3] air density at trim altitude
[~, ~, ~, rho0]     =   atmosisa(0);                                        %.. [kg/m^3] air density at sea level
vt_trim     =	EAS_trim/sqrt(rho/rho0);                                    %.. [m/s] conversion to true air speed

%.. level flight conditions
% gamma_trim      =   0.0*const.deg2rad;                                    %.. [rad] flight path angle for level-wing trim
% turn_rate_trim  =   0.0*const.deg2rad;                                    %.. [rad/s] turning rate for level coordinated-turn trim
% heading_trim    =   0.0*const.deg2rad;                                    %.. [rad] initial heading of aircraft
% alpha_trim      =   15.0*const.deg2rad;                                   %.. [rad] angle of attack constraint (might be used for climb) - initial guess for level trim

%.. trim options
%.. maximum number of function evaluations to find a trim point
options(1)  =   0;
options(2)  =   1e-8;
options(3)  =   1e-8;
options(4)  =   1e-8;
options(14) =   1e5;

%.. model specific data and parameters
%.. disable wind
% Wind        =   0;

%.. initialize landing gear model
% load("data\contact.mat")
% contact = struct('spring', 1.28931184836e5, 'vd', 0.02, ...
%                  'slidingFriction', 0.8, 'rollingFriction', 0.2, ...
%                  'gLimit', 100);

%.. load bus interfaces for plant
% load_digital_twin_interface();

%.. trim
%.. initial guess for inputs
%.. u = [fwd_thr(1)   rwd_thr(2)  tilt(3)  dA(4)  dE(5)  dR(6)]
u0  =   [0.4    rwd_thr_trim    tilt_trim    0    0    0]';

%.. initial guess/constraints for trim states
%.. x = [vt(1)      aoa(2)      aos(3)  phi/theta/psi(4:6)  p/q/r(7:9)  speed/roc/ctc]
x0  =   [vt_trim    alpha_trim	0       0 pitch_trim heading_trim    0 0 0       0 0 0]';   
dx0 =   [0    	    0           0 	    0 0 0  	            0 0 0       0 0 0]';
y0  =	[vt_trim	alpha_trim 	0       0 pitch_trim heading_trim    0 0 0	    0 0 0]';

%.. trim constraints set (hard constraints for optimization)
x_const     =   [1 2 3 4 6 7 8 9 10 11 12];
y_const     =   [1 2 3 4 6 7 8 9 10 11 12];
dx_const    =   1:12;
u_const     =   [2 4 6];

[x_trim, u_trim, y_trim, xd_trim, options]  =   trim('tr_trim_VTAOAS', x0, u0, y0, x_const, u_const, y_const, dx0, dx_const, options);

%.. post-processing of trim tilt angle
if u_trim(3) > 1
    u_trim(3)   =   1;
elseif u_trim(3) < 0
    u_trim(3)   =   0;
end

%.. display trim results
disp('================== Trim Results ==================');
fprintf('    fwd_dth_trim             =  %10.4f  (%%) \n', u_trim(1)*100);
fprintf('    fwd_rotor_speed_trim     =  %10.4f  (RPM)  \n', u_trim(1)*uavParams.motor.RPMMAX);
fprintf('    fwd_thrust_trim          =  %10.4f  (N)  \n', uavParams.rotor.Ct*(u_trim(1)*uavParams.motor.RPMMAX/60*2*pi)^2);
fprintf('    rwd_dth_trim             =  %10.4f  (%%) \n', u_trim(2)*100);
fprintf('    rwd_rotor_speed_trim     =  %10.4f  (RPM)  \n', u_trim(2)*uavParams.motor.RPMMAX);
fprintf('    rwd_thrust_trim          =  %10.4f  (N)  \n', uavParams.rotor.Ct*(u_trim(2)*uavParams.motor.RPMMAX/60*2*pi)^2);
fprintf('    elevator_trim            =  %10.4f  (deg) \n', u_trim(5)*const.rad2deg);
fprintf('    aileron_trim             =  %10.4f  (deg) \n', u_trim(4)*const.rad2deg);
fprintf('    rudder_trim              =  %10.4f  (deg) \n', u_trim(6)*const.rad2deg);
fprintf('    tilt_trim                =  %10.4f  (deg) \n', u_trim(3)*(pi/2)*const.rad2deg);
fprintf('    flap_trim                =  %10.4f  (deg) \n', flap_trim*const.rad2deg);
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
[A, B, C, D]    =   linmod('tr_linearization_VTAOAS', x_trim, [u_trim(1); u_trim(1); u_trim(2); u_trim(2); u_trim(3); u_trim(3); u_trim(4:6)]);

%.. decoupling lateral/longitudinal dynamics
%.. state variables: vt(1), aoa(2), q(8), theta(5), h(12)  
%.. output variables: vt(1), aoa(2), q(8), theta(5), h(12)  
%.. input variables: thr_fwd_rgt(1), thr_fwd_lft(2), thr_rwd_lft(3), thr_rwd_rgt(4), 
%..                  tilt_lft(5), tilt_rgt(6), dA(7), dE(8), dR(9)
A_LON   =   A([1, 2, 8, 5, 12], [1, 2, 8, 5, 12]);
B_LON   =   B([1, 2, 8, 5, 12], [1, 2, 3, 4, 7, 8, 9]);   
C_LON   =   C([1, 2, 8, 5, 12], [1, 2, 8, 5, 12]);
D_LON   =   D([1, 2, 8, 5, 12], [1, 2, 3, 4, 7, 8, 9]);  

%.. longitudinal dynamics considering control allocation
%.. input variables: pseudo_thr_dE(1), dE(2)
A_lon   =   A_LON;
B_lon   =   [sum(B_LON(:,1:2),2)-sum(B_LON(:,3:4),2) B_LON(:,6)];
C_lon   =   C_LON;
D_lon   =   [sum(D_LON(:,1:2),2)-sum(D_LON(:,3:4),2) D_LON(:,6)];

%.. state variables: aos(3), p(7), r(9), phi(4), psi(6)  
%.. output variables: aos(3), p(7), r(9), phi(4), psi(6)  
%.. input variables: thr_fwd_rgt(1), thr_fwd_lft(2), thr_rwd_lft(3), thr_rwd_rgt(4), 
%..                  tilt_lft(5), tilt_rgt(6), dA(7), dE(8), dR(9)
A_LAT   =   A([3, 7, 9, 4, 6], [3, 7, 9, 4, 6]);
B_LAT   =   B([3, 7, 9, 4, 6], [1, 2, 3, 4, 7, 8, 9]);   
C_LAT   =   C([3, 7, 9, 4, 6], [3, 7, 9, 4, 6]);
D_LAT   =   D([3, 7, 9, 4, 6], [1, 2, 3, 4, 7, 8, 9]);  

%.. lateral/directional dynamics considering control allocation
%.. input variables: pseudo_thr_dA(1), pseudo_thr_dR(2), dA(3), dR(3)
A_lat   =   A_LAT;
B_lat   =   [sum(B_LAT(:,[2,3]),2)-sum(B_LAT(:,[1,4]),2) sum(B_LAT(:,[1,3]),2)-sum(B_LAT(:,[2,4]),2) B_LAT(:,5) B_LAT(:,7)];
C_lat   =   C_LAT;
D_lat   =   [sum(D_LAT(:,[2,3]),2)-sum(D_LAT(:,[1,4]),2) sum(D_LAT(:,[1,3]),2)-sum(D_LAT(:,[2,4]),2) D_LAT(:,5) D_LAT(:,7)];