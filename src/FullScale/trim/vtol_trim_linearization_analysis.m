%=========================================================================%
% trim calculation and linearization                                      %
%-------------------------------------------------------------------------%
% author(s): minhyun                                                      %
% description: computing trim condition of VTOL mode and                  %
%              linearize model to enable controller design (auto-tuner)   %
%=========================================================================%

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
% EAS_trim    =   0*const.kts2mps;                                          %.. [m/s] equivalent air speed for trim
% H_trim      =	300*const.ft2m;                                             %.. [m] trim altitude
[~, a, ~, rho]      =   atmosisa(H_trim);                                   %.. [kg/m^3] air density at trim altitude
[~, ~, ~, rho0]     =   atmosisa(0);                                        %.. [kg/m^3] air density at sea level

%.. trim options
%.. Maximum Number of Function Evaluations to Find a Trim Point
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
%.. u = [w1(1)  w2(2)   w3(3)   w4(4)]
u0  =   [0.6    0.6     0.6     0.6]';

%.. initial guess/constraints for trim states
%.. x = [u/v/w(1:3)  phi/theta/psi(4:6)  p/q/r(7:9)]
x0  =   zeros(1,9)';   
dx0 =   zeros(1,9)';
y0  =	zeros(1,9)';

%.. trim constraints set (hard constraints for optimization)
x_const     =   1:9;
y_const     =   1:9;
dx_const    =   1:9;
u_const     =   [];

[x_trim, u_trim, y_trim, xd_trim, options]  =   trim('vtol_trim_UVW', x0, u0, y0, x_const, u_const, y_const, dx0, dx_const, options);

%.. display trim results
disp('================== Trim Results ==================');
fprintf('    dth_trim                 =  %10.4f  (%%) \n', u_trim(1)*100);
fprintf('    rotor_speed_trim         =  %10.4f  (RPM)  \n', u_trim(1)*uavParams.motor.RPMMAX);
fprintf('    trim_speed (true)        =  %10.4f  (km/h) \n', norm(x_trim(4:6))*const.mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', norm(x_trim(4:6)));
fprintf('                             =  %10.4f  (kts) \n', norm(x_trim(4:6))*const.mps2kts);
fprintf('               (equivalent)  =  %10.4f  (km/h) \n', EAS_trim*const.mps2kph);
fprintf('                             =  %10.4f  (m/s) \n', EAS_trim);
fprintf('                             =  %10.4f  (kts) \n', EAS_trim*const.mps2kts);
fprintf('    trim_altitude            =  %10.4f  (m) \n', H_trim);
fprintf('                             =  %10.4f  (ft) \n', H_trim*const.m2ft);
fprintf('    roll_trim                =  %10.4f  (deg) \n', x_trim(4)/const.deg2rad);
fprintf('    pitch_trim               =  %10.4f  (deg) \n', x_trim(5)/const.deg2rad);
fprintf('    heading_trim             =  %10.4f  (deg) \n', x_trim(6)/const.deg2rad); 
disp('--------------------------------------------------');

%.. linearization
[A_vtol, B_vtol, C_vtol, D_vtol]    =   linmod('vtol_linearization_UVW', [zeros(3,1); x_trim], u_trim);

%.. decoupling lateral/longitudinal dynamics
%.. state variables: x(1), u(4), theta(8), q(11)
%.. output variables: x(1), u(4), theta(8), q(11)  
%.. input variables: w1(1), w2(2), w3(3), w4(4)
A_vtol_LON  =   A_vtol([1, 4, 8, 11], [1, 4, 8, 11]);
B_vtol_LON  =   B_vtol([1, 4, 8, 11], [1, 2, 3, 4]);    
C_vtol_LON  =   C_vtol([1, 4, 8, 11], [1, 4, 8, 11]);
D_vtol_LON  =   D_vtol([1, 4, 8, 11], [1, 2, 3, 4]);

%.. longitudinal dynamics considering control allocation
A_vtol_lon  =   A_vtol_LON;
B_vtol_lon  =   sum(B_vtol_LON(:,1:2),2)-sum(B_vtol_LON(:,3:4),2);
C_vtol_lon  =   C_vtol_LON;
D_vtol_lon  =   sum(D_vtol_LON(:,1:2),2)-sum(D_vtol_LON(:,3:4),2);

%.. state variables: y(2), v(5), phi(7), p(10)
%.. output variables: y(2), v(5), phi(7), p(10)  
%.. input variables: w1(1), w2(2), w3(3), w4(4)
A_vtol_LAT  =   A_vtol([2, 5, 7, 10], [2, 5, 7, 10]);
B_vtol_LAT  =   B_vtol([2, 5, 7, 10], [1, 2, 3, 4]); 
C_vtol_LAT  =   C_vtol([2, 5, 7, 10], [2, 5, 7, 10]);
D_vtol_LAT  =   D_vtol([2, 5, 7, 10], [1, 2, 3, 4]);

%.. lateral dynamics considering control allocation
A_vtol_lat  =   A_vtol_LAT;
B_vtol_lat  =   sum(B_vtol_LAT(:,[2,3]),2)-sum(B_vtol_LAT(:,[1,4]),2);
C_vtol_lat  =   C_vtol_LAT;
D_vtol_lat  =   sum(D_vtol_LAT(:,[2,3]),2)-sum(D_vtol_LAT(:,[1,4]),2);

%.. state variables: psi(9), r(12)
%.. output variables: psi(9), r(12) 
%.. input variables: w1(1), w2(2), w3(3), w4(4)
A_vtol_DIR  =   A_vtol([9, 12], [9, 12]);
B_vtol_DIR  =   B_vtol([9, 12], [1, 2, 3, 4]); 
C_vtol_DIR  =   C_vtol([9, 12], [9, 12]);
D_vtol_DIR  =   D_vtol([9, 12], [1, 2, 3, 4]);

%.. directional dynamics considering control allocation
A_vtol_dir  =   A_vtol_DIR;
B_vtol_dir  =   sum(B_vtol_DIR(:,[1,3]),2)-sum(B_vtol_DIR(:,[2,4]),2);
C_vtol_dir  =   C_vtol_DIR;
D_vtol_dir  =   sum(D_vtol_DIR(:,[1,3]),2)-sum(D_vtol_DIR(:,[2,4]),2);

%.. state variables: z(3), w(6)
%.. output variables: z(3), w(6) 
%.. input variables: w1(1), w2(2), w3(3), w4(4)
A_vtol_ALT  =   A_vtol([3, 6], [3, 6]);
B_vtol_ALT  =   B_vtol([3, 6], [1, 2, 3, 4]); 
C_vtol_ALT  =   C_vtol([3, 6], [3, 6]);
D_vtol_ALT  =   D_vtol([3, 6], [1, 2, 3, 4]);

%.. vertical dynamics considering control allocation
A_vtol_alt  =   A_vtol_ALT;
B_vtol_alt  =   sum(B_vtol_ALT,2);
C_vtol_alt  =   C_vtol_ALT;
D_vtol_alt  =   sum(D_vtol_ALT,2);