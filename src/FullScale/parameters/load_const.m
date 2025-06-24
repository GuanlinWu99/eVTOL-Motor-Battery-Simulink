%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function const = load_const()

%% load constants
%.. constants
const.rho0  =   1.225;                                                      % [kg/m^3] air density @ zero altitude of standard atmosphere model 
const.g     =   9.806;                                                      % [m/s^2] gravitational acceleration

%.. unit conversion
const.kph2mps   =   1/3.6;                                                  % [-]
const.mps2kph   =   3.6;                                                    % [-]

const.kts2mps   =   0.51445;                                                % [-]
const.mps2kts   =   1/0.51445;                                              % [-]

const.ft2m      =   0.3048;                                                 % [-]
const.m2ft      =   1/0.3048;                                               % [-]
const.nm2m      =   1852;                                                   % [-]

const.deg2rad   =   pi/180;                                                 % [-]
const.rad2deg   =   180/pi;                                                 % [-]