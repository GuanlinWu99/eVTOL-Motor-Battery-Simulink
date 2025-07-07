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
const.kph2mps   =   1/3.6;                                                  % [-] from km/hour to m/sec
const.mps2kph   =   3.6;                                                    % [-] from m/sec to km/h

const.kts2mps   =   0.51445;                                                % [-] from knot/sec to m/sec
const.mps2kts   =   1/0.51445;                                              % [-] from m/sec to knot/sec

const.ft2m      =   0.3048;                                                 % [-] from ft to m
const.m2ft      =   1/0.3048;                                               % [-] from m to ft
const.nm2m      =   1852;                                                   % [-] from nautical mile to m
const.m2nm      =   1/1852;                                                 % [-] from m to nautical mile

const.deg2rad   =   pi/180;                                                 % [-] from degree to radian
const.rad2deg   =   180/pi;                                                 % [-] from radian to degree

const.rpm2rps   =   1/60*2*pi;                                              % [-] from rpm to radian/sec
const.rps2rpm   =   60/2/pi;                                                % [-] from radian/sec to rpm