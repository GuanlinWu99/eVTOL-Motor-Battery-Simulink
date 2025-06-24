%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uavParam = load_vtol_dynamics_7000lb()

%% define aerodynamics
%.. lift coefficients
% uavParam.aero.CL0   =   0.81857;                                            % [-] CL @ AOA = 0
% uavParam.aero.CLa   =   4.09127;                                            % [-] CL-alpha slope
uavParam.aero.CLa_dot   =   0;                                              % [-] CL-dot(alpha) slope
uavParam.aero.CLq       =   7.31097;                                        % [-] CL-q slope
uavParam.aero.CLDe      =   0.50787;                                        % [-] CL-elevator slope
uavParam.aero.CLDa      =   -0.85291;                                       % [-] CL-aileron slope

%.. drag coefficients
% uavParam.aero.CDmin     =   0.06047;                                        % [-] minimum drag 
% uavParam.aero.K         =   0.1328;                                         % [-] drag polar quadratic coefficient
% uavParam.aero.CL_CDmin  =   0.4806;                                         % [-] CL at minimum CD point
% uavParam.aero.A1        =   0;                                              % [-] scale factor for drag model
uavParam.aero.CdDe      =   0.016043;                                       % [-] CD-elevator slope
uavParam.aero.CdDa      =   0.092132;                                       % [-] CD-aileron slope
uavParam.aero.CdDr      =   0.0097403;                                      % [-] CD-rudder slope
% uavParam.aero.dragCoeffMov  =   0.027;                                      % [-] fuselage moving part drag coefficient

%.. moment coefficient
% uavParam.aero.Cm0       =   0.00763;                                        % [-] CM @ AOA = 0
% uavParam.aero.Cma       =   -1.76966;                                       % [-] CM-alpha slope
uavParam.aero.Cmq       =   -19.22663;                                      % [-] pitch damping
uavParam.aero.CmDe      =   -1.83747*1.4;                                   % [-] CM-elevator slope
% uavParam.aero.CmDr      =   0.02991;                                        % [-] CM-rudder slope

%.. side force
% uavParam.aero.CYb       =   -0.001570;                                      % [-] CY-beta slope
uavParam.aero.CYp       =   -0.001570;                                      % [-] CY-p slope
uavParam.aero.CYr       =   0.18368;                                        % [-] CY-r slope
uavParam.aero.CYDr      =   0.128915;                                       % [-] CY-rudder slope

%.. roll moment
% uavParam.aero.Clb       =   -0.001610;                                      % [-] Cl-beta slope
uavParam.aero.Clp       =   -0.47584;                                       % [-] Cl-p slope
uavParam.aero.Clr       =   0.27153;                                        % [-] Cl-r slope
uavParam.aero.ClDa      =   0.286593;                                       % [-] Cl-aileron slope
uavParam.aero.ClDr      =   0.013407;                                       % [-] Cl-rudder slope

%.. yaw moment
% uavParam.aero.Cnb       =   0.09180;                                        % [-] Cn-beta slope
uavParam.aero.Cnp       =   -0.13267;                                       % [-] Cn-p slope
uavParam.aero.Cnr       =   -0.08829;                                       % [-] Cn-r slope
uavParam.aero.CnDa      =   0.0049274;                                      % [-] Cn-aileron slope
uavParam.aero.CnDr      =   -0.067036;                                      % [-] Cn-rudder slope

% .. load custom aerodynamic coefficients from openVSPaero
[alpha_temp, CL_temp, CD_temp, CM_temp]     =   eVTOL_data_alpha();
[beta_temp, CY_temp, CR_temp, CN_temp]      =   eVTOL_data_beta();          % [-] CR = Cl

uavParam.aero.alpha     =   alpha_temp/180*pi;
uavParam.aero.beta      =   beta_temp/180*pi;
uavParam.aero.CL        =   CL_temp;
uavParam.aero.CD        =   CD_temp;
uavParam.aero.CM        =   CM_temp;
uavParam.aero.CY        =   CY_temp;
uavParam.aero.CR        =   CR_temp;
uavParam.aero.CN        =   CN_temp;

%% scale up the parameters
n                   =   14/2;                                               % [-] lenth scale parameter
sigma               =   (3175/6.025)/n^3;                                   % [-] density scale parameter

%.. geometry
uavParam.geom.b     =       n*2;                                            % [m] full-scale UAV span
uavParam.geom.AR    =       7.2727;                                         % [-] AR ratio of wing
uavParam.geom.c     =       n*0.275;                                        % [m] full-scale UAV chord
uavParam.geom.e     =       0.986;                                          % [-] span efficiency factor (suppose inviscid flow)
uavParam.geom.RotorArm1     =   n*[ 0.375  0.375 0];                        % [m] full-scale propeller #1 location
uavParam.geom.RotorArm2     =   n*[ 0.375 -0.375 0];                        % [m] full-scale propeller #2 location
uavParam.geom.RotorArm3     =   n*[-0.375 -0.375 0];                        % [m] full-scale propeller #3 location
uavParam.geom.RotorArm4     =   n*[-0.375  0.375 0];                        % [m] full-scale propeller #4 location

%.. mass and inertia
uavParam.geom.mass  =   sigma*n^3*6.023;                                    % [kg] full-scale UAV mass
uavParam.geom.Ixx   =   sigma*n^5*(0.089*3);                                % [kg*m^5] full-scale UAV x-inertia
uavParam.geom.Iyy   =   sigma*n^5*(0.089*3);                                % [kg*m^5] full-scale UAV y-inertia
uavParam.geom.Izz   =   sigma*n^5*(0.125*3);                                % [kg*m^5] full-scale UAV z-inertia
uavParam.geom.Ixz   =   0.0;                                                % [kg*m^5] full-scale UAV xz-product inertia

%.. motor
uavParam.motor.RPMMAX       =   10000;                                      % [rpm] maximum rpm of motors
uavParam.motor.tilt_trim    =   0;                                          % [rad] motor tilt trim angle 
uavParam.motor.minPWM       =   0.1;                                        % [-] motor minimum pwm level
uavParam.motor.maxPWM       =   1.0;                                        % [-] motor maximum pwm level

%.. adjust the thrust coefficient
uavParam.rotor.Ct           =   4/(pi^3)*0.1142;                            % [-] thrust coefficient of rotor
uavParam.rotor.Cq           =   8/(pi^3)*0.007048;                          % [-] torque coefficient of rotor
uavParam.rotor.N            =   3;                                          % [-] number of blades
uavParam.geom.PropDiameter  =   (2/3)*n*0.3052;                             % [m] full-scale UAV propeller diameter
uavParam.geom.PropChord     =   n*0.0080;                                   % [m] rotor blade chord
uavParam.geom.PropOffHinge  =   0;                                          % [-] offhinge angle
uavParam.geom.PropCLa       =   5.5;                                        % [-] propeller CL-alpha slope
uavParam.geom.PropLock      =   0.6051;                                     % [-] lock number
uavParam.geom.Propblroot    =   14.5990/180*pi;                             % [rad] blade root aoa
uavParam.geom.Propbltwist   =   -7.7980/180*pi;                             % [rad] blade tip aoa

end
