%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uavParams = load_vtol_dynamics_7000lb(const)

%% define aerodynamics
% .. load custom aerodynamic coefficients from openVSPaero
[alpha_lon_temp, CL_temp, CD_temp, CM_temp]                         =   eVTOL_data_lon();
[alpha_latdir_temp, beta_latdir_temp, CS_temp, CR_temp, CN_temp]    =   eVTOL_data_latdir();        % [-] CR = Cl
uavParams.aero.alpha_lon    =   alpha_lon_temp*const.deg2rad;
uavParams.aero.alpha_latdir =   alpha_latdir_temp*const.deg2rad;
uavParams.aero.beta_latdir  =   beta_latdir_temp*const.deg2rad;

%.. lift coefficients
% uavParams.aero.CL0      =   0.81857;                                        % [-] CL @ AOA = 0
% uavParams.aero.CLa      =   4.09127;                                        % [-] CL-alpha slope
uavParams.aero.CL       =   CL_temp;                                        % [-] CL
uavParams.aero.CLadot   =   0;                                              % [-] CL-dot(alpha) slope
uavParams.aero.CLq      =   7.31097;                                        % [-] CL-q slope
uavParams.aero.CLDe     =   0.50787;                                        % [-] CL-elevator slope
uavParams.aero.CLDa     =   -0.85291;                                       % [-] CL-aileron slope
uavParams.aero.CLadot_tfnum =   [1 0];
uavParams.aero.CLadot_tfden =   [0.1 1];

%.. drag coefficients
% uavParams.aero.CDmin    =   0.06047;                                        % [-] minimum drag 
% uavParams.aero.K        =   0.1328;                                         % [-] drag polar quadratic coefficient
% uavParams.aero.CL_CDmin =   0.4806;                                         % [-] CL at minimum CD point
% uavParams.aero.A1       =   0;                                              % [-] scale factor for drag model
uavParams.aero.CD       =   CD_temp;                                        % [-] CD
uavParams.aero.CdDe     =   0.016043;                                       % [-] CD-elevator slope
uavParams.aero.CdDa     =   0.092132;                                       % [-] CD-aileron slope
uavParams.aero.CdDr     =   0.0097403;                                      % [-] CD-rudder slope
% uavParams.aero.dragCoeffMov =   0.027;                                      % [-] fuselage moving part drag coefficient

%.. moment coefficient
% uavParams.aero.Cm0      =   0.00763;                                        % [-] CM @ AOA = 0
% uavParams.aero.Cma      =   -1.76966;                                       % [-] CM-alpha slope
uavParams.aero.CM       =   CM_temp;                                        % CM
uavParams.aero.Cmq      =   -19.22663;                                      % [-] pitch damping
uavParams.aero.CmDe     =   -1.83747*1.4;                                   % [-] CM-elevator slope
% uavParams.aero.CmDr     =   0.02991;                                        % [-] CM-rudder slope

%.. side force
% uavParams.aero.CYb      =   -0.001570;                                      % [-] CY-beta slope
uavParams.aero.CS       =   CS_temp;                                        % [-] CS
uavParams.aero.CYp      =   -0.001570;                                      % [-] CY-p slope
uavParams.aero.CYr      =   0.18368;                                        % [-] CY-r slope
uavParams.aero.CYDr     =   0.128915;                                       % [-] CY-rudder slope

%.. roll moment
% uavParams.aero.Clb      =   -0.001610;                                      % [-] Cl-beta slope
uavParams.aero.CR       =   CR_temp;                                        % CR
uavParams.aero.Clp      =   -0.47584;                                       % [-] Cl-p slope
uavParams.aero.Clr      =   0.27153;                                        % [-] Cl-r slope
uavParams.aero.ClDa     =   0.286593;                                       % [-] Cl-aileron slope
uavParams.aero.ClDr     =   0.013407;                                       % [-] Cl-rudder slope

%.. yaw moment
% uavParams.aero.Cnb      =   0.09180;                                        % [-] Cn-beta slope
uavParams.aero.CN       =   CN_temp;                                        % CN
uavParams.aero.Cnp      =   -0.13267;                                       % [-] Cn-p slope
uavParams.aero.Cnr      =   -0.08829;                                       % [-] Cn-r slope
uavParams.aero.CnDa     =   0.0049274;                                      % [-] Cn-aileron slope
uavParams.aero.CnDr     =   -0.067036*3;                                      % [-] Cn-rudder slope


%% scale up the parameters
n                   =   14/2;                                               % [-] lenth scale parameter
sigma               =   (3175/6.025)/n^3;                                   % [-] density scale parameter

%.. geometry
uavParams.geom.b    =       n*2;                                            % [m] full-scale UAV span
uavParams.geom.AR   =       7.2727;                                         % [-] AR ratio of wing
uavParams.geom.c    =       n*0.275;                                        % [m] full-scale UAV chord
uavParams.geom.e    =       0.986;                                          % [-] span efficiency factor (suppose inviscid flow)
uavParams.geom.RotorArm1    =   n*[ 0.375  0.375 0];                        % [m] full-scale propeller #1 location
uavParams.geom.RotorArm2    =   n*[ 0.375 -0.375 0];                        % [m] full-scale propeller #2 location
uavParams.geom.RotorArm3    =   n*[-0.375 -0.375 0];                        % [m] full-scale propeller #3 location
uavParams.geom.RotorArm4    =   n*[-0.375  0.375 0];                        % [m] full-scale propeller #4 location

%.. mass and inertia
uavParams.geom.mass =   sigma*n^3*6.023;                                    % [kg] full-scale UAV mass
uavParams.geom.Ixx  =   sigma*n^5*(0.089*3);                                % [kg*m^5] full-scale UAV x-inertia
uavParams.geom.Iyy  =   sigma*n^5*(0.089*3);                                % [kg*m^5] full-scale UAV y-inertia
uavParams.geom.Izz  =   sigma*n^5*(0.125*3);                                % [kg*m^5] full-scale UAV z-inertia
uavParams.geom.Ixz  =   0.0;                                                % [kg*m^5] full-scale UAV xz-product inertia

%.. motor
uavParams.motor.RPMMAX      =   10000;                                      % [rpm] maximum rpm of motors
uavParams.motor.tilt_trim   =   0;                                          % [rad] motor tilt trim angle 
uavParams.motor.minPWM      =   0.1;                                        % [-] motor minimum pwm level
uavParams.motor.maxPWM      =   1.0;                                        % [-] motor maximum pwm level
uavParams.motor.tfnum       =   [1];
uavParams.motor.tfden       =   [0.1 1];

%.. adjust the thrust coefficient
uavParams.rotor.Ct          =   4/(pi^3)*0.1142;                            % [-] thrust coefficient of rotor
uavParams.rotor.Cq          =   8/(pi^3)*0.007048;                          % [-] torque coefficient of rotor
uavParams.rotor.N           =   3;                                          % [-] number of blades
uavParams.geom.PropDiameter =   (2/3)*n*0.3052;                             % [m] full-scale UAV propeller diameter
uavParams.geom.PropChord    =   n*0.0080;                                   % [m] rotor blade chord
uavParams.geom.PropOffHinge =   0;                                          % [-] offhinge angle
uavParams.geom.PropCLa      =   5.5;                                        % [-] propeller CL-alpha slope
uavParams.geom.PropLock     =   0.6051;                                     % [-] lock number
uavParams.geom.Propblroot   =   14.5990*const.deg2rad;                      % [rad] blade root aoa
uavParams.geom.Propbltwist  =   -7.7980*const.deg2rad;                      % [rad] blade tip aoa

%.. servo/actuators
uavParams.tiltservo.tfnum       =   [1];
uavParams.tiltservo.tfden       =   [0.1 1];
uavParams.ctrlsurfservo.tfnum   =   [1];
uavParams.ctrlsurfservo.tfden   =   [1/(5*2*pi)^2 2*0.707/(5*2*pi) 1];
uavParams.maxAileron            =   30*const.deg2rad;
uavParams.minAileron            =   -30*const.deg2rad;
uavParams.maxElevator           =   30*const.deg2rad;
uavParams.minElevator           =   -30*const.deg2rad;
uavParams.maxRudder             =   30*const.deg2rad;
uavParams.minRudder             =   -30*const.deg2rad;
end
