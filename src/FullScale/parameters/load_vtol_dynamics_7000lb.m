%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project                   %
% Author(s): Minhyun and Sounghwan                                       %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [uavParams, HEV_Param] = load_vtol_dynamics_7000lb(const)

%% [1] Define Aerodynamics
% ... Load custom aerodynamic coefficients from openVSPaero
[alpha_lon_temp, CL_temp, CD_temp, CM_temp]                          =   eVTOL_data_lon();
[alpha_latdir_temp, beta_latdir_temp, CS_temp, CR_temp, CN_temp]     =   eVTOL_data_latdir();        % [-] CR = Cl
[alpha_l_20, CL_flap_20, CD_flap_20, CM_flap_20]                     =   eVTOL_data_lon_flap_20();
[alpha_l_30, CL_flap_30, CD_flap_30, CM_flap_30]                     =   eVTOL_data_lon_flap_30();

%... Flap (20 degree) data
uavParams.aero.CL_flap_20       =   CL_flap_20;                             % [-] CL with flap 20 degree
uavParams.aero.CD_flap_20       =   CD_flap_20;                             % [-] CL with flap 20 degree
uavParams.aero.CM_flap_20       =   CM_flap_20;                             % [-] CL with flap 20 degree
uavParams.aero.alpha_lon_20     =   alpha_l_20*const.deg2rad;

%... Flap (30 degree) data
uavParams.aero.CL_flap_30       =   CL_flap_30;                             % [-] CL with flap 30 degree
uavParams.aero.CD_flap_30       =   CD_flap_30;                             % [-] CL with flap 30 degree
uavParams.aero.CM_flap_30       =   CM_flap_30;                             % [-] CL with flap 30 degree
uavParams.aero.alpha_lon_30     =   alpha_l_30*const.deg2rad;

uavParams.aero.alpha_lon        =   alpha_lon_temp*const.deg2rad;
uavParams.aero.alpha_latdir     =   alpha_latdir_temp*const.deg2rad;
uavParams.aero.beta_latdir      =   beta_latdir_temp*const.deg2rad;
uavParams.aero.air_density      =   1.225;                                  % [kg/m^3] Air density

%... Lift coefficients
% uavParams.aero.CL0      =   0.81857;                                      % [-] CL @ AOA = 0
% uavParams.aero.CLa      =   4.09127;                                      % [-] CL-alpha slope
uavParams.aero.CL               =   CL_temp;                                % [-] CL
uavParams.aero.CLadot           =   0;                                      % [-] CL-dot(alpha) slope
uavParams.aero.CLq              =   7.31097;                                % [-] CL-q slope
uavParams.aero.CLDe             =   0.50787;                                % [-] CL-elevator slope
uavParams.aero.CLDa             =   -0.85291;                               % [-] CL-aileron slope
uavParams.aero.CLadot_tfnum     =   [1 0];                                  % [-] Transfer function (numerator)
uavParams.aero.CLadot_tfden     =   [0.1 1];                                % [-] Transfer function (denominator)

%... Drag coefficients
% uavParams.aero.CDmin    =   0.06047;                                      % [-] minimum drag 
% uavParams.aero.K        =   0.1328;                                       % [-] drag polar quadratic coefficient
% uavParams.aero.CL_CDmin =   0.4806;                                       % [-] CL at minimum CD point
% uavParams.aero.A1       =   0;                                            % [-] scale factor for drag model
uavParams.aero.CD                =   CD_temp;                               % [-] CD
uavParams.aero.CdDe              =   0.016043;                              % [-] CD-elevator slope
uavParams.aero.CdDa              =   0.092132;                              % [-] CD-aileron slope
uavParams.aero.CdDr              =   0.0097403;                             % [-] CD-rudder slope
% uavParams.aero.dragCoeffMov =   0.027;                                    % [-] fuselage moving part drag coefficient

%... Moment coefficient
% uavParams.aero.Cm0      =   0.00763;                                      % [-] CM @ AOA = 0
% uavParams.aero.Cma      =   -1.76966;                                     % [-] CM-alpha slope
uavParams.aero.CM                =   CM_temp;                               % CM
uavParams.aero.Cmq               =   -19.22663;                             % [-] pitch damping
uavParams.aero.CmDe              =   -1.83747*1.4;                          % [-] CM-elevator slope
% uavParams.aero.CmDr     =   0.02991;                                      % [-] CM-rudder slope

%... Side force
% uavParams.aero.CYb      =   -0.001570;                                    % [-] CY-beta slope
uavParams.aero.CS               =   CS_temp;                                % [-] CS
uavParams.aero.CYp              =   -0.001570;                              % [-] CY-p slope
uavParams.aero.CYr              =   0.18368;                                % [-] CY-r slope
uavParams.aero.CYDr             =   0.128915;                               % [-] CY-rudder slope

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.4 0.25], 'Color','w'); 
% subplot(1,3,1)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CL,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CL')
% 
% subplot(1,3,2)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CD,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CD')
% 
% subplot(1,3,3)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CM,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CM')
% 
% sgtitle('Aerodynamic Coefficients','FontSize',15,'FontWeight','bold');

%... Roll moment
% uavParams.aero.Clb      =   -0.001610;                                    % [-] Cl-beta slope
uavParams.aero.CR               =   CR_temp;                                % CR
uavParams.aero.Clp              =   -0.47584;                               % [-] Cl-p slope
uavParams.aero.Clr              =   0.27153;                                % [-] Cl-r slope
uavParams.aero.ClDa             =   0.286593;                               % [-] Cl-aileron slope
uavParams.aero.ClDr             =   0.013407;                               % [-] Cl-rudder slope

%... Yaw moment
% uavParams.aero.Cnb      =   0.09180;                                      % [-] Cn-beta slope
uavParams.aero.CN               =   CN_temp;                                % CN
uavParams.aero.Cnp              =   -0.13267;                               % [-] Cn-p slope
uavParams.aero.Cnr              =   -0.08829;                               % [-] Cn-r slope
uavParams.aero.CnDa             =   0.0049274;                              % [-] Cn-aileron slope
uavParams.aero.CnDr             =   -0.067036*3;                            % [-] Cn-rudder slope

% --- Scale up the parameters ---
n                               =   14/2;                                   % [-] lenth scale parameter
sigma                           =   (3175/6.025)/n^3;                       % [-] density scale parameter

%.. geometry
uavParams.geom.b                =   n*2;                                    % [m] full-scale UAV span
uavParams.geom.AR               =   7.2727;                                 % [-] AR ratio of wing
uavParams.geom.c                =   n*0.275;                                % [m] full-scale UAV chord
uavParams.geom.e                =   0.986;                                  % [-] span efficiency factor (suppose inviscid flow)
uavParams.geom.RotorArm1        =   n*[ 0.375  0.375 0];                    % [m] full-scale propeller #1 location
uavParams.geom.RotorArm2        =   n*[ 0.375 -0.375 0];                    % [m] full-scale propeller #2 location
uavParams.geom.RotorArm3        =   n*[-0.375 -0.375 0];                    % [m] full-scale propeller #3 location
uavParams.geom.RotorArm4        =   n*[-0.375  0.375 0];                    % [m] full-scale propeller #4 location

%.. mass and inertia
uavParams.geom.mass             =   sigma*n^3*6.023;                        % [kg] full-scale UAV mass
uavParams.geom.Ixx              =   sigma*n^5*(0.089*3);                    % [kg*m^5] full-scale UAV x-inertia
uavParams.geom.Iyy              =   sigma*n^5*(0.089*3);                    % [kg*m^5] full-scale UAV y-inertia
uavParams.geom.Izz              =   sigma*n^5*(0.125*3);                    % [kg*m^5] full-scale UAV z-inertia
uavParams.geom.Ixz              =   0.0;                                    % [kg*m^5] full-scale UAV xz-product inertia

%.. motor
uavParams.motor.RPMMAX          =   10000;                                  % [rpm] maximum rpm of motors
uavParams.motor.tilt_trim       =   0;                                      % [rad] motor tilt trim angle 
uavParams.motor.minPWM          =   0.1;                                    % [-] motor minimum pwm level
uavParams.motor.maxPWM          =   1.0;                                    % [-] motor maximum pwm level
uavParams.motor.tfnum           =   1;
uavParams.motor.tfden           =   [0.1 1];

%.. adjust the thrust coefficient
uavParams.rotor.Ct              =   4/(pi^3)*0.1142;                        % [-] thrust coefficient of rotor
uavParams.rotor.Cq              =   8/(pi^3)*0.007048;                      % [-] torque coefficient of rotor
uavParams.rotor.N               =   3;                                      % [-] number of blades
uavParams.geom.PropDiameter     =   (2/3)*n*0.3052;                         % [m] full-scale UAV propeller diameter
uavParams.geom.PropChord        =   n*0.0080;                               % [m] rotor blade chord
uavParams.geom.PropOffHinge     =   0;                                      % [-] offhinge angle
uavParams.geom.PropCLa          =   5.5;                                    % [-] propeller CL-alpha slope
uavParams.geom.PropLock         =   0.6051;                                 % [-] lock number
uavParams.geom.Propblroot       =   14.5990*const.deg2rad;                  % [rad] blade root aoa
uavParams.geom.Propbltwist      =   -7.7980*const.deg2rad;                  % [rad] blade tip aoa
uavParams.geom.RotorArea        =   pi*(uavParams.geom.PropDiameter/2)^2;   % [m^2] rotor blade area

% uavParams.rotor.Ct = 4/(pi^3)*0.1142;
% uavParams.rotor.N = 4;

%.. servo/actuators
uavParams.tiltservo.tfnum       =   1;
uavParams.tiltservo.tfden       =   [0.1 1.0];
uavParams.ctrlsurfservo.tfnum   =   1;
uavParams.ctrlsurfservo.tfden   =   [1/(5*2*pi)^2 2*0.707/(5*2*pi) 1.0];
uavParams.maxAileron            =   30*const.deg2rad;
uavParams.minAileron            =   -30*const.deg2rad;
uavParams.maxElevator           =   30*const.deg2rad;
uavParams.minElevator           =   -30*const.deg2rad;
uavParams.maxRudder             =   30*const.deg2rad;
uavParams.minRudder             =   -30*const.deg2rad;

%% [2] Battery model parameters

% [2-1] PREDEFINED LI-ION BATTERY PARAMS
HEV_Param.Battery_Det.Nominal_Voltage      = 200;                    % [v]
HEV_Param.Battery_Det.Rated_Capacity       = 8.1;                    % [Ah]
HEV_Param.Battery_Det.Initial_SOC          = 97;                     % [%]
HEV_Param.Battery_Det.Series_Resistance    = 0.2/10;                 % [Ohm]

% [2-2] GENERIC BATTERY PARAMS
HEV_Param.Battery_Sys.Nominal_Voltage      = 217;                    % [v]
HEV_Param.Battery_Sys.Internal_Resistance  = 0.24691;                % [Ohm]
HEV_Param.Battery_Sys.Rated_Capacity       = 250;                    % [Ah]
HEV_Param.Battery_Sys.Initial_Charge       = 250;                    % [Ah]
HEV_Param.Battery_Sys.Expn_Voltage         = 215.0342;               % V
HEV_Param.Battery_Sys.Expn_Charge          = 2.3438;
HEV_Param.Battery_Sys.C1.Capacitance       = 2500;
HEV_Param.Battery_Sys.C1.Initial_Voltage   = 19;                     % [v]
HEV_Param.Battery_Sys.C1.Series_Resistance = 1e-6;
HEV_Param.Battery_Sys.R2                   = 0.3;                    % [Ohm]
HEV_Param.Battery_Sys.R1                   = 1.8;                    % [Ohm]
HEV_Param.Battery_Sys.Maximum_Capacity     = HEV_Param.Battery_Sys.Rated_Capacity;    % [Ah]

% [2-3] Battery Model Parameters 
HEV_Param.Battery_Cell.Rated_Capacity     = 5;                       % [Ah]
HEV_Param.Battery_Cell.SOC_init           = 0.9;                     
HEV_Param.Battery_Cell.theta_init         = 25;
HEV_Param.Battery_Cell.Ctheta             = 200;                     % (J/°C) Thermal Capacitance
HEV_Param.Battery_Cell.Area               = 0.01;                    % (m^2) Surface area of battery exposed to air 
HEV_Param.Battery_Cell.Rtheta             = 20;                      % (W/m^2/K) Convective heat transfer coefficient 
HEV_Param.Battery_Cell.Kc                 = 1.2;                     % [-]
HEV_Param.Battery_Cell.Costar             = 1.8e+005;                % (As)
HEV_Param.Battery_Cell.Kt_Temps           = [25 40 60 75];           % Temperature breakpoints for Kt LUT
HEV_Param.Battery_Cell.Kt                 = [0.80,1.10,1.20,1.12;];  % () LUT output values
HEV_Param.Battery_Cell.delta              = 0.73;                    % ()
HEV_Param.Battery_Cell.Istar              = 20;                      % (A) Nominal Current (=cap/disch_t)
HEV_Param.Battery_Cell.theta_f            = -40;                     % (°C) Electrolyte Freezing Temp

%% *Editted by Sounghwan (Battery cell to pack scaling)*
Ns                                        = 200;                      % number of series
Np                                        = 200;                      % number of parallel

% Samsung INR18650-300 cell 
% Scaling law from Prof. Jung's group
R0                                        = 0.012;
R1                                        = 0.004;
R2                                        = 0.0015;
C1                                        = 136.29;
C2                                        = 872.87;
Tau1                                      = R1*C1;
Capacity                                  = 3;                       % [Ah] cell rated capacity
Voltage                                   = 4;                       % [V] cell nominal voltage

HEV_Param.Battery_Cell.Emo                = Voltage*Ns;              % [800V]
HEV_Param.Battery_Cell.R0                 = (Ns/Np)*R0;              % [Ohm]
HEV_Param.Battery_Cell.R1                 = (Ns/Np)*R1;              % [Ohm]
HEV_Param.Battery_Cell.R2                 = (Ns/Np)*R2;              % [Ohm]
HEV_Param.Battery_Cell.C2                 = (Np/Ns)*C2;              % [F]
HEV_Param.Battery_Cell.Tau1               = Tau1;                    % [s]
HEV_Param.Ns                              = Ns;
HEV_Param.Np                              = Np;
HEV_Param.Capacity                        = Capacity;

% HEV_Param.Battery_Cell.Emo                = 4.8*10;    % [V] [max o.c. volts per cell] (open circuit voltage at full charge)
% HEV_Param.Battery_Cell.R0                 = 0.001;     % [Ohm]
% HEV_Param.Battery_Cell.R1                 = 0.01;      % [Ohm]
% HEV_Param.Battery_Cell.R2                 = 0.05;      % [Ohm]
% HEV_Param.Battery_Cell.C2                 = 200;       % [F]
% HEV_Param.Battery_Cell.Tau1               = 1;         % [s]

% Compute initial extracted charge
HEV_Param.Battery_Cell.Qe_init = (1-HEV_Param.Battery_Cell.SOC_init)*HEV_Param.Battery_Cell.Kc*HEV_Param.Battery_Cell.Costar*interp1([HEV_Param.Battery_Cell.theta_f HEV_Param.Battery_Cell.Kt_Temps],[0 HEV_Param.Battery_Cell.Kt],HEV_Param.Battery_Cell.theta_init,'spline');

% [3] UltraCapacitor Parameters
HEV_Param.UltraCapacitor.Nominal_Capacitance = 1000;        % Farad
HEV_Param.UltraCapacitor.Rate_C_V            = 0.2;         % Farad/Volt
HEV_Param.UltraCapacitor.Series_R            = 30/3;        % [Ohm]
HEV_Param.UltraCapacitor.Self_Discharge_R    = 500;         % [Ohm]
HEV_Param.UltraCapacitor.Initial_Voltage     = 217;         % [V]

% [4] Motor Parameters
HEV_Param.Motor.Stator_Resistance            = 0.0910;        
HEV_Param.Motor.TorqSpdLUT.SpeedRPM          = [0    1404  2800  4200  5600  7000  8000  8500  9000    10000];
HEV_Param.Motor.TorqSpdLUT.TorqueNm          = [800   800   800   800   800   800   700   650   500        0];
HEV_Param.Motor.Damping                      = 0.001;       % N*m/(rad/s)
HEV_Param.Motor.TorqueControl_TimeConst      = 0.00267;     % [-]
HEV_Param.Motor.Shaft_Inertia                = 0.009;       % [kg*m^2]
HEV_Param.Motor.Series_Resistance            = 0.001;       % [Ohm]
HEV_Param.Motor.Inductances                  = [0.001597972349731   0.002057052250467];
HEV_Param.Motor.Efficiency                   = 95;          % motor efficiency

%Original value
%HEV_Param.Motor.TorqueControl_TimeConst      = 0.0267;
%HEV_Param.Motor.Shaft_Inertia                = 0.001;       % [kg*m^2]
%HEV_Param.DCDCConv.Resistance_Losses         = 0.6250;      % [Ohm]
%HEV_Param.Motor.TorqSpdLUT.SpeedRPM          = [0 1000 2000 3000 4000 5000 6000 7000 8000 9000 10000];
%HEV_Param.Motor.TorqSpdLUT.TorqueNm          = [40  40   40   40   40   40   35   30   20   15 0.01];

% Note 
% [*] Continuous Operation Maximum Torque Envelope Tc (N*m) : 
% maximum torque that motor can continuously generate
% [*] Corresponding Rotational Speed (RPM)
% corresponding RPM to the maximum torque

% [5] DC-DC Converter Parameters
HEV_Param.DCDCConv.Output_Voltage            = 500;         % [V]
HEV_Param.DCDCConv.Resistance_Losses         = 0.00625;     % [Ohm]
HEV_Param.DCDCConv.Kp                        = 0.01;
HEV_Param.DCDCConv.Ki                        = 10;
HEV_Param.DCDCConv.MinVin                    = 50;
HEV_Param.DCDCConv.Mean_Boost.Kp             = 0.001;
HEV_Param.DCDCConv.Mean_Boost.Ki             = 1;
HEV_Param.DCDCConv.EPower2Heat               = 0.1;         % Watts/Watts
HEV_Param.DCDCConv.Thermal_Mass              = 0.1*10;      % [kg]
HEV_Param.DCDCConv.Specific_Heat             = 100;         % J/kg/K
HEV_Param.DCDCConv.Initial_Temperature       = 25;          % [C]
HEV_Param.DCDCConv.Air_Temperature           = 298;         % [K]
HEV_Param.DCDCConv.Convection.Area           = 20;          % [cm^2]
HEV_Param.DCDCConv.Convection.HT_Coefficient = 100;         % W/(m^2*K)

% [6] Controller Parameters
HEV_Param.Control.Engine_Start_RPM           = 800;         % RPM
HEV_Param.Control.Engine_Stop_RPM            = 790;         % RPM
HEV_Param.Control.Mode_Logic_TS              = 0.1;
HEV_Param.Control.ICE.Kp                     = 0.02;
HEV_Param.Control.ICE.Ki                     = 0.01;
HEV_Param.Control.Gen.Kp                     = 10;
HEV_Param.Control.Gen.Ki                     = 3;
HEV_Param.Control.Mot.Kp                     = 500;
HEV_Param.Control.Mot.Ki                     = 300;
HEV_Param.Control.Veh_Spd.Kp                 = 0.02;
HEV_Param.Control.Veh_Spd.Ki                 = 0.04;

end
