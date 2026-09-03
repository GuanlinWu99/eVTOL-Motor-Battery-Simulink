%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Update UAV parameters for AAM Battery Project              %
% Author(s): Minhyun, Sounghwan, Guanlin                            %
% Description:                                                      %
% 1. This function includes 1) aerodynamic coefficients             %
%                           2) aircraft geometry                    %
%                           3) battery pack spec                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uavParams = load_vtol_dynamics_7000lb(const, Profile, eVTOL_MTOW, target_temperature)


%% Define Aerodynamics 
% ... Load custom aerodynamic coefficients from openVSPaero
[alpha_lon_temp, flap_lon_temp, CL_temp, CD_temp, CM_temp] = eVTOL_data_lon();
[alpha_latdir_temp, beta_latdir_temp, CS_temp, CR_temp, CN_temp] = eVTOL_data_latdir();  % [-] CR = Cl

uavParams.aero.alpha_lon        =   alpha_lon_temp*const.deg2rad;
uavParams.aero.flap_lon         =   flap_lon_temp*const.deg2rad;
uavParams.aero.alpha_latdir     =   alpha_latdir_temp*const.deg2rad;
uavParams.aero.beta_latdir      =   beta_latdir_temp*const.deg2rad;
uavParams.aero.air_density      =   1.225;                                  % [kg/m^3] Air density

%... Lift coefficients
% uavParams.aero.CL0            =   0.81857;                                % [-] CL @ AOA = 0
% uavParams.aero.CLa            =   4.09127;                                % [-] CL-alpha slope
uavParams.aero.CL               =   CL_temp;                                % [-] CL
uavParams.aero.CLadot           =   0;                                      % [-] CL-dot(alpha) slope
uavParams.aero.CLq              =   7.31097;                                % [-] CL-q slope
uavParams.aero.CLDe             =   0.50787;                                % [-] CL-elevator slope
uavParams.aero.CLDa             =   -0.85291;                               % [-] CL-aileron slope
uavParams.aero.CLadot_tfnum     =   [1 0];                                  % [-] Transfer function (numerator)
uavParams.aero.CLadot_tfden     =   [0.1 1];                                % [-] Transfer function (denominator)
uavParams.aero.CLDs             =   -mean(uavParams.aero.CL(:,5) ...
                                    -uavParams.aero.CL(:,1))/ ...
                                    (40*const.deg2rad)/2;                   % [-] CL-spoiler slope

%... Drag coefficients
% uavParams.aero.CDmin           =   0.06047;                               % [-] minimum drag 
% uavParams.aero.K               =   0.1328;                                % [-] drag polar quadratic coefficient
% uavParams.aero.CL_CDmin        =   0.4806;                                % [-] CL at minimum CD point
% uavParams.aero.A1              =   0;                                     % [-] scale factor for drag model
uavParams.aero.CD                =   CD_temp;                               % [-] CD
uavParams.aero.CdDe              =   0.016043;                              % [-] CD-elevator slope
uavParams.aero.CdDa              =   0.092132;                              % [-] CD-aileron slope
uavParams.aero.CdDr              =   0.0097403;                             % [-] CD-rudder slope
uavParams.aero.CdDs              =   mean(uavParams.aero.CD(:,5) ...
                                     -uavParams.aero.CD(:,1))/ ...
                                     (40*const.deg2rad);                    % [-] CD-spoiler slope
% uavParams.aero.dragCoeffMov    =   0.027;                                 % [-] fuselage moving part drag coefficient

%... Moment coefficient
% uavParams.aero.Cm0             =   0.00763;                               % [-] CM @ AOA = 0
% uavParams.aero.Cma             =   -1.76966;                              % [-] CM-alpha slope
uavParams.aero.CM                =   CM_temp;                               % CM
uavParams.aero.Cmq               =   -19.22663;                             % [-] pitch damping
uavParams.aero.CmDe              =   -1.83747*1.4;                          % [-] CM-elevator slope
% uavParams.aero.CmDr            =   0.02991;                               % [-] CM-rudder slope
uavParams.aero.CmDs              =   -mean(uavParams.aero.CM(:,5)... 
                                     -uavParams.aero.CM(:,1))/ ...
                                     (40*const.deg2rad);                    % [-] CM-spoiler slope

%... Side force
% uavParams.aero.CYb            =   -0.001570;                              % [-] CY-beta slope
uavParams.aero.CS               =   CS_temp;                                % [-] CS
uavParams.aero.CYp              =   -0.001570;                              % [-] CY-p slope
uavParams.aero.CYr              =   0.18368;                                % [-] CY-r slope
uavParams.aero.CYDr             =   0.128915;                               % [-] CY-rudder slope


%... Roll moment
% uavParams.aero.Clb            =   -0.001610;                              % [-] Cl-beta slope
uavParams.aero.CR               =   CR_temp;                                % CR
uavParams.aero.Clp              =   -0.47584;                               % [-] Cl-p slope
uavParams.aero.Clr              =   0.27153;                                % [-] Cl-r slope
uavParams.aero.ClDa             =   0.286593;                               % [-] Cl-aileron slope
uavParams.aero.ClDr             =   0.013407;                               % [-] Cl-rudder slope

%... Yaw moment
% uavParams.aero.Cnb            =   0.09180;                                % [-] Cn-beta slope
uavParams.aero.CN               =   CN_temp;                                % CN
uavParams.aero.Cnp              =   -0.13267;                               % [-] Cn-p slope
uavParams.aero.Cnr              =   -0.08829;                               % [-] Cn-r slope
uavParams.aero.CnDa             =   0.0049274;                              % [-] Cn-aileron slope
uavParams.aero.CnDr             =   -0.067036*3;                            % [-] Cn-rudder slope

% --- Scale up the parameters ---
MTOW                            =   eVTOL_MTOW*0.453592;                    % [kg]
n                               =   14/2;                                   % [-] length scale parameter
sigma                           =   (MTOW/6.025)/n^3;                       % [-] density scale parameter

%.. eVTOL Geometry
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
uavParams.motor.RPMMAX          =   1500;                                   % [rpm] max motor rpm
% uavParams.motor.tilt_trim       =   0;                                      % [rad] bias added to the tilt command
uavParams.motor.minPWM          =   0.1;                                    % [-] throttle floor
uavParams.motor.maxPWM          =   1.0;                                    % [-] throttle ceiling; 1.0 commands RPMMAX
uavParams.motor.tfnum           =   1;                                      % [-] motor lag 1/(0.1s+1); read only by the linearization models
uavParams.motor.tfden           =   [0.1 1];                                % [-] denominator; 0.1 s time constant

%.. adjust the thrust coefficient
uavParams.rotor.Ct              =   4/(pi^3)*0.1142;                        % [-] thrust coefficient of rotor
uavParams.rotor.Cq              =   8/(pi^3)*0.007048;                      % [-] torque coefficient of rotor
uavParams.rotor.N               =   3;                                      % [-] number of blades
uavParams.geom.PropDiameter     =   3.9;                                    % [m] full-scale UAV propeller diameter
uavParams.geom.PropChord        =   0.1022;                                 % [m] rotor blade chord
uavParams.geom.PropOffHinge     =   0;                                      % [-] offhinge angle
uavParams.geom.PropCLa          =   5.5;                                    % [-] propeller CL-alpha slope
uavParams.geom.PropLock         =   0.6051;                                 % [-] lock number
uavParams.geom.Propblroot       =   14.5990*const.deg2rad;                  % [rad] blade root aoa
uavParams.geom.Propbltwist      =   -7.7980*const.deg2rad;                  % [rad] blade tip aoa
uavParams.geom.RotorArea        =   pi*(uavParams.geom.PropDiameter/2)^2;   % [m^2] rotor blade area

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
uavParams.maxFlap               =   40*const.deg2rad;
uavParams.minFlap               =   0*const.deg2rad;
uavParams.maxSpoiler            =   40*const.deg2rad;
uavParams.minSpoiler            =   0*const.deg2rad;

%% Battery & Motor parameters
% ...Battery Parameters
Q_flat                                       = 4500*(eVTOL_MTOW/6020);  % [Nm] flat torque limit

Ns                                           = 200;                         % [-] cells in series; sets pack voltage
Np                                           = 21;                          % [-] cells in parallel; 200S21P for every MTOW
Norminal_Capacity                            = 10.5;                        % [Ah] per cell
Norminal_Voltage                             = 3.45;                        % [V] per cell
Cell_Mass                                    = 0.0965;   % [kg] per cell

%  Each 1-RC parameter's activation energy was obtained by fitting ln(parameter) versus 1/T (Arrhenius form) across the SA-88 sheets at
%  25/30/35/40 °C (3C), over the SOC ≥ 30% range to avoid low-SOC data noise. Slope = Ea/R_g gives Ea_R0 = 1.23, Ea_R1 = 3.34, Ea_C1 = 3.44
%  kJ/mol. （2026/06/14 Guanlin Wu）

here                                         = fileparts(mfilename('fullpath'));   % sheets sit here, not in the cwd
switch target_temperature
    case 20
        load(fullfile(here,"SA88_25_Degree.mat"));
        T_ref_K                              = 25 + 273.15;
    case 25
        load(fullfile(here,"SA88_25_Degree.mat"));
        T_ref_K                              = 25 + 273.15;
    case 45
        load(fullfile(here,"SA88_40_Degree.mat"));
        T_ref_K                              = 40 + 273.15;
    otherwise
        error('Unsupported target_temperature = %g degC (expected 20, 25, or 45).', target_temperature);
end

Ea_R0                                        = 1.23;                                % [kJ/mol] from R0(T) fit
Ea_R1                                        = 3.34;                                % [kJ/mol] from R1(T) fit
Ea_C1                                        = 3.44;                                % [kJ/mol] from C1(T) fit
R_gas                                        = 8.314462618;                         % [J/(mol*K)] gas constant
T_target_K                                   = target_temperature + 273.15;         % [K]
delta                                        = 1/T_target_K - 1/T_ref_K;            % [1/K]
k_R0                                         = exp( (1000*Ea_R0/R_gas) * delta );   % [-] 1000 converts kJ/mol -> J/mol
k_R1                                         = exp( (1000*Ea_R1/R_gas) * delta );   % [-]
k_C1                                         = exp(-(1000*Ea_C1/R_gas) * delta );   % [-] sign flipped: C1 rises with T

soc                                          = SOC_bp / 100;                        % [%] -> 0...1.0 fraction
Voltage                                      = OCV_table';   
R0                                           = k_R0 * Rs_table;
R1                                           = k_R1 * R1_table;
C1                                           = k_C1 * C1_table;
Tau1                                         = R1 .* C1;            

uavParams.Battery_Cell.SOC                   = soc';                        % [-] 0..1; breakpoints for the tables below
uavParams.Battery_Cell.Emo                   = Voltage*Ns;                  % [V] pack open-circuit voltage
uavParams.Battery_Cell.R0                    = (Ns/Np)*R0;                  % [Ohm] ohmic resistance
uavParams.Battery_Cell.R1                    = (Ns/Np)*R1;                  % [Ohm] polarisation resistance of the single RC branch
uavParams.Battery_Cell.C1                    = (Np/Ns)*C1;                  % [F] polarisation capacitance
uavParams.Battery_Cell.Tau1                  = Tau1;                        % [s] R1*C1; relaxation time of the RC branch
uavParams.Capacity                           = Norminal_Capacity;           % [Ah] per cell; the block scales by Np internally
uavParams.Temperature                        = target_temperature;          % [degC] fixed input, not a state; no self-heating is modelled
uavParams.Ns                                 = Ns;
uavParams.Np                                 = Np;

uavParams.Battery_Cell.Mass                  = Cell_Mass;  % [kg] per cell
uavParams.Battery_Pack_Voltage               = Ns*Norminal_Voltage;
uavParams.Battery_Pack_Capacity              = Np*Norminal_Capacity;
uavParams.Battery_Pack_Mass                  = Ns*Np*Cell_Mass;  % [kg] cells only
uavParams.Battery_Cell.init_SOC              = 0.95;                        % [-] 0..1; 0.95 is the top of the SOC breakpoint table

% ...eVTOL Motor Parameters
uavParams.MotorElec.TorqSpdLUT.SpeedRPM = [0    300   600   900   1200  1380  1500];  % [RPM] breakpoints, 0 to RPMMAX
uavParams.MotorElec.TorqSpdLUT.TorqueNm = Q_flat*[1 1 1 1 1 0.978 0.900];       % [Nm] constant torque then constant power
uavParams.MotorElec.Damping                      = 0.001;                   % [Nm/(rad/s)] kept small, motor treated as an ideal torque source
uavParams.MotorElec.TorqueControl_TimeConst      = 0.00267;                 % [s] lag of the drive's inner torque loop  
uavParams.MotorElec.Shaft_Inertia                = 0.009;                   % [kg*m^2] motor rotor inertia, inside the Servomotor block
uavParams.MotorElec.Prop_Inertia                 = 0.0837*sqrt(eVTOL_MTOW/6020);  % [kg*m^2] propeller inertia on the shaft
uavParams.MotorElec.Series_Resistance            = 0.001;                   % [Ohm] winding resistance seen from the DC bus
uavParams.MotorElec.w_eff                        = 900;                     % [RPM] speed at which Efficiency is measured
uavParams.MotorElec.q_eff                        = 2769;                    % [Nm] torque at which Efficiency is measured
uavParams.MotorElec.Efficiency                   = 95;                      % [%] motor + drive efficiency at (w_eff, q_eff)

% ...DC-DC Converter Parameters
uavParams.DCDCConv.Output_Voltage            = 1000;                        % [V] regulated bus that feeds the four drives
uavParams.DCDCConv.Resistance_Losses         = 0.0188;                     % [Ohm] conduction loss (GAIA converter, 4%)
uavParams.DCDCConv.Kp                        = 0.01;                        % [-] output voltage proportional controller
uavParams.DCDCConv.Ki                        = 10;                          % [-] output voltage integral controller
uavParams.DCDCConv.MinVin                    = 50;                          % [V] converter stops below this input voltage

% ...PMSM Drive Parameters
% Evolito D1500 x2 per rotor; Rs, Ld, Lq, psim, p are estimates, only ratings published.
Rs        = 0.010;                      % [Ohm]     stator resistance / phase
Ld        = 200e-6;                     % [H]       d-axis inductance
Lq        = 200e-6;                     % [H]       q-axis, non-salient
psim      = 0.16;                       % [Wb]      PM flux linkage
p         = 15;                         % [-]       pole pairs
Kt        = 1.5*p*psim;                 % [Nm/A]    torque constant, 3.6
Bm        = 1e-3;                       % [N*m*s]   viscous damping
Imax      = 800;                        % [A]       Kt*Imax = 2880 Nm peak
Jm_flight = 0.09;                       % [kg*m^2]  shaft 0.009 + prop 0.08
RPMMAX    = 1500;                       % [rpm]     normalises Rotor Assembly.N
bw_i      = 2*pi*1000;                  % [rad/s]   current-loop bandwidth
Kp_i      = Ld*bw_i;
Ki_i      = Rs*bw_i;
tau_dc    = 1e-4;                       % [s]       DC-link filter time constant
SOC0      = 0.90;                       % [-]       initial pack SOC

% 1-RC pack tables off the corrected cell data; breakpoints must rise, SOC_bp does not
[soc_bp, iSort] = sort(soc(:));
OCV_pack  = Ns*col(Voltage, iSort);     % [V]
R0_pack   = (Ns/Np)*col(R0, iSort);     % [Ohm]
R1_pack   = (Ns/Np)*col(R1, iSort);     % [Ohm]
C1_pack   = (Np/Ns)*col(C1, iSort);     % [F]
Cap_pack  = Np*Norminal_Capacity;       % [Ah]

for v = {'Rs','Ld','Lq','psim','p','Kt','Bm','Imax','Jm_flight','RPMMAX','bw_i', ...
         'Kp_i','Ki_i','tau_dc','SOC0','soc_bp','OCV_pack','R0_pack','R1_pack', ...
         'C1_pack','Cap_pack'}
    assignin('base', v{1}, eval(v{1}));
end
end

function y = col(x, iSort)
y = reshape(x, [], 1);  y = y(iSort);
end
