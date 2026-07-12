% % Copyright 2011-2024 The MathWorks, Inc.
% clc;
% clear;

% Battery Model Parameters
% Ts = 2e-6; 
% set_param('Test_FourMotor_PID_Ave','FixedStep','Ts');
ts_vec = 50e-6;
target_temperature = 25;

%% ===== Cell Spec (LG Chem INR 18650 / Samsung SA-88, 18650-class) =======
Ns                                           = 200;                         % [-] number of series
Np                                           = 200;                         % [-] number of parallel
Norminal_Capacity                            = 3.5;                         % [Ah] norminal capacity
Norminal_Voltage                             = 3.6;                         % [V] norminal voltage

switch target_temperature
    case 20
        load("src/FullScale/parameters/SA88_25_Degree.mat");
        T_ref_K                              = 25 + 273.15;
    case 25
        load("src/FullScale/parameters/SA88_25_Degree.mat");
        T_ref_K                              = 25 + 273.15;
    case 45
        load("src/FullScale/parameters/SA88_40_Degree.mat");
        T_ref_K                              = 40 + 273.15;
    otherwise
        error('Unsupported target_temperature = %g degC (expected 20, 25, or 45).', ...
              target_temperature);
end

 %  Each 1-RC parameter's activation energy was obtained by fitting ln(parameter) versus 1/T (Arrhenius form) across the SA-88 sheets at
 %  25/30/35/40 °C (3C), over the SOC ≥ 30% range to avoid low-SOC data noise. Slope = Ea/R_g gives Ea_R0 = 1.23, Ea_R1 = 3.34, Ea_C1 = 3.44
 %  kJ/mol. （2026/06/14 Guanlin Wu）
Ea_R0                                        = 1.23;             % [kJ/mol] from R0(T) fit
Ea_R1                                        = 3.34;             % [kJ/mol] from R1(T) fit
Ea_C1                                        = 3.44;             % [kJ/mol] from C1(T) fit
Ea_OCV                                       = 0.73;             % [kJ/mol] from measured OCV(T) fit; OCV rises with T -> C-style sign
R_gas                                        = 8.314462618;
T_target_K                                   = target_temperature + 273.15;
delta                                        = 1/T_target_K - 1/T_ref_K;
k_R0                                         = exp( (1000*Ea_R0/R_gas) * delta );
k_R1                                         = exp( (1000*Ea_R1/R_gas) * delta );
k_C1                                         = exp(-(1000*Ea_C1/R_gas) * delta );
k_OCV                                        = exp(-(1000*Ea_OCV/R_gas) * delta );  % OCV ref = R/C ref: 20C from 25 (-0.5%), 45C from 40 (+0.44%)

soc                                          = SOC_bp / 100;   % 0..95 [%] -> 0..0.95 fraction (Simscape battery block requires SOC <= 1)
Voltage                                      = k_OCV * OCV_table';   % measured SA-88 OCV (25C/40C), Arrhenius-scaled to target T
R0                                           = k_R0 * Rs_table;
R1                                           = k_R1 * R1_table;
C1                                           = k_C1 * C1_table;
Tau1                                         = R1 .* C1;            % recomputed; now T-dependent (Ea_R1 != Ea_C1)
T_target                                     = target_temperature + 273.15;


% Initial State of Charge (% of full charge)
% Initial Electrolyte Temperature (°C, typically same as ambient temp)
% 
HEV_Param.Battery_Sys.Internal_Resistance = 0.006; % Ohm
Q_Ah = 48;
HEV_Param.Battery_Cell.Rated_Capacity = Q_Ah;  % Ampere-hours
HEV_Param.Battery_Cell.SOC_init = 0.95;
% HEV_Param.Battery_Cell.SOC = 0.95;
HEV_Param.Battery_Cell.theta_init = 25;
HEV_Param.Battery_Pack_Capacity = 700; 
% Battery Block Thermal Parameters
HEV_Param.Battery_Cell.Ctheta = 200; %(J/°C) Thermal Capacitance
HEV_Param.Battery_Cell.Area = 0.01; % (m^2) Surface area of battery exposed to air
HEV_Param.Battery_Cell.Rtheta = 20;  %(W/m^2/K) Convective heat transfer coefficient
    
% Battery Block Capacity Parameters
% Charge/discharge cycles at ranges of current/temp
HEV_Param.Battery_Cell.Kc = 1.2; %()
% HEV_Param.Battery_Cell.Costar = 1.8e+005; %(As)
% HEV_Param.Battery_Cell.Kt_Temps = [25 40 60 75]; % Temperature breakpoints for Kt LUT
% HEV_Param.Battery_Cell.Kt = [0.80,1.10,1.20,1.12;]; %() LUT output values
% HEV_Param.Battery_Cell.delta = 0.73; %()
% HEV_Param.Battery_Cell.Istar = 20; %(A) Nominal Current (=cap/disch_t)
% HEV_Param.Battery_Cell.theta_f = -40; %(°C) Electrolyte Freezing Temp
    
% Battery Block Parasitic Branch Parameters
% End of charge cycle at ranges of current/temp
% HEV_Param.Battery_Cell.Ep = 1.95; %(V) Parasitic emf
% HEV_Param.Battery_Cell.Gpo = 2.0e-011; %(s)
% HEV_Param.Battery_Cell.Vpo = 0.12; %(V)
% HEV_Param.Battery_Cell.Ap = 2.0; %()
% HEV_Param.Battery_Cell.Taup = 3; % (s)
    
% Battery Block Main Branch Parameters
HEV_Param.Battery_Cell.Emo = Voltage*Ns; % (V) [max o.c. volts per cell]
HEV_Param.Ns                                 = Ns;
HEV_Param.Np                                 = Np;                                            
% HEV_Param.Battery_Cell.Ke = 0.0006; %(V/°C)
HEV_Param.Capacity = Q_Ah; %(Ohm)
% HEV_Param.Battery_Cell.Ao = -0.6; % ()
HEV_Param.Battery_Cell.SOC                   = soc;                         % [0-1]
HEV_Param.Battery_Cell.Emo                   = Voltage*Ns;                  % [V]
HEV_Param.Battery_Cell.R0                    = (Ns/Np)*R0;                  % [Ohm]
HEV_Param.Battery_Cell.R1                    = (Ns/Np)*R1;                  % [Ohm]
HEV_Param.Battery_Cell.C1                    = (Np/Ns)*C1;                  % [F]
HEV_Param.Battery_Cell.Tau1                  = Tau1;                        % [s]
% HEV_Param.Battery_Cell.R2 = 0.05; %(Ohm)
% HEV_Param.Battery_Cell.C1 = 10000; % (F)
% HEV_Param.Battery_Cell.C2 = 200; % (F)
    
% Compute initial extracted charge
% HEV_Param.Battery_Cell.Qe_init = (1-HEV_Param.Battery_Cell.SOC_init)*HEV_Param.Battery_Cell.Kc*HEV_Param.Battery_Cell.Costar*interp1([HEV_Param.Battery_Cell.theta_f HEV_Param.Battery_Cell.Kt_Temps],[0 HEV_Param.Battery_Cell.Kt],HEV_Param.Battery_Cell.theta_init,'spline');
HEV_Param.Battery_Cell.Costar = Q_Ah*3600/(HEV_Param.Battery_Cell.Kc*0.8);

%% MOTOR PARAMETERS
HEV_Param.Motor.Stator_Resistance = 29.4e-3;        

HEV_Param.Motor.TorqSpdLUT.SpeedRPM          = [0     1404   2800   4200   5600   7000   8000   8500   9000   10000]; %from ermax348
HEV_Param.Motor.TorqSpdLUT.TorqueNm          = [800    800    800    800    800    800    700    650    500       0]; %from ermax348

HEV_Param.Motor.Damping = 0.001; %N*m/(rad/s)
HEV_Param.Motor.TorqueControl_TimeConst = 0.00267;                                  
HEV_Param.Motor.Shaft_Inertia = 0.22042;                                    % from ermax348
% HEV_Param.Motor.Series_Resistance = 0.01;                                 % CHG
HEV_Param.Motor.Inductances = [425.2e-6   425.3e-6];                        % imitate surface pmsm, from ermax348
HEV_Param.Motor.Magnetic_flux = 0.06249;                                    % from ermax 348
HEV_Param.Motor.Efficiency = 96;                                            % from ermax 348

%% DC-DC CONVERTER PARAMETERS
HEV_Param.DCDCConv.Output_Voltage = 800;      % Volts
HEV_Param.DCDCConv.Resistance_Losses = 1000/40^2;      % Ohm
HEV_Param.DCDCConv.Kp = 0.01;
HEV_Param.DCDCConv.Ki = 10;
HEV_Param.DCDCConv.MinVin = 50;

HEV_Param.DCDCConv.Mean_Boost.Kp = 0.001;
HEV_Param.DCDCConv.Mean_Boost.Ki = 1;

HEV_Param.DCDCConv.EPower2Heat = 0.1;      % Watts/Watts
HEV_Param.DCDCConv.Thermal_Mass = 0.1*10;    % kg
HEV_Param.DCDCConv.Specific_Heat = 100;   % J/kg/K
HEV_Param.DCDCConv.Initial_Temperature = 25;     % C
HEV_Param.DCDCConv.Air_Temperature = 298;     % K
HEV_Param.DCDCConv.Convection.Area = 20;     % cm^2
HEV_Param.DCDCConv.Convection.HT_Coefficient = 100; % W/(m^2*K)


%% CONTROLLER PARAMETERS
HEV_Param.Control.Engine_Start_RPM = 800; % RPM
HEV_Param.Control.Engine_Stop_RPM = 790; % RPM
HEV_Param.Control.Mode_Logic_TS = 0.1;
HEV_Param.Control.ICE.Kp = 0.02;
HEV_Param.Control.ICE.Ki = 0.01;
HEV_Param.Control.Gen.Kp = 10;
HEV_Param.Control.Gen.Ki = 3;
HEV_Param.Control.Mot.Kp = 500;
HEV_Param.Control.Mot.Ki = 300;
HEV_Param.Control.Veh_Spd.Kp = 0.02;
HEV_Param.Control.Veh_Spd.Ki = 0.04;


