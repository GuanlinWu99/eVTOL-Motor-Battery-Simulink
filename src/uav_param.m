% % Copyright 2011-2024 The MathWorks, Inc.

% PREDEFINED LI-ION BATTERY PARAMS
HEV_Param.Battery_Det.Nominal_Voltage = 200;
HEV_Param.Battery_Det.Rated_Capacity = 8.1;      % Ampere-hours
HEV_Param.Battery_Det.Initial_SOC = 97;       % Percent
HEV_Param.Battery_Det.Series_Resistance = 0.2/10;   % Ohm


% FOR ACCEL TEST
%HEV_Param.Battery_Det.Initial_SOC = 97;         % Percent

% GENERIC BATTERY PARAMS
HEV_Param.Battery_Sys.Nominal_Voltage = 217;
HEV_Param.Battery_Sys.Internal_Resistance = 0.24691;% Ohm
HEV_Param.Battery_Sys.Rated_Capacity = 21;     % Ampere-hours
HEV_Param.Battery_Sys.Initial_Charge = 21;    % Ampere-hours
HEV_Param.Battery_Sys.Expn_Voltage = 215.0342;       % V
HEV_Param.Battery_Sys.Expn_Charge = 2.3438;

HEV_Param.Battery_Sys.C1.Capacitance = 2500;
HEV_Param.Battery_Sys.C1.Initial_Voltage = 19;
HEV_Param.Battery_Sys.C1.Series_Resistance = 1e-6;
HEV_Param.Battery_Sys.R2 = 0.3;
HEV_Param.Battery_Sys.R1 = 1.8;
HEV_Param.Battery_Sys.Maximum_Capacity = HEV_Param.Battery_Sys.Rated_Capacity;    % Ampere-hours


% Battery Model Parameters
% To compute soc, use battery_sys.maximum_capacity.
    
% Battery Block Initial Conditions
% Initial State of Charge (% of full charge)
% Initial Electrolyte Temperature (°C, typically same as ambient temp)
HEV_Param.Battery_Cell.Rated_Capacity = 5;  % Ampere-hours
HEV_Param.Battery_Cell.SOC_init = 0.9;
HEV_Param.Battery_Cell.theta_init = 25;
    
% Battery Block Thermal Parameters
HEV_Param.Battery_Cell.Ctheta = 200; %(J/°C) Thermal Capacitance
HEV_Param.Battery_Cell.Area = 0.01; % (m^2) Surface area of battery exposed to air
HEV_Param.Battery_Cell.Rtheta = 20;  %(W/m^2/K) Convective heat transfer coefficient
    
% Battery Block Capacity Parameters
% Charge/discharge cycles at ranges of current/temp
HEV_Param.Battery_Cell.Kc = 1.2; %()
HEV_Param.Battery_Cell.Costar = 1.8e+005; %(As)
HEV_Param.Battery_Cell.Kt_Temps = [25 40 60 75]; % Temperature breakpoints for Kt LUT
HEV_Param.Battery_Cell.Kt = [0.80,1.10,1.20,1.12;]; %() LUT output values
HEV_Param.Battery_Cell.delta = 0.73; %()
HEV_Param.Battery_Cell.Istar = 20; %(A) Nominal Current (=cap/disch_t)
HEV_Param.Battery_Cell.theta_f = -40; %(°C) Electrolyte Freezing Temp
    
% Battery Block Parasitic Branch Parameters
% End of charge cycle at ranges of current/temp
% HEV_Param.Battery_Cell.Ep = 1.95; %(V) Parasitic emf
% HEV_Param.Battery_Cell.Gpo = 2.0e-011; %(s)
% HEV_Param.Battery_Cell.Vpo = 0.12; %(V)
% HEV_Param.Battery_Cell.Ap = 2.0; %()
% HEV_Param.Battery_Cell.Taup = 3; % (s)

% Battery Block Main Branch Parameters
HEV_Param.Battery_Cell.Emo = 4.8 * 4; % (V) [max o.c. volts per cell]
HEV_Param.Battery_Cell.Ke = 0.0006; %(V/°C)
HEV_Param.Battery_Cell.Ao = -0.6; % ()
HEV_Param.Battery_Cell.R0 = 0.001; % (Ohm)
HEV_Param.Battery_Cell.R1 = 0.01; %(Ohm)
HEV_Param.Battery_Cell.R2 = 0.05; %(Ohm)
% HEV_Param.Battery_Cell.C1 = 10000; % (F)
HEV_Param.Battery_Cell.C2 = 200; % (F)
HEV_Param.Battery_Cell.Tau1 = 30; % (s)
    
% Compute initial extracted charge
HEV_Param.Battery_Cell.Qe_init = (1-HEV_Param.Battery_Cell.SOC_init)*HEV_Param.Battery_Cell.Kc*HEV_Param.Battery_Cell.Costar*interp1([HEV_Param.Battery_Cell.theta_f HEV_Param.Battery_Cell.Kt_Temps],[0 HEV_Param.Battery_Cell.Kt],HEV_Param.Battery_Cell.theta_init,'spline');


%% ULTRACAPACITOR PARAMETERS
HEV_Param.UltraCapacitor.Nominal_Capacitance = 1000; % Farad
HEV_Param.UltraCapacitor.Rate_C_V = 0.2;          % Farad/Volt
HEV_Param.UltraCapacitor.Series_R = 30/3;            % Ohm
HEV_Param.UltraCapacitor.Self_Discharge_R = 500;  % Ohm
HEV_Param.UltraCapacitor.Initial_Voltage = 217;     % Volt


%% MOTOR PARAMETERS
HEV_Param.Motor.Stator_Resistance = 0.0065*14;        
HEV_Param.Motor.Stator_Resistance = 0.0065*14;

% HEV_Param.Motor.TorqSpdLUT.SpeedRPM = [0 1000 2000 3000 4000 5000];
% HEV_Param.Motor.TorqSpdLUT.TorqueNm = [20 18 15 10 3 0.01];
HEV_Param.Motor.TorqSpdLUT.SpeedRPM = [0  1000 2000 3000 4000 5000 6000 7000 8000 9000 10000];
HEV_Param.Motor.TorqSpdLUT.TorqueNm = [40 40   40   40   40   40   35   30   20   15    0.01]*1.4;
% HEV_Param.Motor.TorqSpdLUT.SpeedRPM = [0  1000 2000 3000 4000 5000 6000 7000 8000];
% HEV_Param.Motor.TorqSpdLUT.TorqueNm = [30 28   25   21   18   15   11   8    0.01];
HEV_Param.Motor.Damping = 5e-5; %N*m/(rad/s)
HEV_Param.Motor.TorqueControl_TimeConst = 0.02*2/1.5;
HEV_Param.Motor.Shaft_Inertia = 0.002;
HEV_Param.Motor.Series_Resistance = 0.001; %CHG
HEV_Param.Motor.Inductances = [0.001597972349731   0.002057052250467];
HEV_Param.Motor.Efficiency = 91;


%% DC-DC CONVERTER PARAMETERS
HEV_Param.DCDCConv.Output_Voltage = 500;      % Volts
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

% Ts = 0.001;



% 08/02 2325 for faster the w response
% % PREDEFINED LI-ION BATTERY PARAMS
% HEV_Param.Battery_Det.Nominal_Voltage = 200;
% HEV_Param.Battery_Det.Rated_Capacity = 8.1;      % Ampere-hours
% HEV_Param.Battery_Det.Initial_SOC = 97;       % Percent
% HEV_Param.Battery_Det.Series_Resistance = 0.2/10;   % Ohm
% 
% 
% % FOR ACCEL TEST
% %HEV_Param.Battery_Det.Initial_SOC = 97;         % Percent
% 
% % GENERIC BATTERY PARAMS
% HEV_Param.Battery_Sys.Nominal_Voltage = 217;
% HEV_Param.Battery_Sys.Internal_Resistance = 0.24691;% Ohm
% HEV_Param.Battery_Sys.Rated_Capacity = 21;     % Ampere-hours
% HEV_Param.Battery_Sys.Initial_Charge = 21;    % Ampere-hours
% HEV_Param.Battery_Sys.Expn_Voltage = 215.0342;       % V
% HEV_Param.Battery_Sys.Expn_Charge = 2.3438;
% 
% HEV_Param.Battery_Sys.C1.Capacitance = 2500;
% HEV_Param.Battery_Sys.C1.Initial_Voltage = 19;
% HEV_Param.Battery_Sys.C1.Series_Resistance = 1e-6;
% HEV_Param.Battery_Sys.R2 = 0.3;
% HEV_Param.Battery_Sys.R1 = 1.8;
% HEV_Param.Battery_Sys.Maximum_Capacity = HEV_Param.Battery_Sys.Rated_Capacity;    % Ampere-hours
% 
% 
% % Battery Model Parameters
% % To compute soc, use battery_sys.maximum_capacity.
% 
% % Battery Block Initial Conditions
% % Initial State of Charge (% of full charge)
% % Initial Electrolyte Temperature (°C, typically same as ambient temp)
% HEV_Param.Battery_Cell.Rated_Capacity = 5;  % Ampere-hours
% HEV_Param.Battery_Cell.SOC_init = 0.9;
% HEV_Param.Battery_Cell.theta_init = 25;
% 
% % Battery Block Thermal Parameters
% HEV_Param.Battery_Cell.Ctheta = 200; %(J/°C) Thermal Capacitance
% HEV_Param.Battery_Cell.Area = 0.01; % (m^2) Surface area of battery exposed to air
% HEV_Param.Battery_Cell.Rtheta = 20;  %(W/m^2/K) Convective heat transfer coefficient
% 
% % Battery Block Capacity Parameters
% % Charge/discharge cycles at ranges of current/temp
% HEV_Param.Battery_Cell.Kc = 1.2; %()
% HEV_Param.Battery_Cell.Costar = 1.8e+005; %(As)
% HEV_Param.Battery_Cell.Kt_Temps = [25 40 60 75]; % Temperature breakpoints for Kt LUT
% HEV_Param.Battery_Cell.Kt = [0.80,1.10,1.20,1.12;]; %() LUT output values
% HEV_Param.Battery_Cell.delta = 0.73; %()
% HEV_Param.Battery_Cell.Istar = 20; %(A) Nominal Current (=cap/disch_t)
% HEV_Param.Battery_Cell.theta_f = -40; %(°C) Electrolyte Freezing Temp
% 
% % Battery Block Parasitic Branch Parameters
% % End of charge cycle at ranges of current/temp
% HEV_Param.Battery_Cell.Ep = 1.95; %(V) Parasitic emf
% HEV_Param.Battery_Cell.Gpo = 2.0e-011; %(s)
% HEV_Param.Battery_Cell.Vpo = 0.12; %(V)
% HEV_Param.Battery_Cell.Ap = 2.0; %()
% HEV_Param.Battery_Cell.Taup = 3; % (s)
% 
% % Battery Block Main Branch Parameters
% HEV_Param.Battery_Cell.Emo = 24; % (V) [max o.c. volts per cell]
% HEV_Param.Battery_Cell.Ke = 0.0006; %(V/°C)
% HEV_Param.Battery_Cell.Ao = -0.6; % ()
% HEV_Param.Battery_Cell.R0 = 0.003; % (Ohm)
% HEV_Param.Battery_Cell.R1 = 0.01; %(Ohm)
% HEV_Param.Battery_Cell.R2 = 0.05; %(Ohm)
% % HEV_Param.Battery_Cell.C1 = 10000; % (F)
% HEV_Param.Battery_Cell.C2 = 200; % (F)
% HEV_Param.Battery_Cell.Tau1 = 30; % (s)
% 
% % Compute initial extracted charge
% HEV_Param.Battery_Cell.Qe_init = (1-HEV_Param.Battery_Cell.SOC_init)*HEV_Param.Battery_Cell.Kc*HEV_Param.Battery_Cell.Costar*interp1([HEV_Param.Battery_Cell.theta_f HEV_Param.Battery_Cell.Kt_Temps],[0 HEV_Param.Battery_Cell.Kt],HEV_Param.Battery_Cell.theta_init,'spline');
% 
% 
% %% ULTRACAPACITOR PARAMETERS
% HEV_Param.UltraCapacitor.Nominal_Capacitance = 1000; % Farad
% HEV_Param.UltraCapacitor.Rate_C_V = 0.2;          % Farad/Volt
% HEV_Param.UltraCapacitor.Series_R = 30/3;            % Ohm
% HEV_Param.UltraCapacitor.Self_Discharge_R = 500;  % Ohm
% HEV_Param.UltraCapacitor.Initial_Voltage = 217;     % Volt
% 
% 
% %% MOTOR PARAMETERS
% HEV_Param.Motor.Stator_Resistance = 0.0065*14;        
% HEV_Param.Motor.Stator_Resistance = 0.0065*14;
% 
% % HEV_Param.Motor.TorqSpdLUT.SpeedRPM = [0 1000 2000 3000 4000 5000];
% % HEV_Param.Motor.TorqSpdLUT.TorqueNm = [20 18 15 10 3 0.01];
% HEV_Param.Motor.TorqSpdLUT.SpeedRPM = [0  1000 2000 3000 4000 5000 6000 7000 8000];
% HEV_Param.Motor.TorqSpdLUT.TorqueNm = [30 28   25   21   18   15   11   8    0.01];
% 
% HEV_Param.Motor.Damping = 3e-4; %N*m/(rad/s)
% HEV_Param.Motor.TorqueControl_TimeConst = 0.02*2/1.5;
% HEV_Param.Motor.Shaft_Inertia = 0.02;
% HEV_Param.Motor.Series_Resistance = 0.01; %CHG
% HEV_Param.Motor.Inductances = [0.001597972349731   0.002057052250467];
% HEV_Param.Motor.Efficiency = 91;
% 
% 
% %% DC-DC CONVERTER PARAMETERS
% HEV_Param.DCDCConv.Output_Voltage = 300;      % Volts
% HEV_Param.DCDCConv.Resistance_Losses = 1000/40^2;      % Ohm
% HEV_Param.DCDCConv.Kp = 0.01;
% HEV_Param.DCDCConv.Ki = 10;
% HEV_Param.DCDCConv.MinVin = 50;
% 
% HEV_Param.DCDCConv.Mean_Boost.Kp = 0.001;
% HEV_Param.DCDCConv.Mean_Boost.Ki = 1;
% 
% HEV_Param.DCDCConv.EPower2Heat = 0.1;      % Watts/Watts
% HEV_Param.DCDCConv.Thermal_Mass = 0.1*10;    % kg
% HEV_Param.DCDCConv.Specific_Heat = 100;   % J/kg/K
% HEV_Param.DCDCConv.Initial_Temperature = 25;     % C
% HEV_Param.DCDCConv.Air_Temperature = 298;     % K
% HEV_Param.DCDCConv.Convection.Area = 20;     % cm^2
% HEV_Param.DCDCConv.Convection.HT_Coefficient = 100; % W/(m^2*K)
% 
% 
% %% CONTROLLER PARAMETERS
% HEV_Param.Control.Engine_Start_RPM = 800; % RPM
% HEV_Param.Control.Engine_Stop_RPM = 790; % RPM
% HEV_Param.Control.Mode_Logic_TS = 0.1;
% HEV_Param.Control.ICE.Kp = 0.02;
% HEV_Param.Control.ICE.Ki = 0.01;
% HEV_Param.Control.Gen.Kp = 10;
% HEV_Param.Control.Gen.Ki = 3;
% HEV_Param.Control.Mot.Kp = 500;
% HEV_Param.Control.Mot.Ki = 300;
% HEV_Param.Control.Veh_Spd.Kp = 0.02;
% HEV_Param.Control.Veh_Spd.Ki = 0.04;
% 
% % Ts = 0.001;
% 
