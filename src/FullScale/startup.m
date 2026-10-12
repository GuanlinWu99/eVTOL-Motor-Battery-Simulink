%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: start up script for tilt-rotor eVTOL MATLAB/Simulink simulator          %
% Author(s): Minhyun, Sounghwan, and Guanlin                                     %
% Description and updates (08/25/2026):                                          %
% 1. Include drag torque calculation in the powertrain                           %
% 2. Upgrade battery pack in the powertrain                                      %
% 3. Changed from Variable-Type to Fixed-Type Ts = 0.001 (09/26/2025)            %
% 4. Included flaps and spoilers (11/13/2025)                                    %
% 5. Replacement of Prof. Jung's group 2RC-ECM battery model (04/01/2026)        %
% 6. Completed Arrhenius relation for battery parameter fitting (04/15/2026)     %
% 7. Completed the P8 flight profile (most critical case) (05/11/2026)           %
% 8. Rotor diameter increased to 3.9 m and powertrain rescaled (08/09/2026)      %
% 9. One battery pack for all MTOW via BATTERY_NP (08/12/2026)                   %
% 10. Switched the cell to the Amprius SA88 data sheet values (08/14/2026)       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ... Clear workspace, command window and close figures
bdclose('all')
clear all; close all; clear mex;
cleanup; clc; 

%% ... Load eVTOL model
mdl ='VTOLTiltrotor';
load_system(mdl);

cfgRefTop    = getActiveConfigSet('VTOLTiltrotor');         % mdl = 'VTOLTiltrotor'
cfgTop       = getRefConfigSet(cfgRefTop);
set_param(cfgTop,'SolverType','Fixed-step','Solver','ode4');
Ts           = 0.001;                                       % [s] sample time
mdlSub       = 'VTOLDynamics';
load_system(mdlSub);
cfgSubActive = getActiveConfigSet(mdlSub);

%% ... Define the aircraft modes of flight
Simulink.defineIntEnumType('flightState',{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3],'StorageType','uint8');

% Hover mode : 0
% Transition mode : 1
% Fixedwing mode : 2
% Back Transition mode : 3

%% ... Initialize simulator: velocity defined later
xGround     =   0;
yGround     =   0;
zGround     =   0;
iniRoll     =   0;
iniYaw      =   0/180*pi;
initPitch   =   0;
iniP        =   0;
iniQ        =   0;
iniR        =   0;

%... Initialize eVTOL landing gear model
load("contact.mat")

%... Load bus interfaces for controller
load_ctrl_interface();

%... Load bus interfaces for plant
load_digital_twin_interface;

%... Initialize Control and Guidance gains for Tiltrotor
load_controller_gains;

%... Load constants
const = load_const();

% ... Call motor speed PID controller gains
motorctrl.p  = 2.28;
motorctrl.i  = 0.001;
motorctrl.d  = 0;
motorctrl.n  = 100;
Limit        = 700;                    % [Nm] saturation of the motor speed PID correction

% P1, P2, P3 : Short Distance (8.2 NM)
% P4, P5, P9, P10 : Middle Distance (16 NM)
% P6, P7, P8 : Long Distance (23 NM)
% P11 : Test profile

%... Flight profile
Profile = 1;

%... Set up VTOL dynamics parameters
switch Profile
    case {1,2,6}
        eVTOL_MTOW = 5600;      % [lbs]
    case {9,10}
        eVTOL_MTOW = 6020;      % [lbs]
    case {3,4,5,7,8}
        eVTOL_MTOW = 7000;      % [lbs]
    case 11
        eVTOL_MTOW = 5600;      % [lbs]
end

%... Operating temperature
switch Profile
    case {1,3,7,10}
        target_temperature = 20;
    case {4,11}
        target_temperature = 25;
    case{2,5,6,8,9}
        target_temperature = 45;    
end

%... 10 Knots = 5 m/s (Moderate Crosswind)
%... 15 Knots = 7 m/s (Most Strong Crosswind)
switch Profile
    case {1,4,5,7}
        Wind = 0;            % Disable Crosswind function
        Wind_Speed = 0;      % Wind velocity [m/s]
    case {3,6}               
        Wind = 1;            % Active Crosswind function   
        Wind_Speed = 5;      % Wind velocity [m/s] 
    case {2,8,9,10}       
        Wind = 1;            % Active Crosswind function
        Wind_Speed = 7;      % Wind velocity [m/s]
    case 11
        Wind = 1;     
        Wind_Speed = 5;
end

if Wind_Speed == 0
%% No Wind
    Yaw_Wind_Moment = 0;

%% Moderate Wind
elseif (Wind_Speed == 5) && (eVTOL_MTOW == 5600)
    Yaw_Wind_Moment = 1400;
elseif (Wind_Speed == 5) && (eVTOL_MTOW == 6020)
    Yaw_Wind_Moment = 2500;
    VTOLcontrolGains.P_YAW_RATE = VTOLcontrolGains.P_YAW_RATE * 1;
    VTOLcontrolGains.D_YAW_RATE = VTOLcontrolGains.D_YAW_RATE * 1;
elseif (Wind_Speed == 5) && (eVTOL_MTOW == 7000)
    Yaw_Wind_Moment = 3500;
    VTOLcontrolGains.P_YAW_RATE = VTOLcontrolGains.P_YAW_RATE * 1;
    VTOLcontrolGains.D_YAW_RATE = VTOLcontrolGains.D_YAW_RATE * 1;

%% Strong Wind (To be updated...)
elseif (Wind_Speed == 7) && (eVTOL_MTOW == 5600)
    Yaw_Wind_Moment = 3000;
elseif (Wind_Speed == 7) && (eVTOL_MTOW == 6020)
    Yaw_Wind_Moment = 3000;
    VTOLcontrolGains.P_YAW_RATE = VTOLcontrolGains.P_YAW_RATE * 1;
    VTOLcontrolGains.D_YAW_RATE = VTOLcontrolGains.D_YAW_RATE * 1;
elseif (Wind_Speed == 7) && (eVTOL_MTOW == 7000)
    Yaw_Wind_Moment = 3000;
    VTOLcontrolGains.P_YAW_RATE = VTOLcontrolGains.P_YAW_RATE * 1;
    VTOLcontrolGains.D_YAW_RATE = VTOLcontrolGains.D_YAW_RATE * 1;
end

uavParams = load_vtol_dynamics_7000lb(const, eVTOL_MTOW, target_temperature);

if eVTOL_MTOW == 7000
    VTOLcontrolGains.P_Z  = 1.5 * VTOLcontrolGains.P_Z;
end

%% Other setting...
%... Load controllercontrolParams.TiltScheduleRate parameters
controlParams           =   load_controller_parameters(uavParams, const);

%... Flag to enable/disable visualization
Visualization           =   1;

%... Disable Sensors
SensorType              =   0;

%... Setup tuning flag
TuningMode              =   0;
Deployment              =   false;

%... Initialize initial velocity
vIni = 0*const.kts2mps;
disp("Initialized VTOL model.")

%... Initialize hover configuration
setupHoverConfiguration_mod
setupTransitionGuidanceMission_mod;

%... Setup configuration set
configObj = getActiveConfigSet('VTOLAutopilotController');
set_param(configObj, 'SourceName', 'VTOLConfiguration');
transition_throttle = 0.2;

%% Display eVTOL Mission Profiles
fprintf('\n')
disp(['✅ Flight Profile: P', num2str(Profile)]);
disp(['📍 Travel Distance: ', num2str(TransitionMission(end).position(1)), ' (m)']);
disp(['🕒 Total Simulation Time: ', num2str(Total_sim_time), ' (s)']);
disp(['🛩️ eVTOL MTOW: ', num2str(eVTOL_MTOW), ' (lbs)']);
disp(['🌪️ Cross-Wind Speed: ', num2str(Wind_Speed), ' (m/s)']);
disp(['🔋 Battery Pack Capacity: ', num2str((uavParams.Battery_Pack_Capacity*uavParams.Battery_Pack_Voltage)/1000), ' (kWh)']);
disp(['⚡ Battery Pack Voltage: ', num2str(uavParams.Battery_Pack_Voltage), ' (V)']);
disp(['🌡️ Operation Temperature: ', num2str(uavParams.Temperature), ' (°C)']);
Drive_Name = ["BLDC + Abstract inverter", "PMSM + inverter"];
disp(['⚙️ Motor Drive: ', char(Drive_Name(uavParams.InverterModel + 1))]);

if uavParams.Temperature == 20
    Pre_Conditioning_Energy = uavParams.Battery_Pack_Mass*950*50/3.6e6;           % [kWh]
    disp(['🎯 Pre-Conditioning from -30 °C to 20 °C ... ', num2str(Pre_Conditioning_Energy), ' (kWh)', ' is required.']);
    fprintf('\n')
end

%% Run Simulink
set_param(mdl,'SimulationMode','accelerator'); 

LOG_DEC = 500;         % 500 × 20 µs = 10 ms
for m = {mdl, mdlSub}  % VTOLTiltrotor, VTOLDynamics
  load_system(m{1});
  b = find_system(m{1}, 'LookUnderMasks','all', 'FollowLinks','on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'ToWorkspace');
  for i = 1:numel(b)
    if str2double(get_param(b{i},'Decimation')) <= 2
      set_param(b{i}, 'Decimation', num2str(LOG_DEC));
    end
  end
end
set_param(cfgTop, 'Decimation', num2str(LOG_DEC));   

outTuned = sim(mdl);
keyboard;

switch uavParams.Battery_Cell.type
    case 1
        filename = sprintf('P%d_MTOW_%d_Wind_%d_Temp_%d_SA88.mat', Profile, eVTOL_MTOW, Wind_Speed, target_temperature);
    case 2
        filename = sprintf('P%d_MTOW_%d_Wind_%d_Temp_%d_LG.mat', Profile, eVTOL_MTOW, Wind_Speed, target_temperature);
end
if uavParams.InverterModel == 0, filename = strrep(filename, '.mat', '_NoInverter.mat'); end
save(filename, 'outTuned', 'TransitionMission', 'uavParams', 'controlParams', 'const', '-v7.3');

%% Plot Simulation Plots
switch Profile
    case 11
    Simulation_Plot_Debugging();      
    otherwise
    %Simulation_Plot();
    %Simulation_Plot_Cleaned();
    Simulation_Compare();
end

