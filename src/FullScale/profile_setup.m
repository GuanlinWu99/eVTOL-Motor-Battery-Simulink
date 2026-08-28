%% profile_setup.m
% Lines 83..186 of startup.m, extracted verbatim so a multi-profile
% driver cannot drift from it. The only change is that Profile comes from the
% caller. Regenerate whenever the profile block in startup.m changes.

%... Flight profile
% Profile is set by the caller

%... Set up VTOL dynamics parameters
switch Profile
    case {1,2,6}
        eVTOL_MTOW = 5600;
    case {9,10}
        eVTOL_MTOW = 6020;
    case {3,4,5,7,8}
        eVTOL_MTOW = 7000;
    case 11
        eVTOL_MTOW = 5600;
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

uavParams = load_vtol_dynamics_7000lb(const, Profile, eVTOL_MTOW, target_temperature, BATTERY_NP);


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
