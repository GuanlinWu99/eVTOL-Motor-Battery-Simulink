
%% eVTOL hover simulation (fixed) - rotor aero torque + feedforward

clc;
clear;
close all;

m_total       = 3175;      % kg
g             = 9.81;      % m/s^2
n_rotors      = 4;
R             = 0.7;       % rotor radius [m]
rho           = 1.225;     % air density [kg/m^3]
omega_ref_rpm = 7000;      % RPM target
C_T           = 0.0147;    % thrust coeff (given)
K_t           = 0.1047;    % motor torque constant [Nm/A] (given)
K_tau         = 0.0018;    % rotor torque constant mapping (given) (T_aero = K_tau * F)
J             = 0.009;     % rotor inertia [kg*m^2] (your earlier value)
b             = 0.001;     % viscous damping [Nm/(rad/s)] (small)
Kp            = 0.5;       % P gain (tuneable)

% Simulation settings
dt            = 0.001;
t_end         = 2.0;
time          = 0:dt:t_end;
N             = length(time);

% Prealloc
omega         = zeros(1,N);  % rad/s
T_motor_cmd   = zeros(1,N);
I_motor       = zeros(1,N);
T_aero        = zeros(1,N);
F_rotor       = zeros(1,N);

% Derived
A             = pi*R^2;
omega_ref     = omega_ref_rpm * 2*pi/60;

% Feedforward: torque required at reference speed to produce hover thrust
F_total           = m_total * g;
F_per_rotor_hover = F_total / n_rotors;

% Compute omega_ref-based aero-torque (feedforward target)
% We need the rotor speed that produces F_per_rotor_hover if using C_T model.
% But user wants omega_ref=7000RPM as commanded; we'll compute aero torque at omega_ref
F_at_omega_ref   = C_T * rho * A * (omega_ref * R)^2;        % Thrust generated at 7000RPM
T_aero_ref       = K_tau * F_at_omega_ref;                   % Aero torque at 7000RPM

% If thrust at 7000RPM doesn't equal required hover thrust, 
% you might want to solve for omega_ref that satisfies F_per_rotor_hover.
% For now we keep omega_ref fixed and show whether thrust matches hover need.

% Initial condition
omega(1) = 0;

%% Main Loop
for k = 1 : N-1
    % aerodynamic thrust & torque at current speed
    F_rotor(k) = C_T * rho * A * (omega(k) * R)^2;    % [N]
    T_aero(k) = K_tau * F_rotor(k);                   % [Nm]
    
    % Controller: feedforward (T_aero at omega_ref) + P feedback on speed error
    T_ff = T_aero_ref;                                % feedforward to sustain hover at omega_ref
    T_fb = Kp * (omega_ref - omega(k));               % P controller
    T_motor_cmd(k) = T_ff + T_fb;
    
    % Motor current command
    I_motor(k) = T_motor_cmd(k) / K_t;
    
    % Rotor dynamics: J*omega_dot = T_motor - T_aero - b*omega
    omega_dot = (T_motor_cmd(k) - T_aero(k) - b*omega(k)) / J;
    omega(k+1) = omega(k) + dt * omega_dot;
end

%%
% Final step values
F_rotor(end)     = C_T*rho*A*(omega(end)*R)^2;
T_aero(end)      = K_tau*F_rotor(end);
T_motor_cmd(end) = T_aero_ref + Kp*(omega_ref - omega(end));
I_motor(end)     = T_motor_cmd(end)/K_t;

% Convert to RPM
omega_rpm = omega * 60/(2*pi);

% Plots
figure('Position',[100 100 700 600]);
subplot(4,1,1);
plot(time, omega_rpm, 'LineWidth',2); grid on;
ylabel('Rotor speed [RPM]');
title('Rotor speed');

subplot(4,1,2);
plot(time, F_rotor, 'LineWidth',2); grid on;
ylabel('Rotor thrust [N]');
title('Per-rotor aerodynamic thrust');

subplot(4,1,3);
plot(time, T_motor_cmd, 'LineWidth',2); hold on;
plot(time, T_aero, 'LineWidth',2);
ylabel('Torque [Nm]');
legend('T_{motor\_cmd}','T_{Aero}');
grid on;
title('Torque [Nm]');

subplot(4,1,4);
plot(time, I_motor, 'LineWidth',2);
ylabel('Motor current [A]');
xlabel('Time [s]');
legend('I_{motor}');
grid on;
title('Current [A]');

% Display final steady values
fprintf('Final rotor RPM: %.1f RPM\n', omega_rpm(end));
fprintf('Final per-rotor thrust: %.1f N (required: %.1f N)\n', F_rotor(end), F_per_rotor_hover);
fprintf('Final motor torque cmd: %.2f Nm, aero torque: %.2f Nm\n', T_motor_cmd(end), T_aero(end));
fprintf('Final motor current command: %.2f A\n', I_motor(end));
