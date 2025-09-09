%% Fixed hover sim: feedforward + P control on RPM, correct aero torque & dynamics
clc; clear; close all;

% --- given parameters ---
m         = 3000;              % kg
g         = 9.81;              % m/s^2
n_rotor   = 4;

R         = 0.7;               % rotor radius [m]
A         = pi*R^2;
rho       = 1.225;

RPM_ref   = 7000;
omega_ref = RPM_ref*2*pi/60;   % [rad/s]

C_T       = 0.0147;            % thrust coeff 
C_Q       = 0.0018;            % torque coeff 
K_t       = 0.1047;            % motor torque constant [Nm/A] 

J         = 0.007;             % rotor inertia [kg*m^2] (example from your data)
b         = 0.001;             % viscous damping [Nm/(rad/s)]
Kp        = 0.1;               % P gain (tuneable)

% --- derived ---
F_total   = m*g;
F_per     = F_total / n_rotor;

% Aerodynamic torque model: Q_aero = C_Q * rho * A * R^3 * omega^2
% (dimensionally: rho [M/L^3]*A[L^2]*R^3[L^3]*omega^2[1/T^2] -> M L^2 / T^2 = Nm)
aeroTorque = @(omega) C_Q * rho * A * R^3 .* (omega.^2);

% Aerodynamic thrust (for info): T = C_T * rho * A * (omega*R)^2
aeroThrust = @(omega) C_T * rho * A .* ( (omega.*R).^2 );

% Compute feedforward torque at omega_ref
Q_aero_ref = aeroTorque(omega_ref);
T_ff       = Q_aero_ref + b*omega_ref;   % include viscous term in feedforward

% --- sim settings ---
dt         = 0.001;
t_end      = 1;
time       = 0:dt:t_end;
N          = length(time);

omega      = zeros(1,N);
T_cmd      = zeros(1,N);
I_cmd      = zeros(1,N);
Q_a        = zeros(1,N);
F_r        = zeros(1,N);

omega(1)   = 0;

for k = 1 : N-1
    % current aerodynamic values
    Q_a(k)     = aeroTorque(omega(k));
    F_r(k)     = aeroThrust(omega(k));
    
    % controller: feedforward + P on omega (rad/s)
    T_cmd(k)   = T_ff + Kp * (omega_ref - omega(k));
    
    % Note that T_ff will be updated every time step.

    % motor current (from commanded torque)
    I_cmd(k)   = T_cmd(k) / K_t;
    
    % rotor dynamics: J * omega_dot = T_cmd - Q_a - b*omega
    omega_dot  = (T_cmd(k) - Q_a(k) - b*omega(k))/J;
    omega(k+1) = omega(k) + omega_dot*dt;
end

% last values
Q_a(end) = aeroTorque(omega(end));
F_r(end) = aeroThrust(omega(end));
T_cmd(end) = T_ff + Kp*(omega_ref - omega(end));
I_cmd(end) = T_cmd(end)/K_t;

% convert to RPM
RPM = omega*60/(2*pi);

% plot
figure('Position',[100 100 800 700]);
subplot(5,1,1); plot(time,RPM,'LineWidth',2); grid on;
ylabel('RPM'); title('Rotor speed');

subplot(5,1,2); plot(time, F_r,'LineWidth',2); grid on;
ylabel('Thrust [N] per rotor');

subplot(5,1,3); plot(time, Q_a,'LineWidth',2); grid on;
ylabel('Aero torque [Nm]');

subplot(5,1,4); 
plot(time, T_cmd,'LineWidth',2); 
ylabel('T_{cmd} [Nm]');
legend('T_{cmd}')
grid on;

subplot(5,1,5); 
plot(time, I_cmd,'LineWidth',2); 
ylabel('I_{cmd} [A]');
xlabel('Time [s]'); grid on;
legend('I_{cmd}')

% print final summary
fprintf('Final RPM: %.1f RPM\n', RPM(end));
fprintf('Final per-rotor thrust (at final omega): %.1f N (required per rotor: %.1f N)\n', F_r(end), F_per);
fprintf('Final aero torque: %.2f Nm, Final T_cmd: %.2f Nm, Final I_cmd: %.2f A\n', Q_a(end), T_cmd(end), I_cmd(end));
