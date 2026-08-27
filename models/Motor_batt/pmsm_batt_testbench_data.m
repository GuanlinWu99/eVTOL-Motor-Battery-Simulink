%% PMSM / inverter / SA88 parameters (base workspace).

% ---- Motor: Evolito D1500 x2 stacked per rotor (axial flux, ~80 kg, <=2500 rpm) ----
% Ratings ~2880 Nm peak / ~540 kW at 720V; Rs/Ld/psim/p are ESTIMATES (Evolito unpublished).
Rs   = 0.010;      % [Ohm]    stator resistance / phase (est, large axial-flux)
Ld   = 200e-6;     % [H]      d-axis inductance (est)
Lq   = 200e-6;     % [H]      q-axis inductance (est, non-salient)
psim = 0.16;       % [Wb]     PM flux (est; keeps back-EMF < 720/sqrt3 up to 1500 rpm)
p    = 15;         % [-]      pole pairs (est, high-pole axial flux)
Bm   = 1e-3;       % [N*m*s]  viscous damping
Jtot = 2.0;        % [kg*m^2] motor+rotor inertia (standalone testbench)
Imax = 800;        % [A]      peak current (Kt*Imax = 2880 Nm peak, covers 2769)
Kt   = 1.5*p*psim; % [Nm/A]   torque constant = 3.6

% ---- Rotor load / operating point (3.9 m rotor, MTOW 5600 lb) ----
% k_drag = Cq*rho*A*R^3 ; Cq=8/pi^3*0.007048, A=pi*(3.9/2)^2, R=1.95, rho=1.225.
% Hover per rotor: ~832 rpm (87.2 rad/s), ~1500 Nm, ~131 kW.
k_drag    = 0.1974;                        % [N*m*s^2] rotor drag  T = k_drag*w^2
w_ref_val = [87.2; 87.4; 87.2; 87.4];      % [rad/s]   per-rotor hover speed refs (~832 rpm)

% ---- Flight integration (PMSM_Drive), 3.9 m rotor ----
Jm_flight  = 0.09;                          % [kg*m^2] physical (shaft 0.009 + prop 0.08)
Vdc_fixed  = 720;                           % [V]      DC bus (low-speed motor is not voltage-limited)
RPMMAX     = 1500;                          % [rpm]    max motor speed (normalises Rotor Assembly.N)
trqR_hover = [1500; 1505; 1500; 1505];      % [N*m]    hover torque (standalone test)

% ---- Control gains ----
bw_i = 2*pi*1000;  Kp_i = Ld*bw_i;  Ki_i = Rs*bw_i;            % current loop, 1 kHz
bw_w = 2*pi*5;     Kp_w = Jtot*bw_w/Kt;  Ki_w = Kp_w*bw_w/10;  % speed loop, 5 Hz

% ---- Battery (SA88 200S200P, 1-RC ECM) ----
Ns = 200; Np = 200; Cell_Cap = 3.5;         % [-] [-] [Ah]
here = fileparts(mfilename('fullpath'));
D = load(fullfile(here,'..','..','src','FullScale','parameters','SA88_25_Degree.mat'));
[soc_bp, iSort] = sort(D.SOC_bp(:)/100);
col = @(v) reshape(v, [], 1);
ocv_cell = col(D.OCV_table); ocv_cell = ocv_cell(iSort);
rs_cell  = col(D.Rs_table);  rs_cell  = rs_cell(iSort);
r1_cell  = col(D.R1_table);  r1_cell  = r1_cell(iSort);
c1_cell  = col(D.C1_table);  c1_cell  = c1_cell(iSort);
OCV_pack = Ns*ocv_cell;      % [V]
R0_pack  = (Ns/Np)*rs_cell;  % [Ohm]
R1_pack  = (Ns/Np)*r1_cell;  % [Ohm]
C1_pack  = (Np/Ns)*c1_cell;  % [F]
Cap_pack = Np*Cell_Cap;      % [Ah]
SOC0   = 0.90;
tau_dc = 1e-4;     % [s] DC-link filter time constant

Ts_solver = 2e-5;  % [s] fixed step
StopTime  = 10;    % [s]

fprintf('PMSM+SA88 data loaded: Kt=%.3f, %.0fV, %.0fAh\n', Kt, Ns*mean(ocv_cell), Cap_pack);
