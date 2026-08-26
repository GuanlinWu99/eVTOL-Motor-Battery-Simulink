%% PMSM / inverter / SA88 parameters (base workspace).

% ---- Motor (Evolito D500-class) ----
Rs   = 0.003;      % [Ohm]    stator resistance / phase
Ld   = 40e-6;      % [H]      d-axis inductance
Lq   = 40e-6;      % [H]      q-axis inductance
psim = 0.045;      % [Wb]     PM flux linkage
p    = 10;         % [-]      pole pairs
Bm   = 1e-3;       % [N*m*s]  viscous damping
Jtot = 2.0;        % [kg*m^2] motor+rotor inertia (standalone testbench)
Imax = 1450;       % [A]      peak current limit
Kt   = 1.5*p*psim; % [Nm/A]   torque constant

% ---- Rotor load / operating point ----
k_drag    = 0.0012718;                     % [N*m*s^2] rotor drag  T = k_drag*w^2
w_ref_val = [724.9; 726.5; 725.1; 726.7];  % [rad/s]   per-rotor hover speed refs

% ---- Flight integration (PMSM_Drive) ----
Jm_flight  = 0.0600;                        % [kg*m^2] drive inertia
Vdc_fixed  = 780;                           % [V]      fixed DC bus
RPMMAX     = 10000;                         % [rpm]    max motor speed (normalises Rotor Assembly.N)
trqR_hover = [668.4; 671.3; 668.6; 671.5];  % [N*m]    hover torque (standalone test)

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
