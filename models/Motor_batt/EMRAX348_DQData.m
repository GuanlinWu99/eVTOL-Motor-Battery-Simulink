%% Parameters for DQ-frame Simscape PMSM + FOC — EMRAX 348 (340 kW / 1000 Nm)
%
% Drop-in replacement for the MathWorks IPMSM 35 kW demo data script
% (IPMSMAxleDriveEVDQData.m). Same variable names, EMRAX 348 values.
%
% Motor : EMRAX 348, axial-flux PMSM, 700 Vdc winding (matched to a 720 V bus).
% Source: EMRAX 348 datasheet V1.6.
% NOTE  : EMRAX publishes a single inductance -> treat as NON-salient
%         (Ld = Lq, id = 0 control; no reluctance / MTPA table needed).
%
% This configuration uses the ONLINE (computed) PMSM Current Reference
% Generator. In the block mask set  Reference type = Computed  (not
% "Lookup-table based"); then ID_MAP/IQ_MAP/RPM_VECT/... are NOT used.


%% Machine Parameters
Pmax = 340000;     % Maximum (peak S2 30s) power       [W]
Tmax = 1000;       % Maximum (peak 30s) torque         [N*m]
Ld   = 52e-6;      % Stator d-axis inductance          [H]
Lq   = 52e-6;      % Stator q-axis inductance (= Ld)   [H]   non-salient
L0   = 26e-6;      % Stator zero-sequence inductance   [H]   (~Ld/2, demo ratio)
Rs   = 0.00474;    % Stator resistance per phase       [Ohm]
psim = 0.06249;    % Permanent magnet flux linkage     [Wb]
p    = 10;         % Number of pole pairs
Jm   =  0.09042;    % Rotor inertia                     [kg*m^2]
Bm   = 1e-3;       % Rotor viscous damping             [N*m/(rad/s)]  small, not critical

% Ratings (for saturation / current limits)
Nmax  = 3250;      % Limiting speed                    [rpm]
Imax  = 1070;      % Peak phase current (30 s)         [A rms]
Icont = 450;       % Continuous phase current          [A rms]
% Derived: Kt = 1.5*p*psim = 0.937 Nm/A ; iq(1000 Nm) ~ 1067 A

%% High-Voltage Battery / DC-link Parameters
Cdc  = 0.001;      % DC-link capacitor                 [F]
Vnom = 720;        % Nominal DC bus voltage            [V]
V1   = 700;        % Voltage V1 (< Vnom)               [V]
AH0  = 280;        % Initial battery charge            [hr*A]   <-- set to your pack

%% Control Parameters — current-loop PI
% Scaled from the working 35 kW demo gains to preserve its design/bandwidth
% (Kp proportional to L, Ki proportional to Rs). Verify with an id/iq step.
Kp_id = 0.8779 ;  %* (Ld / 0.00024368);   % ~0.187  Proportional gain, id
Ki_id = 710.3004; %* (Rs / 0.010087);     % ~334    Integrator  gain, id
Kp_iq = 1.0744  ; %* (Lq / 0.00029758);   % ~0.188  Proportional gain, iq
Ki_iq = 1.0615e3 ;%* (Rs / 0.010087);     % ~499    Integrator  gain, iq

%% Current References
% ONLINE reference generator (Reference type = Computed) -> no lookup .mat 
% or generate an EMRAX id*/iq* map (surface PM: id=0, iq=Te/(1.5*p*psim)).
