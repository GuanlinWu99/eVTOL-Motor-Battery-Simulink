%% Parameters for DQ-frame Simscape PMSM + FOC — Evolito D500-class (~350 kW)
%
% Drop-in alternative to EMRAX348_DQData.m. Same variable names.
% Purpose: DIRECT-DRIVE the EXISTING rotor (which needs ~6250 rpm to hover),
% i.e. a high-speed, high-power axial-flux PMSM that EMRAX 348 (3250 rpm) could
% not reach. Requirement per motor (x4), reverse-engineered from the sim:
%    ~6250 rpm hover, ~535 Nm, ~360 kW, on a 720 V bus.
%
% Motor  : Evolito D500-class axial-flux PMSM (350 kW, 12 kW/kg, ~30 kg,
%          aerospace, Rolls-Royce ACCEL). Surface-magnet -> NON-salient (Ld=Lq).
% NOTE   : Evolito does NOT publish Ld/Lq/psim/pole-pairs. The ELECTRICAL values
%          below are REPRESENTATIVE, chosen so the motor can deliver ~535 Nm at
%          6250 rpm within a 720 V bus (back-EMF + reactance fit under the bus).
%          The TORQUE-SPEED-POWER LIMITS (Nmax/Tmax/Pmax) are what actually
%          decide whether it flies, and are set to D500-class + margin.
%
% Reference type = Computed (online PMSM Current Reference Generator).
% (Auto-generated 2026-07-13)

%% Machine Parameters  (STACKED ~700 kW config, e.g. 2x D500)
% Takeoff needs ~35000 N total (system-level reference) -> ~600 kW & ~770 Nm
% per rotor at ~7400 rpm. A single D500 (400 kW) is power-limited; Evolito axial
% flux stacks to ~1 MW, so a ~700 kW stack is used here.
Pmax = 700000;     % Maximum (peak) power              [W]   (~2x D500 stack)
Tmax = 950;        % Maximum (peak) torque             [N*m] (takeoff needs ~770)
Ld   = 40e-6;      % Stator d-axis inductance          [H]   low L -> voltage headroom @7400 rpm
Lq   = 40e-6;      % Stator q-axis inductance (= Ld)   [H]   non-salient
L0   = 20e-6;      % Stator zero-sequence inductance   [H]
Rs   = 0.004;      % Stator resistance per phase       [Ohm]
psim = 0.045;      % Permanent magnet flux linkage     [Wb]  back-EMF@7400rpm < bus
p    = 10;         % Number of pole pairs
Jm   = 0.08;       % Rotor inertia                     [kg*m^2] (stacked axial-flux disks)
Bm   = 1e-3;       % Rotor viscous damping             [N*m/(rad/s)]  small, not critical

% Ratings (for saturation / current limits)
Nmax  = 8000;      % Max mechanical speed              [rpm]  ABOVE the ~7400 rpm takeoff point
Imax  = 1450;      % Peak phase current                [A rms] (~950 Nm at Kt=0.675)
Icont = 800;       % Continuous phase current          [A rms]
% Derived: Kt = 1.5*p*psim = 0.675 Nm/A ; iq(770 Nm) ~ 1141 A ; iq(950 Nm) ~ 1407 A
% Back-EMF @7400rpm: psim*p*775 = 349 V (LN peak) -> ~604 V LL peak < 720 V bus  OK
% NOTE: 600 kW @ 7400 rpm on 720 V is near the voltage limit -> field weakening
%       will engage; current is high (~1200 A). Physically borderline (small rotor).

%% High-Voltage Battery / DC-link Parameters
Cdc  = 0.001;      % DC-link capacitor                 [F]
Vnom = 720;        % Nominal DC bus voltage            [V]
V1   = 700;        % Voltage V1 (< Vnom)               [V]
AH0  = 280;        % Initial battery charge            [hr*A]   <-- set to your pack

%% Control Parameters — current-loop PI
% Scaled from the 35 kW demo gains (Kp ~ L, Ki ~ Rs) to preserve its bandwidth.
Kp_id = 0.8779   * (Ld / 0.00024368);   % ~0.180  Proportional gain, id
Ki_id = 710.3004 * (Rs / 0.010087);     % ~423    Integrator  gain, id
Kp_iq = 1.0744   * (Lq / 0.00029758);   % ~0.181  Proportional gain, iq
Ki_iq = 1.0615e3 * (Rs / 0.010087);     % ~631    Integrator  gain, iq

%% Current References
% ONLINE reference generator (Reference type = Computed) -> no lookup .mat.
% Surface PM (Ld=Lq): id=0 below base speed, id<0 (field weakening) near 6250 rpm.
