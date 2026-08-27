%% run2s.m - headless short-sim driver (setup copied from startup.m, sim shortened)
proj = openProject('E:\file\matlab\eVTOL_Simulator_260511_v15_with_sa88\kiat_battery_project\VTOLRefApp.prj');

%% ===== setup (verbatim from startup.m lines 18-205) =====
bdclose('all')
clear all; close all; clear mex;
cleanup; clc;

mdl ='VTOLTiltrotor';
load_system(mdl);
cfgRefTop    = getActiveConfigSet('VTOLTiltrotor');
cfgTop       = getRefConfigSet(cfgRefTop);
set_param(cfgTop,'SolverType','Fixed-step','Solver','ode4');
Ts           = 0.001;
mdlSub       = 'VTOLDynamics';
load_system(mdlSub);
cfgSubActive = getActiveConfigSet(mdlSub);

Simulink.defineIntEnumType('flightState',{'Hover','Transition','FixedWing','BackTransition'},[0;1;2;3],'StorageType','uint8');

xGround=0; yGround=0; zGround=0; iniRoll=0; iniYaw=0/180*pi; initPitch=0; iniP=0; iniQ=0; iniR=0;

load("contact.mat")
pmsm_batt_testbench_data;
load_ctrl_interface();
load_digital_twin_interface;
load_controller_gains;
const = load_const();

motorctrl.p=2.28; motorctrl.i=0.001; motorctrl.d=0; motorctrl.n=100;
Limit=700;
BATTERY_NP=21;
Profile=4;
eVTOL_MTOW=7000; target_temperature=25;
Wind=0; Wind_Speed=0; Yaw_Wind_Moment=0;

uavParams = load_vtol_dynamics_7000lb(const, Profile, eVTOL_MTOW, target_temperature, BATTERY_NP);
if eVTOL_MTOW==7000, VTOLcontrolGains.P_Z = 1.5*VTOLcontrolGains.P_Z; end
controlParams = load_controller_parameters(uavParams, const);
Visualization=0; SensorType=0; TuningMode=0; Deployment=false;   % headless: no animator
vIni=0*const.kts2mps;
setupHoverConfiguration_mod
setupTransitionGuidanceMission_mod;
configObj = getActiveConfigSet('VTOLAutopilotController');
set_param(configObj,'SourceName','VTOLConfiguration');
transition_throttle=0.2;

%% ===== short sim =====
try, Simulink.sdi.clear; catch, end                 % free C-drive SDI archive
set_param(cfgTop,'StopTime','0.4');
fprintf('\n>>> running 0.4 s sim (Jm_flight=%.4g)...\n', Jm_flight);
tic; outTuned = sim(mdl); fprintf('sim wall time: %.0f s\n', toc);

%% ===== report =====
g=@(n) squeeze(outTuned.(n).Data);
rpm1=g('Rotor1_RPM');
fprintf('\n================ RESULT ================\n');
fprintf('Rotor1_RPM: max=%.1f  final=%.2f  (was stuck at 1.6)\n', max(abs(rpm1)), rpm1(end));
try
  us=outTuned.UAV_State; xd=squeeze(us.Xe.Data); if size(xd,1)==3, xd=xd.'; end
  fprintf('Altitude:   max=%.2f  final=%.2f m\n', max(-xd(:,3)), -xd(end,3));
catch, end
for k=1:4
  try, r=g(sprintf('Rotor%d_RPM',k)); fprintf('  Rotor%d max rpm=%.1f\n',k,max(abs(r))); catch, end
end
fprintf('--- diagnostic taps (motor 1) ---\n');
d=@(n) squeeze(outTuned.(n).Data);
try, v=d('trqR1_dbg');   fprintf('trqR (Sum4)   : start=%.2f  end=%.2f  max=%.2f Nm\n', v(1), v(end), max(abs(v))); catch e, fprintf('trqR: %s\n',e.message); end
try, v=d('pid1_dbg');    fprintf('PID out       : start=%.2f  end=%.2f  max=%.2f Nm\n', v(1), v(end), max(abs(v))); catch e, fprintf('pid: %s\n',e.message); end
try, v=d('dragfed_dbg'); if size(v,1)<size(v,2), v=v.'; end; fprintf('Drag fed(m1)  : start=%.2f  end=%.2f  max=%.2f Nm\n', v(1,1), v(end,1), max(abs(v(:,1)))); catch e, fprintf('drag: %s\n',e.message); end
try, v=d('idq1_dbg');    if size(v,1)<size(v,2), v=v.'; end; fprintf('id (m1)       : end=%.2f | iq (m1): end=%.2f A\n', v(end,1), v(end,2)); catch e, fprintf('idq: %s\n',e.message); end
try, v=d('motrpm_dbg');  if size(v,1)<size(v,2), v=v.'; end; fprintf('Mot_RPM out m1: end=%.3f rpm  max=%.3f\n', v(end,1), max(abs(v(:,1)))); catch e, fprintf('motrpm: %s\n',e.message); end
fprintf('--- settling trajectory (t : rotor1_rpm  motor1_rpm  trqR) ---\n');
tt=outTuned.tout;
try, mr=d('motrpm_dbg'); if size(mr,1)<size(mr,2), mr=mr.'; end; catch, mr=nan(numel(tt),1); end
try, tq=d('trqR1_dbg'); catch, tq=nan(numel(tt),1); end
for tsec=[0.02 0.05 0.1 0.15 0.2 0.3 0.4]
  [~,ix]=min(abs(tt-tsec));
  fprintf('  %.2f :  %8.1f   %8.1f   %8.1f\n', tt(ix), rpm1(ix), mr(ix,1), tq(ix));
end
% save compact for plotting
R.t=outTuned.tout;
R.rpm=[g('Rotor1_RPM') g('Rotor2_RPM') g('Rotor3_RPM') g('Rotor4_RPM')];
try, R.drag=[g('Rotor1_Drag_Tq') g('Rotor2_Drag_Tq') g('Rotor3_Drag_Tq') g('Rotor4_Drag_Tq')]; catch, end
save('E:\file\matlab\eVTOL_Simulator_260511_v15_with_sa88\kiat_battery_project\run2s_out.mat','R','-v7.3');
fprintf('saved run2s_out.mat\n========================================\n');
