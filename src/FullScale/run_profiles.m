%% run2s.m - headless short-sim driver (setup copied from startup.m, sim shortened)

%% ===== setup (verbatim from startup.m lines 18-205) =====
bdclose('all')
clear all; close all; clear mex;
%% mac: no MATLAB project file here, set the paths explicitly
W='/Volumes/PortableSSD/files/maltab/kiat_pmsm';
addpath(genpath([W '/src'])); addpath([W '/models']); addpath([W '/models/Motor_batt']);
% src/FullScale/full_scale holds a stale copy of setupTransitionGuidanceMission_mod
% that loads a 3600 m demo mission instead of the profile. genpath puts it ahead of
% src/full_scale, so prepend the good directory to win.
addpath([W '/src/full_scale']);
addpath([W '/data']); addpath(genpath([W '/utilities'])); addpath([W '/steps']);
cd(W);

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
%% ===== multi-profile driver =====
% One MATLAB session, one accelerator build, every profile reused. P1 to P10 all
% carry 11 waypoints, so only parameter VALUES change between them and the compiled
% target stays valid. A '### Starting serial model build' line on the second or
% later profile would mean that assumption broke.
zzT = getenv('SIM_STOPTIME'); if isempty(zzT), zzT = '900'; end
zzList = getenv('SIM_PROFILES'); if isempty(zzList), zzList = '4'; end
zzP = str2double(strsplit(zzList, ','));
zzAccel = getenv('SIM_ACCEL'); if isempty(zzAccel), zzAccel = '1'; end
if strcmp(zzAccel,'1')
    try, set_param(mdl,'SimulationMode','accelerator'); catch zzE, fprintf(2,'accel: %s\n',zzE.message); end
else
    set_param(mdl,'SimulationMode','normal');
end
fprintf('mode = %s | profiles = %s | stop = %s s\n', get_param(mdl,'SimulationMode'), zzList, zzT);

zzWall = zeros(1,numel(zzP));
for zzI = 1:numel(zzP)
    Profile = zzP(zzI);
    fprintf('\n########## P%d  (%d of %d)  %s ##########\n', Profile, zzI, numel(zzP), datestr(now,'HH:MM:SS'));
    run('profile_setup.m');           % verbatim from startup.m
    zzTic = tic;
    % Accelerator: compiles the model to native code. The first run pays a build, then
    % the solver loop runs compiled. Set SIM_ACCEL=0 to fall back to normal mode.
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
    fprintf('--- battery and currents (To Workspace) ---\n');
    % To Workspace variables are packaged into the SimulationOutput, they do not land
    % in the base workspace when the model is run through sim().
    zzHas = @(n) ismember(n, outTuned.who);
    zzGet = @(n) outTuned.(n);
    try
        vd = zzGet('pmsm_Vdc');  ic = zzGet('pmsm_Idc');
        sc = zzGet('pmsm_SOC');  ia = zzGet('pmsm_Iabc');
        vdD=vd.Data(:); icD=ic.Data; scD=sc.Data(:); iaD=ia.Data;
        if size(icD,2)==1, itot=icD; else, itot=sum(icD,2); end
        fprintf('Vdc   : end=%.1f  min=%.1f V\n', vdD(end), min(vdD(vdD>100)));
        fprintf('Idc   : end=%.0f A total, peak %.0f A\n', itot(end), max(itot));
        fprintf('SOC   : %.4f -> %.4f\n', scD(1), scD(end));
        fprintf('Iabc  : %d samples x %d ch, peak %.0f A, rms %.0f A\n', ...
                size(iaD,1), size(iaD,2), max(abs(iaD(:))), rms(iaD(:)));
        zzdt = mean(diff(ia.Time(1:min(200,numel(ia.Time)))));
        fprintf('        record rate %.0f Hz (%d samples over %.1f s)\n', 1/zzdt, numel(ia.Time), ia.Time(end));
    catch e
        fprintf(2,'To Workspace read failed: %s\n', e.message);
        fprintf(2,'outTuned has: %s\n', strjoin(outTuned.who','; '));
    end

    % validation set for section 9 of PMSM_INVERTER_EQUATIONS.md: the detailed iq
    % against the algebraic reconstruction iq = Tdrag/Kt on the same run.
    try
        V.t_det = outTuned.pmsm_Idq.Time;
        V.idq   = outTuned.pmsm_Idq.Data;                   % [id1 iq1 id2 iq2 ...]
        V.t_drg = outTuned.Rotor1_Drag_Tq.Time;
        V.drag  = [squeeze(outTuned.Rotor1_Drag_Tq.Data(:)) squeeze(outTuned.Rotor2_Drag_Tq.Data(:)) ...
                   squeeze(outTuned.Rotor3_Drag_Tq.Data(:)) squeeze(outTuned.Rotor4_Drag_Tq.Data(:))];
        V.rpm   = [squeeze(outTuned.Rotor1_RPM.Data(:)) squeeze(outTuned.Rotor2_RPM.Data(:)) ...
                   squeeze(outTuned.Rotor3_RPM.Data(:)) squeeze(outTuned.Rotor4_RPM.Data(:))];
        V.t_rpm = outTuned.Rotor1_RPM.Time;
        V.rpmref = [squeeze(outTuned.Rotor1_RPM_Reference.Data(:)) squeeze(outTuned.Rotor2_RPM_Reference.Data(:)) ...
                    squeeze(outTuned.Rotor3_RPM_Reference.Data(:)) squeeze(outTuned.Rotor4_RPM_Reference.Data(:))];
        V.t_ref = outTuned.Rotor1_RPM_Reference.Time;
        zzXe = squeeze(outTuned.UAV_State.Xe.Data); if size(zzXe,1)==3, zzXe=zzXe.'; end
        V.alt = -zzXe(:,3); V.t_alt = outTuned.UAV_State.Xe.Time;
        V.mode = double(outTuned.Flight_Mode.Data); V.t_mode = outTuned.Flight_Mode.Time;
        V.airspeed = squeeze(outTuned.UAV_State.airspeed.Data);
        V.tilt = [squeeze(outTuned.UAV_State.RotorParameters.Tilt1.Data(:)) squeeze(outTuned.UAV_State.RotorParameters.Tilt2.Data(:))];
        V.t_tilt = outTuned.UAV_State.RotorParameters.Tilt1.Time;
        V.Kt = Kt; V.p = p; V.psim = psim; V.Jm = Jm_flight; V.Bm = Bm;
        try
            V.Iabc = outTuned.pmsm_Iabc.Data; V.t_Iabc = outTuned.pmsm_Iabc.Time;
            V.Idc  = outTuned.pmsm_Idc.Data;  V.Vdc = outTuned.pmsm_Vdc.Data;
            V.SOC  = outTuned.pmsm_SOC.Data;  V.t_elec = outTuned.pmsm_Vdc.Time;
        catch, end
        save(['/Volumes/PortableSSD/files/maltab/kiat_pmsm/pmsm_P' num2str(Profile) '_' zzT 's.mat'],'V','-v7.3');
        fprintf('saved pmsm_P%d_%ss.mat\n', Profile, zzT);
    catch e
        fprintf(2,'validation save failed: %s\n', e.message);
    end

    zzTt = outTuned.tout; zzN = max(1, round(numel(zzTt)/(200*max(zzTt(end),1))));  % ~200 Hz
    R.t=zzTt(1:zzN:end);
    R.rpm=[g('Rotor1_RPM') g('Rotor2_RPM') g('Rotor3_RPM') g('Rotor4_RPM')];
    R.rpm=R.rpm(1:zzN:end,:);
    try, R.drag=[g('Rotor1_Drag_Tq') g('Rotor2_Drag_Tq') g('Rotor3_Drag_Tq') g('Rotor4_Drag_Tq')]; R.drag=R.drag(1:zzN:end,:); catch, end
    save(['/Volumes/PortableSSD/files/maltab/kiat_pmsm/run_out_P' num2str(Profile) '_' zzT 's.mat'],'R','-v7.3');
    fprintf('DONE %s\n', datestr(now,'HH:MM:SS')); fprintf('saved run_out\n========================================\n');

    zzWall(zzI) = toc(zzTic);
    fprintf('P%d wall %.0f s\n', Profile, zzWall(zzI));
    clear outTuned R V
end
fprintf('\n=== wall time per profile ===\n');
for zzI = 1:numel(zzP), fprintf('  P%-3d %6.0f s\n', zzP(zzI), zzWall(zzI)); end
fprintf('total %.1f min\n', sum(zzWall)/60);
