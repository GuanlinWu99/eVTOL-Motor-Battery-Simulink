function build_pmsm_drive_forflight(mdl)
%BUILD_PMSM_DRIVE_FORFLIGHT  Build the 4-motor drive subsystem 'PMSM_Drive'.
%   IN : trqR (4x1) [N*m], Drag_tq (4x1) [N*m]
%   OUT: Mot_RPM (4x1) [rpm], Iabc (12x1) [A], Idq (8x1) [A], Idc (4x1) [A],
%        Vdc [V], SOC [-]
% iq_ref = trqR/Kt; the flight PID owns the speed loop. Vdc comes from the
% shared SA88 pack. Copy PMSM_Drive into Propulsion; the surrounding blocks
% are a standalone test harness (trqR=const, Drag_tq=k_drag*wm^2).
%   Run:  pmsm_batt_testbench_data; build_pmsm_drive_forflight; sim('PMSM_Drive_ForFlight')

if nargin < 1 || isempty(mdl), mdl = 'PMSM_Drive_ForFlight'; end
if bdIsLoaded(mdl), close_system(mdl, 0); end
here = pwd; slx = fullfile(here, [mdl '.slx']);
if exist(slx,'file'), delete(slx); end
new_system(mdl); open_system(mdl);

%% ===== PMSM_Drive =====
add_block('built-in/Subsystem', [mdl '/PMSM_Drive'], 'Position',[260 120 420 260]);
buildDrive([mdl '/PMSM_Drive']);

%% ===== test harness =====
add_block('simulink/Sources/Constant', [mdl '/trqR'], ...
    'Value','trqR_hover', 'Position',[80 150 140 180]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/RotorLoad'], ...
    'Position',[260 320 420 380]);
setChart([mdl '/RotorLoad'], { ...
'function Drag = fcn(Mot_RPM)', ...
'wm   = Mot_RPM*2*pi/60;', ...
'Drag = k_drag .* wm .* abs(wm);'}, {'k_drag'}, '4');   % 4-vector I/O (one per motor)
set_param([mdl '/RotorLoad'],'Position',[260 320 420 380]);
add_block('built-in/Memory',[mdl '/DragLag'], ...      % algebraic-loop break
    'InitialCondition','0','Position',[150 320 210 360]);

tw=@(n,v,pos) add_block('simulink/Sinks/To Workspace',[mdl '/' n], ...
   'VariableName',v,'SaveFormat','Timeseries','Position',pos);
tw('ToWs_RPM','Mot_RPM',[480 120 540 150]);
tw('ToWs_Iabc','Iabc',  [480 165 540 195]);
tw('ToWs_Idq','Idq',    [480 210 540 240]);
tw('ToWs_Idc','Idc',    [480 255 540 285]);
tw('ToWs_Vdc','Vdc',    [480 300 540 330]);
tw('ToWs_SOC','SOC',    [480 345 540 375]);

L=@(a,b) add_line(mdl,a,b,'autorouting','on');
L('trqR/1','PMSM_Drive/1');
L('RotorLoad/1','DragLag/1'); L('DragLag/1','PMSM_Drive/2');
L('PMSM_Drive/1','RotorLoad/1');
L('PMSM_Drive/1','ToWs_RPM/1');
L('PMSM_Drive/2','ToWs_Iabc/1');
L('PMSM_Drive/3','ToWs_Idq/1');
L('PMSM_Drive/4','ToWs_Idc/1');
L('PMSM_Drive/5','ToWs_Vdc/1');
L('PMSM_Drive/6','ToWs_SOC/1');

set_param(mdl,'SolverType','Fixed-step','Solver','ode4', ...
             'FixedStep','Ts_solver','StopTime','StopTime');
save_system(mdl, slx);
fprintf('Built %s\n', slx);
end

%% ================= PMSM_Drive (For Each x4) =================
function buildDrive(s)
port(s,'In',{'trqR','Drag_tq'});
port(s,'Out',{'Mot_RPM','Iabc','Idq','Idc','Vdc','SOC'});

add_block('simulink/Ports & Subsystems/For Each Subsystem',[s '/Motors']);
set_param([s '/Motors'],'Position',[120 60 360 320]);
FE=[s '/Motors'];
% repurpose default In1/Out1
delete_line(FE,'In1/1','Out1/1');
set_param([FE '/In1'],'Name','trqR','Position',[20 40 50 60]);
set_param([FE '/Out1'],'Name','Mot_RPM','Position',[760 40 790 60]);
add_block('simulink/Sources/In1',[FE '/Drag_tq'],'Port','2','Position',[20 100 50 120]);
add_block('simulink/Sinks/Out1',[FE '/Iabc'],'Port','2','Position',[760 100 790 120]);
add_block('simulink/Sinks/Out1',[FE '/Idq'], 'Port','3','Position',[760 140 790 160]);
add_block('simulink/Sinks/Out1',[FE '/Idc'], 'Port','4','Position',[760 180 790 200]);

add_block('simulink/Sources/In1',[FE '/Vdc'],'Port','3','Position',[20 200 50 220]);
add_block('built-in/Subsystem',[FE '/FOC'],     'Position',[110 40 230 140]);
add_block('built-in/Subsystem',[FE '/Inverter'],'Position',[300 40 420 140]);
add_block('built-in/Subsystem',[FE '/PMSM'],    'Position',[490 40 610 170]);
add_block('simulink/Math Operations/Gain',[FE '/toRPM'],'Gain','60/(2*pi)','Position',[650 45 690 75]);
add_block('simulink/Signal Routing/Mux',[FE '/Mux_dq'], 'Inputs','2','Position',[650 120 655 160]);

buildFOC([FE '/FOC']);
buildInverter([FE '/Inverter']);
buildPMSM([FE '/PMSM']);

l=@(a,b) add_line(FE,a,b,'autorouting','on');
% FOC: 1 trqR, 2 wm, 3 id, 4 iq, 5 Vdc -> 1 vd_pi, 2 vq_pi
l('trqR/1','FOC/1'); l('PMSM/3','FOC/2'); l('PMSM/1','FOC/3'); l('PMSM/2','FOC/4'); l('Vdc/1','FOC/5');
% Inverter: 1 vd_pi,2 vq_pi,3 id,4 iq,5 wm,6 Vdc -> 1 vd,2 vq,3 idc
l('FOC/1','Inverter/1'); l('FOC/2','Inverter/2'); l('PMSM/1','Inverter/3');
l('PMSM/2','Inverter/4'); l('PMSM/3','Inverter/5'); l('Vdc/1','Inverter/6');
% PMSM: 1 vd,2 vq,3 Drag_tq -> 1 id,2 iq,3 wm,4 Te,5 Iabc
l('Inverter/1','PMSM/1'); l('Inverter/2','PMSM/2'); l('Drag_tq/1','PMSM/3');
% outputs
l('PMSM/3','toRPM/1'); l('toRPM/1','Mot_RPM/1');
l('PMSM/5','Iabc/1');
l('PMSM/1','Mux_dq/1'); l('PMSM/2','Mux_dq/2'); l('Mux_dq/1','Idq/1');
l('Inverter/3','Idc/1');

% trqR, Drag_tq partitioned per motor; Vdc broadcast
fe=[FE '/For Each'];
set_param(fe,'InputPartition',{'on','on','off'});
set_param(fe,'InputPartitionDimension',{'1','1','1'});
set_param(fe,'InputPartitionWidth',{'1','1','1'});
set_param(fe,'OutputConcatenationDimension',repmat({'1'},1,4));
set_param(fe,'SpecifiedNumIters','4');

% shared SA88 pack: sum the 4 motor DC currents
add_block('simulink/Math Operations/Gain',[s '/Total_Idc'], ...
   'Gain','ones(1,4)','Multiplication','Matrix(K*u)','Position',[420 360 460 390]);
add_block('built-in/Subsystem',[s '/Battery'],'Position',[500 340 620 420]);
buildBattery([s '/Battery']);

L=@(a,b) add_line(s,a,b,'autorouting','on');
L('trqR/1','Motors/1'); L('Drag_tq/1','Motors/2');
L('Battery/1','Motors/3');
L('Motors/1','Mot_RPM/1'); L('Motors/2','Iabc/1');
L('Motors/3','Idq/1');     L('Motors/4','Idc/1');
L('Motors/4','Total_Idc/1'); L('Total_Idc/1','Battery/1');
L('Battery/1','Vdc/1');    L('Battery/2','SOC/1');
end

%% ================= Battery (SA88 pack, 1-RC ECM) =================
function buildBattery(s)
port(s,'In',{'idc'});
port(s,'Out',{'Vdc','SOC'});

add_block('simulink/Math Operations/Gain',[s '/Coulomb_Count'], ...
   'Gain','-1/(3600*Cap_pack)','Position',[110 300 170 330]);
add_block('simulink/Continuous/Integrator',[s '/Int_SOC'], ...
   'InitialCondition','SOC0','Position',[200 300 230 330]);

add_block('simulink/Lookup Tables/1-D Lookup Table',[s '/OCV_vs_SOC'], ...
   'BreakpointsForDimension1','soc_bp','Table','OCV_pack','Position',[280 100 340 130]);
add_block('simulink/Lookup Tables/1-D Lookup Table',[s '/R0_vs_SOC'], ...
   'BreakpointsForDimension1','soc_bp','Table','R0_pack','Position',[280 150 340 180]);
add_block('simulink/Lookup Tables/1-D Lookup Table',[s '/R1_vs_SOC'], ...
   'BreakpointsForDimension1','soc_bp','Table','R1_pack','Position',[280 200 340 230]);
add_block('simulink/Lookup Tables/1-D Lookup Table',[s '/C1_vs_SOC'], ...
   'BreakpointsForDimension1','soc_bp','Table','C1_pack','Position',[280 250 340 280]);

add_block('simulink/User-Defined Functions/MATLAB Function',[s '/ECM_1RC'],'Position',[400 120 500 240]);
add_block('simulink/Continuous/Integrator',[s '/Int_V1'],'InitialCondition','0','Position',[540 200 570 230]);
add_block('simulink/Continuous/Transfer Fcn',[s '/DC_Link'], ...
   'Numerator','[1]','Denominator','[tau_dc 1]','Position',[540 120 600 160]);

setChart([s '/ECM_1RC'],{ ...
'function [Vterm, dV1] = fcn(idc, V1, OCV, R0, R1, C1)', ...
'Vterm = OCV - idc*R0 - V1;', ...
'dV1   = idc/C1 - V1/(R1*C1);'}, {});

l=@(a,b) add_line(s,a,b,'autorouting','on');
l('idc/1','Coulomb_Count/1'); l('Coulomb_Count/1','Int_SOC/1');
l('Int_SOC/1','OCV_vs_SOC/1'); l('Int_SOC/1','R0_vs_SOC/1');
l('Int_SOC/1','R1_vs_SOC/1');  l('Int_SOC/1','C1_vs_SOC/1');
l('Int_SOC/1','SOC/1');
l('idc/1','ECM_1RC/1');        l('Int_V1/1','ECM_1RC/2');
l('OCV_vs_SOC/1','ECM_1RC/3'); l('R0_vs_SOC/1','ECM_1RC/4');
l('R1_vs_SOC/1','ECM_1RC/5');  l('C1_vs_SOC/1','ECM_1RC/6');
l('ECM_1RC/2','Int_V1/1');
l('ECM_1RC/1','DC_Link/1');    l('DC_Link/1','Vdc/1');   % filter also breaks the Vdc->idc->Vdc loop
Simulink.BlockDiagram.arrangeSystem(s);
end

%% ================= FOC (torque-request based) =================
function buildFOC(s)
port(s,'In',{'trqR','wm','id','iq','Vdc'});
port(s,'Out',{'vd_pi','vq_pi'});

add_block('simulink/Math Operations/Gain',[s '/iq_ref'],'Gain','1/Kt','Position',[110 95 150 125]);
add_block('simulink/Sources/Constant',[s '/id_ref'],'Value','0','Position',[110 250 150 280]);
add_block('simulink/Math Operations/Sum',[s '/Sum_iq'],'Inputs','+-','Position',[330 175 350 205]);
add_block('simulink/Math Operations/Sum',[s '/Sum_id'],'Inputs','+-','Position',[190 250 210 280]);

add_block('simulink/User-Defined Functions/MATLAB Function',[s '/Iq_Limit'],'Position',[170 20 250 70]);
add_block('simulink/Math Operations/Gain',[s '/Neg'],'Gain','-1','Position',[270 20 300 50]);
add_block('simulink/Discontinuities/Saturation Dynamic',[s '/Iq_Sat'],'Position',[200 95 230 135]);

add_block('simulink/Continuous/PID Controller',[s '/PI_iq'], ...
   'Controller','PI','P','Kp_i','I','Ki_i','Position',[390 170 440 210]);
add_block('simulink/Continuous/PID Controller',[s '/PI_id'], ...
   'Controller','PI','P','Kp_i','I','Ki_i','Position',[250 245 300 285]);

setChart([s '/Iq_Limit'],{ ...
'function iq_lim = fcn(wm, Vdc)', ...
'we   = p*abs(wm);', ...
'Vmax = Vdc/sqrt(3);', ...
'Eb   = we*psim;', ...
'marg = Vmax^2 - Eb^2;', ...
'if we < 1 || marg <= 0', ...
'    iq_v = Imax;', ...
'else', ...
'    iq_v = sqrt(marg)/(we*Lq);', ...
'end', ...
'iq_lim = min(Imax, iq_v);'}, {'p','psim','Lq','Imax'});

l=@(a,b) add_line(s,a,b,'autorouting','on');
l('trqR/1','iq_ref/1');
l('iq_ref/1','Iq_Sat/2');            % signal to clamp
l('wm/1','Iq_Limit/1'); l('Vdc/1','Iq_Limit/2');
l('Iq_Limit/1','Iq_Sat/1'); l('Iq_Limit/1','Neg/1'); l('Neg/1','Iq_Sat/3');
l('Iq_Sat/1','Sum_iq/1'); l('iq/1','Sum_iq/2'); l('Sum_iq/1','PI_iq/1'); l('PI_iq/1','vq_pi/1');
l('id_ref/1','Sum_id/1'); l('id/1','Sum_id/2'); l('Sum_id/1','PI_id/1'); l('PI_id/1','vd_pi/1');
Simulink.BlockDiagram.arrangeSystem(s);
end

%% ================= Inverter (averaged, d-axis priority) =================
function buildInverter(s)
port(s,'In',{'vd_pi','vq_pi','id','iq','wm','Vdc'});
port(s,'Out',{'vd','vq','idc'});
add_block('simulink/User-Defined Functions/MATLAB Function',[s '/Bridge'],'Position',[150 80 240 180]);
add_block('simulink/User-Defined Functions/MATLAB Function',[s '/DC_Current'],'Position',[320 200 410 290]);
setChart([s '/Bridge'],{ ...
'function [vd, vq] = fcn(vd_pi, vq_pi, id, iq, wm, Vdc)', ...
'we   = p*wm;', ...
'vd_c = vd_pi - we*Lq*iq;', ...
'vq_c = vq_pi + we*(Ld*id + psim);', ...
'Vmax = Vdc/sqrt(3);', ...
'vd      = max(-Vmax, min(Vmax, vd_c));', ...
'vq_room = sqrt(max(Vmax^2 - vd^2, 0));', ...
'vq      = max(-vq_room, min(vq_room, vq_c));'}, {'p','Ld','Lq','psim'});
setChart([s '/DC_Current'],{ ...
'function idc = fcn(vd, vq, id, iq, Vdc)', ...
'Pac = 1.5*(vd*id + vq*iq);', ...
'idc = Pac/max(Vdc, 1);'}, {});
l=@(a,b) add_line(s,a,b,'autorouting','on');
l('vd_pi/1','Bridge/1'); l('vq_pi/1','Bridge/2'); l('id/1','Bridge/3');
l('iq/1','Bridge/4'); l('wm/1','Bridge/5'); l('Vdc/1','Bridge/6');
l('Bridge/1','vd/1'); l('Bridge/2','vq/1');
l('Bridge/1','DC_Current/1'); l('Bridge/2','DC_Current/2');
l('id/1','DC_Current/3'); l('iq/1','DC_Current/4'); l('Vdc/1','DC_Current/5');
l('DC_Current/1','idc/1');
Simulink.BlockDiagram.arrangeSystem(s);
end

%% ================= PMSM (Drag_tq as external load) =================
function buildPMSM(s)
port(s,'In',{'vd','vq','Drag_tq'});
port(s,'Out',{'id','iq','wm','Te','Iabc'});
add_block('simulink/User-Defined Functions/MATLAB Function',[s '/Electrical'],'Position',[150 100 240 200]);
add_block('simulink/Signal Routing/Mux',[s '/Mux_didq'],'Inputs','2','Position',[280 110 285 160]);
add_block('simulink/Continuous/Integrator',[s '/Int_idq'],'InitialCondition','[0;0]','Position',[310 120 340 150]);
add_block('simulink/Signal Routing/Demux',[s '/Demux_idq'],'Outputs','2','Position',[370 110 375 160]);
add_block('simulink/User-Defined Functions/MATLAB Function',[s '/Mechanics'],'Position',[290 280 380 350]);
add_block('simulink/Continuous/Integrator',[s '/Int_wm'],'InitialCondition','0','Position',[420 300 450 330]);
add_block('simulink/Math Operations/Gain',[s '/Elec_Speed'],'Gain','p','Position',[490 300 520 330]);
add_block('simulink/Continuous/Integrator',[s '/Int_theta'],'InitialCondition','0','Position',[550 300 580 330]);
add_block('simulink/User-Defined Functions/MATLAB Function',[s '/Park_inv'],'Position',[480 120 570 200]);
add_block('simulink/Signal Routing/Mux',[s '/Mux_abc'],'Inputs','3','Position',[610 130 615 180]);

setChart([s '/Electrical'],{ ...
'function [did, diq, Te] = fcn(vd, vq, id, iq, wm)', ...
'we  = p*wm;', ...
'did = (vd - Rs*id + we*Lq*iq)/Ld;', ...
'diq = (vq - Rs*iq - we*(Ld*id + psim))/Lq;', ...
'Te  = 1.5*p*(psim*iq + (Ld - Lq)*id*iq);'}, {'p','Rs','Ld','Lq','psim'});
setChart([s '/Mechanics'],{ ...
'function dwm = fcn(Te, Drag, wm)', ...
'dwm = (Te - Drag - Bm*wm)/Jm_flight;'}, {'Bm','Jm_flight'});
setChart([s '/Park_inv'],{ ...
'function [ia, ib, ic] = fcn(id, iq, th)', ...
'ia = id*cos(th)        - iq*sin(th);', ...
'ib = id*cos(th-2*pi/3) - iq*sin(th-2*pi/3);', ...
'ic = id*cos(th+2*pi/3) - iq*sin(th+2*pi/3);'}, {});

l=@(a,b) add_line(s,a,b,'autorouting','on');
l('vd/1','Electrical/1'); l('vq/1','Electrical/2');
l('Demux_idq/1','Electrical/3'); l('Demux_idq/2','Electrical/4'); l('Int_wm/1','Electrical/5');
l('Electrical/1','Mux_didq/1'); l('Electrical/2','Mux_didq/2');
l('Mux_didq/1','Int_idq/1'); l('Int_idq/1','Demux_idq/1');
l('Demux_idq/1','id/1'); l('Demux_idq/2','iq/1');
l('Electrical/3','Mechanics/1'); l('Drag_tq/1','Mechanics/2'); l('Int_wm/1','Mechanics/3');
l('Mechanics/1','Int_wm/1'); l('Int_wm/1','wm/1'); l('Electrical/3','Te/1');
l('Int_wm/1','Elec_Speed/1'); l('Elec_Speed/1','Int_theta/1');
l('Demux_idq/1','Park_inv/1'); l('Demux_idq/2','Park_inv/2'); l('Int_theta/1','Park_inv/3');
l('Park_inv/1','Mux_abc/1'); l('Park_inv/2','Mux_abc/2'); l('Park_inv/3','Mux_abc/3');
l('Mux_abc/1','Iabc/1');
Simulink.BlockDiagram.arrangeSystem(s);
end

%% ================= helpers =================
function port(s,kind,names)
if strcmp(kind,'In'), src='simulink/Sources/In1'; x=20; else, src='simulink/Sinks/Out1'; x=700; end
for k=1:numel(names)
   add_block(src,[s '/' names{k}],'Port',num2str(k),'Position',[x,60+40*k,x+30,80+40*k]);
end
end

function setChart(blkPath,lines,params,pin)
% pin: true -> pin I/O to scalar '1' (blocks INSIDE For Each); a char like '4'
%      -> pin to that size; false -> leave inherited.
if nargin < 4, pin = true; end
code=strjoin(lines,newline);
sfr=sfroot(); ch=sfr.find('-isa','Stateflow.EMChart','Path',blkPath);
ch.Script=code;
for k=1:numel(params), d=Stateflow.Data(ch); d.Name=params{k}; d.Scope='Parameter'; end
if ~isequal(pin,false)
   if ischar(pin), sz=pin; else, sz='1'; end
   dd=ch.find('-isa','Stateflow.Data');
   for k=1:numel(dd)
      if any(strcmp(dd(k).Scope,{'Input','Output'})), dd(k).Props.Array.Size=sz; end
   end
end
end
