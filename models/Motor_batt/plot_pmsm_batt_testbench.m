function plot_pmsm_batt_testbench(out)
%PLOT_PMSM_BATT_TESTBENCH  Plot + summarise a PMSM/SA88 testbench run.
%   plot_pmsm_batt_testbench(out)   % out = sim('PMSM_SA88_4Motor')
% Handles the N-motor models (Id/Iq/wm/Te/Iabc/Idc/Vdc/SOC) and the single-motor
% testbench (Idq). With no argument it picks up 'out' (or 'ans') from the base ws.

if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')')
        out = evalin('base', 'out');
    elseif evalin('base', 'exist(''ans'',''var'')')
        out = evalin('base', 'ans');
    else
        error(['No simulation result found. Run:  out = sim(''PMSM_SA88_4Motor'');' ...
               '  then:  plot_pmsm_batt_testbench(out)']);
    end
end

g   = @(n) evalin('base', n);            % motor params live in the base workspace
has = @(n) isprop(out, n) || (isa(out,'Simulink.SimulationOutput') && ismember(n, out.who));

t  = out.wm.Time(:);
wm = squeeze(out.wm.Data);
Te = squeeze(out.Te.Data);

if has('Id')                              % N-motor model
    id = squeeze(out.Id.Data);
    iq = squeeze(out.Iq.Data);
else                                      % original single-motor testbench
    idq = out.Idq.Data;
    id  = idq(:,1);  iq = idq(:,2);
end
if size(wm,1) ~= numel(t), wm = wm.'; end     % guard against Nx1 squeeze flips
if size(id,1) ~= numel(t), id = id.'; iq = iq.'; Te = Te.'; end

Vdc = squeeze(out.Vdc.Data);
Idc = squeeze(out.Idc.Data);
if size(Idc,1) ~= numel(t), Idc = Idc.'; end
SOC = squeeze(out.SOC.Data);
Iab = squeeze(out.Iabc.Data);
nM  = size(wm, 2);

if has('Idc_pack'), Idc_pack = squeeze(out.Idc_pack.Data); else, Idc_pack = Idc; end

%% ---- numeric summary ----
p_  = g('p');   Rs_ = g('Rs');  Ld_ = g('Ld');  Lq_ = g('Lq');  psim_ = g('psim');
kd  = g('k_drag');
wref = g('w_ref_val');  wref = wref(:);      % one reference per motor
if numel(wref) < nM, wref = repmat(wref(1), nM, 1); end

fprintf('\n===== PMSM + SA88 testbench: %d motor(s), steady state at t = %.1f s =====\n', nM, t(end));
for k = 1:nM
    we   = p_*wm(end,k);
    Vmax = Vdc(end)/sqrt(3);
    vq   = Rs_*iq(end,k) + we*(Ld_*id(end,k) + psim_);
    vd   = Rs_*id(end,k) - we*Lq_*iq(end,k);
    fprintf(['  M%d: %6.1f rad/s (%4.0f rpm, ref %4.0f, miss %+5.0f) | id %6.1f  iq %6.1f A | ' ...
             'Te %6.1f Nm (drag %6.1f) | phase %4.0f A pk / %4.0f A rms | V %3.0f%%\n'], ...
        k, wm(end,k), wm(end,k)*60/2/pi, wref(k)*60/2/pi, (wm(end,k)-wref(k))*60/2/pi, ...
        id(end,k), iq(end,k), Te(end,k), kd*wm(end,k)^2, ...
        abs(iq(end,k)), abs(iq(end,k))/sqrt(2), 100*hypot(vd,vq)/Vmax);
end
fprintf('  pack: Vdc %.1f V | Idc %.1f A | %.1f kW | SOC %.4f -> %.4f\n', ...
    Vdc(end), Idc_pack(end), Vdc(end)*Idc_pack(end)/1e3, SOC(1), SOC(end));

shortfall = mean(wref(1:nM)) - mean(wm(end,:));
if shortfall > 1
    fprintf(2, '  NOTE: the motors fall %.1f rad/s (%.0f rpm) short of the reference --\n', ...
        shortfall, shortfall*60/2/pi);
    fprintf(2, '        the shared bus sags under the combined load, cutting Vmax and the\n');
    fprintf(2, '        feasible iq. Thrust ~ w^2, so this is a real hover deficit.\n');
end

%% ---- plots ----
figure('Color','w','Name','PMSM + SA88 testbench','Position',[60 50 1150 850]);

subplot(3,2,1);
multiplot(t, wm*60/2/pi, nM); hold on;
for k = 1:nM, yline(wref(k)*60/2/pi, 'r--'); end
ylabel('rpm'); title('Rotor speed (red = per-motor references)');
legend(motorNames(nM), 'Location','southeast');

spread = max(wm(end,:)) - min(wm(end,:));
if nM > 1 && spread < 0.5
    fprintf(2, '  NOTE: all %d motors converge to the SAME speed despite different\n', nM);
    fprintf(2, '        references (spread %.2g rad/s) -- they are all pinned at the\n', spread);
    fprintf(2, '        voltage ceiling, so the differential command has no effect.\n');
    fprintf(2, '        The speed curves overlap exactly; that is the result, not a plot bug.\n');
end

subplot(3,2,2);
multiplot(t, iq, nM); hold on;
set(gca,'ColorOrderIndex',1); plot(t, id, ':', 'LineWidth',1.0);
ylabel('A'); title('dq currents (i_q thick, i_d dotted)');

subplot(3,2,3);
multiplot(t, Te, nM); hold on;
plot(t, kd*wm.^2, ':k', 'LineWidth',1.0);
ylabel('N\cdotm'); title('Torque (dotted black = rotor drag)');

subplot(3,2,4);
plot(t, Vdc, 'LineWidth',1.3); grid on;
ylabel('V'); title('Shared pack terminal voltage V_{dc}');

subplot(3,2,5);
plot(t, Idc, 'LineWidth',1.0); grid on; hold on;
plot(t, Idc_pack, 'k', 'LineWidth',1.5);
ylabel('A'); xlabel('Time (s)');
title('DC current (thick black = pack total)');

% last 20 ms of motor 1's phase currents
subplot(3,2,6);
sel = t >= t(end)-0.02;
plot(t(sel), Iab(sel,1:3), 'LineWidth',1.1); grid on;
legend('i_a','i_b','i_c','Location','northeast');
ylabel('A'); xlabel('Time (s)');
we1 = p_*wm(end,1);
title(sprintf('Motor 1 phase currents (last 20 ms, f_e = %.0f Hz)', we1/(2*pi)));
end

function c = motorNames(n)
c = arrayfun(@(k) sprintf('M%d',k), 1:n, 'UniformOutput', false);
end

function multiplot(t, y, nM)
% One curve per motor, different line style + decreasing width, so overlapping
% (identical) curves stay distinguishable.
sty = {'-','--',':','-.'};
hold on;
for k = 1:nM
    plot(t, y(:,k), sty{mod(k-1,4)+1}, 'LineWidth', 2.2-0.4*k);
end
grid on;
end
