function Simulation_Plot_with_Currents(data, p, psim)
%SIMULATION_PLOT_WITH_CURRENTS  Battery-performance panel (as Simulation_Plot)
% PLUS the 3-phase inverter currents. Accepts either input format:
%
%   Simulation_Plot_with_Currents(outTuned)   % raw sim output (has Battery_Data,
%                                             % Rotor#_RPM, Rotor#_Drag_Tq)
%   Simulation_Plot_with_Currents(R)          % the mission-summary struct saved
%                                             % in mission_run.mat (.rpm/.Idq/.wave)
%   Simulation_Plot_with_Currents             % loads mission_run.mat (R)
%
% 3-phase currents come from the logged Idq/wave when present, otherwise they are
% reconstructed from torque & speed (iq=Te/Kt, id=0, theta=int(p*omega), inv Park).

if nargin < 1 || isempty(data)
    S = load('mission_run.mat'); data = S.R;
end
if nargin < 2 || isempty(p),    p = getparam('p', 10);        end
if nargin < 3 || isempty(psim), psim = getparam('psim', 0.045); end
Kt = 1.5*p*psim;

D = extract(data, p, Kt);          % common data struct (fields may be empty)

%% ================= figure =================
figure('Units','normalized','OuterPosition',[0.05 0.06 0.62 0.86],'Color','w');
LW = 1.4; F = 12;
haveBatt = ~isempty(D.soc) || ~isempty(D.volt) || ~isempty(D.cur);

% ---- top row: battery (or flight if no battery logged) ----
subplot(3,3,1);
if ~isempty(D.soc), plot(D.tb,D.soc,'LineWidth',LW); ylabel('%'); title('SOC','FontSize',F);
elseif ~isempty(D.alt), plot(D.t,D.alt,'LineWidth',LW); ylabel('m'); title('Altitude','FontSize',F);
end
grid on; xlabel('Time (s)');

subplot(3,3,2);
if ~isempty(D.volt), plot(D.tb,D.volt,'LineWidth',LW); ylabel('V'); title('Battery Voltage','FontSize',F);
elseif ~isempty(D.eul), plot(D.t,D.eul*180/pi,'LineWidth',1.0); ylabel('deg'); title('Attitude (Euler)','FontSize',F); legend('roll','pitch','yaw','Location','best');
end
grid on; xlabel('Time (s)');

subplot(3,3,3);
if ~isempty(D.cur), plot(D.tb,D.cur,'LineWidth',LW); ylabel('A'); title('Battery DC Current','FontSize',F);
elseif ~isempty(D.idc), plot(D.t,sum(D.idc,2),'LineWidth',LW); ylabel('A'); title('Pack DC Current (sum)','FontSize',F);
end
grid on; xlabel('Time (s)');

% ---- pack power ----
subplot(3,3,4);
if haveBatt && ~isempty(D.volt) && ~isempty(D.cur)
    plot(D.tb, D.volt(:).*D.cur(:)/1000,'LineWidth',LW);
elseif ~isempty(D.idc)
    plot(D.t, getVdc(D)*sum(D.idc,2)/1000,'LineWidth',LW);
end
grid on; title('Pack Power','FontSize',F); ylabel('kW'); xlabel('Time (s)');

% ---- rotor speed ----
subplot(3,3,5);
if ~isempty(D.rpm), plot(D.t, D.rpm,'LineWidth',1.0); end
grid on; title('Rotor speed','FontSize',F); ylabel('rpm'); xlabel('Time (s)'); legend('1','2','3','4','Location','best');

% ---- NEW: 3-phase current RMS envelope (per motor) ----
subplot(3,3,6);
if ~isempty(D.iq), plot(D.t, abs(D.iq)/sqrt(2),'LineWidth',LW); end
grid on; title('Phase current RMS (per motor)','FontSize',F); ylabel('A rms'); xlabel('Time (s)');
legend('M1','M2','M3','M4','Location','best');

% ---- NEW: reconstructed 3-phase current waveform (zoom) ----
subplot(3,3,[7 8]);
if ~isempty(D.tw)
    plot(D.tw, D.ia, D.tw, D.ib, D.tw, D.ic,'LineWidth',1.1); grid on;
    legend('i_a','i_b','i_c','Location','northeast');
    title(sprintf('Motor 1 — 3-phase inverter currents  (%.3g–%.3g s, f_e=%.0f Hz)', ...
          D.tw(1), D.tw(end), D.fe),'FontSize',F);
    ylabel('A'); xlabel('Time (s)');
end

% ---- summary ----
subplot(3,3,9); axis off;
lines = {sprintf('K_t = %.3f Nm/A', Kt)};
if ~isempty(D.iq),  lines{end+1} = sprintf('peak phase I: %.0f A', max(abs(D.iq(:)))); end
if ~isempty(D.iq),  lines{end+1} = sprintf('max RMS: %.0f A', max(abs(D.iq(:))/sqrt(2))); end
lines{end+1} = sprintf('run: %.1f s', D.t(end));
if ~haveBatt, lines{end+1} = '(no battery logged in this run)'; end
text(0.05,0.75, lines,'FontSize',13,'VerticalAlignment','top');

sgtitle('eVTOL — Battery Performance + Inverter 3-Phase Currents','FontSize',18,'FontWeight','bold');
end

% =====================================================================
function D = extract(x, p, Kt)
D = struct('t',[],'tb',[],'rpm',[],'iq',[],'idc',[],'alt',[],'eul',[], ...
           'soc',[],'volt',[],'cur',[],'tw',[],'ia',[],'ib',[],'ic',[],'fe',NaN);

if isa(x,'Simulink.SimulationOutput')          % ---- raw outTuned ----
    names = x.who; has = @(n) any(strcmp(names,n));
    w = []; Te = []; t = [];
    for k = 1:4
        if has(sprintf('Rotor%d_RPM',k))
            [t,rpm] = ts(x, sprintf('Rotor%d_RPM',k)); w(:,k) = rpm(:)*2*pi/60; %#ok<AGROW>
            dn = sprintf('Rotor%d_Drag_Tq',k);
            if has(dn), [td,dd]=ts(x,dn); Te(:,k)=interp1(td,abs(dd(:)),t,'linear','extrap'); %#ok<AGROW>
            else, Te(:,k)=Kt*ones(size(t)); end
        end
    end
    if ~isempty(w), D.t=t; D.rpm=w*60/2/pi; D.iq=Te/Kt; end
    [D.tb,D.soc]  = batt(x,'SOC____');
    [~,  D.volt]  = batt(x,'Voltage__V_');
    [~,  D.cur]   = batt(x,'Current__A_');

elseif isstruct(x) && isfield(x,'rpm')          % ---- mission-summary R ----
    D.t   = x.t(:);
    D.rpm = x.rpm;
    if isfield(x,'Idq'),  D.iq  = x.Idq(:,2:2:end); end     % iq columns
    if isfield(x,'Idc'),  D.idc = x.Idc; end
    if isfield(x,'Xe'),   xe=x.Xe; if size(xe,2)==3, D.alt=-xe(:,3); end; end
    if isfield(x,'Euler'),D.eul = x.Euler; end
    if isfield(x,'drag_tq') && isempty(D.iq), D.iq = x.drag_tq/Kt; end
    if isfield(x,'wave') && isstruct(x.wave)                % pre-computed waveform
        D.tw = x.wave.t(:); iab = x.wave.Iabc;
        D.ia = iab(:,1); D.ib = iab(:,2); D.ic = iab(:,3);
    end
end

% reconstruct a waveform window if none was supplied
if isempty(D.tw) && ~isempty(D.rpm)
    dt = 5e-5; t0 = min(2, D.t(end)*0.3);
    D.tw = (t0:dt:t0+0.04).';
    om = interp1(D.t, D.rpm(:,1)*2*pi/60, D.tw,'linear','extrap');
    iq = interp1(D.t, D.iq(:,1),          D.tw,'linear','extrap');
    th = cumtrapz(D.tw, p*om);
    D.ia=-iq.*sin(th); D.ib=-iq.*sin(th-2*pi/3); D.ic=-iq.*sin(th+2*pi/3);
    D.fe = p*mean(om)/(2*pi);
end
if isnan(D.fe) && ~isempty(D.rpm), D.fe = p*mean(D.rpm(:,1))*2*pi/60/(2*pi); end
end

function v = getVdc(D); v = 780; end %#ok<INUSD>  fixed bus for power estimate

function v = getparam(nm,dflt); try, v=evalin('base',nm); catch, v=dflt; end; end

function [t,d] = ts(o,n)
x=o.(n);
if isa(x,'timeseries'), t=x.Time(:); d=squeeze(x.Data); else, d=squeeze(x); t=(1:numel(d)).'; end
if size(d,1)~=numel(t)&&size(d,2)==numel(t), d=d.'; end
end

function [t,d] = batt(o,field)
t=[]; d=[];
try, b=o.Battery_Data.Batt.(field); t=b.Time(:); d=squeeze(b.Data); catch, end
end
