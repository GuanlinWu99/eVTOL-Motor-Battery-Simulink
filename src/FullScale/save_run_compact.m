function R = save_run_compact(outTuned, fname, win, dec)
%SAVE_RUN_COMPACT  Save a small result file instead of the full outTuned.
%   R = save_run_compact(outTuned, 'run40.mat')
%   R = save_run_compact(outTuned, 'run40.mat', [10 10.05], 50)
%
% Slow signals (flight states, rotor speed, current envelope) are decimated by
% `dec` over the whole run; the fast 3-phase currents are kept at full
% resolution only inside `win` [s]. Turns a 1.2 GB save into a few MB.
%
% win  waveform window [s]      (default: last 50 ms)
% dec  decimation for slow data (default 50 -> 1 ms at a 20 us step)

if nargin < 2 || isempty(fname), fname = 'run_compact.mat'; end
if nargin < 4 || isempty(dec),   dec   = 50; end

us = outTuned.UAV_State;
t  = us.Xe.Time(:);
k  = 1:dec:numel(t);

R.t     = t(k);
R.Xe    = pick(us.Xe.Data, k);          % [m]     NED position
R.Euler = pick(us.Euler.Data, k);       % [rad]
R.pqr   = pick(us.pqr.Data, k);         % [rad/s]
R.Vb    = pick(us.Vb.Data, k);          % [m/s]

for m = 1:4
    n = sprintf('Rotor%d_RPM',m);
    if isprop(outTuned,n) || any(strcmp(outTuned.who,n))
        d = squeeze(outTuned.(n).Data);  R.rpm(:,m) = d(k);
    end
    n = sprintf('Rotor%d_Drag_Tq',m);
    if any(strcmp(outTuned.who,n))
        d = squeeze(outTuned.(n).Data);  R.drag_tq(:,m) = d(k);
    end
end

% currents: Battery_Data = [Iabc(12) Idq(8) Idc(4) Vdc SOC]
if any(strcmp(outTuned.who,'Battery_Data'))
    bd = outTuned.Battery_Data;  ti = bd.Time(:);  d = squeeze(bd.Data);
    if size(d,2) >= 24
        ki = 1:dec:numel(ti);
        R.t_i     = ti(ki);
        R.Idq     = d(ki,13:20);            % [A] id,iq per motor (slow)
        R.Idc     = d(ki,21:24);            % [A]
        if size(d,2) >= 26
            R.Vdc = d(ki,25);               % [V] pack terminal
            R.SOC = d(ki,26);               % [-] 0..1
        end
        if nargin < 3 || isempty(win), win = [ti(end)-0.05 ti(end)]; end
        sel = ti >= win(1) & ti <= win(2);
        R.wave.t    = ti(sel);              % full-resolution window
        R.wave.Iabc = d(sel,1:12);          % [A] 4 motors x 3 phases
        R.wave.win  = win;
    end
end

% Vdc/SOC logged separately (mux still 24 wide)
for n = {'Vdc','SOC'}
    if ~isfield(R,n{1}) && any(strcmp(outTuned.who,n{1}))
        s = outTuned.(n{1});  d = squeeze(s.Data);
        R.(n{1}) = d(1:dec:numel(d));
        R.t_b    = s.Time(1:dec:numel(d));
    end
end

save(fname, 'R', '-v7');
d = dir(fname);
fprintf('saved %s (%.1f MB, %d slow pts, %d waveform pts)\n', ...
        fname, d.bytes/1e6, numel(R.t), numel(R.wave.t));
end

function M = pick(D, k)
M = squeeze(D);
if size(M,1) == 3 && size(M,2) ~= 3, M = M.'; end
M = M(k,:);
end
