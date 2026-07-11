function export_battery_csv(base)
% Export battery pack Time + Current + Voltage + SOC for P1..P10 to one CSV
% per profile (P<n>_battery.csv) in the batch folder. Voltage/SOC are
% interpolated onto the current time base if their sample counts differ.
if nargin < 1 || isempty(base)
    base = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
end

d = dir(fullfile(base,'P*_MTOW*'));
d = d([d.isdir]);
for i = 1:numel(d)
    name = d(i).name;
    tok = regexp(name,'^P(\d+)_','tokens','once');
    if isempty(tok), continue; end
    pn = str2double(tok{1});
    if pn < 1 || pn > 10, continue; end
    matf = fullfile(base, name, [name '.mat']);
    if ~exist(matf,'file'); fprintf('skip (no mat): %s\n', name); continue; end

    S = load(matf,'outTuned');
    B = S.outTuned.Battery_Data.Batt;
    tI = double(B.Current__A_.Time(:)); I = double(B.Current__A_.Data(:));
    tV = double(B.Voltage__V_.Time(:)); V = double(B.Voltage__V_.Data(:));
    tS = double(B.SOC____.Time(:));     SOC = double(B.SOC____.Data(:));

    % align V, SOC onto the current time base if lengths differ
    if numel(tV) ~= numel(tI) || any(tV ~= tI), V   = interp1(tV, V,   tI, 'linear','extrap'); end
    if numel(tS) ~= numel(tI) || any(tS ~= tI), SOC = interp1(tS, SOC, tI, 'linear','extrap'); end

    out = fullfile(base, sprintf('P%d_battery.csv', pn));
    writetable(table(tI, I, V, SOC, 'VariableNames', {'Time_s','Current_A','Voltage_V','SOC_pct'}), out);
    fprintf('wrote %s  (%d rows, %s)\n', out, numel(tI), name);
    clear S B tI I tV V tS SOC;
end
fprintf('DONE.\n');
end
