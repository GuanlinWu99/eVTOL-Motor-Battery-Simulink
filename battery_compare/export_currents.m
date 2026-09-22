function export_currents()
% Export battery PACK current (time + current) for P1..P10 to CSV files,
% one CSV per profile, saved into the batch folder.
base = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';

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
    C = S.outTuned.Battery_Data.Batt.Current__A_;
    T = double(C.Time(:));
    I = double(C.Data(:));

    out = fullfile(base, sprintf('P%d_current.csv', pn));
    writetable(table(T, I, 'VariableNames', {'Time_s','Current_A'}), out);
    fprintf('wrote %s  (%d rows, %s)\n', out, numel(T), name);
    clear S C T I;
end
fprintf('DONE.\n');
end
