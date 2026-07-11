function diag_P2_windy()
% Why does LG-Chem P2 (7 m/s wind) terminate early while SA-88 P2 completes?
f_sa = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409\P2_MTOW_5600_Wind_7_Temp_45\P2_MTOW_5600_Wind_7_Temp_45.mat';
f_lg = 'E:\file\matlab\eVTOL_Simulator_260511_v15_origin\kiat_battery_project\output\lgchem_batch_20260613_150544\P2_MTOW_5600_Wind_7_Temp_45\P2_MTOW_5600_Wind_7_Temp_45.mat';
look('SA-88', f_sa);
look('LG Chem', f_lg);
end

function look(name, f)
    S = load(f,'outTuned');
    B = S.outTuned.Battery_Data.Batt;
    t = double(B.Voltage__V_.Time(:));
    V = double(B.Voltage__V_.Data(:));
    I = double(B.Current__A_.Data(:));
    SOC = double(B.SOC____.Data(:));
    fprintf('\n==== %s  P2 (7 m/s wind, 45C) ====\n', name);
    fprintf('  sim end time   : %.1f s   (samples=%d)\n', t(end), numel(t));
    fprintf('  V  min/mean/max: %.1f / %.1f / %.1f V\n', min(V), mean(V), max(V));
    fprintf('  I  mean/peak   : %.1f / %.1f A\n', mean(abs(I)), max(abs(I)));
    fprintf('  SOC start/end/min: %.2f / %.2f / %.2f\n', SOC(1), SOC(end), min(SOC));
    % flight-mode transitions (enum may not reconstruct -> try numeric)
    try
        fm = S.outTuned.Flight_Mode.Data;
        fmn = double(fm(:));
        ch = sum(fmn(1:end-1) ~= fmn(2:end));
        fprintf('  flight-mode changes: %d  (final mode=%g)\n', ch, fmn(end));
    catch e
        fprintf('  flight-mode: <could not read: %s>\n', e.message);
    end
    % look at the last 5%: is it a clean landing or a divergence?
    k = max(1,round(numel(t)*0.95)):numel(t);
    fprintf('  last 5%% : V %.0f..%.0f, |I| up to %.0f A, SOC %.2f->%.2f\n', ...
        min(V(k)), max(V(k)), max(abs(I(k))), SOC(k(1)), SOC(end));
end
