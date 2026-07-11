function diag_voltage_offset()
% Find the source of the ~60 V steady terminal-voltage offset (SA-88 vs LG)
% by comparing the PACK OCV (Emo) vs SOC mapping each model actually used.
% HEV_Param is saved in each result .mat, so load only that (fast).

base = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
f_sa = fullfile(base,'P8_MTOW_7000_Wind_7_Temp_45','P8_MTOW_7000_Wind_7_Temp_45.mat');
f_lg = fullfile(base,'origin_P8_MTOW_7000_Wind_7_Temp_45.mat');
outdir = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare';

A = load(f_sa,'HEV_Param'); HsA = A.HEV_Param;
B = load(f_lg,'HEV_Param'); HlG = B.HEV_Param;

sa.soc = HsA.Battery_Cell.SOC(:);  sa.emo = HsA.Battery_Cell.Emo(:);
lg.soc = HlG.Battery_Cell.SOC(:);  lg.emo = HlG.Battery_Cell.Emo(:);

fprintf('\n--- Pack/cell config ---\n');
fprintf('            %12s %12s\n','SA-88','LG Chem');
fprintf('Ns          %12d %12d\n', HsA.Ns, HlG.Ns);
fprintf('Np          %12d %12d\n', HsA.Np, HlG.Np);
fprintf('SOC range   %12s %12s\n', sprintf('%.2f..%.2f',min(sa.soc),max(sa.soc)), sprintf('%.2f..%.2f',min(lg.soc),max(lg.soc)));
fprintf('Emo range V %12s %12s\n', sprintf('%.0f..%.0f',min(sa.emo),max(sa.emo)), sprintf('%.0f..%.0f',min(lg.emo),max(lg.emo)));
try fprintf('Pack_V      %12.0f %12.0f\n', HsA.Battery_Pack_Voltage, HlG.Battery_Pack_Voltage); catch, end

fprintf('\n--- PACK OCV (Emo) at the same actual SOC ---\n');
fprintf('%6s %12s %12s %10s\n','SOC','SA-88 Emo','LG Emo','diff [V]');
for s = [0.95 0.90 0.88 0.85 0.80 0.70 0.50 0.30]
    es = interp1(sa.soc, sa.emo, s, 'linear','extrap');
    el = interp1(lg.soc, lg.emo, s, 'linear','extrap');
    fprintf('%6.2f %12.1f %12.1f %10.1f\n', s, es, el, es-el);
end

% Per-cell OCV (Emo/Ns) to see if it's a per-cell mapping difference
fprintf('\n--- per-CELL OCV (Emo/Ns) ---\n');
fprintf('%6s %12s %12s %10s\n','SOC','SA-88 [V]','LG [V]','diff [mV]');
for s = [0.90 0.88 0.50]
    es = interp1(sa.soc, sa.emo, s,'linear','extrap')/HsA.Ns;
    el = interp1(lg.soc, lg.emo, s,'linear','extrap')/HlG.Ns;
    fprintf('%6.2f %12.4f %12.4f %10.1f\n', s, es, el, (es-el)*1000);
end

% plot
f = figure('Color','w','Position',[80 80 820 560]);
plot(lg.soc*100, lg.emo,'-','Color',[0.85 0.33 0.10],'LineWidth',2.2); hold on; grid on;
plot(sa.soc*100, sa.emo,'-','Color',[0 0.45 0.74],'LineWidth',2.2);
xlabel('SOC [%]'); ylabel('Pack OCV  E_{mo}  [V]'); title('Pack open-circuit voltage map (as used in P8)');
legend('LG Chem (2-RC)','SA-88 (1-RC)','Location','southeast'); set(gca,'FontSize',13);
exportgraphics(f, fullfile(outdir,'diag_Emo_map.png'),'Resolution',200);
fprintf('\nSaved: %s\n', fullfile(outdir,'diag_Emo_map.png'));
end
