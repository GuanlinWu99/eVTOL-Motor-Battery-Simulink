function diag_drop_timealigned()
% Resolve the terminal-voltage gap: at matched times, compare OCV(SOC),
% terminal V, current, and implied voltage drop for SA-88 vs LG (P8).

base = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
f_sa = fullfile(base,'P8_MTOW_7000_Wind_7_Temp_45','P8_MTOW_7000_Wind_7_Temp_45.mat');
f_lg = fullfile(base,'origin_P8_MTOW_7000_Wind_7_Temp_45.mat');
outdir = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare';

sa = grab(f_sa); lg = grab(f_lg);

report('SA-88', sa);
report('LG Chem', lg);

% time-series of OCV vs terminal V
f = figure('Color','w','Position',[60 60 1100 460]);
subplot(1,2,1);
plot(sa.t, sa.OCV,'--','Color',[0 0.45 0.74],'LineWidth',1.5); hold on; grid on;
plot(sa.t, sa.V,  '-','Color',[0 0.45 0.74],'LineWidth',1.8);
plot(lg.t, lg.OCV,'--','Color',[0.85 0.33 0.10],'LineWidth',1.5);
plot(lg.t, lg.V,  '-','Color',[0.85 0.33 0.10],'LineWidth',1.8);
xlabel('Time [s]'); ylabel('V'); title('OCV (dashed) vs terminal V (solid)');
legend('SA-88 OCV','SA-88 V','LG OCV','LG V','Location','best'); set(gca,'FontSize',11);

subplot(1,2,2);
plot(sa.t, sa.OCV-sa.V,'-','Color',[0 0.45 0.74],'LineWidth',1.8); hold on; grid on;
plot(lg.t, lg.OCV-lg.V,'-','Color',[0.85 0.33 0.10],'LineWidth',1.8);
xlabel('Time [s]'); ylabel('OCV - V_{terminal} [V]'); title('Voltage drop under load');
legend('SA-88 drop','LG drop','Location','best'); set(gca,'FontSize',11);
exportgraphics(f, fullfile(outdir,'diag_drop_timealigned.png'),'Resolution',200);
fprintf('Saved: %s\n', fullfile(outdir,'diag_drop_timealigned.png'));
end

function s = grab(f)
    S = load(f,'outTuned','HEV_Param');
    B = S.outTuned.Battery_Data.Batt; H = S.HEV_Param;
    s.t   = double(B.Voltage__V_.Time(:));
    s.V   = double(B.Voltage__V_.Data(:));
    s.I   = double(B.Current__A_.Data(:));
    s.SOC = double(B.SOC____.Data(:));
    if max(s.SOC) <= 1.5, socf = s.SOC; else, socf = s.SOC/100; end
    grid_ = H.Battery_Cell.SOC(:); emo = H.Battery_Cell.Emo(:);
    s.OCV = interp1(grid_, emo, socf, 'linear','extrap');
end

function report(name, s)
    % cruise = middle 40% of mission, lowest-current quartile
    n = numel(s.t); win = round(n*0.30):round(n*0.70);
    Iw = abs(s.I(win));
    qlo = Iw <= quantile(Iw,0.5);
    idx = win(qlo);
    fprintf('\n==== %s : cruise window (mid-mission, low-current half) ====\n', name);
    fprintf('  SOC        ~ %.1f %%\n', mean(s.SOC(idx)));
    fprintf('  OCV(SOC)   = %.1f V\n', mean(s.OCV(idx)));
    fprintf('  terminal V = %.1f V\n', mean(s.V(idx)));
    fprintf('  current    = %.1f A (mean |I| in window)\n', mean(abs(s.I(idx))));
    fprintf('  drop OCV-V = %.1f V\n', mean(s.OCV(idx) - s.V(idx)));
    R = mean(s.OCV(idx)-s.V(idx)) / max(mean(abs(s.I(idx))),1e-9);
    fprintf('  implied R  = %.1f mOhm (drop / current)\n', R*1000);
end
