function compare_pack_P8()
% Battery PACK electrical performance: SA-88 (1-RC) vs origin/LG-Chem (2-RC)
% for Profile 8 (MTOW 7000, Wind 7 m/s, 45 degC).

base   = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
f_sa   = fullfile(base,'P8_MTOW_7000_Wind_7_Temp_45','P8_MTOW_7000_Wind_7_Temp_45.mat');
f_lg   = fullfile(base,'origin_P8_MTOW_7000_Wind_7_Temp_45.mat');
outdir = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare';

sa = grab(f_sa);
lg = grab(f_lg);

col_sa=[0 0.45 0.74]; col_lg=[0.85 0.33 0.10]; LW=1.8; FS=12;

f = figure('Color','w','Position',[60 60 1150 820]);

subplot(2,2,1);
plot(sa.t, sa.V,'-','Color',col_sa,'LineWidth',LW); hold on; grid on;
plot(lg.t, lg.V,'-','Color',col_lg,'LineWidth',LW);
xlabel('Time [s]'); ylabel('Pack Voltage [V]'); title('Pack Voltage');
legend('SA-88 (1-RC)','LG Chem (2-RC)','Location','best'); set(gca,'FontSize',FS);

subplot(2,2,2);
plot(sa.t, sa.I,'-','Color',col_sa,'LineWidth',LW); hold on; grid on;
plot(lg.t, lg.I,'-','Color',col_lg,'LineWidth',LW);
xlabel('Time [s]'); ylabel('Pack Current [A]'); title('Pack Current');
legend('SA-88','LG Chem','Location','best'); set(gca,'FontSize',FS);

subplot(2,2,3);
plot(sa.t, sa.P,'-','Color',col_sa,'LineWidth',LW); hold on; grid on;
plot(lg.t, lg.P,'-','Color',col_lg,'LineWidth',LW);
xlabel('Time [s]'); ylabel('Pack Power [kW]'); title('Pack Power  (V \times I)');
legend('SA-88','LG Chem','Location','best'); set(gca,'FontSize',FS);

subplot(2,2,4);
plot(sa.t, sa.SOC,'-','Color',col_sa,'LineWidth',LW); hold on; grid on;
plot(lg.t, lg.SOC,'-','Color',col_lg,'LineWidth',LW);
xlabel('Time [s]'); ylabel('SOC [%]'); title('State of Charge');
legend('SA-88','LG Chem','Location','best'); set(gca,'FontSize',FS);

sgtitle('Battery Pack Electrical Performance — P8 (MTOW 7000, Wind 7 m/s, 45\circC)', ...
        'FontSize',14,'FontWeight','bold');
out = fullfile(outdir,'compare_pack_P8.png');
exportgraphics(f, out, 'Resolution',200);
fprintf('SA-88 : V %.1f..%.1f V, I_pk %.0f A, SOC %.1f->%.1f %%\n', min(sa.V),max(sa.V),max(abs(sa.I)),sa.SOC(1),sa.SOC(end));
fprintf('LGChem: V %.1f..%.1f V, I_pk %.0f A, SOC %.1f->%.1f %%\n', min(lg.V),max(lg.V),max(abs(lg.I)),lg.SOC(1),lg.SOC(end));
fprintf('Saved: %s\n', out);
end

function s = grab(f)
    S = load(f,'outTuned');
    B = S.outTuned.Battery_Data.Batt;
    s.t   = double(B.Voltage__V_.Time(:));
    s.V   = double(B.Voltage__V_.Data(:));
    s.I   = double(B.Current__A_.Data(:));
    s.SOC = double(B.SOC____.Data(:));
    % SOC may be a fraction (0..1) or percent; normalize to percent for plot
    if max(s.SOC) <= 1.5, s.SOC = s.SOC*100; end
    s.P   = s.V .* s.I / 1000;   % kW
end
