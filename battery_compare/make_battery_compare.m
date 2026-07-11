function make_battery_compare()
% Generate LG Chem (2-RC) vs Samsung SA-88 (1-RC) comparison figures + numbers
% Uses the 25 degC reference datasets from both projects (cell-level params).

lg_dir   = 'E:\file\matlab\eVTOL_Simulator_260511_v15_origin\kiat_battery_project\src\FullScale\parameters';
sa_dir   = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\src\FullScale\parameters';
out_dir  = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare';
if ~exist(out_dir,'dir'); mkdir(out_dir); end

LG = load(fullfile(lg_dir,'LG_Chem_INR_18650_25_Degree.mat'));
SA = load(fullfile(sa_dir,'SA88_25_Degree.mat'));

% --- LG Chem (2-RC), SOC_bp in percent (0..100), OCV sliced 15:101 ---
lg.soc = LG.SOC_bp(:)';            if max(lg.soc)>1, lg.soc = lg.soc/100; end
lg.ocv = LG.OCV_table(15:101); lg.ocv = lg.ocv(:)';
lg.R0  = LG.Rs_table(:)';
lg.R1  = LG.R1_table(:)';
lg.R2  = LG.R2_table(:)';
lg.C1  = LG.C1_table(:)';
lg.C2  = LG.C2_table(:)';

% --- SA-88 (1-RC), SOC_bp 0..95 percent ---
sa.soc = SA.SOC_bp(:)';            if max(sa.soc)>1, sa.soc = sa.soc/100; end
sa.ocv = SA.OCV_table(:)';
sa.R0  = SA.Rs_table(:)';
sa.R1  = SA.R1_table(:)';
sa.C1  = SA.C1_table(:)';

% total DC internal resistance (drives voltage sag + heat)
lg.Rtot = lg.R0 + lg.R1 + lg.R2;   % 2-RC
sa.Rtot = sa.R0 + sa.R1;           % 1-RC

% ===================== FIGURE 1: 2x2 parameter panels =====================
f1 = figure('Color','w','Position',[80 80 1100 800]);
LWlg=2.2; LWsa=2.2; FS=13;
col_lg=[0.85 0.33 0.10]; col_sa=[0 0.45 0.74];

subplot(2,2,1);
plot(lg.soc*100, lg.ocv,'-','Color',col_lg,'LineWidth',LWlg); hold on; grid on;
plot(sa.soc*100, sa.ocv,'-','Color',col_sa,'LineWidth',LWsa);
xlabel('SOC [%]'); ylabel('OCV [V]'); title('Open-Circuit Voltage');
legend('LG Chem (2-RC)','SA-88 (1-RC)','Location','southeast'); set(gca,'FontSize',FS);

subplot(2,2,2);
plot(lg.soc*100, lg.R0*1000,'-','Color',col_lg,'LineWidth',LWlg); hold on; grid on;
plot(sa.soc*100, sa.R0*1000,'-','Color',col_sa,'LineWidth',LWsa);
xlabel('SOC [%]'); ylabel('R_0 [m\Omega]'); title('Ohmic Resistance R_0');
legend('LG Chem','SA-88','Location','northeast'); set(gca,'FontSize',FS);

subplot(2,2,3);
semilogy(lg.soc*100, lg.R1*1000,'-','Color',col_lg,'LineWidth',LWlg); hold on; grid on;
semilogy(lg.soc*100, lg.R2*1000,'--','Color',col_lg,'LineWidth',1.6);
semilogy(sa.soc*100, sa.R1*1000,'-','Color',col_sa,'LineWidth',LWsa);
xlabel('SOC [%]'); ylabel('R [m\Omega]  (log)'); title('Polarization Resistance');
legend('LG R_1 (slow)','LG R_2 (fast)','SA-88 R_1','Location','east'); set(gca,'FontSize',FS);

subplot(2,2,4);
semilogy(lg.soc*100, lg.C1,'-','Color',col_lg,'LineWidth',LWlg); hold on; grid on;
semilogy(lg.soc*100, lg.C2,'--','Color',col_lg,'LineWidth',1.6);
semilogy(sa.soc*100, sa.C1,'-','Color',col_sa,'LineWidth',LWsa);
xlabel('SOC [%]'); ylabel('C [F]  (log)'); title('Polarization Capacitance');
legend('LG C_1','LG C_2','SA-88 C_1','Location','east'); set(gca,'FontSize',FS);

sgtitle('LG Chem (2-RC)  vs  Samsung SA-88 (1-RC)  @ 25\circC  (cell level)','FontSize',15,'FontWeight','bold');
exportgraphics(f1, fullfile(out_dir,'battery_compare_params.png'),'Resolution',200);

% ===================== FIGURE 2: total DC resistance =====================
f2 = figure('Color','w','Position',[80 80 760 540]);
semilogy(lg.soc*100, lg.Rtot*1000,'-','Color',col_lg,'LineWidth',2.6); hold on; grid on;
semilogy(sa.soc*100, sa.Rtot*1000,'-','Color',col_sa,'LineWidth',2.6);
xlabel('SOC [%]'); ylabel('Total DC internal resistance R_0+R_1(+R_2) [m\Omega]  (log)');
title('Cell DC Internal Resistance vs SOC  @ 25\circC');
legend('LG Chem (R_0+R_1+R_2)','SA-88 (R_0+R_1)','Location','northeast');
set(gca,'FontSize',13);
exportgraphics(f2, fullfile(out_dir,'battery_compare_Rtotal.png'),'Resolution',200);

% ===================== numeric summary (for the table) =====================
function v = at50(soc,y), v = interp1(soc,y,0.5,'linear','extrap'); end
fprintf('\n================ NUMBERS @ SOC=50%%, 25 degC (cell) ================\n');
fprintf('%-22s %12s %12s\n','quantity','LG Chem 2RC','SA-88 1RC');
fprintf('%-22s %12.2f %12.2f\n','OCV [V]',        at50(lg.soc,lg.ocv),  at50(sa.soc,sa.ocv));
fprintf('%-22s %12.2f %12.2f\n','R0 [mOhm]',      at50(lg.soc,lg.R0)*1e3, at50(sa.soc,sa.R0)*1e3);
fprintf('%-22s %12.2f %12.2f\n','R1 [mOhm]',      at50(lg.soc,lg.R1)*1e3, at50(sa.soc,sa.R1)*1e3);
fprintf('%-22s %12.2f %12s\n','R2 [mOhm]',        at50(lg.soc,lg.R2)*1e3, '-- (none)');
fprintf('%-22s %12.2f %12.2f\n','Rtot [mOhm]',    at50(lg.soc,lg.Rtot)*1e3, at50(sa.soc,sa.Rtot)*1e3);
fprintf('%-22s %12.2f %12.2f\n','C1 [F]',         at50(lg.soc,lg.C1),   at50(sa.soc,sa.C1));
fprintf('%-22s %12.2f %12s\n','C2 [F]',           at50(lg.soc,lg.C2),   '-- (none)');
fprintf('%-22s %12d %12d\n','# SOC points',       numel(lg.soc),        numel(sa.soc));
fprintf('%-22s %12s %12s\n','SOC range [%]', sprintf('%g..%g',round(min(lg.soc)*100),round(max(lg.soc)*100)), ...
                                              sprintf('%g..%g',round(min(sa.soc)*100),round(max(sa.soc)*100)));
fprintf('===================================================================\n');
fprintf('Saved figures to: %s\n', out_dir);
end
