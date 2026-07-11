function assess_Ea_impact()
% Quantify the impact of changing the Arrhenius Ea (32.3667 -> SA-88 fitted)
% on the battery and the simulation, using the actual completed SA-88 results.

Rg = 8.314462618;
Ea_old = 32366.7;          % J/mol (hard-coded, from LG-Chem)
% SA-88 fitted (global, SOC>=30%): R0~1.2, R1~3.3, C1~3.4 kJ/mol -> use ~2.5 avg for R
Ea_new = 2500;             % J/mol (representative SA-88 value for R)

% ---- 1) Parameter-level scaling factors --------------------------------
fprintf('\n========== (1) Parameter scaling k_R = R_used / R_reference ==========\n');
cases = {  % name, T_target, T_ref
    '20C (P1,3,7,10): ref 25C', 20, 25;
    '45C (P2,5,6,8,9): ref 40C', 45, 40};
fprintf('%-26s | %10s %10s | %s\n','case','k_R(old)','k_R(new)','R change old->new');
for c = 1:size(cases,1)
    Tt = cases{c,2}+273.15; Tr = cases{c,3}+273.15;
    d = 1/Tt - 1/Tr;
    kR_old = exp((Ea_old/Rg)*d);
    kR_new = exp((Ea_new/Rg)*d);
    fprintf('%-26s | %10.4f %10.4f | %+5.1f%%  (R_new/R_old=%.3f)\n', ...
        cases{c,1}, kR_old, kR_new, (kR_new/kR_old-1)*100, kR_new/kR_old);
end
fprintf('25C (P4,P11): k=1 both -> NO change\n');

% ---- 2) System-level: real operating envelope + sag impact -------------
base = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
probes = {
    'P1_MTOW_5600_Wind_0_Temp_20',  20;
    'P2_MTOW_5600_Wind_7_Temp_45',  45;
    'P4_MTOW_7000_Wind_0_Temp_25',  25};

fprintf('\n========== (2) Real operating envelope (from completed SA-88 runs) ==========\n');
fprintf('%-8s %4s | %7s %7s | %7s %7s | %7s %7s | %8s\n', ...
    'profile','T','I_mean','I_pk','V_mean','V_min','SOC0','SOCend','Reff_mOhm');
for i = 1:size(probes,1)
    f = fullfile(base, probes{i,1}, [probes{i,1} '.mat']);
    S = load(f, 'outTuned', 'HEV_Param');
    B = S.outTuned.Battery_Data.Batt;
    I  = double(B.Current__A_.Data(:));
    V  = double(B.Voltage__V_.Data(:));
    SOC= double(B.SOC____.Data(:));
    % effective pack resistance estimate: regress V on I (V = Voc - I*Reff),
    % using high-current samples so OCV drift is negligible over the window
    n = numel(I);
    w = max(1, round(n*0.02)) : min(n, round(n*0.20));   % early aggressive segment
    Iw = I(w); Vw = V(w);
    A = [ones(numel(Iw),1), -Iw];
    cc = A\Vw;                 % cc(2) = Reff
    Reff = max(cc(2),0);
    fprintf('%-8s %4d | %7.1f %7.1f | %7.1f %7.1f | %7.3f %7.3f | %8.2f\n', ...
        probes{i,1}(1:min(8,end)), probes{i,2}, mean(abs(I)), max(abs(I)), ...
        mean(V), min(V), SOC(1), SOC(end), Reff*1000);
end

% ---- 3) Translate 20% R change into voltage / SOC / heat ---------------
fprintf('\n========== (3) Impact of ~20%% R change on outputs ==========\n');
fprintf('Use P2 (45C) as worst case: load its envelope...\n');
S = load(fullfile(base,'P2_MTOW_5600_Wind_7_Temp_45','P2_MTOW_5600_Wind_7_Temp_45.mat'),'outTuned');
B = S.outTuned.Battery_Data.Batt;
I = double(B.Current__A_.Data(:)); V = double(B.Voltage__V_.Data(:));
n=numel(I); w = max(1,round(n*0.02)):min(n,round(n*0.20));
A=[ones(numel(w),1),-I(w)]; cc=A\V(w); Reff=max(cc(2),0);
Ipk = max(abs(I)); Imean = mean(abs(I)); Vmean = mean(V);
dR = 0.20;                                  % 45C: R ~20% higher with fix
fprintf('  effective pack R        : %.2f mOhm\n', Reff*1000);
fprintf('  peak / mean current     : %.0f / %.0f A\n', Ipk, Imean);
fprintf('  IR sag @ peak (now)      : %.2f V  (%.2f%% of %.0f V)\n', Ipk*Reff, Ipk*Reff/Vmean*100, Vmean);
fprintf('  extra sag from +20%% R    : %.2f V  (%.3f%% of pack V)\n', Ipk*Reff*dR, Ipk*Reff*dR/Vmean*100);
fprintf('  resistive heat change    : +20%% of I^2R term (peak %.2f kW -> +%.2f kW)\n', ...
        Ipk^2*Reff/1000, Ipk^2*Reff*dR/1000);
fprintf('  -> SOC/energy: voltage moves ~%.2f%%, so current(for same power) and\n', Ipk*Reff*dR/Vmean*100);
fprintf('     SOC trajectory move by the same tiny fraction (sub-0.5%%).\n');
fprintf('============================================================\n');
end
