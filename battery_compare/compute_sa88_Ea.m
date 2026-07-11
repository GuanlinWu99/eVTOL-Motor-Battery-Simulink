function compute_sa88_Ea()
% Derive the Arrhenius activation energy Ea for the SA-88 1-RC parameters
% from the 25 degC and 30 degC sheets (3C column), per SOC, for R0/R1/C1.
%
%   R(T) = R_ref * exp[ (Ea/Rg)*(1/T - 1/T_ref) ]   (resistance: drops as T up)
%   C(T) = C_ref * exp[ -(Ea/Rg)*(1/T - 1/T_ref) ]  (capacitance: rises as T up)
%
%   Ea_R = Rg * ln(R_lo/R_hi) / (1/T_lo - 1/T_hi)     (T_lo=25, T_hi=30)
%   Ea_C = Rg * ln(C_hi/C_lo) / (1/T_lo - 1/T_hi)

folder = ['E:\file\matlab\eVTOL_Simulator_260511_v14\eVTOL_Simulator_260511_v14\' ...
          'kiat_battery_project\Update on SA-88 Battery Model and 1-RC Parameters (ABMS Project)'];
Rg    = 8.314462618;       % J/mol/K
crate = '3C';
Tlo = 25; Thi = 30;
TloK = Tlo + 273.15; ThiK = Thi + 273.15;
invD = 1/TloK - 1/ThiK;    % > 0

[s25_R0, R0_25] = readsheet(folder, 25, 'R0_by_SOC', crate);
[s30_R0, R0_30] = readsheet(folder, 30, 'R0_by_SOC', crate);
[s25_R1, R1_25] = readsheet(folder, 25, 'R1_by_SOC', crate);
[s30_R1, R1_30] = readsheet(folder, 30, 'R1_by_SOC', crate);
[s25_C1, C1_25] = readsheet(folder, 25, 'C1_by_SOC', crate);
[s30_C1, C1_30] = readsheet(folder, 30, 'C1_by_SOC', crate);

% align on common SOC grid
[soc, i25, i30] = intersect(s25_R0, s30_R0);
R0_25=R0_25(i25); R0_30=R0_30(i30);
[~, j25, j30] = intersect(s25_R1, s30_R1); R1_25=R1_25(j25); R1_30=R1_30(j30);
[~, k25, k30] = intersect(s25_C1, s30_C1); C1_25=C1_25(k25); C1_30=C1_30(k30);

Ea_R0 = Rg * log(R0_25 ./ R0_30) / invD / 1000;   % kJ/mol
Ea_R1 = Rg * log(R1_25 ./ R1_30) / invD / 1000;
Ea_C1 = Rg * log(C1_30 ./ C1_25) / invD / 1000;    % note hi/lo for C

fprintf('\n================ SA-88 Arrhenius Ea  (25 vs 30 degC, %s) ================\n', crate);
fprintf('%5s | %10s %10s %10s\n','SOC%','Ea_R0','Ea_R1','Ea_C1');
fprintf('      | %10s %10s %10s  [kJ/mol]\n','','','');
for n = 1:numel(soc)
    fprintf('%5g | %10.3f %10.3f %10.3f\n', soc(n), Ea_R0(n), Ea_R1(n), Ea_C1(n));
end
fprintf('-------------------------------------------------------------------\n');
rpt = @(name,v) fprintf('%-6s mean=%7.3f  median=%7.3f  std=%6.3f  [kJ/mol]\n', ...
                        name, mean(v,'omitnan'), median(v,'omitnan'), std(v,'omitnan'));
rpt('Ea_R0', Ea_R0);
rpt('Ea_R1', Ea_R1);
rpt('Ea_C1', Ea_C1);
allEa = [Ea_R0; Ea_R1];
fprintf('-------------------------------------------------------------------\n');
fprintf('R0+R1 combined : mean=%7.3f  median=%7.3f  [kJ/mol]\n', mean(allEa,'omitnan'), median(allEa,'omitnan'));
fprintf('Currently hard-coded in loader : 32.3667 kJ/mol (borrowed from LG-Chem)\n');
fprintf('===================================================================\n');

% plot Ea vs SOC
f = figure('Color','w','Position',[80 80 760 520]);
plot(soc, Ea_R0,'-o','LineWidth',2); hold on; grid on;
plot(soc, Ea_R1,'-s','LineWidth',2);
plot(soc, Ea_C1,'-^','LineWidth',2);
yline(32.3667,'--k','LineWidth',1.5);
xlabel('SOC [%]'); ylabel('E_a [kJ/mol]');
title('SA-88 Arrhenius E_a from 25\circC & 30\circC (3C)');
legend('E_a from R_0','E_a from R_1','E_a from C_1','hard-coded 32.367','Location','best');
set(gca,'FontSize',13);
out = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare\sa88_Ea_25_30.png';
exportgraphics(f, out, 'Resolution',200);
fprintf('Saved figure: %s\n', out);
end

function [soc, val] = readsheet(folder, temp, sheet, crate)
    fname = sprintf('SOC_R0_R1_C1_CRate_Comparison_%d''C.xlsx', temp);
    f = fullfile(folder, fname);
    C = readcell(f, 'Sheet', sheet);
    hdr = string(C(1,:));
    col = find(strcmpi(strtrim(hdr), crate), 1);
    if isempty(col)
        error('C-rate %s not found in %s/%s. Header: %s', crate, fname, sheet, strjoin(hdr,', '));
    end
    raw = C(2:end, [1 col]);
    soc = nan(size(raw,1),1); val = nan(size(raw,1),1);
    for r = 1:size(raw,1)
        a = raw{r,1}; b = raw{r,2};
        if isnumeric(a) && isnumeric(b) && ~isempty(a) && ~isempty(b) && ~ismissing(a) && ~ismissing(b)
            soc(r) = a; val(r) = b;
        end
    end
    keep = ~isnan(soc) & ~isnan(val);
    soc = soc(keep); val = val(keep);
    [soc, o] = sort(soc); val = val(o);
end
