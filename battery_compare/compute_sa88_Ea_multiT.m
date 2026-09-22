function compute_sa88_Ea_multiT()
% Robust Arrhenius Ea for SA-88 using ALL available temperatures
% (25/30/35/40 degC, 3C). Per SOC: linear fit ln(R) vs 1/T, slope = Ea/Rg.
% Also a single global Ea via regression-through-origin of
%   y = ln(R(soc,T)/R(soc,25))  vs  x = (1/T - 1/T25)   pooled over all points.

folder = ['E:\file\matlab\eVTOL_Simulator_260511_v14\eVTOL_Simulator_260511_v14\' ...
          'kiat_battery_project\Update on SA-88 Battery Model and 1-RC Parameters (ABMS Project)'];
Rg    = 8.314462618;
crate = '3C';
Temps = [25 30 35 40];
TK    = Temps + 273.15;
SOC_MIN = 30;   % low-SOC SA-88 data is very noisy; restrict robust fit to >=30%

names = {'R0_by_SOC','R1_by_SOC','C1_by_SOC'};
labels= {'R0','R1','C1'};
signC = [+1 +1 -1];   % C scales with -Ea

fprintf('\n=========== SA-88 Arrhenius Ea  (multi-T fit 25/30/35/40, %s) ===========\n', crate);

f = figure('Color','w','Position',[60 60 1150 420]);
for p = 1:3
    % read all temps, align SOC across them
    socs = cell(1,numel(Temps)); vals = cell(1,numel(Temps));
    for t = 1:numel(Temps)
        [socs{t}, vals{t}] = readsheet(folder, Temps(t), names{p}, crate);
    end
    soc = socs{1};
    for t = 2:numel(Temps), soc = intersect(soc, socs{t}); end
    V = zeros(numel(soc), numel(Temps));
    for t = 1:numel(Temps)
        [~, ia] = ismember(soc, socs{t});
        V(:,t) = vals{t}(ia);
    end

    % per-SOC slope of ln(V) vs 1/T  ->  Ea = sign * Rg * slope
    invT = 1./TK;
    EaSOC = nan(numel(soc),1);
    for n = 1:numel(soc)
        y = log(V(n,:))';
        A = [ones(numel(TK),1), invT'];
        c = A\y;                       % c(2) = slope
        EaSOC(n) = signC(p) * Rg * c(2) / 1000;   % kJ/mol
    end

    % global Ea (regression through origin, ref=25C), restricted to clean SOC
    mask = soc >= SOC_MIN;
    x = []; y = [];
    iref = 1; % 25C
    for t = 1:numel(Temps)
        if t==iref, continue; end
        xx = (1/TK(t) - 1/TK(iref)) * ones(sum(mask),1);
        yy = log(V(mask,t) ./ V(mask,iref));
        x = [x; xx]; y = [y; yy]; %#ok<AGROW>
    end
    slope = (x'*y)/(x'*x);
    Ea_global = signC(p) * Rg * slope / 1000;

    Ea_clean = EaSOC(mask);
    fprintf('%-3s : per-SOC(>=%d%%) mean=%6.2f median=%6.2f  | GLOBAL fit=%6.2f kJ/mol\n', ...
        labels{p}, SOC_MIN, mean(Ea_clean,'omitnan'), median(Ea_clean,'omitnan'), Ea_global);

    subplot(1,3,p);
    plot(soc, EaSOC,'-o','LineWidth',1.8); hold on; grid on;
    yline(Ea_global,'-','LineWidth',2,'Color',[0 0.5 0],'Label',sprintf('global %.1f',Ea_global));
    yline(32.3667,'--k','LineWidth',1.3,'Label','LG 32.37');
    xlabel('SOC [%]'); ylabel('E_a [kJ/mol]'); title(['E_a from ' labels{p}]);
    set(gca,'FontSize',11); ylim([-40 80]);
end
sgtitle('SA-88 Arrhenius E_a (fit over 25/30/35/40\circC, 3C)  — low-SOC noisy','FontSize',13,'FontWeight','bold');
out = 'E:\file\chrome\eVTOL_Simulator_260511_v15\kiat_battery_project\battery_compare\sa88_Ea_multiT.png';
exportgraphics(f, out, 'Resolution',200);
fprintf('-------------------------------------------------------------------\n');
fprintf('Hard-coded in loader: 32.3667 kJ/mol (from LG-Chem). Saved: %s\n', out);
fprintf('===================================================================\n');
end

function [soc, val] = readsheet(folder, temp, sheet, crate)
    fname = sprintf('SOC_R0_R1_C1_CRate_Comparison_%d''C.xlsx', temp);
    C = readcell(fullfile(folder, fname), 'Sheet', sheet);
    hdr = string(C(1,:));
    col = find(strcmpi(strtrim(hdr), crate), 1);
    if isempty(col), error('C-rate %s not found in %s/%s', crate, fname, sheet); end
    raw = C(2:end, [1 col]);
    soc = nan(size(raw,1),1); val = nan(size(raw,1),1);
    for r = 1:size(raw,1)
        a = raw{r,1}; b = raw{r,2};
        if isnumeric(a) && isnumeric(b) && ~isempty(a) && ~isempty(b) && ~ismissing(a) && ~ismissing(b)
            soc(r) = a; val(r) = b;
        end
    end
    keep = ~isnan(soc) & ~isnan(val); soc = soc(keep); val = val(keep);
    [soc, o] = sort(soc); val = val(o);
end
