%% Fixed-wing flight of two saved runs: altitude, airspeed and battery power along the track
Files = {'P1_MTOW_5600_Wind_0_Temp_20_SA88.mat', 'P3_MTOW_7000_Wind_5_Temp_20_SA88.mat'};
DataFolder = pwd;
if ~isfile(fullfile(DataFolder, Files{1}))
    DataFolder = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end
Colors = [42 120 214; 235 104 52]/255;
PowerMax = 450;                                                 % [kW]

if exist('flightState', 'class') ~= 8
    Simulink.defineIntEnumType('flightState', {'Hover','Transition','FixedWing','BackTransition'}, [0;1;2;3], 'StorageType', 'uint8');
end
kt = 0.514444; ft = 0.3048; nm = 1852;
Battery = struct('SA88', "SA-88", 'LG', "LG-Chem");
Drive = ["BLDC + Abstract inverter", "PMSM + inverter"];

%% Fixed-wing legs
S = load(fullfile(DataFolder, Files{1}), 'TransitionMission'); M = S.TransitionMission;
iT = find([M.mode] == 6);
iW = iT(1) + find([M(iT(1) + 1:iT(2) - 1).mode] == 2);
pw = [M(iW).position]; xw = pw(1, :); hw = -pw(3, :); vw = arrayfun(@(m) m.params(4), M(iW));
leg = repmat("level", 1, numel(xw) - 1); leg(diff(hw) > 1) = "climb"; leg(diff(hw) < -1) = "descent";
len = diff(xw); len(leg ~= "level") = 0; [~, ic] = max(len); leg(ic) = "cruise";
xl = [floor(xw(1)/nm*10) - 1, ceil(xw(end)/nm*10) + 1]/10;

%% Runs
fg = figure('Color', 'w', 'Position', [100 50 1300 980]);
tl = tiledlayout(fg, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(3, 1); h = gobjects(1, 2); name = strings(1, 2); lab = strings(1, 2); cfg = strings(1, 2); cr = strings(1, 2);
for k = 1:3, ax(k) = nexttile(tl); hold(ax(k), 'on'); grid(ax(k), 'on'); set(ax(k), 'FontName', 'Arial', 'FontSize', 11); end
lim = [inf -inf; inf -inf];
for j = 1:2
    t = regexp(Files{j}, 'P(\d+)_MTOW_(\d+)_Wind_(\d+)_Temp_\d+_(SA88|LG)', 'tokens', 'once');
    if isempty(t)
        name(j) = erase(Files{j}, '.mat'); lab(j) = name(j);
    else
        w = str2double(t{3}); ws = "no wind"; if w > 0, ws = sprintf("%g m/s wind", w); end
        name(j) = "P" + t{1}; lab(j) = sprintf("%s, %s lb, %s", name(j), t{2}, ws);
        cfg(j) = Battery.(t{4}) + ", " + Drive(~contains(Files{j}, '_NoInverter') + 1);
    end
    S = load(fullfile(DataFolder, Files{j}), 'outTuned'); o = S.outTuned; u = o.UAV_State;
    ta = u.airspeed.Time(:); Va = squeeze(u.airspeed.Data); Va = Va(:); dt = median(diff(ta));
    Xe = squeeze(u.Xe.Data); if size(Xe, 1) == 3, Xe = Xe.'; end
    Xe = interp1(u.Xe.Time(:), Xe, ta); x = Xe(:, 1); alt = -Xe(:, 3);
    P = interp1(o.pmsm_Vdc.Time(:), o.pmsm_Vdc.Data(:), ta).*interp1(o.pmsm_Idc.Time(:), sum(o.pmsm_Idc.Data, 2), ta)/1e3;
    f = double(o.Flight_Mode.Data(:)); [tf, iu] = unique(o.Flight_Mode.Time(:), 'last');
    fw = interp1(tf, f(iu), ta, 'previous', 'extrap') == 2 & x >= xl(1)*nm & x <= xl(2)*nm;
    plot(ax(1), x(fw)/nm, alt(fw)/ft, 'Color', Colors(j, :), 'LineWidth', 1.6);
    plot(ax(2), x(fw)/nm, Va(fw)/kt, 'Color', Colors(j, :), 'LineWidth', 1.6);
    h(j) = plot(ax(3), x(fw)/nm, movmean(P(fw), round(1/dt)), 'Color', Colors(j, :), 'LineWidth', 1.6);
    v = [alt(fw), Va(fw)]; lim = [min(lim(:, 1), min(v)'), max(lim(:, 2), max(v)')];
    c = fw & x >= xw(ic) & x < xw(ic + 1);
    cr(j) = sprintf(": cruise leg %.0f s, mean %.0f kW, %.2f kWh", ta(find(c, 1, 'last')) - ta(find(c, 1)), mean(P(c)), trapz(ta(c), P(c))/3600);
end

%% Axes
same = cfg(1) == cfg(2);
if ~same, lab(cfg ~= "") = lab(cfg ~= "") + ", " + cfg(cfg ~= ""); end
lab = lab + cr;
ya = lim(1, :)/ft; ya = ya + [-0.05 0.25]*diff(ya);
for k = 1:3
    xlim(ax(k), xl);
    for q = 1:numel(xw), xline(ax(k), xw(q)/nm, ':', 'Color', [0.45 0.45 0.45], 'HandleVisibility', 'off'); end
end
for q = 1:numel(leg)
    text(ax(1), mean(xw(q:q + 1))/nm, ya(2) - 0.06*diff(ya), sprintf('%s, %.0f kt', leg(q), vw(q + 1)/kt), 'HorizontalAlignment', 'center', ...
         'FontName', 'Arial', 'FontSize', 10, 'Color', [0.35 0.35 0.35], 'Clipping', 'on');
end
yline(ax(1), hw(ic + 1)/ft, '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
yline(ax(2), vw(ic + 1)/kt, '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
ylim(ax(1), ya); ylim(ax(2), [floor(lim(2, 1)/kt/5)*5 - 5, ceil(lim(2, 2)/kt/5)*5 + 5]); ylim(ax(3), [0 PowerMax]);
ylabel(ax(1), 'Altitude (ft)'); ylabel(ax(2), 'Airspeed (kt)'); ylabel(ax(3), 'Battery power (kW), 1 s mean'); xlabel(ax(3), 'Track distance (NM)');
legend(ax(3), h, lab, 'Location', 'northeast', 'Box', 'off', 'FontSize', 11, 'Interpreter', 'none');
ttl = name(1) + " against " + name(2) + " in fixed-wing flight";
if same && cfg(1) ~= "", ttl = ttl + ", " + cfg(1); end
title(ax(1), ttl, 'FontSize', 13, 'Interpreter', 'none');
