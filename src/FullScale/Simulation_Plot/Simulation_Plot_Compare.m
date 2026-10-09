%% Figure 1: SA-88 1RC versus LG-Chem 2RC
% Compare saved simulation results without running the simulator.
% Each MAT file supplies its own outTuned AND uavParams.

%% Settings
SA88_FileName = 'P1_MTOW_5600_Wind_0_Temp_20_SA88.mat';
LG_FileName   = 'P1_MTOW_5600_Wind_0_Temp_20_LG.mat';

% Start in the MAT-file folder, or set DataFolder to that folder explicitly.
DataFolder = pwd;
if ~isfile(fullfile(DataFolder, SA88_FileName))
    % Default project location: this script is in src/FullScale/Simulation_Plot.
    ScriptFolder = fileparts(mfilename('fullpath'));
    DataFolder = fileparts(fileparts(fileparts(ScriptFolder)));
end

% Preserve the preconditioning estimate in the original figure(1):
% at 20 degC, add mass * 950 J/(kg*K) * 50 K to the energy budget.
% This is a plotting adjustment, not a simulated thermal process.
% Set false to show logged SOC and terminal energy without this adjustment.
IncludePreconditioning = true;
Font = 13;
Line_Width = 2;
Transparency = 0.6;
SA88_Color = [0.0000 0.4470 0.7410];
LG_Color   = [0.8500 0.3250 0.0980];

% Saved Flight_Mode logs use the enum defined by the project startup script.
% Define only that type here; do not execute startup.m to make plots.
if exist('flightState', 'class') ~= 8
    Simulink.defineIntEnumType('flightState', ...
        {'Hover','Transition','FixedWing','BackTransition'}, [0;1;2;3], ...
        'StorageType', 'uint8');
end

%% Read each run independently
SA88 = readBatteryRun(fullfile(DataFolder, SA88_FileName), IncludePreconditioning);
LG   = readBatteryRun(fullfile(DataFolder, LG_FileName), IncludePreconditioning);

% The runs do not need equal lengths or matching sample times.
PlotEnd = max([SA88.Time(end), LG.Time(end), ...
               SA88.AltitudeTime(end), LG.AltitudeTime(end)]);

%% Draw only figure(1), retaining the original 3-by-3 layout
figure(1)
clf;
fig = gcf;
set(gcf,'Units','normalized','OuterPosition',[0.1 0.1 0.5 0.8],'Color','w');

PlotFields = {'SOC_Plot','Crate','Current_A','Voltage_V', ...
              'Power_kW','Energy_Plot','Altitude_m'};
PlotTitles = {'SOC','C-rate','Current','Voltage', ...
              'Required Power','Cumulative Energy Consumption','Altitude'};
PlotUnits  = {'(%)','(-)','(A)','(V)','(kW)','(kWh)','(m)'};

% Preserve the original figure(1) axis limits.
AX_SOC     = [50 95];
AX_CRATE   = [0 8];
AX_CURRENT = [0 1500];
AX_VOLTAGE = [600 850];
AX_POWER   = [0 1500];
AX_ENERGY  = [0 80];

PlotLimits = {AX_SOC, AX_CRATE, AX_CURRENT, AX_VOLTAGE, AX_POWER, AX_ENERGY};
PlotAxes = gobjects(1,7);
PlotLines = gobjects(7,2);

for k = 1:6
    ax = subplot(3,3,k, 'Parent', fig);
    PlotAxes(k) = ax;
    hold(ax, 'on');
    if k == 7
        Time_SA88 = SA88.AltitudeTime;
        Time_LG = LG.AltitudeTime;
        Width = 2;
    else
        Time_SA88 = SA88.Time;
        Time_LG = LG.Time;
        Width = Line_Width;
    end
    PlotLines(k,1) = plot(ax, Time_SA88, SA88.(PlotFields{k}), '-', 'Color', SA88_Color, 'LineWidth', Width, 'DisplayName', 'KENCOA SA-88 (1RC)');
    PlotLines(k,2) = plot(ax, Time_LG, LG.(PlotFields{k}), '-.', 'Color', LG_Color, 'LineWidth', Width, 'DisplayName', 'Prof. Jung LG-Chem (2RC)');
    grid(ax, 'on');
    box(ax, 'on');
    if k == 7
        title(ax, PlotTitles{k});
        xlabel(ax, 'Time (s)');
        ylabel(ax, PlotUnits{k});
    else
        title(ax, PlotTitles{k}, 'FontSize', Font);
        xlabel(ax, 'Time (s)', 'FontSize', 12);
        ylabel(ax, PlotUnits{k}, 'FontSize', 12);
        ylim(ax, PlotLimits{k});
    end
    xlim(ax, [0 PlotEnd]);
end

% Keep a small battery legend separate from the original flight-mode legend.
BatteryLegend = legend(PlotAxes(1), PlotLines(1,:), {'KENCOA SA-88 (1-RC)', 'Prof. Jung LG-Chem (2-RC)'}, 'Location', 'southwest', 'Box', 'off', 'FontSize', 9);
BatteryLegend.AutoUpdate = 'off';
sgtitle('Battery Pack Electrical Performance','FontSize',20,'FontWeight','bold');

% Supply actual SA-88 transition times to the original plotting helpers.
% Pad missing transitions to the reference end time for incomplete flights.
changeIdx = find(diff(SA88.Mode) ~= 0) + 1;
Tvec = repmat(SA88.ModeTime(end), 1, 4);
nT = min(numel(changeIdx), 4);
Tvec(1:nT) = SA88.ModeTime(changeIdx(1:nT));
T1 = Tvec(1); T2 = Tvec(2); T3 = Tvec(3); T4 = Tvec(4);
shadeFlightModes(gcf, [T1 T2 T3 T4], PlotEnd, Transparency, 'default');
addFlightModeLegend(gcf, Transparency, 'bilingual', 'vertical', 'default');
linkaxes(PlotAxes, 'x');

fprintf('SA-88: %.3f s, %.1f Ah, %.3f kWh nominal; final logged SOC %.2f %%\n', ...
    SA88.Time(end), SA88.Capacity_Ah, SA88.NominalEnergy_kWh, SA88.SOC_Percent(end));
fprintf('LG:    %.3f s, %.1f Ah, %.3f kWh nominal; final logged SOC %.2f %%\n', ...
    LG.Time(end), LG.Capacity_Ah, LG.NominalEnergy_kWh, LG.SOC_Percent(end));
fprintf('Preconditioning adjustment: SA-88 %.2f kWh / LG %.2f kWh. Shading: SA-88 reference.\n', ...
    SA88.Preconditioning_kWh, LG.Preconditioning_kWh);

%% Local functions
function result = readBatteryRun(fileName, includePreconditioning)
% Read only the signals needed by figure(1); keep pack parameters per run.
    if ~isfile(fileName)
        error('BatteryCompare:MissingFile', 'MAT file not found: %s', fileName);
    end
    saved = load(fileName, 'outTuned', 'uavParams');
    if ~isfield(saved, 'outTuned') || ~isfield(saved, 'uavParams')
        error('BatteryCompare:MissingVariables', ...
            'The file must contain outTuned and uavParams: %s', fileName);
    end
    out = saved.outTuned;
    params = saved.uavParams;
    [tV, voltage] = readLoggedSignal(out.pmsm_Vdc, 'pmsm_Vdc');
    [tI, currents] = readLoggedSignal(out.pmsm_Idc, 'pmsm_Idc');
    [tSOC, soc] = readLoggedSignal(out.pmsm_SOC, 'pmsm_SOC');
    if size(voltage,2) ~= 1 || size(soc,2) ~= 1
        error('BatteryCompare:ScalarSignals', 'Vdc and SOC must be scalar signals.');
    end
    % The existing simulator logs one DC current channel per motor.
    % Summing all columns also supports a single already-summed pack channel.
    packCurrent = sum(currents, 2);

    % Align signals within this run, using only their common recorded interval.
    % Never extrapolate a shorter log or align one battery to the other.
    tStart = max([tV(1), tI(1), tSOC(1)]);
    tStop  = min([tV(end), tI(end), tSOC(end)]);
    keep = tV >= tStart & tV <= tStop;
    result.Time = tV(keep);
    if numel(result.Time) < 2
        error('BatteryCompare:NoTimeOverlap', 'Insufficient overlapping log times: %s', fileName);
    end
    result.Voltage_V = voltage(keep);
    result.Current_A = interp1(tI, packCurrent, result.Time, 'linear');
    result.SOC_Percent = 100 * interp1(tSOC, soc, result.Time, 'linear');
    result.Capacity_Ah = params.Battery_Pack_Capacity;
    validateattributes(result.Capacity_Ah, {'numeric'}, {'scalar','finite','positive'});
    result.NominalEnergy_kWh = result.Capacity_Ah * params.Battery_Pack_Voltage / 1000;
    validateattributes(result.NominalEnergy_kWh, {'numeric'}, {'scalar','finite','positive'});
    result.Crate = result.Current_A / result.Capacity_Ah;
    result.Power_kW = result.Voltage_V .* result.Current_A / 1000;
    % Integrate kW over the recorded seconds to obtain net terminal energy.
    result.Energy_kWh = cumtrapz(result.Time, result.Power_kW) / 3600;

    result.Preconditioning_kWh = 0;
    if includePreconditioning && params.Temperature == 20
        result.Preconditioning_kWh = params.Battery_Pack_Mass * 950 * 50 / 3.6e6;
    end
    result.SOC_Plot = result.SOC_Percent ...
        - 100 * result.Preconditioning_kWh / result.NominalEnergy_kWh;
    result.Energy_Plot = result.Energy_kWh + result.Preconditioning_kWh;

    result.AltitudeTime = double(out.PositionCmdFdbk.time(:));
    position = squeeze(double(out.PositionCmdFdbk.signals.values));
    if size(position,2) == numel(result.AltitudeTime) && size(position,1) == 6
        result.Altitude_m = -position(6,:).';
    elseif size(position,1) == numel(result.AltitudeTime) && size(position,2) == 6
        result.Altitude_m = -position(:,6);
    else
        error('BatteryCompare:PositionShape', 'Expected six PositionCmdFdbk channels.');
    end
    [result.ModeTime, result.Mode] = readLoggedSignal(out.Flight_Mode, 'Flight_Mode');
    if size(result.Mode,2) ~= 1 || any(~ismember(result.Mode, 0:3))
        error('BatteryCompare:FlightMode', 'Expected flightState values 0, 1, 2, or 3.');
    end
end

function [time, data] = readLoggedSignal(signal, name)
% Normalize scalar/vector timeseries to sample-by-channel arrays.
    time = double(signal.Time(:));
    data = squeeze(double(signal.Data));
    if isvector(data)
        data = data(:);
    elseif size(data,1) ~= numel(time) && size(data,2) == numel(time)
        data = data.';
    end
    if numel(time) < 2 || size(data,1) ~= numel(time) ...
            || any(~isfinite(time)) || any(~isfinite(data(:))) || any(diff(time) < 0)
        error('BatteryCompare:InvalidLog', 'Invalid data/time array in %s.', name);
    end
    % At duplicate timestamps, use the last logged value.
    [time, lastIndex] = unique(time, 'last');
    data = data(lastIndex,:);
    if numel(time) < 2
        error('BatteryCompare:InvalidLog', '%s needs two distinct timestamps.', name);
    end
end

%% ========================================================================
%  Local plotting helper functions
%  ========================================================================
function shadeFlightModes(figHandle, boundaries, xEnd, faceAlpha, palette)
%SHADEFLIGHTMODES Shade plot backgrounds according to flight mode.
%
%   boundaries = [T1 T2 T3 T4] for time-based plots, or
%                [D1 D2 D3 D4] for distance-based plots.
%
%   The five shaded intervals are:
%     1) Multi-Copter
%     2) Forward Transition
%     3) Fixed-Wing
%     4) Backward Transition
%     5) Multi-Copter

    modeColors = getFlightModeColors(palette);
    boundaries = boundaries(:).';

    if numel(boundaries) ~= 4
        error('shadeFlightModes:InvalidBoundaries', ...
              'boundaries must contain exactly four transition locations.');
    end

    modeIntervals = [0              boundaries(1);
                     boundaries(1)  boundaries(2);
                     boundaries(2)  boundaries(3);
                     boundaries(3)  boundaries(4);
                     boundaries(4)  xEnd];

    axs = findall(figHandle, 'Type', 'Axes');
    axs = flipud(axs);

    for ax = transpose(axs)
        % Ignore helper/legend axes if this function is called again.
        if strcmpi(ax.Visible, 'off')
            continue;
        end

        hold(ax, 'on');
        yl = ylim(ax);
        ylim(ax, yl);  % Freeze current y-limits before adding patches.

        for k = 1:size(modeIntervals,1)
            x1 = modeIntervals(k,1);
            x2 = modeIntervals(k,2);

            % In an incomplete mission, padded transition locations can be
            % identical. Skip zero-width or reversed intervals.
            if ~isfinite(x1) || ~isfinite(x2) || x2 <= x1
                continue;
            end

            h = patch(ax, ...
                [x1 x2 x2 x1], ...
                [yl(1) yl(1) yl(2) yl(2)], ...
                modeColors(k,:), ...
                'FaceAlpha', faceAlpha, ...
                'EdgeColor', 'none');

            uistack(h, 'bottom');
            h.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end
end

function addFlightModeLegend(figHandle, faceAlpha, language, orientation, palette)
%ADDFLIGHTMODELEGEND Add a separate legend for flight-mode shading.
% A dedicated invisible axes is used so existing subplot legends are not
% overwritten.

    modeColors = getFlightModeColors(palette);

    switch lower(language)
        case 'bilingual'
            modeLabels = { ...
                ' Multi-Copter Mode (멀티콥터 모드) ', ...
                ' Forward Transition Mode (전방 변환 모드) ', ...
                ' Fixed-Wing Mode (고정익 모드) ', ...
                ' Backward Transition Mode (후방 변환 모드) '};
        otherwise
            modeLabels = { ...
                ' Multi-Copter Mode ', ...
                ' Forward Transition Mode ', ...
                ' Fixed-Wing Mode ', ...
                ' Backward Transition Mode '};
    end

    legAx = axes(figHandle, ...
        'Position', [0 0 1 1], ...
        'Visible', 'off', ...
        'HitTest', 'off', ...
        'Tag', 'FlightModeLegendAxes');
    hold(legAx, 'on');

    legendHandles = gobjects(1,4);
    for k = 1:4
        legendHandles(k) = patch(legAx, NaN, NaN, modeColors(k,:), ...
            'FaceAlpha', faceAlpha, ...
            'EdgeColor', 'none');
    end

    switch lower(orientation)
        case 'horizontal'
            lgd = legend(legAx, legendHandles, modeLabels, ...
                'Orientation', 'horizontal', ...
                'NumColumns', 4, ...
                'Box', 'off', ...
                'FontSize', 13, ...
                'Position', [0.015 0.015 1.5 0.01]);
            lgd.ItemTokenSize = [35 18];
        otherwise
            lgd = legend(legAx, legendHandles, modeLabels, ...
                'Orientation', 'vertical', ...
                'Box', 'off', ...
                'FontSize', 15, ...
                'Position', [0.005 0.11 1.25 0.2]);
            lgd.ItemTokenSize = [50 30];
    end

    lgd.AutoUpdate = 'off';
end

function modeColors = getFlightModeColors(palette)
%GETFLIGHTMODECOLORS Return the grayscale palette used for mode shading.

    switch lower(palette)
        case 'soft'
            modeColors = [0.20 0.20 0.20;   % Multi-Copter
                          0.50 0.50 0.50;   % Forward Transition
                          0.80 0.80 0.80;   % Fixed-Wing
                          0.50 0.50 0.50;   % Backward Transition
                          0.20 0.20 0.20];  % Multi-Copter
        otherwise
            modeColors = [0.15 0.15 0.15;   % Multi-Copter
                          0.50 0.50 0.50;   % Forward Transition
                          0.90 0.90 0.90;   % Fixed-Wing
                          0.50 0.50 0.50;   % Backward Transition
                          0.15 0.15 0.15];  % Multi-Copter
    end
end

