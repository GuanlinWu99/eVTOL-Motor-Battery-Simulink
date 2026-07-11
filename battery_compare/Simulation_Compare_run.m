function Simulation_Compare_run(Profile, path_proto, path_jung, out_png)
%SIMULATION_COMPARE_RUN  Parametrized version of Simulation_Compare.m.
% Solid  line = "Prototype" = SA-88 (path_proto)
% Dash-dot   = "Jung"      = Prof. Jung / LG-Chem (path_jung)
% Saves the 3x3 "Battery Pack Electrical Performance" figure to out_png.

S1 = load(path_jung);    outTuned_Jung      = S1.outTuned;
S2 = load(path_proto);   outTuned_Prototype = S2.outTuned;  HEV_Param = S2.HEV_Param;
outTuned = outTuned_Prototype;     % used for transitions + mode background

L_Prototype = length(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time);
L_Jung      = length(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time);
Time        = outTuned_Prototype.Battery_Data.Batt.SOC____.Time;

% --- transition times (robust to <4 transitions) ---
Flight_Condition = string(outTuned.Flight_Mode.Data);
changeIdx = find(Flight_Condition(1:end-1) ~= Flight_Condition(2:end));
transitions = struct('Index', {}, 'From', {}, 'To', {});
for i = 1:length(changeIdx)
    transitions(i).Index = changeIdx(i);
    transitions(i).From  = Flight_Condition(changeIdx(i));
    transitions(i).To    = Flight_Condition(changeIdx(i)+1);
end
t_scale = size(outTuned.UAV_State.Vb.Data,3)/size(outTuned.Flight_Mode.Data,1)*0.001;
T_end   = size(outTuned.UAV_State.Vb.Data,3)*0.001;
nT = numel(transitions);
Tv = [T_end T_end T_end T_end];
for i = 1:min(nT,4), Tv(i) = transitions(i).Index*t_scale; end
T1=Tv(1); T2=Tv(2); T3=Tv(3); T4=Tv(4);

%% Prototype (SA-88) power / energy
Pack_Power_Prototype  = outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Data .* outTuned_Prototype.Battery_Data.Batt.Current__A_.Data;
Pack_Energy_Prototype = zeros(L_Prototype,1);
for i = 2:L_Prototype
    Pack_Energy_Prototype(i) = Pack_Power_Prototype(i)*0.001 + Pack_Energy_Prototype(i-1);
end
%% Jung power / energy
Pack_Power_Jung  = outTuned_Jung.Battery_Data.Batt.Voltage__V_.Data .* outTuned_Jung.Battery_Data.Batt.Current__A_.Data;
Pack_Energy_Jung = zeros(L_Jung,1);
for i = 2:L_Jung
    Pack_Energy_Jung(i) = Pack_Power_Jung(i)*0.001 + Pack_Energy_Jung(i-1);
end

Font=13; Buffer=10; Transparency=0.6; Line_Width=2.5;
Battery_Capacity = (HEV_Param.Capacity*HEV_Param.Np*HEV_Param.Ns*3.6)/1000;

figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.8],'Color','w');
if ismember(Profile,[1 3 7 10])
    socOff = (24.17/Battery_Capacity)*100; engOff = 24.17;
else
    socOff = 0; engOff = 0;
end

bp = @(ts,proto,jung) deal(ts);  %#ok<NASGU>
PB = outTuned_Prototype.Battery_Data.Batt;  JB = outTuned_Jung.Battery_Data.Batt;

subplot(3,3,1)
plot(PB.SOC____.Time, PB.SOC____.Data - socOff,'LineWidth',Line_Width); hold on; grid on;
plot(JB.SOC____.Time, JB.SOC____.Data - socOff,'-.','LineWidth',Line_Width);
title('SOC','FontSize',Font); xlabel('Time (s)'); ylabel('(%)'); xlim([0 Time(end)]); ylim([50 100])

subplot(3,3,2)
plot(PB.C_rate.Time, PB.C_rate.Data,'LineWidth',Line_Width); grid on; hold on;
plot(JB.C_rate.Time, JB.C_rate.Data,'-.','LineWidth',Line_Width);
title('C-rate','FontSize',Font); xlabel('Time (s)'); ylabel('(-)'); xlim([0 Time(end)])

subplot(3,3,3)
plot(PB.Current__A_.Time, PB.Current__A_.Data,'LineWidth',Line_Width); grid on; hold on;
plot(JB.Current__A_.Time, JB.Current__A_.Data,'-.','LineWidth',Line_Width);
title('Current','FontSize',Font); xlabel('Time (s)'); ylabel('(A)'); xlim([0 Time(end)])

subplot(3,3,4)
plot(PB.Voltage__V_.Time, PB.Voltage__V_.Data,'LineWidth',Line_Width); grid on; hold on;
plot(JB.Voltage__V_.Time, JB.Voltage__V_.Data,'-.','LineWidth',Line_Width);
title('Voltage','FontSize',Font); xlabel('Time (s)'); ylabel('(V)'); xlim([0 Time(end)])

subplot(3,3,5)
plot(PB.Voltage__V_.Time, Pack_Power_Prototype/1000,'LineWidth',Line_Width); grid on; hold on;
plot(JB.Voltage__V_.Time, Pack_Power_Jung/1000,'-.','LineWidth',Line_Width);
title('Required Power','FontSize',Font); xlabel('Time (s)'); ylabel('(kW)'); xlim([0 Time(end)])

subplot(3,3,6)
plot(PB.Voltage__V_.Time, Pack_Energy_Prototype/(3.6*10^6) + engOff,'LineWidth',Line_Width); grid on; hold on;
plot(JB.Voltage__V_.Time, Pack_Energy_Jung/(3.6*10^6) + engOff,'-.','LineWidth',Line_Width);
title('Cumulative Energy Consumption','FontSize',Font); xlabel('Time (s)'); ylabel('(kWh)'); xlim([0 Time(end)-Buffer])

sgtitle(sprintf('Battery Pack Electrical Performance — P%d  (solid: SA-88,  dash-dot: Prof. Jung)', Profile), ...
        'FontSize',16,'FontWeight','bold');

% --- flight-mode shaded background ---
modeIntervals = [0 T1; T1 T2; T2 T3; T3 T4; T4 5000];
modeColors = [0.15 0.15 0.15; 0.5 0.5 0.5; 0.9 0.9 0.9; 0.5 0.5 0.5; 0.15 0.15 0.15];
axs = flipud(findall(gcf,'Type','Axes'));
for ax = transpose(axs)
    hold(ax,'on'); yl = ylim(ax);
    for k = 1:size(modeIntervals,1)
        h = patch(ax, [modeIntervals(k,1) modeIntervals(k,2) modeIntervals(k,2) modeIntervals(k,1)], ...
                  [yl(1) yl(1) yl(2) yl(2)], modeColors(k,:), 'FaceAlpha', Transparency, 'EdgeColor','none');
        uistack(h,'bottom'); h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end
modeLabels = {' Multi-Copter Mode ',' Forward Transition Mode ',' Fixed-Wing Mode ',' Backward Transition Mode '};
lh = [];
for k = 1:numel(modeLabels)
    lh(end+1) = patch(NaN, NaN, modeColors(k,:), 'FaceAlpha', Transparency, 'EdgeColor','none'); %#ok<AGROW>
end
legend(lh, modeLabels, 'Orientation','vertical','Box','off','FontSize',12,'Position',[0.005 0.02 0.5 0.06]);

exportgraphics(gcf, out_png, 'Resolution',200);
close(gcf);
fprintf('  saved %s\n', out_png);
end
