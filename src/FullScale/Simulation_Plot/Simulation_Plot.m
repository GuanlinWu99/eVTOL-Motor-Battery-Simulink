
%% This is a function for drawing plots.
% V = squeeze(outTuned.Body_Velocity.Data);
% plot(outTuned.Body_Velocity.Time,V(1,:),'LineWidth',2); hold on; grid on
% plot(outTuned.Body_Velocity.Time,V(2,:),'LineWidth',2); hold on
% plot(outTuned.Body_Velocity.Time,V(3,:),'LineWidth',2);
% legend('Vx','Vy','Vz')

%% Read Simulation 
Time                   =   outTuned.Rotor1_RPM_Reference.Time;
Time_motor             =   outTuned.pmsm_Idc.Time(:);
%Time_torque           =   outTuned.Rotor1_Drag_Tq.Time;
% pack signals from the PMSM drive logs, the Batt bus is gone
bt   = outTuned.pmsm_Vdc.Time(:);
bv   = outTuned.pmsm_Vdc.Data(:);
bi   = sum(outTuned.pmsm_Idc.Data,2);                               % per-motor -> pack
bsoc = interp1(outTuned.pmsm_SOC.Time(:),outTuned.pmsm_SOC.Data(:),bt,'linear','extrap')*100;
bcr  = bi/uavParams.Battery_Pack_Capacity;
Time_battery           =   bt;
Time_flight            =   outTuned.UAV_State.Xe.Time;

positionFeedbackData   =   squeeze(outTuned.PositionCmdFdbk.signals.values);

Rotor1_RPM_Reference   =   outTuned.Rotor1_RPM_Reference.Data;
Rotor2_RPM_Reference   =   outTuned.Rotor2_RPM_Reference.Data;
Rotor3_RPM_Reference   =   outTuned.Rotor3_RPM_Reference.Data;
Rotor4_RPM_Reference   =   outTuned.Rotor4_RPM_Reference.Data;

Rotor1_RPM             =   outTuned.Rotor1_RPM.Data;
Rotor2_RPM             =   outTuned.Rotor2_RPM.Data;
Rotor3_RPM             =   outTuned.Rotor3_RPM.Data;
Rotor4_RPM             =   outTuned.Rotor4_RPM.Data;

% Motor#_* replaced by the PMSM drive logs; Vdc is the shared bus
Idc = outTuned.pmsm_Idc.Data;  Vdc = outTuned.pmsm_Vdc.Data(:);
if size(Idc,2) < 4, Idc = repmat(Idc(:,1)/4,1,4); end

Motor1_Current         =   Idc(:,1);
Motor2_Current         =   Idc(:,2);
Motor3_Current         =   Idc(:,3);
Motor4_Current         =   Idc(:,4);

Motor1_Voltage         =   Vdc;
Motor2_Voltage         =   Vdc;
Motor3_Voltage         =   Vdc;
Motor4_Voltage         =   Vdc;

Motor1_Power           =   Vdc.*Idc(:,1)/1000;      % [kW]
Motor2_Power           =   Vdc.*Idc(:,2)/1000;      % [kW]
Motor3_Power           =   Vdc.*Idc(:,3)/1000;      % [kW]
Motor4_Power           =   Vdc.*Idc(:,4)/1000;      % [kW]

% Motor1_Drag_Tq       =   outTuned.Rotor1_Drag_Tq.Data;
% Motor2_Drag_Tq         =   outTuned.Rotor2_Drag_Tq.Data;
% Motor3_Drag_Tq         =   outTuned.Rotor3_Drag_Tq.Data;
% Motor4_Drag_Tq         =   outTuned.Rotor4_Drag_Tq.Data;

Battery_SOC            =   bsoc;
Battery_Crate          =   bcr;
Battery_Current        =   bi;

v1 = reshape(outTuned.UAV_State.Vb.Data(:,3,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);
v2 = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2+reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]).^2);
v3 = reshape(outTuned.UAV_State.Vb.Data(:,2,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1])./reshape(outTuned.UAV_State.Vb.Data(:,1,:),[size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]);

% Determine the transition time 
Flight_Condition = string(outTuned.Flight_Mode.Data);
changeIdx = find(Flight_Condition(1:end-1) ~= Flight_Condition(2:end));
transitions = struct('Index', {}, 'From', {}, 'To', {});

for i = 1:length(changeIdx)
    transitions(i).Index = changeIdx(i);
    transitions(i).From = Flight_Condition(changeIdx(i));
    transitions(i).To = Flight_Condition(changeIdx(i)+1);
end

Travel_Distance = positionFeedbackData(4,:);

% --- Transition times/distances, robust to INCOMPLETE flights ----------
% A mission that does not complete all four flight-mode changes (e.g. a
% strong cross-wind run that never finishes the forward transition) yields
% fewer than 4 entries in `transitions`. Indexing transitions(2..4) then
% throws "Index exceeds the number of array elements" and the whole plot
% crashes. Pad any missing transition with the end-of-flight time/distance
% so the mode-interval matrices below stay well-formed and the plots (incl.
% the battery curves) still render. When all 4 transitions are present the
% values are identical to the original code.
% use the logged time vectors; the three signals are no longer all at Ts
t_mode  = outTuned.Flight_Mode.Time(:);
t_pos   = outTuned.PositionCmdFdbk.time(:);
T_end   = outTuned.UAV_State.Vb.Time(end);
D_end   = Travel_Distance(end);
nT      = numel(transitions);

Tvec = [T_end T_end T_end T_end];
Dvec = [D_end D_end D_end D_end];
for i = 1:min(nT,4)
    Tvec(i) = t_mode(transitions(i).Index);
    Dvec(i) = interp1(t_pos, Travel_Distance(:), Tvec(i), 'linear', 'extrap');   % own time base
end
T1 = Tvec(1); T2 = Tvec(2); T3 = Tvec(3); T4 = Tvec(4);   % Hover->Tr, Tr->FW, FW->Tr, Tr->Hover
D1 = Dvec(1); D2 = Dvec(2); D3 = Dvec(3); D4 = Dvec(4);

if nT < 4
    fprintf(2, '[Simulation_Plot] WARNING: only %d/4 flight-mode transitions detected; missing ones padded to end-of-flight (incomplete mission).\n', nT);
end
for i = 1:min(nT,4)
    fprintf('Transition Time: %f: %s -> %s\n', Tvec(i), transitions(i).From, transitions(i).To);
end

%% Plot Simulations (Updated: 11/07/2025) 
%% [1] 4 Rotor Performance
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.65 0.9], 'Color','w');
% t = tiledlayout(4,4,'TileSpacing','compact','Padding','compact');
% names = ["Motor1","Motor2","Motor3","Motor4"];
% Width = 2;
% AxisFont = 17;
% 
% % -------- Row 1: Speed --------
% ax1 = gobjects(1,4);
% for k = 1:4
%     ax1(k) = nexttile(k);
%     plot(Time, eval("Rotor"+k+"_RPM"), 'LineWidth',Width); hold on;
%     plot(Time, eval("Rotor"+k+"_RPM_Reference"), '-.', 'LineWidth', Width);
%     grid on; box on;
%     title(names(k) + " Speed");
%     if k==1, ylabel('RPM','FontSize',AxisFont); end
%     if k==1, legend({'Actual','Cmd'},'Location','southeast'); else, legend off; end
%     ylim([0, max(eval("Rotor"+k+"_RPM_Reference"))])
% end
% linkaxes(ax1,'y');
% 
% % -------- Row 2: Power --------
% ax2 = gobjects(1,4);
% for k = 1:4
%     ax2(k) = nexttile(4+k);
%     plot(Time_motor, eval("Motor"+k+"_Power"), 'LineWidth', Width);
%     grid on; box on;
%     title(names(k) + " Power");
%     if k==1, ylabel('Power (kW)','FontSize',AxisFont); end
% end
% linkaxes(ax2,'y');
% 
% % -------- Row 3: Current --------
% ax3 = gobjects(1,4);
% for k = 1:4
%     ax3(k) = nexttile(8+k);
%     plot(Time_motor, eval("Motor"+k+"_Current"), 'LineWidth', Width);
%     grid on; box on;
%     title(names(k) + " Current");
%     if k==1, ylabel('Current (A)','FontSize',AxisFont); end
% end
% linkaxes(ax3,'y');
% 
% % -------- Row 4: Voltage --------
% ax4 = gobjects(1,4);
% for k = 1:4
%     ax4(k) = nexttile(12+k);
%     plot(Time_motor, eval("Motor"+k+"_Voltage"), 'LineWidth', Width);
%     grid on; box on;
%     title(names(k) + " Voltage");
%     if k==1, ylabel('Voltage (V)','FontSize',AxisFont); end
%     xlabel('Time (s)','FontSize',AxisFont);
% end
% linkaxes(ax4,'y');
% 
% sgtitle('Tiltrotor eVTOL (Rotors 1 & 2 Tilted, 3 & 4 Fixed)','FontSize',15,'FontWeight','bold');

% %% [2] Drag Torque
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.65 0.3], 'Color','w'); 
% subplot(1,4,1)
% plot(Time_torque, Motor1_Drag_Tq, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); title('Motor1 Drag Torque')
% 
% subplot(1,4,2)
% plot(Time_torque, Motor2_Drag_Tq, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); title('Motor2 Drag Torque')
% 
% subplot(1,4,3)
% plot(Time_torque, Motor3_Drag_Tq, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); title('Motor3 Drag Torque')
% 
% subplot(1,4,4)
% plot(Time_torque, Motor4_Drag_Tq, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); title('Motor4 Drag Torque')

% %% [2.1] Drag Torque
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.55 0.3], 'Color','w'); 
% subplot(1,3,1)
% plot(outTuned.Rotor_Torque.Time, outTuned.Rotor_Torque.Data(:,1)/1000, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(kNm)'); title('Torque X')
% 
% subplot(1,3,2)
% plot(outTuned.Rotor_Torque.Time, outTuned.Rotor_Torque.Data(:,2)/1000, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(kNm)'); title('Torque Y')
% 
% subplot(1,3,3)
% plot(outTuned.Rotor_Torque.Time, outTuned.Rotor_Torque.Data(:,3)/1000, 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(kNm)'); title('Torque Z')

% % %% [2.2] Drag Torque
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.65 0.3], 'Color','w'); 
% subplot(1,4,1)
% plot(outTuned.Rotor1_Mxyz.Time, -outTuned.Rotor1_Mxyz.Data(:,3), 'LineWidth', 2.5); grid on; 
% xlabel('Time (s)'); ylabel('(Nm)'); legend('Rotor 1'); title('Motor1 Drag Torque')
% 
% subplot(1,4,2)
% plot(outTuned.Rotor1_Mxyz.Time, -outTuned.Rotor2_Mxyz.Data(:,3), 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); legend('Rotor 2'); title('Motor2 Drag Torque')
% 
% subplot(1,4,3)
% plot(outTuned.Rotor1_Mxyz.Time, -outTuned.Rotor3_Mxyz3.Data(:,3), 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); legend('Rotor 3'); title('Motor3 Drag Torque')
% 
% subplot(1,4,4)
% plot(outTuned.Rotor1_Mxyz.Time, -outTuned.Rotor4_Mxyz.Data(:,3), 'LineWidth', 2.5); grid on;
% xlabel('Time (s)'); ylabel('(Nm)'); legend('Rotor 4'); title('Motor4 Drag Torque')

%% [3] Battery Pack Electrical Performance with Different Background Color
L            = numel(bt);
Pack_Power   = bv .* bi;
Font         = 13;
Transparency = 0.6;
Line_Width   = 3;

Pack_Energy  = cumtrapz(bt, Pack_Power);            % [J], step is no longer Ts

% % This is for calculating SOP
% Vmin       = 2.5 * 200;
% Imax_dis   = 22.6 * 200;
% V          = outTuned.Battery_Data.Batt.Voltage__V_.Data;
% I          = outTuned.Battery_Data.Batt.Current__A_.Data;
% SOC        = outTuned.Battery_Data.Batt.SOC____.Data;  % current SOC time series
% 
% % Lookup table breakpoints (SOC axis for R0, R1, R2)
% SOC_bkpt   = uavParams.Battery_Cell.SOC_bkpt;  % [87x1]
% 
% % Interpolate each R component at the operating SOC
% R0         = interp1(SOC_bkpt, uavParams.Battery_Cell.R0, SOC, 'linear', 'extrap');
% R1         = interp1(SOC_bkpt, uavParams.Battery_Cell.R1, SOC, 'linear', 'extrap');
% R2         = interp1(SOC_bkpt, uavParams.Battery_Cell.R2, SOC, 'linear', 'extrap');
% R          = R0 + R1 + R2;      % time varying resistance
% Voc        = V + I .* R; 
% 
% % Voltage-limited discharge current & power
% Iv_dis     = max((Voc - Vmin) ./ R, 0);
% Pv_dis     = Vmin .* Iv_dis;
% 
% % Current-limited discharge power
% Pi_dis     = Voc .* Imax_dis - R .* (Imax_dis.^2);
% Pi_dis     = max(Pi_dis, 0);
% SOP_dis_kW = min(Pv_dis, Pi_dis) / 1000;

% Plot all figure
figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.8], 'Color','w'); 

if uavParams.Temperature == 20
    Pre_Cond_kWh = uavParams.Battery_Pack_Mass*950*50/3.6e6;
else
    Pre_Cond_kWh = 0;
end
Pack_kWh     = uavParams.Battery_Pack_Capacity*uavParams.Battery_Pack_Voltage/1000;
Pre_Cond_SOC = 100*Pre_Cond_kWh/Pack_kWh;                                   % [%] share of the pack spent on heating
if Pre_Cond_kWh > 0
    Pre_Cond_Note = sprintf('incl. %.2f kWh preconditioning', Pre_Cond_kWh);
    SOC_Title     = {'SOC', Pre_Cond_Note};
    Energy_Title  = {'Cumulative Energy Consumption', Pre_Cond_Note};
else
    SOC_Title     = 'SOC';
    Energy_Title  = 'Cumulative Energy Consumption';
end
SOC_Plot    = bsoc - Pre_Cond_SOC;
Energy_Plot = Pack_Energy/(3.6*10^6) + Pre_Cond_kWh;

if ~exist('AX_SOC','var'),     AX_SOC     = [20 100];  end
if ~exist('AX_CRATE','var'),   AX_CRATE   = [0 13];    end
if ~exist('AX_CURRENT','var'), AX_CURRENT = [0 3500];  end
if ~exist('AX_VOLTAGE','var'), AX_VOLTAGE = [650 850]; end
if ~exist('AX_POWER','var'),   AX_POWER   = [0 2500];  end
if ~exist('AX_ENERGY','var'),  AX_ENERGY  = [0 150];   end

subplot(3,3,1)
plot(bt, SOC_Plot,'LineWidth',Line_Width); grid on;
title(SOC_Title,'FontSize',Font); 
xlabel('Time (s)','FontSize',12);
ylabel('(%)','FontSize',12); 
xlim([0 Time(end)]); 
ylim(AX_SOC)

subplot(3,3,2)
plot(bt, bcr,'LineWidth',Line_Width); grid on;
title('C-rate','FontSize',Font);  
xlabel('Time (s)','FontSize',12);
ylabel('(-)','FontSize',12);
xlim([0 Time(end)])
ylim(AX_CRATE)

subplot(3,3,3)
plot(bt, bi,'LineWidth',Line_Width); grid on;
title('Current','FontSize',Font);  
xlabel('Time (s)','FontSize',12); 
ylabel('(A)','FontSize',12);
xlim([0 Time(end)])
ylim(AX_CURRENT)

subplot(3,3,4)
plot(bt, bv,'LineWidth',Line_Width); grid on;
title('Voltage','FontSize',Font);  
xlabel('Time (s)','FontSize',12); 
ylabel('(V)','FontSize',12);
xlim([0 Time(end)])
ylim(AX_VOLTAGE)

subplot(3,3,5)
plot(bt, Pack_Power/1000,'LineWidth',Line_Width); grid on;
title('Required Power','FontSize',Font); 
xlabel('Time (s)','FontSize',12);  
ylabel('(kW)','FontSize',12); 
xlim([0 Time(end)])
ylim(AX_POWER)

subplot(3,3,6)
plot(bt, Energy_Plot,'LineWidth',Line_Width); grid on;
title(Energy_Title,'FontSize',Font);  
xlabel('Time (s)','FontSize',12);
ylabel('(kWh)','FontSize',12);
xlim([0 Time(end)])
ylim(AX_ENERGY)
 
% subplot(3,3,7)
% plot(outTuned.Battery_Data.Batt.Voltage__V_.Time, SOP_dis_kW,'LineWidth',Line_Width); 
% title('SOP','FontSize',Font); ylabel('(kW)','FontSize',12); xlabel('Time (s)','FontSize',12); grid on; xlim([0 Time(end)])

sgtitle('Battery Pack Electrical Performance','FontSize',20,'FontWeight','bold');

modeIntervals = [  0   T1;        % Hover
                  T1   T2;        % Forward Transition
                  T2   T3;        % Fixed-Wing
                  T3   T4;        % Backward Transition
                  T4  5000];      % Hover
                        
modeColors = [0.15 0.15 0.15;     % Hover
              0.5 0.5 0.5;        % Forward Transition
              0.9 0.9 0.9;        % Fixed-Wing
              0.5 0.5 0.5;        % Backward Transition
              0.15 0.15 0.15];    % Hover

axs = findall(gcf,'Type','Axes');
axs = flipud(axs); 

for ax = transpose(axs)
    hold(ax,'on');
    yl = ylim(ax);
    ylim(ax, yl);
    for k = 1:size(modeIntervals,1)
        x1 = modeIntervals(k,1);
        x2 = modeIntervals(k,2);
        h = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], modeColors(k,:), 'FaceAlpha', Transparency, 'EdgeColor', 'none');
        uistack(h,'bottom'); 
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

modeLabels = {' Multi-Copter Mode (멀티콥터 모드) ', ' Forward Transition Mode (전방 변환 모드) ', ' Fixed-Wing Mode (고정익 모드) ', ' Backward Transition Mode (후방 변환 모드) '};
modeLabels_1 = {' Multi-Copter Mode ', ' Forward Transition Mode ', ' Fixed-Wing Mode ', ' Backward Transition Mode '};

legendHandles = [];
legendNames = {};

for k = 1:length(modeLabels)
    if ~ismember(modeLabels{k}, legendNames)
        legendHandles(end+1) = patch(NaN, NaN, modeColors(k,:), 'FaceAlpha', Transparency, 'EdgeColor', 'none');
        legendNames{end+1} = modeLabels{k}; 
    end
end

%lgd = legend(legendHandles, legendNames, 'Orientation', 'horizontal', 'NumColumns', 4, 'Box', 'off', 'FontSize', 15, 'Position', [0.3 0.3 1.5 0.01]); 
lgd = legend(legendHandles, legendNames, 'Orientation', 'vertical', 'Box', 'off', 'FontSize', 15, 'Position', [0.005 0.11 1.25 0.2]); 
lgd.ItemTokenSize = [50, 30];

% If we increase 1.25, then the box moves to the right.
% If we increase 0.2, then the overall size will be increased.

%% [4] Flight Dynamics Simulations 
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.7], 'Color','w'); 
% subplot(3,3,1)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('North & Flight Mode') 
% yyaxis left
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(4,:)/1000,'LineWidth',2)
% ylabel('(km)'); legend('North','Mode','Location','northwest'); xlim([0 Total_sim_time])
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,2)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('West & Flight Mode') 
% yyaxis left
% plot(outTuned.PositionCmdFdbk.time,positionFeedbackData(5,:),'LineWidth',2)
% ylabel('(km)'); legend('West','Mode','Location','northwest'); xlim([0 Total_sim_time]); ylim([-50 50]);
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,3)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('Altitude & Flight Mode') 
% yyaxis left
% plot(outTuned.PositionCmdFdbk.time,-positionFeedbackData(6,:),'LineWidth',2); 
% ylabel('(m)'); legend('Altitude','Mode','Location','northwest'); xlim([0 Total_sim_time])
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,4)
% plot(outTuned.UAV_State.airspeed.Time, atand(v1./v2), 'Linewidth', 2); 
% grid on; ylim([-5 35]); xlabel('Time (s)'); ylabel('(deg)'); title('AOA Angle'); xlim([0 Total_sim_time])
% 
% subplot(3,3,5)
% plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', 2); hold on;
% plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', 2, 'LineStyle', '--')
% grid on; xlabel('Time (s)'); ylabel('(deg)'); title('Tilt Angles'); xlim([0 Total_sim_time])
% 
% subplot(3,3,6)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'k','LineWidth',2); 
% grid on; xlabel('Time (s)'); ylabel('(-)'); title('Body Velocity & Flight Mode')
% yyaxis left
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'r', 'Linewidth', 2); hold on; grid on;
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'g', 'Linewidth', 2); hold on;
% plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'b', 'Linewidth', 2); hold on;
% xlabel('Time (s)'); ylabel('(deg)'); title('Euler Angles'); legend('Roll','Pitch','Yaw','Mode','Location','southwest'); xlim([0 Total_sim_time])
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,7)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% grid on; xlabel('Time (s)'); ylabel('(-)'); title('Body Velocity & Flight Mode')
% yyaxis left
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,1,:), [size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]), 'LineWidth', 2);
% ylabel('(m/s)'); legend('Vx','Mode','Location','southwest'); xlim([0 Total_sim_time]); 
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,8)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('Body Velocity & Flight Mode')
% yyaxis left
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,2,:), [size(outTuned.UAV_State.Vb.Data(:,2,:),3),1]), 'LineWidth', 2);
% ylabel('(m/s)'); legend('Vy','Mode','Location','southwest'); xlim([0 Total_sim_time]);
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% subplot(3,3,9)
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('Body Velocity & Flight Mode')
% yyaxis left
% plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,3,:), [size(outTuned.UAV_State.Vb.Data(:,3,:),3),1]), 'LineWidth', 2);
% ylabel('(m/s)'); legend('Vz','Mode','Location','southwest'); xlim([0 Total_sim_time]);
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k';  
% 
% sgtitle('Flight Dynamics Simulations','FontSize',15,'FontWeight','bold');

%% [4-1] Flight Trajectory in North–Altitude Plane (Visually Stretched)
RGB         = orderedcolors("gem");
H           = rgb2hex(RGB);
ref_2d_traj = zeros(3,size(TransitionMission,2));

for idx = 1:size(ref_2d_traj,2)
    ref_2d_traj(:,idx) = TransitionMission(idx).position;
end

ref_2d_traj(:,3) = [];
ref_air_spd      = zeros(2,2*size(TransitionMission,2)-2);

for idx = 1 : size(TransitionMission,2) - 1
    ref_air_spd(1,2*idx-1) = sqrt(TransitionMission(idx).position(1).^2+TransitionMission(idx).position(2).^2);
    ref_air_spd(1,2*idx) = sqrt(TransitionMission(idx+1).position(1).^2+TransitionMission(idx+1).position(2).^2);
    ref_air_spd(2,2*idx-1) = TransitionMission(idx+1).params(4)/const.kts2mps;
    ref_air_spd(2,2*idx) = TransitionMission(idx+1).params(4)/const.kts2mps;
end

ref_air_spd(:,4:5) = [];
air_spd = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:), [], 1).^2 + ...
               reshape(outTuned.UAV_State.Vb.Data(:,2,:), [], 1).^2 + ...
               reshape(outTuned.UAV_State.Vb.Data(:,3,:), [], 1).^2)/const.kts2mps;

% Vb/PositionCmdFdbk no longer 5:1, so resample instead of a fixed stride
air_spd_pos = interp1(outTuned.UAV_State.Vb.Time(:), air_spd, t_pos, 'linear', 'extrap').';

figure('Units','normalized','OuterPosition',[0.1 0.1 0.9 0.6], 'Color','w'); 

subplot(2,1,1);
plot(ref_2d_traj(1,:),-ref_2d_traj(3,:), 'LineWidth', Line_Width, 'LineStyle',"--",'Color',H(7)); hold on;
plot(positionFeedbackData(4,:), -positionFeedbackData(6,:), 'LineWidth', 1.5,'Color',H(1)); grid on; box on; 
xlabel('Distance (North, m)', 'FontSize', 12); 
ylabel('Altitude (m)', 'FontSize', 12); 
xlim([0 max(positionFeedbackData(4,:))+50]);
if ~exist('AX_ALT','var'), AX_ALT = [0 1000]; end
ylim(AX_ALT);
legend('Mission Profile', 'Actual Trajectory')
title('Flight Trajectory in 2D Vertical Plane', 'FontSize', Font, 'FontWeight', 'bold');
% axis tight; pbaspect([15 1 1]);

subplot(2,1,2);
plot(ref_air_spd(1,:), ref_air_spd(2,:), 'LineWidth', Line_Width, 'LineStyle',"--",'Color',H(7)); hold on; grid on; box on; 
plot(positionFeedbackData(4,:), air_spd_pos, 'LineWidth', 1.5,'Color',H(1));
xlabel('Distance (North, m)', 'FontSize', 12); 
ylabel('Airspeed (kts)', 'FontSize', 12); 
xlim([0 max(positionFeedbackData(4,:))+50]);
if ~exist('AX_SPD','var'), AX_SPD = [0 110]; end
ylim(AX_SPD);
legend('Mission Profile', 'Actual Trajectory')
title('Flight Speed Profile', 'FontSize', Font, 'FontWeight', 'bold');
% axis tight; pbaspect([15 1 1]);

sgtitle('Tiltrotor eVTOL Flight Performance','FontSize',20,'FontWeight','bold');

modeIntervals = [  0   D1;        % Hover
                  D1   D2;        % Forward Transition
                  D2   D3;        % Fixed-Wing
                  D3   D4;        % Backward Transition
                  D4   50000];     % Hover
                        
modeColors = [0.15 0.15 0.15;     % Hover
              0.5 0.5 0.5;        % Forward Transition
              0.9 0.9 0.9;        % Fixed-Wing
              0.5 0.5 0.5;        % Backward Transition
              0.15 0.15 0.15];    % Hover

axs = findall(gcf, 'Type', 'Axes'); axs = flipud(axs); 

for ax = transpose(axs)
    hold(ax, 'on');
    yl = ylim(ax);
    ylim(ax, yl);
    for k = 1:size(modeIntervals,1)
        x1 = modeIntervals(k,1);
        x2 = modeIntervals(k,2);
        h = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], modeColors(k,:), 'FaceAlpha', 0.6, 'EdgeColor', 'none');
        uistack(h, 'bottom'); 
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

%% [4-2] Other Flight Dynamics Simulation Results
figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.6], 'Color','w'); 

subplot(4,1,1)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'Linewidth', Line_Width-1); hold on; grid on;
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, '-.', 'Linewidth', Line_Width); hold on;
xlabel('Time (s)','FontSize',12); 
ylabel('Att (deg)','FontSize',12); 
title('Euler Angles','FontSize',13); 
legend('Roll','Pitch','Yaw','Location','northeast');
xlim([0 Time(end)])
ylim([-10 45]);

% subplot(4,1,2)
% plot(outTuned.UAV_State.RotorParameters.w1.Time, reshape(outTuned.UAV_State.RotorParameters.w1.Data(:,1,:)*60/(2*pi), [size(outTuned.UAV_State.RotorParameters.w1.Data(:,1,:),1),1]), 'LineWidth', Line_Width-1); hold on; grid on;
% plot(outTuned.UAV_State.RotorParameters.w2.Time, reshape(outTuned.UAV_State.RotorParameters.w2.Data(:,1,:)*60/(2*pi), [size(outTuned.UAV_State.RotorParameters.w2.Data(:,1,:),1),1]), 'LineWidth', Line_Width-1); hold on;
% plot(outTuned.UAV_State.RotorParameters.w3.Time, reshape(outTuned.UAV_State.RotorParameters.w3.Data(:,1,:)*60/(2*pi), [size(outTuned.UAV_State.RotorParameters.w3.Data(:,1,:),1),1]), 'LineWidth', Line_Width-1); hold on;
% plot(outTuned.UAV_State.RotorParameters.w4.Time, reshape(outTuned.UAV_State.RotorParameters.w4.Data(:,1,:)*60/(2*pi), [size(outTuned.UAV_State.RotorParameters.w4.Data(:,1,:),1),1]), 'LineWidth', Line_Width-1); hold on;
% xlabel('Time (s)','FontSize',12); ylabel('Rot Spd(rpm)','FontSize',12); title('Rotors Speed','FontSize',13); legend('Front Left','Front Right','Rear Right','Rear Left','Location','northeast'); xlim([0 Time(end)])

subplot(4,1,2)
plot(outTuned.UAV_State.RotorParameters.w1.Time, Rotor1_RPM, 'LineWidth', Line_Width-1); hold on; grid on;
plot(outTuned.UAV_State.RotorParameters.w2.Time, Rotor2_RPM, 'LineWidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.RotorParameters.w3.Time, Rotor3_RPM, 'LineWidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.RotorParameters.w4.Time, Rotor4_RPM, 'LineWidth', Line_Width-1); hold on;
xlabel('Time (s)','FontSize',12); 
ylabel('Rot Spd(rpm)','FontSize',12); 
title('Rotors Speed','FontSize',13); 
legend('Front Left','Front Right','Rear Right','Rear Left','Location','northeast'); 
xlim([0 Time(end)])

subplot(4,1,3)
plot(outTuned.UAV_State.aileron.Time, outTuned.UAV_State.aileron.Data*180/pi, 'Linewidth', Line_Width-1); hold on; grid on;
plot(outTuned.UAV_State.elevator.Time, outTuned.UAV_State.elevator.Data*180/pi, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.rudder.Time, outTuned.UAV_State.rudder.Data*180/pi, 'Linewidth', Line_Width-1); hold on; 
xlabel('Time (s)','FontSize',12); 
ylabel('Def (deg)','FontSize',12); 
title('Control Surfaces','FontSize',13); 
legend('Aileron','Elevator','Rudder','Location','northeast'); 
xlim([0 Time(end)])

subplot(4,1,4)
plot(outTuned.UAV_State.RotorParameters.Tilt1.Time, outTuned.UAV_State.RotorParameters.Tilt1.Data/pi*180, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.RotorParameters.Tilt2.Time, outTuned.UAV_State.RotorParameters.Tilt2.Data/pi*180, 'Linewidth', Line_Width-1, 'LineStyle', '--'); grid on;
xlabel('Time (s)','FontSize',12); 
ylabel('Tilt (deg)','FontSize',13); 
title('Tilt Angles','FontSize',13); 
lgd4 = legend('Frong Left','Front Right','Location','northeast');
lgd4.AutoUpdate = 'off'; 
xlim([0 Time(end)]); 

sgtitle('Tiltrotor eVTOL Flight Performance','FontSize',20,'FontWeight','bold');

modeIntervals = [  0   T1;     % Hover
                  T1   T2;     % Forward Transition
                  T2   T3;     % Fixed-Wing
                  T3   T4;     % Backward Transition
                  T4  5000];   % Hover
                        
modeColors = [0.2 0.2 0.2;     % Hover
              0.5 0.5 0.5;     % Forward Transition
              0.8 0.8 0.8;     % Fixed-Wing
              0.5 0.5 0.5;     % Backward Transition
              0.2 0.2 0.2];    % Hover

axs = findall(gcf,'Type','Axes');
axs = flipud(axs); 

for ax = transpose(axs)
    hold(ax,'on');
    yl = ylim(ax);
    ylim(ax, yl);
    for k = 1:size(modeIntervals,1)
        x1 = modeIntervals(k,1);
        x2 = modeIntervals(k,2);
        h = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], modeColors(k,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        uistack(h,'bottom'); 
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

legendHandles = [];
legendNames = {};

for k = 1:length(modeLabels_1)
    if ~ismember(modeLabels_1{k}, legendNames)
        legendHandles(end+1) = patch(NaN, NaN, modeColors(k,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        legendNames{end+1} = modeLabels_1{k}; 
    end
end
add_ax = axes('position',get(gca,'position'),'visible','off');
lgd = legend(add_ax, legendHandles, legendNames, 'Orientation', 'horizontal', 'NumColumns', 4, 'Box', 'off', 'FontSize', 13, 'Position', [0.015 0.015 1.5 0.01]); 
lgd.ItemTokenSize = [35, 18];

%% [5] Flight Dynamics Simulations (3D)
figure('Units','normalized','OuterPosition',[0.1 0.1 0.3 0.45], 'Color','w'); 
plot3(positionFeedbackData(4,:),-positionFeedbackData(5,:),-positionFeedbackData(6,:),'LineWidth',2); grid on
xlabel('North (m)'); ylabel('West (m)'); zlabel('Altitude (m)'); ylim([-200 200])

% %% [6] Thrust
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.25 0.35], 'Color','w'); 
% yyaxis right
% plot(outTuned.Flight_Mode.Time, outTuned.Flight_Mode.Data,'k','LineWidth',2); 
% xlabel('Time (s)'); ylabel('(-)'); grid on; title('Thrust & Flight Mode')
% yyaxis left
% plot(outTuned.Rotor_Force.Time, outTuned.Rotor_Force.Data(:,1)/1000, 'r', 'LineWidth', 2); grid on; hold on;
% plot(outTuned.Rotor_Force.Time, outTuned.Rotor_Force.Data(:,2)/1000, 'g','LineWidth', 2); hold on;
% plot(outTuned.Rotor_Force.Time, outTuned.Rotor_Force.Data(:,3)/1000, 'b','LineWidth', 2); hold on;
% xlabel('Time (s)'); ylabel('(kN)'); legend('Fx','Fy','Fz','location','best')
% ax = gca; ax.YAxis(1).Color = 'k'; ax.YAxis(2).Color = 'k'; ax.XAxis.Color = 'k'; 

%% [7] 2D Plots for Flight Simulation
% figure('Units','normalized','OuterPosition',[0.1 0.1 0.75 0.35], 'Color','w'); 
% plot(positionFeedbackData(4,:),-positionFeedbackData(6,:),'LineWidth',2.5); 
% xlabel('North (m)','FontSize',12); ylabel('Altitude (m)','FontSize',12); title('P4 Flight Profile','FontSize',13); xlim([-100 35000]); ylim([0 700]); grid on;

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.95 0.35], 'Color','w'); hold on; grid on;
% patch([0 50 50 0], [0 0 700 700], [0.4 0.4 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % 짙은 회색
% patch([10 16000 16000 10], [0 0 700 700], [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % 연한 회색
% patch([16000 18000 18000 16000], [0 0 700 700], [0.4 0.4 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % 짙은 회색
% plot(positionFeedbackData(4,:), -positionFeedbackData(6,:), 'k', 'LineWidth', 2.5);
% xlabel('North (m)', 'FontSize', 12); ylabel('Altitude (m)', 'FontSize', 12); title('P4 Flight Profile', 'FontSize', 13);
% xlim([-100 18100]); ylim([0 700]); grid on; box on; hold off;

%% SOP Plot
% ========= SOH & SOP parameters ========= %
C_nom_Ah   = 3*200;                                                         % 【Ref】https://www.batemo.com/products/batemo-cell-explorer/samsung-inr18650-30q/
Vmin       = 2.5*200;     
Vmax       = 3.6*200;     
Imax_dis   = 22.6*200;                                                      
R0_ref     = [];      

% ========= Pull data from outTuned ========= %
t = bt;
V = bv;
I = bi;
SOC = bsoc;
% Temp = outTuned.Battery_Data.Batt.Temperature.Data(:);
SOC = SOC/100;
dt = [0; diff(t)];                                                          % [s]
Ts_med = median(diff(t));

% ========= SOP (discharge), kW ========= %
Ns                                        = 200;                            % number of series
Np                                        = 200;                            % number of parallel
% Samsung INR18650-300 cell
% Scaling law from Prof. Jung's group at 50% soc
R0                                        = 0.012;
R1                                        = 0.004;
R2                                        = 0.0015;
C1                                        = 136.29;
C2                                        = 872.87;
Tau1                                      = R1*C1;
R                                      = ones(size(I)) * ((Ns/Np)*R0   +   (Ns/Np)*R1  + (Ns/Np)*R2);             % Battery pack resistance[Ohm]
Voc = V + I .* R;                                                           % no OCV tap, terminal + IR
% Voc                                       = 800;%V + I .* R;                     % approximate ocv = terminal voltage + I*Ruse
% Not violate minimum voltage
Iv_dis = max((Voc - Vmin) ./ R, 0);                                         % 【Ref】Review of State of Power Estimation for Li-Ion Batteries:
Pv_dis = Vmin .* Iv_dis;                                                    % Methods, Issues, and Prospects
% Not violate maximum current
Pi_dis = Voc .* Imax_dis - R .* (Imax_dis.^2);
Pi_dis = max(Pi_dis, 0);
SOP_dis_kW = min(Pv_dis, Pi_dis) / 1000;

% ========= Plot SOP ========= %
% figure('Units','normalized','OuterPosition',[0.52 0.1 0.42 0.7],'Color','w');
% tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
% nexttile; plot(t,SOC,'LineWidth',1.8); grid on; ylabel('SOC (-)'); xlabel('t (s)'); title('SOC');
% nexttile; plot(t,V,'LineWidth',1.8); grid on; ylabel('V (V)'); xlabel('t (s)'); title('Terminal Voltage');
% %nexttile; plot(t,Temp,'LineWidth',1.8); grid on; ylabel('Temp (K)'); xlabel('t (s)'); title('Temperature');
% nexttile; plot(t,Voc, 'LineWidth', 1.8);grid on; ylabel('OCV(V)');xlabel('t(s)');title('Open Circuit Voltage');
% nexttile; plot(t,SOP_dis_kW,'LineWidth',1.8); grid on; ylabel('kW'); xlabel('t (s)'); title('SOP_{dis} (max deliverable)');
 