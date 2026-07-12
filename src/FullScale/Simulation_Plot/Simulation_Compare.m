
%% Read simulation data
Profile = 1;

% With Prof. Jung's battery model
load("P1_MTOW_5600_Wind_0_Temp_20.mat");
outTuned_Jung = outTuned;

% With Prototype battery model
load("Profile_P1_5600_Wind_0_Success.mat");
outTuned_Prototype = outTuned;

L_Prototype      = length(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time);
L_Jung           = length(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time);
Time             = outTuned_Prototype.Battery_Data.Batt.SOC____.Time;

% Determine the transition time 
Flight_Condition = string(outTuned.Flight_Mode.Data);
changeIdx        = find(Flight_Condition(1:end-1) ~= Flight_Condition(2:end));
transitions      = struct('Index', {}, 'From', {}, 'To', {});

for i = 1:length(changeIdx)
    transitions(i).Index = changeIdx(i);
    transitions(i).From = Flight_Condition(changeIdx(i));
    transitions(i).To = Flight_Condition(changeIdx(i)+1);
end

T1  =  transitions(1).Index*size(outTuned.UAV_State.Vb.Data,3)/size(outTuned.Flight_Mode.Data,1)*0.001;       % Hover to Transition
T2  =  transitions(2).Index*size(outTuned.UAV_State.Vb.Data,3)/size(outTuned.Flight_Mode.Data,1)*0.001;       % Transition to Fixed-Wing
T3  =  transitions(3).Index*size(outTuned.UAV_State.Vb.Data,3)/size(outTuned.Flight_Mode.Data,1)*0.001;       % Fixed-Wing to Transition
T4  =  transitions(4).Index*size(outTuned.UAV_State.Vb.Data,3)/size(outTuned.Flight_Mode.Data,1)*0.001;       % Transition to Hover

%% Prototype Case
Pack_Power_Prototype   =  outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Data .* outTuned_Prototype.Battery_Data.Batt.Current__A_.Data;
Pack_Energy_Prototype  =  zeros(L_Prototype,1);

for i = 2 : length(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time)
    Energy_tmp_Prototype = Pack_Power_Prototype(i)*0.001;
    Pack_Energy_Prototype(i) = Energy_tmp_Prototype + Pack_Energy_Prototype(i-1); 
end

%% Prof. Jung's Case
Pack_Power_Jung  = outTuned_Jung.Battery_Data.Batt.Voltage__V_.Data .* outTuned_Jung.Battery_Data.Batt.Current__A_.Data;
Pack_Energy_Jung = zeros(L_Jung ,1);

for i = 2 : length(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time)
    Energy_tmp_Jung = Pack_Power_Jung(i)*0.001;
    Pack_Energy_Jung(i) = Energy_tmp_Jung + Pack_Energy_Jung(i-1); 
end

% Plot settings ...
Font = 13; 
Buffer = 10; 
Transparency = 0.6; 
Line_Width = 2.5;

Battery_Capacity = (HEV_Param.Capacity*HEV_Param.Np*HEV_Param.Ns*3.6)/1000;
disp(['🔋 Battery Pack Capacity: ', num2str(Battery_Capacity), ' (kWh)']);

%% Plot Battery Pack Electrical Performance
figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.8], 'Color','w'); 
switch Profile
    case {1, 3, 7, 10}
        subplot(3,3,1)
        plot(outTuned_Prototype.Battery_Data.Batt.SOC____.Time, outTuned_Prototype.Battery_Data.Batt.SOC____.Data - (24.17/Battery_Capacity)*100, 'LineWidth', Line_Width); hold on;
        plot(outTuned_Jung.Battery_Data.Batt.SOC____.Time, outTuned_Jung.Battery_Data.Batt.SOC____.Data - (24.17/Battery_Capacity)*100, '-.', 'LineWidth', Line_Width); grid on;
        title('SOC','FontSize',Font); 
        xlabel('Time (s)','FontSize',12);
        ylabel('(%)','FontSize',12); 
        xlim([0 Time(end)]); 
        ylim([50 100])
        
        subplot(3,3,2)
        plot(outTuned_Prototype.Battery_Data.Batt.C_rate.Time, outTuned_Prototype.Battery_Data.Batt.C_rate.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.C_rate.Time, outTuned_Jung.Battery_Data.Batt.C_rate.Data,'-.','LineWidth',Line_Width); 
        title('C-rate','FontSize',Font);  
        xlabel('Time (s)','FontSize',12);
        ylabel('(-)','FontSize',12);
        xlim([0 Time(end)])
        
        subplot(3,3,3)
        plot(outTuned_Prototype.Battery_Data.Batt.Current__A_.Time, outTuned_Prototype.Battery_Data.Batt.Current__A_.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Current__A_.Time, outTuned_Jung.Battery_Data.Batt.Current__A_.Data,'-.','LineWidth',Line_Width); 
        title('Current','FontSize',Font);  
        xlabel('Time (s)','FontSize',12); 
        ylabel('(A)','FontSize',12);
        xlim([0 Time(end)])
        
        subplot(3,3,4)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, outTuned_Jung.Battery_Data.Batt.Voltage__V_.Data,'-.','LineWidth',Line_Width); grid on;
        title('Voltage','FontSize',Font);  
        xlabel('Time (s)','FontSize',12); 
        ylabel('(V)','FontSize',12);
        xlim([0 Time(end)])
        
        switch Profile
            case 2
                ylim([620 730])
            case 3
                ylim([520 730])
        end
        
        subplot(3,3,5)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, Pack_Power_Prototype/1000,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, Pack_Power_Jung/1000,'-.','LineWidth',Line_Width); grid on;
        title('Required Power','FontSize',Font); 
        xlabel('Time (s)','FontSize',12);  
        ylabel('(kW)','FontSize',12); 
        xlim([0 Time(end)])
        
        subplot(3,3,6)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, Pack_Energy_Prototype/(3.6*10^6) + 24.17,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, Pack_Energy_Jung/(3.6*10^6) + 24.17,'-.','LineWidth',Line_Width); grid on;
        title('Cumulative Energy Consumption','FontSize',Font);  
        xlabel('Time (s)','FontSize',12);
        ylabel('(kWh)','FontSize',12);
        xlim([0 Time(end)-Buffer])

        switch Profile
            case {3, 10}
                ylim([0 100])    
        end

    case {2, 4, 5, 6, 8, 9}
        subplot(3,3,1)
        plot(outTuned_Prototype.Battery_Data.Batt.SOC____.Time, outTuned_Prototype.Battery_Data.Batt.SOC____.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.SOC____.Time, outTuned_Jung.Battery_Data.Batt.SOC____.Data,'-.','LineWidth',Line_Width); 
        title('SOC','FontSize',Font); 
        xlabel('Time (s)','FontSize',12);
        ylabel('(%)','FontSize',12); 
        xlim([0 Time(end)]); 
        ylim([50 100])
        
        subplot(3,3,2)
        plot(outTuned_Prototype.Battery_Data.Batt.C_rate.Time, outTuned_Prototype.Battery_Data.Batt.C_rate.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.C_rate.Time, outTuned_Jung.Battery_Data.Batt.C_rate.Data,'-.','LineWidth',Line_Width); 
        title('C-rate','FontSize',Font);  
        xlabel('Time (s)','FontSize',12);
        ylabel('(-)','FontSize',12);
        xlim([0 Time(end)])
        
        subplot(3,3,3)
        plot(outTuned_Prototype.Battery_Data.Batt.Current__A_.Time, outTuned_Prototype.Battery_Data.Batt.Current__A_.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Current__A_.Time, outTuned_Jung.Battery_Data.Batt.Current__A_.Data,'-.','LineWidth',Line_Width); 
        title('Current','FontSize',Font);  
        xlabel('Time (s)','FontSize',12); 
        ylabel('(A)','FontSize',12);
        xlim([0 Time(end)])
        
        subplot(3,3,4)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Data,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, outTuned_Jung.Battery_Data.Batt.Voltage__V_.Data,'-.','LineWidth',Line_Width); grid on;
        title('Voltage','FontSize',Font);  
        xlabel('Time (s)','FontSize',12); 
        ylabel('(V)','FontSize',12);
        xlim([0 Time(end)])
        
        switch Profile
            case 2
                ylim([620 730])
            case 3
                ylim([520 730])
            case {4, 5}
                ylim([550 730])
            case 9
                ylim([620 730])
            case 8
                ylim([620 740])
        end

        subplot(3,3,5)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, Pack_Power_Prototype/1000,'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, Pack_Power_Jung/1000,'-.','LineWidth',Line_Width); grid on;
        title('Required Power','FontSize',Font); 
        xlabel('Time (s)','FontSize',12);  
        ylabel('(kW)','FontSize',12); 
        xlim([0 Time(end)])
        
        subplot(3,3,6)
        plot(outTuned_Prototype.Battery_Data.Batt.Voltage__V_.Time, Pack_Energy_Prototype/(3.6*10^6),'LineWidth',Line_Width); grid on; hold on;
        plot(outTuned_Jung.Battery_Data.Batt.Voltage__V_.Time, Pack_Energy_Jung/(3.6*10^6),'-.','LineWidth',Line_Width); grid on;
        title('Cumulative Energy Consumption','FontSize',Font);  
        xlabel('Time (s)','FontSize',12);
        ylabel('(kWh)','FontSize',12);
        xlim([0 Time(end)-Buffer])
        ylim([0 100])

        switch Profile
            case {4, 5}
                ylim([0 170])
            case 9
                ylim([0 80])   
            case 8
                ylim([0 120])
        end
end

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

lgd = legend(legendHandles, legendNames, 'Orientation', 'vertical', 'Box', 'off', 'FontSize', 15, 'Position', [0.005 0.11 1.25 0.2]); 
lgd.ItemTokenSize = [50, 30];
