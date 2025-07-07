figure;
tl = tiledlayout(9,1,'TileSpacing','compact','Padding','compact');


% Battery Power
nexttile; plot(Batt_power.Time, Batt_power.Data(:,1),'LineWidth',1.5);
ylabel('W'); title('Battery Power'); grid on;

% Battery Temperature
nexttile; plot(Batt_meas.Batt.signal5.Time, ...
               Batt_meas.Batt.signal5.Data-273,'LineWidth',1.5);
ylabel('°C'); title('Battery Temperature'); grid on;

% Currentsolverprofiler.open('Test_FourMotor')
nexttile; plot(Batt_meas.Batt.Current__A_.Time, ...
               Batt_meas.Batt.Current__A_.Data,'LineWidth',1.5);
ylabel('A'); title('Battery Current'); grid on;

% Voltage
nexttile; plot(Batt_meas.Batt.Voltage__V_.Time, ...
               Batt_meas.Batt.Voltage__V_.Data,'LineWidth',1.5);
ylabel('V'); title('Battery Voltage'); grid on;

% SOC
nexttile; plot(Batt_meas.Batt.SOC____.Time, ...
               Batt_meas.Batt.SOC____.Data,'LineWidth',1.5);
ylabel('SOC'); xlabel('Time (s)'); title('State of Charge'); grid on;


% Motor 1-4 RPM
nexttile; plot(Batt_meas.Mot_RPM_1.Time, ...
               Batt_meas.Mot_RPM_1.Data,'LineWidth',1.5);
ylabel('RPM'); title('Motor 1 Speed'); grid on;

nexttile; plot(Batt_meas.Mot_RPM_2.Time, ...
               Batt_meas.Mot_RPM_2.Data,'LineWidth',1.5);
ylabel('RPM'); title('Motor 2 Speed'); grid on;

nexttile; plot(Batt_meas.Mot_RPM_3.Time, ...
               Batt_meas.Mot_RPM_3.Data,'LineWidth',1.5);
ylabel('RPM'); title('Motor 3 Speed'); grid on;

nexttile; plot(Batt_meas.Mot_RPM_4.Time, ...
               Batt_meas.Mot_RPM_4.Data,'LineWidth',1.5);
ylabel('RPM'); title('Motor 4 Speed'); grid on;


% % plot input
% figure;
% plot(thrust1.Time, thrust1.Data, 'LineWidth', 1.5);
% ylabel('Torque(N·m)');
% xlabel('Time(s)');
% title('Motor Torque');
% grid on;
