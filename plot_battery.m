% figure;
% 
% % 1. 电机转速（RPM）
% subplot(6,1,1);
% plot(Batt_meas.Mot_RPM.Time, Batt_meas.Mot_RPM.Data, 'LineWidth', 1.5);
% ylabel('Motor RPM');
% title('Motor Speed');
% grid on;
% 
% % 2. 电池功率
% subplot(6,1,2);
% plot(Batt_power.Time, Batt_power.Data(:, 1), 'LineWidth', 1.5);
% ylabel('Power (W)');
% title('Battery Power');
% grid on;
% 
% % 3. 电池温度
% subplot(6,1,3);
% plot(Batt_Temp.Time, Batt_Temp.Data, 'LineWidth', 1.5);
% ylabel('Temp (°C)');
% title('Battery Temperature');
% grid on;
% 
% % 4. 电流
% subplot(6,1,4);
% plot(Batt_meas.Batt.Current__A_.Time, Batt_meas.Batt.Current__A_.Data, 'LineWidth', 1.5);
% ylabel('Current (A)');
% title('Battery Current');
% grid on;
% 
% % 5. 电压
% subplot(6,1,5);
% plot(Batt_meas.Batt.Voltage__V_.Time, Batt_meas.Batt.Voltage__V_.Data, 'LineWidth', 1.5);
% ylabel('Voltage (V)');
% title('Battery Voltage');
% grid on;
% 
% % 6. SOC
% subplot(6,1,6);
% plot(Batt_meas.Batt.SOC____.Time, Batt_meas.Batt.SOC____.Data, 'LineWidth', 1.5);
% ylabel('SOC');
% xlabel('Time (s)');
% title('State of Charge');
% grid on;
% % 
% % 
% % % plot input
% % figure;
% % plot(thrust.Time, thrust.Data, 'LineWidth', 1.5);
% % ylabel('Torque(N·m)');
% % xlabel('Time(s)');
% % title('Motor Torque');
% % grid on;
% % 
figure;

% 1. 电机转速（RPM）
subplot(6,1,1);
plot(Batt_meas.Mot_RPM.Time, Batt_meas.Mot_RPM.Data, 'LineWidth', 1.5);
ylabel('Motor RPM');
title('Motor Speed');
grid on;

% 2. 电池功率
subplot(6,1,2);
plot(Batt_power.Time, Batt_power.Data(:, 1), 'LineWidth', 1.5);
ylabel('Power (W)');
title('Battery Power');
grid on;

% 3. 电池温度
subplot(6,1,3);
% plot(Batt_Temp.Time, Batt_Temp.Data, 'LineWidth', 1.5);
plot(Batt_meas.Batt.signal5.signal1.Time, Batt_meas.Batt.signal5.signal1.Data-273,  'LineWidth', 1.5)
ylabel('Temp (°C)');
title('Battery Temperature');
grid on;

% 4. 电流
subplot(6,1,4);
plot(Batt_meas.Batt.Current__A_.Time, Batt_meas.Batt.Current__A_.Data, 'LineWidth', 1.5);
ylabel('Current (A)');
title('Battery Current');
grid on;

% 5. 电压
subplot(6,1,5);
plot(Batt_meas.Batt.Voltage__V_.Time, Batt_meas.Batt.Voltage__V_.Data, 'LineWidth', 1.5);
ylabel('Voltage (V)');
title('Battery Voltage');
grid on;

% 6. SOC
subplot(6,1,6);
plot(Batt_meas.Batt.SOC____.Time, Batt_meas.Batt.SOC____.Data, 'LineWidth', 1.5);
ylabel('SOC');
xlabel('Time (s)');
title('State of Charge');
grid on;


% % plot input
% figure;
% plot(thrust.Time, thrust.Data, 'LineWidth', 1.5);
% ylabel('Torque(N·m)');
% xlabel('Time(s)');
% title('Motor Torque');
% grid on;
% 
% % plot Heat
% figure;
% subplot(3,1,1);
% plot(Batt_meas.Batt.signal5.signal1.Time, Batt_meas.Batt.signal5.signal1.Data-273,  'LineWidth', 1.5);
% ylabel('signal1');
% xlabel('Time(s)');
% title('signal1');
% grid on;
% 
% subplot(3,1,2);
% plot(Batt_meas.Batt.signal5.signal2.Time, Batt_meas.Batt.signal5.signal2.Data,  'LineWidth', 1.5);
% ylabel('signal2');
% xlabel('Time(s)');
% title('signal2');
% grid on;
% 
% subplot(3,1,3);
% plot(Batt_meas.Batt.signal5.signal3.Time, Batt_meas.Batt.signal5.signal3.Data,  'LineWidth', 1.5);
% ylabel('signal3');
% xlabel('Time(s)');
% title('signal3');
% grid on;