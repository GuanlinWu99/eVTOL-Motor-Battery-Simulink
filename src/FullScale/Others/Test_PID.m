
clear all
close all
clc

test = 1;

if test == 1
    motorctrl.p = 0.2;
    motorctrl.i = 0.001;
    motorctrl.d = 0;
    motorctrl.n = 100;
else
    motorctrl.p = 0.003;
    motorctrl.i = 0.001;
    motorctrl.d = 0;
    motorctrl.n = 100;
end

const = load_const();
[uavParams, HEV_Param] = load_vtol_dynamics_7000lb(const);

Thrust          = 7800;    % [N] (when vertical takeoff)
RPM             = 7000;
Limit           = 500;     % [Nm]

required_torque = Thrust*(uavParams.rotor.Cq/uavParams.rotor.Ct)*(uavParams.geom.PropDiameter/2);     
compen_torque   = 450; 
step_index      = 5;

keyboard;

%%
figure(2)
set(gcf, 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.8 0.5]);
subplot(2,5,1)
plot(MotorRPM.Time, MotorRPM.Data,'LineWidth',2); hold on; grid on;
plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data,'-.','LineWidth',2); hold on;
title('Motor Speed (RPM)')
ylabel('RPM')
legend('RPM','RPM cmd')

subplot(2,5,2)
plot(MotorRPM.Time, MotorRPM.Data*2*pi/60,'LineWidth',2); hold on; grid on;
plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data*2*pi/60,'-.','LineWidth',2); hold on;
legend('rad/s','rad/s cmd')
ylabel('(rad/s)')
title('Motor Speed (rad/s)')

subplot(2,5,3)
plot(Motor_Current_Test.Time, Motor_Current_Test.Data,'LineWidth',2); grid on;
legend('Motor Current')
title('Motor Current')
ylabel('(A)')

subplot(2,5,4)
plot(Torque_Command_Test.Time, Torque_Command_Test.Data,'LineWidth',2); hold on; grid on;
legend('Motor Torque')
ylabel('(Nm)')
title('Motor Torque')
ylim([0 Torque_Command_Test.Data(1)])

subplot(2,5,5)
plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,1)/1000,'LineWidth',2); grid on;
legend('Motor Electrical Power')
title('Motor Power')
ylabel('(kW)')

subplot(2,5,6)
plot(Battery_Output_Data.Batt.SOC____.Time, Battery_Output_Data.Batt.SOC____.Data,'LineWidth',2);
grid on
title('SOC')
legend('SOC')
ylabel('(%)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.SOC____.Time(end)])

subplot(2,5,7)
plot(Battery_Output_Data.Batt.Voltage__V_.Time, Battery_Output_Data.Batt.Voltage__V_.Data,'LineWidth',2);
grid on
title('Voltage')
legend('Battery Voltage')
ylabel('(V)')
xlabel('Time (s)')
xlim([Battery_Output_Data.Batt.Voltage__V_.Time(1) Battery_Output_Data.Batt.Voltage__V_.Time(end)])

subplot(2,5,8)
plot(Battery_Output_Data.Batt.Current__A_.Time, Battery_Output_Data.Batt.Current__A_.Data,'LineWidth',2);
grid on
title('Current')
legend('Battery Pack')
ylabel('(A)')
xlabel('Time (s)')
xlim([Battery_Output_Data.Batt.Current__A_.Time(1) Battery_Output_Data.Batt.Current__A_.Time(end)])

subplot(2,5,9)
plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
grid on
title('C-rate')
legend('Battery C-rate')
ylabel('(-)')
xlabel('Time (s)')
xlim([Battery_Output_Data.Batt.C_rate.Time(1) Battery_Output_Data.Batt.C_rate.Time(end)])

subplot(2,5,10)
plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,3)/10^5,'LineWidth',2); hold on; grid on;
title('Power')
legend('Battery Pack Power')
xlabel('Time (s)')
ylabel('(MW)')

keyboard;

%%
% figure(1);
% set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 0.3 0.9]);
% subplot(5,2,1)
% plot(MotorRPM.Time, MotorRPM.Data,'LineWidth',2); hold on; grid on;
% plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data,'LineWidth',2); hold on;
% title('Motor Data')
% ylabel('RPM')
% legend('RPM','RPM cmd')
% 
% subplot(5,2,3)
% plot(MotorRPM.Time, MotorRPM.Data*2*pi/60,'LineWidth',2); hold on; grid on;
% plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data*2*pi/60,'LineWidth',2); hold on;
% legend('rad/s','rad/s cmd')
% ylabel('(rad/s)')
% title('Motor Speed (rad/s)')
% 
% subplot(5,2,5)
% plot(Torque_Command_Test.Time, Torque_Command_Test.Data,'LineWidth',2); hold on; grid on;
% legend('Torque Command')
% ylabel('(Nm)')
% title('Torque')
% ylim([0 Torque_Command_Test.Data(1)])
% 
% subplot(5,2,7)
% plot(Motor_Current_Test.Time, Motor_Current_Test.Data,'LineWidth',2); grid on;
% xlabel('Time (s)')
% legend('Motor Current')
% title('Current')
% ylabel('(A)')
% 
% subplot(5,2,9)
% plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,1)/1000,'LineWidth',2); grid on;
% legend('Motor Power')
% title('Motor Power')
% xlabel('Time (s)')
% ylabel('(kW)')
% 
% subplot(5,2,2)
% plot(Battery_Output_Data.Batt.SOC____.Time, Battery_Output_Data.Batt.SOC____.Data,'LineWidth',2);
% grid on
% title('Battery Pack Data')
% legend('Battery SOC')
% ylabel('(%)')
% xlim([0 Battery_Output_Data.Batt.SOC____.Time(end)])
% 
% subplot(5,2,4)
% plot(Battery_Output_Data.Batt.Voltage__V_.Time, Battery_Output_Data.Batt.Voltage__V_.Data,'LineWidth',2);
% grid on
% title('Voltage')
% legend('Battery Voltage')
% ylabel('(V)')
% xlim([0 Battery_Output_Data.Batt.Voltage__V_.Time(end)])
% 
% subplot(5,2,6)
% plot(Battery_Output_Data.Batt.Current__A_.Time, Battery_Output_Data.Batt.Current__A_.Data,'LineWidth',2);
% grid on
% title('Current')
% legend('Battery Pack')
% ylabel('(A)')
% xlim([0 Battery_Output_Data.Batt.Current__A_.Time(end)])
% 
% subplot(5,2,8)
% plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
% grid on
% title('C-rate')
% legend('Battery C-rate')
% ylabel('(-)')
% xlabel('Time (s)')
% xlim([0 Battery_Output_Data.Batt.C_rate.Time(end)])

%%
figure(1);
subplot(5,1,1)
plot(Battery_Output_Data.Batt.SOC____.Time, Battery_Output_Data.Batt.SOC____.Data,'LineWidth',2);
grid on
title('SOC')
ylabel('(%)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.SOC____.Time(end)])

subplot(5,1,2)
plot(Battery_Output_Data.Batt.Voltage__V_.Time, Battery_Output_Data.Batt.Voltage__V_.Data,'LineWidth',2);
grid on
title('Voltage')
ylabel('(V)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.Voltage__V_.Time(end)])

subplot(5,1,3)
plot(Battery_Output_Data.Batt.Current__A_.Time, Battery_Output_Data.Batt.Current__A_.Data,'LineWidth',2);
grid on
title('Current')
ylabel('(A)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.Current__A_.Time(end)])

subplot(5,1,4)
plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
grid on
title('C-rate')
ylabel('(-)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.C_rate.Time(end)])

subplot(5,1,5)
plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
grid on
title('C-rate')
ylabel('(-)')
xlabel('Time (s)')
xlim([0 Battery_Output_Data.Batt.C_rate.Time(end)])

keyboard

%%
figure(2);
plot(MotorRPM.Time, MotorRPM.Data,'LineWidth',2); hold on; grid on;
plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data,'LineWidth',2); hold on;
legend('RPM cmd','RPM')

figure(3);
plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,1)/1000,'LineWidth',2); hold on; grid on;
legend('Motor Power')
xlabel('Time (s)')
ylabel('(kW)')

figure(4);
plot(Motor_Current_Test.Time, Motor_Current_Test.Data,'LineWidth',2); hold on; grid on;
xlabel('Time (s)')
ylabel('(A)')

figure(6);
plot(Torque_Command_Test.Time, Torque_Command_Test.Data,'LineWidth',2); hold on; grid on;
xlabel('Time (s)')
legend('Torque Command')
ylabel('(Nm)')

keyboard

