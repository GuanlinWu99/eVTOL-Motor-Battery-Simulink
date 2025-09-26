
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
RPM             = 7000;    % [-]
Limit           = 500;     % [Nm]

Ts              = 0.001;

required_torque = Thrust*(uavParams.rotor.Cq/uavParams.rotor.Ct)*(uavParams.geom.PropDiameter/2);     
step_index      = 5;

keyboard;

%%
figure(1)
set(gcf, 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.45 0.5]);
subplot(3,3,1)
plot(MotorRPM.Time, MotorRPM.Data,'LineWidth',2); hold on; grid on;
plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data,'-.','LineWidth',2); hold on;
title('Motor Speed (RPM)')
ylabel('RPM')
legend('RPM','RPM cmd','Location','southeast')

subplot(3,3,2)
plot(MotorRPM.Time, MotorRPM.Data*2*pi/60,'LineWidth',2); hold on; grid on;
plot(MotorRPM_Reference.Time, MotorRPM_Reference.Data*2*pi/60,'-.','LineWidth',2); hold on;
legend('rad/s','rad/s cmd','Location','southeast')
ylabel('(rad/s)')
title('Motor Speed (rad/s)')

subplot(3,3,3)
plot(Motor_Current_Test.Time, Motor_Current_Test.Data,'LineWidth',2); grid on;
legend('Motor Current')
title('Motor Current')
ylabel('(A)')

subplot(3,3,4)
plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,1)/1000,'LineWidth',2); grid on;
legend('Motor Electrical Power')
title('Motor Power')
ylabel('(kW)')

subplot(3,3,5)
plot(Battery_Output_Data.Batt.SOC____.Time, Battery_Output_Data.Batt.SOC____.Data,'LineWidth',2);
grid on
title('SOC')
legend('SOC')
ylabel('(%)')
xlim([0 Battery_Output_Data.Batt.SOC____.Time(end)])

subplot(3,3,6)
plot(Battery_Output_Data.Batt.Voltage__V_.Time, Battery_Output_Data.Batt.Voltage__V_.Data,'LineWidth',2);
grid on
title('Voltage')
legend('Battery Voltage')
ylabel('(V)')
xlim([Battery_Output_Data.Batt.Voltage__V_.Time(1) Battery_Output_Data.Batt.Voltage__V_.Time(end)])

subplot(3,3,7)
plot(Battery_Output_Data.Batt.Current__A_.Time, Battery_Output_Data.Batt.Current__A_.Data,'LineWidth',2);
grid on
title('Current')
legend('Battery Pack')
ylabel('(A)')
xlabel('Time (s)')
xlim([Battery_Output_Data.Batt.Current__A_.Time(1) Battery_Output_Data.Batt.Current__A_.Time(end)])

subplot(3,3,8)
plot(Battery_Output_Data.Batt.C_rate.Time, Battery_Output_Data.Batt.C_rate.Data, 'Linewidth', 2)
grid on
title('C-rate')
legend('Battery C-rate')
ylabel('(-)')
xlabel('Time (s)')
xlim([Battery_Output_Data.Batt.C_rate.Time(1) Battery_Output_Data.Batt.C_rate.Time(end)])

subplot(3,3,9)
plot(Battery_Output_Data.signal6.Time, Battery_Output_Data.signal6.Data(:,3)/10^5,'LineWidth',2); hold on; grid on;
title('Power')
legend('Battery Pack Power')
xlabel('Time (s)')
ylabel('(MW)')

sgtitle('Test Four Motor PID Speed Control','FontSize',15,'FontWeight','bold');

keyboard;

%% 