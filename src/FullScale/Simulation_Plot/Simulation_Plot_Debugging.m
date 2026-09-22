
Line_Width = 3;
positionFeedbackData = squeeze(outTuned.PositionCmdFdbk.signals.values);

RGB = orderedcolors("gem");
H = rgb2hex(RGB);

ref_2d_traj     =   zeros(3,size(TransitionMission,2));
for idx = 1:size(ref_2d_traj,2)
    ref_2d_traj(:,idx) = TransitionMission(idx).position;
end
ref_2d_traj(:,3) = [];

ref_air_spd     =   zeros(2,2*size(TransitionMission,2)-2);
for idx = 1:size(TransitionMission,2)-1
    ref_air_spd(1,2*idx-1)  = sqrt(TransitionMission(idx).position(1).^2+TransitionMission(idx).position(2).^2);
    ref_air_spd(1,2*idx)    = sqrt(TransitionMission(idx+1).position(1).^2+TransitionMission(idx+1).position(2).^2);
    ref_air_spd(2,2*idx-1)  = TransitionMission(idx+1).params(4)/const.kts2mps;
    ref_air_spd(2,2*idx)    = TransitionMission(idx+1).params(4)/const.kts2mps;
end
ref_air_spd(:,4:5) = [];

air_spd = sqrt(reshape(outTuned.UAV_State.Vb.Data(:,1,:), [], 1).^2 + ...
               reshape(outTuned.UAV_State.Vb.Data(:,2,:), [], 1).^2 + ...
               reshape(outTuned.UAV_State.Vb.Data(:,3,:), [], 1).^2)/const.kts2mps;

Roll_ref = interp1(outTuned.Attitude_Ref.Time, outTuned.Attitude_Ref.Data(:,1)/pi*180, outTuned.UAV_State.Euler.Time, 'linear', 'extrap');
Pitch_ref = interp1(outTuned.Attitude_Ref.Time, outTuned.Attitude_Ref.Data(:,2)/pi*180, outTuned.UAV_State.Euler.Time, 'linear', 'extrap');
Yaw_ref = interp1(outTuned.Attitude_Ref.Time, outTuned.Attitude_Ref.Data(:,3)/pi*180, outTuned.UAV_State.Euler.Time, 'linear', 'extrap');

%% Simulation Results
figure(1)
set(gcf,'Color','w','Position',[300 300 1300 800])
subplot(2,3,1)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'r', 'Linewidth', Line_Width-1); hold on; grid on;
plot(outTuned.UAV_State.Euler.Time, Roll_ref, 'r-.', 'Linewidth', Line_Width-1); hold on; 
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'g', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.Euler.Time, Pitch_ref, 'g-.', 'Linewidth', Line_Width-1); hold on; 
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'b', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.Euler.Time, Yaw_ref, 'b-.', 'Linewidth', Line_Width-1); hold on; 
xlabel('Time (s)','FontSize',12); 
ylabel('Att (deg)','FontSize',12); 
title('Euler Angles','FontSize',13); 
legend('Roll','Roll-ref','Pitch','Pitch-ref','Yaw','Yaw-ref','Location','best'); 

subplot(2,3,2)
plot(outTuned.Yaw_Rate_Cmd.Time, outTuned.Yaw_Rate_Cmd.Data, 'b', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Yaw_Rate_Measure.Time, squeeze(outTuned.Yaw_Rate_Measure.Data), 'b-.', 'Linewidth', Line_Width-1); hold on; grid on;
xlabel('Time (s)','FontSize',12); 
ylabel('Rate (rad/s)','FontSize',12);
%xlim([0 85]);
%ylim([-1 4]);
title('Yaw Rates','FontSize',13); 
legend('Yaw-Rate-Measure','Yaw-Rate-ref','Location','best'); 

subplot(2,3,3)
plot(outTuned.Yaw_Torque_Cmd.Time, outTuned.Yaw_Torque_Cmd.Data, 'Linewidth', Line_Width-1); hold on; grid on;
xlabel('Time (s)','FontSize',12); 
title('Yaw Torque Cmd','FontSize',13); 

subplot(2,3,4)
plot(outTuned.Roll_Rate_Cmd.Time, outTuned.Roll_Rate_Cmd.Data, 'r', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Roll_Rate_Measure.Time, squeeze(outTuned.Roll_Rate_Measure.Data), 'r-.', 'Linewidth', Line_Width-1); hold on; grid on;
xlabel('Time (s)','FontSize',12); 
ylabel('Rate (rad/s)','FontSize',12); 
title('Roll Rates (Angular Velocity)','FontSize',13); 
legend('Roll-Rate-Measure','Roll-Rate-ref','Location','best'); 

subplot(2,3,5)
plot(outTuned.Pitch_Rate_Cmd.Time, outTuned.Pitch_Rate_Cmd.Data, 'g', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Pitch_Rate_Measure.Time, squeeze(outTuned.Pitch_Rate_Measure.Data), 'g-.', 'Linewidth', Line_Width-1); hold on; grid on;
xlabel('Time (s)','FontSize',12); 
ylabel('Rate (rad/s)','FontSize',12); 
title('Pitch Rates (Angular Velocity)','FontSize',13); 
legend('Pitch-Rate-Measure','Pitch-Rate-ref','Location','best'); 

subplot(2,3,6)
plot(outTuned.Motor_RPM.w1.Time, outTuned.Motor_RPM.w1.Data, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Motor_RPM.w2.Time, outTuned.Motor_RPM.w2.Data, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Motor_RPM.w3.Time, outTuned.Motor_RPM.w3.Data, 'Linewidth', Line_Width-1); hold on;
plot(outTuned.Motor_RPM.w4.Time, outTuned.Motor_RPM.w4.Data, 'Linewidth', Line_Width-1); hold on; grid on;
%xlim([0 85]);
xlabel('Time (s)','FontSize',12); 
ylabel('Motor RPM','FontSize',12); 
title('Motor RPM','FontSize',13); 
legend('W1','W2','W3','W4','Location','best'); 

%%
figure(2)
set(gcf,'Color','w','Position',[300 300 900 800])
subplot(2,2,1)
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(1,:,:),[size(outTuned.UAV_State.Euler.Data(1,:,:),3),1])/pi*180, 'r', 'Linewidth', Line_Width-1); hold on; grid on;
plot(outTuned.UAV_State.Euler.Time, Roll_ref, 'r-.', 'Linewidth', Line_Width-1); hold on; 
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(2,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'g', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.Euler.Time, Pitch_ref, 'g-.', 'Linewidth', Line_Width-1); hold on; 
plot(outTuned.UAV_State.Euler.Time, reshape(outTuned.UAV_State.Euler.Data(3,:,:),[size(outTuned.UAV_State.Euler.Data(2,:,:),3),1])/pi*180, 'b', 'Linewidth', Line_Width-1); hold on;
plot(outTuned.UAV_State.Euler.Time, Yaw_ref, 'b-.', 'Linewidth', Line_Width-1); hold on; 
xlabel('Time (s)','FontSize',12); 
ylabel('Att (deg)','FontSize',12); 
title('Euler Angles','FontSize',13); 
legend('Roll','Roll_ref','Pitch','Pitch_ref','Yaw','Yaw_ref','Location','best'); 

subplot(2,2,2);
plot(ref_2d_traj(1,:),-ref_2d_traj(3,:), 'LineWidth', Line_Width, 'LineStyle',"--",'Color',H(7)); hold on;
plot(positionFeedbackData(4,:), -positionFeedbackData(6,:), 'LineWidth', 1.5,'Color',H(1)); grid on; box on; 
xlabel('Distance (North, m)', 'FontSize', 12); 
ylabel('Altitude (m)', 'FontSize', 12);
title('Position','FontSize',13); 
legend('Mission Profile', 'Actual Trajectory','Location','best');

subplot(2,2,3);
plot(positionFeedbackData(4,:), air_spd(1:5:end,:), 'LineWidth', 1.5,'Color',H(1)); grid on;
xlabel('Distance (North, m)', 'FontSize', 12); 
ylabel('Airspeed (kts)', 'FontSize', 12); 
legend('Actual Trajectory','Location','best');
title('Flight Speed Profile', 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,2,4);
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,1,:), [size(outTuned.UAV_State.Vb.Data(:,1,:),3),1]), 'LineWidth', 2); hold on;
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,2,:), [size(outTuned.UAV_State.Vb.Data(:,2,:),3),1]), 'LineWidth', 2); hold on;
plot(outTuned.UAV_State.Vb.Time, reshape(outTuned.UAV_State.Vb.Data(:,3,:), [size(outTuned.UAV_State.Vb.Data(:,3,:),3),1]), 'LineWidth', 2); grid on;
xlabel('Time (s)','FontSize',12); 
ylabel('Airspeed (kts)', 'FontSize', 12); 
legend('Vx','Vy','Vz','Location','northwest');
xlim([0 85]);
title('Body Velocity', 'FontSize', 12, 'FontWeight', 'bold');

sgtitle('Tiltrotor eVTOL Flight Performance','FontSize',20, 'FontWeight', 'bold');

%%
% figure(3)
% plot(outTuned.Torque_Cmd.w_phi.Time,outTuned.Torque_Cmd.w_phi.Data, 'LineWidth', 2); hold on; grid on;
% plot(outTuned.Torque_Cmd.w_theta.Time,outTuned.Torque_Cmd.w_theta.Data, 'LineWidth', 2); hold on; 
% plot(outTuned.Torque_Cmd.w_psi.Time,outTuned.Torque_Cmd.w_psi.Data, 'LineWidth', 2); hold on; 
% xlabel('Time (s)','FontSize',12); 
% legend('w_1','w_2','w_3','Location','northwest');
