function exampleHelperPlotFixedWingControlTrackingResults(out,FixedWingMission)
% exampleHelperPlotFixedWingControlTrackingResults plot results for fixed
% wing control analysis.

% Copyright 2023 The MathWorks, Inc.

%% Plot of Altitude and Airspeed
figure;
tiledlayout(2,1);
% Plot Airspeed
nexttile;
plot(out.FixedWingCmdFdbk.time,out.FixedWingCmdFdbk.signals.values(:,1),LineWidth=2);
hold on;
plot(out.FixedWingCmdFdbk.time,out.FixedWingCmdFdbk.signals.values(:,4),LineWidth=2);
grid on;
ylabel('Airspeed');
title('Command vs Output');
legend('Command','Output');
% Plot Altitude
nexttile;
plot(out.FixedWingCmdFdbk.time,out.FixedWingCmdFdbk.signals.values(:,2),LineWidth=2);
hold on;
plot(out.FixedWingCmdFdbk.time,out.FixedWingCmdFdbk.signals.values(:,5),LineWidth=2);
grid on;
ylabel('Altitude');
xlabel('Time (seconds)');
legend('Command','Output');

%% Plot of navigation, pitch and roll
figure;
tiledlayout(2,2);

% Plot Aircraft Path
positionData = squeeze(out.PositionCmdFdbk.signals.values);
nexttile(1,[2 1]);
missionWayPoints = [FixedWingMission(1:5).position];
plot3(missionWayPoints(1,:),-missionWayPoints(2,:),-missionWayPoints(3,:),Marker='o',LineStyle=':',LineWidth=2);
hold on;
plot3(positionData(4,:),-positionData(5,:),-positionData(6,:),LineWidth=2);

view([0 90]);
ylim([-10 360]);
axis equal
grid on;
title('Navigation')
xlabel('North (m)')
ylabel('West (m)')
zlabel('Up')
legend('Waypoints','Aircraft Path')

% Plot Pitch Tracking
nexttile(2);
plot(getElement(out.logsout,"Pitch Tracking").Values,LineWidth=2);
grid on;
legend('Command','Output');

% Plot Roll Tracking
nexttile(4);
plot(getElement(out.logsout,"Roll Tracking").Values,LineWidth=2);
grid on;
legend('Command','Output');

end
