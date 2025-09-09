
clear all
close all
clc

%% Five vehicles in a line + centered ellipses (Automated Driving Toolbox)
% t, road, vehicles
scenario = drivingScenario('SampleTime',0.05,'StopTime',20);

% --- Straight 2-lane road (along +x) ---
roadCent = [0 0 0; 200 0 0];    % start -> end
ls = lanespec(2);               % 2 lanes (y>0 is lane 1, y<0 lane 2)
road(scenario, roadCent, 'Lanes', ls);

% --- Spawn 5 vehicles in lane 1, spaced 20 m ---
N = 5;
veh = driving.scenario.Vehicle.empty(N,0);   % proper preallocation
baseX     = 10;           
spacing   = 20;           
laneWidth = ls.Width(1);  
laneY     = +laneWidth/4; 

for i = 1:N
    veh(i) = vehicle(scenario,'ClassID',1,'Length',4.5,'Width',1.9);
    x0 = baseX + (i-1)*spacing;
    waypoints = [x0 laneY;  200 laneY];
    speed     = 10;
    trajectory(veh(i), waypoints, speed);
end

% --- Open scenario plot (top-down) and prepare ellipse overlays ---
plot(scenario); ax = gca; axis(ax,'equal'); hold(ax,'on');
title(ax, 'Five Vehicles with Centered Ellipses');

% ellipse radii (semi-axes) and rotation follows vehicle yaw each step
a = 6;               % semi-major axis (m)   — along vehicle heading
b = 3;               % semi-minor axis (m)   — lateral
theta = linspace(0,2*pi,120).';            % for drawing

% Precreate line handles for ellipses (one per vehicle)
hEll = gobjects(N,1);
for i = 1:N
    hEll(i) = plot(ax, nan, nan, '-', 'LineWidth',1.5);
end

% --- Sim loop: advance scenario and update ellipses ---
while advance(scenario)
    for i = 1:N
        % read current pose (x,y,yaw) of vehicle i
        p   = veh(i).Position;      % [x y z]
        psi = deg2rad(veh(i).Yaw);  % radians

        % rotated ellipse centered at vehicle i
        % body-frame param: [a*cos t; b*sin t]
        c = cos(psi); s = sin(psi);
        ex = p(1) + a*c.*cos(theta) - b*s.*sin(theta);
        ey = p(2) + a*s.*cos(theta) + b*c.*sin(theta);

        set(hEll(i), 'XData', ex, 'YData', ey);
    end
    drawnow limitrate
end
