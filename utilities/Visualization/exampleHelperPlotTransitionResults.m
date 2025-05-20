function exampleHelperPlotTransitionResults(T,out)
% exampleHelperPlotTransitionResults plot results for transition for
% analysis.

% Copyright 2023 The MathWorks, Inc.

%% Plot of TransitionFeedback
if(isempty(T.Children))
    for i=1:1:4
        nexttile(T);
    end
end
ax=findobj(T,'Type','Axes');
for i=1:1:4
    hold(ax(i),'on');
end

plot(ax(1),out.TransitionFeedback.time,out.TransitionFeedback.signals.values(:,1),'LineWidth',2)
grid
ylabel(ax(1),'Airspeed (m/s)')

plot(ax(2),out.TransitionFeedback.time,rad2deg(out.TransitionFeedback.signals.values(:,2)),'LineWidth',2)
grid
ylabel(ax(2),'Tilt (deg)')

plot(ax(3),out.TransitionFeedback.time,rad2deg(out.TransitionFeedback.signals.values(:,3)),'LineWidth',2)
grid
xlabel('Time (sec)')
ylabel(ax(3),'Pitch (deg)');

plot(ax(4),out.TransitionFeedback.time,out.TransitionFeedback.signals.values(:,4),'LineWidth',2)
grid
xlabel('Time (sec)')
ylabel(ax(4),'Altitude (m)')

for i=1:1:4
    hold(ax(i),'off');
end
end