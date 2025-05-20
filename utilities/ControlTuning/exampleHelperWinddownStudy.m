%exampleHelperWinddownStudy Study effect of Wind Down Rate on transition

% Copyright 2023 The MathWorks, Inc.
%% 3.2 Study effect of Wind Down Rate on transition
% To control the tilt schedule rate, change the  WindDownRate parameter. WindDownRate () is the rate at which the back rotors reduce their speed during transition. A fast rate would mean the back rotors lose authority to balance pitch of the aircraft as it transitions to fixed wing flight. A slow wind down rate will increase the period of transition for aircraft and consume additional battery.
windRateParam = getEntry(dDataSectObj,'WindDownRate');
%Transition Rate is 15 deg/sec.
setValue(windRateParam,10)
outTW10=sim(mdl);
%Transition Rate is 30 deg/sec.
setValue(windRateParam,20)
outTW20=sim(mdl);
%Transition Rate is 45 deg/sec.
setValue(windRateParam,45)
outTW45=sim(mdl);
%% Plot results for comparision.
tLayout=tiledlayout(4,1);
exampleHelperPlotTransitionResults(tLayout,outTW10);
exampleHelperPlotTransitionResults(tLayout,outTW20);
exampleHelperPlotTransitionResults(tLayout,outTW45);
leg=legend({'Wind Down 10 rev/sec','Wind Down 20 rev/sec','Wind Down 45 rev/sec'});
leg.Layout.Tile='north';
%% We observe that wind down rate does not significantly impact the transition performance of the aircraft characterized by pitch and altitude variations. Thus we choose the orignial setting of the parameter as 10. 
setValue(windRateParam,10);
%This can be explained by the backrotors winding down during the narrow time window between critical angle of Tilt Transition (60 degrees) being reached and the aircraft achieving full transition. 