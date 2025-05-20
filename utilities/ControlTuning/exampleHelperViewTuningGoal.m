function exampleHelperViewTuningGoal(tuningGoals,hTuner)

for k = 1:length(tuningGoals)
    figure;
    viewGoal(tuningGoals(k),hTuner);
end

end

