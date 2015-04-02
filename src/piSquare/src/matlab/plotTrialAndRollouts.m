function [] = plotTrialAndRollouts(directoryPath, trialIndex, numRollouts)

%acquire both trial and rollouts data
fileName = [directoryPath 'TrajectoryTrial' int2str(trialIndex) '.txt'];
trialData = load(fileName);
data = trialData(:,1);
size(data, 1)

newColumnIndex = 2;
for i = 0:numRollouts-1
    fileName = [directoryPath 'TrajectoryTrial' int2str(trialIndex) 'Rollout' int2str(i) '.txt'];
    rolloutData = load(fileName);
    data(:, newColumnIndex) = rolloutData(:, 1);
    newColumnIndex = newColumnIndex + 1;
end

%plot rollouts and resulting trial
x = 1:1:size(data, 1);

%plotting trial
figure,
plot(x, data(:, 1));
ylim([-1 1]);
hold on

%plotting rollouts
for i = 0:numRollouts-1
        p = plot(x, data(:, i+2));
        set(p, 'Color', 'red')
        hold on
end

xlabel('t');
ylabel('trajectories');
legend('trial', 'rollouts', 'Location', 'NorthOutside');

end

