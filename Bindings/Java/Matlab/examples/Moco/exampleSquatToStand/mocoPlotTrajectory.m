function mocoPlotTrajectory(trajA, trajB, nameA, nameB)
% Plot a MocoTrajectory. Optionally, specify a second trajectory and names
% for the trajectories.
% For a generic version of this function, see the utility
% osimMocoTrajectoryReport.m.

import org.opensim.modeling.*;

if ischar(trajA)
    trajA = MocoTrajectory(trajA);
end
if nargin > 1 && ischar(trajB)
    trajB = MocoTrajectory(trajB);
end
%% States.
figure;
stateNames = trajA.getStateNames();
numStates = stateNames.size();
dim = sqrt(numStates);
if dim == ceil(dim)
    numRows = dim;
    numCols = dim;
else
    numCols = min(numStates, 4);
    numRows = floor(numStates / 4);
    if mod(numStates, 4) ~= 0
        numRows = numRows + 1;
    end
end
for i = 0:numStates-1
    subplot(numRows, numCols, i+1);
    plot(trajA.getTimeMat(), ...
         trajA.getStateMat(stateNames.get(i)), '-r', ...
         'linewidth', 3);
    if nargin > 1
        hold on
        plot(trajB.getTimeMat(), ...
             trajB.getStateMat(stateNames.get(i)), '--b', ...
             'linewidth', 2.5);
        hold off
    end
    
    stateName = stateNames.get(i);
    title(stateName(11:end), 'Interpreter', 'none')
    xlabel('time (s)')
    if contains(stateName, 'value')
        ylabel('position (rad)')
    elseif contains(stateName, 'speed')
        ylabel('speed (rad/s)')
    elseif contains(stateName, 'activation')
        ylabel('activation (-)')
        ylim([0, 1])
    end
    if i == 0 && nargin > 1
        if nargin == 4
            legend(nameA, nameB);
        else
            legend('A', 'B');
        end
    end
end

%% Controls.
figure;
controlNames = trajA.getControlNames();
numControls = controlNames.size();
dim = sqrt(numControls);
if dim == ceil(dim)
    numRows = dim;
    numCols = dim;
else
    numCols = min(numControls, 4);
    numRows = floor(numControls / 4);
    if mod(numControls, 4) ~= 0
        numRows = numRows + 1;
    end
end
for i = 0:numControls-1
    subplot(numRows, numCols, i+1);
    yA = trajA.getControlMat(controlNames.get(i));
    plot(trajA.getTimeMat(), yA, '-r', 'linewidth', 3);
    if nargin > 1
        hold on
        yB = trajB.getControlMat(controlNames.get(i));
        plot(trajB.getTimeMat(), yB, '--b', 'linewidth', 2.5);
        hold off
    end
    title(controlNames.get(i), 'Interpreter', 'none')
    xlabel('time (s)')
    ylabel('value')
    if max(yA) <= 1 && min(yA) >= 0
        fixYLim = true;
        if nargin > 1 && (max(yB) > 1 || min(yB) < 0)
            fixYLim = false;
        end
        if fixYLim
            ylim([0, 1]);
        end
    end
    if i == 0 && nargin > 1
        if nargin == 4
            legend(nameA, nameB);
        else
            legend('A', 'B');
        end
    end
end
