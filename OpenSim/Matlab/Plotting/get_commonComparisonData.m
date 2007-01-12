function [q, time, timeRange, ictoEvents] = get_commonComparisonData(description, subject, fnames, tInfo)

% Display status on screen.
status = sprintf('\n\n\nPlotting %s for Subject %s', description, subject);
disp(status);

% Read motion files specified by fnames{} and store in structure array.
for fileNum = 1:length(fnames)
    q(fileNum) = read_motionFile(fnames{fileNum});
end

% Extract time arrays from motion files, and store timeRange.max and 
% timeRange.min for scaling the time axes of plots.
timeRange.min = 100;
timeRange.max = 0;
for fileNum = 1:length(fnames)
    timeIndex = find(strcmpi(q(fileNum).labels, 'time'));
    time{fileNum} = q(fileNum).data(:, timeIndex);
    timeRange.min = min(timeRange.min, min(time{fileNum}));
    timeRange.max = max(timeRange.max, max(time{fileNum}));
end

% Get analog frames corresponding to IC and TO events from tInfo,
% and store in structure array; needed to scale Perry's data to subject.
for fpHitNum = 1:length(tInfo.FP)     
    ictoEvents(fpHitNum).ic  = tInfo.ictoMatrix(fpHitNum, 1);
    ictoEvents(fpHitNum).oto = tInfo.ictoMatrix(fpHitNum, 2);
    ictoEvents(fpHitNum).oic = tInfo.ictoMatrix(fpHitNum, 3);
    ictoEvents(fpHitNum).to  = tInfo.ictoMatrix(fpHitNum, 4);
    ictoEvents(fpHitNum).icNext = tInfo.ictoMatrix(fpHitNum, 5);
end

