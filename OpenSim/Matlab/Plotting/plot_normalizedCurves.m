function plot_normalizedCurves(timeInterval, stoOrMotFileArray, labelArray, startTimeArray, endTimeArray, plotColorStyleArray, plotLineThicknessArray)

% plotMultipleCurves(timeScale, stoOrMotFileArray, labelArray, startTimeArray,
% endTimeArray, plotColorArray) plots curves from the storage (.sto) or
% motion (.mot) files listed in stoOrMotFileArray.  We assume labelArray,
% startTimeArray, endTimeArray, and plotColorArray have the same size as
% stoOrMotFileArray.  For each i from 1 to length(stoOrMotFileArray), this
% function will plot the data in the column with label labelArray(i) from
% the file stoOrMotFileArray(i) using the time column from the same file,
% from the time startTimeArray(i) to the time endTimeArray(i), using the
% color/style plotColorStyleArray(i) for the color of the plotted curve.  The time
% arrays for all curves will be normalized, so that curves with different
% time intervals will be scaled so that all curves start and end at the
% same time.  This can be useful for plotting say, the activation of a
% selected muscle over a gait cycle for multiple trials at different speeds
% of walking.  This function will scale all curves so that their time
% domain interval is timeInterval.  The first curve will not actually
% have any interpolation done; its time column will simply be replaced by
% timeScale.  All subsequent curves will be interpolated to scale them to
% have the same time domain as the first curve (the goal is to have the
% same number of entries in the time column of every curve to be the same),
% and then their time columns will also be changed to be timeInterval.

% Check that each input array has the correct number of entries.
if length(timeInterval) ~= 2
    error('Need exactly two entries in timeInterval array.');
end
if timeInterval(1) > timeInterval(2)
    error('First time interval entry should not be greater than second entry!');
end
numberOfCurves = length(stoOrMotFileArray);
if numberOfCurves < 2
    error('Need at least two curves/input files/start and end times/plot colors!');
end
if length(labelArray) ~= numberOfCurves
    error('Incorrect number of labels: %d, expected %d',length(labelArray),numberOfCurves);
end
if length(startTimeArray) ~= numberOfCurves
    error('Incorrect number of start times: %d, expected %d',length(startTimeArray),numberOfCurves);
end
if length(endTimeArray) ~= numberOfCurves
    error('Incorrect number of end times: %d, expected %d',length(endTimeArray),numberOfCurves);
end
if length(plotColorStyleArray) ~= numberOfCurves
    error('Incorrect number of plot colors: %d, expected %d',length(plotColorStyleArray),numberOfCurves);
end
if length(plotLineThicknessArray) ~= numberOfCurves
    error('Incorrect number of plot line thicknesses: %d, expected %d',length(plotLineThicknessArray),numberOfCurves);
end

% Compute first curve.
disp(['Reading first motion/storage file: ' stoOrMotFileArray{1}]);
q = read_motionFile(stoOrMotFileArray{1});
columnIndex = find(strcmp(q.labels,labelArray{1}));
if length(columnIndex) ~= 1
    error(['Incorrect number (%d) of columns with label ' labelArray{1} '.'],columnIndex);
end
timeColumnIndex = find(strcmp(q.labels,'time'));
if timeColumnIndex ~= 1
    error('Expected time column to be first column and not column number %d.',timeColumnIndex);
end
time = q.data(:,1);
indices = find(time >= startTimeArray(1) & time <= endTimeArray(1));
normalizedTimeDt = (timeInterval(2) - timeInterval(1)) / (length(indices) - 1);
normalizedTime = transpose(timeInterval(1):normalizedTimeDt:timeInterval(2));

% Plot first curve.
plot(normalizedTime,q.data(indices,columnIndex),plotColorStyleArray{1},'LineWidth',plotLineThicknessArray(1));
hold on;

% Compute and plot curves 2:numberOfCurves.
for i = 2:numberOfCurves
    
    % Compute ith curve.
    disp(['Reading motion/storage file: ' stoOrMotFileArray{i}]);
    q = read_motionFile(stoOrMotFileArray{i});
    columnIndex = find(strcmp(q.labels,labelArray{i}));
    if length(columnIndex) ~= 1
        error(['Incorrect number (%d) of columns with label ' labelArray{i} '.'],columnIndex);
    end
    timeColumnIndex = find(strcmp(q.labels,'time'));
    if timeColumnIndex ~= 1
        error('Expected time column to be first column and not column number %d.',timeColumnIndex);
    end
    time = q.data(:,1);
    indices = find(time >= startTimeArray(i) & time <= endTimeArray(i));
    normalizedTimeDt = (timeInterval(2) - timeInterval(1)) / (length(indices) - 1);
    normalizedTime = transpose(timeInterval(1):normalizedTimeDt:timeInterval(2));
    
    % Plot ith curve.
    plot(normalizedTime,q.data(indices,columnIndex),plotColorStyleArray{i},'LineWidth',plotLineThicknessArray(i));
    
end

hold off;
