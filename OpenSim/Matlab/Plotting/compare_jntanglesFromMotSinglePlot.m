function [cycTimes] = compare_jntanglesFromMotSinglePlot(subject, fnames, fnameMeasured, ...
                                                    tInfo, figHandleArray, figContents, ref_dataFormat)
% Purpose:  Creates three figure windows specified by figHandleArray and,
%           for each file in fnames, overlays plots of:
%           (1) pelvis translations, pelvis angles, and back angles vs time
%           (2) R hip angles, knee angles, and ankle angles vs time
%           (3) L hip angles, knee angles, and ankle angles vs time
%           (4) measured joint angles from file fnameMeasured
%           (5) averaged joint angles from control subjects,
%               from read_gcdMean()
%
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured joint angle data
%           Each element of the cell array tInfo is a structure containing
%           the following 'trial info':
%               *.speed  - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%               *.analogRate - analog sample rate
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
%           figContents is a structure, formatted as follows:
%               figContents(figNum).qPlotLabel{plotNum}
%               figContents(figNum).subplotTitle{plotNum}
%               figContents(figNum).subplotAxisLabel{plotNum}
%               figContents(figNum).subplotRange{plotNum}
%               figContents(figNum).subplotTick{plotNum}
%
%           NOTES:
%           (1) Column headings for the joint angle data are assumed to
%               correspond to the generalized coordinates in 
%               Darryl's gait23*.jnt model.
%           (2) Joint angles are converted to a conventional 
%               gait lab coordinate system for plotting.
%         
% Called Functions:
%   read_motionFile(fname)
%   read_gcdMean(fname)
%   convert_cycleToTime(cycle, tInfo, ictoEvents, analogRate)
%   ref_jntanglePlotLabels()
%   overlay_jntanglesMean(jntangles, gcdMeanLabel, timeMatrix)
%   overlay_jntanglesMeasured(qMeasured, qMeasuredLabel, time)
%   overlay_jntanglesFromMot(q, qPlotLabel, time, fileNum)
%   format_jntanglesFromMot(fnames, fnameMeasured, timeRange, ...
%               subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   Suptitle(titleString)
%
% ASA, 11-05, revised 2-06


% Display status on screen.
status = sprintf('\n\n\n%s%s', ...
            'Plotting Joint Angles vs Time for Subject ', subject);
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
    time{fileNum} = q(fileNum).data(:, strcmpi(q(fileNum).labels, 'time'));
    timeRange.min = min(timeRange.min, min(time{fileNum}));
    timeRange.max = max(timeRange.max, max(time{fileNum}));
end

% Read motion file specified by fnameMeasured and store in structure array;
% extract time array.
if ~isempty(fnameMeasured)
    qMeasured = read_motionFile(fnameMeasured);
    timeMeasured = qMeasured.data(:, strcmpi(qMeasured.labels, 'time'));
end

% Read averaged data from control subjects.
g = read_gcdMean('gilletteMeanSD.GCD');

% Specify attributes of figure windows.
nPlotRows = 1;                      
nPlotCols = 1;
nSubPlots = nPlotRows * nPlotCols;  
figPos = [2, 2, 3.25, 2.25];

% For each figure...
for figNum = 1:length(figHandleArray)
    figure(figHandleArray(figNum)); 
    clf
    set(gcf, 'Units', 'inches');
    set(gcf, 'Position', figPos, 'Color', tInfo{1}.bgColor);
    axesPos = get(gcf, 'DefaultAxesPosition');
    axesPos = [axesPos(1)-0.05, axesPos(2)-0.05, axesPos(3)+0.12, axesPos(4)+0.12];
    set(gcf, 'DefaultAxesPosition', axesPos);

    for i = 1:length(tInfo)
        % Get analog frames corresponding to IC and TO events from tInfo{i},
        % and store in structure array.
        for fpHitNum = 1:length(tInfo{i}.FP)     
            ictoEvents(fpHitNum).ic  = tInfo{i}.ictoMatrix(fpHitNum, 1);
            ictoEvents(fpHitNum).oto = tInfo{i}.ictoMatrix(fpHitNum, 2);
            ictoEvents(fpHitNum).oic = tInfo{i}.ictoMatrix(fpHitNum, 3);
            ictoEvents(fpHitNum).to  = tInfo{i}.ictoMatrix(fpHitNum, 4);
            ictoEvents(fpHitNum).icNext = tInfo{i}.ictoMatrix(fpHitNum, 5);
        end

        % Get arrays that convert %gait cycle values to time values corresponding
        % to 'simulateable' segment for R and L limbs.
        % NOTE: I've defined tInfo{i} and tInfo{i}.analogRate as separate input arguments
        %       to be consistent with the function as written for my 
        %       pre-processing code.
        cycTimes = ...
            convert_cycleToTime(g.cycle, tInfo{i}, ictoEvents, tInfo{i}.analogRate, ref_dataFormat);

        timeAxisIsPercentGc = strcmpi( tInfo{i}.timeAxisLabel, 'percent of gait cycle' );
        if timeAxisIsPercentGc
            if strcmp( tInfo{i}.gcLimb, 'R' )
                times = cycTimes.R;
            else % Assuming tInfo{i}.gcLimb is 'L'
                times = cycTimes.L;
            end
            sizeOfTimes = size( times );
            cycPercents = zeros( sizeOfTimes );
            numberOfColumnsInTimes = sizeOfTimes(2);
            for colNum = 1:numberOfColumnsInTimes
                cycPercents(:,colNum) = convert_timeToCycle( times(:,colNum), tInfo{i}.gcLimb, tInfo{i}, ...
                                                   ictoEvents, tInfo{i}.analogRate, ref_dataFormat );
            end
            if ~isempty(fnameMeasured)
                percentMeasured = convert_timeToCycle( timeMeasured, tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
                                                       tInfo{i}.analogRate, ref_dataFormat );
            end
            percent = time;
            for fileNum = 1:length(fnames)
                percent{fileNum} = convert_timeToCycle( time{fileNum}, tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
                                                        tInfo{i}.analogRate, ref_dataFormat );
            end
            percentAux = convert_timeToCycle( [timeRange.min, timeRange.max], ...
                                              tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
                                              tInfo{i}.analogRate, ref_dataFormat );
            percentRange.min = percentAux(1);
            percentRange.max = percentAux(2);
        end
        % THIS IS TEMPORARY, and only applies to delaware3 fast walking
        % trial where I started the simulation at the second gait cycle
        % instead of the first, and although I could switch back to using
        % the first gait cycle, I never got around to it.
        if strcmp( subject, 'de3' )
            if i == 1
                % Assuming fast trial is when i == 1
                percent{1} = percent{1} - 100.0;
                percent{2} = percent{2} - 100.0;
            end
        end

        % For each file in fnames{}...
        for fileNum = 2*i-1 : 2*i

            % Plot data from SimTrack Workflow.
            for plotIndex = 1:nSubPlots
                subplot(nPlotRows, nPlotCols, plotIndex);
                if timeAxisIsPercentGc
                    overlay_jntanglesFromMotSinglePlot(q(fileNum), ...
                        figContents(figNum).qPlotLabel{plotIndex}, ...
                        percent{fileNum}, fileNum);
                else
                    overlay_jntanglesFromMotSinglePlot(q(fileNum), ...
                        figContents(figNum).qPlotLabel{plotIndex}, ...
                        time{fileNum}, fileNum);
                end
            end
        end

        % Format figure.
        if timeAxisIsPercentGc
            format_jntanglesFromMotSinglePlot(fnames, fnameMeasured, percentRange, ...
                    tInfo{i}.timeAxisLabel, tInfo{i}.bgColor, tInfo{i}.fgColor, ...
                    figContents(figNum).subplotTitle, ...
                    figContents(figNum).subplotAxisLabel, ...
                    figContents(figNum).subplotRange, ...
                    figContents(figNum).subplotTick);
        else
            format_jntanglesFromMotSinglePlot(fnames, fnameMeasured, timeRange, ...
                    tInfo{i}.timeAxisLabel, tInfo{i}.bgColor, tInfo{i}.fgColor, ...
                    figContents(figNum).subplotTitle, ...
                    figContents(figNum).subplotAxisLabel, ...
                    figContents(figNum).subplotRange, ...
                    figContents(figNum).subplotTick);
        end

        % Set foreground colors
        set(findobj(gcf, 'Type', 'Text'), 'Color', tInfo{i}.fgColor);
    end
end
      
% Query user.
done = printmenu(figHandleArray, [subject '_jntangles']);
return;
