function [] = compare_muscleActFromMotDirectOverlaySinglePlot(subject, fnames, fnameMeasured, ...
                                                    tInfo, figHandleArray, figContents, ref_dataFormat)
% Purpose:  Creates figure windows specified by figHandleArray and, 
%           for each file in fnames, overlays plots of:
%           (1) CMC-generated muscle activations vs time,
%                   for muscles specified in ref_muscleActPlotLabels()
%           (2) L & R vertical GRF vs time, for reference
%           (3) EMG on/off times from Perry (1992), scaled to subject,
%                   for muscles specified in ref_muscleActPlotLabels()
%           (4) measured EMG data from file fnameMeasured, if available.
%         
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured EMG data.
%               NOTE:  set fnameMeasured == [] to suppress plotting of 
%                      measured EMG data.
%           tInfo is a structure containing the following 'trial info':
%               *.mass - subject mass (used to scale axes for force data)
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%               *.analogRate - analog sample rate
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
%           figContents returns a structure, formatted as follows:
%               figContents(figNum).qPlotLabel{plotNum} 
%               figContents(figNum).perryAbbr{muscleAbbrNum}
%               figContents(figNum).muscleActivationPlotIndices{limbIndex,muscleAbbrNum}
%               figContents(figNum).muscleNumbersInPlot{muscleAbbrNum}
%               figContents(figNum).subplotTitle{plotNum}
%               figContents(figNum).subplotAxisLabel{plotNum}
%               figContents(figNum).subplotRange{plotNum}
%               figContents(figNum).subplotTick{plotNum}    
%
% Called Functions:
%   read_motionFile(fname)
%   ref_muscleActPlotLabels()
%   overlay_emgMeasured(qMeasured, measuredLabel, time)
%   overlay_grftVertical(q, qPlotLabel, time, limb, fileNum)
%   overlay_muscleDataFromMot(q, qPlotLabel, time, fileNum)
%   format_muscleActFromMot(fnames, fnameMeasured, timeRange, tInfo ...
%              subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   get_emgTimingFromPerry(muscAbbr)
%   convert_perryDataToTime(perryData, ictoEvents, analogRate, limb, tInfo)
%   overlay_perryEMG(perryScaled, timeRange)
%   Suptitle(titleString)
%
% CTJ, 08-07
% ASA, 11-05, revised 2-06

% Specify attributes of figure windows.
nPlotRows = 1;                      
nPlotCols = 1;
nSubPlots = nPlotRows * nPlotCols;  
figPos = [2, 2, 3.25, 2.25];

%pLineColor = { [1 0.75 0.75] [1 0.75 1] [0.75 1 1] };
pLineColor = { 'k' 'k' 'k' };
pLineStyle = { '-' ':' '--' };

% For each figure...

for i = 1:length(tInfo)

    [q, time, timeRange, ictoEvents] = get_commonComparisonData('Muscle Activations vs Time', subject, fnames, tInfo{i});

    % Read motion file specified by fnameMeasured and store in structure array;
    % extract time array.
    if ~isempty(fnameMeasured)
        qMeasured = read_motionFile(fnameMeasured);
        timeMeasured = qMeasured.data(:, strcmpi(qMeasured.labels, 'time'));
    end

    timeAxisIsPercentGc = strcmpi( tInfo{i}.timeAxisLabel, 'percent of gait cycle' );
    if timeAxisIsPercentGc
        if ~isempty(fnameMeasured)
            percentMeasured = convert_timeToCycle( timeMeasured, tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
                                                   tInfo{i}.analogRate, ref_dataFormat );
        end
        percent{i} = convert_timeToCycle( time{i}, tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
                                                    tInfo{i}.analogRate, ref_dataFormat );
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
            percent{i} = percent{i} - 100.0;
            percentRange.min = percentRange.min - 100.0;
            percentRange.max = percentRange.max - 100.0;
        end
    end

end

for figNum = 1:length(figHandleArray)
    figure(figHandleArray(figNum)); 
    clf
    set(gcf, 'Units', 'inches');
    set(gcf, 'Position', figPos, 'Color', tInfo{1}.bgColor);
    axesPos = get(gcf, 'DefaultAxesPosition');
    axesPos = [axesPos(1)-0.05, axesPos(2)-0.05, axesPos(3)+0.12, axesPos(4)+0.12];
    set(gcf, 'DefaultAxesPosition', axesPos);
        
    for i = 1:length(tInfo)

        % Plot measured EMG data, if available.
        % DEBUG THIS CODE: when timeAxisIsPercentGc is true, it doesn't seem to
        % quite work correctly; also, may need to pass it bg, fg colors?
        if ~isempty(fnameMeasured)
           for plotIndex = 1:nSubPlots
                subplot(nPlotRows, nPlotCols, plotIndex);
                if timeAxisIsPercentGc
                    overlay_emgMeasured(qMeasured, ...
                       figContents(figNum).qMeasuredLabel{plotIndex}, percentMeasured);
                else
                    overlay_emgMeasured(qMeasured, ...
                       figContents(figNum).qMeasuredLabel{plotIndex}, timeMeasured);
                end
           end
        end

        % Plot data from SimTrack or UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            if timeAxisIsPercentGc
                overlay_muscleDataFromMotSinglePlot(q(i), ...
                    figContents(figNum).qPlotLabel{plotIndex}, ...
                    percent{i}, pLineColor{i}, pLineStyle{i});
            else
                overlay_muscleDataFromMotSinglePlot(q(i), ...
                    figContents(figNum).qPlotLabel{plotIndex}, ...
                    time{i}, pLineColor{i}, pLineStyle{i});
            end
        end

        % Format figure.
        % Set argument fnameMeasured == [] for figures that 
        % do not contain EMG data.
        if timeAxisIsPercentGc
            format_muscleActFromMotSinglePlot(fnames, [], ...
                percentRange, tInfo{i}, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
        else
            format_muscleActFromMotSinglePlot(fnames, [], ...
                timeRange, tInfo{i}, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
        end

        %getAndDirectlyOverlayPerryEMG(figContents(figNum).perryAbbr, figContents(figNum).muscleActivationPlotIndices, figContents(figNum).muscleNumbersInPlot, nPlotRows, nPlotCols, ictoEvents, tInfo{i}, timeRange, ref_dataFormat);

        % Set foreground colors
        set(findobj(gcf, 'Type', 'Text'), 'Color', tInfo{i}.fgColor);
    end
end

% Query user.
done = printmenu(figHandleArray, [subject '_muscleAct']);
return;
