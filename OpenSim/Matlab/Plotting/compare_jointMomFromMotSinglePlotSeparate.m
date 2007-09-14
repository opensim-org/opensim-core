function [] = compare_jointMomFromMotSinglePlotSeparate(subject, fnames, tInfo, figHandleArray, figContents, ref_dataFormat)
% Purpose:  Creates figure windows specified by figHandleArray and, 
%           for each file in fnames, overlays plots of:
%           (1) CMC-generated actuator forces vs time,
%                   for actuators specified in ref_actuatorFrcPlotLabels()
%           (2) L & R vertical GRF vs time, for reference
%         
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           plotLiteratureData is a Boolean variable (true or false)
%               indicating whether or not to plot literature data against the
%               simulated joint moment data
%           tInfo is a structure containing the following 'trial info':
%               *.mass - subject mass (used to scale axes for force data)
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
% Called Functions:
%   read_motionFile(fname)
%   ref_jointMomentPlotLabels()
%   overlay_grftVertical(q, qPlotLabel, time, limb, fileNum)
%   overlay_jointMomentsFromMot(q, qPlotLabel, time, fileNum)
%   format_jointMomFromMot(fnames, timeRange, tInfo ...
%              subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   Suptitle(titleString)
%
% CTJ, 07-07, adapted from:
% ASA, 11-05, revised 2-06

% Specify attributes of figure windows.
nPlotRows = 1;                      
nPlotCols = 1;
nSubPlots = nPlotRows * nPlotCols;  
figPos = [2, 2, 3.25, 2.25];

pLineColor = { {[1 0.75 0.75], [0.75 0 0]}, ...
               {[1 0.75 1], [0.75  0.5 0]}, ...
               {[0.75 1 1], [0 0.75 0.75]}  };

numberOfSpeeds = length(tInfo);
totalNumberOfFigures = length(figHandleArray);
numberOfFiguresPerSpeed = totalNumberOfFigures / numberOfSpeeds;
% For each figure...
for i = 1:numberOfSpeeds
    firstFigureNumber = (i-1) * numberOfFiguresPerSpeed + 1;
    lastFigureNumber = i * numberOfFiguresPerSpeed;

    [q, time, timeRange, ictoEvents] = get_commonComparisonData('Joint Moments vs Time', subject, fnames(i), tInfo{i});

    timeAxisIsPercentGc = strcmpi( tInfo{i}.timeAxisLabel, 'percent of gait cycle' );
    if timeAxisIsPercentGc
        percent = convert_timeToCycle( time{1}, tInfo{i}.gcLimb, tInfo{i}, ictoEvents, ...
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
            percent = percent - 100.0;
            percentRange.min = percentRange.min - 100.0;
            percentRange.max = percentRange.max - 100.0;
        end
    end

    for figNum = firstFigureNumber:lastFigureNumber
        figure(figHandleArray(figNum)); 
        clf
        set(gcf, 'Units', 'inches');
        set(gcf, 'Position', figPos, 'Color', tInfo{1}.bgColor);
        axesPos = get(gcf, 'DefaultAxesPosition');
        axesPos = [axesPos(1)-0.05, axesPos(2)-0.05, axesPos(3)+0.12, axesPos(4)+0.12];
        set(gcf, 'DefaultAxesPosition', axesPos);

        % Plot data from SimTrack or UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            if timeAxisIsPercentGc
                overlay_jointMomentsFromMotSinglePlot(q(1), ...
                    figContents(figNum).qPlotLabel{plotIndex}, ...
                    percent, pLineColor{i});
            else
                overlay_jointMomentsFromMotSinglePlot(q(1), ...
                    figContents(figNum).qPlotLabel{plotIndex}, ...
                    time{1}, pLineColor{i});
            end
        end

        % Format figure.
        if timeAxisIsPercentGc
            format_jointMomFromMotSinglePlot(fnames(i), percentRange, tInfo{i}, ...
                        figContents(figNum).subplotTitle, ...
                        figContents(figNum).subplotAxisLabel, ...
                        figContents(figNum).subplotRange, ...
                        figContents(figNum).subplotTick);
        else
            format_jointMomFromMotSinglePlot(fnames(i), timeRange, tInfo{i}, ...
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
done = printmenu(figHandleArray, [subject '_muscleFrc']);
return;
   
