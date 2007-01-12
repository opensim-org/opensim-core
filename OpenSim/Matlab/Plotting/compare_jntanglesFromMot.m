function [cycTimes] = compare_jntanglesFromMot(subject, fnames, fnameMeasured, ...
                                                    tInfo, figHandleArray, ref_dataFormat)
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
%           tInfo is a structure containing the following 'trial info':
%               *.speed  - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%               *.analogRate - analog sample rate
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
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
    timeIndex = find(strcmpi(q(fileNum).labels, 'time'));
    time{fileNum} = q(fileNum).data(:, timeIndex);
    timeRange.min = min(timeRange.min, min(time{fileNum}));
    timeRange.max = max(timeRange.max, max(time{fileNum}));
end

% Read motion file specified by fnameMeasured and store in structure array;
% extract time array.
if ~isempty(fnameMeasured)
    qMeasured = read_motionFile(fnameMeasured);
    timeIndex = find(strcmpi(qMeasured.labels, 'time'));
    timeMeasured = qMeasured.data(:, timeIndex);
end

% Read averaged data from control subjects.
g = read_gcdMean('gilletteMeanSD.GCD');

% Get analog frames corresponding to IC and TO events from tInfo,
% and store in structure array.
for fpHitNum = 1:length(tInfo.FP)     
    ictoEvents(fpHitNum).ic  = tInfo.ictoMatrix(fpHitNum, 1);
    ictoEvents(fpHitNum).oto = tInfo.ictoMatrix(fpHitNum, 2);
    ictoEvents(fpHitNum).oic = tInfo.ictoMatrix(fpHitNum, 3);
    ictoEvents(fpHitNum).to  = tInfo.ictoMatrix(fpHitNum, 4);
    ictoEvents(fpHitNum).icNext = tInfo.ictoMatrix(fpHitNum, 5);
end

% Get arrays that convert %gait cycle values to time values corresponding
% to 'simulateable' segment for R and L limbs.
% NOTE: I've defined tInfo and tInfo.analogRate as separate input arguments
%       to be consistent with the function as written for my 
%       pre-processing code.
cycTimes = ...
    convert_cycleToTime(g.cycle, tInfo, ictoEvents, tInfo.analogRate, ref_dataFormat);

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).qMeasuredLabel{plotNum}
%           figContents(figNum).gcdMeanLabel{plotNum}
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}                            
figContents = ref_jntanglePlotLabels;

% Specify attributes of figure windows.
nPlotRows = 4;                      
nPlotCols = 3;
nSubPlots = nPlotRows * nPlotCols;  
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.25*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.85*scnheight];
figColor = 'w';

% For each figure...
for figNum = 1:length(figHandleArray)
    figure(figHandleArray(figNum)); 
    clf
    set(gcf, 'Position', figPos, 'Color', figColor);
        
    % Plot reference joint angles.
    switch figNum
        case {1 2}
            for plotIndex = 1:nSubPlots
                subplot(nPlotRows, nPlotCols, plotIndex);
                overlay_jntanglesMean(g.jntangles, ...
                  figContents(figNum).gcdMeanLabel{plotIndex}, cycTimes.R);
            end
        case 3
            for plotIndex = 1:nSubPlots
                subplot(nPlotRows, nPlotCols, plotIndex);
                overlay_jntanglesMean(g.jntangles, ...
                  figContents(figNum).gcdMeanLabel{plotIndex}, cycTimes.L);
            end
    end
        
    % Plot measured joint angles.
    for plotIndex = 1:nSubPlots
        subplot(nPlotRows, nPlotCols, plotIndex);
        overlay_jntanglesMeasured(qMeasured, ...
            figContents(figNum).qMeasuredLabel{plotIndex}, timeMeasured);
    end
                
    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot data from UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_jntanglesFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, fileNum);
        end
    end
    
    % Format figure.
    format_jntanglesFromMot(fnames, fnameMeasured, timeRange, ...
            figContents(figNum).subplotTitle, ...
            figContents(figNum).subplotAxisLabel, ...
            figContents(figNum).subplotRange, ...
            figContents(figNum).subplotTick);
    
    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of Joint Angles:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end
      
% Query user.
done = printmenu(figHandleArray, [subject '_jntangles']);
return;
