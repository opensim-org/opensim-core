function [] = compare_muscleActFromMotDirectOverlay(subject, fnames, fnameMeasured, ...
                                                    tInfo, figHandleArray, ref_dataFormat)
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
% ASA, 11-05, revised 2-06

[q, time, timeRange, ictoEvents] = get_commonComparisonData('Muscle Activations vs Time', subject, fnames, tInfo);

% Read motion file specified by fnameMeasured and store in structure array;
% extract time array.
if ~isempty(fnameMeasured)
    qMeasured = read_motionFile(fnameMeasured);
    timeIndex = find(strcmpi(qMeasured.labels, 'time'));
    timeMeasured = qMeasured.data(:, timeIndex);
end

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).qMeasuredLabel{plotNum}
%           figContents(figNum).perryAbbr{muscleAbbrNum}
%           figContents(figNum).muscleActivationPlotIndices{limbIndex,muscleAbbrNum}
%           figContents(figNum).muscleNumbersInPlot{muscleAbbrNum}
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}    
figContents = ref_dataFormat.muscleActPlotLabels();

% Specify attributes of figure windows.
nPlotRows = 5;                      
nPlotCols = 2;
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
    
    % Plot measured EMG data, if available.
    if ~isempty(fnameMeasured)
       for plotIndex = 5:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_emgMeasured(qMeasured, ...
               figContents(figNum).qMeasuredLabel{plotIndex}, timeMeasured);
       end
    end

    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot vertical GRFs for reference.
        for plotIndex = 1:2
            switch plotIndex
                case 1
                    limb = 'L';
                case 2 
                    limb = 'R';
            end
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_grftVertical(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, limb, fileNum);
        end
            
        % Plot data from UW-Gait Workflow.
        for plotIndex = 3:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_muscleDataFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, fileNum);
        end
    end
    
    % Format figure.
    % Set argument fnameMeasured == [] for figures that 
    % do not contain EMG data.
    switch figNum
        case {1 2 5 6 9} 
            format_muscleActFromMot(fnames, [], ...
                timeRange, tInfo, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
        case {3 4 7 8}
            format_muscleActFromMot(fnames, fnameMeasured, ...
                timeRange, tInfo, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
    end

	getAndDirectlyOverlayPerryEMG(figContents(figNum).perryAbbr, figContents(figNum).muscleActivationPlotIndices, figContents(figNum).muscleNumbersInPlot, nPlotRows, nPlotCols, ictoEvents, tInfo, timeRange, ref_dataFormat);
    
    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of Muscle Activations:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end

% Query user.
done = printmenu(figHandleArray, [subject '_muscleAct']);
return;
   
