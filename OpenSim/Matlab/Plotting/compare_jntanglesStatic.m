function [] = compare_jntanglesStatic(subject, fnames,  ...
                                        fnameMeasured, figHandleArray)
% Purpose:  Creates three figure windows specified by figHandleArray and,
%           for each file in fnames, overlays plots of:
%           (1) pelvis translations, pelvis angles, and back angles vs time
%           (2) R hip angles, knee angles, and ankle angles vs time
%           (3) L hip angles, knee angles, and ankle angles vs time
%           (4) measured joint angles from file fnameMeasured
%
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured joint angle data from the static trial
%           figHandleArray specifies the numbers of the figure windows
%               to be created
%
%           NOTES:
%           (1) Column headings for the joint angle data are assumed to
%               correspond to the generalized coordinates in 
%               Darryl's gait23*.jnt model.
%           (2) Joint angles are converted to a conventional 
%               gait lab coordinate system for plotting.
%           (3) This function gets the time array from fnameMeasured, 
%               and it assumes that this time array is valid for all
%               of the files being compared. This is necessary since 
%               UW-Gait scale.exe outputs the IK solution for the 
%               static trial *sc.mot without a time array.
%         
% Called Functions:
%   read_motionFile(fname)
%   ref_jntangleStaticPlotLabels()
%   overlay_jntanglesMeasured(qMeasured, qMeasuredLabel, time)
%   overlay_jntanglesStatic(q, qPlotLabel, time, fileNum)
%   format_jntanglesStatic(fnames, fnameMeasured, timeRange, ...
%               subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   Suptitle(titleString)
%
% ASA, 11-05, revised 2-06


% Display status on screen.
status = sprintf('\n\n\n%s%s', ...
     'Plotting Static Trial Joint Angles vs Time for Subject ', subject);
disp(status);

% Read motion files specified by fnames{} and store in structure array.
for fileNum = 1:length(fnames)
    q(fileNum) = read_motionFile(fnames{fileNum});
end

% Read file specified by fnameMeasured and store in structure array.
% Extract time array, and store timeRange.max and timeRange.min for 
% scaling the time axes of plots.
qMeasured = read_motionFile(fnameMeasured);
timeIndex = find(strcmpi(qMeasured.labels, 'time'));
time = qMeasured.data(:, timeIndex);
timeRange.min = min(time);
timeRange.max = max(time);

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).qMeasuredLabel{plotNum}
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}
figContents = ref_jntangleStaticPlotLabels;

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

    % Plot measured joint angles.
    for plotIndex = 1:nSubPlots
        subplot(nPlotRows, nPlotCols, plotIndex);
        overlay_jntanglesMeasured(qMeasured, ...
            figContents(figNum).qMeasuredLabel{plotIndex}, time);
    end
    
    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot data from UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_jntanglesStatic(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, time, fileNum);
        end
    end
    
    % Format figure.
    format_jntanglesStatic(fnames, fnameMeasured, timeRange, ...
            figContents(figNum).subplotTitle, ...
            figContents(figNum).subplotAxisLabel, ...
            figContents(figNum).subplotRange, ...
            figContents(figNum).subplotTick);

    % Add title
    titleString = sprintf('%s%s', ...
        'Comparison of Joint Angles, Static Trial:  Subject ', subject);
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end
      
% Query user.
done = printmenu(figHandleArray, [subject '_jntanglesStatic']);
return;
