function [] = compare_jntmomentsFromMot(subject, fnames, tInfo, figHandleArray)
% Purpose:  Creates three figure windows specified by figHandleArray and,
%           for each file in fnames, overlays plots of:
%           (1) pelvis and back moments/forces vs time
%           (2) R hip, knee, and ankle moments vs time
%           (3) L hip, knee, and ankle moments vs time
%
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.speed  - average walking speed during trial in m/s
%                          (used for displaying plot title)
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
%
%           NOTES:
%           (1) Column headings for the joint moments are assumed to
%               correspond to the output file of UW-Gait invdyn.exe.
%           (2) Joint moments are plotted in Darryl's gait23*.jnt model CS.
%         
% Called Functions:
%       read_motionFile(fname)
%       ref_jntmomentPlotLabels()
%       overlay_jntmomentsFromMot(q, qPlotLabel, time)
%       format_jntmomentsFromMot(fnames, timeRange,...
%               subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%       Suptitle(titleString)
%
% ASA, 11-05, revised 2-06


% Display status on screen.
status = sprintf('\n\n\n%s%s', ...
            'Plotting Joint Moments vs Time for Subject ', subject);
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

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}                               
figContents = ref_jntmomentPlotLabels;

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

    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot data from UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_jntmomentsFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, fileNum);
        end
    end
    
    % Format figure.
    format_jntmomentsFromMot(fnames, timeRange, ...
            figContents(figNum).subplotTitle, ...
            figContents(figNum).subplotAxisLabel, ...
            figContents(figNum).subplotRange, ...
            figContents(figNum).subplotTick);

    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of Joint Moments:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end
      
% Query user.
done = printmenu(figHandleArray, [subject '_jntmoments']);
return;
