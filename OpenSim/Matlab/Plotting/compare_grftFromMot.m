function [] = compare_grftFromMot(subject, fnames, fnameMeasured, ...
                                                tInfo, figHandleArray)
% Purpose:  Creates two figure windows specified by figHandleArray and,
%           for each file in fnames, overlays plots of:
%           (1) R Fx, Fy, Fz, Tvertical, COPx, and COPz vs time
%           (2) L Fx, Fy, Fz, Tvertical, COPx, and COPz vs time
%           (3) measured GRF data from file fnameMeasured

% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured joint angle data
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.speed  - average walking speed during trial in m/s
%                          (used for displaying plot title)
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
%
%           NOTES:
%           (1) Column headings for the force data are assumed to
%               be in the same order as written by write_c3dMotFile().
%           (2) Force data are plotted in the coordinate system
%               corresponding to Darryl's gait23*.jnt model.
%         
% Called Functions:
%       read_motionFile(fname)
%       ref_grftPlotLabels()
%       overlay_grftMeasured(qMeasured, qPlotLabel, time, limb)
%       overlay_grftFromMot(q, qPlotLabel, time, limb, fileNum)
%       format_grftFromMot(fnames, fnameMeasured, timeRange, ...
%               subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%       Suptitle(titleString)
%
% ASA, 11-05, revised 2-06

[q, time, timeRange] = get_commonComparisonData('GRF Data vs Time', subject, fnames, tInfo);

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
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}                            
figContents = ref_grftPlotLabels;

% Specify attributes of figure windows.
nPlotRows = 4;                      
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

    % Specify limb corresponding to current figure.
    switch figNum
        case 1
            limb = 'R';
        case 2
            limb = 'L';
    end
    
    % Plot measured GRF data.
    for plotIndex = 1:nSubPlots
        subplot(nPlotRows, nPlotCols, plotIndex);
        overlay_grftMeasured(qMeasured, ...
            figContents(figNum).qMeasuredLabel{plotIndex}, ...
            timeMeasured, limb);
    end
    
    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot data from UW-Gait Workflow.
        for plotIndex = 1:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_grftFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, limb, fileNum);
        end
    end
           
    % Format figure.
    format_grftFromMot(fnames, fnameMeasured, timeRange, ...
            figContents(figNum).subplotTitle, ...
            figContents(figNum).subplotAxisLabel, ...
            figContents(figNum).subplotRange, ...
            figContents(figNum).subplotTick);

    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of GRFs:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end

% Query user.
done = printmenu(figHandleArray, [subject '_grft']);
return;
