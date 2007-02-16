function [] = compare_actuatorFrcFromMot(subject, fnames, tInfo, figHandleArray, ref_dataFormat)
% Purpose:  Creates figure windows specified by figHandleArray and, 
%           for each file in fnames, overlays plots of:
%           (1) CMC-generated actuator forces vs time,
%                   for actuators specified in ref_actuatorFrcPlotLabels()
%           (2) L & R vertical GRF vs time, for reference
%         
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
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
%   ref_actuatorFrcPlotLabels()
%   overlay_grftVertical(q, qPlotLabel, time, limb, fileNum)
%   overlay_actuatorDataFromMot(q, qPlotLabel, time, fileNum)
%   format_actuatorFrcFromMot(fnames, timeRange, tInfo ...
%              subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   Suptitle(titleString)
%
% CTJ, 02-07, adapted from:
% ASA, 11-05, revised 2-06

[q, time, timeRange, ictoEvents] = get_commonComparisonData('Actuator Forces vs Time', subject, fnames, tInfo);

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}    
figContents = ref_actuatorFrcPlotLabels;

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
             
        % Plot data from SimTrack Workflow.
        for plotIndex = 3:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_actuatorDataFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, fileNum);
        end
    end
    
    % Format figure.
    format_actuatorFrcFromMot(fnames, timeRange, tInfo, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
                                
    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of Actuator Forces:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end

% Query user.
done = printmenu(figHandleArray, [subject '_muscleFrc']);
return;
   
