function [] = overlay_emgFigTitles(limb, muscle, cSim, tInfo, figHandle)
% Purpose:  Adds titles to the current figure, specified by figHandle,
%           created by create_emgOnOffFig().
%
% Input:    limb is the limb corresponding to the current EMG channel
%           muscle is the muscle corresponding to the current EMG channel
%           cSim is a structure returned from extract_c3dSimData()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%           figHandle is the number of the current figure window.
%
% Called Functions:
%          Suptitle(titleString)
%          
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 1;

% Specify attributes of subplot title.
tFontName = 'helvetica';         
tFontSize = 9;
tVerticalAlignment = 'middle';

% Display muscle name corresponding to EMG channel as 1st subplot title.
figure(figHandle); 
subplot(nPlotRows, nPlotCols, 1);
    hold on;
    muscleString = sprintf('%s%s%s', char(limb), ' ', char(muscle));
    title(muscleString);                    
    t = get(gca, 'title');
    set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);

% Display subject and trial information at top of figure window.
titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
   'EMG On/Off Times:  Subject ', char(cSim.subject), '-', tInfo.trial, ...
   ', Speed ', tInfo.speed, ' m/s');
Suptitle(titleString);      % MATLAB m-file for adding "super title".
return;