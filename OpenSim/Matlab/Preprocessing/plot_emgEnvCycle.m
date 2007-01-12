function [] = plot_emgEnvCycle(emgTOI, emgChannel)
% Purpose:  Plots the normalized rectified EMG signal and EMG envelope vs
%           gait cycle, for each cycle in emgTOI, for the EMG channel 
%           specified by emgChannel, on the current subplot.
%
% Input:   emgTOI is a structure with the following format: 
%               *{cycleNum}{emgChannel}
%               *{cycleNum}{emgChannel}
%                       .raw, .band, .rect, .low, .nrect, .nlow
%          emgChannel specifies the EMG channel of interest
%
% ASA, 12-05


% Specify attributes of subplot.
aFontName = 'helvetica';          % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
rLineStyle = '-';                 % line styles for rectified data 
rLineWidth = 0.5;                   
rLineColor = {'b', 'g','r'};
lLineStyle = '-';                 % line styles for low-pass filtered data 
lLineWidth = 1.5;                   
lLineColor = {'b', 'g','r'};

% Set x-axis limits and tick labels.
xmin = 0;                           % x-axis limits and tick labels
xmax = 100;
xval = [0 20 40 60 80 100];

% For each cycle stored in emgTOI ...
nCycles = min(3,length(emgTOI));
for cycleNum = 1:nCycles

    % Create % gait cycle array.
    nAnalogFrames = length(emgTOI{cycleNum}{emgChannel}.nlow);
    delta = 100.0/(nAnalogFrames - 1);   
    aCycle = 0:delta:100;

    % Add data to subplot.
    r = plot(aCycle, emgTOI{cycleNum}{emgChannel}.nrect);
        set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
               'Color', rLineColor{cycleNum});
        hold on;
    l = plot(aCycle, emgTOI{cycleNum}{emgChannel}.nlow);
        set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
               'Color', lLineColor{cycleNum});
        hold on;
end

% Format axes.
ylabel('normalized rectified EMG');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'ylim', [0 1], 'ytick', [0:0.5:1], ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'Box', 'off');     
return;    
  
