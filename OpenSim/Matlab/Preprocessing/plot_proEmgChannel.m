function [] = plot_proEmgChannel(emgData, aTime, emgChannel)
% Purpose:  Creates a new figure window and plots a single channel of 
%           EMG data vs time, before and after five stages of processing:
%           band-passed, rectified, rectified/low-passed, 
%           normalized rectified, and normalized rectified/low-passed 
%           (as processed by process_emgChannel()).
%
% Input:    emgData is a structure corresponding to one EMG channel,  
%             with the following format, sampled at the analog frame rate:
%               *.raw   - raw EMG signal
%               *.band  - after applying band-pass filter
%               *.rect  - after full-wave rectifying the filtered data
%               *.low   - after applying low-pass filter to rectified data
%               *.nrect - after normalizing rectified data
%               *.nlow  - after normalizing low-pass filtered data
%           aTime is an array of time values corresponding to proEMG
%           emgChannel is the EMG channel of interest
%           figHandle is the number of the new figure window.
%
%           NOTE:  The number of the figure window, figHandle, is
%                  defined to be the same as emgChannel.
%
% Called Functions:
%          Suptitle(titleString)
%          
% ASA, 9-05


% Specify attributes of figure window.
figHandle = emgChannel;
nPlotRows = 4;                      
nPlotCols = 1;
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.1*scnwidth, 0.05*scnheight, 0.6*scnwidth, 0.8*scnheight];
figColor = 'w';

% Specify attributes of subplots.
tFontName = 'helvetica';          % attributes of subplot title
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';          % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
pLineStyle = '-';                 % line style for raw EMG data
pLineWidth = 1;
pLineColor = 'k';
bLineStyle = '-';                 % line style for band-bass filtered data 
bLineWidth = 1;                   
bLineColor = 'b';
rLineStyle = '-';                 % line style for rectified data 
rLineWidth = 1;                   
rLineColor = 'r';
lLineStyle = '-';                 % line style for low-pass filtered data 
lLineWidth = 2;                   
lLineColor = 'k';

% Set x-axis limits and tick labels
xmax = max(aTime);                
xmin = 0;
dx = round((xmax-xmin)/4);
xval = [xmin xmin+dx xmin+2*dx xmin+3*dx xmin+4*dx];

% Generate figure.
figure(figHandle);     
clf
set(gcf, 'Position', figPos, 'Color', figColor);
   
% Plot raw EMG data.
subplot(nPlotRows, nPlotCols, 1);
p = plot(aTime, emgData.raw); 
set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
        'Color', pLineColor);
titleString = ['Raw EMG Signal'];    
title(titleString);
t = get(gca, 'title');
set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
ymax = ceil(max(emgData.rect));
ymin = -ymax;
yval = [ymin 0 ymax];
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
set(gca, 'Box', 'off');

% Plot band-pass filtered data.
subplot(nPlotRows, nPlotCols, 2);
b = plot(aTime, emgData.band); 
    set(b, 'LineStyle', bLineStyle, 'LineWidth', bLineWidth, ...
        'Color', bLineColor);
titleString = ['Band-Pass Filtered EMG Signal'];    
title(titleString);
t = get(gca, 'title');
set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
ymax = ceil(max(emgData.rect));
ymin = -ymax;
yval = [ymin 0 ymax];
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
set(gca, 'Box', 'off');

% Overlay rectified and low-pass filtered EMG data.
subplot(nPlotRows, nPlotCols, 3);
r = plot(aTime, emgData.rect); 
set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
        'Color', rLineColor);
hold on;
l = plot(aTime, emgData.low); 
set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
        'Color', lLineColor);
titleString = ['Rectified & Low-Pass Filtered EMG Signal'];    
title(titleString);
t = get(gca, 'title');
set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ymax = ceil(max(emgData.rect));
ymin = 0;
yval = [ymin ymax/2 ymax];
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
set(gca, 'Box', 'off');

% Overlay normalized rectified and low-pass filtered EMG data.
subplot(nPlotRows, nPlotCols, 4);
r = plot(aTime, emgData.nrect); 
set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
        'Color', rLineColor);
hold on;
l = plot(aTime, emgData.nlow); 
set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
        'Color', lLineColor);
titleString = ['Normalized Rectified & Low-Pass Filtered EMG Signal'];
title(titleString);
t = get(gca, 'title');
set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
     'FontSize', aFontSize, 'TickDir', aTickDir);
xlabel('time (s)');
a = get(gca, 'xlabel');
set(a, 'FontName', aFontName, 'FontSize', aFontSize);
ymin = 0;
ymax = 1;
yval = [ymin ymax/2 ymax];
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
set(gca, 'Box', 'off');        

% Add title to figure window.
titleString = sprintf('%s%d', 'Processing of EMG Channel ', emgChannel);
Suptitle(titleString);      % MATLAB m-file for adding "super title".
return;