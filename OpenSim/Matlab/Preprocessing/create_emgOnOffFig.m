function [] = create_emgOnOffFig(emgData, fzData, aTime, figHandle)
% Purpose:  Creates a new figure window and plots processed EMG and 
%           vertical GRF data vs. time, for a single EMG channel.    
%
% Input:    emgData is a structure corresponding to one EMG channel,  
%             with the following format, sampled at the analog frame rate:
%               *.raw   - raw EMG signal
%               *.band  - after applying band-pass filter
%               *.rect  - after full-wave rectifying the filtered data
%               *.low   - after applying low-pass filter to rectified data
%               *.nrect - after normalizing rectified data
%               *.nlow  - after normalizing low-pass filtered data
%           fzData(aFrameNum, cycleNum) is a matrix of vertical GRF data 
%               for the limb of interest, where each column contains the
%               data for one FP strike.
%           aTime is an array of times corresponding to the analog data
%           figHandle is the number of the figure window of interest
%          
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 1;
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.8*scnheight];
figColor = 'w';

% Specify attributes of subplots.
aFontName = 'helvetica';          % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
bLineStyle = '-';                 % line style for band-pass filtered data
bLineWidth = 0.5;
bLineColor = 'k';
rLineStyle = '-';                 % line style for rectified data 
rLineWidth = 0.5;                   
rLineColor = 'k';
lLineStyle = '-';                 % line style for low-pass filtered data 
lLineWidth = 2;                   
lLineColor = 'k';
gLineStyle = '-';                 % line style for GRF data
gLineWidth = 1.5;
gLineColor = 'k';

% Set x-axis limits and tick labels.
xmax = max(aTime);               
xmin = 0;
xval = xmin:0.5:xmax;

% Generate figure.
figure(figHandle);     
clf;
set(gcf, 'Position', figPos, 'Color', figColor);
   
% Plot band-pass filtered EMG data.
subplot(nPlotRows, nPlotCols, 1);
b = plot(aTime, emgData.band); 
set(b, 'LineStyle', bLineStyle, 'LineWidth', bLineWidth, ...
        'Color', bLineColor);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
maxBand = max(abs(emgData.band));
ymax = ceil(maxBand);
ymin = -ymax;
yval = [ymin 0 ymax];
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ylabel('band-pass filtered EMG (V)');
a = get(gca, 'ylabel');
set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'Box', 'off');
    
% Plot normalized rectified and low-pass filtered EMG data.
subplot(nPlotRows, nPlotCols, 2);
r = plot(aTime, emgData.nrect); 
set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
        'Color', rLineColor);
hold on;
l = plot(aTime, emgData.nlow); 
set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
        'Color', lLineColor);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ymax = 1;
ymin = 0;
yval = ymin:0.5:ymax;
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ylabel('normalized rectified EMG');
a = get(gca, 'ylabel');
set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'Box', 'off');
    
% Plot vertical GRF data.
subplot(nPlotRows, nPlotCols, 4);
[nAnalogFrames, nHits] = size(fzData);
for fpHitNum = 1:nHits
    g = plot(aTime, fzData(:, fpHitNum));
    set(g, 'LineStyle', gLineStyle, 'LineWidth', gLineWidth, ...
        'Color', gLineColor);
    hold on;
end
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
xlabel('time (s)');
a = get(gca, 'xlabel');
set(a, 'FontName', aFontName, 'FontSize', aFontSize);
maxFz = max(max(fzData));
ymax = maxFz + 0.1*maxFz;
ymin = 0;
yval = ymin:200:ymax;                         
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ylabel('vertical GRF (N)');
a = get(gca, 'ylabel');
set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'Box', 'off');

return;