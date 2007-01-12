function [] = create_emgAveOnOffFig(emgData, fzData, aTime, figHandle)
% Purpose:  Creates a new figure window and plots processed EMG and 
%           vertical GRF data vs. time, for a single EMG channel.    
%
% Input:  emgData is a structure corresponding to one EMG channel,  
%             with the following format, sampled at the analog frame rate:
%              *.emgRectified - array corresponding to
%                                 rectified, normalized EMG data,
%                                 sampled at analog frame rate
%              *.emgEnvelope  - array corresponding to 
%                                 rectified, filtered, normalized envelope,
%                                 sampled at analog frame rate
%              *.emgAve -       array corresponding to
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%              *.emgSD -        array corresponding to 1SD of
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%           fzData(aFrameNum, cycleNum) is a matrix of vertical GRF data 
%               for the limb of interest, where each column contains the
%               data for one FP strike.
%           aTime is an array of times corresponding to the analog data
%           figHandle is the number of the figure window of interest
%          
% ASA, 9-05, revised 12-05


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
rLineStyle = '-';                 % line style for rectified data 
rLineWidth = 0.5;                   
rLineColor = [0.8 0.8 0.8];
lLineStyle = '-';                 % line style for low-pass filtered data 
lLineWidth = 1.5;                   
lLineColor = [0.5 0.5 0.5];
pLineStyle = '-';                 % style for plotting ensemble average
pLineWidth = 2.0;                  
pLineColor = 'k';
dLineStyle = ':';                 % style for plotting SD
dLineWidth = 1.25;                  
dLineColor = 'k';
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
   
% Plot normalized rectified and low-pass filtered EMG data.
subplot(nPlotRows, nPlotCols, 1);
r = plot(aTime, emgData.emgRectified); 
set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
        'Color', rLineColor);
hold on;
l = plot(aTime, emgData.emgEnvelope); 
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

% Plot normalized rectified and ensemble-averaged EMG data.
subplot(nPlotRows, nPlotCols, 2);
r = plot(aTime, emgData.emgRectified); 
set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
        'Color', rLineColor);
hold on;
p = plot(aTime, emgData.emgAve);
set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
       'Color', pLineColor);
hold on;
d = plot(aTime, emgData.emgAve + emgData.emgSD);
set(d, 'LineStyle', dLineStyle, 'LineWidth', dLineWidth, ...
       'Color', dLineColor);
hold on;
d = plot(aTime, emgData.emgAve - emgData.emgSD);
set(d, 'LineStyle', dLineStyle, 'LineWidth', dLineWidth, ...
       'Color', dLineColor);
hold on;
set(gca, 'xlim', [xmin xmax], 'xtick', xval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ymax = 1;
ymin = 0;
yval = ymin:0.5:ymax;
set(gca, 'ylim', [ymin ymax], 'ytick', yval, 'FontName', aFontName, ...
    'FontSize', aFontSize, 'TickDir', aTickDir);
ylabel('ensemble-averaged EMG');
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