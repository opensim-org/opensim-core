function [] = plot_emgTrialData(emgR, emgL, gaitCycle, perryData, figHandle)
% Purpose:  Plots EMG envelopes vs gait cycle for the muscle of interest,
%           for all R and L cycles of a single trial, as stored in emgR
%           and emgL.  For reference, the EMG on/off times reported 
%           by Perry, for muscles in perryData, are also displayed.
%
% Input:    emgR{nRcycles} and emgL{nLcycles} are cell arrays containing
%               normalized, rectified, filtered EMG data, sampled at the
%               analog rate, for the muscle of interest.
%           gaitCycle is a structure with the following format:
%                       *.R(nRcycles).nAnalogFrames, .aCycle
%                       *.L(nLcycles).nAnalogFrames, .aCycle
%           perryData is a structure with the following format:
%                       *.muscle    - name of muscle
%                       *.onoff     - nx2 matrix of [on off] events, 
%                                      where each row is one 'burst'
%                       *.onoffAlt  - 'alternate' on/off timings reported 
%           figHandle is the handle number of the new figure window
%
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 3;                      
nPlotCols = 1;
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.8*scnheight];
figColor = 'w';

% Specify attributes of subplots.
tFontName = 'helvetica';            % attributes of subplot titles
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';            % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
rLineStyle = {'-', '--'};           % style for plotting R cycle data
rLineWidth = 1.0;                  
rLineColor = 'r';
lLineStyle = {'-', '--'};           % style for plotting L cycle data 
lLineWidth = 1.0;                   
lLineColor = 'b';
pLineStyle = '-';                   % style for Perry's on/off data
pLineWidth = 6.0;                  
pLineColor = 'k';
qLineStyle = '-';                   % style for Perry's on/off alt data
qLineWidth = 2.5;                  
qLineColor = 'k';


% Specify % gait cycle axis.
xmin = 0;                           % x-axis limits and tick labels
xmax = 100;
xval = [0 20 40 60 80 100];

% Generate figure window.
figure(figHandle);     
clf;
set(gcf, 'Position', figPos, 'Color', figColor);

% Plot data for each R gait cycle ...
subplot(nPlotRows, nPlotCols, 1);
nRcycles = length(emgR);
for cycleNum = 1:nRcycles
    r = plot(gaitCycle.R(cycleNum).aCycle, emgR{cycleNum});
    set(r, 'LineStyle', rLineStyle{cycleNum}, 'LineWidth', rLineWidth, ...
                         'Color', rLineColor);
    hold on;
end
title('RIGHT')
    t = get(gca, 'title');
    set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
ylabel('normalized EMG envelope');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'ylim', [0 1], 'ytick', [0:0.5:1], ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'Box', 'off');    
 
% Plot data for each L gait cycle ...
subplot(nPlotRows, nPlotCols, 2);
nLcycles = length(emgL);
for cycleNum = 1:nLcycles
    l = plot(gaitCycle.L(cycleNum).aCycle, emgL{cycleNum});
    set(l, 'LineStyle', lLineStyle{cycleNum}, 'LineWidth', rLineWidth, ...
                         'Color', lLineColor);
    hold on;
end
title('LEFT')
    t = get(gca, 'title');
    set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
ylabel('normalized EMG envelope');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'ylim', [0 1], 'ytick', [0:0.5:1], ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'Box', 'off');    

% Plot EMG on/off timing from Perry for reference.
subplot(nPlotRows, nPlotCols, 3);
nRefMuscles = length(perryData);
ymin = 0;                           
ymax = 1;
dy = 1/(nRefMuscles + 1);
yval = [];
for refNum = 1:nRefMuscles
    yPos = 1 - dy*refNum;
    yArray = [yPos yPos];
    [nOn, nCol] = size(perryData(refNum).onoff);
    for burstNum = 1:nOn
        xArray = [perryData(refNum).onoff(burstNum, 1) ...
                    perryData(refNum).onoff(burstNum, 2)];
        p = plot(xArray, yArray);        
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                         'Color', pLineColor);
        hold on;
    end
    if ~isempty(perryData(refNum).onoffAlt)
        [nOn, nCol] = size(perryData(refNum).onoffAlt);
        for burstNum = 1:nOn
            xArray = [perryData(refNum).onoffAlt(burstNum, 1) ...
                        perryData(refNum).onoffAlt(burstNum, 2)];
            q = plot(xArray, yArray);        
            set(q, 'LineStyle', qLineStyle, 'LineWidth', qLineWidth, ...
                             'Color', qLineColor);
            hold on;
        end
    end
    text(1, yPos+0.06, perryData(refNum).muscle, 'FontSize', 8);
end
title('EMG On/Off Times from Perry')
    t = get(gca, 'title');
    set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
xlabel('% gait cycle');
    a = get(gca, 'xlabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'ylim', [ymin ymax], 'ytick', yval, ...
    'FontName', aFontName, 'FontSize', aFontSize, ...
    'TickDir', aTickDir);
set(gca, 'Box', 'off');           
return;    
  