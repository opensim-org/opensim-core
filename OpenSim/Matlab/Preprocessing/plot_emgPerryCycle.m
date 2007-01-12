function [] = plot_emgPerryCycle(perryData)
% Purpose:  Displays the EMG on/off times reported by Perry (1992) 
%           vs gait cycle, for the muscles specified in perryData, 
%           on the current subplot.
%
%           perryData is a structure with the following format:
%                       *.muscle    - name of muscle
%                       *.onoff     - nx2 matrix of [on off] events, 
%                                      where each row is one 'burst'
%                       *.onoffAlt  - 'alternate' on/off timings reported 
%
% ASA, 12-05


% Specify attributes of subplot.
tFontName = 'helvetica';            % attributes of subplot title
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';            % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
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

% Plot EMG on/off timing from Perry for reference.
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
  