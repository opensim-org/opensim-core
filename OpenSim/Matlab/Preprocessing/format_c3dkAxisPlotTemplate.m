function [] = format_c3dkAxisPlotTemplate(plotIndex, vTime)
% Purpose:  Formats the current subplot, identified by plotIndex,
%           of the figure window created by create_kAxisFig()
%
% Inputs:   plotIndex is the index number of the current subplot
%           vTime is an array of time values corresponding to video frames
%               of the 'simulateable' segment
%
% ASA, 7-05, rev 10-05 for C3D file

% Specify attributes of all subplots.
tFontName = 'helvetica';            % attributes of subplot titles
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';            % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
pLineStyle = '-';                   % attributes of zero line
pLineWidth = 0.5;
pLineColor = [0.3, 0.3, 0.3];

% Specify time axis limits.
tmin = 0;
tmax = max(vTime);
tval = tmin:0.5:tmax;

% Format subplot(nPlotRows, nPlotCols, plotIndex).
switch plotIndex
    case 1
        axis off;  

    case 2
        axis off;  
    
    case 3
        title('hip rotation (int/ext)');
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
        xlabel('time (s)');
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        ylabel('joint angle (deg)');
            a = get(gca, 'ylabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'ylim', [-50 50], 'ytick', [-40:20:40], ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'Box', 'off');
        hold on;
        zeroT = [tmin tmax];
        zeroY = [0 0];
        p = plot(zeroT, zeroY);                      % add zero line
        set(p, 'LineWidth', pLineWidth, 'Color', pLineColor);

     case 4
        title('knee varus/valgus');
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
        xlabel('time (s)');
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        ylabel('joint angle (deg)');
            a = get(gca, 'ylabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'ylim', [-25 25], 'ytick', [-20:20:20], ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'Box', 'off');
        hold on;
        zeroT = [tmin tmax];
        zeroY = [0 0];
        p = plot(zeroT, zeroY);                      % add zero line
        set(p, 'LineWidth', pLineWidth, 'Color', pLineColor);

    case 5
        title('knee flexion/extension');
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
        xlabel('time (s)');
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'ylim', [-15 85], 'ytick', [-15:30:75], ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'Box', 'off');
        hold on;
        zeroT = [tmin tmax];
        zeroY = [0 0];
        p = plot(zeroT, zeroY);                      % add zero line
        set(p, 'LineWidth', pLineWidth, 'Color', pLineColor);

    case 6
        title('knee rotation (int/ext)');
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
        xlabel('time (s)');
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
        set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'ylim', [-50 60], 'ytick', [-40:20:40], ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
        set(gca, 'Box', 'off');       
        hold on;
        zeroT = [tmin tmax];
        zeroY = [0 0];
        p = plot(zeroT, zeroY);                      % add zero line
        set(p, 'LineWidth', pLineWidth, 'Color', pLineColor);
end
return;
