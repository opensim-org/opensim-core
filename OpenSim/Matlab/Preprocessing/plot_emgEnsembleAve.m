function [] = plot_emgEnsembleAve(emgEnsembleAve, cycle)
% Purpose:  Plots ensemble-averaged EMG envelope +/- 1SD vs gait cycle 
%           on the current subplot.
%
% Input:    emgEnsembleAve is a structure with the following format:
%               *.limb 
%                .muscle 
%                .envAve
%                .envSD
%           cycle is an array of % gait cycle vales corresponding to
%               emgEnsembleAve
%
% ASA, 12-05


% Specify attributes of subplot.
tFontName = 'helvetica';            % attributes of subplot title
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';            % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
pLineStyle = '-';                   % style for plotting ensemble average
pLineWidth = 2.0;                  
pLineColor = 'k';
dLineStyle = ':';                   % style for plotting SD
dLineWidth = 1.0;                  
dLineColor = 'k';

% Set x-axis limits and tick labels.
xmin = 0;                           % x-axis limits and tick labels
xmax = 100;
xval = [0 20 40 60 80 100];

% Add averaged data +/- 1 SD to subplot.
p = plot(cycle, emgEnsembleAve.envAve);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
d = plot(cycle, emgEnsembleAve.envAve + emgEnsembleAve.envSD);
        set(d, 'LineStyle', dLineStyle, 'LineWidth', dLineWidth, ...
               'Color', dLineColor);
        hold on;
d = plot(cycle, emgEnsembleAve.envAve - emgEnsembleAve.envSD);
        set(d, 'LineStyle', dLineStyle, 'LineWidth', dLineWidth, ...
               'Color', dLineColor);
        hold on;
    
muscleString = sprintf('%s%s%s', char(emgEnsembleAve.limb), ' ', ...
                                    char(emgEnsembleAve.muscle));
title(muscleString);                    
t = get(gca, 'title');
set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
    'VerticalAlignment', tVerticalAlignment);
ylabel('ensemble-averaged EMG');
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
  