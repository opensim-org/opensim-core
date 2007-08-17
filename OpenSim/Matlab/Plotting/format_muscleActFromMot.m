function [] = format_muscleActFromMot(fnames, fnameMeasured, timeRange, ...
          tInfo, subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
% Purpose:  Formats the current figure window created by
%           compare_muscleActFromMot().
%
% Input:    fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured EMG data
%           timeRange.min and timeRange.max are the min and max values  
%               of the time axis, respectively
%           tInfo contains the following 'trial info', in addition to
%               other information:
%               *.mass - subject mass (used to scale axes for force data)
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% ASA, 11-05, revised 2-06


% Specify attributes of figure windows.
nPlotRows = 5;                      
nPlotCols = 2;
nSubPlots = nPlotRows * nPlotCols;  

% Specify attributes of all subplots.
tFontName = 'helvetica';            % attributes of subplot titles
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';            % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';

% Specify time axis range and tick values.
xmin = timeRange.min;               
xmax = timeRange.max;
timeAxisIsPercentGc = strcmpi( tInfo.timeAxisLabel, 'percent of gait cycle' );
if timeAxisIsPercentGc
    xval = 0:  25: xmax;
elseif xmax - xmin > 1.0
    xval = 0: 0.5: xmax;
elseif (xmax - xmin <= 1.0) && (xmax - xmin > 0.5)
    xval = 0: 0.2: xmax;
else
    xval = 0: 0.1: xmax;
end

% Specify y-axis limits and tick labels for vertical GRF data.
fmin = 0;                                
fmax = 1.5 * 9.8 * tInfo.mass;   
fval = 0:200:fmax;

% Format figure.
for plotIndex = 1:nSubPlots  
    subplot(nPlotRows, nPlotCols, plotIndex)
    hold on;

    % Set background color.
    set(gca, 'Color', tInfo.bgColor);
    
    if ~isempty(subplotTitle{plotIndex})
        title(subplotTitle{plotIndex});
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
    end
    
    set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
    if plotIndex == 1 | plotIndex == 2 | plotIndex == 9 | plotIndex == 10
        xlabel(tInfo.timeAxisLabel);
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    end
    
    if plotIndex == 1 | plotIndex == 2
        ymin = fmin;
        ymax = fmax;
        yval = fval;
        set(gca, 'ylim', [ymin ymax], 'ytick', yval, ...
               'FontName', aFontName, 'FontSize', aFontSize, ...
               'TickDir', aTickDir);
    elseif ~isempty(subplotRange{plotIndex})   
        ymin = subplotRange{plotIndex}(1);
        ymax = subplotRange{plotIndex}(2);
        if ~isempty(subplotTick{plotIndex})
            yval = subplotTick{plotIndex};
        else
            yval = ymin: round((ymax-ymin)/4) :ymax;
        end
        set(gca, 'ylim', [ymin ymax], 'ytick', yval, ...
               'FontName', aFontName, 'FontSize', aFontSize, ...
               'TickDir', aTickDir);
    else
        ytickpositions = get(gca, 'Ytick');
        if length(ytickpositions > 5)
            new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
            set(gca, 'Ytick', new_ytickpositions);
        end  
    end
    
    if ~isempty(subplotAxisLabel{plotIndex})
        ylabel(subplotAxisLabel{plotIndex});
            a = get(gca, 'ylabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);   
    end
    set(gca, 'Box', 'off');
    
    % Set foreground colors.
    set(get(gca, 'Title'), 'Color', tInfo.fgColor);
    set(get(gca, 'XLabel'), 'Color', tInfo.fgColor);
    set(get(gca, 'YLabel'), 'Color', tInfo.fgColor);
    set(gca, 'XColor', tInfo.fgColor);
    set(gca, 'YColor', tInfo.fgColor);
end

% Add legend.
subplot(nPlotRows, nPlotCols, 3);
set(gca, 'xlim', [0 1]);
styles = {'blue', 'red', 'cyan', 'magenta', 'green'};

if ~isempty(fnameMeasured)
    for i = 1:length(fnames)
        legendString{i} = [fnames{i}, ':  ', styles{i}];
    end
    legendString{i+1} = ' ';  
    legendString{i+2} = 'EMG Envelope, Ensemble Average +/- 1SD:  green';
    legendString{i+3} = 'EMG On/Off Times from Perry 1992:  black bars';

else
    for i = 1:length(fnames)
        legendString{i} = [fnames{i}, ':  ', styles{i}];
    end
    legendString{i+1} = ' ';  
    legendString{i+2} = 'EMG On/Off Times from Perry 1992:  black bars';
end

for i = 1:length(legendString)
    x = 0;
    y = 1 - 0.15*i;
    text(x, y, legendString{i}, 'FontSize', 8, 'Interpreter', 'none');
end
axis off;
return;