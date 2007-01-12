function [] = format_jntmomentsSagittal(fnames, fnameMeasured, ...
      timeRange, subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
% Purpose:  Formats the current figure window created by
%           compare_jntmomentsSagittal()
%
% Input:    fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           fnameMeasured is the name of a motion file containing 
%               measured joint moment data
%           timeRange.min and timeRange.max are the min and max values  
%               of the time axis, respectively
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% ASA, 11-05, revised 2-06


% Specify attributes of figure windows.
nPlotRows = 4;                      
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
if xmax - xmin > 1.0
    xval = 0: 0.5: xmax;
else
    xval = 0: 0.2: xmax;
end

% Format figure.
for plotIndex = 1:nSubPlots  
    subplot(nPlotRows, nPlotCols, plotIndex)
    hold on;
    
    if ~isempty(subplotTitle{plotIndex})
        title(subplotTitle{plotIndex});
            t = get(gca, 'title');
            set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
                'VerticalAlignment', tVerticalAlignment);
    end
    
    set(gca, 'xlim', [xmin xmax], 'xtick', xval, ...
            'FontName', aFontName, 'FontSize', aFontSize, ...
            'TickDir', aTickDir);
    if plotIndex == 5 | plotIndex == 6
        xlabel('time (s)');
            a = get(gca, 'xlabel');
            set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    end
    
    if ~isempty(subplotRange{plotIndex})   
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
end

% Add legend.
subplot(nPlotRows, nPlotCols, 7);
set(gca, 'xlim', [0 1]);
styles = {'cyan dotted', 'magenta dashed', ...
          'blue dot-dash', 'red solid', 'green dotted'};

if ~isempty(fnameMeasured)
    legendString{1} = [fnameMeasured, ':  black solid (measured)'];
    for i = 1:length(fnames)
        legendString{1+i} = [fnames{i}, ':  ', styles{i}];
    end
else
    for i = 1:length(fnames)
        legendString{i} = [fnames{i}, ':  ', styles{i}];
    end
end

for i = 1:length(legendString)
    x = 0;
    y = 1 - 0.15*i;
    text(x, y, legendString{i}, 'FontSize', 8, 'Interpreter', 'none');
end
axis off;
return;