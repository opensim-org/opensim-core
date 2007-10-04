function plot_multipleFiguresFromMotOrStoFiles( plotSettings )
%
% Read in all desired columns from multiple .mot or .sto files and plot
% them in separate figures, one figure per column label.  This function
% assumes that plotSettings.figureNumbers is an array of figure numbers
% rather than just a single number.  plotSettings.figureTitles is assumed
% to be an array with the same length as plotSettings.figureNumbers.
%

% Set figure properties that must be set before the data are plotted.
for j = 1 : length( plotSettings.figureNumbers )
    % Initialize current figure.
    figure( plotSettings.figureNumbers(j) );
    clf;
    % Set the figure position and background color.
    set( gcf, ...
        'Units',    plotSettings.figureUnits, ...
        'Position', plotSettings.figurePosition, ...
        'Color',    plotSettings.figureBgColor );
    % Set the plot title.
    title( plotSettings.figureTitles(j) );
    t = get( gca, 'Title' );
    set( t, ...
        'FontName',          plotSettings.titleFontName, ...
        'FontSize',          plotSettings.titleFontSize, ...
        'VerticalAlignment', plotSettings.titleVerticalAlignment );
    % Set the figure axis position and background color.
    axesPosition = get( gcf, 'DefaultAxesPosition' );
    axesPosition = [ ...
        axesPosition(1) - 0.05, axesPosition(2) - 0.05, ...
        axesPosition(3) + 0.12, axesPosition(4) + 0.12 ];
    set( gcf, 'DefaultAxesPosition', axesPosition );
    set( gca, 'Color', plotSettings.figureBgColor );
    % Set axis foreground colors.
    set( get( gca, 'Title'  ), 'Color', plotSettings.figureFgColor );
    set( get( gca, 'XLabel' ), 'Color', plotSettings.figureFgColor );
    set( get( gca, 'YLabel' ), 'Color', plotSettings.figureFgColor );
    set( gca, 'XColor', plotSettings.figureFgColor );
    set( gca, 'YColor', plotSettings.figureFgColor );
end

% Initialize min and max times and data values.
timeMin = NaN;
timeMax = NaN;
minCurveValue = zeros( length( plotSettings.figureNumbers ) ) * NaN;
maxCurveValue = zeros( length( plotSettings.figureNumbers ) ) * NaN;

% Read in each file one at a time, and while a file is loaded, plot all
% data from that file and then close it.
for i = 1 : length( plotSettings.curveSourceFiles )
    % Read data from next input file, unless we already plotted the data
    % for the last file.
    q = read_motionFile( plotSettings.curveSourceFiles{i} );
    time = q.data( :, 1 );
    % Update min and max times.
    timeMin = min( timeMin, min( time ) );
    timeMax = max( timeMax, max( time ) );
    for j = 1 : length( plotSettings.figureNumbers )
        % Set current figure.
        figure( plotSettings.figureNumbers(j) );
        % Get the input data column to plot in this figure from the current
        % file.
        dataIndices = find( strcmpi( ...
            plotSettings.curveSourceColumnLabels{ i, j }, q.labels ) );
        dataIndex = dataIndices( ...
            plotSettings.curveRepeatedSourceColumnNumbers{i} );
        data = q.data( :, dataIndex );
        % Update min and max data values.
        minCurveValue(j) = min( minCurveValue(j), min( data ) );
        maxCurveValue(j) = max( maxCurveValue(j), max( data ) );
        % Plot the data!
        hold on;
        p = plot( time, data );
        set( p, ...
            'LineStyle', plotSettings.curveStyles{i}, ...
            'LineWidth', plotSettings.curveWidths{i}, ...
            'Color',     plotSettings.curveColors{i} );
    end
    % Free up memory used up by input data that is no longer needed.
    clear q;
end

% Compute time axis limits and ticks automatically, if user said to do so.
if plotSettings.computeTimeLimitsAndTicksAutomatically
    timeTickSeparation = ( timeMax - timeMin ) / 3;
    plotSettings.xAxisRange = [ timeMin timeMax ];
    plotSettings.xAxisTicks = timeMin : timeTickSeparation : timeMax;
end

% Set figure properties that must be set after the data are plotted.
for j = 1 : length( plotSettings.figureNumbers )
    % Set current figure.
    figure( plotSettings.figureNumbers(j) );
    % Create a legend.
    legend( plotSettings.curveLabels );
    % Compute vertical axis limits and ticks automatically, if user said to
    % do so.
    if plotSettings.computeVerticalAxisLimitsAndTicksAutomatically
        range = maxCurveValue(j) - minCurveValue(j);
        nextLowerExponentOfTen = floor( log10( range / 2 ) );
        nextLowerPowerOfTen = 10 ^ nextLowerExponentOfTen;
        minRoundedValue = floor( minCurveValue(j) / nextLowerPowerOfTen ) * ...
            nextLowerPowerOfTen;
        maxRoundedValue =  ceil( maxCurveValue(j) / nextLowerPowerOfTen ) * ...
            nextLowerPowerOfTen;
        roundedRange = maxRoundedValue - minRoundedValue;
        plotSettings.yAxisRange = [ minRoundedValue maxRoundedValue ];
        plotSettings.yAxisTicks = ...
            minRoundedValue : roundedRange / 2 : maxRoundedValue;
    end
    % Overlay the zero line.
    if plotSettings.zeroLineOn
        zeroLine = zeros( length( plotSettings.xAxisRange ), 1 );
        z = plot( plotSettings.xAxisRange, zeroLine );
        set( z, ...
            'LineWidth', plotSettings.zeroLineWidth, ...
            'Color',     plotSettings.zeroLineColor );
    end
    % Set x-axis properties.
    set( gca, ...
        'XLim',     plotSettings.xAxisRange, ...
        'XTick',    plotSettings.xAxisTicks, ...
        'FontName', plotSettings.axisFontName, ...
        'FontSize', plotSettings.axisFontSize, ...
        'TickDir',  plotSettings.axisTickDirection );
    xlabel( plotSettings.xAxisLabel );
    a = get( gca, 'XLabel' );
    set( a, ...
        'FontName', plotSettings.axisFontName, ...
        'FontSize', plotSettings.axisFontSize );
    % Set y-axis properties.
    set( gca, ...
        'YLim',     plotSettings.yAxisRange, ...
        'YTick',    plotSettings.yAxisTicks, ...
        'FontName', plotSettings.axisFontName, ...
        'FontSize', plotSettings.axisFontSize, ...
        'TickDir',  plotSettings.axisTickDirection );
    ylabel( plotSettings.yAxisLabel );
    a = get( gca, 'YLabel' );
    set( a, ...
        'FontName', plotSettings.axisFontName, ...
        'FontSize', plotSettings.axisFontSize );
    % Turn off the box surrounding the axes.
    set( gca, 'Box', 'off' );
end
