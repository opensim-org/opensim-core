function plot_dataFromMotOrStoFiles( plotSettings )

% Set the figure position and background color.
figure( plotSettings.figureNumber );
clf;
set( gcf, ...
    'Units',    plotSettings.figureUnits, ...
    'Position', plotSettings.figurePosition, ...
    'Color',    plotSettings.figureBgColor );

% Set the plot title.
title( plotSettings.figureTitle );
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

% Determine data columns and common time column to use for all curves.
timeAndDataColumns = get_timeAndDataColumns( ...
    plotSettings.curveSourceFiles, ...
    plotSettings.curveSourceColumnLabels, ...
    plotSettings.curveRepeatedSourceColumnNumbers );
time = timeAndDataColumns{1};

% replace time with percent gait cycle
% column; use earlier code I have for converting from time to cycle, and in
% plotSettings include all tInfo type stuff as input; put this tInfo info
% into plotSettings in evaluateAndCompareSimulations.m, and later in
% make_plots*.m.  First I'll code stuff here, then make sure I have the
% right parameters coming in from evaluateAndCompareSimulations, and then
% add stuff to plot_multiple*.m, and then to make_plots*.m based on what I
% added to evaluateAndCompareSimulations.m.

% Convert time column and time axis from time to percent of gait cycle if
% necessary.
if strcmpi( plotSettings.timeOrPercent, 'Percent' )
    tInfo = plotSettings.trialInfo;
    ictoEvents = plotSettings.ictoEvents;
    time = convert_timeToCycle( time, tInfo.gcLimb, tInfo, ...
        ictoEvents, tInfo.analogRate, tInfo.tZeroAtFirstIC );
end

% Plot the curves!
hold on;
numberOfCurves = length( timeAndDataColumns ) - 1;
for curveNum = 1 : numberOfCurves
    dataColumnIndex = curveNum + 1;
    p = plot( ...
        time, ...
        timeAndDataColumns{ dataColumnIndex } );
    set( p, ...
        'LineStyle', plotSettings.curveStyles{ curveNum }, ...
        'LineWidth', plotSettings.curveWidths{ curveNum }, ...
        'Color',     plotSettings.curveColors{ curveNum } );
end

% Create a legend.
legend( plotSettings.curveLabels );

% Compute time axis limits and ticks automatically, if user said to do so.
if plotSettings.computeTimeLimitsAndTicksAutomatically
    timeMin = min( time );
    timeMax = max( time );
    timeTickSeparation = ( timeMax - timeMin ) / 3;
    plotSettings.xAxisRange = [ timeMin timeMax ];
    plotSettings.xAxisTicks = timeMin : timeTickSeparation : timeMax;
end

% Compute vertical axis limits and ticks automatically, if user said to do
% so.
if plotSettings.computeVerticalAxisLimitsAndTicksAutomatically
    % Determine min and max values and range of all data being plotted.
    minCurveValue = min( timeAndDataColumns{2} );
    maxCurveValue = max( timeAndDataColumns{2} );
    for curveNum = 2 : numberOfCurves
        dataColumnIndex = curveNum + 1;
        mincv = min( timeAndDataColumns{ dataColumnIndex } );
        maxcv = max( timeAndDataColumns{ dataColumnIndex } );
        minCurveValue = min( mincv, minCurveValue );
        maxCurveValue = max( maxcv, maxCurveValue );
    end
    range = maxCurveValue - minCurveValue;
    nextLowerExponentOfTen = floor( log10( range / 2 ) );
    nextLowerPowerOfTen = 10 ^ nextLowerExponentOfTen;
    minRoundedValue = floor( minCurveValue / nextLowerPowerOfTen ) * ...
        nextLowerPowerOfTen;
    maxRoundedValue =  ceil( maxCurveValue / nextLowerPowerOfTen ) * ...
        nextLowerPowerOfTen;
    roundedRange = maxRoundedValue - minRoundedValue;
    plotSettings.yAxisRange = [ minRoundedValue maxRoundedValue ];
    plotSettings.yAxisTicks = ...
        minRoundedValue : roundedRange / 2 : maxRoundedValue;
end

% Overlay the zero line.
if plotSettings.zeroLineOn
    zeroLine = zeros( length( time ), 1 );
    z = plot( time, zeroLine );
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
