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
        'FontName',          plotSettings.titleFontName,          ...
        'FontSize',          plotSettings.titleFontSize,          ...
        'VerticalAlignment', plotSettings.titleVerticalAlignment, ...
        'Interpreter',       'none'                               );
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
if strcmpi( plotSettings.timeOrPercent, 'Percent' )
    leftTimeMin = NaN;
    leftTimeMax = NaN;
    rightTimeMin = NaN;
    rightTimeMax = NaN;
else
    timeMin = NaN;
    timeMax = NaN;
end
minCurveValue = zeros( length( plotSettings.figureNumbers ) ) * NaN;
maxCurveValue = zeros( length( plotSettings.figureNumbers ) ) * NaN;

% Read in each file one at a time, and while a file is loaded, plot all
% data from that file and then close it.
for i = 1 : length( plotSettings.curveSourceFiles )
    % Read data from next input file, unless we already plotted the data
    % for the last file.
    q = read_motionFile( plotSettings.curveSourceFiles{i} );
    time = q.data( :, 1 );
    % Convert time column and time axis from time to percent of gait cycle
    % if necessary.
    if strcmpi( plotSettings.timeOrPercent, 'Percent' )
        tInfo = plotSettings.trialInfo;
        ictoEvents = plotSettings.ictoEvents;
        leftTime = convert_timeToCycle( time, 'L', tInfo, ...
            ictoEvents, tInfo.analogRate, tInfo.tZeroAtFirstIC );
        rightTime = convert_timeToCycle( time, 'R', tInfo, ...
            ictoEvents, tInfo.analogRate, tInfo.tZeroAtFirstIC );
        % Update min and max times.
        leftTimeMin  = min( leftTimeMin,  min( leftTime  ) );
        leftTimeMax  = max( leftTimeMax,  max( leftTime  ) );
        rightTimeMin = min( rightTimeMin, min( rightTime ) );
        rightTimeMax = max( rightTimeMax, max( rightTime ) );
    else
        % Update min and max times.
        timeMin = min( timeMin, min( time ) );
        timeMax = max( timeMax, max( time ) );
    end
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
        if strcmpi( plotSettings.timeOrPercent, 'Percent' )
            if plotSettings.trialInfo.gcLimb{j} == 'R'
                time = rightTime;
            else
                time = leftTime;
            end
        end
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
    if strcmpi( plotSettings.timeOrPercent, 'Percent' )
        % For the left limb.
        leftTimeTickSeparation = ( leftTimeMax - leftTimeMin ) / 3;
        plotSettings.xAxisRangeLeft = [ leftTimeMin leftTimeMax ];
        plotSettings.xAxisTicksLeft = ...
            leftTimeMin : leftTimeTickSeparation : leftTimeMax;
        % For the right limb.
        rightTimeTickSeparation = ( rightTimeMax - rightTimeMin ) / 3;
        plotSettings.xAxisRangeRight = [ rightTimeMin rightTimeMax ];
        plotSettings.xAxisTicksRight = ...
            rightTimeMin : rightTimeTickSeparation : rightTimeMax;
    else
        timeTickSeparation = ( timeMax - timeMin ) / 3;
        plotSettings.xAxisRange = [ timeMin timeMax ];
        plotSettings.xAxisTicks = timeMin : timeTickSeparation : timeMax;
    end
end

% Determine speed from Cappellini-Ivanenko data to plot against the data
% from simulation, based on the speed of the subject's walk.
%
% Speed in km/h = ( Speed in m/s ) * ( 3600 s / 1 h ) * ( 1 km / 1000 m )
%               = ( Speed in m/s ) * 3.6
speedInKilometersPerHour = plotSettings.speedInMetersPerSecond * 3.6;
if speedInKilometersPerHour < 1.5
    sheet = '1w_';
elseif speedInKilometersPerHour < 2.5
    sheet = '2w_';
elseif speedInKilometersPerHour < 4
    sheet = '3w';
elseif speedInKilometersPerHour < 6
    sheet = '5w';
elseif speedInKilometersPerHour < 8
    sheet = '7w';
else
    sheet = '9w';
end

% Plot Cappellini-Ivanenko EMG data.
if plotSettings.plotLiteratureActivations
    % Read Cappellini-Ivanenko EMG data.
    emgData = xlsread( 'emg_averaged_walk_run.xls', sheet );
    % emgData( :, column ) is from 0% to 100% of gait cycle, with 201
    % data points, so its time vector is 0% to 100% by increments of 0.5%.
    emgTimeAxis = transpose( 0 : 0.005 : 1 );
    % emgData must be scaled from emgTimeAxis to the full percent
    % range of this trial.
    tInfo = plotSettings.trialInfo;
    ictoEvents = plotSettings.ictoEvents;
    [leftTimesScaled, leftEmgScaled] = scale_emgDataToTimeAxis( ...
        emgTimeAxis, emgData, ictoEvents, tInfo.analogRate, ...
        'L', tInfo, tInfo.tZeroAtFirstIC );
    [rightTimesScaled, rightEmgScaled] = scale_emgDataToTimeAxis( ...
        emgTimeAxis, emgData, ictoEvents, tInfo.analogRate, ...
        'R', tInfo, tInfo.tZeroAtFirstIC );
    % Convert timesScaled from time to percent of gait cycle if necessary.
    if strcmpi( plotSettings.timeOrPercent, 'Percent' )
        tInfo = plotSettings.trialInfo;
        ictoEvents = plotSettings.ictoEvents;
        leftTimesScaled = convert_timeToCycle( leftTimesScaled, ...
            'L', tInfo, ictoEvents, tInfo.analogRate, ...
            tInfo.tZeroAtFirstIC );
        rightTimesScaled = convert_timeToCycle( rightTimesScaled, ...
            'R', tInfo, ictoEvents, tInfo.analogRate, ...
            tInfo.tZeroAtFirstIC );
    end
    for j = 1 : length( plotSettings.figureNumbers )
        % Set current figure.
        figure( plotSettings.figureNumbers(j) );
        % Plot the appropriate literature EMG data column in figure j.
        hold on;
        columnIndex = plotSettings.emgColumnIndices(j);
        if columnIndex > 0
            ax1 = gca;
            set( ax1, 'Color', 'none' );
            ax2 = axes( 'Position', get( ax1, 'Position' ), ...
                'YAxisLocation', 'right' );
            set( ax2, 'Color', 'none', 'XTick', [], 'XTickLabel', [] );
            set( ax2, 'XLim', get( ax1, 'XLim' ) );
            % Get the time and EMG data for the appropriate limb.
            if tInfo.gcLimb{j} == 'R'
                timesScaled = rightTimesScaled;
                emgScaled = rightEmgScaled;
            else
                timesScaled = leftTimesScaled;
                emgScaled = leftEmgScaled;
            end
            hEmg = line( timesScaled, emgScaled( :, columnIndex ), ...
                'Parent', ax2 );
            set( hEmg, ...
                'LineStyle', plotSettings.literatureActivationCurveStyle, ...
                'LineWidth', plotSettings.literatureActivationCurveWidth, ...
                'Color',     plotSettings.literatureActivationCurveColor );
            %legend( hEmg, plotSettings.literatureActivationCurveLabel );
        end
    end
end

% Set figure properties that must be set after the data are plotted.
for j = 1 : length( plotSettings.figureNumbers )
    % Set current figure.
    figure( plotSettings.figureNumbers(j) );
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
        if strcmpi( plotSettings.timeOrPercent, 'Percent' )
            if plotSettings.trialInfo.gcLimb{j} == 'R'
                zeroLine = ...
                    zeros( length( plotSettings.xAxisRangeRight ), 1 );
                hold on;
                z = plot( plotSettings.xAxisRangeRight, zeroLine );
            else
                zeroLine = ...
                    zeros( length( plotSettings.xAxisRangeLeft ), 1 );
                hold on;
                z = plot( plotSettings.xAxisRangeLeft, zeroLine );
            end
        else
            zeroLine = zeros( length( plotSettings.xAxisRange ), 1 );
            hold on;
            z = plot( plotSettings.xAxisRange, zeroLine );
        end
        set( z, ...
            'LineWidth', plotSettings.zeroLineWidth, ...
            'Color',     plotSettings.zeroLineColor );
    end
    % Get handles for each axis.
    ax = get( gcf, 'Children' );
    if length( ax ) > 1
        rightAxis = ax(1);
        leftAxis = ax( end );
    else
        leftAxis = ax(1);
    end
    % Create a legend.
    if length( ax ) > 1
        lChildren = get( leftAxis,  'Children' );
        rChildren = get( rightAxis, 'Children' );
        %legend( ...
        %    [ lChildren(2) lChildren(1) rChildren(1) ], ...
        %    [ plotSettings.curveLabels 'Lit' ]          );
    else
        %legend( leftAxis, plotSettings.curveLabels );
    end
    if strcmpi( plotSettings.timeOrPercent, 'Percent' )
        if plotSettings.trialInfo.gcLimb{j} == 'R'
            range = plotSettings.xAxisRangeRight;
            ticks = plotSettings.xAxisTicksRight;
        else
            range = plotSettings.xAxisRangeLeft;
            ticks = plotSettings.xAxisTicksLeft;
        end
    else
        range = plotSettings.xAxisRange;
        ticks = plotSettings.xAxisTicks;
    end
    % Set x-axis properties.
    set( leftAxis, ...
        'XLim',     range,                         ...
        'XTick',    ticks,                         ...
        'FontName', plotSettings.axisFontName,     ...
        'FontSize', plotSettings.axisFontSize,     ...
        'TickDir',  plotSettings.axisTickDirection );
    xlabel( leftAxis, plotSettings.xAxisLabel );
    a = get( leftAxis, 'XLabel' );
    set( a, ...
        'FontName', plotSettings.axisFontName, ...
        'FontSize', plotSettings.axisFontSize );
    if length( ax ) > 1
        set( rightAxis, ...
            'XLim',     range,                     ...
            'FontName', plotSettings.axisFontName, ...
            'FontSize', plotSettings.axisFontSize );
        a = get( rightAxis, 'XLabel' );
        set( a, ...
            'FontName', plotSettings.axisFontName, ...
            'FontSize', plotSettings.axisFontSize );
    end
    % Set y-axis properties.
    set( leftAxis, ...
        'YLim',     plotSettings.yAxisRange, ...
        'YTick',    plotSettings.yAxisTicks );
    % Turn off the box surrounding the axes.
    set( leftAxis, 'Box', 'off' );
    if length( ax ) > 1
        set( rightAxis, 'Position', get( leftAxis, 'Position' ) );
        set( rightAxis, 'Box', 'off' );
    end
end
