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

% Plot literature moments alongside simulation moments if user said to do
% so.
if plotSettings.plotLiteratureMoments
    % Read in literature joint moment data from:
    % - Crowninshield et al. (1978)
    % - Cappozzo (1983)
    % - Cappozzo et al. (1975)
    % - Patriarco et al. (1981)
    % - Inman et al. (1981)
    % - Kadaba et al. (1989) - not used
    % - MacKinnon and Winter (1993) - not used
    momentDataArray = xlsread( 'LiteratureJointMoments.xls' );
    % Time arrays
    cappozzoTimesH = momentDataArray(36:50, 8);
    cappozzoTimesB = momentDataArray(36:52, 2);
    gaitCycleTimes = momentDataArray( 4:25, 3);
    % Back moments from 0% to 100% of a gait cycle
    % From Cappozzo (1983)
    origLumbarExtension = momentDataArray(36:50, 10);
    origLumbarBending   = momentDataArray(36:52,  6);
    origLumbarRotation  = momentDataArray(36:50, 12);
    % Hip flexion moments from 0% to 100% of a gait cycle
    % From Inman et al. 1981, Cappozzo et al. 1975, Crowninshield et al. 1978
    leftHipFlexionInman         = momentDataArray(4:25, 32);
    leftHipFlexionCappozzo      = momentDataArray(4:25, 15);
    leftHipFlexionCrowninshield = momentDataArray(4:25,  9);
    rightHipFlexionInman         = leftHipFlexionInman;
    rightHipFlexionCappozzo      = leftHipFlexionCappozzo;
    rightHipFlexionCrowninshield = leftHipFlexionCrowninshield;
    % Hip adduction moments from 0% to 100% of a gait cycle
    % From Crowninshield et al. 1978, Patriarco et al. 1981
    leftHipAdductionCrowninshield = momentDataArray(4:25,  6);
    leftHipAdductionPatriarco     = momentDataArray(4:25, 26);
    rightHipAdductionCrowninshield = leftHipAdductionCrowninshield;
    rightHipAdductionPatriarco     = leftHipAdductionPatriarco;
    % Hip rotation moments from 0% to 100% of a gait cycle
    % From Crowninshield et al. 1978, Patriarco et al. 1981
    leftHipRotationCrowninshield = momentDataArray(4:25,  8);
    leftHipRotationPatriarco     = momentDataArray(4:25, 25);
    rightHipRotationCrowninshield = leftHipRotationCrowninshield;
    rightHipRotationPatriarco     = leftHipRotationPatriarco;
    % Knee flexion moments from 0% to 100% of a gait cycle
    % From Cappozzo et al. 1975, Inman et al. 1981
    leftKneeFlexionCappozzo = momentDataArray(4:25, 18);
    leftKneeFlexionInman    = momentDataArray(4:25, 33);
    rightKneeFlexionCappozzo = leftKneeFlexionCappozzo;
    rightKneeFlexionInman    = leftKneeFlexionInman;
    % Ankle flexion moments from 0% to 100% of a gait cycle
    % Inman et al. 1981, Cappozzo et al. 1975
    leftAnkleFlexionInman    = momentDataArray(4:25, 34);
    leftAnkleFlexionCappozzo = momentDataArray(4:25, 21);
    rightAnkleFlexionInman    = leftAnkleFlexionInman;
    rightAnkleFlexionCappozzo = leftAnkleFlexionCappozzo;

    % Scale and interpolate back moment data columns to go from being expressed
    % as functions of cappozzoTimesH and cappozzoTimesB to gaitCycleTimes.  Do
    % this by first affinely scaling cappozzoTimesH and cappozzoTimesB to have
    % the same start and end values as gaitCycleTimes, and then
    % re-interpolating the back moment data columns to have gaitCycleTimes as
    % their domain using MATLAB's interp1 function.
    lengthOfCappozzoTimesH = cappozzoTimesH(end) - cappozzoTimesH(1);
    lengthOfCappozzoTimesB = cappozzoTimesB(end) - cappozzoTimesB(1);
    lengthOfGaitCycleTimes = gaitCycleTimes(end) - gaitCycleTimes(1);
    scaleFactorForCappozzoTimesH = lengthOfGaitCycleTimes / lengthOfCappozzoTimesH;
    scaleFactorForCappozzoTimesB = lengthOfGaitCycleTimes / lengthOfCappozzoTimesB;
    cappozzoTimesH = gaitCycleTimes(1) + ( cappozzoTimesH - cappozzoTimesH(1) ) * scaleFactorForCappozzoTimesH;
    cappozzoTimesB = gaitCycleTimes(1) + ( cappozzoTimesB - cappozzoTimesB(1) ) * scaleFactorForCappozzoTimesB;
    lumbarExtension = interp1(cappozzoTimesH, origLumbarExtension, gaitCycleTimes);
    lumbarBending   = interp1(cappozzoTimesB, origLumbarBending  , gaitCycleTimes);
    lumbarRotation  = interp1(cappozzoTimesH, origLumbarRotation , gaitCycleTimes);

    % Now we have all joint moments described as discrete functions of
    % gaitCycleTimes, which ranges from (0% to 100% of a gait cycle).  We then
    % pass these joint moment arrays to convert_jointMomentDataToTime to get
    % the joint moments for the number of gait cycles (e.g., 1.5 gait cycles)
    % of the current simulation, as functions of the time array of the current
    % simulation.
    jointMomentData = [ lumbarExtension lumbarBending lumbarRotation ...
                        leftHipFlexionInman leftHipFlexionCappozzo leftHipFlexionCrowninshield ...
                        leftHipAdductionCrowninshield leftHipAdductionPatriarco ...
                        leftHipRotationCrowninshield leftHipRotationPatriarco ...
                        leftKneeFlexionCappozzo leftKneeFlexionInman ...
                        leftAnkleFlexionInman leftAnkleFlexionCappozzo ];
    % Fill in t = 0.025 row that is empty (NaN) in all columns except the first
    % 3 columns.
    jointMomentData(2, 4:end) = 0.5 * (jointMomentData(1, 4:end) + jointMomentData(3, 4:end));
    % The literature joint moments were scaled by the mass of the UT walking
    % model, so unscale all joint moments by this mass and scale them by the
    % mass of the subject whose movement is being simulated in this trial.
    subjectMass = tInfo.mass; % just giving it a more descriptive name!
    UTModelMass = 71.002; % in kilograms
    scaleUnscaleFactor = subjectMass / UTModelMass;
    jointMomentData = jointMomentData * scaleUnscaleFactor;
    % Get trial info.
    tInfo = plotSettings.trialInfo;
    ictoEvents = plotSettings.ictoEvents;
    % Get the joint moments in the time units of this trial.
    [leftTimesScaled, leftJointMomentsScaled] = ...
        convert_jointMomentDataToTime( gaitCycleTimes, jointMomentData, ...
        ictoEvents, tInfo.analogRate, 'L', tInfo, tInfo.tZeroAtFirstIC );

    lumbarExtension               = leftJointMomentsScaled( :,  1 );
    lumbarBending                 = leftJointMomentsScaled( :,  2 );
    lumbarRotation                = leftJointMomentsScaled( :,  3 );
    leftHipFlexionInman           = leftJointMomentsScaled( :,  4 );
    leftHipFlexionCappozzo        = leftJointMomentsScaled( :,  5 );
    leftHipFlexionCrowninshield   = leftJointMomentsScaled( :,  6 );
    leftHipAdductionCrowninshield = leftJointMomentsScaled( :,  7 );
    leftHipAdductionPatriarco     = leftJointMomentsScaled( :,  8 );
    leftHipRotationCrowninshield  = leftJointMomentsScaled( :,  9 );
    leftHipRotationPatriarco      = leftJointMomentsScaled( :, 10 );
    leftKneeFlexionCappozzo       = leftJointMomentsScaled( :, 11 );
    leftKneeFlexionInman          = leftJointMomentsScaled( :, 12 );
    leftAnkleFlexionInman         = leftJointMomentsScaled( :, 13 );
    leftAnkleFlexionCappozzo      = leftJointMomentsScaled( :, 14 );

    jointMomentData = [ rightHipFlexionInman rightHipFlexionCappozzo rightHipFlexionCrowninshield ...
                        rightHipAdductionCrowninshield rightHipAdductionPatriarco ...
                        rightHipRotationCrowninshield rightHipRotationPatriarco ...
                        rightKneeFlexionCappozzo rightKneeFlexionInman ...
                        rightAnkleFlexionInman rightAnkleFlexionCappozzo ];
    % Fill in t = 0.025 row that is empty (NaN) in all columns.
    jointMomentData(2, :) = 0.5 * (jointMomentData(1, :) + jointMomentData(3, :));
    % The literature joint moments were scaled by the mass of the UT walking
    % model, so unscale all joint moments by this mass and scale them by the
    % mass of the subject whose movement is being simulated in this trial.
    jointMomentData = jointMomentData * scaleUnscaleFactor;
    % Get the joint moments in the time units of this trial.
    [rightTimesScaled, rightJointMomentsScaled] = ...
        convert_jointMomentDataToTime( gaitCycleTimes, jointMomentData, ...
        ictoEvents, tInfo.analogRate, 'R', tInfo, tInfo.tZeroAtFirstIC );

    rightHipFlexionInman           = rightJointMomentsScaled( :,  1 );
    rightHipFlexionCappozzo        = rightJointMomentsScaled( :,  2 );
    rightHipFlexionCrowninshield   = rightJointMomentsScaled( :,  3 );
    rightHipAdductionCrowninshield = rightJointMomentsScaled( :,  4 );
    rightHipAdductionPatriarco     = rightJointMomentsScaled( :,  5 );
    rightHipRotationCrowninshield  = rightJointMomentsScaled( :,  6 );
    rightHipRotationPatriarco      = rightJointMomentsScaled( :,  7 );
    rightKneeFlexionCappozzo       = rightJointMomentsScaled( :,  8 );
    rightKneeFlexionInman          = rightJointMomentsScaled( :,  9 );
    rightAnkleFlexionInman         = rightJointMomentsScaled( :, 10 );
    rightAnkleFlexionCappozzo      = rightJointMomentsScaled( :, 11 );

    % Colors for different literature joint moment curves, indicating which
    % source each curve is from.
    colorInman         = plotSettings.literatureMomentCurveColors{1};
    colorCappozzo1975  = plotSettings.literatureMomentCurveColors{2};
    colorCappozzo1983  = plotSettings.literatureMomentCurveColors{3};
    colorCrowninshield = plotSettings.literatureMomentCurveColors{4};
    colorPatriarco     = plotSettings.literatureMomentCurveColors{5};

    % Determine which moments to plot and add those data to
    % leftMomentsScaled and rightMomentsScaled.  Also save the appropriate
    % curve colors to momentColors, a cell array of color arrays or
    % characters.
    aLiteratureMomentShouldBePlotted = false;
    switch( plotSettings.literatureMomentToPlot )
        case 'hip_flexion_r'
            rightMomentsScaled = { ...
                rightHipFlexionInman ...
                rightHipFlexionCappozzo ...
                rightHipFlexionCrowninshield };
            momentColors = { colorInman colorCappozzo1975 colorCrowninshield };
            aLiteratureMomentShouldBePlotted = true;
        case 'hip_flexion_l'
            leftMomentsScaled = { ...
                leftHipFlexionInman ...
                leftHipFlexionCappozzo ...
                leftHipFlexionCrowninshield };
            momentColors = { colorInman colorCappozzo1975 colorCrowninshield };
            aLiteratureMomentShouldBePlotted = true;
        case 'hip_adduction_r'
            rightMomentsScaled = { ...
                rightHipAdductionCrowninshield ...
                rightHipAdductionPatriarco };
            momentColors = { colorCrowninshield colorPatriarco };
            aLiteratureMomentShouldBePlotted = true;
        case 'hip_adduction_l'
            leftMomentsScaled = { ...
                leftHipAdductionCrowninshield ...
                leftHipAdductionPatriarco };
            momentColors = { colorCrowninshield colorPatriarco };
            aLiteratureMomentShouldBePlotted = true;
        case 'hip_rotation_r'
            rightMomentsScaled = { ...
                rightHipRotationCrowninshield ...
                rightHipRotationPatriarco };
            momentColors = { colorCrowninshield colorPatriarco };
            aLiteratureMomentShouldBePlotted = true;
        case 'hip_rotation_l'
            leftMomentsScaled = { ...
                leftHipRotationCrowninshield ...
                leftHipRotationPatriarco };
            momentColors = { colorCrowninshield colorPatriarco };
            aLiteratureMomentShouldBePlotted = true;
        case 'knee_angle_r'
            rightMomentsScaled = { ...
                rightKneeFlexionCappozzo ...
                rightKneeFlexionInman };
            momentColors = { colorCappozzo1975 colorInman };
            aLiteratureMomentShouldBePlotted = true;
        case 'knee_angle_l'
            leftMomentsScaled = { ...
                leftKneeFlexionCappozzo ...
                leftKneeFlexionInman };
            momentColors = { colorCappozzo1975 colorInman };
            aLiteratureMomentShouldBePlotted = true;
        case 'ankle_angle_r'
            rightMomentsScaled = { ...
                rightAnkleFlexionCappozzo ...
                rightAnkleFlexionInman };
            momentColors = { colorCappozzo1975 colorInman };
            aLiteratureMomentShouldBePlotted = true;
        case 'ankle_angle_l'
            leftMomentsScaled = { ...
                leftAnkleFlexionCappozzo ...
                leftAnkleFlexionInman };
            momentColors = { colorCappozzo1975 colorInman };
            aLiteratureMomentShouldBePlotted = true;
        case 'lumbar_extension'
            leftMomentsScaled = { lumbarExtension };
            momentColors = { colorCappozzo1983 };
            aLiteratureMomentShouldBePlotted = true;
        case 'lumbar_bending'
            leftMomentsScaled = { lumbarBending };
            momentColors = { colorCappozzo1983 };
            aLiteratureMomentShouldBePlotted = true;
        case 'lumbar_rotation'
            leftMomentsScaled = { lumbarRotation };
            momentColors = { colorCappozzo1983 };
            aLiteratureMomentShouldBePlotted = true;
    end
    
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
    % Plot the appropriate literature moment data column.
    hold on;
    if aLiteratureMomentShouldBePlotted
        ax1 = gca;
        set( ax1, 'Color', 'none' );
        ax2 = axes( 'Position', get( ax1, 'Position' ), ...
            'YAxisLocation', 'right' );
        set( ax2, 'Color', 'none', 'XTick', [], 'XTickLabel', [] );
        set( ax2, 'XLim', get( ax1, 'XLim' ) );
        % Get the time and moment data for the appropriate limb.
        if tInfo.gcLimb == 'R'
            timesScaled = rightTimesScaled;
            momentsScaled = rightMomentsScaled;
        else
            timesScaled = leftTimesScaled;
            momentsScaled = leftMomentsScaled;
        end
        momentsScaledDimensions = size( momentsScaled );
        numCurves = momentsScaledDimensions(1);
        for i = 1 : numCurves
            hMoment = line( timesScaled, momentsScaled{i}, 'Parent', ax2 );
            set( hMoment, ...
                'LineStyle', plotSettings.literatureMomentCurveStyles{1}, ...
                'LineWidth', plotSettings.literatureMomentCurveWidths{1}, ...
                'Color',     momentColors{i} );
            %legend( hMoment, plotSettings.literatureMomentCurveLabel );
        end
    end
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
    if strcmpi( plotSettings.timeOrPercent, 'Percent' )
        if plotSettings.trialInfo.gcLimb == 'R'
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
    legend( leftAxis, plotSettings.curveLabels );
end
if strcmpi( plotSettings.timeOrPercent, 'Percent' )
    if plotSettings.trialInfo.gcLimb == 'R'
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
