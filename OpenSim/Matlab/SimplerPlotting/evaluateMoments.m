function evaluateMoments()

%
% Set input file names.
%

% These are the directories containing simulations to compare.
dir = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_simbody_cfsqp_mtpbig';

% Set names of files to compare.
cmcKinematicsQMotFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_Kinematics_q.mot' ) };
cmcKinematicsQStoFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_Kinematics_q.sto' ) };
cmcKinematicsUStoFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_Kinematics_u.sto' ) };
cmcKinematicsDudtStoFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_Kinematics_dudt.sto' ) };
cmcMomentsFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_cmc_InverseDynamics_force.sto' ) };
cmcStatesFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_states.sto' ) };
scaledModelFiles = { ...
    fullfile( dir, 'delaware2.osim' ) };
ikMotFiles = { ...
    fullfile( dir, 'de2_ss_walk1_ik.mot' ) };
ikMomentsFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_ik_InverseDynamics_force.sto' ) };
rraKinematicsQMotFiles = { ...
    fullfile( dir, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_q.mot' ) };
rraKinematicsQStoFiles = { ...
    fullfile( dir, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_q.sto' ) };
rraKinematicsUStoFiles = { ...
    fullfile( dir, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_u.sto' ) };
rraKinematicsDudtStoFiles = { ...
    fullfile( dir, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_dudt.sto' ) };
rraActuationForcesFiles = { ...
    fullfile( dir, 'ResultsRRA/de2_ss_walk1_RRA_Actuation_force.sto' ) };
rraMomentsFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_rra_InverseDynamics_force.sto' ) };
cmcControlsFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_controls.sto' ) };
cmcActuationForcesFiles = { ...
    fullfile( dir, 'ResultsCMC/de2_ss_walk1_Actuation_force.sto' ) };
muscleMomentsFiles = { ...
    fullfile( dir, 'ResultsMuscleAnalysis/de2_ss_walk1_NET_Muscle_Moments.sto' ) };

%
% Set generic plot properties.
%

% Set the figure (window) properties.
plotSettings.figurePosition = [ 2 2 3.25 2.25 ];
plotSettings.figureUnits = 'Inches';
plotSettings.figureBgColor = 'w';
plotSettings.figureFgColor = 'k';

% Set the plot properties (e.g., axes, labels).
plotSettings.yAxisRange = [ -100 100 ];
plotSettings.yAxisTicks = -100 : 50 : 100;
plotSettings.yAxisLabel = 'Angle (degrees)';
plotSettings.zeroLineOn = true;
plotSettings.zeroLineWidth = 0.25;
plotSettings.zeroLineColor = [ 0.3 0.3 0.3 ];
plotSettings.axisFontName = 'Times New Roman';
plotSettings.axisFontSize = 8;
plotSettings.axisTickDirection = 'Out';

% Set the time axis properties.
% If plotSettings.computeTimeLimitsAndTicksAutomatically is false, then
% plotSettings.xAxisRange and plotSettings.xAxisTicks should be set
% manually here by the user.
plotSettings.timeOrPercent = 'Percent';
plotSettings.xAxisLabel = 'Percent of gait cycle';
plotSettings.xAxisRange = [ 0 150 ];
plotSettings.xAxisTicks = 0 : 50 : 150;
plotSettings.xAxisRangeLeft = [ 0 150 ];
plotSettings.xAxisTicksLeft = 0 : 50 : 150;
plotSettings.xAxisRangeRight = [ 40 220 ];
plotSettings.xAxisTicksRight = 40 : 40 : 220;
%plotSettings.timeOrPercent = 'Time';
%plotSettings.xAxisLabel = 'Time (s)';
%plotSettings.xAxisRange = [ 0.905 2.706 ];
%plotSettings.xAxisTicks = 1 : 0.5 : 2.7;
%plotSettings.xAxisRangeLeft = [ 0.905 2.706 ];
%plotSettings.xAxisTicksLeft = 1 : 0.5 : 2.7;
%plotSettings.xAxisRangeRight = [ 0.905 2.706 ];
%plotSettings.xAxisTicksRight = 1 : 0.5 : 2.7;
plotSettings.shiftPercentAxisBy = 0;
plotSettings.computeTimeLimitsAndTicksAutomatically = false;
plotSettings.computeVerticalAxisLimitsAndTicksAutomatically = true;

% Set the trial info for the plot.
currentDirectory = cd;
cd( dir );
[ sInfo, trialInfo ] = ref_trialInfoDelaware2( 'de2', 'ss_walk1' );
cd( currentDirectory );
% Note, trialInfo.gcLimb is set later for each call to a plot function.
trialInfo.mass = 65.9;
trialInfo.FP = trialInfo.ss_walk1.FP;
trialInfo.limb = trialInfo.ss_walk1.limb;
trialInfo.ictoMatrix = trialInfo.ss_walk1.ictoMatrix;
trialInfo.analogRate = 600;
trialInfo.tZeroAtFirstIC = 0;
% Get analog frames corresponding to IC and TO events from tInfo,
% and store in structure array.
for fpHitNum = 1:length(trialInfo.FP)
    ictoEvents(fpHitNum).ic  = trialInfo.ictoMatrix(fpHitNum, 1);
    ictoEvents(fpHitNum).oto = trialInfo.ictoMatrix(fpHitNum, 2);
    ictoEvents(fpHitNum).oic = trialInfo.ictoMatrix(fpHitNum, 3);
    ictoEvents(fpHitNum).to  = trialInfo.ictoMatrix(fpHitNum, 4);
    ictoEvents(fpHitNum).icNext = trialInfo.ictoMatrix(fpHitNum, 5);
end
plotSettings.ictoEvents = ictoEvents;
plotSettings.trialInfo = trialInfo;

% Set the plot title properties.
plotSettings.titleFontName = 'Times New Roman';
plotSettings.titleFontSize = 10;
plotSettings.titleVerticalAlignment = 'Middle';

% Some simulations lack subtalar and mtp angles in IK, RRA, CMC, and all
% inverse dynamics outputs.  This is the user's opportunity to say so.  The
% ith entry in this array is a Boolean variable indicating whether the ith
% simulation (directory) above contains a simulation lacking subtalar and
% mtp angles.
simulationsLackSubtalarAndMtpAngles = false;

%
% Plot generalized coordinates.
%

% Keep a counter for the current figure number.
figNum = 1;

% Set the column labels for all generalized coordinates.
gencoordColumnLabels = { 'pelvis_tx' 'pelvis_ty' 'pelvis_tz' ...
    'pelvis_list' 'pelvis_rotation' 'pelvis_tilt' 'hip_flexion_l' ...
    'hip_flexion_r' 'hip_adduction_l' 'hip_adduction_r' ...
    'hip_rotation_l' 'hip_rotation_r' ...
    'knee_angle_l' 'knee_angle_r' 'ankle_angle_l' 'ankle_angle_r' ...
    'subtalar_angle_l' 'subtalar_angle_r' 'mtp_angle_l' 'mtp_angle_r' ...
    'lumbar_extension' 'lumbar_bending' 'lumbar_rotation' };

%
% Plot joint moments.
%

% Set which quantities to actually plot.
plotIkMoments = true;
plotRraMoments = false;
plotCmcMoments = false;
plotMuscleMoments = false;
plotRraActuationForces = true;
plotCmcActuationForces = true;
plotJointMoments = plotIkMoments || plotRraMoments || plotCmcMoments || ...
    plotMuscleMoments || plotRraActuationForces || plotCmcActuationForces;
plotSettings.plotLiteratureMoments = false;
if ~plotJointMoments
    plotSettings.plotLiteratureMoments = false;
end

if plotJointMoments

    % Set curve properties.
    curveStyles = { '-' '-' '-' '--' '--' '-' };
    curveWidths = {  5   5   2   2    2    2  };
    curveColors = { [ 0 1 0.5 ] [ 0 1 0.5 ] [ 0 0.5 0.25 ] 'r' 'r' ...
        [ 0 0.5 0.25 ] };
    curveLabels = { 'IK->ID moment' 'RRA->ID moment' 'CMC->ID moment' ...
        'CMC muscle moment' 'RRA actuation force' ...
        'CMC reserve actuator force' };
    curveRepeatedSourceColumnNumbers = { 1 1 1 1 1 1 };
    
    % Set the column labels for all joint moments.
    jointMomentColumnLabels = gencoordColumnLabels;

    % Set plot input files and column labels.
    if plotRraActuationForces
        rraActuationForceLabels = jointMomentColumnLabels;
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_tx', 'FX', @strcmp );
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_ty', 'FY', @strcmp );
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_tz', 'FZ', @strcmp );
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_list', 'MX', @strcmp );
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_rotation', 'MY', @strcmp );
        rraActuationForceLabels = ...
            cellFindAndReplace( rraActuationForceLabels, ...
            'pelvis_tilt', 'MZ', @strcmp );
    end
    if any( simulationsLackSubtalarAndMtpAngles )
        missingLabels = { 'subtalar_angle_r' 'mtp_angle_r' 'subtalar_angle_l' ...
            'mtp_angle_l' };
    end
    if plotCmcActuationForces
        cmcActuationForceLabels = jointMomentColumnLabels;
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_tx', 'FX', @strcmp );
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_ty', 'FY', @strcmp );
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_tz', 'FZ', @strcmp );
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_list', 'MX', @strcmp );
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_rotation', 'MY', @strcmp );
        cmcActuationForceLabels = ...
            cellFindAndReplace( cmcActuationForceLabels, ...
            'pelvis_tilt', 'MZ', @strcmp );
        indicesToAppend = 1 : length( cmcActuationForceLabels );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'FX' ) ) );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'FY' ) ) );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'FZ' ) ) );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'MX' ) ) );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'MY' ) ) );
        indicesToAppend = setdiff( indicesToAppend, ...
            find( strcmp( cmcActuationForceLabels, 'MZ' ) ) );
        for i = indicesToAppend
            cmcActuationForceLabels{i} = cat( 2, ...
                cmcActuationForceLabels{i}, '_reserve' );
        end
    end
    
    % Plot the desired data columns.
    for i = figNum : figNum + length( jointMomentColumnLabels ) - 1
        
        j = i - figNum + 1; % j = 1 : length( jointMomentColumnLabels )
        
        % Get column labels, including special ones for RRA and CMC if
        % needed.
        label = jointMomentColumnLabels{j};
        if plotRraActuationForces
            aflabel = rraActuationForceLabels{j};
        end
        if plotCmcActuationForces
            cmcAflabel = cmcActuationForceLabels{j};
        end
        
        % These variables should be changed depending on what moments are
        % being plotted.
        curveSourceColumnLabels = { label };
        if plotRraActuationForces
            rraCurveSourceColumnLabels = { aflabel };
        end
        if plotCmcActuationForces
            cmcCurveSourceColumnLabels = { cmcAflabel };
        end
        
        % Set other plot properties.
        plotSettings.figureTitle = [ label ' moments' ];
        plotSettings.figureNumber = i;
        if strcmpi( label( end - 1 : end ), '_r' )
            plotSettings.trialInfo.gcLimb = 'R';
        else
            plotSettings.trialInfo.gcLimb = 'L';
        end
        
        % Set curve properties depending on what moments are being plotted.
        if any( simulationsLackSubtalarAndMtpAngles ) && ...
                any( find( strcmpi( label, missingLabels ) ) ) && ...
                plotMuscleMoments
            % Set the curve properties.
            plotSettings.curveStyles = curveStyles{4};
            plotSettings.curveWidths = curveWidths{4};
            plotSettings.curveColors = curveColors{4};
            plotSettings.curveLabels = curveLabels{4};
            plotSettings.curveRepeatedSourceColumnNumbers = ...
                curveRepeatedSourceColumnNumbers{4};
            plotSettings.curveSourceFiles = muscleMomentsFiles;
            plotSettings.curveSourceColumnLabels = curveSourceColumnLabels;
        else
            % Set the curve properties.
            plotSettings.curveStyles = {};
            plotSettings.curveWidths = {};
            plotSettings.curveColors = {};
            plotSettings.curveLabels = {};
            plotSettings.curveRepeatedSourceColumnNumbers = {};
            plotSettings.curveSourceColumnLabels = {};
            plotSettings.curveSourceFiles = {};
            if plotIkMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{1} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{1} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{1} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{1} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{1} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ikMomentsFiles ];
            end
            if plotRraMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{2} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{2} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{2} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{2} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{2} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles rraMomentsFiles ];
            end
            if plotCmcMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{3} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{3} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{3} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{3} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{3} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles cmcMomentsFiles ];
            end
            if plotMuscleMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{4} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{4} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{4} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{4} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{4} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles muscleMomentsFiles ];
            end
            if plotRraActuationForces
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{5} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{5} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{5} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{5} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{5} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    rraCurveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ...
                    rraActuationForcesFiles ];
            end
            if plotCmcActuationForces
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles{6} ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths{6} ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors{6} ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels{6} ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers{6} ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    cmcCurveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ...
                    cmcActuationForcesFiles ];
            end
        end
        
        % Set plot properties for literature data.
        plotSettings.literatureMomentCurveStyles = { '-' '-' '-' '-' '-' };
        plotSettings.literatureMomentCurveWidths = {  3   3   3   3   3  };
        plotSettings.literatureMomentCurveColors = { [ 0.9 0.9 0.9 ] ...
            [ 0.8 0.8 0.8 ] [ 0.9 0.9 0.9 ] [ 0.75 0.75 0.75 ] ...
            [ 0.9 0.9 0.9 ] };
        plotSettings.literatureMomentCurveLabel = 'Lit';
        plotSettings.literatureMomentToPlot = label;
        
        % Plot the data.
        plot_dataFromMotOrStoFiles( plotSettings );
    end
    
    % Set size for master figures.
    scnsize = get(0, 'ScreenSize');
    scnwidth = scnsize(3);
    scnheight = scnsize(4);
    figPos = [0.25*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.85*scnheight];

    % Combine figures into master figures.
    masterFigure1 = figNum + length( jointMomentColumnLabels );
    figure( masterFigure1 );
    set( masterFigure1, ...
        'Position', figPos, ...
        'Color', plotSettings.figureBgColor );
    for i = 1 : 12
        temporaryNewAxes = subplot( 4, 3, i );
        oldFigureNumber = figNum + i - 1;
        originalChildren = get( oldFigureNumber, 'Children' );
        % Find an axes object among the children of figure oldFigureNumber.
        for j = 1 : length( originalChildren )
            if strcmp( get( originalChildren(j), 'Type' ), 'axes' )
                firstAxes = originalChildren(j);
                break;
            end
        end
        % Remove x-axis labels from all axes objects.
        for j = 1 : length( originalChildren )
            if strcmp( get( originalChildren(j), 'Type' ), 'axes' )
                xlabel( originalChildren(j), '' );
            end
        end
        % We are assuming firstAxes was assigned at some point, i.e.
        % that figure oldFigureNumber has at least one child that is an
        % axes object.
        copy_figureIntoFigure( oldFigureNumber, masterFigure1, ...
            firstAxes, temporaryNewAxes );
        delete( temporaryNewAxes );
    end
    masterFigure2 = masterFigure1 + 1;
    figure( masterFigure2 );
    set( masterFigure2, ...
        'Position', figPos, ...
        'Color', plotSettings.figureBgColor );
    set( masterFigure2, 'Position', figPos );
    for i = 13 : 23
        temporaryNewAxes = subplot( 4, 3, i - 12 );
        oldFigureNumber = figNum + i - 1;
        originalChildren = get( oldFigureNumber, 'Children' );
        % Find an axes object among the children of figure oldFigureNumber.
        for j = 1 : length( originalChildren )
            if strcmp( get( originalChildren(j), 'Type' ), 'axes' )
                firstAxes = originalChildren(j);
                break;
            end
        end
        % Remove x-axis labels from all axes objects.
        for j = 1 : length( originalChildren )
            if strcmp( get( originalChildren(j), 'Type' ), 'axes' )
                xlabel( originalChildren(j), '' );
            end
        end
        % We are assuming firstAxes was assigned at some point, i.e.
        % that figure oldFigureNumber has at least one child that is an
        % axes object.
        copy_figureIntoFigure( oldFigureNumber, masterFigure2, ...
            firstAxes, temporaryNewAxes );
        delete( temporaryNewAxes );
    end

    % Update figure counter for quantities being plotted next.
    figNum = figNum + length( jointMomentColumnLabels ) + 2;
    
end
