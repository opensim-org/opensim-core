function evaluateIkRraCmcGeneralizedCoordinates()

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

% Set which quantities to actually plot.
plotGeneralizedCoordinates = true;
plotIkMoments = false;
plotRraMoments = false;
plotCmcMoments = false;
plotMuscleMoments = false;
plotRraActuationForces = false;
plotCmcActuationForces = false;
plotJointMoments = plotIkMoments || plotRraMoments || plotCmcMoments || ...
    plotMuscleMoments || plotRraActuationForces || plotCmcActuationForces;
plotSettings.plotLiteratureMoments = false;
if ~plotJointMoments
    plotSettings.plotLiteratureMoments = false;
end

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

if plotGeneralizedCoordinates

    % Set curve properties.
    plotSettings.curveStyles = { '-' '--' '-' };
    plotSettings.curveWidths = {  6   3   2  };
    plotSettings.curveColors = { [ 0 1 0.5 ] 'r' [ 0 0.5 0.25 ] };
    plotSettings.curveLabels = { 'IK' 'RRA' 'CMC' };
    plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 1 };
    
    % Set limb to use to determine percent of gait cycle.
    plotSettings.trialInfo.gcLimb = 'L';

    % Set plot input files and columns and plot the desired data columns.
    plotSettings.curveSourceFiles = [ ikMotFiles rraKinematicsQMotFiles ...
        cmcKinematicsQMotFiles ];
    for i = figNum : figNum + length( gencoordColumnLabels ) - 1
        label = gencoordColumnLabels{i};
        plotSettings.figureTitle             = label;
        plotSettings.figureNumber            = i;
        plotSettings.curveSourceColumnLabels = { label label label };
        plot_dataFromMotOrStoFiles( plotSettings );
    end
    
    % Set size for master figures.
    scnsize = get(0, 'ScreenSize');
    scnwidth = scnsize(3);
    scnheight = scnsize(4);
    figPos = [0.25*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.85*scnheight];

    % Combine figures into master figures.
    masterFigure1 = figNum + length( gencoordColumnLabels );
    figure( masterFigure1 );
    set( masterFigure1, 'Position', figPos );
    for i = 1 : 12
        temporaryNewAxes = subplot( 4, 3, i );
        oldFigureNumber = figNum + i - 1;
        originalAxes = get( oldFigureNumber, 'CurrentAxes' );
        actualNewAxes = copyobj( originalAxes, masterFigure1 );
        newAxesPosition = get( temporaryNewAxes, 'Position' );
        set( actualNewAxes, 'Position', newAxesPosition );
        xlabel( actualNewAxes, '' );
        delete( temporaryNewAxes );
    end
    masterFigure2 = masterFigure1 + 1;
    figure( masterFigure2 );
    set( masterFigure2, 'Position', figPos );
    for i = 13 : 23
        temporaryNewAxes = subplot( 4, 3, i - 12 );
        oldFigureNumber = figNum + i - 1;
        originalAxes = get( oldFigureNumber, 'CurrentAxes' );
        actualNewAxes = copyobj( originalAxes, masterFigure2 );
        newAxesPosition = get( temporaryNewAxes, 'Position' );
        set( actualNewAxes, 'Position', newAxesPosition );
        xlabel( actualNewAxes, '' );
        delete( temporaryNewAxes );
    end

    % Update figure counter for quantities being plotted next.
    figNum = figNum + length( gencoordColumnLabels ) + 2;
    
end
