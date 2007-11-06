function evaluateActivations()

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

% Keep a counter for the current figure number.
figNum = 1;

%
% Plot muscle activations.
%

% Set which quantities to actually plot.
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
plotMuscleActivations = true;
plotSettings.plotLiteratureActivations = true;

if plotMuscleActivations
    
    % Set curve properties.
    curveStyles = { '-' };
    curveWidths = {  1  };
    curveColors = { 'r' };
    curveLabels = { 'CMC' };
    curveRepeatedSourceColumnNumbers = { 1 };

    muscleNames = { ...
        'glut_max1_l' 'glut_max1_r' 'glut_med1_l' 'glut_med1_r' ...
        'glut_min1_l' 'glut_min1_r' ...
        'iliacus_l' 'iliacus_r' 'sar_l' 'sar_r' 'tfl_l' 'tfl_r' ...
        'vas_med_l' 'vas_med_r' 'rect_fem_l' 'rect_fem_r' ...
        'semimem_l' 'semimem_r' 'bifemlh_l' 'bifemlh_r' ...
        'bifemsh_l' 'bifemsh_r' ...
        'add_long_l' 'add_long_r' 'add_mag1_l' 'add_mag1_r' ...
        'med_gas_l' 'med_gas_r' 'soleus_l' 'soleus_r' ...
        'tib_ant_l' 'tib_ant_r' };

    % Cappellini-Ivanenko EMG data column indices for each muscle.
    ciGmax = 16;
    ciGmed = 17;
    ciIlio = 18;
    ciTfl = 19;
    ciAddLong = 20;
    ciSar = 21;
    ciBflh = 22;
    ciSemiten = 23;
    ciRectFem = 24;
    ciVasMed = 25;
    ciGasMed = 27;
    ciSol = 30;
    ciTa = 32;

    % Set, for each figure, what limb to use to determine percent of gait
    % cycle.  For muscles on the right side, it makes more sense to plot
    % vs. percent of gait cycle for the right limb, whereas all other
    % muscles are plotted vs. percent of gait cycle for the left limb.
    plotSettings.trialInfo.gcLimb = cell( 1, length( muscleNames ) );
    for j = 1 : length( muscleNames )
        label = muscleNames{j};
        if strcmpi( label( end - 1 : end ), '_r' )
            plotSettings.trialInfo.gcLimb{j} = 'R';
        else
            plotSettings.trialInfo.gcLimb{j} = 'L';
        end
    end

    % Set the column labels for muscle activations to be plotted.
    activationColumnLabels = strcat( muscleNames, '.activation' );

    % Set the curve properties.
    plotSettings.curveStyles = curveStyles;
    plotSettings.curveWidths = curveWidths;
    plotSettings.curveColors = curveColors;
    plotSettings.curveLabels = curveLabels;
    plotSettings.curveRepeatedSourceColumnNumbers = ...
        curveRepeatedSourceColumnNumbers;
    plotSettings.curveSourceFiles = cmcStatesFiles;
    plotSettings.figureNumbers = ...
        figNum : figNum + length( activationColumnLabels ) - 1;
    plotSettings.figureTitles = activationColumnLabels;
    plotSettings.curveSourceColumnLabels = [ ...
        activationColumnLabels; ...
        activationColumnLabels ];
    plotSettings.literatureActivationCurveStyle = '-';
    plotSettings.literatureActivationCurveWidth = 4;
    plotSettings.literatureActivationCurveColor = [ 0.8 0.8 0.8 ];
    plotSettings.literatureActivationCurveLabel = 'Lit';
    % Indices for columns of EMG data to read for each figure.  If a
    % particular figure should have no literature data plotted, then the
    % column index is set to 0.
    plotSettings.emgColumnIndices = [ ciGmax ciGmax ciGmed ciGmed 0 0 ...
        ciIlio ciIlio ciSar ciSar ciTfl ciTfl ciVasMed ciVasMed ...
        ciRectFem ciRectFem ciSemiten ciSemiten ciBflh ciBflh 0 0 ...
        ciAddLong ciAddLong 0 0 ciGasMed ciGasMed ciSol ciSol ciTa ciTa ];

    % Plot the data!
    plot_multipleFiguresFromMotOrStoFiles( plotSettings );
    
    % Set size for master figures.
    scnsize = get(0, 'ScreenSize');
    scnwidth = scnsize(3);
    scnheight = scnsize(4);
    figPos = [0.25*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.85*scnheight];

    % Combine figures into master figures.
    masterFigure1 = figNum + length( activationColumnLabels );
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
    for i = 13 : 22
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
    masterFigure3 = masterFigure2 + 1;
    figure( masterFigure3 );
    set( masterFigure3, ...
        'Position', figPos, ...
        'Color', plotSettings.figureBgColor );
    set( masterFigure3, 'Position', figPos );
    for i = 23 : 32
        temporaryNewAxes = subplot( 4, 3, i - 22 );
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
        copy_figureIntoFigure( oldFigureNumber, masterFigure3, ...
            firstAxes, temporaryNewAxes );
        delete( temporaryNewAxes );
    end
    
    % Update figure counter for quantities being plotted next.
    figNum = figNum + length( activationColumnLabels ) + 3;
    
end
