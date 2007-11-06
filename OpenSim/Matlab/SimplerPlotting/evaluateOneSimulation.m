function evaluateOneSimulation()

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
%plotSettings.timeOrPercent = 'Time';
%plotSettings.xAxisLabel = 'Time (s)';
plotSettings.timeOrPercent = 'Percent';
plotSettings.xAxisLabel = 'Percent of gait cycle';
plotSettings.shiftPercentAxisBy = 0;
plotSettings.computeTimeLimitsAndTicksAutomatically = false;
plotSettings.xAxisRange = [ 0 150 ];
plotSettings.xAxisTicks = 0 : 50 : 150;
plotSettings.xAxisRangeLeft = [ 0 150 ];
plotSettings.xAxisTicksLeft = 0 : 50 : 150;
plotSettings.xAxisRangeRight = [ 40 220 ];
plotSettings.xAxisTicksRight = 40 : 40 : 220;
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

plotMuscleActivations = false;

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
    'pelvis_list' 'pelvis_rotation' 'pelvis_tilt' 'hip_flexion_r' ...
    'hip_adduction_r' 'hip_rotation_r' 'hip_flexion_l' ...
    'hip_adduction_l' 'hip_rotation_l' 'knee_angle_r' 'knee_angle_l' ...
    'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' ...
    'subtalar_angle_l' 'mtp_angle_r' 'mtp_angle_l' 'lumbar_extension' ...
    'lumbar_bending' 'lumbar_rotation' };

if plotGeneralizedCoordinates

    % Set curve properties.
    plotSettings.curveStyles = { '-' '.' '-' };
    plotSettings.curveWidths = {  3   1   1  };
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
    figNum = figNum + length( gencoordColumnLabels );
    
end

%
% Plot joint moments.
%

if plotJointMoments

    % Set curve properties.
    curveStyles = { '-' };
    curveWidths = {  1  };
    curveColors = { [ 0 0.5 0.25 ] };
    curveLabels = { 'CMC->ID moment' };
    curveRepeatedSourceColumnNumbers = { 1 };
    
    % Set the column labels for all joint moments.
    jointMomentColumnLabels = gencoordColumnLabels;

    % Set plot input files and column labels.
    if plotRraActuationForces
        rraActuationForceLabels = { 'FX' 'FY' 'FZ' 'MX' 'MY' 'MZ' ...
            'hip_flexion_r' ...
            'hip_adduction_r' 'hip_rotation_r' 'hip_flexion_l' ...
            'hip_adduction_l' 'hip_rotation_l' 'knee_angle_r' 'knee_angle_l' ...
            'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' ...
            'subtalar_angle_l' 'mtp_angle_r' 'mtp_angle_l' 'lumbar_extension' ...
            'lumbar_bending' 'lumbar_rotation' };
    end
    if any( simulationsLackSubtalarAndMtpAngles )
        missingLabels = { 'subtalar_angle_r' 'mtp_angle_r' 'subtalar_angle_l' ...
            'mtp_angle_l' };
    end
    if plotCmcActuationForces
        cmcActuationForceLabels = { 'FX' 'FY' 'FZ' 'MX' 'MY' 'MZ' ...
            'hip_flexion_r_reserve' ...
            'hip_adduction_r_reserve' 'hip_rotation_r_reserve' ...
            'hip_flexion_l_reserve' ...
            'hip_adduction_l_reserve' 'hip_rotation_l_reserve' ...
            'knee_angle_r_reserve' 'knee_angle_l_reserve' ...
            'ankle_angle_r_reserve' 'subtalar_angle_r_reserve' ...
            'ankle_angle_l_reserve' ...
            'subtalar_angle_l_reserve' 'mtp_angle_r_reserve' ...
            'mtp_angle_l_reserve' 'lumbar_extension_reserve' ...
            'lumbar_bending_reserve' 'lumbar_rotation_reserve' };
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
            plotSettings.curveStyles = curveStyles;
            plotSettings.curveWidths = curveWidths;
            plotSettings.curveColors = curveColors;
            plotSettings.curveLabels = curveLabels;
            plotSettings.curveRepeatedSourceColumnNumbers = ...
                curveRepeatedSourceColumnNumbers;
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
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ikMomentsFiles ];
            end
            if plotRraMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles rraMomentsFiles ];
            end
            if plotCmcMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles cmcMomentsFiles ];
            end
            if plotMuscleMoments
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    curveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles muscleMomentsFiles ];
            end
            if plotRraActuationForces
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    rraCurveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ...
                    rraActuationForcesFiles ];
            end
            if plotCmcActuationForces
                plotSettings.curveStyles = ...
                    [ plotSettings.curveStyles curveStyles ];
                plotSettings.curveWidths = ...
                    [ plotSettings.curveWidths curveWidths ];
                plotSettings.curveColors = ...
                    [ plotSettings.curveColors curveColors ];
                plotSettings.curveLabels = ...
                    [ plotSettings.curveLabels curveLabels ];
                plotSettings.curveRepeatedSourceColumnNumbers = ...
                    [ plotSettings.curveRepeatedSourceColumnNumbers ...
                    curveRepeatedSourceColumnNumbers ];
                plotSettings.curveSourceColumnLabels = ...
                    [ plotSettings.curveSourceColumnLabels ...
                    cmcCurveSourceColumnLabels ];
                plotSettings.curveSourceFiles = ...
                    [ plotSettings.curveSourceFiles ...
                    cmcActuationForcesFiles ];
            end
        end
        
        % Set plot properties for literature data.
        plotSettings.plotLiteratureMoments = true;
        plotSettings.literatureMomentCurveStyles = { '-' '-' '-' '-' '-' };
        plotSettings.literatureMomentCurveWidths = {  1   1   1   1   1  };
        plotSettings.literatureMomentCurveColors = { 'g' 'b' 'c' 'y' 'm' };
        plotSettings.literatureMomentCurveLabel = 'Lit';
        plotSettings.literatureMomentToPlot = label;
        
        % Plot the data.
        plot_dataFromMotOrStoFiles( plotSettings );
    end
    figNum = figNum + length( jointMomentColumnLabels );
    
end

%
% Plot muscle activations.
%

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
        'semimem_l' 'semimem_r' 'bifemlh_l' ...
        'bifemlh_r' 'bifemsh_l' 'bifemsh_r' 'add_long_l' ...
        'add_long_r' 'add_mag1_l' 'add_mag1_r' ...
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
    plotSettings.plotLiteratureActivations = true;
    plotSettings.literatureActivationCurveStyle = '-';
    plotSettings.literatureActivationCurveWidth = 1;
    plotSettings.literatureActivationCurveColor = 'k';
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
    
    % Update figure number counter.
    figNum = figNum + length( activationColumnLabels );
end
