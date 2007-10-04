function evaluateAndCompareSimulations()

%
% Set input file names.
%

% These are the directories containing simulations to compare.
dir1 = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_simbody_cfsqp';
dir2 = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_OLD';

% Set names of files to compare.
cmcKinematicsQMotFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_Kinematics_q.mot' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_Kinematics_q.mot' ) };
cmcKinematicsQStoFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_Kinematics_q.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_Kinematics_q.sto' ) };
cmcKinematicsUStoFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_Kinematics_u.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_Kinematics_u.sto' ) };
cmcKinematicsDudtStoFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_Kinematics_dudt.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_Kinematics_dudt.sto' ) };
cmcMomentsFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_cmc_InverseDynamics_force.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_cmc_InverseDynamics_force.sto' ) };
cmcStatesFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_states.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_states.sto' ) };
scaledModelFiles = { ...
    fullfile( dir1, 'delaware2.osim' ) ...
    fullfile( dir2, 'delaware2.osim' ) };
ikMotFiles = { ...
    fullfile( dir1, 'de2_ss_walk1_ik.mot' ) ...
    fullfile( dir2, 'de2_ss_walk1_ik.mot' ) };
ikMomentsFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_ik_InverseDynamics_force.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_ik_InverseDynamics_force.sto' ) };
rraKinematicsQMotFiles = { ...
    fullfile( dir1, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_q.mot' ) ...
    fullfile( dir2, 'Results/de2_ss_walk1_RRA2_Kinematics_q.mot' ) };
rraKinematicsQStoFiles = { ...
    fullfile( dir1, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_q.sto' ) ...
    fullfile( dir2, 'Results/de2_ss_walk1_RRA2_Kinematics_q.sto' ) };
rraKinematicsUStoFiles = { ...
    fullfile( dir1, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_u.sto' ) ...
    fullfile( dir2, 'Results/de2_ss_walk1_RRA2_Kinematics_u.sto' ) };
rraKinematicsDudtStoFiles = { ...
    fullfile( dir1, 'ResultsRRA/de2_ss_walk1_RRA_Kinematics_dudt.sto' ) ...
    fullfile( dir2, 'Results/de2_ss_walk1_RRA2_Kinematics_dudt.sto' ) };
rraActuationForcesFiles = { ...
    fullfile( dir1, 'ResultsRRA/de2_ss_walk1_RRA_Actuation_force.sto' ) ...
    fullfile( dir2, 'Results/de2_ss_walk1_RRA2_Actuation_force.sto' ) };
rraMomentsFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_rra_InverseDynamics_force.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_rra2_InverseDynamics_force.sto' ) };
cmcControlsFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_controls.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_controls.sto' ) };
cmcActuationForcesFiles = { ...
    fullfile( dir1, 'ResultsCMC/de2_ss_walk1_Actuation_force.sto' ) ...
    fullfile( dir2, 'ResultsCMC/de2_ss_walk1_Actuation_force.sto' ) };
muscleMomentsFiles = { ...
    fullfile( dir1, 'ResultsMuscleAnalysis/de2_ss_walk1_NET_Muscle_Moments.sto' ) ...
    fullfile( dir2, 'ResultsAnalyze/de2_ss_walk1_NET_Muscle_Moments.sto' ) };

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
plotSettings.timeOrPercent = 'Time';
plotSettings.shiftPercentAxisBy = 0;
plotSettings.computeTimeLimitsAndTicksAutomatically = true;
plotSettings.computeVerticalAxisLimitsAndTicksAutomatically = true;
plotSettings.xAxisLabel = 'Time (s)';

% Set the plot title properties.
plotSettings.titleFontName = 'Times New Roman';
plotSettings.titleFontSize = 10;
plotSettings.titleVerticalAlignment = 'Middle';

% Set which quantities to actually plot.
plotGeneralizedCoordinates = false;
plotJointMoments = false;
plotMuscleActivations = true;

%
% Plot generalized coordinates.
%

% Keep a counter for the current figure number.
figNum = 1;

if plotGeneralizedCoordinates

    % Set the column labels for all generalized coordinates.
    gencoordColumnLabels = { 'pelvis_tx' 'pelvis_ty' 'pelvis_tz' ...
        'pelvis_list' 'pelvis_rotation' 'pelvis_tilt' 'hip_flexion_r' ...
        'hip_adduction_r' 'hip_rotation_r' 'hip_flexion_l' ...
        'hip_adduction_l' 'hip_rotation_l' 'knee_angle_r' 'knee_angle_l' ...
        'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' ...
        'subtalar_angle_l' 'mtp_angle_r' 'mtp_angle_l' 'lumbar_extension' ...
        'lumbar_bending' 'lumbar_rotation' };

    % Set the curve properties.
    plotSettings.curveStyles = { '-' '-.' };
    plotSettings.curveWidths = {  1   3  };
    plotSettings.curveColors = { 'r' 'r' };
    plotSettings.curveLabels = { 'New' 'Old' };
    plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 };

    % Set plot input files and columns and plot the desired data columns.
    plotSettings.curveSourceFiles = cmcKinematicsQMotFiles;
    for i = figNum : figNum + length( gencoordColumnLabels ) - 1
        label = gencoordColumnLabels{i};
        plotSettings.figureTitle = label;
        plotSettings.figureNumber = i;
        plotSettings.curveSourceColumnLabels = { label, label };
        plot_dataFromMotOrStoFiles( plotSettings );
    end
    figNum = figNum + length( gencoordColumnLabels );
    
end

%
% Plot joint moments.
%

if plotJointMoments
    
    % Set the column labels for all joint moments.
    jointMomentColumnLabels = gencoordColumnLabels;

    % Set plot input files and columns and plot the desired data columns.
    rraActuationForceLabels = { 'FX' 'FY' 'FZ' 'MX' 'MY' 'MZ' ...
        'hip_flexion_r' ...
        'hip_adduction_r' 'hip_rotation_r' 'hip_flexion_l' ...
        'hip_adduction_l' 'hip_rotation_l' 'knee_angle_r' 'knee_angle_l' ...
        'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' ...
        'subtalar_angle_l' 'mtp_angle_r' 'mtp_angle_l' 'lumbar_extension' ...
        'lumbar_bending' 'lumbar_rotation' };
    missingLabels = { 'subtalar_angle_r' 'mtp_angle_r' 'subtalar_angle_l' ...
        'mtp_angle_l' };
    for i = figNum : figNum + length( jointMomentColumnLabels ) - 1
        j = i - figNum + 1; % j = 1 : length( jointMomentColumnLabels )
        label = jointMomentColumnLabels{j};
        aflabel = rraActuationForceLabels{j};
        plotSettings.figureTitle = [ label ' moments' ];
        plotSettings.figureNumber = i;
        if any( find( strcmpi( label, missingLabels ) ) )
            % Set the curve properties.
            plotSettings.curveStyles = { '-' '-.' };
            plotSettings.curveWidths = {  1   3   };
            plotSettings.curveColors = { 'r' 'r'  };
            plotSettings.curveLabels = { 'New' 'Old' };
            plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 };
            plotSettings.curveSourceFiles = muscleMomentsFiles;
            plotSettings.curveSourceColumnLabels = { label label };
        else
            % Set the curve properties.
            plotSettings.curveStyles = { '-' '-.' '-' '-.' '-' '-.' '-' '-.' '-' '-.' };
            plotSettings.curveWidths = {  1   3    1   3    1   3    1   3    1   3   };
            plotSettings.curveColors = { 'r' 'r'  'r' 'r'  'r' 'r'  'r' 'r'  'r' 'r'  };
            plotSettings.curveLabels = { 'New' 'Old' 'New' 'Old' 'New' 'Old' 'New' 'Old' 'New' 'Old' };
            plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 1 1 1 1 1 1 1 1 };
            plotSettings.curveSourceColumnLabels = { label label label ...
                label label label label label aflabel aflabel };
            plotSettings.curveSourceFiles = [ ikMomentsFiles rraMomentsFiles ...
                cmcMomentsFiles muscleMomentsFiles rraActuationForcesFiles ];
        end
        plot_dataFromMotOrStoFiles( plotSettings );
    end
    figNum = figNum + length( jointMomentColumnLabels );
    
end

%
% Plot muscle activations.
%

muscleNames = { ...
    'glut_max1_l' 'glut_max1_r' 'glut_med1_l' 'glut_med1_r' ...
    'glut_min1_l' 'glut_min1_r' ...
    'iliacus_l' 'iliacus_r' 'sar_l' 'sar_r' ...
    'tfl_l' 'tfl_r' 'vas_med_l' ...
    'vas_med_r' 'rect_fem_l' 'rect_fem_r' ...
    'semimem_l' 'semimem_r' 'bifemlh_l' ...
    'bifemlh_r' 'bifemsh_l' 'bifemsh_r' 'add_brev_l' ...
    'add_brev_r' 'add_mag1_l' 'add_mag1_r' ...
    'med_gas_l' 'med_gas_r' 'soleus_l' 'soleus_r' ...
    'tib_ant_l' 'tib_ant_r' };

if plotMuscleActivations
    % Set the column labels for muscle activations to be plotted.
    activationColumnLabels = strcat( muscleNames, '.activation' );

    % Set the curve properties.
    plotSettings.curveStyles = { '-' '-.' };
    plotSettings.curveWidths = {  1   3  };
    plotSettings.curveColors = { 'r' 'r' };
    plotSettings.curveLabels = { 'New' 'Old' };
    plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 };
    plotSettings.curveSourceFiles = cmcStatesFiles;
    plotSettings.figureNumbers = ...
        figNum : figNum + length( activationColumnLabels ) - 1;
    plotSettings.figureTitles = activationColumnLabels;
    plotSettings.curveSourceColumnLabels = [ ...
        activationColumnLabels; ...
        activationColumnLabels ];

    % Plot the data!
    plot_multipleFiguresFromMotOrStoFiles( plotSettings );
    
    % Update figure number counter.
    figNum = figNum + length( activationColumnLabels );
end
