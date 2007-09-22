function setSetupValuesForPercentGaitCycle( initialPercent, finalPercent, inputDirectory, outputDirectory, subjectName, trialName, initialTimeCushion, finalTimeCushion, cmcInitialTimeAnticushion, cmcFinalTimeAnticushion )
%
% Set setup files values for simulating an interval of walking given in
% percents of gait cycle.  Read all setup files from inputDirectory, modify
% the initial and final times in the appropriate setup files, and save all
% setup files to outputDirectory.
%
% Author: Chand T. John
%

%
% 0. Do initial setup steps.
%

% Copy gait2392.osim file from gait2392 example directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
sourceSdfastGenericModelFile = fullfile( inputDirectory, 'gait2392.osim' );
destinationSdfastGenericModelFile = fullfile( outputDirectory, 'gait2392.osim' );
outputDirectoryDoesntExist = ( exist( outputDirectory, 'dir' ) ~= 7 );
if outputDirectoryDoesntExist
    mkdir( outputDirectory );
end
copyfile( sourceSdfastGenericModelFile, destinationSdfastGenericModelFile );

% Copy gait2392_simbody.osim file from source directory to
% destination directory, if this file doesn't already exist in the
% destination directory.
sourceSimbodyGenericModelFile = fullfile( inputDirectory, 'gait2392_simbody.osim' );
destinationSimbodyGenericModelFile = fullfile( outputDirectory, 'gait2392_simbody.osim' );
outputDirectoryDoesntExist = ( exist( outputDirectory, 'dir' ) ~= 7 );
if outputDirectoryDoesntExist
    mkdir( outputDirectory );
end
copyfile( sourceSimbodyGenericModelFile, destinationSimbodyGenericModelFile );

% Copy input motion file from source directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
motFile = fullfile( inputDirectory, [trialName '.mot'] );
outputMotFile = fullfile( outputDirectory, [trialName '.mot'] );
copyfile( motFile, outputMotFile );

% Copy input TRC file from source directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
trcFile = fullfile( inputDirectory, [trialName '.trc'] );
outputTrcFile = fullfile( outputDirectory, [trialName '.trc'] );
copyfile( trcFile, outputTrcFile );

% Copy ground reaction data file from source directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
grfFile = fullfile( inputDirectory, [trialName '_grf.mot'] );
outputGrfFile = fullfile( outputDirectory, [trialName '_grf.mot'] );
copyfile( grfFile, outputGrfFile );

% Copy static trial TRC data file from source directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
staticTrcFile = fullfile( inputDirectory, [subjectName '_static.trc'] );
outputStaticTrcFile = fullfile( outputDirectory, [subjectName '_static.trc'] );
copyfile( staticTrcFile, outputStaticTrcFile );

%
% 1. Compute initial and final times for the simulation.
%

ictoInput = compute_ictoMatrixInput( grfFile );
trial.contactRanges = ictoInput.contactRanges;
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix( ictoInput.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP );

% Get analog frames corresponding to IC and TO events; 
% convert icto data matrix to structure.
allocArray = 1:length( trial.FP );
ictoEvents = struct( 'ic', allocArray, 'oto', allocArray, 'oic', allocArray, 'to', allocArray, 'icNext', allocArray );
for fpHitNum = 1:length( trial.FP )     
    ictoEvents( fpHitNum ).ic     = trial.ictoMatrix( fpHitNum, 1 );
    ictoEvents( fpHitNum ).oto    = trial.ictoMatrix( fpHitNum, 2 );
    ictoEvents( fpHitNum ).oic    = trial.ictoMatrix( fpHitNum, 3 );
    ictoEvents( fpHitNum ).to     = trial.ictoMatrix( fpHitNum, 4 );
    ictoEvents( fpHitNum ).icNext = trial.ictoMatrix( fpHitNum, 5 );
end

% The following data is just dummy info that has no effect on the results
% of this code, as far as I can tell.
%trial.mass = 65.9;
%trial.trial = 'ss_walking1';
%trial.speed = 1.3600;
%tInfo.static = {};

cyclePercents = [initialPercent finalPercent];
initialQuotientAfterIntegerDivisionBy100 = floor( initialPercent / 100 );
cyclePercents(1) = initialPercent - initialQuotientAfterIntegerDivisionBy100 * 100;
finalQuotientAfterIntegerDivisionBy100 = floor( finalPercent / 100 );
cyclePercents(2) = finalPercent - finalQuotientAfterIntegerDivisionBy100 * 100;

cycleTimes = convert_cycleToTime( cyclePercents, trial, ictoEvents, 600, ref_dataFormatDelaware );
indexOfInitialTime = 1 + 2 * initialQuotientAfterIntegerDivisionBy100;
initialTime = cycleTimes.L( indexOfInitialTime );
indexOfFinalTime = 2 + 2 * finalQuotientAfterIntegerDivisionBy100;
finalTime = cycleTimes.L( indexOfFinalTime );

% Define cushions for start and end time.  Cushions can be useful for
% dealing with end effects at the beginning and end of a simulation.
initialTime = initialTime - initialTimeCushion;
finalTime = finalTime + finalTimeCushion;
cmcInitialTime = initialTime + cmcInitialTimeAnticushion;
cmcFinalTime = finalTime - cmcFinalTimeAnticushion;

initialTimeStr = num2str( initialTime );
finalTimeStr = num2str( finalTime );
cmcInitialTimeStr = num2str( cmcInitialTime );
cmcFinalTimeStr = num2str( cmcFinalTime );

%
% 2. Read in source setup files.
%

Analyze_SetupFile = fullfile( inputDirectory, [trialName '_Setup_Analyze.xml'] );
CMC_ActuatorsFile = fullfile( inputDirectory, [trialName '_CMC_Actuators.xml'] );
CMC_ControlConstraintsFile = fullfile( inputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
CMC_SetupFile = fullfile( inputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFile = fullfile( inputDirectory, [trialName '_CMC_Tasks.xml'] );
IDIK_SetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_IK.xml'] );
IDRRA_SetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_RRA.xml'] );
IDCMC_SetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_CMC.xml'] );
IK_SetupFile = fullfile( inputDirectory, [trialName '_Setup_IK.xml'] );
IK_TasksFile = fullfile( inputDirectory, [trialName '_IK_Tasks.xml'] );
MuscleAnalysis_SetupFile = fullfile( inputDirectory, [trialName '_Setup_MuscleAnalysis.xml'] );
RRA_ActuatorsFile = fullfile( inputDirectory, [trialName '_RRA_Actuators.xml'] );
RRA_ControlConstraintsFile = fullfile( inputDirectory, [trialName '_RRA_ControlConstraints.xml'] );
RRA_SetupFile = fullfile( inputDirectory, [trialName '_Setup_RRA.xml'] );
RRA_TasksFile = fullfile( inputDirectory, [trialName '_RRA_Tasks.xml'] );
Scale_MarkerSetFile = fullfile( inputDirectory, [subjectName '_Scale_MarkerSet.xml'] );
Scale_MeasurementSetFile = fullfile( inputDirectory, [subjectName '_Scale_MeasurementSet.xml'] );
Scale_ScaleSetFile = fullfile( inputDirectory, [subjectName '_Scale_ScaleSet.xml'] );
Scale_SetupFile = fullfile( inputDirectory, [subjectName '_Setup_Scale.xml'] );
Scale_TasksFile = fullfile( inputDirectory, [subjectName '_Scale_Tasks.xml'] );
Forward_SetupFile = fullfile( inputDirectory, [trialName '_Setup_Forward.xml'] );
Perturb_SetupFile = fullfile( inputDirectory, [trialName '_Setup_Perturb.xml'] );

Analyze_SetupDomNode = xmlread( Analyze_SetupFile );
CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFile );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFile );
CMC_SetupDomNode = xmlread( CMC_SetupFile );
CMC_TasksDomNode = xmlread( CMC_TasksFile );
IDIK_SetupDomNode = xmlread( IDIK_SetupFile );
IDRRA_SetupDomNode = xmlread( IDRRA_SetupFile );
IDCMC_SetupDomNode = xmlread( IDCMC_SetupFile );
IK_SetupDomNode = xmlread( IK_SetupFile );
IK_TasksDomNode = xmlread( IK_TasksFile );
MuscleAnalysis_SetupDomNode = xmlread( MuscleAnalysis_SetupFile );
RRA_ActuatorsDomNode = xmlread( RRA_ActuatorsFile );
RRA_ControlConstraintsDomNode = xmlread( RRA_ControlConstraintsFile );
RRA_SetupDomNode = xmlread( RRA_SetupFile );
RRA_TasksDomNode = xmlread( RRA_TasksFile );
Scale_MarkerSetDomNode = xmlread( Scale_MarkerSetFile );
Scale_MeasurementSetDomNode = xmlread( Scale_MeasurementSetFile );
Scale_ScaleSetDomNode = xmlread( Scale_ScaleSetFile );
Scale_SetupDomNode = xmlread( Scale_SetupFile );
Scale_TasksDomNode = xmlread( Scale_TasksFile );
Forward_SetupDomNode = xmlread( Forward_SetupFile );
Perturb_SetupDomNode = xmlread( Perturb_SetupFile );

%
% 3. Modify start and end times in each setup file as needed.
%

% 3.1 Type all values directly here.

ikTimeRange = [initialTimeStr ' ' finalTimeStr];
rraInitialTime = initialTimeStr;
rraFinalTime = finalTimeStr;
cmcInitialTime = cmcInitialTimeStr;
cmcFinalTime = cmcFinalTimeStr;
forwardInitialTime = cmcInitialTimeStr;
forwardFinalTime = num2str( cmcFinalTime - 0.1 );
perturbInitialTime = cmcInitialTimeStr;
perturbFinalTime = cmcFinalTimeStr;
analyzeInitialTime = cmcInitialTimeStr;
analyzeFinalTime = cmcFinalTimeStr;
muscleAnalysisInitialTime = cmcInitialTimeStr;
muscleAnalysisFinalTime = cmcFinalTimeStr;
idIkInitialTime = initialTimeStr;
idIkFinalTime = finalTimeStr;
idRraInitialTime = initialTimeStr;
idRraFinalTime = finalTimeStr;
idCmcInitialTime = cmcInitialTimeStr;
idCmcFinalTime = cmcFinalTimeStr;

% 3.2 Set values in DOM objects.

% IK setup file
IK_SetupMainBodyNode = IK_SetupDomNode.getChildNodes.item(0);
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SetupIkTrialSetNode = IK_SetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SetupIkTrialNode = IK_SetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'time_range' };
values = { ikTimeRange };
findAndFillInTags( IK_SetupIkTrialNode, desiredTagNames, values );

% RRA setup file
RRA_SetupMainBodyNode = RRA_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { rraInitialTime rraFinalTime };
findAndFillInTags( RRA_SetupMainBodyNode, desiredTagNames, values );

% CMC setup file
CMC_SetupMainBodyNode = CMC_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { cmcInitialTime cmcFinalTime };
findAndFillInTags( CMC_SetupMainBodyNode, desiredTagNames, values );

% Forward setup file
Forward_SetupMainBodyNode = Forward_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { forwardInitialTime forwardFinalTime };
findAndFillInTags( Forward_SetupMainBodyNode, desiredTagNames, values );

% Perturb setup file
Perturb_SetupMainBodyNode = Perturb_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { perturbInitialTime perturbFinalTime };
findAndFillInTags( Perturb_SetupMainBodyNode, desiredTagNames, values );

% Analyze setup file
Analyze_SetupMainBodyNode = Analyze_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { analyzeInitialTime analyzeFinalTime };
findAndFillInTags( Analyze_SetupMainBodyNode, desiredTagNames, values );

% MuscleAnalysis setup file
MuscleAnalysis_SetupMainBodyNode = MuscleAnalysis_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { muscleAnalysisInitialTime muscleAnalysisFinalTime };
findAndFillInTags( MuscleAnalysis_SetupMainBodyNode, desiredTagNames, values );

% ID IK setup file
IDIK_SetupMainBodyNode = IDIK_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idIkInitialTime idIkFinalTime };
findAndFillInTags( IDIK_SetupMainBodyNode, desiredTagNames, values );

% ID RRA setup file
IDRRA_SetupMainBodyNode = IDRRA_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idRraInitialTime idRraFinalTime };
findAndFillInTags( IDRRA_SetupMainBodyNode, desiredTagNames, values );

% ID CMC setup file
IDCMC_SetupMainBodyNode = IDCMC_SetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idCmcInitialTime idCmcFinalTime };
findAndFillInTags( IDCMC_SetupMainBodyNode, desiredTagNames, values );

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
Analyze_SetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze.xml'] );
CMC_ActuatorsFile = fullfile( outputDirectory, [trialName '_CMC_Actuators.xml'] );
CMC_ControlConstraintsFile = fullfile( outputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
CMC_SetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFile = fullfile( outputDirectory, [trialName '_CMC_Tasks.xml'] );
IDIK_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_IK.xml'] );
IDRRA_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_RRA.xml'] );
IDCMC_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_CMC.xml'] );
IK_SetupFile = fullfile( outputDirectory, [trialName '_Setup_IK.xml'] );
IK_TasksFile = fullfile( outputDirectory, [trialName '_IK_Tasks.xml'] );
MuscleAnalysis_SetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis.xml'] );
RRA_ActuatorsFile = fullfile( outputDirectory, [trialName '_RRA_Actuators.xml'] );
RRA_ControlConstraintsFile = fullfile( outputDirectory, [trialName '_RRA_ControlConstraints.xml'] );
RRA_SetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA.xml'] );
RRA_TasksFile = fullfile( outputDirectory, [trialName '_RRA_Tasks.xml'] );
Scale_MarkerSetFile = fullfile( outputDirectory, [subjectName '_Scale_MarkerSet.xml'] );
Scale_MeasurementSetFile = fullfile( outputDirectory, [subjectName '_Scale_MeasurementSet.xml'] );
Scale_ScaleSetFile = fullfile( outputDirectory, [subjectName '_Scale_ScaleSet.xml'] );
Scale_SetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale.xml'] );
Scale_TasksFile = fullfile( outputDirectory, [subjectName '_Scale_Tasks.xml'] );
Forward_SetupFile = fullfile( outputDirectory, [trialName '_Setup_Forward.xml'] );
Perturb_SetupFile = fullfile( outputDirectory, [trialName '_Setup_Perturb.xml'] );

% 4.2 Write all setup files.
xmlwrite( Analyze_SetupFile, Analyze_SetupDomNode );
xmlwrite( CMC_ActuatorsFile, CMC_ActuatorsDomNode );
xmlwrite( CMC_ControlConstraintsFile, CMC_ControlConstraintsDomNode );
xmlwrite( CMC_SetupFile, CMC_SetupDomNode );
xmlwrite( CMC_TasksFile, CMC_TasksDomNode );
xmlwrite( IDIK_SetupFile, IDIK_SetupDomNode );
xmlwrite( IDRRA_SetupFile, IDRRA_SetupDomNode );
xmlwrite( IDCMC_SetupFile, IDCMC_SetupDomNode );
xmlwrite( IK_SetupFile, IK_SetupDomNode );
xmlwrite( IK_TasksFile, IK_TasksDomNode );
xmlwrite( MuscleAnalysis_SetupFile, MuscleAnalysis_SetupDomNode);
xmlwrite( RRA_ActuatorsFile, RRA_ActuatorsDomNode );
xmlwrite( RRA_ControlConstraintsFile, RRA_ControlConstraintsDomNode );
xmlwrite( RRA_SetupFile, RRA_SetupDomNode );
xmlwrite( RRA_TasksFile, RRA_TasksDomNode );
xmlwrite( Scale_MarkerSetFile, Scale_MarkerSetDomNode );
xmlwrite( Scale_MeasurementSetFile, Scale_MeasurementSetDomNode );
xmlwrite( Scale_ScaleSetFile, Scale_ScaleSetDomNode );
xmlwrite( Scale_SetupFile, Scale_SetupDomNode );
xmlwrite( Scale_TasksFile, Scale_TasksDomNode );
xmlwrite( Forward_SetupFile, Forward_SetupDomNode );
xmlwrite( Perturb_SetupFile, Perturb_SetupDomNode );
