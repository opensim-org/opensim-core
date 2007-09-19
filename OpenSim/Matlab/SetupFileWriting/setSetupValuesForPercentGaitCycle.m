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
sourceSimbodyGenericModelFile = fullfile( inputDirectory, 'gait2392.osim' );
destinationSimbodyGenericModelFile = fullfile( outputDirectory, 'gait2392.osim' );
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

Analyze_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_Analyze_Sdfast.xml'] );
Analyze_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_Analyze_Simbody.xml'] );
CMC_ActuatorsFile = fullfile( inputDirectory, [trialName '_CMC_Actuators.xml'] );
CMC_ControlConstraintsFile = fullfile( inputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
CMC_SdfastCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_CMC_Sdfast_Cfsqp.xml'] );
CMC_SdfastIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_CMC_Sdfast_Ipopt.xml'] );
CMC_SimbodyCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_CMC_Simbody_Cfsqp.xml'] );
CMC_SimbodyIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_CMC_Simbody_Ipopt.xml'] );
CMC_TasksFile = fullfile( inputDirectory, [trialName '_CMC_Tasks.xml'] );
IDIK_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_IK_Sdfast.xml'] );
IDIK_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_IK_Simbody.xml'] );
IDRRA_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_RRA_Sdfast.xml'] );
IDRRA_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_RRA_Simbody.xml'] );
IDCMC_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_CMC_Sdfast.xml'] );
IDCMC_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_ID_CMC_Simbody.xml'] );
IK_SdfastCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_IK_Sdfast_Cfsqp.xml'] );
IK_SdfastIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_IK_Sdfast_Ipopt.xml'] );
IK_SimbodyCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_IK_Simbody_Cfsqp.xml'] );
IK_SimbodyIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_IK_Simbody_Ipopt.xml'] );
IK_TasksFile = fullfile( inputDirectory, [trialName '_IK_Tasks.xml'] );
MuscleAnalysis_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_MuscleAnalysis_Sdfast.xml'] );
MuscleAnalysis_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_MuscleAnalysis_Simbody.xml'] );
RRA_ActuatorsFile = fullfile( inputDirectory, [trialName '_RRA_Actuators.xml'] );
RRA_ControlConstraintsFile = fullfile( inputDirectory, [trialName '_RRA_ControlConstraints.xml'] );
RRA_SdfastCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_RRA_Sdfast_Cfsqp.xml'] );
RRA_SdfastIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_RRA_Sdfast_Ipopt.xml'] );
RRA_SimbodyCfsqpSetupFile = fullfile( inputDirectory, [trialName '_Setup_RRA_Simbody_Cfsqp.xml'] );
RRA_SimbodyIpoptSetupFile = fullfile( inputDirectory, [trialName '_Setup_RRA_Simbody_Ipopt.xml'] );
RRA_TasksFile = fullfile( inputDirectory, [trialName '_RRA_Tasks.xml'] );
Scale_MarkerSetFile = fullfile( inputDirectory, [subjectName '_Scale_MarkerSet.xml'] );
Scale_MeasurementSetFile = fullfile( inputDirectory, [subjectName '_Scale_MeasurementSet.xml'] );
Scale_ScaleSetFile = fullfile( inputDirectory, [subjectName '_Scale_ScaleSet.xml'] );
Scale_SdfastSetupFile = fullfile( inputDirectory, [subjectName '_Setup_Scale_Sdfast.xml'] );
Scale_SimbodySetupFile = fullfile( inputDirectory, [subjectName '_Setup_Scale_Simbody.xml'] );
Scale_TasksFile = fullfile( inputDirectory, [subjectName '_Scale_Tasks.xml'] );
Forward_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_Forward_Sdfast.xml'] );
Forward_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_Forward_Simbody.xml'] );
Perturb_SdfastSetupFile = fullfile( inputDirectory, [trialName '_Setup_Perturb_Sdfast.xml'] );
Perturb_SimbodySetupFile = fullfile( inputDirectory, [trialName '_Setup_Perturb_Simbody.xml'] );

Analyze_SdfastSetupDomNode = xmlread( Analyze_SdfastSetupFile );
Analyze_SimbodySetupDomNode = xmlread( Analyze_SimbodySetupFile );
CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFile );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFile );
CMC_SdfastCfsqpSetupDomNode = xmlread( CMC_SdfastCfsqpSetupFile );
CMC_SdfastIpoptSetupDomNode = xmlread( CMC_SdfastIpoptSetupFile );
CMC_SimbodyCfsqpSetupDomNode = xmlread( CMC_SimbodyCfsqpSetupFile );
CMC_SimbodyIpoptSetupDomNode = xmlread( CMC_SimbodyIpoptSetupFile );
CMC_TasksDomNode = xmlread( CMC_TasksFile );
IDIK_SdfastSetupDomNode = xmlread( IDIK_SdfastSetupFile );
IDIK_SimbodySetupDomNode = xmlread( IDIK_SimbodySetupFile );
IDRRA_SdfastSetupDomNode = xmlread( IDRRA_SdfastSetupFile );
IDRRA_SimbodySetupDomNode = xmlread( IDRRA_SimbodySetupFile );
IDCMC_SdfastSetupDomNode = xmlread( IDCMC_SdfastSetupFile );
IDCMC_SimbodySetupDomNode = xmlread( IDCMC_SimbodySetupFile );
IK_SdfastCfsqpSetupDomNode = xmlread( IK_SdfastCfsqpSetupFile );
IK_SdfastIpoptSetupDomNode = xmlread( IK_SdfastIpoptSetupFile );
IK_SimbodyCfsqpSetupDomNode = xmlread( IK_SimbodyCfsqpSetupFile );
IK_SimbodyIpoptSetupDomNode = xmlread( IK_SimbodyIpoptSetupFile );
IK_TasksDomNode = xmlread( IK_TasksFile );
MuscleAnalysis_SdfastSetupDomNode = xmlread( MuscleAnalysis_SdfastSetupFile );
MuscleAnalysis_SimbodySetupDomNode = xmlread( MuscleAnalysis_SimbodySetupFile );
RRA_ActuatorsDomNode = xmlread( RRA_ActuatorsFile );
RRA_ControlConstraintsDomNode = xmlread( RRA_ControlConstraintsFile );
RRA_SdfastCfsqpSetupDomNode = xmlread( RRA_SdfastCfsqpSetupFile );
RRA_SdfastIpoptSetupDomNode = xmlread( RRA_SdfastIpoptSetupFile );
RRA_SimbodyCfsqpSetupDomNode = xmlread( RRA_SimbodyCfsqpSetupFile );
RRA_SimbodyIpoptSetupDomNode = xmlread( RRA_SimbodyIpoptSetupFile );
RRA_TasksDomNode = xmlread( RRA_TasksFile );
Scale_MarkerSetDomNode = xmlread( Scale_MarkerSetFile );
Scale_MeasurementSetDomNode = xmlread( Scale_MeasurementSetFile );
Scale_ScaleSetDomNode = xmlread( Scale_ScaleSetFile );
Scale_SdfastSetupDomNode = xmlread( Scale_SdfastSetupFile );
Scale_SimbodySetupDomNode = xmlread( Scale_SimbodySetupFile );
Scale_TasksDomNode = xmlread( Scale_TasksFile );
Forward_SdfastSetupDomNode = xmlread( Forward_SdfastSetupFile );
Forward_SimbodySetupDomNode = xmlread( Forward_SimbodySetupFile );
Perturb_SdfastSetupDomNode = xmlread( Perturb_SdfastSetupFile );
Perturb_SimbodySetupDomNode = xmlread( Perturb_SimbodySetupFile );

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

% IK SD/FAST CFSQP setup file
IK_SdfastCfsqpSetupMainBodyNode = IK_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SdfastCfsqpSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SdfastCfsqpSetupIkTrialSetNode = IK_SdfastCfsqpSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SdfastCfsqpSetupIkTrialNode = IK_SdfastCfsqpSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'time_range' };
values = { ikTimeRange };
findAndFillInTags( IK_SdfastCfsqpSetupIkTrialNode, desiredTagNames, values );

% IK SD/FAST IPOPT setup file
IK_SdfastIpoptSetupMainBodyNode = IK_SdfastIpoptSetupDomNode.getChildNodes.item(0);
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SdfastIpoptSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SdfastIpoptSetupIkTrialSetNode = IK_SdfastIpoptSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SdfastIpoptSetupIkTrialNode = IK_SdfastIpoptSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'time_range' };
values = { ikTimeRange };
findAndFillInTags( IK_SdfastIpoptSetupIkTrialNode, desiredTagNames, values );

% IK Simbody CFSQP setup file
IK_SimbodyCfsqpSetupMainBodyNode = IK_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SimbodyCfsqpSetupIkTrialSetNode = IK_SimbodyCfsqpSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SimbodyCfsqpSetupIkTrialNode = IK_SimbodyCfsqpSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'time_range' };
values = { ikTimeRange };
findAndFillInTags( IK_SimbodyCfsqpSetupIkTrialNode, desiredTagNames, values );

% IK Simbody IPOPT setup file
IK_SimbodyIpoptSetupMainBodyNode = IK_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SimbodyIpoptSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SimbodyIpoptSetupIkTrialSetNode = IK_SimbodyIpoptSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SimbodyIpoptSetupIkTrialNode = IK_SimbodyIpoptSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'time_range' };
values = { ikTimeRange };
findAndFillInTags( IK_SimbodyIpoptSetupIkTrialNode, desiredTagNames, values );

% RRA SD/FAST CFSQP setup file
RRA_SdfastCfsqpSetupMainBodyNode = RRA_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { rraInitialTime rraFinalTime };
findAndFillInTags( RRA_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );

% RRA SD/FAST IPOPT setup file
RRA_SdfastIpoptSetupMainBodyNode = RRA_SdfastIpoptSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { rraInitialTime rraFinalTime };
findAndFillInTags( RRA_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );

% RRA SD/FAST Simbody setup file
RRA_SimbodyCfsqpSetupMainBodyNode = RRA_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { rraInitialTime rraFinalTime };
findAndFillInTags( RRA_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );

% RRA Simbody IPOPT setup file
RRA_SimbodyIpoptSetupMainBodyNode = RRA_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { rraInitialTime rraFinalTime };
findAndFillInTags( RRA_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );

% CMC SD/FAST CFSQP setup file
CMC_SdfastCfsqpSetupMainBodyNode = CMC_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { cmcInitialTime cmcFinalTime };
findAndFillInTags( CMC_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );

% CMC SD/FAST IPOPT setup file
CMC_SdfastIpoptSetupMainBodyNode = CMC_SdfastIpoptSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { cmcInitialTime cmcFinalTime };
findAndFillInTags( CMC_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );

% CMC Simbody CFSQP setup file
CMC_SimbodyCfsqpSetupMainBodyNode = CMC_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { cmcInitialTime cmcFinalTime };
findAndFillInTags( CMC_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );

% CMC Simbody IPOPT setup file
CMC_SimbodyIpoptSetupMainBodyNode = CMC_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { cmcInitialTime cmcFinalTime };
findAndFillInTags( CMC_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );

% Forward SD/FAST setup file
Forward_SdfastSetupMainBodyNode = Forward_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { forwardInitialTime forwardFinalTime };
findAndFillInTags( Forward_SdfastSetupMainBodyNode, desiredTagNames, values );

% Forward Simbody setup file
Forward_SimbodySetupMainBodyNode = Forward_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { forwardInitialTime forwardFinalTime };
findAndFillInTags( Forward_SimbodySetupMainBodyNode, desiredTagNames, values );

% Perturb SD/FAST setup file
Perturb_SdfastSetupMainBodyNode = Perturb_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { perturbInitialTime perturbFinalTime };
findAndFillInTags( Perturb_SdfastSetupMainBodyNode, desiredTagNames, values );

% Perturb Simbody setup file
Perturb_SimbodySetupMainBodyNode = Perturb_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { perturbInitialTime perturbFinalTime };
findAndFillInTags( Perturb_SimbodySetupMainBodyNode, desiredTagNames, values );

% Analyze SD/FAST setup file
Analyze_SdfastSetupMainBodyNode = Analyze_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { analyzeInitialTime analyzeFinalTime };
findAndFillInTags( Analyze_SdfastSetupMainBodyNode, desiredTagNames, values );

% Analyze Simbody setup file
Analyze_SimbodySetupMainBodyNode = Analyze_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { analyzeInitialTime analyzeFinalTime };
findAndFillInTags( Analyze_SimbodySetupMainBodyNode, desiredTagNames, values );

% MuscleAnalysis SD/FAST setup file
MuscleAnalysis_SdfastSetupMainBodyNode = MuscleAnalysis_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { muscleAnalysisInitialTime muscleAnalysisFinalTime };
findAndFillInTags( MuscleAnalysis_SdfastSetupMainBodyNode, desiredTagNames, values );

% MuscleAnalysis Simbody setup file
MuscleAnalysis_SimbodySetupMainBodyNode = MuscleAnalysis_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { muscleAnalysisInitialTime muscleAnalysisFinalTime };
findAndFillInTags( MuscleAnalysis_SimbodySetupMainBodyNode, desiredTagNames, values );

% ID IK SD/FAST setup file
IDIK_SdfastSetupMainBodyNode = IDIK_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idIkInitialTime idIkFinalTime };
findAndFillInTags( IDIK_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID IK Simbody setup file
IDIK_SimbodySetupMainBodyNode = IDIK_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idIkInitialTime idIkFinalTime };
findAndFillInTags( IDIK_SimbodySetupMainBodyNode, desiredTagNames, values );

% ID RRA SD/FAST setup file
IDRRA_SdfastSetupMainBodyNode = IDRRA_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idRraInitialTime idRraFinalTime };
findAndFillInTags( IDRRA_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID RRA Simbody setup file
IDRRA_SimbodySetupMainBodyNode = IDRRA_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idRraInitialTime idRraFinalTime };
findAndFillInTags( IDRRA_SimbodySetupMainBodyNode, desiredTagNames, values );

% ID CMC SD/FAST setup file
IDCMC_SdfastSetupMainBodyNode = IDCMC_SdfastSetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idCmcInitialTime idCmcFinalTime };
findAndFillInTags( IDCMC_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID CMC Simbody setup file
IDCMC_SimbodySetupMainBodyNode = IDCMC_SimbodySetupDomNode.getChildNodes.item(0);
desiredTagNames = { 'initial_time' 'final_time' };
values = { idCmcInitialTime idCmcFinalTime };
findAndFillInTags( IDCMC_SimbodySetupMainBodyNode, desiredTagNames, values );

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
Analyze_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze_Sdfast.xml'] );
Analyze_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze_Simbody.xml'] );
CMC_ActuatorsFile = fullfile( outputDirectory, [trialName '_CMC_Actuators.xml'] );
CMC_ControlConstraintsFile = fullfile( outputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
CMC_SdfastCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Sdfast_Cfsqp.xml'] );
CMC_SdfastIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Sdfast_Ipopt.xml'] );
CMC_SimbodyCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Simbody_Cfsqp.xml'] );
CMC_SimbodyIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Simbody_Ipopt.xml'] );
CMC_TasksFile = fullfile( outputDirectory, [trialName '_CMC_Tasks.xml'] );
IDIK_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_IK_Sdfast.xml'] );
IDIK_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_IK_Simbody.xml'] );
IDRRA_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_RRA_Sdfast.xml'] );
IDRRA_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_RRA_Simbody.xml'] );
IDCMC_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_CMC_Sdfast.xml'] );
IDCMC_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_CMC_Simbody.xml'] );
IK_SdfastCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_IK_Sdfast_Cfsqp.xml'] );
IK_SdfastIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_IK_Sdfast_Ipopt.xml'] );
IK_SimbodyCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_IK_Simbody_Cfsqp.xml'] );
IK_SimbodyIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_IK_Simbody_Ipopt.xml'] );
IK_TasksFile = fullfile( outputDirectory, [trialName '_IK_Tasks.xml'] );
MuscleAnalysis_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis_Sdfast.xml'] );
MuscleAnalysis_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis_Simbody.xml'] );
RRA_ActuatorsFile = fullfile( outputDirectory, [trialName '_RRA_Actuators.xml'] );
RRA_ControlConstraintsFile = fullfile( outputDirectory, [trialName '_RRA_ControlConstraints.xml'] );
RRA_SdfastCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Sdfast_Cfsqp.xml'] );
RRA_SdfastIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Sdfast_Ipopt.xml'] );
RRA_SimbodyCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Simbody_Cfsqp.xml'] );
RRA_SimbodyIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Simbody_Ipopt.xml'] );
RRA_TasksFile = fullfile( outputDirectory, [trialName '_RRA_Tasks.xml'] );
Scale_MarkerSetFile = fullfile( outputDirectory, [subjectName '_Scale_MarkerSet.xml'] );
Scale_MeasurementSetFile = fullfile( outputDirectory, [subjectName '_Scale_MeasurementSet.xml'] );
Scale_ScaleSetFile = fullfile( outputDirectory, [subjectName '_Scale_ScaleSet.xml'] );
Scale_SdfastSetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale_Sdfast.xml'] );
Scale_SimbodySetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale_Simbody.xml'] );
Scale_TasksFile = fullfile( outputDirectory, [subjectName '_Scale_Tasks.xml'] );
Forward_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_Forward_Sdfast.xml'] );
Forward_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_Forward_Simbody.xml'] );
Perturb_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_Perturb_Sdfast.xml'] );
Perturb_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_Perturb_Simbody.xml'] );

% 4.2 Write all setup files.
xmlwrite( Analyze_SdfastSetupFile, Analyze_SdfastSetupDomNode );
xmlwrite( Analyze_SimbodySetupFile, Analyze_SimbodySetupDomNode );
xmlwrite( CMC_ActuatorsFile, CMC_ActuatorsDomNode );
xmlwrite( CMC_ControlConstraintsFile, CMC_ControlConstraintsDomNode );
xmlwrite( CMC_SdfastCfsqpSetupFile, CMC_SdfastCfsqpSetupDomNode );
xmlwrite( CMC_SdfastIpoptSetupFile, CMC_SdfastIpoptSetupDomNode );
xmlwrite( CMC_SimbodyCfsqpSetupFile, CMC_SimbodyCfsqpSetupDomNode );
xmlwrite( CMC_SimbodyIpoptSetupFile, CMC_SimbodyIpoptSetupDomNode );
xmlwrite( CMC_TasksFile, CMC_TasksDomNode );
xmlwrite( IDIK_SdfastSetupFile, IDIK_SdfastSetupDomNode );
xmlwrite( IDIK_SimbodySetupFile, IDIK_SimbodySetupDomNode );
xmlwrite( IDRRA_SdfastSetupFile, IDRRA_SdfastSetupDomNode );
xmlwrite( IDRRA_SimbodySetupFile, IDRRA_SimbodySetupDomNode );
xmlwrite( IDCMC_SdfastSetupFile, IDCMC_SdfastSetupDomNode );
xmlwrite( IDCMC_SimbodySetupFile, IDCMC_SimbodySetupDomNode );
xmlwrite( IK_SdfastCfsqpSetupFile, IK_SdfastCfsqpSetupDomNode );
xmlwrite( IK_SdfastIpoptSetupFile, IK_SdfastIpoptSetupDomNode );
xmlwrite( IK_SimbodyCfsqpSetupFile, IK_SimbodyCfsqpSetupDomNode );
xmlwrite( IK_SimbodyIpoptSetupFile, IK_SimbodyIpoptSetupDomNode );
xmlwrite( IK_TasksFile, IK_TasksDomNode );
xmlwrite( MuscleAnalysis_SdfastSetupFile, MuscleAnalysis_SdfastSetupDomNode);
xmlwrite( MuscleAnalysis_SimbodySetupFile, MuscleAnalysis_SimbodySetupDomNode);
xmlwrite( RRA_ActuatorsFile, RRA_ActuatorsDomNode );
xmlwrite( RRA_ControlConstraintsFile, RRA_ControlConstraintsDomNode );
xmlwrite( RRA_SdfastCfsqpSetupFile, RRA_SdfastCfsqpSetupDomNode );
xmlwrite( RRA_SdfastIpoptSetupFile, RRA_SdfastIpoptSetupDomNode );
xmlwrite( RRA_SimbodyCfsqpSetupFile, RRA_SimbodyCfsqpSetupDomNode );
xmlwrite( RRA_SimbodyIpoptSetupFile, RRA_SimbodyIpoptSetupDomNode );
xmlwrite( RRA_TasksFile, RRA_TasksDomNode );
xmlwrite( Scale_MarkerSetFile, Scale_MarkerSetDomNode );
xmlwrite( Scale_MeasurementSetFile, Scale_MeasurementSetDomNode );
xmlwrite( Scale_ScaleSetFile, Scale_ScaleSetDomNode );
xmlwrite( Scale_SdfastSetupFile, Scale_SdfastSetupDomNode );
xmlwrite( Scale_SimbodySetupFile, Scale_SimbodySetupDomNode );
xmlwrite( Scale_TasksFile, Scale_TasksDomNode );
xmlwrite( Forward_SdfastSetupFile, Forward_SdfastSetupDomNode );
xmlwrite( Forward_SimbodySetupFile, Forward_SimbodySetupDomNode );
xmlwrite( Perturb_SdfastSetupFile, Perturb_SdfastSetupDomNode );
xmlwrite( Perturb_SimbodySetupFile, Perturb_SimbodySetupDomNode );
