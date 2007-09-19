function setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings )
%
% Write setup files for Simulation workflow.
%
% Author: Chand T. John
%

%
% 0. Do initial setup steps.
%

% Copy gait2392.osim file from gait2392 example directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
sourceSdfastGenericModelFile = fullfile( gait2392ExampleDirectory, 'gait2392.osim' );
destinationSdfastGenericModelFile = fullfile( outputDirectory, 'gait2392.osim' );
outputDirectoryDoesntExist = ( exist( outputDirectory, 'dir' ) ~= 7 );
if outputDirectoryDoesntExist
    mkdir( outputDirectory );
end
copyfile( sourceSdfastGenericModelFile, destinationSdfastGenericModelFile );

% Copy gait2392_simbody.osim file from gait2392 example directory to
% destination  directory, if this file doesn't already exist in the
% destination directory.
sourceSimbodyGenericModelFile = fullfile( gait2392ExampleDirectory, 'gait2392_simbody.osim' );
destinationSimbodyGenericModelFile = fullfile( outputDirectory, 'gait2392_simbody.osim' );
outputDirectoryDoesntExist = ( exist( outputDirectory, 'dir' ) ~= 7 );
if outputDirectoryDoesntExist
    mkdir( outputDirectory );
end
copyfile( sourceSimbodyGenericModelFile, destinationSimbodyGenericModelFile );

% Set default indices for values in specialSettings cell.
MASS = 1;
HEIGHT = 2;
FEMURSCALEFACTOR = 3;
TIBIASCALEFACTOR = 4;
FXMINCCRRA = 5;
FXMAXCCRRA = 6;
FYMINCCRRA = 7;
FYMAXCCRRA = 8;
FZMINCCRRA = 9;
FZMAXCCRRA = 10;
MXMINCCRRA = 11;
MXMAXCCRRA = 12;
MYMINCCRRA = 13;
MYMAXCCRRA = 14;
MZMINCCRRA = 15;
MZMAXCCRRA = 16;
FXOPTFORCERRA = 17;
FYOPTFORCERRA = 18;
FZOPTFORCERRA = 19;
MXOPTFORCERRA = 20;
MYOPTFORCERRA = 21;
MZOPTFORCERRA = 22;
FXMINCCCMC = 23;
FXMAXCCCMC = 24;
FYMINCCCMC = 25;
FYMAXCCCMC = 26;
FZMINCCCMC = 27;
FZMAXCCCMC = 28;
MXMINCCCMC = 29;
MXMAXCCCMC = 30;
MYMINCCCMC = 31;
MYMAXCCCMC = 32;
MZMINCCCMC = 33;
MZMAXCCCMC = 34;
FXOPTFORCECMC = 35;
FYOPTFORCECMC = 36;
FZOPTFORCECMC = 37;
MXOPTFORCECMC = 38;
MYOPTFORCECMC = 39;
MZOPTFORCECMC = 40;
IDRESULTSDIRECTORY = 41;

%
% 1. Construct full paths for all gait2392 example files from OpenSim.
%

Analyze_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Analyze.xml' );
CMC_ActuatorsFile = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_Actuators.xml' );
CMC_ControlConstraintsFile = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_ControlConstraints.xml' );
CMC_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_CMC.xml' );
CMC_TasksFile = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_Tasks.xml' );
InverseDynamics_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_InverseDynamics.xml' );
IK_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_IK.xml' );
IK_TasksFile = fullfile( gait2392ExampleDirectory, 'gait2392_IK_Tasks.xml' );
RRA_ActuatorsFile = fullfile( gait2392ExampleDirectory, 'gait2392_RRA_Actuators.xml' );
RRA_ControlConstraintsFile = fullfile( gait2392ExampleDirectory, 'gait2392_RRA_ControlConstraints.xml' );
RRA_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_RRA.xml' );
RRA_TasksFile = fullfile( gait2392ExampleDirectory, 'gait2392_RRA_Tasks.xml' );
Scale_MarkerSetFile = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_MarkerSet.xml' );
Scale_MeasurementSetFile = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_MeasurementSet.xml' );
Scale_ScaleSetFile = fullfile( gait2392ExampleDirectory, 'subject01_Scale_ScaleSet.xml' );
Scale_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Scale.xml' );
Scale_TasksFile = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_Tasks.xml' );
Forward_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Forward.xml' );
Perturb_SetupFile = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Perturb.xml' );

%
% 2. Read in all relevant gait2392 example setup files from OpenSim.
%

Analyze_SdfastSetupDomNode = xmlread( Analyze_SetupFile );
Analyze_SimbodySetupDomNode = xmlread( Analyze_SetupFile );
CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFile );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFile );
CMC_SdfastCfsqpSetupDomNode = xmlread( CMC_SetupFile );
CMC_SdfastIpoptSetupDomNode = xmlread( CMC_SetupFile );
CMC_SimbodyCfsqpSetupDomNode = xmlread( CMC_SetupFile );
CMC_SimbodyIpoptSetupDomNode = xmlread( CMC_SetupFile );
CMC_TasksDomNode = xmlread( CMC_TasksFile );
IDIK_SdfastSetupDomNode = xmlread( InverseDynamics_SetupFile );
IDIK_SimbodySetupDomNode = xmlread( InverseDynamics_SetupFile );
IDRRA_SdfastSetupDomNode = xmlread( InverseDynamics_SetupFile );
IDRRA_SimbodySetupDomNode = xmlread( InverseDynamics_SetupFile );
IDCMC_SdfastSetupDomNode = xmlread( InverseDynamics_SetupFile );
IDCMC_SimbodySetupDomNode = xmlread( InverseDynamics_SetupFile );
IK_SdfastCfsqpSetupDomNode = xmlread( IK_SetupFile );
IK_SdfastIpoptSetupDomNode = xmlread( IK_SetupFile );
IK_SimbodyCfsqpSetupDomNode = xmlread( IK_SetupFile );
IK_SimbodyIpoptSetupDomNode = xmlread( IK_SetupFile );
IK_TasksDomNode = xmlread( IK_TasksFile );
MuscleAnalysis_SdfastSetupDomNode = xmlread( Analyze_SetupFile );
MuscleAnalysis_SimbodySetupDomNode = xmlread( Analyze_SetupFile );
RRA_ActuatorsDomNode = xmlread( RRA_ActuatorsFile );
RRA_ControlConstraintsDomNode = xmlread( RRA_ControlConstraintsFile );
RRA_SdfastCfsqpSetupDomNode = xmlread( RRA_SetupFile );
RRA_SimbodyCfsqpSetupDomNode = xmlread( RRA_SetupFile );
RRA_SdfastIpoptSetupDomNode = xmlread( RRA_SetupFile );
RRA_SimbodyIpoptSetupDomNode = xmlread( RRA_SetupFile );
RRA_TasksDomNode = xmlread( RRA_TasksFile );
Scale_MarkerSetDomNode = xmlread( Scale_MarkerSetFile );
Scale_MeasurementSetDomNode = xmlread( Scale_MeasurementSetFile );
Scale_ScaleSetDomNode = xmlread( Scale_ScaleSetFile );
Scale_SdfastSetupDomNode = xmlread( Scale_SetupFile );
Scale_SimbodySetupDomNode = xmlread( Scale_SetupFile );
Scale_TasksDomNode = xmlread( Scale_TasksFile );
Forward_SdfastSetupDomNode = xmlread( Forward_SetupFile );
Forward_SimbodySetupDomNode = xmlread( Forward_SetupFile );
Perturb_SdfastSetupDomNode = xmlread( Perturb_SetupFile );
Perturb_SimbodySetupDomNode = xmlread( Perturb_SetupFile );

%
% 3. Modify each setup file according to values in big SimTrack setup file.
%

% 3.1 Type all values directly here.

% 3.1.1 Set scale values.

% 3.1.1.1 Set scale setup file values.
scaleName = subjectName;
mass = specialSettings{ MASS };
height = specialSettings{ HEIGHT };
notes = 'Age unknown.';
sdfastScaleModelFile = 'gait2392.osim';
simbodyScaleModelFile = 'gait2392_simbody.osim';
markerSetFile = [subjectName '_Scale_MarkerSet.xml'];
scaleSetFile = [subjectName '_Scale_ScaleSet.xml'];
measurementSetFile = [subjectName '_Scale_MeasurementSet.xml'];
scaleModelScalerMarkerFile = [subjectName '_static.trc'];
modelScalerTimeRange = '1 2';
preserveMassDistribution = 'true';
scaledOnlyModelFile = [subjectName '_scaledOnly.osim'];
scaledOnlyScaleFile = [subjectName '_scaleSet_applied.xml'];
scaleMarkerPlacerMarkerFile = scaleModelScalerMarkerFile;
scaleTaskSetFile = [subjectName '_Scale_Tasks.xml'];
markerPlacerTimeRange = '1 2';
sdfastSubjectSpecificModelFile = [subjectName '.osim'];
simbodySubjectSpecificModelFile = [subjectName '_simbody.osim'];
scaleOutputMotionFile = [subjectName '_static_output.mot'];

% 3.1.1.2 Set scale set values.
femurScaleFactor = specialSettings{ FEMURSCALEFACTOR };
tibiaScaleFactor = specialSettings{ TIBIASCALEFACTOR };

% 3.1.2 Set IK values.

% 3.1.2.1 Set IK setup file values.
ikName = subjectName;
ikSdfastModelFile = sdfastSubjectSpecificModelFile;
ikSimbodyModelFile = simbodySubjectSpecificModelFile;
ikCfsqpOptimizerAlgorithm = 'cfsqp';
ikIpoptOptimizerAlgorithm = 'ipopt';
ikTaskSetFile = [trialName '_IK_Tasks.xml'];
ikMarkerFile = [trialName '.trc'];
ikCoordinateFile = [trialName '.mot'];
ikTimeRange = '0.5 4';
desiredKinematicsFile = [trialName '_ik.mot'];
ikOutputMotionFile = desiredKinematicsFile;

% 3.1.3 Set RRA values.

% 3.1.3.1 Set RRA setup file values.
rraName = [trialName '_RRA'];
sdfastDynamicModelFile = [subjectName '_sdfast.osim'];
rraSdfastModelFile = sdfastDynamicModelFile;
rraSimbodyModelFile = simbodySubjectSpecificModelFile;
rraActuatorSetFiles = [trialName '_RRA_Actuators.xml'];
rraResultsDirectory = './ResultsRRA';
rraInitialTime = '0.5';
rraFinalTime = '2';
rraSdfastOutputModelFile = [subjectName '_sdfast_adjusted.osim'];
sdfastAdjustedModelFile = rraSdfastOutputModelFile;
rraSimbodyOutputModelFile = [subjectName '_simbody_adjusted.osim'];
simbodyAdjustedModelFile = rraSimbodyOutputModelFile;
rraDesiredKinematicsFile = desiredKinematicsFile;
rraTaskSetFile = [trialName '_RRA_Tasks.xml'];
rraConstraintsFile = [trialName '_RRA_ControlConstraints.xml'];
externalLoadsFile = [trialName '_grf.mot'];
rraExternalLoadsFile = externalLoadsFile;
rraExternalLoadsModelKinematicsFile = desiredKinematicsFile;
rraCfsqpOptimizerAlgorithm = 'cfsqp';
rraIpoptOptimizerAlgorithm = 'ipopt';

% 3.1.3.2 Set RRA control constraint file values.
fxMinControlConstraintRRA = specialSettings{ FXMINCCRRA };
fxMaxControlConstraintRRA = specialSettings{ FXMAXCCRRA };
fyMinControlConstraintRRA = specialSettings{ FYMINCCRRA };
fyMaxControlConstraintRRA = specialSettings{ FYMAXCCRRA };
fzMinControlConstraintRRA = specialSettings{ FZMINCCRRA };
fzMaxControlConstraintRRA = specialSettings{ FZMAXCCRRA };
mxMinControlConstraintRRA = specialSettings{ MXMINCCRRA };
mxMaxControlConstraintRRA = specialSettings{ MXMAXCCRRA };
myMinControlConstraintRRA = specialSettings{ MYMINCCRRA };
myMaxControlConstraintRRA = specialSettings{ MYMAXCCRRA };
mzMinControlConstraintRRA = specialSettings{ MZMINCCRRA };
mzMaxControlConstraintRRA = specialSettings{ MZMAXCCRRA };

% 3.1.3.3 Set RRA actuator file values.
fxOptimalForceRRA = specialSettings{ FXOPTFORCERRA };
fyOptimalForceRRA = specialSettings{ FYOPTFORCERRA };
fzOptimalForceRRA = specialSettings{ FZOPTFORCERRA };
mxOptimalForceRRA = specialSettings{ MXOPTFORCERRA };
myOptimalForceRRA = specialSettings{ MYOPTFORCERRA };
mzOptimalForceRRA = specialSettings{ MZOPTFORCERRA };

% 3.1.4 Set CMC values.

% 3.1.4.1 Set CMC setup file values.
cmcName = trialName;
cmcSdfastModelFile = sdfastAdjustedModelFile;
cmcSimbodyModelFile = simbodyAdjustedModelFile;
cmcActuatorSetFiles = [trialName '_CMC_Actuators.xml'];
cmcResultsDirectory = './ResultsCMC';
cmcInitialTime = '0.5';
cmcFinalTime = '2';
cmcDesiredKinematicsFile = fullfile( rraResultsDirectory, [trialName '_RRA_Kinematics_q.sto'] );
cmcTaskSetFile = [trialName '_CMC_Tasks.xml'];
cmcConstraintsFile = [trialName '_CMC_ControlConstraints.xml'];
cmcExternalLoadsFile = externalLoadsFile;
cmcExternalLoadsModelKinematicsFile = desiredKinematicsFile;
cmcOptimizerConvergenceCriterion = '1e-006';
cmcCfsqpOptimizerAlgorithm = 'cfsqp';
cmcIpoptOptimizerAlgorithm = 'ipopt';
cmcUseCurvatureFilter = 'true';

% 3.1.4.2 Set CMC control constraint file values.
fxMinControlConstraintCMC = specialSettings{ FXMINCCCMC };
fxMaxControlConstraintCMC = specialSettings{ FXMAXCCCMC };
fyMinControlConstraintCMC = specialSettings{ FYMINCCCMC };
fyMaxControlConstraintCMC = specialSettings{ FYMAXCCCMC };
fzMinControlConstraintCMC = specialSettings{ FZMINCCCMC };
fzMaxControlConstraintCMC = specialSettings{ FZMAXCCCMC };
mxMinControlConstraintCMC = specialSettings{ MXMINCCCMC };
mxMaxControlConstraintCMC = specialSettings{ MXMAXCCCMC };
myMinControlConstraintCMC = specialSettings{ MYMINCCCMC };
myMaxControlConstraintCMC = specialSettings{ MYMAXCCCMC };
mzMinControlConstraintCMC = specialSettings{ MZMINCCCMC };
mzMaxControlConstraintCMC = specialSettings{ MZMAXCCCMC };

% 3.1.4.3 Set CMC actuator file values.
fxOptimalForceCMC = specialSettings{ FXOPTFORCECMC };
fyOptimalForceCMC = specialSettings{ FYOPTFORCECMC };
fzOptimalForceCMC = specialSettings{ FZOPTFORCECMC };
mxOptimalForceCMC = specialSettings{ MXOPTFORCECMC };
myOptimalForceCMC = specialSettings{ MYOPTFORCECMC };
mzOptimalForceCMC = specialSettings{ MZOPTFORCECMC };

% 3.1.5 Set Forward and Perturb values.

% 3.1.5.1 Set Forward setup file values.
forwardName = trialName;
forwardSdfastModelFile = sdfastAdjustedModelFile;
forwardSimbodyModelFile = simbodyAdjustedModelFile;
forwardActuatorSetFiles = [trialName '_CMC_Actuators.xml'];
forwardResultsDirectory = './ResultsForward';
forwardInitialTime = '0.5';
forwardFinalTime = '1.9';
forwardControlsFile = fullfile( cmcResultsDirectory, [cmcName '_controls.xml'] );
forwardStatesFile = fullfile( cmcResultsDirectory, [cmcName '_states.sto'] );
useSpecifiedDt = 'true';
forwardExternalLoadsFile = externalLoadsFile;
forwardExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.5.2 Set Perturb setup file values.
perturbName = trialName;
perturbSdfastModelFile = sdfastAdjustedModelFile;
perturbSimbodyModelFile = simbodyAdjustedModelFile;
perturbActuatorSetFiles = [trialName '_CMC_Actuators.xml'];
perturbResultsDirectory = './ResultsPerturb';
perturbInitialTime = '0.5';
perturbFinalTime = '1.9';
perturbControlsFile = fullfile( cmcResultsDirectory, [cmcName '_controls.xml'] );
perturbCopFile = externalLoadsFile;
perturbCoordinatesFile = fullfile( cmcResultsDirectory, [cmcName '_Kinematics_q.sto'] );
perturbSpeedsFile = fullfile( cmcResultsDirectory, [cmcName '_Kinematics_u.sto'] );
perturbStatesFile = fullfile( cmcResultsDirectory, [cmcName '_states.sto'] );
perturbExternalLoadsFile = externalLoadsFile;
perturbExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.6 Set Analyze values.

% 3.1.6.1 Set Analyze setup file values.
analyzeName = trialName;
analyzeSdfastModelFile = sdfastAdjustedModelFile;
analyzeSimbodyModelFile = simbodyAdjustedModelFile;
analyzeActuatorSetFiles = [trialName '_CMC_Actuators.xml'];
analyzeResultsDirectory = './ResultsAnalyze';
analyzeOutputPrecision = '20';
analyzeInitialTime = '0.5';
analyzeFinalTime = '2.0';
analyzeMuscleAnalysisMomentArmCoordinateList = 'all';
analyzeControlsFile = fullfile( cmcResultsDirectory, [cmcName '_controls.xml'] );
analyzeStatesFile = fullfile( cmcResultsDirectory, [cmcName '_states.sto'] );
analyzeExternalLoadsFile = externalLoadsFile;
analyzeExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.6.2 Set Muscle Analysis setup file values.
muscleAnalysisName = trialName;
muscleAnalysisSdfastModelFile = sdfastAdjustedModelFile;
muscleAnalysisSimbodyModelFile = simbodyAdjustedModelFile;
muscleAnalysisActuatorSetFiles = [trialName '_CMC_Actuators.xml'];
muscleAnalysisResultsDirectory = './ResultsMuscleAnalysis';
muscleAnalysisOutputPrecision = '20';
muscleAnalysisInitialTime = '0.5';
muscleAnalysisFinalTime = '2.0';
muscleAnalysisMuscleAnalysisMomentArmCoordinateList = 'all';
muscleAnalysisControlsFile = fullfile( cmcResultsDirectory, [cmcName '_controls.xml'] );
muscleAnalysisStatesFile = fullfile( cmcResultsDirectory, [cmcName '_states.sto'] );
muscleAnalysisExternalLoadsFile = externalLoadsFile;
muscleAnalysisExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.7 Set ID (Inverse Dynamics) values.

% 3.1.7.1 Set setup file values for ID for IK results.
idIkName = [trialName '_ik'];
idIkSdfastModelFile = ikSdfastModelFile;
idIkSimbodyModelFile = ikSimbodyModelFile;
idIkResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idIkInitialTime = '0.5';
idIkFinalTime = '2.0';
idIkCoordinatesFile = desiredKinematicsFile;
idIkExternalLoadsFile = externalLoadsFile;
idIkExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.7.2 Set setup file values for ID for RRA results.
idRraName = [trialName '_rra'];
idRraSdfastModelFile = sdfastAdjustedModelFile;
idRraSimbodyModelFile = simbodyAdjustedModelFile;
idRraResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idRraInitialTime = '0.5';
idRraFinalTime = '2.0';
idRraCoordinatesFile = fullfile( rraResultsDirectory, [rraName '_Kinematics_q.sto'] );
idRraExternalLoadsFile = externalLoadsFile;
idRraExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.7.3 Set setup file values for ID for CMC results.
idCmcName = [trialName '_cmc'];
idCmcSdfastModelFile = sdfastAdjustedModelFile;
idCmcSimbodyModelFile = simbodyAdjustedModelFile;
idCmcResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idCmcInitialTime = '0.5';
idCmcFinalTime = '2.0';
idCmcCoordinatesFile = fullfile( cmcResultsDirectory, [cmcName '_Kinematics_q.sto'] );
idCmcExternalLoadsFile = externalLoadsFile;
idCmcExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.2 Set these new values in the setup file DOM nodes.

% Scale SD/FAST setup file
Scale_SdfastSetupMainBodyNode = Scale_SdfastSetupDomNode.getChildNodes.item(0);
% Name
Scale_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( scaleName );
% Mass, height, notes
desiredTagNames = { 'mass' 'height' 'notes' };
values = { mass height notes };
findAndFillInTags( Scale_SdfastSetupMainBodyNode, desiredTagNames, values );
% GenericModelMaker, ModelScaler, MarkerPlacer
desiredTagNames = { 'GenericModelMaker' 'ModelScaler' 'MarkerPlacer' };
classTagIndices = { -1 -1 -1 };
classTagIndices = findAndFillInTagIndices( Scale_SdfastSetupMainBodyNode, desiredTagNames, classTagIndices );
% Model file, marker set file
genericModelMakerIndex = classTagIndices{1};
Scale_SdfastSetupGenericModelMakerNode = Scale_SdfastSetupMainBodyNode.getChildNodes.item( genericModelMakerIndex );
desiredTagNames = { 'model_file' 'marker_set_file' };
values = { sdfastScaleModelFile markerSetFile };
findAndFillInTags( Scale_SdfastSetupGenericModelMakerNode, desiredTagNames, values );
% ScaleSet, MeasurementSet
modelScalerIndex = classTagIndices{2};
Scale_SdfastSetupModelScalerNode = Scale_SdfastSetupMainBodyNode.getChildNodes.item( modelScalerIndex );
desiredTagNames = { 'ScaleSet' 'MeasurementSet' };
values = { scaleSetFile measurementSetFile };
findAndFillInFirstAttributesOfTags( Scale_SdfastSetupModelScalerNode, desiredTagNames, values );
% Marker file, time range, preserve mass distribution, output model file,
% output scale file
desiredTagNames = { 'marker_file' 'time_range' 'preserve_mass_distribution' ...
    'output_model_file' 'output_scale_file' };
values = { scaleModelScalerMarkerFile modelScalerTimeRange preserveMassDistribution ...
    scaledOnlyModelFile scaledOnlyScaleFile };
findAndFillInTags( Scale_SdfastSetupModelScalerNode, desiredTagNames, values );
% IKTaskSet
markerPlacerIndex = classTagIndices{3};
Scale_SdfastSetupMarkerPlacerNode = Scale_SdfastSetupMainBodyNode.getChildNodes.item( markerPlacerIndex );
desiredTagNames = { 'IKTaskSet' };
values = { scaleTaskSetFile };
findAndFillInFirstAttributesOfTags( Scale_SdfastSetupMarkerPlacerNode, desiredTagNames, values );
% Marker file, time range, output model file, output motion file
desiredTagNames = { 'marker_file' 'time_range' 'output_model_file' 'output_motion_file' };
values = { scaleMarkerPlacerMarkerFile markerPlacerTimeRange ...
    sdfastSubjectSpecificModelFile scaleOutputMotionFile };
findAndFillInTags( Scale_SdfastSetupMarkerPlacerNode, desiredTagNames, values );

% Scale Simbody setup file
Scale_SimbodySetupMainBodyNode = Scale_SimbodySetupDomNode.getChildNodes.item(0);
% Name
Scale_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( scaleName );
% Mass, height, notes
desiredTagNames = { 'mass' 'height' 'notes' };
values = { mass height notes };
findAndFillInTags( Scale_SimbodySetupMainBodyNode, desiredTagNames, values );
% GenericModelMaker, ModelScaler, MarkerPlacer
desiredTagNames = { 'GenericModelMaker' 'ModelScaler' 'MarkerPlacer' };
classTagIndices = { -1 -1 -1 };
classTagIndices = findAndFillInTagIndices( Scale_SimbodySetupMainBodyNode, desiredTagNames, classTagIndices );
% Model file, marker set file
genericModelMakerIndex = classTagIndices{1};
Scale_SimbodySetupGenericModelMakerNode = Scale_SimbodySetupMainBodyNode.getChildNodes.item( genericModelMakerIndex );
desiredTagNames = { 'model_file' 'marker_set_file' };
values = { simbodyScaleModelFile markerSetFile };
findAndFillInTags( Scale_SimbodySetupGenericModelMakerNode, desiredTagNames, values );
% ScaleSet, MeasurementSet
modelScalerIndex = classTagIndices{2};
Scale_SimbodySetupModelScalerNode = Scale_SimbodySetupMainBodyNode.getChildNodes.item( modelScalerIndex );
desiredTagNames = { 'ScaleSet' 'MeasurementSet' };
values = { scaleSetFile measurementSetFile };
findAndFillInFirstAttributesOfTags( Scale_SimbodySetupModelScalerNode, desiredTagNames, values );
% Marker file, time range, preserve mass distribution, output model file,
% output scale file
desiredTagNames = { 'marker_file' 'time_range' 'preserve_mass_distribution' ...
    'output_model_file' 'output_scale_file' };
values = { scaleModelScalerMarkerFile modelScalerTimeRange preserveMassDistribution ...
    scaledOnlyModelFile scaledOnlyScaleFile };
findAndFillInTags( Scale_SimbodySetupModelScalerNode, desiredTagNames, values );
% IKTaskSet
markerPlacerIndex = classTagIndices{3};
Scale_SimbodySetupMarkerPlacerNode = Scale_SimbodySetupMainBodyNode.getChildNodes.item( markerPlacerIndex );
desiredTagNames = { 'IKTaskSet' };
values = { scaleTaskSetFile };
findAndFillInFirstAttributesOfTags( Scale_SimbodySetupMarkerPlacerNode, desiredTagNames, values );
% Marker file, time range, output model file, output motion file
desiredTagNames = { 'marker_file' 'time_range' 'output_model_file' 'output_motion_file' };
values = { scaleMarkerPlacerMarkerFile markerPlacerTimeRange ...
    simbodySubjectSpecificModelFile scaleOutputMotionFile };
findAndFillInTags( Scale_SimbodySetupMarkerPlacerNode, desiredTagNames, values );

% Scale set file
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(3).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(5).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(7).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );

% IK SD/FAST CFSQP setup file
IK_SdfastCfsqpSetupMainBodyNode = IK_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
% Name
IK_SdfastCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( ikName );
% Model file, optimizer algorithm
desiredTagNames = { 'model_file' 'optimizer_algorithm' };
values = { ikSdfastModelFile ikCfsqpOptimizerAlgorithm };
findAndFillInTags( IK_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );
% IKTaskSet
desiredTagNames = { 'IKTaskSet' };
values = { ikTaskSetFile };
findAndFillInFirstAttributesOfTags( IK_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SdfastCfsqpSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SdfastCfsqpSetupIkTrialSetNode = IK_SdfastCfsqpSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SdfastCfsqpSetupIkTrialNode = IK_SdfastCfsqpSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'marker_file' 'coordinate_file' 'time_range' 'output_motion_file' };
values = { ikMarkerFile ikCoordinateFile ikTimeRange ikOutputMotionFile };
findAndFillInTags( IK_SdfastCfsqpSetupIkTrialNode, desiredTagNames, values );

% IK SD/FAST IPOPT setup file
IK_SdfastIpoptSetupMainBodyNode = IK_SdfastIpoptSetupDomNode.getChildNodes.item(0);
% Name
IK_SdfastIpoptSetupMainBodyNode.getAttributes.item(0).setValue( ikName );
% Model file, optimizer algorithm
desiredTagNames = { 'model_file' 'optimizer_algorithm' };
values = { ikSdfastModelFile ikIpoptOptimizerAlgorithm };
findAndFillInTags( IK_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );
% IKTaskSet
desiredTagNames = { 'IKTaskSet' };
values = { ikTaskSetFile };
findAndFillInFirstAttributesOfTags( IK_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SdfastIpoptSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SdfastIpoptSetupIkTrialSetNode = IK_SdfastIpoptSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SdfastIpoptSetupIkTrialNode = IK_SdfastIpoptSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'marker_file' 'coordinate_file' 'time_range' 'output_motion_file' };
values = { ikMarkerFile ikCoordinateFile ikTimeRange ikOutputMotionFile };
findAndFillInTags( IK_SdfastIpoptSetupIkTrialNode, desiredTagNames, values );

% IK Simbody CFSQP setup file
IK_SimbodyCfsqpSetupMainBodyNode = IK_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
% Name
IK_SimbodyCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( ikName );
% Model file, optimizer algorithm
desiredTagNames = { 'model_file' 'optimizer_algorithm' };
values = { ikSimbodyModelFile ikCfsqpOptimizerAlgorithm };
findAndFillInTags( IK_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );
% IKTaskSet
desiredTagNames = { 'IKTaskSet' };
values = { ikTaskSetFile };
findAndFillInFirstAttributesOfTags( IK_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SimbodyCfsqpSetupIkTrialSetNode = IK_SimbodyCfsqpSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SimbodyCfsqpSetupIkTrialNode = IK_SimbodyCfsqpSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'marker_file' 'coordinate_file' 'time_range' 'output_motion_file' };
values = { ikMarkerFile ikCoordinateFile ikTimeRange ikOutputMotionFile };
findAndFillInTags( IK_SimbodyCfsqpSetupIkTrialNode, desiredTagNames, values );

% IK Simbody IPOPT setup file
IK_SimbodyIpoptSetupMainBodyNode = IK_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
% Name
IK_SimbodyIpoptSetupMainBodyNode.getAttributes.item(0).setValue( ikName );
% Model file, optimizer algorithm
desiredTagNames = { 'model_file' 'optimizer_algorithm' };
values = { ikSimbodyModelFile ikIpoptOptimizerAlgorithm };
findAndFillInTags( IK_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );
% IKTaskSet
desiredTagNames = { 'IKTaskSet' };
values = { ikTaskSetFile };
findAndFillInFirstAttributesOfTags( IK_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SimbodyIpoptSetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SimbodyIpoptSetupIkTrialSetNode = IK_SimbodyIpoptSetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SimbodyIpoptSetupIkTrialNode = IK_SimbodyIpoptSetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'marker_file' 'coordinate_file' 'time_range' 'output_motion_file' };
values = { ikMarkerFile ikCoordinateFile ikTimeRange ikOutputMotionFile };
findAndFillInTags( IK_SimbodyIpoptSetupIkTrialNode, desiredTagNames, values );

% RRA SD/FAST CFSQP setup file
RRA_SdfastCfsqpSetupMainBodyNode = RRA_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
% Name
RRA_SdfastCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( rraName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'output_model_file' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_algorithm' };
values = { rraSdfastModelFile rraActuatorSetFiles rraResultsDirectory ...
    rraInitialTime rraFinalTime rraSdfastOutputModelFile rraDesiredKinematicsFile ...
    rraTaskSetFile rraConstraintsFile rraExternalLoadsFile ...
    rraExternalLoadsModelKinematicsFile rraCfsqpOptimizerAlgorithm };
findAndFillInTags( RRA_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );

% RRA SD/FAST IPOPT setup file
RRA_SdfastIpoptSetupMainBodyNode = RRA_SdfastIpoptSetupDomNode.getChildNodes.item(0);
% Name
RRA_SdfastIpoptSetupMainBodyNode.getAttributes.item(0).setValue( rraName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'output_model_file' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_algorithm' };
values = { rraSdfastModelFile rraActuatorSetFiles rraResultsDirectory ...
    rraInitialTime rraFinalTime rraSdfastOutputModelFile rraDesiredKinematicsFile ...
    rraTaskSetFile rraConstraintsFile rraExternalLoadsFile ...
    rraExternalLoadsModelKinematicsFile rraIpoptOptimizerAlgorithm };
findAndFillInTags( RRA_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );

% RRA SD/FAST Simbody setup file
RRA_SimbodyCfsqpSetupMainBodyNode = RRA_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
% Name
RRA_SimbodyCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( rraName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'output_model_file' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_algorithm' };
values = { rraSimbodyModelFile rraActuatorSetFiles rraResultsDirectory ...
    rraInitialTime rraFinalTime rraSimbodyOutputModelFile rraDesiredKinematicsFile ...
    rraTaskSetFile rraConstraintsFile rraExternalLoadsFile ...
    rraExternalLoadsModelKinematicsFile rraCfsqpOptimizerAlgorithm };
findAndFillInTags( RRA_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );

% RRA Simbody IPOPT setup file
RRA_SimbodyIpoptSetupMainBodyNode = RRA_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
% Name
RRA_SimbodyIpoptSetupMainBodyNode.getAttributes.item(0).setValue( rraName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'output_model_file' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_algorithm' };
values = { rraSimbodyModelFile rraActuatorSetFiles rraResultsDirectory ...
    rraInitialTime rraFinalTime rraSimbodyOutputModelFile rraDesiredKinematicsFile ...
    rraTaskSetFile rraConstraintsFile rraExternalLoadsFile ...
    rraExternalLoadsModelKinematicsFile rraIpoptOptimizerAlgorithm };
findAndFillInTags( RRA_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );

% RRA control constraints file
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 1).getChildNodes.item(1).getChildNodes.item(0).setData( fxMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 1).getChildNodes.item(3).getChildNodes.item(0).setData( fxMaxControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fyMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(3).getChildNodes.item(0).setData( fyMaxControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(1).getChildNodes.item(0).setData( fzMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(3).getChildNodes.item(0).setData( fzMaxControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( mxMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(3).getChildNodes.item(0).setData( mxMaxControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( myMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(3).getChildNodes.item(0).setData( myMaxControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( mzMinControlConstraintRRA );
RRA_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(3).getChildNodes.item(0).setData( mzMaxControlConstraintRRA );

% RRA actuators file
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fxOptimalForceRRA );
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(1).getChildNodes.item(0).setData( fyOptimalForceRRA );
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( fzOptimalForceRRA );
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( mxOptimalForceRRA );
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( myOptimalForceRRA );
RRA_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(13).getChildNodes.item(1).getChildNodes.item(0).setData( mzOptimalForceRRA );

% CMC SD/FAST CFSQP setup file
CMC_SdfastCfsqpSetupMainBodyNode = CMC_SdfastCfsqpSetupDomNode.getChildNodes.item(0);
% Name
CMC_SdfastCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( cmcName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_convergence_criterion' ...
    'optimizer_algorithm' 'use_curvature_filter' };
values = { cmcSdfastModelFile cmcActuatorSetFiles cmcResultsDirectory ...
    cmcInitialTime cmcFinalTime cmcDesiredKinematicsFile ...
    cmcTaskSetFile cmcConstraintsFile cmcExternalLoadsFile ...
    cmcExternalLoadsModelKinematicsFile cmcOptimizerConvergenceCriterion ...
    cmcCfsqpOptimizerAlgorithm cmcUseCurvatureFilter };
findAndFillInTags( CMC_SdfastCfsqpSetupMainBodyNode, desiredTagNames, values );

% CMC SD/FAST IPOPT setup file
CMC_SdfastIpoptSetupMainBodyNode = CMC_SdfastIpoptSetupDomNode.getChildNodes.item(0);
% Name
CMC_SdfastIpoptSetupMainBodyNode.getAttributes.item(0).setValue( cmcName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_convergence_criterion' ...
    'optimizer_algorithm' 'use_curvature_filter' };
values = { cmcSdfastModelFile cmcActuatorSetFiles cmcResultsDirectory ...
    cmcInitialTime cmcFinalTime cmcDesiredKinematicsFile ...
    cmcTaskSetFile cmcConstraintsFile cmcExternalLoadsFile ...
    cmcExternalLoadsModelKinematicsFile cmcOptimizerConvergenceCriterion ...
    cmcIpoptOptimizerAlgorithm cmcUseCurvatureFilter };
findAndFillInTags( CMC_SdfastIpoptSetupMainBodyNode, desiredTagNames, values );

% CMC Simbody CFSQP setup file
CMC_SimbodyCfsqpSetupMainBodyNode = CMC_SimbodyCfsqpSetupDomNode.getChildNodes.item(0);
% Name
CMC_SimbodyCfsqpSetupMainBodyNode.getAttributes.item(0).setValue( cmcName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_convergence_criterion' ...
    'optimizer_algorithm' 'use_curvature_filter' };
values = { cmcSimbodyModelFile cmcActuatorSetFiles cmcResultsDirectory ...
    cmcInitialTime cmcFinalTime cmcDesiredKinematicsFile ...
    cmcTaskSetFile cmcConstraintsFile cmcExternalLoadsFile ...
    cmcExternalLoadsModelKinematicsFile cmcOptimizerConvergenceCriterion ...
    cmcCfsqpOptimizerAlgorithm cmcUseCurvatureFilter };
findAndFillInTags( CMC_SimbodyCfsqpSetupMainBodyNode, desiredTagNames, values );

% CMC Simbody IPOPT setup file
CMC_SimbodyIpoptSetupMainBodyNode = CMC_SimbodyIpoptSetupDomNode.getChildNodes.item(0);
% Name
CMC_SimbodyIpoptSetupMainBodyNode.getAttributes.item(0).setValue( cmcName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_convergence_criterion' ...
    'optimizer_algorithm' 'use_curvature_filter' };
values = { cmcSimbodyModelFile cmcActuatorSetFiles cmcResultsDirectory ...
    cmcInitialTime cmcFinalTime cmcDesiredKinematicsFile ...
    cmcTaskSetFile cmcConstraintsFile cmcExternalLoadsFile ...
    cmcExternalLoadsModelKinematicsFile cmcOptimizerConvergenceCriterion ...
    cmcIpoptOptimizerAlgorithm cmcUseCurvatureFilter };
findAndFillInTags( CMC_SimbodyIpoptSetupMainBodyNode, desiredTagNames, values );

% CMC control constraints file
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 1).getChildNodes.item(1).getChildNodes.item(0).setData( fxMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 1).getChildNodes.item(3).getChildNodes.item(0).setData( fxMaxControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fyMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(3).getChildNodes.item(0).setData( fyMaxControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(1).getChildNodes.item(0).setData( fzMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(3).getChildNodes.item(0).setData( fzMaxControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( mxMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(3).getChildNodes.item(0).setData( mxMaxControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( myMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(3).getChildNodes.item(0).setData( myMaxControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( mzMinControlConstraintCMC );
CMC_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(3).getChildNodes.item(0).setData( mzMaxControlConstraintCMC );

% CMC actuators file
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fxOptimalForceCMC );
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(1).getChildNodes.item(0).setData( fyOptimalForceCMC );
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( fzOptimalForceCMC );
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( mxOptimalForceCMC );
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( myOptimalForceCMC );
CMC_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(13).getChildNodes.item(1).getChildNodes.item(0).setData( mzOptimalForceCMC );

% Forward SD/FAST setup file
Forward_SdfastSetupMainBodyNode = Forward_SdfastSetupDomNode.getChildNodes.item(0);
% Name
Forward_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( forwardName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'use_specified_dt' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { forwardSdfastModelFile forwardActuatorSetFiles forwardResultsDirectory ...
    forwardInitialTime forwardFinalTime forwardControlsFile ...
    forwardStatesFile useSpecifiedDt forwardExternalLoadsFile ...
    forwardExternalLoadsModelKinematicsFile };
findAndFillInTags( Forward_SdfastSetupMainBodyNode, desiredTagNames, values );

% Forward Simbody setup file
Forward_SimbodySetupMainBodyNode = Forward_SimbodySetupDomNode.getChildNodes.item(0);
% Name
Forward_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( forwardName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'use_specified_dt' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { forwardSimbodyModelFile forwardActuatorSetFiles forwardResultsDirectory ...
    forwardInitialTime forwardFinalTime forwardControlsFile ...
    forwardStatesFile useSpecifiedDt forwardExternalLoadsFile ...
    forwardExternalLoadsModelKinematicsFile };
findAndFillInTags( Forward_SimbodySetupMainBodyNode, desiredTagNames, values );

% Perturb SD/FAST setup file
Perturb_SdfastSetupMainBodyNode = Perturb_SdfastSetupDomNode.getChildNodes.item(0);
% Name
Perturb_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( perturbName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'cop_file' 'coordinates_file' 'speeds_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { perturbSdfastModelFile perturbActuatorSetFiles perturbResultsDirectory ...
    perturbInitialTime perturbFinalTime perturbControlsFile ...
    perturbCopFile perturbCoordinatesFile perturbSpeedsFile ...
    perturbStatesFile perturbExternalLoadsFile ...
    perturbExternalLoadsModelKinematicsFile };
findAndFillInTags( Perturb_SdfastSetupMainBodyNode, desiredTagNames, values );

% Perturb Simbody setup file
Perturb_SimbodySetupMainBodyNode = Perturb_SimbodySetupDomNode.getChildNodes.item(0);
% Name
Perturb_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( perturbName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'cop_file' 'coordinates_file' 'speeds_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { perturbSimbodyModelFile perturbActuatorSetFiles perturbResultsDirectory ...
    perturbInitialTime perturbFinalTime perturbControlsFile ...
    perturbCopFile perturbCoordinatesFile perturbSpeedsFile ...
    perturbStatesFile perturbExternalLoadsFile ...
    perturbExternalLoadsModelKinematicsFile };
findAndFillInTags( Perturb_SimbodySetupMainBodyNode, desiredTagNames, values );

% Analyze SD/FAST setup file
Analyze_SdfastSetupMainBodyNode = Analyze_SdfastSetupDomNode.getChildNodes.item(0);
% Name
Analyze_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( analyzeName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { analyzeSdfastModelFile analyzeActuatorSetFiles analyzeResultsDirectory ...
    analyzeOutputPrecision analyzeInitialTime analyzeFinalTime ...
    analyzeControlsFile analyzeStatesFile analyzeExternalLoadsFile ...
    analyzeExternalLoadsModelKinematicsFile };
findAndFillInTags( Analyze_SdfastSetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SdfastSetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
Analyze_SdfastSetupAnalysisSetNode = Analyze_SdfastSetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SdfastSetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
Analyze_SdfastSetupObjectsNode = Analyze_SdfastSetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SdfastSetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
Analyze_SdfastSetupMuscleAnalysisNode = Analyze_SdfastSetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { analyzeMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( Analyze_SdfastSetupMuscleAnalysisNode, desiredTagNames, values );

% Analyze Simbody setup file
Analyze_SimbodySetupMainBodyNode = Analyze_SimbodySetupDomNode.getChildNodes.item(0);
% Name
Analyze_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( analyzeName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { analyzeSimbodyModelFile analyzeActuatorSetFiles analyzeResultsDirectory ...
    analyzeOutputPrecision analyzeInitialTime analyzeFinalTime ...
    analyzeControlsFile analyzeStatesFile analyzeExternalLoadsFile ...
    analyzeExternalLoadsModelKinematicsFile };
findAndFillInTags( Analyze_SimbodySetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SimbodySetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
Analyze_SimbodySetupAnalysisSetNode = Analyze_SimbodySetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SimbodySetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
Analyze_SimbodySetupObjectsNode = Analyze_SimbodySetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SimbodySetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
Analyze_SimbodySetupMuscleAnalysisNode = Analyze_SimbodySetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { analyzeMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( Analyze_SimbodySetupMuscleAnalysisNode, desiredTagNames, values );

% MuscleAnalysis SD/FAST setup file
MuscleAnalysis_SdfastSetupMainBodyNode = MuscleAnalysis_SdfastSetupDomNode.getChildNodes.item(0);
% Name
MuscleAnalysis_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( muscleAnalysisName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { muscleAnalysisSdfastModelFile muscleAnalysisActuatorSetFiles muscleAnalysisResultsDirectory ...
    muscleAnalysisOutputPrecision muscleAnalysisInitialTime muscleAnalysisFinalTime ...
    muscleAnalysisControlsFile muscleAnalysisStatesFile muscleAnalysisExternalLoadsFile ...
    muscleAnalysisExternalLoadsModelKinematicsFile };
findAndFillInTags( MuscleAnalysis_SdfastSetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SdfastSetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
MuscleAnalysis_SdfastSetupAnalysisSetNode = MuscleAnalysis_SdfastSetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SdfastSetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
MuscleAnalysis_SdfastSetupObjectsNode = MuscleAnalysis_SdfastSetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SdfastSetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
MuscleAnalysis_SdfastSetupMuscleAnalysisNode = MuscleAnalysis_SdfastSetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { muscleAnalysisMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( MuscleAnalysis_SdfastSetupMuscleAnalysisNode, desiredTagNames, values );

% MuscleAnalysis Simbody setup file
MuscleAnalysis_SimbodySetupMainBodyNode = MuscleAnalysis_SimbodySetupDomNode.getChildNodes.item(0);
% Name
MuscleAnalysis_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( muscleAnalysisName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { muscleAnalysisSimbodyModelFile muscleAnalysisActuatorSetFiles muscleAnalysisResultsDirectory ...
    muscleAnalysisOutputPrecision muscleAnalysisInitialTime muscleAnalysisFinalTime ...
    muscleAnalysisControlsFile muscleAnalysisStatesFile muscleAnalysisExternalLoadsFile ...
    muscleAnalysisExternalLoadsModelKinematicsFile };
findAndFillInTags( MuscleAnalysis_SimbodySetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SimbodySetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
MuscleAnalysis_SimbodySetupAnalysisSetNode = MuscleAnalysis_SimbodySetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SimbodySetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
MuscleAnalysis_SimbodySetupObjectsNode = MuscleAnalysis_SimbodySetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SimbodySetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
MuscleAnalysis_SimbodySetupMuscleAnalysisNode = MuscleAnalysis_SimbodySetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { muscleAnalysisMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( MuscleAnalysis_SimbodySetupMuscleAnalysisNode, desiredTagNames, values );

% ID IK SD/FAST setup file
IDIK_SdfastSetupMainBodyNode = IDIK_SdfastSetupDomNode.getChildNodes.item(0);
% Name
IDIK_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( idIkName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idIkSdfastModelFile idIkResultsDirectory ...
    idIkInitialTime idIkFinalTime idIkCoordinatesFile idIkExternalLoadsFile ...
    idIkExternalLoadsModelKinematicsFile };
findAndFillInTags( IDIK_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID IK Simbody setup file
IDIK_SimbodySetupMainBodyNode = IDIK_SimbodySetupDomNode.getChildNodes.item(0);
% Name
IDIK_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( idIkName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idIkSimbodyModelFile idIkResultsDirectory ...
    idIkInitialTime idIkFinalTime idIkCoordinatesFile idIkExternalLoadsFile ...
    idIkExternalLoadsModelKinematicsFile };
findAndFillInTags( IDIK_SimbodySetupMainBodyNode, desiredTagNames, values );

% ID RRA SD/FAST setup file
IDRRA_SdfastSetupMainBodyNode = IDRRA_SdfastSetupDomNode.getChildNodes.item(0);
% Name
IDRRA_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( idRraName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idRraSdfastModelFile idRraResultsDirectory ...
    idRraInitialTime idRraFinalTime idRraCoordinatesFile idRraExternalLoadsFile ...
    idRraExternalLoadsModelKinematicsFile };
findAndFillInTags( IDRRA_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID RRA Simbody setup file
IDRRA_SimbodySetupMainBodyNode = IDRRA_SimbodySetupDomNode.getChildNodes.item(0);
% Name
IDRRA_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( idRraName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idRraSimbodyModelFile idRraResultsDirectory ...
    idRraInitialTime idRraFinalTime idRraCoordinatesFile idRraExternalLoadsFile ...
    idRraExternalLoadsModelKinematicsFile };
findAndFillInTags( IDRRA_SimbodySetupMainBodyNode, desiredTagNames, values );

% ID CMC SD/FAST setup file
IDCMC_SdfastSetupMainBodyNode = IDCMC_SdfastSetupDomNode.getChildNodes.item(0);
% Name
IDCMC_SdfastSetupMainBodyNode.getAttributes.item(0).setValue( idCmcName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idCmcSdfastModelFile idCmcResultsDirectory ...
    idCmcInitialTime idCmcFinalTime idCmcCoordinatesFile idCmcExternalLoadsFile ...
    idCmcExternalLoadsModelKinematicsFile };
findAndFillInTags( IDCMC_SdfastSetupMainBodyNode, desiredTagNames, values );

% ID CMC Simbody setup file
IDCMC_SimbodySetupMainBodyNode = IDCMC_SimbodySetupDomNode.getChildNodes.item(0);
% Name
IDCMC_SimbodySetupMainBodyNode.getAttributes.item(0).setValue( idCmcName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idCmcSimbodyModelFile idCmcResultsDirectory ...
    idCmcInitialTime idCmcFinalTime idCmcCoordinatesFile idCmcExternalLoadsFile ...
    idCmcExternalLoadsModelKinematicsFile };
findAndFillInTags( IDCMC_SimbodySetupMainBodyNode, desiredTagNames, values );

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
Analyze_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze_Sdfast.xml'] );
Analyze_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze_Simbody.xml'] );
CMC_ActuatorsFile = fullfile( outputDirectory, cmcActuatorSetFiles );
CMC_ControlConstraintsFile = fullfile( outputDirectory, cmcConstraintsFile );
CMC_SdfastCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Sdfast_Cfsqp.xml'] );
CMC_SdfastIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Sdfast_Ipopt.xml'] );
CMC_SimbodyCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Simbody_Cfsqp.xml'] );
CMC_SimbodyIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC_Simbody_Ipopt.xml'] );
CMC_TasksFile = fullfile( outputDirectory, cmcTaskSetFile );
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
IK_TasksFile = fullfile( outputDirectory, ikTaskSetFile );
MuscleAnalysis_SdfastSetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis_Sdfast.xml'] );
MuscleAnalysis_SimbodySetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis_Simbody.xml'] );
RRA_ActuatorsFile = fullfile( outputDirectory, rraActuatorSetFiles );
RRA_ControlConstraintsFile = fullfile( outputDirectory, rraConstraintsFile );
RRA_SdfastCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Sdfast_Cfsqp.xml'] );
RRA_SdfastIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Sdfast_Ipopt.xml'] );
RRA_SimbodyCfsqpSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Simbody_Cfsqp.xml'] );
RRA_SimbodyIpoptSetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA_Simbody_Ipopt.xml'] );
RRA_TasksFile = fullfile( outputDirectory, rraTaskSetFile );
Scale_MarkerSetFile = fullfile( outputDirectory, markerSetFile );
Scale_MeasurementSetFile = fullfile( outputDirectory, measurementSetFile );
Scale_ScaleSetFile = fullfile( outputDirectory, scaleSetFile );
Scale_SdfastSetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale_Sdfast.xml'] );
Scale_SimbodySetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale_Simbody.xml'] );
Scale_TasksFile = fullfile( outputDirectory, scaleTaskSetFile );
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
