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
DYNAMICSENGINE = 42;
IKOPTIMIZERALGORITHM = 43;
RRAOPTIMIZERALGORITHM = 44;
CMCOPTIMIZERALGORITHM = 45;

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

Analyze_SetupDomNode = xmlread( Analyze_SetupFile );
CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFile );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFile );
CMC_SetupDomNode = xmlread( CMC_SetupFile );
CMC_TasksDomNode = xmlread( CMC_TasksFile );
IDIK_SetupDomNode = xmlread( InverseDynamics_SetupFile );
IDRRA_SetupDomNode = xmlread( InverseDynamics_SetupFile );
IDCMC_SetupDomNode = xmlread( InverseDynamics_SetupFile );
IK_SetupDomNode = xmlread( IK_SetupFile );
IK_TasksDomNode = xmlread( IK_TasksFile );
MuscleAnalysis_SetupDomNode = xmlread( Analyze_SetupFile );
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
% 3. Modify each setup file according to values in big SimTrack setup file.
%

% 3.1 Type all values directly here.

% 3.1.1 Set scale values.

% 3.1.1.1 Set scale setup file values.
scaleName = subjectName;
mass = specialSettings{ MASS };
height = specialSettings{ HEIGHT };
notes = 'Age unknown.';
dynamicsEngine = specialSettings{ DYNAMICSENGINE };
if strcmpi( dynamicsEngine, 'sdfast' )
    scaleModelFile = 'gait2392.osim';
elseif strcmpi( dynamicsEngine, 'simbody' )
    scaleModelFile = 'gait2392_simbody.osim';
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
if strcmpi( dynamicsEngine, 'sdfast' )
    scaleOutputModelFile = [subjectName '.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    scaleOutputModelFile = [subjectName '_simbody.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
scaleOutputMotionFile = [subjectName '_static_output.mot'];

% 3.1.1.2 Set scale set values.
femurScaleFactor = specialSettings{ FEMURSCALEFACTOR };
tibiaScaleFactor = specialSettings{ TIBIASCALEFACTOR };

% 3.1.2 Set IK values.

% 3.1.2.1 Set IK setup file values.
ikName = subjectName;
if strcmpi( dynamicsEngine, 'sdfast' )
    ikModelFile = [subjectName '.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    ikModelFile = [subjectName '_simbody.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
ikOptimizerAlgorithm = specialSettings{ IKOPTIMIZERALGORITHM };
ikTaskSetFile = [trialName '_IK_Tasks.xml'];
ikMarkerFile = [trialName '.trc'];
ikCoordinateFile = [trialName '.mot'];
ikTimeRange = '0.5 4';
desiredKinematicsFile = [trialName '_ik.mot'];
ikOutputMotionFile = desiredKinematicsFile;

% 3.1.3 Set RRA values.

% 3.1.3.1 Set RRA setup file values.
rraName = [trialName '_RRA'];
if strcmpi( dynamicsEngine, 'sdfast' )
    rraModelFile = [subjectName '_sdfast.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    rraModelFile = [subjectName '_simbody.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
rraActuatorSetFiles = [trialName '_RRA_Actuators.xml'];
rraResultsDirectory = './ResultsRRA';
rraInitialTime = '0.5';
rraFinalTime = '2';
if strcmpi( dynamicsEngine, 'sdfast' )
    rraOutputModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    rraOutputModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
rraDesiredKinematicsFile = desiredKinematicsFile;
rraTaskSetFile = [trialName '_RRA_Tasks.xml'];
rraConstraintsFile = [trialName '_RRA_ControlConstraints.xml'];
externalLoadsFile = [trialName '_grf.mot'];
rraExternalLoadsFile = externalLoadsFile;
rraExternalLoadsModelKinematicsFile = desiredKinematicsFile;
rraOptimizerAlgorithm = specialSettings{ RRAOPTIMIZERALGORITHM };

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
if strcmpi( dynamicsEngine, 'sdfast' )
    cmcModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    cmcModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
cmcOptimizerAlgorithm = specialSettings{ CMCOPTIMIZERALGORITHM };
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
if strcmpi( dynamicsEngine, 'sdfast' )
    forwardModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    forwardModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
if strcmpi( dynamicsEngine, 'sdfast' )
    perturbModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    perturbModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
if strcmpi( dynamicsEngine, 'sdfast' )
    analyzeModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    analyzeModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
if strcmpi( dynamicsEngine, 'sdfast' )
    muscleAnalysisModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    muscleAnalysisModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
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
if strcmpi( dynamicsEngine, 'sdfast' )
    idIkModelFile = [subjectName '_sdfast.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    idIkModelFile = [subjectName '_simbody.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
idIkResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idIkInitialTime = '0.5';
idIkFinalTime = '2.0';
idIkCoordinatesFile = desiredKinematicsFile;
idIkExternalLoadsFile = externalLoadsFile;
idIkExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.7.2 Set setup file values for ID for RRA results.
idRraName = [trialName '_rra'];
if strcmpi( dynamicsEngine, 'sdfast' )
    idRraModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    idRraModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
idRraResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idRraInitialTime = '0.5';
idRraFinalTime = '2.0';
idRraCoordinatesFile = fullfile( rraResultsDirectory, [rraName '_Kinematics_q.sto'] );
idRraExternalLoadsFile = externalLoadsFile;
idRraExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.1.7.3 Set setup file values for ID for CMC results.
idCmcName = [trialName '_cmc'];
if strcmpi( dynamicsEngine, 'sdfast' )
    idCmcModelFile = [subjectName '_sdfast_adjusted.osim'];
elseif strcmpi( dynamicsEngine, 'simbody' )
    idCmcModelFile = [subjectName '_simbody_adjusted.osim'];
else
    error( ['Invalid dynamics engine: ' dynamicsEngine] );
end
idCmcResultsDirectory = specialSettings{ IDRESULTSDIRECTORY };
idCmcInitialTime = '0.5';
idCmcFinalTime = '2.0';
idCmcCoordinatesFile = fullfile( cmcResultsDirectory, [cmcName '_Kinematics_q.sto'] );
idCmcExternalLoadsFile = externalLoadsFile;
idCmcExternalLoadsModelKinematicsFile = desiredKinematicsFile;

% 3.2 Set these new values in the setup file DOM nodes.

% Scale setup file
Scale_SetupMainBodyNode = Scale_SetupDomNode.getChildNodes.item(0);
% Name
Scale_SetupMainBodyNode.getAttributes.item(0).setValue( scaleName );
% Mass, height, notes
desiredTagNames = { 'mass' 'height' 'notes' };
values = { mass height notes };
findAndFillInTags( Scale_SetupMainBodyNode, desiredTagNames, values );
% GenericModelMaker, ModelScaler, MarkerPlacer
desiredTagNames = { 'GenericModelMaker' 'ModelScaler' 'MarkerPlacer' };
classTagIndices = { -1 -1 -1 };
classTagIndices = findAndFillInTagIndices( Scale_SetupMainBodyNode, desiredTagNames, classTagIndices );
% Model file, marker set file
genericModelMakerIndex = classTagIndices{1};
Scale_SetupGenericModelMakerNode = Scale_SetupMainBodyNode.getChildNodes.item( genericModelMakerIndex );
desiredTagNames = { 'model_file' 'marker_set_file' };
values = { scaleModelFile markerSetFile };
findAndFillInTags( Scale_SetupGenericModelMakerNode, desiredTagNames, values );
% ScaleSet, MeasurementSet
modelScalerIndex = classTagIndices{2};
Scale_SetupModelScalerNode = Scale_SetupMainBodyNode.getChildNodes.item( modelScalerIndex );
desiredTagNames = { 'ScaleSet' 'MeasurementSet' };
values = { scaleSetFile measurementSetFile };
findAndFillInFirstAttributesOfTags( Scale_SetupModelScalerNode, desiredTagNames, values );
% Marker file, time range, preserve mass distribution, output model file,
% output scale file
desiredTagNames = { 'marker_file' 'time_range' 'preserve_mass_distribution' ...
    'output_model_file' 'output_scale_file' };
values = { scaleModelScalerMarkerFile modelScalerTimeRange preserveMassDistribution ...
    scaledOnlyModelFile scaledOnlyScaleFile };
findAndFillInTags( Scale_SetupModelScalerNode, desiredTagNames, values );
% IKTaskSet
markerPlacerIndex = classTagIndices{3};
Scale_SetupMarkerPlacerNode = Scale_SetupMainBodyNode.getChildNodes.item( markerPlacerIndex );
desiredTagNames = { 'IKTaskSet' };
values = { scaleTaskSetFile };
findAndFillInFirstAttributesOfTags( Scale_SetupMarkerPlacerNode, desiredTagNames, values );
% Marker file, time range, output model file, output motion file
desiredTagNames = { 'marker_file' 'time_range' 'output_model_file' 'output_motion_file' };
values = { scaleMarkerPlacerMarkerFile markerPlacerTimeRange ...
    scaleOutputModelFile scaleOutputMotionFile };
findAndFillInTags( Scale_SetupMarkerPlacerNode, desiredTagNames, values );

% Scale set file
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(3).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(5).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(7).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );

% IK setup file
IK_SetupMainBodyNode = IK_SetupDomNode.getChildNodes.item(0);
% Name
IK_SetupMainBodyNode.getAttributes.item(0).setValue( ikName );
% Model file, optimizer algorithm
desiredTagNames = { 'model_file' 'optimizer_algorithm' };
values = { ikModelFile ikOptimizerAlgorithm };
findAndFillInTags( IK_SetupMainBodyNode, desiredTagNames, values );
% IKTaskSet
desiredTagNames = { 'IKTaskSet' };
values = { ikTaskSetFile };
findAndFillInFirstAttributesOfTags( IK_SetupMainBodyNode, desiredTagNames, values );
% IKTrialSet
desiredTagNames = { 'IKTrialSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( IK_SetupMainBodyNode, desiredTagNames, classTagIndices );
% IKTrial
ikTrialSetIndex = classTagIndices{1};
IK_SetupIkTrialSetNode = IK_SetupMainBodyNode.getChildNodes.item( ikTrialSetIndex );
IK_SetupIkTrialNode = IK_SetupIkTrialSetNode.getChildNodes.item(1).getChildNodes.item(1);
desiredTagNames = { 'marker_file' 'coordinate_file' 'time_range' 'output_motion_file' };
values = { ikMarkerFile ikCoordinateFile ikTimeRange ikOutputMotionFile };
findAndFillInTags( IK_SetupIkTrialNode, desiredTagNames, values );

% RRA setup file
RRA_SetupMainBodyNode = RRA_SetupDomNode.getChildNodes.item(0);
% Name
RRA_SetupMainBodyNode.getAttributes.item(0).setValue( rraName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'output_model_file' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_algorithm' };
values = { rraModelFile rraActuatorSetFiles rraResultsDirectory ...
    rraInitialTime rraFinalTime rraOutputModelFile rraDesiredKinematicsFile ...
    rraTaskSetFile rraConstraintsFile rraExternalLoadsFile ...
    rraExternalLoadsModelKinematicsFile rraOptimizerAlgorithm };
findAndFillInTags( RRA_SetupMainBodyNode, desiredTagNames, values );

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

% CMC setup file
CMC_SetupMainBodyNode = CMC_SetupDomNode.getChildNodes.item(0);
% Name
CMC_SetupMainBodyNode.getAttributes.item(0).setValue( cmcName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'desired_kinematics_file' ...
    'task_set_file' 'constraints_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' 'optimizer_convergence_criterion' ...
    'optimizer_algorithm' 'use_curvature_filter' };
values = { cmcModelFile cmcActuatorSetFiles cmcResultsDirectory ...
    cmcInitialTime cmcFinalTime cmcDesiredKinematicsFile ...
    cmcTaskSetFile cmcConstraintsFile cmcExternalLoadsFile ...
    cmcExternalLoadsModelKinematicsFile cmcOptimizerConvergenceCriterion ...
    cmcOptimizerAlgorithm cmcUseCurvatureFilter };
findAndFillInTags( CMC_SetupMainBodyNode, desiredTagNames, values );

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

% Forward setup file
Forward_SetupMainBodyNode = Forward_SetupDomNode.getChildNodes.item(0);
% Name
Forward_SetupMainBodyNode.getAttributes.item(0).setValue( forwardName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'use_specified_dt' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { forwardModelFile forwardActuatorSetFiles forwardResultsDirectory ...
    forwardInitialTime forwardFinalTime forwardControlsFile ...
    forwardStatesFile useSpecifiedDt forwardExternalLoadsFile ...
    forwardExternalLoadsModelKinematicsFile };
findAndFillInTags( Forward_SetupMainBodyNode, desiredTagNames, values );

% Perturb setup file
Perturb_SetupMainBodyNode = Perturb_SetupDomNode.getChildNodes.item(0);
% Name
Perturb_SetupMainBodyNode.getAttributes.item(0).setValue( perturbName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'initial_time' 'final_time' 'controls_file' ...
    'cop_file' 'coordinates_file' 'speeds_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { perturbModelFile perturbActuatorSetFiles perturbResultsDirectory ...
    perturbInitialTime perturbFinalTime perturbControlsFile ...
    perturbCopFile perturbCoordinatesFile perturbSpeedsFile ...
    perturbStatesFile perturbExternalLoadsFile ...
    perturbExternalLoadsModelKinematicsFile };
findAndFillInTags( Perturb_SetupMainBodyNode, desiredTagNames, values );

% Analyze setup file
Analyze_SetupMainBodyNode = Analyze_SetupDomNode.getChildNodes.item(0);
% Name
Analyze_SetupMainBodyNode.getAttributes.item(0).setValue( analyzeName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { analyzeModelFile analyzeActuatorSetFiles analyzeResultsDirectory ...
    analyzeOutputPrecision analyzeInitialTime analyzeFinalTime ...
    analyzeControlsFile analyzeStatesFile analyzeExternalLoadsFile ...
    analyzeExternalLoadsModelKinematicsFile };
findAndFillInTags( Analyze_SetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
Analyze_SetupAnalysisSetNode = Analyze_SetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
Analyze_SetupObjectsNode = Analyze_SetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( Analyze_SetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
Analyze_SetupMuscleAnalysisNode = Analyze_SetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { analyzeMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( Analyze_SetupMuscleAnalysisNode, desiredTagNames, values );

% MuscleAnalysis setup file
MuscleAnalysis_SetupMainBodyNode = MuscleAnalysis_SetupDomNode.getChildNodes.item(0);
% Name
MuscleAnalysis_SetupMainBodyNode.getAttributes.item(0).setValue( muscleAnalysisName );
% Main properties
desiredTagNames = { 'model_file' 'actuator_set_files' 'results_directory' ...
    'output_precision' 'initial_time' 'final_time' 'controls_file' ...
    'states_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { muscleAnalysisModelFile muscleAnalysisActuatorSetFiles muscleAnalysisResultsDirectory ...
    muscleAnalysisOutputPrecision muscleAnalysisInitialTime muscleAnalysisFinalTime ...
    muscleAnalysisControlsFile muscleAnalysisStatesFile muscleAnalysisExternalLoadsFile ...
    muscleAnalysisExternalLoadsModelKinematicsFile };
findAndFillInTags( MuscleAnalysis_SetupMainBodyNode, desiredTagNames, values );
% AnalysisSet
desiredTagNames = { 'AnalysisSet' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SetupMainBodyNode, desiredTagNames, classTagIndices );
% MuscleAnalysis
analysisSetIndex = classTagIndices{1};
MuscleAnalysis_SetupAnalysisSetNode = MuscleAnalysis_SetupMainBodyNode.getChildNodes.item( analysisSetIndex );
desiredTagNames = { 'objects' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SetupAnalysisSetNode, desiredTagNames, classTagIndices );
% Objects tag
objectsIndex = classTagIndices{1};
MuscleAnalysis_SetupObjectsNode = MuscleAnalysis_SetupAnalysisSetNode.getChildNodes.item( objectsIndex );
desiredTagNames = { 'MuscleAnalysis' };
classTagIndices = { -1 };
classTagIndices = findAndFillInTagIndices( MuscleAnalysis_SetupObjectsNode, desiredTagNames, classTagIndices );
% Muscle analysis moment arm coordinate list
muscleAnalysisIndex = classTagIndices{1};
MuscleAnalysis_SetupMuscleAnalysisNode = MuscleAnalysis_SetupObjectsNode.getChildNodes.item( muscleAnalysisIndex );
desiredTagNames = { 'moment_arm_coordinate_list' };
values = { muscleAnalysisMuscleAnalysisMomentArmCoordinateList };
findAndFillInTags( MuscleAnalysis_SetupMuscleAnalysisNode, desiredTagNames, values );

% ID IK setup file
IDIK_SetupMainBodyNode = IDIK_SetupDomNode.getChildNodes.item(0);
% Name
IDIK_SetupMainBodyNode.getAttributes.item(0).setValue( idIkName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idIkModelFile idIkResultsDirectory ...
    idIkInitialTime idIkFinalTime idIkCoordinatesFile idIkExternalLoadsFile ...
    idIkExternalLoadsModelKinematicsFile };
findAndFillInTags( IDIK_SetupMainBodyNode, desiredTagNames, values );

% ID RRA setup file
IDRRA_SetupMainBodyNode = IDRRA_SetupDomNode.getChildNodes.item(0);
% Name
IDRRA_SetupMainBodyNode.getAttributes.item(0).setValue( idRraName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idRraModelFile idRraResultsDirectory ...
    idRraInitialTime idRraFinalTime idRraCoordinatesFile idRraExternalLoadsFile ...
    idRraExternalLoadsModelKinematicsFile };
findAndFillInTags( IDRRA_SetupMainBodyNode, desiredTagNames, values );

% ID CMC setup file
IDCMC_SetupMainBodyNode = IDCMC_SetupDomNode.getChildNodes.item(0);
% Name
IDCMC_SetupMainBodyNode.getAttributes.item(0).setValue( idCmcName );
% Main properties
desiredTagNames = { 'model_file' 'results_directory' ...
    'initial_time' 'final_time' 'coordinates_file' 'external_loads_file' ...
    'external_loads_model_kinematics_file' };
values = { idCmcModelFile idCmcResultsDirectory ...
    idCmcInitialTime idCmcFinalTime idCmcCoordinatesFile idCmcExternalLoadsFile ...
    idCmcExternalLoadsModelKinematicsFile };
findAndFillInTags( IDCMC_SetupMainBodyNode, desiredTagNames, values );

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
Analyze_SetupFile = fullfile( outputDirectory, [trialName '_Setup_Analyze.xml'] );
CMC_ActuatorsFile = fullfile( outputDirectory, cmcActuatorSetFiles );
CMC_ControlConstraintsFile = fullfile( outputDirectory, cmcConstraintsFile );
CMC_SetupFile = fullfile( outputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFile = fullfile( outputDirectory, cmcTaskSetFile );
IDIK_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_IK.xml'] );
IDRRA_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_RRA.xml'] );
IDCMC_SetupFile = fullfile( outputDirectory, [trialName '_Setup_ID_CMC.xml'] );
IK_SetupFile = fullfile( outputDirectory, [trialName '_Setup_IK.xml'] );
IK_TasksFile = fullfile( outputDirectory, ikTaskSetFile );
MuscleAnalysis_SetupFile = fullfile( outputDirectory, [trialName '_Setup_MuscleAnalysis.xml'] );
RRA_ActuatorsFile = fullfile( outputDirectory, rraActuatorSetFiles );
RRA_ControlConstraintsFile = fullfile( outputDirectory, rraConstraintsFile );
RRA_SetupFile = fullfile( outputDirectory, [trialName '_Setup_RRA.xml'] );
RRA_TasksFile = fullfile( outputDirectory, rraTaskSetFile );
Scale_MarkerSetFile = fullfile( outputDirectory, markerSetFile );
Scale_MeasurementSetFile = fullfile( outputDirectory, measurementSetFile );
Scale_ScaleSetFile = fullfile( outputDirectory, scaleSetFile );
Scale_SetupFile = fullfile( outputDirectory, [subjectName '_Setup_Scale.xml'] );
Scale_TasksFile = fullfile( outputDirectory, scaleTaskSetFile );
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
