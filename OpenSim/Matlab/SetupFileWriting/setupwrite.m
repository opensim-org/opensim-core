function setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings )
%
% Write setup files for SimTrack.
%
% Author: Chand T. John
%

%
% 0. Do initial setup steps.
%

% Copy gait2392.osim file from gait2392 example directory to destination
% directory, if this file doesn't already exist in the destination
% directory.
sourceGenericModelFileName = fullfile( gait2392ExampleDirectory, 'gait2392.osim' );
destinationGenericModelFileName = fullfile( outputDirectory, 'gait2392.osim' );
outputDirectoryDoesntExist = ( exist( outputDirectory, 'dir' ) ~= 7 );
if outputDirectoryDoesntExist
    mkdir( outputDirectory );
end
copyfile( sourceGenericModelFileName, destinationGenericModelFileName );

% Set default indices for values in specialSettings cell.
MASS = 1;
HEIGHT = 2;
FEMURSCALEFACTOR = 3;
TIBIASCALEFACTOR = 4;
FYMINCCRRA2 = 5;
FYMAXCCRRA2 = 6;
MXMINCCRRA2 = 7;
MXMAXCCRRA2 = 8;
MYMINCCRRA2 = 9;
MYMAXCCRRA2 = 10;
MZMINCCRRA2 = 11;
MZMAXCCRRA2 = 12;
FXOPTFORCERRA2 = 13;
FYOPTFORCERRA2 = 14;
FZOPTFORCERRA2 = 15;
MXOPTFORCERRA2 = 16;
MYOPTFORCERRA2 = 17;
MZOPTFORCERRA2 = 18;
CMCOPTIMIZERALGORITHM = 19;

%
% 1. Construct full paths for all gait2392 example files from OpenSim.
%

Analyze_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Analyze.xml' );
CMC_ActuatorsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_Actuators.xml' );
CMC_ControlConstraintsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_ControlConstraints.xml' );
CMC_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_CMC.xml' );
CMC_TasksFileName = fullfile( gait2392ExampleDirectory, 'gait2392_CMC_Tasks.xml' );
ID_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_ID.xml' );
IK_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_IK.xml' );
IK_TasksFileName = fullfile( gait2392ExampleDirectory, 'gait2392_IK_Tasks.xml' );
RRA1_ActuatorsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA1_Actuators.xml' );
RRA1_ControlConstraintsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA1_ControlConstraints.xml' );
RRA1_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_RRA1.xml' );
RRA1_TasksFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA1_Tasks.xml' );
RRA2_ActuatorsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA2_Actuators.xml' );
RRA2_ControlConstraintsFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA2_ControlConstraints.xml' );
RRA2_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_RRA2.xml' );
RRA2_TasksFileName = fullfile( gait2392ExampleDirectory, 'gait2392_RRA2_Tasks.xml' );
Scale_MarkerSetFileName = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_MarkerSet.xml' );
Scale_MeasurementSetFileName = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_MeasurementSet.xml' );
Scale_ScaleSetFileName = fullfile( gait2392ExampleDirectory, 'subject01_Scale_ScaleSet.xml' );
Scale_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Scale.xml' );
Scale_TasksFileName = fullfile( gait2392ExampleDirectory, 'gait2392_Scale_Tasks.xml' );
Forward_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Forward.xml' );
Perturb_SetupFileName = fullfile( gait2392ExampleDirectory, 'subject01_Setup_Perturb.xml' );

%
% 2. Read in all relevant gait2392 example setup files from OpenSim.
%

Analyze_SetupDomNode = xmlread( Analyze_SetupFileName );
CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFileName );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFileName );
CMC_SetupDomNode = xmlread( CMC_SetupFileName );
CMC_TasksDomNode = xmlread( CMC_TasksFileName );
ID_SetupDomNode = xmlread( ID_SetupFileName );
IK_SetupDomNode = xmlread( IK_SetupFileName );
IK_TasksDomNode = xmlread( IK_TasksFileName );
RRA1_ActuatorsDomNode = xmlread( RRA1_ActuatorsFileName );
RRA1_ControlConstraintsDomNode = xmlread( RRA1_ControlConstraintsFileName );
RRA1_SetupDomNode = xmlread( RRA1_SetupFileName );
RRA1_TasksDomNode = xmlread( RRA1_TasksFileName );
RRA2_ActuatorsDomNode = xmlread( RRA2_ActuatorsFileName );
RRA2_ControlConstraintsDomNode = xmlread( RRA2_ControlConstraintsFileName );
RRA2_SetupDomNode = xmlread( RRA2_SetupFileName );
RRA2_TasksDomNode = xmlread( RRA2_TasksFileName );
Scale_MarkerSetDomNode = xmlread( Scale_MarkerSetFileName );
Scale_MeasurementSetDomNode = xmlread( Scale_MeasurementSetFileName );
Scale_ScaleSetDomNode = xmlread( Scale_ScaleSetFileName );
Scale_SetupDomNode = xmlread( Scale_SetupFileName );
Scale_TasksDomNode = xmlread( Scale_TasksFileName );
Forward_SetupDomNode = xmlread( Forward_SetupFileName );
Perturb_SetupDomNode = xmlread( Perturb_SetupFileName );

%
% 3. Modify each setup file according to values in big SimTrack setup file.
%

% 3.1 Type all values directly here.

% 3.1.1 Set scale values.

% 3.1.1.1 Set scale setup file values.
scaleName = subjectName;
%mass = '65.9';
%height = '1830.0';
mass = specialSettings{ MASS };
height = specialSettings{ HEIGHT };
notes = 'Age unknown.';
scaleModelFileName = 'gait2392.osim';
markerSetFileName = [subjectName '_Scale_MarkerSet.xml'];
scaleSetFileName = [subjectName '_Scale_ScaleSet.xml'];
measurementSetFileName = [subjectName '_Scale_MeasurementSet.xml'];
scaleMarkerFileName = [subjectName '_static.trc'];
modelScalerTimeRange = '1 2';
preserveMassDistribution = 'true';
scaledOnlyJointFileName = [subjectName '_scaledOnly.jnt'];
scaledOnlyMuscleFileName = [subjectName '_scaledOnly.msl'];
scaledOnlyModelFileName = [subjectName '_scaledOnly.osim'];
scaledOnlyScaleFileName = [subjectName '_scaleSet_applied.xml'];
% Scale setup file node: marker_file := scaleMarkerFileName
scaleTaskSetFileName = [subjectName '_Scale_Tasks.xml'];
coordinateFile = [subjectName '_zeros.mot'];
markerPlacerTimeRange = '1 2';
outputJointFileName = [subjectName '.jnt'];
outputMuscleFileName = [subjectName '.msl'];
subjectSpecificModelFileName = [subjectName '.osim'];
scaleOutputMotionFile = [subjectName '_static_output.mot'];

% 3.1.1.2 Set scale set values.
femurScaleFactor = specialSettings{ FEMURSCALEFACTOR };
tibiaScaleFactor = specialSettings{ TIBIASCALEFACTOR };

% 3.1.2 Set IK values.

% 3.1.2.1 Set IK setup file values.
ikName = subjectName;
% IK setup file node: model_file := subjectSpecificModelFileName
ikTaskSetFileName = [trialName '_IK_Tasks.xml'];
ikMarkerFileName = [trialName '.trc'];
ikCoordinateFileName = [trialName '.mot'];
ikTimeRange = '0.5 4';
desiredMotionFileName = [trialName '_ik.mot'];
% IK setup file node: output_motion_file := desiredMotionFileName

% 3.1.3 Set RRA1 values.

% 3.1.3.1 Set RRA1 setup file values.
rra1Name = [trialName '_RRA1'];
dynamicModelFileName = [subjectName '_sdfast.osim'];
rra1ActuatorSetFileName = [trialName '_RRA1_Actuators.xml'];
rra1ResultsDirectoryName = 'Results/';
rra1InitialTime = '0.5';
rra1FinalTime = '2';
% RRA1 setup file node: desired_kinematics_file := desiredMotionFileName
rra1TaskSetFileName = [trialName '_RRA1_Tasks.xml'];
rra1ConstraintsFileName = [trialName '_RRA1_ControlConstraints.xml'];
externalLoadsFileName = [trialName '_grf.mot'];
% RRA1 setup file node: external_loads_model_kinematics_file := desiredMotionFileName
adjustedModelFileName = [subjectName '_sdfast_adjusted.osim'];

% 3.1.4 Set RRA2 values.

% 3.1.4.1 Set RRA2 setup file values.
rra2Name = [trialName '_RRA2'];
% RRA2 setup file node: model_file := adjustedModelFileName
rra2ActuatorSetFileName = [trialName '_RRA2_Actuators.xml'];
rra2ResultsDirectoryName = 'Results/';
rra2InitialTime = '0.5';
rra2FinalTime = '2';
% RRA2 setup file node: desired_kinematics_file := desiredMotionFileName
rra2TaskSetFileName = [trialName '_RRA2_Tasks.xml'];
rra2ConstraintsFileName = [trialName '_RRA2_ControlConstraints.xml'];
% RRA2 setup file node: external_loads_file := externalLoadsFileName
% RRA2 setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.1.4.2 Set RRA2 control constraint file values.
fyMinControlConstraintRRA2 = specialSettings{ FYMINCCRRA2 };
fyMaxControlConstraintRRA2 = specialSettings{ FYMAXCCRRA2 };
mxMinControlConstraintRRA2 = specialSettings{ MXMINCCRRA2 };
mxMaxControlConstraintRRA2 = specialSettings{ MXMAXCCRRA2 };
myMinControlConstraintRRA2 = specialSettings{ MYMINCCRRA2 };
myMaxControlConstraintRRA2 = specialSettings{ MYMAXCCRRA2 };
mzMinControlConstraintRRA2 = specialSettings{ MZMINCCRRA2 };
mzMaxControlConstraintRRA2 = specialSettings{ MZMAXCCRRA2 };

% 3.1.4.3 Set RRA2 actuator file values.
fxOptimalForceRRA2 = specialSettings{ FXOPTFORCERRA2 };
fyOptimalForceRRA2 = specialSettings{ FYOPTFORCERRA2 };
fzOptimalForceRRA2 = specialSettings{ FZOPTFORCERRA2 };
mxOptimalForceRRA2 = specialSettings{ MXOPTFORCERRA2 };
myOptimalForceRRA2 = specialSettings{ MYOPTFORCERRA2 };
mzOptimalForceRRA2 = specialSettings{ MZOPTFORCERRA2 };

% 3.1.5 Set CMC values.

% 3.1.5.1 Set CMC setup file values.
cmcName = trialName;
% CMC setup file node: model_file := adjustedModelFileName
cmcActuatorSetFileName = [trialName '_CMC_Actuators.xml'];
cmcResultsDirectoryName = './ResultsCMC';
cmcInitialTime = '0.5';
cmcFinalTime = '2';
cmcDesiredKinematicsFileName = fullfile( rra2ResultsDirectoryName, [trialName '_RRA2_Kinematics_q.sto'] );
cmcTaskSetFileName = [trialName '_CMC_Tasks.xml'];
cmcConstraintsFileName = [trialName '_CMC_ControlConstraints.xml'];
% CMC setup file node: external_loads_file := externalLoadsFileName
% CMC setup file node: external_loads_model_kinematics_file := desiredMotionFileName
cmcOptimizerAlgorithm = specialSettings{ CMCOPTIMIZERALGORITHM };

% 3.1.6 Set Forward values.

% 3.1.6.1 Set Forward setup file values.
forwardName = trialName;
% Forward setup file node: model_file := adjustedModelFileName
% Forward setup file node: actuator_set_files := cmcActuatorSetFileName
forwardResultsDirectory = './ResultsForward';
forwardInitialTime = '0.5';
forwardFinalTime = '1.9';
forwardControlsFileName = fullfile( cmcResultsDirectoryName, [cmcName '_controls.xml'] );
forwardInitialStatesFileName = fullfile( cmcResultsDirectoryName, [cmcName '_states.sto'] );
useSpecifiedDt = 'true';
% Forward setup file node: external_loads_file := externalLoadsFileName
% Forward setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.1.7 Set Analyze values.

% 3.1.7.1 Set Analyze setup file values.
analyzeName = trialName;
% Analyze setup file node: model_file := adjustedModelFileName
% Analyze setup file node: actuator_set_files := cmcActuatorSetFileName
analyzeResultsDirectory = './ResultsAnalyze';
analyzeInitialTime = '0.5';
analyzeFinalTime = '2.0';
analyzeControlsFileName = fullfile( cmcResultsDirectoryName, [cmcName '_controls.xml'] );
analyzeInitialStatesFileName = fullfile( cmcResultsDirectoryName, [cmcName '_states.sto'] );
% Analyze setup file node: external_loads_file := externalLoadsFileName
% Analyze setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.1.8 Set ID (Inverse Dynamics) values.

% 3.1.8.1 Set ID (special case of Analyze) setup file values.
idName = trialName;
% ID setup file node: model_file := adjustedModelFileName
% ID setup file node: actuator_set_files := rra1ActuatorSetFileName
idResultsDirectory = './ResultsID';
idInitialTime = '0.5';
idFinalTime = '2.0';
% ID setup file node: coordinates_file := desiredMotionFileName
% ID setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.2 Set these new values in the setup file DOM nodes.
Scale_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( scaleName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( mass );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 7).getChildNodes.item(0).setData( height );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(15).getChildNodes.item(0).setData( notes );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(19).getChildNodes.item(3).getChildNodes.item(0).setData( scaleModelFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(19).getChildNodes.item(7).getChildNodes.item(0).setData( markerSetFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(7).getAttributes.item(0).setValue( scaleSetFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(11).getAttributes.item(0).setValue( measurementSetFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(15).getChildNodes.item(0).setData( scaleMarkerFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(19).getChildNodes.item(0).setData( modelScalerTimeRange );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(23).getChildNodes.item(0).setData( preserveMassDistribution )
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(27).getChildNodes.item(0).setData( scaledOnlyJointFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(31).getChildNodes.item(0).setData( scaledOnlyMuscleFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(35).getChildNodes.item(0).setData( scaledOnlyModelFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(39).getChildNodes.item(0).setData( scaledOnlyScaleFileName );
% Scale setup file node: marker_file := scaleMarkerFileName
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(3).getChildNodes.item(0).setData( scaleMarkerFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(7).getAttributes.item(0).setValue( scaleTaskSetFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(11).getChildNodes.item(0).setData( coordinateFile );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(15).getChildNodes.item(0).setData( markerPlacerTimeRange );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(19).getChildNodes.item(0).setData( outputJointFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(23).getChildNodes.item(0).setData( outputMuscleFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(27).getChildNodes.item(0).setData( subjectSpecificModelFileName );
Scale_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(31).getChildNodes.item(0).setData( scaleOutputMotionFile );

Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(3).getChildNodes.item(1).getChildNodes.item(0).setData( femurScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(5).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );
Scale_ScaleSetDomNode.getChildNodes.item(0).getChildNodes.item(1).getChildNodes.item(7).getChildNodes.item(1).getChildNodes.item(0).setData( tibiaScaleFactor );

IK_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( ikName );
% IK setup file node: model_file := subjectSpecificModelFileName
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( subjectSpecificModelFileName );
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 7).getAttributes.item(0).setValue( ikTaskSetFileName );
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(3).getChildNodes.item(0).setData( ikMarkerFileName );
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(7).getChildNodes.item(0).setData( ikCoordinateFileName );
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(11).getChildNodes.item(0).setData( ikTimeRange );
% IK setup file node: output_motion_file := desiredMotionFileName
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(15).getChildNodes.item(0).setData( desiredMotionFileName );

RRA1_SetupDomNode.getChildNodes.item(0).getAttributes.item(  0).setValue( rra1Name );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item(  5).getChildNodes.item(0).setData( dynamicModelFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 13).getChildNodes.item(0).setData( rra1ActuatorSetFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 17).getChildNodes.item(0).setData( rra1ResultsDirectoryName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 25).getChildNodes.item(0).setData( rra1InitialTime );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 29).getChildNodes.item(0).setData( rra1FinalTime );
% RRA1 setup file node: desired_kinematics_file := desiredMotionFileName
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 49).getChildNodes.item(0).setData( desiredMotionFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 57).getChildNodes.item(0).setData( rra1TaskSetFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 61).getChildNodes.item(0).setData( rra1ConstraintsFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 65).getChildNodes.item(0).setData( externalLoadsFileName );
% RRA1 setup file node: external_loads_model_kinematics_file := desiredMotionFileName
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 69).getChildNodes.item(0).setData( desiredMotionFileName );
RRA1_SetupDomNode.getChildNodes.item(0).getChildNodes.item(117).getChildNodes.item(0).setData( adjustedModelFileName );

RRA2_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( rra2Name );
% RRA2 setup file node: model_file := adjustedModelFileName
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 5).getChildNodes.item(0).setData( adjustedModelFileName );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(13).getChildNodes.item(0).setData( rra2ActuatorSetFileName );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(17).getChildNodes.item(0).setData( rra2ResultsDirectoryName );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(25).getChildNodes.item(0).setData( rra2InitialTime );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(29).getChildNodes.item(0).setData( rra2FinalTime );
% RRA2 setup file node: desired_kinematics_file := desiredMotionFileName
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(49).getChildNodes.item(0).setData( desiredMotionFileName );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(57).getChildNodes.item(0).setData( rra2TaskSetFileName );
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(61).getChildNodes.item(0).setData( rra2ConstraintsFileName );
% RRA2 setup file node: external_loads_file := externalLoadsFileName
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(65).getChildNodes.item(0).setData( externalLoadsFileName );
% RRA2 setup file node: external_loads_model_kinematics_file := desiredMotionFileName
RRA2_SetupDomNode.getChildNodes.item(0).getChildNodes.item(69).getChildNodes.item(0).setData( desiredMotionFileName );

RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fyMinControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(3).getChildNodes.item(0).setData( fyMaxControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( mxMinControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(3).getChildNodes.item(0).setData( mxMaxControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( myMinControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(3).getChildNodes.item(0).setData( myMaxControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( mzMinControlConstraintRRA2 );
RRA2_ControlConstraintsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(3).getChildNodes.item(0).setData( mzMaxControlConstraintRRA2 );

RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 3).getChildNodes.item(1).getChildNodes.item(0).setData( fxOptimalForceRRA2 );
RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 5).getChildNodes.item(1).getChildNodes.item(0).setData( fyOptimalForceRRA2 );
RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 7).getChildNodes.item(1).getChildNodes.item(0).setData( fzOptimalForceRRA2 );
RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( 9).getChildNodes.item(1).getChildNodes.item(0).setData( mxOptimalForceRRA2 );
RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(11).getChildNodes.item(1).getChildNodes.item(0).setData( myOptimalForceRRA2 );
RRA2_ActuatorsDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(13).getChildNodes.item(1).getChildNodes.item(0).setData( mzOptimalForceRRA2 );

CMC_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( cmcName );
% CMC setup file node: model_file := adjustedModelFileName
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( adjustedModelFileName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(0).setData( cmcActuatorSetFileName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(15).getChildNodes.item(0).setData( cmcResultsDirectoryName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(0).setData( cmcInitialTime );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(0).setData( cmcFinalTime );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(51).getChildNodes.item(0).setData( cmcDesiredKinematicsFileName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(59).getChildNodes.item(0).setData( cmcTaskSetFileName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(63).getChildNodes.item(0).setData( cmcConstraintsFileName );
% CMC setup file node: external_loads_file := externalLoadsFileName
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(67).getChildNodes.item(0).setData( externalLoadsFileName );
% RRA2 setup file node: external_loads_model_kinematics_file := desiredMotionFileName
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(71).getChildNodes.item(0).setData( desiredMotionFileName );
CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(123).getChildNodes.item(0).setData( cmcOptimizerAlgorithm );

Forward_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( forwardName );
% Forward setup file node: model_file := adjustedModelFileName
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( adjustedModelFileName );
% Forward setup file node: actuator_set_files := cmcActuatorSetFileName
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(0).setData( cmcActuatorSetFileName );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(15).getChildNodes.item(0).setData( forwardResultsDirectory );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(0).setData( forwardInitialTime );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(0).setData( forwardFinalTime );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(51).getChildNodes.item(0).setData( forwardControlsFileName );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(55).getChildNodes.item(0).setData( forwardInitialStatesFileName );
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(59).getChildNodes.item(0).setData( useSpecifiedDt );
% Forward setup file node: external_loads_file := externalLoadsFileName
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(63).getChildNodes.item(0).setData( externalLoadsFileName );
% Forward setup file node: external_loads_model_kinematics_file := desiredMotionFileName
Forward_SetupDomNode.getChildNodes.item(0).getChildNodes.item(67).getChildNodes.item(0).setData( desiredMotionFileName );

Analyze_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( analyzeName );
% Analyze setup file node: model_file := adjustedModelFileName
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( adjustedModelFileName );
% Analyze setup file node: actuator_set_files := cmcActuatorSetFileName
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(0).setData( cmcActuatorSetFileName );
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(15).getChildNodes.item(0).setData( analyzeResultsDirectory );
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(0).setData( analyzeInitialTime );
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(0).setData( analyzeFinalTime );
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(35).getChildNodes.item(0).setData( analyzeControlsFileName );
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(39).getChildNodes.item(0).setData( analyzeInitialStatesFileName );
% Analyze setup file node: external_loads_file := externalLoadsFileName
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(43).getChildNodes.item(0).setData( externalLoadsFileName );
% Analyze setup file node: external_loads_model_kinematics_file := desiredMotionFileName
Analyze_SetupDomNode.getChildNodes.item(0).getChildNodes.item(47).getChildNodes.item(0).setData( desiredMotionFileName );

ID_SetupDomNode.getChildNodes.item(0).getAttributes.item( 0).setValue( idName );
% ID setup file node: model_file := adjustedModelFileName
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item( 3).getChildNodes.item(0).setData( adjustedModelFileName );
% ID setup file node: actuator_set_files := rra1ActuatorSetFileName
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(11).getChildNodes.item(0).setData( rra1ActuatorSetFileName );
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(15).getChildNodes.item(0).setData( idResultsDirectory );
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(0).setData( idInitialTime );
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(0).setData( idFinalTime );
% ID setup file node: coordinates_file := desiredMotionFileName
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(41).getChildNodes.item(0).setData( desiredMotionFileName );
% ID setup file node: external_loads_model_kinematics_file := desiredMotionFileName
ID_SetupDomNode.getChildNodes.item(0).getChildNodes.item(57).getChildNodes.item(0).setData( desiredMotionFileName );

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
Analyze_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_Analyze.xml'] );
CMC_ActuatorsFileName = fullfile( outputDirectory, cmcActuatorSetFileName );
CMC_ControlConstraintsFileName = fullfile( outputDirectory, cmcConstraintsFileName );
CMC_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFileName = fullfile( outputDirectory, cmcTaskSetFileName );
ID_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_ID.xml'] );
IK_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_IK.xml'] );
IK_TasksFileName = fullfile( outputDirectory, ikTaskSetFileName );
RRA1_ActuatorsFileName = fullfile( outputDirectory, rra1ActuatorSetFileName );
RRA1_ControlConstraintsFileName = fullfile( outputDirectory, rra1ConstraintsFileName );
RRA1_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_RRA1.xml'] );
RRA1_TasksFileName = fullfile( outputDirectory, rra1TaskSetFileName );
RRA2_ActuatorsFileName = fullfile( outputDirectory, rra2ActuatorSetFileName );
RRA2_ControlConstraintsFileName = fullfile( outputDirectory, rra2ConstraintsFileName );
RRA2_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_RRA2.xml'] );
RRA2_TasksFileName = fullfile( outputDirectory, rra2TaskSetFileName );
Scale_MarkerSetFileName = fullfile( outputDirectory, markerSetFileName );
Scale_MeasurementSetFileName = fullfile( outputDirectory, measurementSetFileName );
Scale_ScaleSetFileName = fullfile( outputDirectory, scaleSetFileName );
Scale_SetupFileName = fullfile( outputDirectory, [subjectName '_Setup_Scale.xml'] );
Scale_TasksFileName = fullfile( outputDirectory, scaleTaskSetFileName );
Forward_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_Forward.xml'] );
Perturb_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_Perturb.xml'] );

% 4.2 Write all setup files.
xmlwrite( Analyze_SetupFileName, Analyze_SetupDomNode );
xmlwrite( CMC_ActuatorsFileName, CMC_ActuatorsDomNode );
xmlwrite( CMC_ControlConstraintsFileName, CMC_ControlConstraintsDomNode );
xmlwrite( CMC_SetupFileName, CMC_SetupDomNode );
xmlwrite( CMC_TasksFileName, CMC_TasksDomNode );
xmlwrite( ID_SetupFileName, ID_SetupDomNode );
xmlwrite( IK_SetupFileName, IK_SetupDomNode );
xmlwrite( IK_TasksFileName, IK_TasksDomNode );
xmlwrite( RRA1_ActuatorsFileName, RRA1_ActuatorsDomNode );
xmlwrite( RRA1_ControlConstraintsFileName, RRA1_ControlConstraintsDomNode );
xmlwrite( RRA1_SetupFileName, RRA1_SetupDomNode );
xmlwrite( RRA1_TasksFileName, RRA1_TasksDomNode );
xmlwrite( RRA2_ActuatorsFileName, RRA2_ActuatorsDomNode );
xmlwrite( RRA2_ControlConstraintsFileName, RRA2_ControlConstraintsDomNode );
xmlwrite( RRA2_SetupFileName, RRA2_SetupDomNode );
xmlwrite( RRA2_TasksFileName, RRA2_TasksDomNode );
xmlwrite( Scale_MarkerSetFileName, Scale_MarkerSetDomNode );
xmlwrite( Scale_MeasurementSetFileName, Scale_MeasurementSetDomNode );
xmlwrite( Scale_ScaleSetFileName, Scale_ScaleSetDomNode );
xmlwrite( Scale_SetupFileName, Scale_SetupDomNode );
xmlwrite( Scale_TasksFileName, Scale_TasksDomNode );
xmlwrite( Forward_SetupFileName, Forward_SetupDomNode );
xmlwrite( Perturb_SetupFileName, Perturb_SetupDomNode );
