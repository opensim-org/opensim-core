function setSetupValuesForPercentGaitCycle( initialPercent, finalPercent, inputDirectory, outputDirectory, subjectName, trialName )
%
% Set setup files values for simulating an interval of walking given in
% percents of gait cycle.
%
% Author: Chand T. John
%

%
% 1. Compute initial and final times for the simulation.
%

grfFileName = fullfile( inputDirectory, [trialName '_grf.mot'] );
ictoInput = compute_ictoMatrixInput( grfFileName );
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
trial.mass = 65.9;
trial.trial = 'ss_walking1';
trial.speed = 1.3600;
tInfo.static = {};

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
initialTimeCushion = 0.1;
finalTimeCushion = 0.1;

initialTime = initialTime - initialTimeCushion;
finalTime = finalTime + finalTimeCushion;

initialTimeStr = num2str( initialTime );
finalTimeStr = num2str( finalTime );

%
% 2. Read in source setup files.
%

CMC_ActuatorsFileName = fullfile( inputDirectory, [trialName '_CMC_Actuators.xml'] );
CMC_ControlConstraintsFileName = fullfile( inputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
CMC_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFileName = fullfile( inputDirectory, [trialName '_CMC_Tasks.xml'] );
IK_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_IK.xml'] );
IK_TasksFileName = fullfile( inputDirectory, [trialName '_IK_Tasks.xml'] );
RRA1_ActuatorsFileName = fullfile( inputDirectory, [trialName '_RRA1_Actuators.xml'] );
RRA1_ControlConstraintsFileName = fullfile( inputDirectory, [trialName '_RRA1_ControlConstraints.xml'] );
RRA1_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_RRA1.xml'] );
RRA1_TasksFileName = fullfile( inputDirectory, [trialName '_RRA1_Tasks.xml'] );
RRA2_ActuatorsFileName = fullfile( inputDirectory, [trialName '_RRA2_Actuators.xml'] );
RRA2_ControlConstraintsFileName = fullfile( inputDirectory, [trialName '_RRA2_ControlConstraints.xml'] );
RRA2_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_RRA2.xml'] );
RRA2_TasksFileName = fullfile( inputDirectory, [trialName '_RRA2_Tasks.xml'] );
Scale_MarkerSetFileName = fullfile( inputDirectory, [subjectName '_Scale_MarkerSet.xml'] );
Scale_MeasurementSetFileName = fullfile( inputDirectory, [subjectName '_Scale_MeasurementSet.xml'] );
Scale_ScaleSetFileName = fullfile( inputDirectory, [subjectName '_Scale_ScaleSet.xml'] );
Scale_SetupFileName = fullfile( inputDirectory, [subjectName '_Setup_Scale.xml'] );
Scale_TasksFileName = fullfile( inputDirectory, [subjectName '_Scale_Tasks.xml'] );
Forward_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_Forward.xml'] );
Perturb_SetupFileName = fullfile( inputDirectory, [trialName '_Setup_Perturb.xml'] );

CMC_ActuatorsDomNode = xmlread( CMC_ActuatorsFileName );
CMC_ControlConstraintsDomNode = xmlread( CMC_ControlConstraintsFileName );
CMC_SetupDomNode = xmlread( CMC_SetupFileName );
CMC_TasksDomNode = xmlread( CMC_TasksFileName );
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

% 3.1.1 Set some scale values.

% 3.1.1.1 Set some scale setup file values.
markerSetFileName = [subjectName '_Scale_MarkerSet.xml'];
scaleSetFileName = [subjectName '_Scale_ScaleSet.xml'];
measurementSetFileName = [subjectName '_Scale_MeasurementSet.xml'];
scaleTaskSetFileName = [subjectName '_Scale_Tasks.xml'];
subjectSpecificModelFileName = [subjectName '.osim'];

% 3.1.2 Set IK values.

% 3.1.2.1 Set IK setup file values.
ikName = subjectName;
% IK setup file node: model_file := subjectSpecificModelFileName
ikTaskSetFileName = [trialName '_IK_Tasks.xml'];
ikMarkerFileName = [trialName '.trc'];
ikCoordinateFileName = [trialName '.mot'];
ikTimeRange = [initialTimeStr ' ' finalTimeStr];
desiredMotionFileName = [trialName '_ik.mot'];
% IK setup file node: output_motion_file := desiredMotionFileName

% 3.1.3 Set RRA1 values.

% 3.1.3.1 Set RRA1 setup file values.
rra1Name = [trialName '_RRA1'];
dynamicModelFileName = [subjectName '_sdfast.osim'];
rra1ActuatorSetFileName = [trialName '_RRA1_Actuators.xml'];
rra1ResultsDirectoryName = 'Results/';
rra1InitialTime = initialTimeStr;
rra1FinalTime = finalTimeStr;
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
rra2InitialTime = initialTimeStr;
rra2FinalTime = finalTimeStr;
% RRA2 setup file node: desired_kinematics_file := desiredMotionFileName
rra2TaskSetFileName = [trialName '_RRA2_Tasks.xml'];
rra2ConstraintsFileName = [trialName '_RRA2_ControlConstraints.xml'];
% RRA2 setup file node: external_loads_file := externalLoadsFileName
% RRA2 setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.1.5 Set CMC values.

% 3.1.5.1 Set CMC setup file values.
cmcName = trialName;
% CMC setup file node: model_file := adjustedModelFileName
cmcActuatorSetFileName = [trialName '_CMC_Actuators.xml'];
cmcResultsDirectoryName = './ResultsCMC';
cmcInitialTime = initialTimeStr;
cmcFinalTime = finalTimeStr;
cmcDesiredKinematicsFileName = fullfile( rra2ResultsDirectoryName, [trialName '_RRA2_Kinematics_q.sto'] );
cmcTaskSetFileName = [trialName '_CMC_Tasks.xml'];
cmcConstraintsFileName = [trialName '_CMC_ControlConstraints.xml'];
% CMC setup file node: external_loads_file := externalLoadsFileName
% RRA2 setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.1.5.2 Set CMC control (excitation) constraints file values.


% 3.1.6 Set Forward values.

% 3.1.6.1 Set Forward setup file values.
forwardName = trialName;
% Forward setup file node: model_file := adjustedModelFileName
% Forward setup file node: actuator_set_files := cmcActuatorSetFileName
forwardResultsDirectory = './ResultsForward';
forwardInitialTime = initialTimeStr;
forwardFinalTime = num2str( finalTime - 0.1 );
forwardControlsFileName = fullfile( cmcResultsDirectoryName, [cmcName '_controls.xml'] );
forwardInitialStatesFileName = fullfile( cmcResultsDirectoryName, [cmcName '_states.sto'] );
useSpecifiedDt = 'true';
% Forward setup file node: external_loads_file := externalLoadsFileName
% Forward setup file node: external_loads_model_kinematics_file := desiredMotionFileName

% 3.2 Set values in DOM objects.
IK_SetupDomNode.getChildNodes.item(0).getAttributes.item(0).setValue( ikName );
% IK setup file node: model_file := subjectSpecificModelFileName
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item(0).setData( subjectSpecificModelFileName );
IK_SetupDomNode.getChildNodes.item(0).getChildNodes.item(7).getAttributes.item(0).setValue( ikTaskSetFileName );
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

%
% 4. Write new setup files to outputDirectory.
%

% 4.1 Form full paths for all output setup files.
CMC_ActuatorsFileName = fullfile( outputDirectory, cmcActuatorSetFileName );
CMC_ControlConstraintsFileName = fullfile( outputDirectory, cmcConstraintsFileName );
CMC_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_TasksFileName = fullfile( outputDirectory, cmcTaskSetFileName );
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
xmlwrite( CMC_ActuatorsFileName, CMC_ActuatorsDomNode );
xmlwrite( CMC_ControlConstraintsFileName, CMC_ControlConstraintsDomNode );
xmlwrite( CMC_SetupFileName, CMC_SetupDomNode );
xmlwrite( CMC_TasksFileName, CMC_TasksDomNode );
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

%
% Copy other necessary files to destination directory.
%

outputGrfFileName = fullfile( outputDirectory, [trialName '_grf.mot'] );
copyfile( grfFileName, outputGrfFileName );
