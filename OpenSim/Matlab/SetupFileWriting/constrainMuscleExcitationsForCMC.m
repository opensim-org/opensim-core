function constrainMuscleExcitationsForCMC( excitationOffRegions, outputDirectory, outputFileName, trialName )
%
% Author: Chand T. John
%

%
% Default values.
%

% Minimum possible excitation value, indicating that an actuator is off.
off = 0.02;

% Maximum possible excitation value, indicating that an actuator is fully
% on.
on = 1.0;

% Extra time cushions on stance phase and swing phase start and end times.
% Positive value extends the stance/swing phase length, while negative
% value shortens the phase interval length.  That is, for example, if
% stance phase start time is 0.5 and the cushion is +0.1, then the final
% stance phase start time will be 0.4, thereby making stance phase 0.1
% seconds longer than it originally was.  If cushion is instead -0.1, then
% the final stance phase start time will be 0.6, thereby making stance
% phase 0.1 seconds shorter than it originally was.  Basically,
%  (final start time) = (original start time) - (start cushion), and
%  (final end   time) = (original end   time) + (end   cushion).
stanceStartCushion = 0.0;
stanceEndCushion = 0.0;
swingStartCushion = 0.0;
swingEndCushion = 0.0;

% Like the above cushions, these describe how much time to delay the start
% of an off constraint for an actuator, and how much earlier to make the
% end time of an off constraint for an actuator, respectively.
offIntervalStartCushion = 0.0;
offIntervalEndCushion = 0.0;

% Phase name constants.
FIRST_PHASE_STANCE = -1;
FIRST_PHASE_SWING  =  0;
STANCE             =  1;
SWING              =  2;
LAST_PHASE_STANCE  =  3;
LAST_PHASE_SWING   =  4;

% Index constants.
% START = 1;
END   = 2;
PHASE = 3;
RIGHT = 1;
LEFT  = 2;

%
% Read in existing CMC control constraints file.
%

CMC_ControlConstraintsFileName = fullfile( outputDirectory, [trialName '_CMC_ControlConstraints.xml'] );
node = xmlread( CMC_ControlConstraintsFileName );
CMC_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_CMC.xml'] );
CMC_SetupDomNode = xmlread( CMC_SetupFileName );

%
% Compute on-off time intervals for all muscles from excitationOffRegions.
%

% Read right and left foot contact data from ground reaction file.
grfFileName = fullfile( outputDirectory, [trialName '_grf.mot'] );
grf = read_motionFile( grfFileName );

% Compute right foot stance and swing start and end times.
timeColumn = find( strcmp( grf.labels, 'time' ) );
time = grf.data( :, timeColumn(1) );
fyIndices = find( strcmp( grf.labels, 'ground_force_vy' ) );
rightFoot = fyIndices(1);
contact = grf.data( :, rightFoot );
if contact(1) == 0
    currentPhase = SWING;
    rightLimbTimes = [ time(1) 0 FIRST_PHASE_SWING ];
else
    currentPhase = STANCE;
    rightLimbTimes = [ time(1) 0 FIRST_PHASE_STANCE ];
end
currentRow = 1;
% Assumption: grf file has alternating groups of 0s and non-zeros in the
% ground_force_fy columns, representing the swing and stance phases,
% respectively.
for i = 1:length( contact )
    currentTimeIsTheStartOfANewPhase = ( currentPhase == STANCE && contact(i) == 0 ||  ...
                                         currentPhase == SWING  && contact(i) ~= 0 );
    if currentTimeIsTheStartOfANewPhase
        if currentPhase == STANCE
            currentPhaseEndCushion = stanceEndCushion;
        else
            currentPhaseEndCushion = swingEndCushion;
        end
        if contact(i) == 0
            newPhase = SWING;
            newPhaseStartCushion = swingStartCushion;
        else
            newPhase = STANCE;
            newPhaseStartCushion = stanceStartCushion;
        end
        rightLimbTimes( currentRow, END ) = time(i) + currentPhaseEndCushion;
        rightLimbTimes( currentRow + 1, : ) = [ time(i) + newPhaseStartCushion 0 newPhase ];
        currentRow = currentRow + 1;
        currentPhase = newPhase;
    end
end
rightLimbTimes( end, END ) = time( end );
if rightLimbTimes( end, PHASE ) == STANCE
    rightLimbTimes( end, PHASE ) = LAST_PHASE_STANCE;
end
if rightLimbTimes( end, PHASE ) == SWING
    rightLimbTimes( end, PHASE ) = LAST_PHASE_SWING;
end

% Compute left foot stance and swing start and end times.
leftFoot = fyIndices(2);
contact = grf.data( :, leftFoot );
if contact(1) == 0
    currentPhase = SWING;
    leftLimbTimes = [ time(1) 0 FIRST_PHASE_SWING ];
else
    currentPhase = STANCE;
    leftLimbTimes = [ time(1) 0 FIRST_PHASE_STANCE ];
end
currentRow = 1;
% Assumption: grf file has alternating groups of 0s and non-zeros in the
% ground_force_fy columns, representing the swing and stance phases,
% respectively.
for i = 1:length( contact )
    currentTimeIsTheStartOfANewPhase = ( currentPhase == STANCE && contact(i) == 0 || ...
                                         currentPhase == SWING  && contact(i) ~= 0 );
    if currentTimeIsTheStartOfANewPhase
        if currentPhase == STANCE
            currentPhaseEndCushion = stanceEndCushion;
        else
            currentPhaseEndCushion = swingEndCushion;
        end
        if contact(i) == 0
            newPhase = SWING;
            newPhaseStartCushion = swingStartCushion;
        else
            newPhase = STANCE;
            newPhaseStartCushion = stanceStartCushion;
        end
        leftLimbTimes( currentRow, END ) = time(i) + currentPhaseEndCushion;
        leftLimbTimes( currentRow + 1, : ) = [ time(i) + newPhaseStartCushion 0 newPhase ];
        currentRow = currentRow + 1;
        currentPhase = newPhase;
    end
end
leftLimbTimes( end, END ) = time( end );
if leftLimbTimes( end, PHASE ) == STANCE
    leftLimbTimes( end, PHASE ) = LAST_PHASE_STANCE;
end
if leftLimbTimes( end, PHASE ) == SWING
    leftLimbTimes( end, PHASE ) = LAST_PHASE_SWING;
end
stanceSwingStartEndTimes = {rightLimbTimes, leftLimbTimes};

% Get names of actuator controls.
actuatorNames = excitationOffRegions( :, 1 );
actuatorNames = strcat( actuatorNames, '.excitation' );

%
% Compute time intervals in which each actuator is constrained to be off.
% Each row represents the values for a different actuator.  The rows are
% ordered to correspond with the order of the actuators in actuatorNames
% and actuatorItemNumbers.
%
% NOTE: [t1 t2 t3 t4] means the actuator should be off during the intervals
% [t1, t2] and [t3, t4], and not off in between t2 and t3, and not off
% before t1 or after t4 (unless t1 or t4 is equal to one of the end times
% of the trial.
%

% Compute gait cycle start/end indices in right and left limbs, based on
% the initial and final times for this trial specified in the CMC setup
% file.
cmcInitialTimeJavaString = CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(23).getChildNodes.item(0).getData();
cmcFinalTimeJavaString = CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(27).getChildNodes.item(0).getData();
cmcInitialTime = str2double( char( cmcInitialTimeJavaString ) );
cmcFinalTime = str2double( char( cmcFinalTimeJavaString ) );
if cmcInitialTime > cmcFinalTime
    error( ['CMC initial time (' num2str( cmcInitialTime ) ') is greater than CMC final time (' num2str( cmcFinalTime ) ')!'] );
end
%rightStanceIndices = 1:2:length( rightLimbTimes( :, 1 ) );
%leftStanceIndices  = 1:2:length(  leftLimbTimes( :, 1 ) );
%rightStanceStartTimes = rightLimbTimes( rightStanceIndices, 1 );
%leftStanceStartTimes  =  leftLimbTimes(  leftStanceIndices, 1 );
%rightIndices = find( cmcInitialTime <= rightStanceStartTimes & rightStanceStartTimes <= cmcFinalTime );
%leftIndices  = find( cmcInitialTime <=  leftStanceStartTimes &  leftStanceStartTimes <= cmcFinalTime );
% Transform indices in rightStanceIndices to indices in rightLimbTimes
%rightIndices = 2 * rightIndices - 1;
% Transform indices in  leftStanceIndices to indices in  leftLimbTimes
%leftIndices  = 2 *  leftIndices - 1;
%stanceStartIndicesWithinCmcTimeRange = {rightIndices, leftIndices};

% Construct cell containing all actuators' off time interval sets.
offTimeIntervalsCell = cell( length( actuatorNames ), 1 );
for i = 1:length( actuatorNames )
    % For actuator i, compute all off intervals and store them in
    % offInterval.

    % Determine whether actuator is for left or right limb.
    actuatorNameWithoutDotExcitation = char( excitationOffRegions( i, 1 ) );
    suffix = actuatorNameWithoutDotExcitation( end - 1 : end );
    if strcmp( suffix, '_r' )
        limbOfActuator = RIGHT;
    else
        if strcmp( suffix, '_l' )
            limbOfActuator = LEFT;
        else
            % If actuator name doesn't end in '_r' or 
            % '_l', then throw an error since we don't know
            % whether this actuator should be constrained based on the
            % right limb or based on the left limb.
            error( ['Does actuator ' actuatorNameWithoutDotExcitation ' use left or right limb? Clarify with suffix of _r or _l.'] );
        end
    end

    % Constrain actuator based on appropriate limb's times.  Be sure to
    % repeat constraint for each gait cycle between cmcInitialTime and
    % cmcFinalTime.
    limbTimes = stanceSwingStartEndTimes{ limbOfActuator };
    offTimeIntervalsCell{i} = computeOffIntervalsForActuator( cmcInitialTime, cmcFinalTime, excitationOffRegions( i, 2 ), limbTimes, offIntervalStartCushion, offIntervalEndCushion );
end

% Create times and values for min and max control nodes for each actuator.
% Each row represents the values for a different actuator.  The rows are
% ordered to correspond with the order of the actuators in actuatorNames
% and actuatorItemNumbers.
timesCell = offTimeIntervalsCell;
minValuesCell = offTimeIntervalsCell;
maxValuesCell = offTimeIntervalsCell;
for i = 1:length( minValuesCell )
    % Set all min values to off.
    minValuesCell{i} = minValuesCell{i} * 0 + off;
    row = maxValuesCell{i};
    for j = 1:2:length( row )
        % For the ((j+1)/2)th interval, set initial maximum to on, and
        % final maximum to off.
        row( j:j+1 ) = [on off];
    end
    maxValuesCell{i} = row;
end

% Sanity check: all arrays in offTimeIntervalsCell should be nondecreasing.
for i = 1:length( offTimeIntervalsCell )
    row = offTimeIntervalsCell{i};
    currentTime = row(1);
    for j = 2:length( row )
        if row(j) < currentTime
            error( ['Off time intervals array for actuator ' actuatorNames{i} ' is not nondecreasing: row(' num2str( j-1 ) ') = ' num2str( currentTime ) ' is greater than row(' num2str( j ) ') = ' num2str( row(j) )] );
        end
        currentTime = row(j);
    end
end

% Add cmcFinalTime to each actuator's timesCell entry, if needed.
% Also do a sanity check.
for i = 1:length( timesCell )
    currentActuatorTimeInterval = timesCell{i};
    %firstTime = currentActuatorTimeInterval(1);
    % Sanity check: firstTime should occur at or after cmcInitialTime.
    %if firstTime < cmcInitialTime
    %    error( ['First off interval start time, ' num2str( firstTime ) ', for actuator ' actuatorNames{i} ' occurs before initial time ' num2str( cmcInitialTime ) ' of trial.'] );
    %end
    lastTime = currentActuatorTimeInterval( end );
    if lastTime < cmcFinalTime
        timesCell{i} = [timesCell{i} cmcFinalTime];
        minValuesCell{i} = [minValuesCell{i} off];
        maxValuesCell{i} = [maxValuesCell{i} on];
    end
end

%
% Add on-off time intervals to CMC control constraints DOM object.
%

% These are currently (OpenSim revision number 2928) the item numbers for
% each of the actuator controls in the CMC control constraints setup file's
% DOM object.
%
% ACTUATOR               ITEM NUMBER
% semimem_r.excitation   59
% semiten_r.excitation   61
% bifemlh_r.excitation   63
% bifemsh_r.excitation   65
% sar_r.excitation       67
% add_mag1_r.excitation  73
% add_mag2_r.excitation  75
% add_mag3_r.excitation  77
% tfl_r.excitation       79
% rect_fem_r.excitation 101
% semimem_l.excitation  145
% semiten_l.excitation  147
% bifemlh_l.excitation  149
% bifemsh_l.excitation  151
% sar_l.excitation      153
% add_mag1_l.excitation 159
% add_mag2_l.excitation 161
% add_mag3_l.excitation 163
% tfl_l.excitation      165
% rect_fem_l.excitation 187
parentNodeOfAllControlLinearTags = node.getChildNodes.item(0).getChildNodes.item(3);
totalNumberOfChildNodes = parentNodeOfAllControlLinearTags.getChildNodes.getLength;
actuatorItemNumbers = zeros( length( actuatorNames ), 1 );
for i = 0:totalNumberOfChildNodes-1
    currentItem = parentNodeOfAllControlLinearTags.getChildNodes.item(i);
    if currentItem.hasAttributes
        nameOfCurrentControlLinear = currentItem.getAttributes.item(0).getValue;
        anyActuatorNamesMatchCurrentItem = strcmp( actuatorNames, nameOfCurrentControlLinear );
        indexOfActuatorInActuatorNames = find( anyActuatorNamesMatchCurrentItem, 1 );
        if ~isempty( indexOfActuatorInActuatorNames )
            indexOfCurrentActuator = indexOfActuatorInActuatorNames(1);
            if length( indexOfActuatorInActuatorNames ) > 1
                error( ['Repeated index of actuator ' actuatorNames( indexOfCurrentActuator ) ' in actuatorNames array.'] );
            end
            actuatorItemNumbers( indexOfCurrentActuator ) = i;
        end
    end
end

% Build and add min and max control nodes to the CMC control constraints
% DOM object.
for actuatorNum = 1:length( actuatorNames )
    min_nodes = node.createElement( 'min_nodes' );
    max_nodes = node.createElement( 'max_nodes' );
    times = timesCell{ actuatorNum };
    minValues = minValuesCell{ actuatorNum };
    maxValues = maxValuesCell{ actuatorNum };
    for timeNum = 1:length( times )
        % Create minimum control nodes
        ControlLinearNode1min = node.createElement( 'ControlLinearNode' );
        timemin = node.createElement( 't' );
        timevalmin = node.createTextNode( num2str( times( timeNum ) ) );
        valuemin = node.createElement( 'value' );
        minValueStr = num2str( minValues( timeNum ) );
        valuevalmin = node.createTextNode( minValueStr );
        valuemin.appendChild( valuevalmin );
        timemin.appendChild( timevalmin );
        ControlLinearNode1min.appendChild( timemin );
        ControlLinearNode1min.appendChild( valuemin );
        min_nodes.appendChild( ControlLinearNode1min );

        % Create maximum control nodes
        ControlLinearNode1max = node.createElement( 'ControlLinearNode' );
        timemax = node.createElement( 't' );
        timevalmax = node.createTextNode( num2str( times( timeNum ) ) );
        valuemax = node.createElement( 'value' );
        maxValueStr = num2str( maxValues( timeNum ) );
        valuevalmax = node.createTextNode( maxValueStr );
        valuemax.appendChild( valuevalmax );
        timemax.appendChild( timevalmax );
        ControlLinearNode1max.appendChild( timemax );
        ControlLinearNode1max.appendChild( valuemax );
        max_nodes.appendChild( ControlLinearNode1max );
    end
    %indexInCellForm = actuatorItemNumbers( actuatorNum, 2 );
    %index = str2double( indexInCellForm{1} );
    index = actuatorItemNumbers( actuatorNum );
    % Add minimum and maximum control nodes to DOM object
    node.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( index ).appendChild( min_nodes );
    node.getChildNodes.item(0).getChildNodes.item(3).getChildNodes.item( index ).appendChild( max_nodes );
end

%
% Write CMC control constraints DOM object to outputFileName.
%

outputFullFilePath = fullfile( outputDirectory, outputFileName );
xmlwrite( outputFullFilePath, node );

%
% Write modified CMC setup file that contains the modified control
% constraints file in the appropriate spot in the setup file.
%

CMC_SetupDomNode.getChildNodes.item(0).getChildNodes.item(63).getChildNodes.item(0).setData( outputFileName );
New_CMC_SetupFileName = fullfile( outputDirectory, [trialName '_Setup_Constrained_CMC.xml'] );
xmlwrite( New_CMC_SetupFileName, CMC_SetupDomNode );
