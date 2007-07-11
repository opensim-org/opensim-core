function offIntervals = computeOffIntervalsForActuator( cmcInitialTime, cmcFinalTime, offRegion, limbTimes, offIntervalStartCushion, offIntervalEndCushion )

% Phase name constants.
FIRST_PHASE_STANCE = -1;
FIRST_PHASE_SWING  =  0;
STANCE             =  1;
SWING              =  2;
LAST_PHASE_STANCE  =  3;
LAST_PHASE_SWING   =  4;

% Index constants.
START = 1;
END   = 2;
PHASE = 3;

% Extract off time information from offRegion.
offReg = char( offRegion );
constraintStartPhase = offReg( 1:6 );
constraintEndPhase = offReg( 12:17 );
constraintStartPercentage = str2double( offReg( 8:10 ) );
constraintEndPercentage = str2double( offReg( 19:21 ) );
constraintStartTimeAsFractionOfPhaseDuration = constraintStartPercentage / 100.0;
constraintEndTimeAsFractionOfPhaseDuration = constraintEndPercentage / 100.0;

% Make truncated version of limbTimes array that goes only from the phase
% containing cmcInitialTime to the phase containing cmcFinalTime.
if limbTimes( 1, 1 ) > cmcInitialTime
    error( ['Start time ' num2str( limbTimes( 1, 1 ) ) ' occurs after CMC initial time ' num2str( cmcInitialTime ) '!'] );
end
if limbTimes( end, 2 ) < cmcFinalTime
    error( ['End time ' num2str( limbTimes( end, 2 ) ) ' occurs before CMC final time ' num2str( cmcFinalTime ) '!'] );
end
if cmcInitialTime > cmcFinalTime
    error( ['CMC initial time ' num2str( cmcInitialTime ) ' occurs after CMC final time ' num2str( cmcFinalTime ) '!'] );
end
% Now can assume that cmcInitialTime and cmcFinalTime lie within or at the
% boundaries (which we count as "within") of the times in the limbTimes
% array.
rowNumberContainingCmcInitialTime = -1;
rowNumberContainingCmcFinalTime   = -1;
sizeOfLimbTimes = size( limbTimes );
numberOfRowsInLimbTimes = sizeOfLimbTimes(1);
for currentPhaseRowNumberInLimbTimes = 1:numberOfRowsInLimbTimes
    currentPhaseStartTime = limbTimes( currentPhaseRowNumberInLimbTimes, START );
    currentPhaseEndTime   = limbTimes( currentPhaseRowNumberInLimbTimes, END   );
    if currentPhaseStartTime <= cmcInitialTime && cmcInitialTime <= currentPhaseEndTime
        rowNumberContainingCmcInitialTime = currentPhaseRowNumberInLimbTimes;
    end
    if currentPhaseStartTime <= cmcFinalTime && cmcFinalTime <= currentPhaseEndTime
        rowNumberContainingCmcFinalTime = currentPhaseRowNumberInLimbTimes;
    end
end
truncatedLimbTimes = limbTimes( rowNumberContainingCmcInitialTime:rowNumberContainingCmcFinalTime, : );

% Create an array, limbPercents, which is the same size as
% truncatedLimbTimes, but contains percent gait cycle in lieu of "start"
% and "end" times of each phase.
limbPercents = truncatedLimbTimes;
sizeOfLimbPercents = size( limbPercents );
numberOfRowsInLimbPercents = sizeOfLimbPercents(1);
for currentPhaseRowNumberInLimbPercents = 1:numberOfRowsInLimbPercents
    currentPhase = limbPercents( currentPhaseRowNumberInLimbPercents, PHASE );
    % If the current phase is a complete phase from start to end, then the
    % times given in truncatedLimbTimes are in fact the 0% and 100% times
    % of that phase.  So, set start and end percents in limbPercents
    % accordingly.
    if currentPhase == STANCE || currentPhase == SWING
        limbPercents( currentPhaseRowNumberInLimbPercents, START ) = 0;
        limbPercents( currentPhaseRowNumberInLimbPercents, END   ) = 100;
    end
    if currentPhase == FIRST_PHASE_STANCE || currentPhase == FIRST_PHASE_SWING
        % We assume that, since the current phase really is the first phase
        % in the whole trial, that there is another stance and swing phase
        % following the current phase from which we can guess how long the
        % current phase must have been.  We assume the current phase was
        % the same length as the equivalent next occurrence of the same
        % phase.  Well let's just do a sanity check anyway.
        nextSamePhaseRowNumberInLimbPercents = currentPhaseRowNumberInLimbPercents + 2;
        phaseNameOfNextOccurrenceOfSamePhase = limbPercents( nextSamePhaseRowNumberInLimbPercents, PHASE );
        if phaseNameOfNextOccurrenceOfSamePhase == LAST_PHASE_STANCE || phaseNameOfNextOccurrenceOfSamePhase == LAST_PHASE_SWING
            error( 'Not enough complete phases in this trial to estimate lengths of phases at the ends of the motion correctly.' );
        end
        nextSamePhaseLength = limbPercents( nextSamePhaseRowNumberInLimbPercents, END ) - limbPercents( nextSamePhaseRowNumberInLimbPercents, START );
        % If want to estimate the length of the current phase in some other
        % way, change the next line.
        estimateOfCurrentPhaseLength = nextSamePhaseLength;
        currentPhaseStartTime = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, START );
        currentPhaseEndTime   = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, END   );
        currentPhaseRealStartTime = currentPhaseEndTime - estimateOfCurrentPhaseLength;
        currentPhaseStartPercent = ( currentPhaseStartTime - currentPhaseRealStartTime ) / estimateOfCurrentPhaseLength * 100.0;
        limbPercents( currentPhaseRowNumberInLimbPercents, START ) = currentPhaseStartPercent;
        limbPercents( currentPhaseRowNumberInLimbPercents, END   ) = 100;
    end
    if currentPhase == LAST_PHASE_STANCE || currentPhase == LAST_PHASE_SWING
        % We assume that, since the current phase really is the last phase
        % in the whole trial, that there is another stance and swing phase
        % before the current phase from which we can guess how long the
        % current phase must have been.  We assume the current phase was
        % the same length as the equivalent previous occurrence of the same
        % phase.  Well let's just do a sanity check anyway.
        previousSamePhaseRowNumberInLimbPercents = currentPhaseRowNumberInLimbPercents - 2;
        phaseNameOfPreviousOccurrenceOfSamePhase = limbPercents( previousSamePhaseRowNumberInLimbPercents, PHASE );
        if phaseNameOfPreviousOccurrenceOfSamePhase == LAST_PHASE_STANCE || phaseNameOfPreviousOccurrenceOfSamePhase == LAST_PHASE_SWING
            error( 'Not enough complete phases in this trial to estimate lengths of phases at the ends of the motion correctly.' );
        end
        previousSamePhaseLength = limbPercents( previousSamePhaseRowNumberInLimbPercents, END ) - limbPercents( previousSamePhaseRowNumberInLimbPercents, START );
        % If want to estimate the length of the current phase in some other
        % way, change the next line.
        estimateOfCurrentPhaseLength = previousSamePhaseLength;
        currentPhaseStartTime = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, START );
        currentPhaseEndTime   = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, END   );
        currentPhaseRealEndTime = currentPhaseStartTime + estimateOfCurrentPhaseLength;
        currentPhaseEndPercent = ( currentPhaseEndTime - currentPhaseStartTime ) / ( currentPhaseRealEndTime - currentPhaseStartTime ) * 100.0;
        limbPercents( currentPhaseRowNumberInLimbPercents, START ) = 0;
        limbPercents( currentPhaseRowNumberInLimbPercents, END   ) = currentPhaseEndPercent;
    end
end

% Each row of limbPercents corresponds to one phase.  So for each phase,
% i.e. for each row in limbPercents, compute the time of constraint start,
% and the time of constraint end, and store these values into two column
% cell arrays that have the same number of rows as limbPercents.  If there
% is no constraint start or no constraint end in the current phase, put
% 'none' in the corresponding row of the appropriate cell array(s).
constraintIntervals = cell( numberOfRowsInLimbPercents, 2 );
for currentPhaseRowNumberInLimbPercents = 1:numberOfRowsInLimbPercents
    currentPhaseStartPercentage = limbPercents( currentPhaseRowNumberInLimbPercents, START );
    currentPhaseEndPercentage = limbPercents( currentPhaseRowNumberInLimbPercents, END );
    currentPhaseStartTime = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, START );
    currentPhaseEndTime = truncatedLimbTimes( currentPhaseRowNumberInLimbPercents, END );
    currentPhase = limbPercents( currentPhaseRowNumberInLimbPercents, PHASE );
    if currentPhase == STANCE || currentPhase == FIRST_PHASE_STANCE || currentPhase == LAST_PHASE_STANCE
        if strcmp( constraintStartPhase, 'stance' )
            t = ( constraintStartPercentage - currentPhaseStartPercentage ) / ( currentPhaseEndPercentage - currentPhaseStartPercentage );
            constraintStartTime = currentPhaseStartTime + t * ( currentPhaseEndTime - currentPhaseStartTime );
            if currentPhaseStartTime <= constraintStartTime && constraintStartTime <= currentPhaseEndTime
                constraintStartTimeOrNone = num2str( constraintStartTime );
            else
                constraintStartTimeOrNone = 'none';
            end
        else % strcmp( constraintStartPhase, 'swing ' )
            constraintStartTimeOrNone = 'none';
        end
        if strcmp( constraintEndPhase, 'stance' )
            t = ( constraintEndPercentage - currentPhaseStartPercentage ) / ( currentPhaseEndPercentage - currentPhaseStartPercentage );
            constraintEndTime = currentPhaseStartTime + t * ( currentPhaseEndTime - currentPhaseStartTime );
            if currentPhaseStartTime <= constraintEndTime && constraintEndTime <= currentPhaseEndTime
                constraintEndTimeOrNone = num2str( constraintEndTime );
            else
                constraintEndTimeOrNone = 'none';
            end
        else % strcmp( constraintEndPhase, 'swing ' )
            constraintEndTimeOrNone = 'none';
        end
    else % currentPhase is SWING or FIRST_PHASE_SWING or LAST_PHASE_STANCE
        if strcmp( constraintStartPhase, 'stance' )
            constraintStartTimeOrNone = 'none';
        else % strcmp( constraintStartPhase, 'swing ' )
            t = ( constraintStartPercentage - currentPhaseStartPercentage ) / ( currentPhaseEndPercentage - currentPhaseStartPercentage );
            constraintStartTime = currentPhaseStartTime + t * ( currentPhaseEndTime - currentPhaseStartTime );
            if currentPhaseStartTime <= constraintStartTime && constraintStartTime <= currentPhaseEndTime
                constraintStartTimeOrNone = num2str( constraintStartTime );
            else
                constraintStartTimeOrNone = 'none';
            end
        end
        if strcmp( constraintEndPhase, 'stance' )
            constraintEndTimeOrNone = 'none';
        else % strcmp( constraintEndPhase, 'swing ' )
            t = ( constraintEndPercentage - currentPhaseStartPercentage ) / ( currentPhaseEndPercentage - currentPhaseStartPercentage );
            constraintEndTime = currentPhaseStartTime + t * ( currentPhaseEndTime - currentPhaseStartTime );
            if currentPhaseStartTime <= constraintEndTime && constraintEndTime <= currentPhaseEndTime
                constraintEndTimeOrNone = num2str( constraintEndTime );
            else
                constraintEndTimeOrNone = 'none';
            end
        end
    end
    constraintIntervals{ currentPhaseRowNumberInLimbPercents, START } = constraintStartTimeOrNone;
    constraintIntervals{ currentPhaseRowNumberInLimbPercents, END   } = constraintEndTimeOrNone;
end

% Go through each row of the two column cell arrays created above and
% compute the time intervals during which the actuator should be
% constrained to be off.
offIntervals = [];
currentlyLookingForStartTime = true;
currentlyLookingForEndTime = false;
currentStartTime = truncatedLimbTimes( 1, START );
for currentPhaseRowNumberInLimbPercents = 1:numberOfRowsInLimbPercents
    initial = constraintIntervals{ currentPhaseRowNumberInLimbPercents, START };
    final   = constraintIntervals{ currentPhaseRowNumberInLimbPercents, END   };
    initialIsNone = strcmp( initial, 'none' );
    finalIsNone = strcmp( final, 'none' );
    if initialIsNone && finalIsNone
        % Do nothing, and go to next iteration of this loop.
    end
    if ~initialIsNone && finalIsNone
        if currentlyLookingForStartTime
            currentStartTime = str2double( initial );
            offIntervals = [ offIntervals currentStartTime - offIntervalStartCushion ];
            currentlyLookingForStartTime = false;
            currentlyLookingForEndTime = true;
        else % currentlyLookingForEndTime
            error( 'Looking for constraint end time, but found constraint start time!' );
        end
    end
    if initialIsNone && ~finalIsNone
        if currentlyLookingForEndTime
            currentEndTime = str2double( final );
            offIntervals = [ offIntervals currentEndTime + offIntervalEndCushion ];
            currentlyLookingForStartTime = true;
            currentlyLookingForEndTime = false;
        else % currentlyLookingForStartTime
            currentEndTime = str2double( final );
            offIntervals = [ offIntervals currentStartTime - offIntervalStartCushion currentEndTime + offIntervalEndCushion ];
        end
    end
    if ~initialIsNone && ~finalIsNone
        if currentlyLookingForStartTime
            offIntervals = [ offIntervals str2double( initial ) - offIntervalStartCushion str2double( final ) + offIntervalEndCushion ];
        else % currentlyLookingForEndTime
            error( 'Looking for constraint end time, but found constraint start time!' );
        end
    end
end
if currentlyLookingForEndTime
    offIntervals = [ offIntervals truncatedLimbTimes( end, END ) ];
end
