function cycPercents = convert_timeToCycle(times, limb, tInfo, ictoEvents, analogRate, ref_dataFormat)
% Purpose:  Converts time values to %gait cycle values corresponding to 
%           a subject's 'simulateable' segment, for all FP hits specified 
%           in tInfo.FP.
%
% Input:    times is an array of times values
%           tInfo is a structure containing the following 'trial info',
%             in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           ictoEvents is a structure describing the timing of events
%             associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%               NOTE:  frame numbers in ictoEvents correspond to the 
%                      original C3D file, not the simulateable segment.
%           analogRate is the analog sampling frequency from the C3D file
%
% Output:   cycTimes returns an array of % gait cycle values for limb,  
%               corresponding to the time values in the times array.
%           NOTE:  t = 0 at IC of the 1st FP hit.
%
% CTJ, 08-07, based on
% ASA, 10-05

% Identify FPs hit by the limb of interest, 
% and thus the gait cycles corresponding to the limb of interest.  
fpIndices = strmatch(limb, tInfo.limb);
nCycles = length(fpIndices);

% Note the analog frame number corresponding to IC of the 1st FP hit;
% this is defined to be t = 0 in the 'simulateable' segment.
if ref_dataFormat.tZeroAtFirstIC
	tZeroInFrames = ictoEvents(1).ic; 
else
	% EG: the gcd values lined up with my computed motions better
	%     if I use 1 here rather than the first IC time
	tZeroInFrames = 1;
end

% Determine, in units of time corresponding to the 'simulateable' segment,  
% the time of IC for all gait cycles of interest.
cycleIC = zeros(1,nCycles);
for cycleNum = 1:nCycles
    ic = ictoEvents(fpIndices(cycleNum)).ic;      
    cycleIC(cycleNum) = (ic - tZeroInFrames)/analogRate;
end

% Convert each time value (from times array) to a % gait cycle value
% (to be stored in cycPercents array).
% NOTE: this code can be made more efficient by making use of the fact that
% the times array is monotonically increasing, so a lot of time can be
% saved when trying to determine the previous and next IC times for a given
% element of the times array.
numTimes = length(times);
cycPercents = times;
for j = 1:numTimes
    fpIndexBeforeTimesJ = 1;
    for k = 2:length(cycleIC)
        if cycleIC(k) <= times(j)
            fpIndexBeforeTimesJ = k;
        else
            break;
        end
    end
    previousICTime = cycleIC(fpIndexBeforeTimesJ);
    if fpIndexBeforeTimesJ == length( cycleIC )
        if fpIndexBeforeTimesJ < 2
            error( 'There are not enough gait cycles for estimating cycle duration!' );
        end
        previousPreviousICTime = cycleIC(fpIndexBeforeTimesJ - 1);
        previousGcDuration = previousICTime - previousPreviousICTime;
        % Assume current cycle's duration is same as previous cycle's
        cycleIC(fpIndexBeforeTimesJ + 1) = previousICTime + previousGcDuration;
    end
    nextICTime = cycleIC(fpIndexBeforeTimesJ + 1);
    gaitCycleDuration = nextICTime - previousICTime;
    distanceFromPreviousIC = times(j) - cycleIC(fpIndexBeforeTimesJ);
    previousICPercent = (fpIndexBeforeTimesJ - 1) * 100.0;
    cycPercents(j) = previousICPercent + distanceFromPreviousIC / gaitCycleDuration * 100.0;
end

return;
