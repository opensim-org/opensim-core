function hoTimes = detect_hoTimes(Z, t, toTimes, threshold)
% Purpose:  Detects HO events from the vertical position of the AJC;
%           HO occurs when the vertical position of the AJC transitions
%           from a positive value in swing to its median value at HO.
%
% Input:    Z is an array of position values for the limb of interest
%           t is an array of times corresponding to Z
%           toTimes is an array of TO event times for the limb of interest
%           threshold is the value of Z at which an event is detected
%
% Output:   hoTimes returns an array of times corresponding to HO events
%
% ASA, 9-05


% Initialize additional variables used for event detection.
cycleFlag = 1;
termstanceFlag = 0;                 % "terminal stance" flag
nTO = length(toTimes);              % number of known TO events
for i = 1:nTO                       
    toBackward(i) = toTimes(nTO - i + 1);
end                                 % array of TO events, in reverse order
nEvents = 1;

% Detect HO events prior to each known TO event;
% HO occurs when the vertical position of the AJC transitions from
% a positive value in swing to its median value at HO.
for sampleNum = length(t): -1 : 1           % go backward in time
    if cycleFlag
        if nEvents <= nTO
            if t(sampleNum) < toBackward(nEvents)  
                termstanceFlag = 1;          % terminal stance
                cycleFlag = 0;
            end
        end
    elseif termstanceFlag
        if Z(sampleNum) <= threshold
            termstanceFlag = 0;
            cycleFlag = 1;
            hoTimes(nEvents) = t(sampleNum);
            nEvents = nEvents + 1;
        end
    end
end
return;