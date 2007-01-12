function toTimes = detect_toTimes(dX, dt, threshold)
% Purpose:  Detects TO events from the fore-aft velocity of the TOE,
%           at transitions from swing to stance.
%
% Input:    dX is an array of velocity values
%           dt is an array of times corresponding to dX
%           threshold is the average dX value at known TO events
%
% Output:   toTimes returns an array of times corresponding to TO events
%
% ASA, 9-05


% Initialize variables used for event detection.
hiThreshold = 0.5*max(dX);      % used to set the stance/swing flags
stanceFlag = 1;
swingFlag = 0;
nEvents = 0;

% Detect TO events, at transitions from swing to stance.
for sampleNum = length(dt): -1 : 1                % go backward in time
    if stanceFlag
        if dX(sampleNum) > hiThreshold
            stanceFlag = 0;
            swingFlag = 1;
        end
    elseif swingFlag
        if dX(sampleNum) <= threshold
            stanceFlag = 1;
            swingFlag = 0;
            nEvents = nEvents + 1;
            toTimes(nEvents) = dt(sampleNum);
        end
    end
end
return;
