function icTimes = detect_icTimes(dX, dt, threshold)
% Purpose:  Detects IC events from the fore-aft velocity of the AJC,
%           at transitions from swing to stance.
%
% Input:    dX is an array of velocity values
%           dt is an array of times corresponding to dX
%           threshold is the average dX value at known IC events
%
% Output:   icTimes returns an array of times corresponding to IC events
%
% ASA, 9-05


% Initialize variables used for event detection.
hiThreshold = 0.5*max(dX);      % used to set the stance/swing flags
stanceFlag = 1;
swingFlag = 0;
nEvents = 0;

% Detect IC events, at transitions from swing to stance.
for sampleNum = 1:length(dt)                % go forward in time
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
            icTimes(nEvents) = dt(sampleNum);
        end
    end
end
return;
