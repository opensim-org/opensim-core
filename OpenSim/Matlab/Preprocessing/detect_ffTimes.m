function ffTimes = detect_ffTimes(Z, t, icTimes, threshold)
% Purpose:  Detects FF events from the vertical position of the TOE;
%           FF occurs when the vertical position of the TOE transitions
%           from a positive value in swing to its median value at FF.
%
% Input:    Z is an array of position values for the limb of interest
%           t is an array of times corresponding to Z
%           icTimes is an array of IC event times for the limb of interest
%           threshold is the value of Z at which an event is detected
%
% Output:   ffTimes returns an array of times corresponding to FF events
%
% ASA, 9-05


% Initialize additional variables used for event detection.
cycleFlag = 1;
lrFlag = 0;                 % "loading response" flag
nIC = length(icTimes);      % number of known IC events
nEvents = 1;

% Detect FF events after each known IC event;
% FF occurs when the vertical position of the TOE transitions from
% a positive value in swing to its 'median' value at FF.
for sampleNum = 1:length(t)             % go forward in time    
    if cycleFlag
        if nEvents <= nIC
            if t(sampleNum) > icTimes(nEvents)  
                lrFlag = 1;             % start of loading response
                cycleFlag = 0;
            end
        end
    elseif lrFlag
        if Z(sampleNum) <= threshold
            lrFlag = 0;
            cycleFlag = 1;
            ffTimes(nEvents) = t(sampleNum);
            nEvents = nEvents + 1;
        end
    end
end
return;