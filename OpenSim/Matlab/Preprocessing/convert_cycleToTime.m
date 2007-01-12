function cycTimes = convert_cycleToTime(cycle, tInfo, ictoEvents, analogRate, ref_dataFormat)
% Purpose:  Converts %gait cycle values to time values corresponding to 
%           a subject's 'simulateable' segment, for all FP hits specified 
%           in tInfo.FP.
%
% Input:    cycle is an array of % gait cycle values
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
% Output:   cycTimes returns a matrix of time arrays for each limb,  
%             in the following format:
%               *.R(npts in cycle, ncycles)
%                .L(npts in cycle, ncycles)
%           NOTE:  t = 0 at IC of the 1st FP hit.
%
% ASA, 10-05


% Identify FP hits for each limb.
rIndices = strmatch('R', tInfo.limb);
lIndices = strmatch('L', tInfo.limb);

% Get relevant array lengths.
nPts = length(cycle);
nRcycles = length(rIndices);
nLcycles = length(lIndices);

% Initialize output structure.
cycTimes.R = zeros(nPts, nRcycles);
cycTimes.L = zeros(nPts, nLcycles);

% Note the analog frame number corresponding to IC of the 1st FP hit;
% this is defined to be t = 0 in the 'simulateable' segment.
if ref_dataFormat.tZeroAtFirstIC
	tZeroInFrames = ictoEvents(1).ic; 
else
	% EG: the gcd values lined up with my computed motions better
	%     if I use 1 here rather than the first IC time
	tZeroInFrames = 1;
end

%%% R LIMB CYCLES
% Determine the time of IC and the duration of each R limb cycle,
% in units of time corresponding to the 'simulateable' segment.
for cycleNum = 1:nRcycles
    ic = ictoEvents(rIndices(cycleNum)).ic;           
    icNext = ictoEvents(rIndices(cycleNum)).icNext;    
    cycleIC(cycleNum) = (ic - tZeroInFrames)/analogRate;
    cycleDuration(cycleNum) = (icNext - ic)/analogRate; 
    
    % Get times corresponding to %gait cycle values.
    for ptNum = 1:nPts
        cycTimes.R(ptNum, cycleNum) = ...
            cycleIC(cycleNum) + cycle(ptNum)*cycleDuration(cycleNum)/100.0;
    end
end


%%% L LIMB CYCLES
% Determine the time of IC and the duration of each L limb cycle,
% in units of time corresponding to the 'simulateable' segment.
for cycleNum = 1:nLcycles
    ic = ictoEvents(lIndices(cycleNum)).ic;           
    icNext = ictoEvents(lIndices(cycleNum)).icNext;    
    cycleIC(cycleNum) = (ic - tZeroInFrames)/analogRate;
    cycleDuration(cycleNum) = (icNext - ic)/analogRate; 
    
    % Get times corresponding to %gait cycle values.
    for ptNum = 1:nPts
        cycTimes.L(ptNum, cycleNum) = ...
            cycleIC(cycleNum) + cycle(ptNum)*cycleDuration(cycleNum)/100.0;
    end
end
return;





