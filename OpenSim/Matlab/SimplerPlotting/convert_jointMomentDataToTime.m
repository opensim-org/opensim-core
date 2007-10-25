function [timesScaled, jointMomentsScaled] = ...
    convert_jointMomentDataToTime( gaitCycleTimes, jointMomentData, ...
    ictoEvents, analogRate, limb, tInfo, tZeroAtFirstIC )
% Purpose:  Scales literature joint moments given in jointMomentData as a
%           function of gaitCycleTimes (fraction of gait cycle), so that
%           the jointMomentsScaled are given as a function of the time
%           array of a simulation.
%
% Input:    gaitCycleTimes is an array of percents of gait cycle, ranging
%               from 0 to 1 (0% to 100%)
%           jointMomentData is a structure in which each column represents
%               each joint moment as a function of gaitCycleTimes
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%           analogRate is the analog sampling frequency from the C3D file
%           limb represents the limb of interest (either 'L' or 'R')
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           tZeroAtFirstIC is a Boolean that helps determine what frame
%               number to use for the first initial contact event.  In
%               general, this variable should probably be false (0).
%
% Output:   timesScaled is a time array representing the duration of the
%               simulation.
%           jointMomentsScaled is a structure similar to jointMomentData, 
%               but with on/off events in units of the time interval of the
%               simulation. 
%             NOTE: jointMomentsScaled returns the scaled moments for *all*
%                   FP hits of the limb of interest.
%
% CTJ, 8-07, developed based on
% ASA, 9-05


% Identify FPs hit by the limb of interest, 
% and thus the gait cycles corresponding to the limb of interest.  
fpIndices = strmatch(limb, tInfo.limb);
nCycles = length(fpIndices);

% Note the analog frame number corresponding to IC of the 1st FP hit;
% this is defined to be t = 0 in the 'simulateable' segment.
if tZeroAtFirstIC
	tZeroInFrames = ictoEvents(1).ic; 
else
	% EG: the gcd values lined up with my computed motions better
	%     if I use 1 here rather than the first IC time
	tZeroInFrames = 1;
end

% Determine, in units of time corresponding to the 'simulateable' segment,  
% the time of IC, TO, and the duration of the stance and swing phases for
% each gait cycle of interest.  
cycleIC = zeros(nCycles);
cycleTO = zeros(nCycles);
stanceDuration = zeros(nCycles);
swingDuration = zeros(nCycles);
for cycleNum = 1:nCycles
    ic = ictoEvents(fpIndices(cycleNum)).ic;      
    to = ictoEvents(fpIndices(cycleNum)).to;
    icNext = ictoEvents(fpIndices(cycleNum)).icNext;    
    cycleIC(cycleNum) = (ic - tZeroInFrames)/analogRate;
    cycleTO(cycleNum) = (to - tZeroInFrames)/analogRate;
    stanceDuration(cycleNum) = (to - ic)/analogRate; 
    swingDuration(cycleNum) = (icNext - to)/analogRate; 
end

% Get indices of stance times and swing times.
stanceIndices = find(gaitCycleTimes <  0.60);
swingIndices  = find(gaitCycleTimes >= 0.60);
% We assume neither of these arrays is empty.  Thus, do a sanity check.
if isempty(stanceIndices)
    error( 'stanceIndices is empty!' );
end
if isempty(swingIndices)
    error( 'swingIndices is empty!' );
end

% Compute output time array.
timesScaled = [];
jointMomentsScaled = [];
for cycleNum = 1:nCycles
    timesScaled1 = gaitCycleTimes;
    timesScaled1(stanceIndices) = cycleIC(cycleNum) + ...
        gaitCycleTimes(stanceIndices) * ...
        stanceDuration(cycleNum) / 0.60;
    timesScaled1(swingIndices) = cycleTO(cycleNum) + ...
        (gaitCycleTimes(swingIndices) - 0.60) * ...
        swingDuration(cycleNum) / 0.40;
    timesScaled = [timesScaled; timesScaled1];
    jointMomentsScaled = [jointMomentsScaled; jointMomentData];
end

return;
