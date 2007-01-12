function perryScaled = convert_perryDataToTime(perryData, ictoEvents, ...
                            analogRate, limb, tInfo, ref_dataFormat)
% Purpose:  Scales EMG on/off times reported by Perry (1992) as a percent 
%           of the stance and swing phases, for the list of muscles in 
%           perryData, to units of time corresponding to a 'simulateable' 
%           segment of data, read from the C3D file of a Gillette control 
%           subject, where t=0 at IC of the 1st FP hit.  
%
% Input:    perryData is a structure with the following format, with
%             EMG on/off times reported as a % of the gait cycle:
%             *(nRefMusles).muscle    - name of muscle
%                          .onoff     - nx2 matrix of [on off] events, 
%                                       where each row is one 'burst'
%                          .onoffAlt  - 'alternate' on/off timings reported
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%           analogRate is the analog sampling frequency from the C3D file
%           limb is the limb ('R' or 'L') corresponding to the current 
%               EMG channel of interest
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%
% Output:   perryScaled is a structure similar to perryData, 
%               but with on/off events in units of time, 
%               where t=0 at IC of the 1st FP hit:
%             *(nRefMusles).muscle    - name of muscle
%                          .onoff     - nx2 matrix of [on off] events, 
%                                       where each row is one 'burst'
%                          .onoffAlt  - 'alternate' on/off timings reported
%             NOTE: perryScaled returns the scaled on/off times for *all* 
%                   FP hits of the specified limb.  
%
% ASA, 9-05


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
% the time of IC, TO, and the duration of the stance and swing phases for
% each gait cycle of interest.  
for cycleNum = 1:nCycles
    ic = ictoEvents(fpIndices(cycleNum)).ic;      
    to = ictoEvents(fpIndices(cycleNum)).to;
    icNext = ictoEvents(fpIndices(cycleNum)).icNext;    
    cycleIC(cycleNum) = (ic - tZeroInFrames)/analogRate;
    cycleTO(cycleNum) = (to - tZeroInFrames)/analogRate;
    stanceDuration(cycleNum) = (to - ic)/analogRate; 
    swingDuration(cycleNum) = (icNext - to)/analogRate; 
end


% For each muscle in the reference list ...
nRefMuscles = length(perryData);
for refNum = 1:nRefMuscles

    % Check whether on/off times from Perry correspond to stance or swing;
    % get row and column indices of stance times and swing times.
    clear onoff onoffAlt;
    clear onoff1 onoffAlt1;
    clear stRow stCol swRow swCol;
    clear stAltRow stAltCol swAltRow swAltCol;
    [stRow, stCol] = find(perryData(refNum).onoff < 60);
    [swRow, swCol] = find(perryData(refNum).onoff >= 60);
    [stAltRow, stAltCol] = find(perryData(refNum).onoffAlt < 60);
    [swAltRow, swAltCol] = find(perryData(refNum).onoffAlt >= 60);

    % Scale Perry's data to gait cycles of interest, in units of time.
	onoff = [];
	onoffAlt = [];
	for i=1:nCycles
        if ~isempty(stRow)
            onoff1(stRow, stCol) = cycleIC(i) + ...
                perryData(refNum).onoff(stRow, stCol) * ...
                stanceDuration(i)/60.0;
        end
        if ~isempty(swRow)
            onoff1(swRow, swCol) = cycleTO(i) + ...
               (perryData(refNum).onoff(swRow, swCol) - 60) * ...
               swingDuration(i)/40.0;
        end
		onoff = [onoff; onoff1];
        
        if ~isempty(perryData(refNum).onoffAlt)
            if ~isempty(stAltRow)
                onoffAlt1(stAltRow, stAltCol) = cycleIC(i) + ...
                  perryData(refNum).onoffAlt(stAltRow, stAltCol) * ...
                  stanceDuration(i)/60.0;
            end
            if ~isempty(swAltRow)
                onoffAlt1(swAltRow, swAltCol) = cycleTO(i) + ...
                  (perryData(refNum).onoffAlt(swAltRow, swAltCol) - 60) * ...
                   swingDuration(i)/40.0;
            end
			onoffAlt = [onoffAlt; onoffAlt1];
        end
    end

    perryScaled(refNum).muscle = perryData(refNum).muscle;
    perryScaled(refNum).onoff = onoff;
    perryScaled(refNum).onoffAlt = onoffAlt;
end
return;
