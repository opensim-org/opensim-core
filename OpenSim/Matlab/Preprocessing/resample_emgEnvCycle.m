function emgEnvResamp = resample_emgEnvCycle(emgCyc, cycle)
% Purpose:  Re-samples each cycle of EMG envelope data in emgCyc
%           to have the same # pts, as specified by cycle.
%
% Input:    emgCyc is a structure with the following format,
%               in analog frames:
%               *.R{cycleNum}{emgChannel}
%               *.L{cycleNum}{emgChannel}
%                       .raw, .band, .rect, .low, .nrect, .nlow
%               *.subject - subject number  
%               *.trial   - trial number
%           cycle is an array of % gait cycle values, at which the 
%               EMG envelopes are to be re-sampled.
%
% Output:   emgEnvResamp is a structure with the following format:
%               *.R{cycleNum}{emgChannel}.nlow
%               *.L{cycleNum}{emgChannel}.nlow
%               *.subject   - subject number 
%               *.trial     - trial number
%     
% Called Functions:
%   mirror_array(inArray, mirrorIndex, gapIndex)
%
% ASA, 12-05


% For each channel ...
nChannels = length(emgCyc.R{1});
for channelNum = 1:nChannels
	for limb = ['R' 'L']
		nCycles = length(emgCyc.(limb));
    
		for cycleNum = 1:nCycles   
					
			% Prepare to pad EMG envelope data by mirroring.
			emgEnv = emgCyc.(limb){cycleNum}{channelNum}.nlow;    
			nAnalogFrames = length(emgEnv);
			nMirrorFrames = round(0.25*nAnalogFrames);
			nPadFrames = 2*nMirrorFrames + nAnalogFrames;
			padEnv = zeros(1, nPadFrames);
			padEnv(nMirrorFrames+1 : nMirrorFrames+nAnalogFrames) = emgEnv;
			
			% Pad beginning of emgEnv array.
			mirrorIndex = nMirrorFrames + 1;
			gapIndex = 1;
			padEnv = mirror_array(padEnv, mirrorIndex, gapIndex);
			
			% Pad end of emgEnv array.
			mirrorIndex = nMirrorFrames + nAnalogFrames;
			gapIndex = nPadFrames;
			padEnv = mirror_array(padEnv, mirrorIndex, gapIndex);
			
			% Convert from analog frames to % gait cycle.
			delta = 150/(nPadFrames - 1);
			padCycle = -25:delta:125;
		   
			% Fit spline to padded data.
			cycleMin = padCycle(1);
			cycleMax = padCycle(length(padCycle));
			el = 20;
			k = 5;      % try values between 5-10
			breaks = cycleMin + [0:el]*(cycleMax - cycleMin)/el;
			knots = augknt(breaks, k);
			splEnv = spap2(knots, k, padCycle, padEnv);
			
			% Re-sample at values corresponding to cycle.
			emgEnvResamp.(limb){cycleNum}{channelNum}.nlow = fnval(splEnv, cycle);
		end    
    end
end

emgEnvResamp.subject = emgCyc.subject; 
emgEnvResamp.trial = emgCyc.trial;
return;


