function emgCyc = extract_emgCycleData(proEMG, c, ictoEvents, tInfo)
% Purpose:  Extracts segments of data from structure proEMG corresponding
%           to R and L gait cycles read from a C3D file and stored in
%           structure c.  A new structure, similar to proEMG, is returned.
%           All cycles (from one IC to the next IC) corresponding to
%           a clean FP hit, as specified in tInfo, are extracted.
%           
% Input:    proEMG(emgChannel) is an array of structures returned from 
%               process_emgChannel(), one structure per EMG channel.
%           c is a structure returned from read_c3DFile()
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.limb  - limbs corresponding to FP strikes (cell array)
%        
% Output:  emgCyc is a structure with the following format,
%               in analog frames:
%               *.R{cycleNum}{emgChannel}
%               *.L{cycleNum}{emgChannel}
%                       .raw, .band, .rect, .low, .nrect, .nlow
%               *.subject - subject number  
%               *.trial   - trial number
%
% Notes:   1. All EMG channels (corresponding to both R and L muscles)
%                  are written to emgCyc.R and emgCyc.L.
%
%          2. When a segment of data is extracted, the ratio of analog
%             to video frames is maintained.  That is, ALL analog frames
%             (there will be analog.ratio of them) corresponding to the
%             'stop' video frame are extracted.  
%
% ASA, 9-05, rev 10-05

for limb = ['R' 'L']
	% Extract emg data for R limb cycles.
	indices = strmatch(limb, tInfo.limb);
	for cycleNum = 1:length(indices)
		
		% Determine video and analog frame numbers to start extracting data;
		% to do this, find video frame closest to IC event.
		fpIndex = indices(cycleNum);
		start.vframe = round(ictoEvents(fpIndex).ic/c.analog.ratio) + 1;
		start.aframe = (start.vframe - 1) * c.analog.ratio + 1;

		% Determine video and analog frame numbers to stop extracting data;
		% to do this, find video frame closest to next IC event.
		stop.vframe = round(ictoEvents(fpIndex).icNext/c.analog.ratio) + 1;
		stop.aframe = (stop.vframe - 1) * c.analog.ratio + c.analog.ratio;
		
		% Extract data from proEMG and store in new structure ...
		nChannels = length(proEMG);
		for emgChannel = 1:nChannels
			emgCyc.(limb){cycleNum}{emgChannel}.raw = ...
				proEMG(emgChannel).raw(start.aframe:stop.aframe);
			emgCyc.(limb){cycleNum}{emgChannel}.band = ...
				proEMG(emgChannel).band(start.aframe:stop.aframe);
			emgCyc.(limb){cycleNum}{emgChannel}.rect = ...
				proEMG(emgChannel).rect(start.aframe:stop.aframe);
			emgCyc.(limb){cycleNum}{emgChannel}.low = ...
				proEMG(emgChannel).low(start.aframe:stop.aframe);
			emgCyc.(limb){cycleNum}{emgChannel}.nrect = ...
				proEMG(emgChannel).nrect(start.aframe:stop.aframe);
			emgCyc.(limb){cycleNum}{emgChannel}.nlow = ...
				proEMG(emgChannel).nlow(start.aframe:stop.aframe);
		end    
	end
end

% Store subject number and trial number for reference.
emgCyc.subject = c.subject;
emgCyc.trial = tInfo.trial;
return;
