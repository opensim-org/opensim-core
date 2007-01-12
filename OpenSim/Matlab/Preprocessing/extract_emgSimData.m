function emgSim = extract_emgSimData(proEMG, c, ictoEvents)
% Purpose:  Extracts a 'simulateable' segment of processed EMG data from
%           structure proEMG, and returns a new structure, similar in
%           format to proEMG, but with fewer frames of data. 
%
%           To obtain a 'simulateable' segment, proEMG data are extracted 
%           starting at the video frame closest to ictoEvents(1).oto, and
%           ending at the video frame closest to ictoEvents(nHits).oic.
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
%        
% Output:  emgSim is a structure with the following format,
%           in analog frames:
%               *(emgChannel).raw - raw EMG signal, as read from C3D file
%                            .band  - result of applying band-pass filter
%                            .rect  - result of full-wave rectification
%                            .low   - result of applying low-pass filter
%                            .nrect - result of normalizing rectified data 
%                            .nlow  - result of normalizing filtered data
%
% Notes:   1. Even though the 'simulateable' segment of data begins at SLS, 
%             it may be desireable to define time t = 0 at IC, rather than 
%             OTO, of the 1st FP hit.  The time offset stored in cSim.DS 
%             can be used to do this.  
%
%          2. When a segment of data is extracted, the ratio of analog
%             to video frames is maintained.  That is, ALL analog frames
%             (there will be analog.ratio of them) corresponding to the
%             'stop' video frame are extracted.  
%
% ASA, 9-05, rev 10-05


% Determine video and analog frame numbers to start extracting data;
% to do this, find video frame closest to ictoEvents(firstHit).oto.
firstHit = 1;
start.vframe = round(ictoEvents(firstHit).oto/c.analog.ratio) + 1;
start.aframe = (start.vframe - 1) * c.analog.ratio + 1;

% Determine analog and video frame numbers to stop extracting data;
% to do this, find video frame closest to ictoEvents(lastHit).oic.
lastHit = length(ictoEvents);
stop.vframe = round(ictoEvents(lastHit).oic/c.analog.ratio) + 1;
stop.aframe = (stop.vframe - 1) * c.analog.ratio + c.analog.ratio;

% Extract data from proEMG and store in new structure ...
nChannels = length(proEMG);
for emgChannel = 1:nChannels
    emgSim(emgChannel).raw = ...
        proEMG(emgChannel).raw(start.aframe:stop.aframe);
    emgSim(emgChannel).band = ...
        proEMG(emgChannel).band(start.aframe:stop.aframe);
    emgSim(emgChannel).rect = ...
        proEMG(emgChannel).rect(start.aframe:stop.aframe);
    emgSim(emgChannel).low = ...
        proEMG(emgChannel).low(start.aframe:stop.aframe);
    emgSim(emgChannel).nrect = ...
        proEMG(emgChannel).nrect(start.aframe:stop.aframe);
    emgSim(emgChannel).nlow = ...
        proEMG(emgChannel).nlow(start.aframe:stop.aframe);
end
return;