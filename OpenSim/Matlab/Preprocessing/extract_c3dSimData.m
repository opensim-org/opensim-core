function cSim = extract_c3dSimData(c, ictoEvents)
% Purpose:  Extracts a 'simulateable' segment of data from structure c,
%           read from the C3D file of a Gillette subject, and returns
%           a new structure, similar in format to c, but with fewer
%           frames of data. 
%
%           To obtain a 'simulateable' segment, data in c are extracted 
%           starting at the video frame closest to ictoEvents(1).oto, and
%           ending at the video frame closest to ictoEvents(nHits).oic.
%
% Input:    c is a structure returned from read_c3DFile()
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%        
% Output:   cSim is a structure with the following format:  
%               cSim.subject
%               cSim.trial                 .num, .type 
%               cSim.events()              .label, .time
%               cSim.fpInfo()              .corners, .origin
%               cSim.tDS  - gives the time of double support (in seconds) 
%                           between IC and OTO of the 1st FP hit; 
%                 
%               POINT/VIDEO Data:
%               cSim.video         .rate,  .nframes, .units
%               cSim.markers()             .label, .data
%               cSim.jntcenters    .L(),   .label, .data
%                                  .R(),   .label, .data
%               cSim.jntangles     .L(),   .label, .data 
%                                  .R(),   .label, .data
%               cSim.jntmoments    .L(),   .label, .data
%                                  .R(),   .label, .data
%               cSim.jntpowers     .L(),   .label, .data
%                                  .R(),   .label, .data
%
%               ANALOG Data:
%               cSim.analog                .rate,  .ratio
%               cSim.grf()                 .label, .data
%               cSim.emg()                 .label, .data
%
% Notes:   1. Even though the 'simulateable' segment of data begins at SLS, 
%             it may be desireable to define time t = 0 at IC, rather than 
%             OTO, of the 1st FP hit.  The time offset stored in cSim.DS 
%             can be used to do this.  
%
%          2. Event times stored in cSim.events() are shifted from those
%             stored in c.events(), assuming t=0 at IC of the 1st FP hit.
%
%          3. Remember that events in ictoEvents are in analog frames 
%             relative to c, not cSim.  To convert between frame numbers 
%             in c and cSim, note that the first frame in cSim corresponds
%             to the video frame that is closest to the analog frame stored 
%             in ictoEvents(1).oto.
%
%          4. When a segment of data is extracted, the ratio of analog
%             to video frames is maintained.  That is, ALL analog frames
%             (there will be analog.ratio of them) corresponding to the
%             'stop' video frame are extracted.  
%
% ASA, 9-05, rev 10-05


% Determine video and analog frame numbers to start extracting data;
% to do this, find video frame closest to ictoEvents(firstHit).oto,
% and find corresponding analog frame.
firstHit = 1;
start.vframe = round(ictoEvents(firstHit).oto/c.analog.ratio) + 1;
start.aframe = (start.vframe - 1) * c.analog.ratio + 1;

% Determine analog and video frame numbers to stop extracting data;
% to do this, find video frame closest to ictoEvents(lastHit).oic,
% and extract all analog frames prior to the next video frame. 
lastHit = length(ictoEvents);
stop.vframe = round(ictoEvents(lastHit).oic/c.analog.ratio) + 1;
stop.aframe = (stop.vframe - 1) * c.analog.ratio + c.analog.ratio;

% Extract data from c and store in new structure ...
cSim.subject = c.subject;
cSim.trial = c.trial;
cSim.fpInfo = c.fpInfo;

% Get time of double support (in seconds, to nearest video frames) between 
% IC and OTO of the 1st FP hit; this offset can be used to define t=0
% at IC, even though the 'simulateable' segment starts at OTO.
time0.vframe = round(ictoEvents(firstHit).ic/c.analog.ratio) + 1;
time0.vtime = time0.vframe/c.video.rate;
start.vtime = start.vframe/c.video.rate;
cSim.tDS = start.vtime - time0.vtime;

% Update event times stored in c.events (as read from C3D file) such that 
% t=0 at IC of 1st FP hit.
for eventNum = 1:length(c.events)
    cSim.events(eventNum).label = c.events(eventNum).label;
    cSim.events(eventNum).time = c.events(eventNum).time - time0.vtime;
end

% Update number of video frames and Extract POINT/VIDEO data.
cSim.video.rate = c.video.rate;
cSim.video.nframes = stop.vframe - start.vframe + 1;
cSim.video.units = c.video.units;
for mkrNum = 1:length(c.markers)
    cSim.markers(mkrNum).label = c.markers(mkrNum).label;
    cSim.markers(mkrNum).data = ...
        c.markers(mkrNum).data(start.vframe:stop.vframe, :);
end
for jcNum = 1:length(c.jntcenters.R)
    cSim.jntcenters.L(jcNum).label = c.jntcenters.L(jcNum).label;
    cSim.jntcenters.L(jcNum).data = ...
        c.jntcenters.L(jcNum).data(start.vframe:stop.vframe, :);
    cSim.jntcenters.R(jcNum).label = c.jntcenters.R(jcNum).label;
    cSim.jntcenters.R(jcNum).data = ...
        c.jntcenters.R(jcNum).data(start.vframe:stop.vframe, :);
end
for jaNum = 1:length(c.jntangles.R)
    cSim.jntangles.L(jaNum).label = c.jntangles.L(jaNum).label;
    cSim.jntangles.L(jaNum).data = ...
        c.jntangles.L(jaNum).data(start.vframe:stop.vframe, :);
    cSim.jntangles.R(jaNum).label = c.jntangles.R(jaNum).label;
    cSim.jntangles.R(jaNum).data = ...
        c.jntangles.R(jaNum).data(start.vframe:stop.vframe, :);
end
for jmNum = 1:length(c.jntmoments.R)
    cSim.jntmoments.L(jmNum).label = c.jntmoments.L(jmNum).label;
    cSim.jntmoments.L(jmNum).data = ...
        c.jntmoments.L(jmNum).data(start.vframe:stop.vframe, :);
    cSim.jntmoments.R(jmNum).label = c.jntmoments.R(jmNum).label;
    cSim.jntmoments.R(jmNum).data = ...
        c.jntmoments.R(jmNum).data(start.vframe:stop.vframe, :);
end
for jpNum = 1:length(c.jntpowers.R)
    cSim.jntpowers.L(jpNum).label = c.jntmoments.L(jpNum).label;
    cSim.jntpowers.L(jpNum).data = ...
        c.jntpowers.L(jpNum).data(start.vframe:stop.vframe, :);
    cSim.jntpowers.R(jpNum).label = c.jntpowers.R(jpNum).label;
    cSim.jntpowers.R(jpNum).data = ...
        c.jntpowers.R(jpNum).data(start.vframe:stop.vframe, :);
end

% Extract ANALOG data.
cSim.analog = c.analog;
for grfNum = 1:length(c.grf)
    cSim.grf(grfNum).label = c.grf(grfNum).label;
    cSim.grf(grfNum).data = c.grf(grfNum).data(start.aframe:stop.aframe);
end
for emgNum = 1:length(c.emg)
    cSim.emg(emgNum).label = c.emg(emgNum).label;
    cSim.emg(emgNum).data = c.emg(emgNum).data(start.aframe:stop.aframe);
end
return;