function [gcR, gcL] = extract_c3dCycleData(c, ictoEvents, tInfo)
% Purpose:  Extracts cycles of R and L gait data from structure c, 
%           read from the C3D files of Gillette control subjects.  
%           All cycles (from one IC to the next IC) corresponding to
%           a clean FP hit, as specified in tInfo, are extracted.
%           
% Input:    c is a structure returned from read_c3DFile()
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%        
% Output:   gcR and gcL are structures with the following format:
%               *(cycleNum).subject
%               *(cycleNum).trial            .num, .type 
%               *(cycleNum).fpInfo           .corners, .origin
%                 
%               POINT/VIDEO Data:
%               *(cycleNum).video            .rate,  .nframes, .units
%               *(cycleNum).jntcenters()     .label, .data
%               *(cycleNum).jntangles()      .label, .data 
%               *(cycleNum).jntmoments()     .label, .data
%               *(cycleNum).jntpowers()      .label, .data
%
%               ANALOG Data:
%               *(cycleNum).analog            .rate,  .ratio
%               *(cycleNum).grf()             .label, .data
%               *(cycleNum).emg()             .label, .data
%
% Notes:   1. The format of gcR and gcL is NOT the same as c,
%             and does not include all fields that appear in c.
%
%          2. The analog GRF data in gcR and gcL is has not 
%             been processed (i.e., it is 'raw').
%
%          3. When a segment of data is extracted, the ratio of analog
%             to video frames is maintained.  That is, ALL analog frames
%             (there will be analog.ratio of them) corresponding to the
%             'stop' video frame are extracted.  
%
% ASA, 9-05, rev 10-05


% Extract gait cycle data for R limb.
rIndices = strmatch('R', tInfo.limb);
for cycleNum = 1:length(rIndices)
    
    % Determine video and analog frame numbers to start extracting data;
    % to do this, find video frame closest to IC event.
    fpIndex = rIndices(cycleNum);
    start.vframe = round(ictoEvents(fpIndex).ic/c.analog.ratio) + 1;
    start.aframe = (start.vframe - 1) * c.analog.ratio + 1;

    % Determine video and analog frame numbers to stop extracting data;
    % to do this, find video frame closest to next IC event.
    stop.vframe = round(ictoEvents(fpIndex).icNext/c.analog.ratio) + 1;
    stop.aframe = (stop.vframe -1) * c.analog.ratio + c.analog.ratio;
    
    % Extract and store trial data.
    gcR(cycleNum).subject = c.subject;
    gcR(cycleNum).trial = c.trial;
    gcR(cycleNum).fpInfo = c.fpInfo(rIndices(cycleNum));
    
    % Update number of video frames and Extract POINT/VIDEO data.
    gcR(cycleNum).video.rate = c.video.rate;
    gcR(cycleNum).video.nframes = stop.vframe - start.vframe + 1;
    gcR(cycleNum).video.units = c.video.units;
    for jcNum = 1:length(c.jntcenters.R)
       gcR(cycleNum).jntcenters(jcNum).label = c.jntcenters.R(jcNum).label;
       gcR(cycleNum).jntcenters(jcNum).data = ...
            c.jntcenters.R(jcNum).data(start.vframe:stop.vframe, :);
    end
    for jaNum = 1:length(c.jntangles.R)
       gcR(cycleNum).jntangles(jaNum).label = c.jntangles.R(jaNum).label;
       gcR(cycleNum).jntangles(jaNum).data = ...
            c.jntangles.R(jaNum).data(start.vframe:stop.vframe, :);
    end
    for jmNum = 1:length(c.jntmoments.R)
       gcR(cycleNum).jntmoments(jmNum).label = c.jntmoments.R(jmNum).label;
       gcR(cycleNum).jntmoments(jmNum).data = ...
            c.jntmoments.R(jmNum).data(start.vframe:stop.vframe, :);
    end
    for jpNum = 1:length(c.jntpowers.R)
       gcR(cycleNum).jntpowers(jpNum).label = c.jntpowers.R(jpNum).label;
       gcR(cycleNum).jntpowers(jpNum).data = ...
            c.jntpowers.R(jpNum).data(start.vframe:stop.vframe, :);
    end

     % Extract ANALOG data.
    gcR(cycleNum).analog = c.analog;
    for grfNum = 1:length(c.grf)
        gcR(cycleNum).grf(grfNum).label = c.grf(grfNum).label;
        gcR(cycleNum).grf(grfNum).data = ...
            c.grf(grfNum).data(start.aframe:stop.aframe);
    end
    for emgNum = 1:length(c.emg)
        gcR(cycleNum).emg(emgNum).label = c.emg(emgNum).label;
        gcR(cycleNum).emg(emgNum).data = ...
            c.emg(emgNum).data(start.aframe:stop.aframe);
    end 
end


% Extract gait cycle data for L limb.
lIndices = strmatch('L', tInfo.limb);
for cycleNum = 1:length(lIndices)
    
    % Determine video and analog frame numbers to start extracting data;
    % to do this, find video frame closest to IC event.
    fpIndex = lIndices(cycleNum);
    start.vframe = round(ictoEvents(fpIndex).ic/c.analog.ratio) + 1;
    start.aframe = (start.vframe - 1) * c.analog.ratio + 1;

    % Determine video and analog frame numbers to stop extracting data;
    % to do this, find video frame closest to next IC event.
    stop.vframe = round(ictoEvents(fpIndex).icNext/c.analog.ratio) + 1;
    stop.aframe = (stop.vframe -1) * c.analog.ratio + c.analog.ratio;
    
    % Extract and store trial data.
    gcL(cycleNum).subject = c.subject;
    gcL(cycleNum).trial = c.trial;
    gcL(cycleNum).fpInfo = c.fpInfo(lIndices(cycleNum));

    % Update number of video frames and Extract POINT/VIDEO data.
    gcL(cycleNum).video.rate = c.video.rate;
    gcL(cycleNum).video.nframes = stop.vframe - start.vframe + 1;
    gcL(cycleNum).video.units = c.video.units;
    for jcNum = 1:length(c.jntcenters.L)
       gcL(cycleNum).jntcenters(jcNum).label = c.jntcenters.L(jcNum).label;
       gcL(cycleNum).jntcenters(jcNum).data = ...
            c.jntcenters.L(jcNum).data(start.vframe:stop.vframe, :);
    end
    for jaNum = 1:length(c.jntangles.L)
       gcL(cycleNum).jntangles(jaNum).label = c.jntangles.L(jaNum).label;
       gcL(cycleNum).jntangles(jaNum).data = ...
            c.jntangles.L(jaNum).data(start.vframe:stop.vframe, :);
    end
    for jmNum = 1:length(c.jntmoments.L)
       gcL(cycleNum).jntmoments(jmNum).label = c.jntmoments.L(jmNum).label;
       gcL(cycleNum).jntmoments(jmNum).data = ...
            c.jntmoments.L(jmNum).data(start.vframe:stop.vframe, :);
    end
    for jpNum = 1:length(c.jntpowers.L)
       gcL(cycleNum).jntpowers(jpNum).label = c.jntpowers.L(jpNum).label;
       gcL(cycleNum).jntpowers(jpNum).data = ...
            c.jntpowers.L(jpNum).data(start.vframe:stop.vframe, :);
    end

     % Extract ANALOG data.
    gcL(cycleNum).analog = c.analog;
    for grfNum = 1:length(c.grf)
        gcL(cycleNum).grf(grfNum).label = c.grf(grfNum).label;
        gcL(cycleNum).grf(grfNum).data = ...
            c.grf(grfNum).data(start.aframe:stop.aframe);
    end
    for emgNum = 1:length(c.emg)
        gcL(cycleNum).emg(emgNum).label = c.emg(emgNum).label;
        gcL(cycleNum).emg(emgNum).data = ...
            c.emg(emgNum).data(start.aframe:stop.aframe);
    end 
end
return;