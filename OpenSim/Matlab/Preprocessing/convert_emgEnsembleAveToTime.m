function [emgEnsembleAveSim, emgEnsembleAveTime] = ...
    convert_emgEnsembleAveToTime(emgEnsembleAve, cycle, tInfo, ...
                                                ictoEvents, analogRate, ref_dataFormat)
% Purpose:  Converts ensemble-averaged EMG data from % gait cycle to time 
%           corresponding to a subject's 'simulateable' segment,
%           for all FP hits specified in tInfo.FP.
%
% Input:    emgEnsembleAve is a structure with the following format:
%           *(emgChannel).limb 
%                        .muscle 
%                        .envAve
%                        .envSD
%           cycle is an array of % gait cycle values corresponding to
%               emgEnsembleAve
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
% Output:   emgEnsembleAveSim returns a structure in which the
%             ensemble-averaged EMG data are mapped to the subject's
%             simulateable segment:
%               *(emgChannel).limb 
%                            .muscle 
%                            .envAve
%                            .envSD
%           emgEnsembleAveTime returns a structure containing arrays of
%             time values, for each limb, corresponding to the subject's 
%             simulateable segment (but at sample intervals corresponding
%             to emgEnsembleAve:
%               *.R
%               *.L
%             NOTE:  t = 0 at IC of the 1st FP hit.
%
% ASA, 12-05

cycTimes = convert_cycleToTime(cycle, tInfo, ictoEvents, analogRate, ref_dataFormat);
nPts = size(cycTimes.R,1);
maxCycles = 2;

% GENERATE TIME ARRAYS
for fpHit = 1:2
    limb = tInfo.limb{fpHit};
    if ~strcmp(limb,'R') & ~strcmp(limb,'L') error('Expected limb R or L'); end
    nCycles = min(maxCycles,size(cycTimes.(limb),2));

    emgEnsembleAveTime.(limb) = cycTimes.(limb)(:, 1);  % 1st cycle
    for cycle = 2:nCycles
        emgEnsembleAveTime.(limb)(end:(end+nPts-1)) = cycTimes.(limb)(:,cycle);  % the rest of the cycles
    end

    % For the limb that hit the FP second (i.e. later), we add an initial padding to its time
    % vector to get it to start at the same time as the first hit limb.
    if fpHit == 2
        otherlimb=setdiff('LR',limb);
        appendIndex = ...
            max(find(emgEnsembleAveTime.(otherlimb) <= emgEnsembleAveTime.(limb)(1)));
        emgEnsembleAveTime.(limb) = ...
            [emgEnsembleAveTime.(otherlimb)(1:appendIndex); emgEnsembleAveTime.(limb)];
    end
end

% GENERATE EMG ENSEMBLE AVE STRUCTURE
nChannels = length(emgEnsembleAve);
for emgChannel = 1:nChannels       
    
    % Store limb, muscle corresponding to current EMG channel.
    emgEnsembleAveSim(emgChannel).limb = emgEnsembleAve(emgChannel).limb;
    emgEnsembleAveSim(emgChannel).muscle = emgEnsembleAve(emgChannel).muscle;

    limb = emgEnsembleAve(emgChannel).limb;
    if ~strcmp(limb, tInfo.limb{1}) & ~strcmp(limb, tInfo.limb{2}) 
        error('Expected limb R or L');
    end

    nCycles = min(maxCycles,size(cycTimes.(limb),2));
    emgEnsembleAveSim(emgChannel).envAve = emgEnsembleAve(emgChannel).envAve';
    emgEnsembleAveSim(emgChannel).envSD = emgEnsembleAve(emgChannel).envSD';
    for cycle = 2:nCycles
        emgEnsembleAveSim(emgChannel).envAve(end:(end+nPts-1)) = emgEnsembleAve(emgChannel).envAve';
        emgEnsembleAveSim(emgChannel).envSD(end:(end+nPts-1)) = emgEnsembleAve(emgChannel).envSD';
    end

    if strcmp(limb, tInfo.limb{2})
        % Append data before IC of 2nd FP hit.
        appendStart = nPts - appendIndex + 1;
        appendEnd = nPts;
        emgEnsembleAveSim(emgChannel).envAve = ...
            [emgEnsembleAve(emgChannel).envAve(appendStart:appendEnd)';  ...
             emgEnsembleAveSim(emgChannel).envAve];
        emgEnsembleAveSim(emgChannel).envSD = ...
            [emgEnsembleAve(emgChannel).envSD(appendStart:appendEnd)';  ...
             emgEnsembleAveSim(emgChannel).envSD];
    end
end
return;
