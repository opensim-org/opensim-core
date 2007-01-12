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
nRcycles = size(cycTimes.R,2);
nLcycles = size(cycTimes.L,2);
nPts = size(cycTimes.R,1);

% GENERATE TIME ARRAYS
% Generate array for limb that hit first.
if strcmpi(tInfo.limb{1}, 'R')
    emgEnsembleAveTime.R = cycTimes.R(:, 1);    % 1st FP hit by this limb
    if nRcycles > 1                             % additional FP hit
        startIndex = nPts;
        stopIndex = 2*nPts - 1;
        emgEnsembleAveTime.R(startIndex:stopIndex) = cycTimes.R(:, 2);
    end
    first_nCycles = nRcycles;
    first_nPts = length(emgEnsembleAveTime.R);
     
elseif strcmpi(tInfo.limb{1}, 'L')
    emgEnsembleAveTime.L = cycTimes.L(:, 1);    % 1st FP hit by this limb
    if nLcycles > 1                             % additional FP hit
        startIndex = nPts;
        stopIndex = 2*nPts - 1;
        emgEnsembleAveTime.L(startIndex:stopIndex) = cycTimes.L(:, 2);
    end
    first_nCycles = nLcycles;
    first_nPts = length(emgEnsembleAveTime.L);
end

% Generate array for limb that hit second.
if strcmpi(tInfo.limb{2}, 'R')
    emgEnsembleAveTime.R = cycTimes.R(:, 1);    % 1st FP hit by this limb
    if nRcycles > 1                             % additional FP hit
        startIndex = nPts;
        stopIndex = 2*nPts - 1;
        emgEnsembleAveTime.R(startIndex:stopIndex) = cycTimes.R(:, 2);
    end
    appendIndex = ...
        max(find(emgEnsembleAveTime.L <= emgEnsembleAveTime.R(1)));
    emgEnsembleAveTime.R = ...
        [emgEnsembleAveTime.L(1:appendIndex); emgEnsembleAveTime.R];
   second_nCycles = nRcycles;
   second_nPts = length(emgEnsembleAveTime.R);
        
elseif strcmpi(tInfo.limb{2}, 'L')
    emgEnsembleAveTime.L = cycTimes.L(:, 1);    % 1st FP hit
    if nLcycles > 1                             % additional FP hit
        startIndex = nPts;
        stopIndex = 2*nPts - 1;
        emgEnsembleAveTime.L(startIndex:stopIndex) = cycTimes.L(:, 2);
    end
    appendIndex = ...
        max(find(emgEnsembleAveTime.R <= emgEnsembleAveTime.L(1)));
    emgEnsembleAveTime.L = ...
        [emgEnsembleAveTime.R(1:appendIndex); emgEnsembleAveTime.L];
    second_nCycles = nLcycles;
    second_nPts = length(emgEnsembleAveTime.L);
end
    

% % GENERATE EMG ENSEMBLE AVE STRUCTURE
nChannels = length(emgEnsembleAve);
for emgChannel = 1:nChannels       
    
    % Store limb, muscle corresponding to current EMG channel.
    emgEnsembleAveSim(emgChannel).limb = emgEnsembleAve(emgChannel).limb;
    emgEnsembleAveSim(emgChannel).muscle = emgEnsembleAve(emgChannel).muscle;

    % If EMG is from the limb that hit first ...
    if strcmpi(emgEnsembleAve(emgChannel).limb, tInfo.limb{1}) 
        emgEnsembleAveSim(emgChannel).envAve = ...
                                emgEnsembleAve(emgChannel).envAve';
        emgEnsembleAveSim(emgChannel).envSD = ...
                                emgEnsembleAve(emgChannel).envSD';
        if first_nCycles > 1                    
            startIndex = nPts;
            stopIndex = 2*nPts - 1;
            emgEnsembleAveSim(emgChannel).envAve(startIndex:stopIndex) = ...
                                emgEnsembleAve(emgChannel).envAve';
            emgEnsembleAveSim(emgChannel).envSD(startIndex:stopIndex) = ...
                                emgEnsembleAve(emgChannel).envSD';
        end
        
    % Else if EMG is from the limb that hit second ...
    elseif strcmpi(emgEnsembleAve(emgChannel).limb, tInfo.limb{2}) 
        emgEnsembleAveSim(emgChannel).envAve = ...
                                emgEnsembleAve(emgChannel).envAve';
        emgEnsembleAveSim(emgChannel).envSD = ...
                                emgEnsembleAve(emgChannel).envSD';
        if second_nCycles > 1                    
            startIndex = nPts;
            stopIndex = 2*nPts - 1;
            emgEnsembleAveSim(emgChannel).envAve(startIndex:stopIndex) = ...
                                emgEnsembleAve(emgChannel).envAve';
            emgEnsembleAveSim(emgChannel).envSD(startIndex:stopIndex) = ...
                                emgEnsembleAve(emgChannel).envSD';
        end
        
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



