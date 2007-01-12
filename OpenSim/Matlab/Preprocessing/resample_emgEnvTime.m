function emgEnsembleAveResamp = ...
   resample_emgEnvTime(emgEnsembleAveSim, emgEnsembleAveTime, aTime)
% Purpose:  Re-samples each channel of ensemble-averaged EMG envelope, SD 
%           in emgEnsembleAveSim at the points specified by aTime.
%
% Input:    emgEnsembleAveSim is a structure with the following format:
%               *(emgChannel).limb 
%                            .muscle 
%                            .envAve
%                            .envSD
%           emgEnsembleAveTime is a structure containing arrays of
%             time values, for each limb, corresponding to 
%             emgEnsembleAveSim:
%               *.R
%               *.L
%           aTime is an array of time values corresponding to the
%               'simulateable' data.
%
% Output:   emgEnsembleAveResamp is a structure with the following format:
%               *(emgChannel).limb 
%                            .muscle 
%                            .envAve
%                            .envSD     
%     
% Called Functions:
%   mirror_array(inArray, mirrorIndex, gapIndex)
%
% ASA, 12-05


% For each channel ...
nChannels = length(emgEnsembleAveSim);
for emgChannel = 1:nChannels

    % If EMG is from the R limb ...
    if strcmpi(emgEnsembleAveSim(emgChannel).limb, 'R') 
        envTime = emgEnsembleAveTime.R;
    elseif strcmpi(emgEnsembleAveSim(emgChannel).limb, 'L') 
        envTime = emgEnsembleAveTime.L;
    end
    
    % Prepare to pad EMG data by mirroring.
    envAve = emgEnsembleAveSim(emgChannel).envAve;  
    envSD = emgEnsembleAveSim(emgChannel).envSD;
    nPts = length(envAve);
    nMirrorPts = round(0.25*nPts);
    nPadPts = 2*nMirrorPts + nPts;
    padAve = zeros(1, nPadPts);
    padAve(nMirrorPts+1 : nMirrorPts+nPts) = envAve;
    padSD = zeros(1, nPadPts);
    padSD(nMirrorPts+1 : nMirrorPts+nPts) = envSD;
    padTime = zeros(1, nPadPts);
    padTime(nMirrorPts+1 : nMirrorPts+nPts) = envTime;
    
    % Pad beginning of arrays.
    mirrorIndex = nMirrorPts + 1;
    gapIndex = 1;
    padAve = mirror_array(padAve, mirrorIndex, gapIndex);
    padSD = mirror_array(padSD, mirrorIndex, gapIndex);
    padTime = mirror_array(padTime, mirrorIndex, gapIndex);

    % Pad end of arrays.
    mirrorIndex = nMirrorPts + nPts;
    gapIndex = nPadPts;
    padAve = mirror_array(padAve, mirrorIndex, gapIndex);
    padSD = mirror_array(padSD, mirrorIndex, gapIndex);
    padTime = mirror_array(padTime, mirrorIndex, gapIndex);

    % Fit splines to padded data.
    cycleMin = padTime(1);
    cycleMax = padTime(length(padTime));
    el = 20;
    k = 5;      % try values between 5-10
    breaks = cycleMin + [0:el]*(cycleMax - cycleMin)/el;
    knots = augknt(breaks, k);
    splAve = spap2(knots, k, padTime, padAve);
    splSD = spap2(knots, k, padTime, padSD);
    
    % Re-sample at values corresponding to aTime.
    emgEnsembleAveResamp(emgChannel).envAve = fnval(splAve, aTime);
    emgEnsembleAveResamp(emgChannel).envSD = fnval(splSD, aTime);
        
    % Store limb, muscle corresponding to current EMG channel.
    emgEnsembleAveResamp(emgChannel).limb = ...
                        emgEnsembleAveSim(emgChannel).limb;
    emgEnsembleAveResamp(emgChannel).muscle = ...
                        emgEnsembleAveSim(emgChannel).muscle;
end
return;