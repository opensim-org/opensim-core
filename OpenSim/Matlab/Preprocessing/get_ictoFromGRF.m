function [icFromGRF, toFromGRF, fzData] = get_ictoFromGRF(c, tInfo, ref_dataFormat)
% Purpose:  Detects events corresponding to initial contact and 
%           toe-off from vertical GRF data, stored in structure c, 
%           read from the C3D files of Gillette control subjects.  
%
%           Only forceplates specified in tInfo.FP are analyzed.
%
% Input:    c is a structure returned from read_c3DFile()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in the order hit (cell array) 
%
% Output:   icFromGRF(fpHitNum) and toFromGRF(fpHitNum) return arrays of 
%               IC and TO events, in analog frames, one per FP hit, 
%               in the order specified by tInfo.FP.
%           fzData(aFrameNum, fpHitNum) returns a matrix of  
%               vertical GRF data, one column per FP hit.
%
% ASA, 9-05


% Get relevant dimensions.
nAnalogFrames = c.video.nframes * c.analog.ratio;
nHits = length(tInfo.FP);

% Get list of labels corresponding to GRF data in structure c.
for grfIndex = 1:length(c.grf)
    grfLabels{grfIndex} = c.grf(grfIndex).label;
end

% For each forceplate that was hit...
for fpHitNum = 1:nHits
    
    % Get vertical GRF data; flip sign so that vertical force is positive.
    % NOTE:  The C3D Server returns a cell array;
    %        this array is converted to a numeric array.
    fzLabel = sprintf(ref_dataFormat.c3dAnalogLabels.groundForcePattern{3}, tInfo.FP{fpHitNum});
    fzIndex = strmatch(fzLabel, grfLabels);
    temp = c.grf(fzIndex).data;         
    for aFrameNum = 1:nAnalogFrames     
        fzData(aFrameNum, fpHitNum) = -1*temp{aFrameNum};
    end
    
    % Find indices of FZ greater than a threshold, indicating 'contact';
    %   define threshold = 2% of the max vertical force.
    threshold = 0.02*max(fzData(:, fpHitNum));     % in units of Newtons
    contactIndices = find(fzData(:, fpHitNum) > threshold);
    
    % Make sure that max(contactIndices) represents toe-off, and not an
    %   abberent large Fz value at the end of the trial; to do this,  
    %   define tolerance (# analog frames = time * sampling rate) 
    %   for max # analog frames between IC and TO
    tol = 3.5*c.analog.rate;
    while max(contactIndices) - min(contactIndices) > tol
        contactIndices(length(contactIndices)) = [];
    end
    
    % Get analog frames corresponding to IC and TO events.
    icFromGRF(fpHitNum) = min(contactIndices);
    toFromGRF(fpHitNum) = max(contactIndices) + 1;
    
end
return;
