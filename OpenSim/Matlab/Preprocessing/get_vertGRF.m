function [fzR, fzL] = get_vertGRF(c, tInfo)
% Purpose:  Gets vertical GRF data for the trial of interest.
%
% Input:    c is a structure returned from read_c3dFile() or 
%               extract_c3dSimData(), as indicated by inputFlag
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%
% Output:   fzR and fzL each return a matrix(aFrameNum, cycleNum) 
%           of vertical GRFs for the specified limb, sampled at the 
%           analog rate, where each column corresponds to one FP strike.
%
% ASA, 9-05, rev 10-05


% Get number of analog frames.
nAnalogFrames = c.video.nframes * c.analog.ratio;

% Get list of labels corresponding to GRF data in structure c.
for grfIndex = 1:length(c.grf)
    grfLabels{grfIndex} = c.grf(grfIndex).label;
end

% For each forceplate of interest ...
nHits = length(tInfo.FP);       % total number of FP hits
rHits = 0;                      % counter for R FP hits
lHits = 0;                      % counter for L FP hits

for fpHitNum = 1:nHits
    % Get vertical GRF data; flip sign so that vertical force is positive.
    % NOTE:  The C3D Server returns a cell array;
    %        this array is converted to a numeric array.
    fzLabel = strcat(num2str(tInfo.FP{fpHitNum}), 'FZ');
    fzIndex = strmatch(fzLabel, grfLabels);
    temp = c.grf(fzIndex).data;
    if strcmpi(tInfo.limb{fpHitNum}, 'R') 
        rHits = rHits + 1;
        for aFrameNum = 1:nAnalogFrames  
            fzR(aFrameNum, rHits) = -1*temp{aFrameNum};
        end
    elseif strcmpi(tInfo.limb{fpHitNum}, 'L')
        lHits = lHits + 1;
        for aFrameNum = 1:nAnalogFrames  
            fzL(aFrameNum, lHits) = -1*temp{aFrameNum};
        end
    end
end
return;
