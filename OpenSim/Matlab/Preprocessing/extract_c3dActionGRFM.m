function [actionGRFM_FP, fpInfo] = extract_c3dActionGRFM(c, tInfo)
% Purpose:  Unpacks 'reaction' forces and moments (Fx, Fy, Fz, Mx, My, Mz)
%           in the FP coordinate systems from structure c, read from the 
%           C3D file of a Gillette subject, converts these data to 
%           'action' forces and moments, and applies a low-pass filter,
%           for each FP hit specified by tInfo.FP. 
%           
%           Data read from the C3D FORCE_PLATFORM group:
%           (1) 'corners' - the coordinates of each FP corner in the lab CS
%           (2) 'origin'  - the vector from the center of the FP surface to 
%                           the FP origin, in the FP CS
%               are also stored for reference, for each FP that was hit.  
%
% Input:    c is a structure returned from read_c3dFile() or from
%               extract_c3dSimData()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in the order hit (cell array) 
%
% Output:   actionGRFM_FP is a structure that contains the following data,
%             in the FP coordinate systems, in units of N, Nmm or mm, 
%               *(fpHitNum) .Fx(nAnalogFrames)
%                           .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Mx(nAnalogFrames)
%                           .My(nAnalogFrames)
%                           .Mz(nAnalogFrames)
%           fpInfo is a structure that contains the following data:
%               *(fpHitNum) .corners(nCorners, XYZ coordinates, in lab CS)
%                           .origin(vector from center of FP surface to
%                                   FP origin, in the FP CSs)
%
% ASA, 10-05


% Get number of analog frames.
nAnalogFrames = c.video.nframes * c.analog.ratio;

% Get list of labels corresponding to reaction force/moment data in c.
for grfIndex = 1:length(c.grf)
    grfLabels{grfIndex} = c.grf(grfIndex).label;
end

% For each forceplate of interest ...
nHits = length(tInfo.FP);       
for fpHitNum = 1:nHits

    % Get number of FP that was hit.
    fpNumber = tInfo.FP{fpHitNum};
    
    % Get labels and indices corresponding to force and moment data.
    % NOTE:  Gillette C3D files may use either of two different sets of
    %        labels; this function considers both possibilities.
    fxLabel = strcat(num2str(fpNumber), 'FX');
    fyLabel = strcat(num2str(fpNumber), 'FY');
    fzLabel = strcat(num2str(fpNumber), 'FZ');
    mxLabel = strcat(num2str(fpNumber), 'MX');
    myLabel = strcat(num2str(fpNumber), 'MY');
    mzLabel = strcat(num2str(fpNumber), 'MZ');
    
    fxIndex = strmatch(fxLabel, grfLabels);
    if isempty(fxIndex)
        fxLabel = strcat(num2str(fpNumber), 'Fx');
        fxIndex = strmatch(fxLabel, grfLabels);
    end
    fyIndex = strmatch(fyLabel, grfLabels);
    if isempty(fyIndex)
        fyLabel = strcat(num2str(fpNumber), 'Fy');
        fyIndex = strmatch(fyLabel, grfLabels);
    end
    fzIndex = strmatch(fzLabel, grfLabels);
    if isempty(fzIndex)
        fzLabel = strcat(num2str(fpNumber), 'Fz');
        fzIndex = strmatch(fzLabel, grfLabels);
    end
    mxIndex = strmatch(mxLabel, grfLabels);
    if isempty(mxIndex)
        mxLabel = strcat(num2str(fpNumber), 'Mx');
        mxIndex = strmatch(fxLabel, grfLabels);
    end
    myIndex = strmatch(myLabel, grfLabels);
    if isempty(myIndex)
        myLabel = strcat(num2str(fpNumber), 'My');
        myIndex = strmatch(myLabel, grfLabels);
    end
    mzIndex = strmatch(mzLabel, grfLabels);
    if isempty(mzIndex)
        mzLabel = strcat(num2str(fpNumber), 'Mz');
        mzIndex = strmatch(mzLabel, grfLabels);
    end

    % Extract force and moment data from structure c and convert 
    % 'reaction' forces and moments to 'action' forces and moments.
    % NOTE:  The C3D Server returns a cell array;
    %        this array is converted to a numeric array.
    fxTemp = c.grf(fxIndex).data;  
    fyTemp = c.grf(fyIndex).data;
    fzTemp = c.grf(fzIndex).data;
    mxTemp = c.grf(mxIndex).data;
    myTemp = c.grf(myIndex).data;
    mzTemp = c.grf(mzIndex).data;
    for aFrameNum = 1:nAnalogFrames   
        Fx(aFrameNum) = -1 * fxTemp{aFrameNum};
        Fy(aFrameNum) = -1 * fyTemp{aFrameNum};
        Fz(aFrameNum) = -1 * fzTemp{aFrameNum};
        Mx(aFrameNum) = -1 * mxTemp{aFrameNum};
        My(aFrameNum) = -1 * myTemp{aFrameNum};
        Mz(aFrameNum) = -1 * mzTemp{aFrameNum};
    end
    
    % Apply 2nd order, 0-lag, Butterworth filter with cutoff of 20 Hz.
    order = 2;
    cutoff = 20;
    fs = c.analog.rate;
    [b, a] = butter(order, cutoff/(0.5*fs));
    actionGRFM_FP(fpHitNum).Fx = filtfilt(double(b), double(a), double(Fx));  
    actionGRFM_FP(fpHitNum).Fy = filtfilt(double(b), double(a), double(Fy));  
    actionGRFM_FP(fpHitNum).Fz = filtfilt(double(b), double(a), double(Fz));  
    actionGRFM_FP(fpHitNum).Mx = filtfilt(double(b), double(a), double(Mx));  
    actionGRFM_FP(fpHitNum).My = filtfilt(double(b), double(a), double(My));  
    actionGRFM_FP(fpHitNum).Mz = filtfilt(double(b), double(a), double(Mz));  
    
    % Store information about location of FP corners and origin.
    fpInfo(fpHitNum).corners = c.fpInfo(fpNumber).corners;
    fpInfo(fpHitNum).origin = c.fpInfo(fpNumber).origin;       
end
return;
    