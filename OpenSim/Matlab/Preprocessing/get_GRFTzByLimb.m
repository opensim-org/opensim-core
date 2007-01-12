function GRFTz_byLimb = get_GRFTzByLimb(GRFTz_byFP, tInfo);
% Purpose:  Adds arrays of GRFTz data corresponding to each limb 
%           (as stored by FP hit in GRFTz_byFP) into a new set of arrays
%           (as stored by limb in GRFTz_byLimb).
%
% Input:    GRFTz_byFP is a structure with the following format:
%               *(fpHitNum) .Fx(nAnalogFrames)
%                           .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Tz(nAnalogFrames)
%                           .COPx(nAnalogFrames)
%                           .COPy(nAnalogFrames)
%                           .startIndex - analog frame number indicating
%                                         start of COP, Tz data
%                           .stopIndex  - analog frame number indicating
%                                         end of COP, Tz data
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in  the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%
% Output:   GRFTz_byLimb returns a structure with the following format:
%               *.R         .Fx(nAnalogFrames)
%               *.L         .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Tz(nAnalogFrames)
%                           .COPx(nAnalogFrames)
%                           .COPy(nAnalogFrames)
%                           .startIndex - analog frame numbers indicating
%                                         start of COP, Tz data
%                           .stopIndex  - analog frame numbers indicating
%                                         end of COP, Tz data
%
% ASA, 10-05


% Get number of analog frames.
nAnalogFrames = length(GRFTz_byFP(1).Fx);

% Get indices corresponding to R and L FP hits.
rIndices = strmatch('R', tInfo.limb);
lIndices = strmatch('L', tInfo.limb);

% Initialize arrays in output structure.
GRFTz_byLimb.R.Fx = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.Fy = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.Fz = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.Tz = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.COPx = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.COPy = zeros(nAnalogFrames, 1);
GRFTz_byLimb.R.startIndex = zeros(1, length(rIndices));
GRFTz_byLimb.R.stopIndex = zeros(1, length(rIndices));
GRFTz_byLimb.L.Fx = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.Fy = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.Fz = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.Tz = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.COPx = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.COPy = zeros(nAnalogFrames, 1);
GRFTz_byLimb.L.startIndex = zeros(1, length(lIndices));
GRFTz_byLimb.L.stopIndex = zeros(1, length(lIndices));

% Add data for R limb.
for cycleNum = 1:length(rIndices)
    fpHitNum = rIndices(cycleNum);
    GRFTz_byLimb.R.Fx = GRFTz_byLimb.R.Fx + ...
                            GRFTz_byFP(fpHitNum).Fx;
    GRFTz_byLimb.R.Fy = GRFTz_byLimb.R.Fy + ...
                            GRFTz_byFP(fpHitNum).Fy;
    GRFTz_byLimb.R.Fz = GRFTz_byLimb.R.Fz + ...
                            GRFTz_byFP(fpHitNum).Fz;
    GRFTz_byLimb.R.Tz = GRFTz_byLimb.R.Tz + ...
                            GRFTz_byFP(fpHitNum).Tz; 
    GRFTz_byLimb.R.COPx = GRFTz_byLimb.R.COPx + ...
                            GRFTz_byFP(fpHitNum).COPx; 
    GRFTz_byLimb.R.COPy = GRFTz_byLimb.R.COPy + ...
                            GRFTz_byFP(fpHitNum).COPy; 
    GRFTz_byLimb.R.startIndex(cycleNum) = GRFTz_byFP(fpHitNum).startIndex;
    GRFTz_byLimb.R.stopIndex(cycleNum) = GRFTz_byFP(fpHitNum).stopIndex;
end

% Add data for L limb.
for cycleNum = 1:length(lIndices)
    fpHitNum = lIndices(cycleNum);
    GRFTz_byLimb.L.Fx = GRFTz_byLimb.L.Fx + ...
                            GRFTz_byFP(fpHitNum).Fx;
    GRFTz_byLimb.L.Fy = GRFTz_byLimb.L.Fy + ...
                            GRFTz_byFP(fpHitNum).Fy;
    GRFTz_byLimb.L.Fz = GRFTz_byLimb.L.Fz + ...
                            GRFTz_byFP(fpHitNum).Fz;
    GRFTz_byLimb.L.Tz = GRFTz_byLimb.L.Tz + ...
                            GRFTz_byFP(fpHitNum).Tz; 
    GRFTz_byLimb.L.COPx = GRFTz_byLimb.L.COPx + ...
                            GRFTz_byFP(fpHitNum).COPx; 
    GRFTz_byLimb.L.COPy = GRFTz_byLimb.L.COPy + ...
                            GRFTz_byFP(fpHitNum).COPy; 
    GRFTz_byLimb.L.startIndex(cycleNum) = GRFTz_byFP(fpHitNum).startIndex;
    GRFTz_byLimb.L.stopIndex(cycleNum) = GRFTz_byFP(fpHitNum).stopIndex;
end 
return;
   