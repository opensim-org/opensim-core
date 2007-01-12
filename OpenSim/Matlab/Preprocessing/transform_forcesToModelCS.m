function forces = transform_forcesToModelCS(GRFTz_byLimb, COP_smoothed, ...
                                                              cSim, tInfo)
% Purpose:  Performs the following computations:
%           (1) transforms GRFs to a coordinate system consistent with
%               Darryl's model
%           (2) transforms vertical torques to a coordinate system 
%               and units consistent with Darryl's model
%           (3) transforms COP values to a coordinate system 
%               consistent with Darryl's model
%           (4) re-samples the data at the video frame rate
%
% Input:    GRFTz_byLimb is a structure with the following format:
%               *.R         .Fx(nAnalogFrames of 'simulateable' segment)
%               *.L         .Fy(")
%                           .Fz(")
%                           .Tz(")
%                           .COPx(")
%                           .COPy(")
%                           .startIndex - analog frame numbers indicating
%                                         start of COP, Tz data
%                           .stopIndex  - analog frame numbers indicating
%                                         end of COP, Tz data
%           COP_smoothed is a structure with the following format:
%               *.R         .COPx("), discontinuities removed
%               *.L         .COPy("), discontinuities removed  
%           cSim is a structure that contains the following data, 
%             in addition to other information:
%                   *.video        .rate,  .nframes,    .units
%                   *.analog       .rate,  .ratio
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in the order hit (cell array) 
%
% Output:   forces returns a structure containing the transformed 
%             force data, with the following format:
%               *.dataName(nVideoFrames), where dataName corresponds to
%                the force components rGRF, lGRF, rCOP, lCOP, rT, lT
%   
% Notes:    Gillette Lab CS (if walking in +X) has X anterior, Y left, Z up
%           Darryl's Model CS has X anterior, Y up, Z right
%
% ASA, 10-05


% Get number of analog frames.
nAnalogFrames = cSim.video.nframes * cSim.analog.ratio;

% Get walking direction.
if tInfo.FP{1} < tInfo.FP{2}        % walked in +x direction
     walkDir = 'posX';
else
     walkDir = 'negX';              % walked in -x direction
end

% Convert data to be consistent with model CS.
if strcmpi(walkDir, 'posX')
    forces.rGRF = [   GRFTz_byLimb.R.Fx ...
                      GRFTz_byLimb.R.Fz ...
                   -1*GRFTz_byLimb.R.Fy];
               
    forces.lGRF = [   GRFTz_byLimb.L.Fx ...
                      GRFTz_byLimb.L.Fz ...
                   -1*GRFTz_byLimb.L.Fy];
               
    forces.rT =   [zeros(nAnalogFrames, 1) ...
                   GRFTz_byLimb.R.Tz      ...
                   zeros(nAnalogFrames, 1)];
               
    forces.lT =   [zeros(nAnalogFrames, 1) ...
                   GRFTz_byLimb.L.Tz      ...
                   zeros(nAnalogFrames, 1)];
               
    forces.rCOP = [  COP_smoothed.R.COPx    ...
                     zeros(nAnalogFrames, 1) ...
                   -1*COP_smoothed.R.COPy];
    
    forces.lCOP = [  COP_smoothed.L.COPx    ...
                     zeros(nAnalogFrames, 1) ...
                   -1*COP_smoothed.L.COPy];
    
elseif strcmpi(walkDir, 'negX')
    forces.rGRF = [-1*GRFTz_byLimb.R.Fx ...
                      GRFTz_byLimb.R.Fz ...
                      GRFTz_byLimb.R.Fy];
               
    forces.lGRF = [-1*GRFTz_byLimb.L.Fx ...
                      GRFTz_byLimb.L.Fz ...
                      GRFTz_byLimb.L.Fy];
               
    forces.rT =   [zeros(nAnalogFrames, 1) ...
                   GRFTz_byLimb.R.Tz      ...
                   zeros(nAnalogFrames, 1)];
               
    forces.lT =   [zeros(nAnalogFrames, 1) ...
                   GRFTz_byLimb.L.Tz      ...
                   zeros(nAnalogFrames, 1)];
               
    forces.rCOP = [-1*COP_smoothed.R.COPx    ...
                      zeros(nAnalogFrames, 1) ...
                      COP_smoothed.R.COPy];
    
    forces.lCOP = [-1*COP_smoothed.L.COPx    ...
                      zeros(nAnalogFrames, 1) ...
                      COP_smoothed.L.COPy];    
end
 
% Convert units to be consistent with model (Nm, m).
if strcmpi(cSim.video.units, 'mm')
    forces.rT = forces.rT/1000.0;
    forces.lT = forces.lT/1000.0;
    forces.rCOP = forces.rCOP/1000.0;
    forces.lCOP = forces.lCOP/1000.0;
end

% Re-sample data at video frame rate.
videoIndices = 1 : cSim.analog.ratio : nAnalogFrames;
forces.rGRF = forces.rGRF(videoIndices, :);
forces.lGRF = forces.lGRF(videoIndices, :);
forces.rT = forces.rT(videoIndices, :);
forces.lT = forces.lT(videoIndices, :);
forces.rCOP = forces.rCOP(videoIndices, :);
forces.lCOP = forces.lCOP(videoIndices, :);
return;
