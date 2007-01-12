function reactionGRFTz = convert_actionToReaction(actionGRFTz)
% Purpose:  Converts a structure containing Fx, Fy, Fz, Tz, and COP data,
%           defined as 'action' forces and moments, to 'reaction' forces 
%           and moments.
%
% Input:    actionGRFTz is a structure with the following format:
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
%
% Output:   reactionGRFTz is a structure similar to actionGRFTz, but with
%           the signs of Fx, Fy, Fz, and Tz flipped.
%
% ASA, 10-05


% For each forceplate of interest ...
nHits = length(actionGRFTz);           
for fpHitNum = 1:nHits
    
    % Flip signs of 'action' forces and moments.
    reactionGRFTz(fpHitNum).Fx = -1 * actionGRFTz(fpHitNum).Fx;
    reactionGRFTz(fpHitNum).Fy = -1 * actionGRFTz(fpHitNum).Fy;
    reactionGRFTz(fpHitNum).Fz = -1 * actionGRFTz(fpHitNum).Fz;
    reactionGRFTz(fpHitNum).Tz = -1 * actionGRFTz(fpHitNum).Tz;
    
    % Copy other GRFTz data to output strucure.
    reactionGRFTz(fpHitNum).COPx = actionGRFTz(fpHitNum).COPx;
    reactionGRFTz(fpHitNum).COPy = actionGRFTz(fpHitNum).COPy;
    reactionGRFTz(fpHitNum).startIndex = actionGRFTz(fpHitNum).startIndex;
    reactionGRFTz(fpHitNum).stopIndex = actionGRFTz(fpHitNum).stopIndex;
    
end
return;
   