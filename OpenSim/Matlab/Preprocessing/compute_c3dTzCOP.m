function actionGRFTz_FP = ...
                compute_c3dTzCOP(actionGRFM_FP, fpInfo, analogRate)
% Purpose:  Computes the center of pressure (COP) and vertical torque (Tz)
%           from the 'action' forces and moments (Fx, Fy, Fz, Mx, My, Mz) 
%           stored in actionGRFM_FP, in the FP coordinate systems, for
%           each FP that was hit.
%
% Input:    actionGRFM_FP is a structure with the following format, 
%             in the FP coordinate systems, sampled at the analog rate:
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
%           analogRate is the sampling rate of the analog data
%
% Output:   actionGRFTz_FP returns the 'action' forces, vertical torque,
%             and COP, in the FP coordinate systems, sampled at the
%             analog rate:
%               *(fpHitNum) .Fx(nAnalogFrames)
%                           .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Tz(nAnalogFrames)
%                           .COPx(nAnalogFrames)
%                           .COPy(nAnalogFrames)
%                           .startIndex - analog frame number indicating
%                                           start of COP, Tz data
%                           .stopIndex  - analog frame number indicating
%                                           end of COP, Tz data
%
% NOTES:    1. The equations for COP go to infinity when Fz = 0; thus,
%              COPx, COPy, and Tz are computed between IC and TO for
%              each FP hit, and are set = 0 for all other frames of
%              the 'simulateable' segment.  
%           2. For a Gillette CP subject, a 'simulateable' segment is
%              defined based on events in the Paradox DB, and not on
%              vertical GRF data.  Therefore, the IC and TO events must be
%              detected before the COP and Tz can be reliably computed.
%
% ASA, 10-05


% Get number of analog frames in 'simulateable' segment.
nAnalogFrames = length(actionGRFM_FP(1).Fx);
           
% For each forceplate of interest ...
nHits = length(actionGRFM_FP);
for fpHitNum = 1:nHits
        
    % Get indices of Fz greater than a threshold, indicating 'contact',
    % and define threshold = 1% of the max vertical force.
    % NOTE: the 'action' Fz in the FP coordinate system has a + sign.
    threshold = 0.01*max(actionGRFM_FP(fpHitNum).Fz);
    contactIndices = find(actionGRFM_FP(fpHitNum).Fz > threshold);
    
    % Make sure that max(contactIndices) represents toe-off, and not an
    % abberent large Fz value at the end of the trial; to do this,  
    % define tolerance (# analog frames = time * sampling rate) 
    % for max # analog frames between IC and TO
    tol = 3.5*analogRate;
    while max(contactIndices) - min(contactIndices) > tol
        contactIndices(length(contactIndices)) = [];
    end
    
    % Get analog frames corresponding to the beginning and end of contact, 
    % as determined from Fz.
    startIndex = min(contactIndices);
    stopIndex = max(contactIndices);
    
    % Get forces and moments between startIndex and stopIndex, 
    % during contact, for the current FP hit.
    Fx = actionGRFM_FP(fpHitNum).Fx(startIndex:stopIndex);
    Fy = actionGRFM_FP(fpHitNum).Fy(startIndex:stopIndex);
    Fz = actionGRFM_FP(fpHitNum).Fz(startIndex:stopIndex);
    Mx = actionGRFM_FP(fpHitNum).Mx(startIndex:stopIndex);
    My = actionGRFM_FP(fpHitNum).My(startIndex:stopIndex);
    Mz = actionGRFM_FP(fpHitNum).Mz(startIndex:stopIndex);
        
    % Prevent unwanted end effects in the COP values near IC and TO by
    % limiting how small Fz can get.
    % NOTE:  Not needed for Gillette control subjects.
%     tol = 10;   
%     tolIndices = find(Fz < tol);
%     Fz(tolIndices) = tol;       % during 'contact', require Fz >= 10 N.
    
    % Get vertical component of vector from FP origin to FP surface,
    % in the FP CS.  In the FP CS, this should be a negative number.
    % NOTE:  The 'origin' stored in the C3D file for AMTI plates gives
    %        the vector from the FP surface to the FP origin; thus,
    %        dz = -1 * zCoord as read from fpInfo.origin.
    dz = -1 * fpInfo(fpHitNum).origin(3);
   
    % Compute COP between startIndex and stopIndex, during contact.
    COPx = (-1*My + dz*Fx)./Fz;
    COPy = (Mx + dz*Fy)./Fz;
    
    % Compute Tz between startIndex and stopIndex, during contact.
    Tz = Mz + COPy.*Fx - COPx.*Fy;     
      
    % Store 'action' forces, vertical torque, and COP in the FP CSs,
    % for 'simulateable' segment, as column arrays in structure.
    % NOTE:  set values == 0 when foot is not in contact with ground.
    actionGRFTz_FP(fpHitNum).Fx = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).Fx(startIndex:stopIndex) = Fx;
    
    actionGRFTz_FP(fpHitNum).Fy = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).Fy(startIndex:stopIndex) = Fy;
    
    actionGRFTz_FP(fpHitNum).Fz = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).Fz(startIndex:stopIndex) = Fz;
    
    actionGRFTz_FP(fpHitNum).Tz = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).Tz(startIndex:stopIndex) = Tz;
    
    actionGRFTz_FP(fpHitNum).COPx = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).COPx(startIndex:stopIndex) = COPx;
    
    actionGRFTz_FP(fpHitNum).COPy = zeros(nAnalogFrames, 1);
    actionGRFTz_FP(fpHitNum).COPy(startIndex:stopIndex) = COPy;
    
    actionGRFTz_FP(fpHitNum).startIndex = startIndex;
    actionGRFTz_FP(fpHitNum).stopIndex = stopIndex;
end
return;