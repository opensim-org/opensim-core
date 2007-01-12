function angleSternumVertical = get_sternumAngle(sternumVector, source)
% Purpose:  Computes the angle(s), in the sagittal plane, between the
%           vertical axis and the vector(s) (sternal_notch - xyphoid).
%
% Input:    sternumVector is a matrix of vectors (sternal_notch - xyphoid)
%               with dimension (nVideoFrames, 3)
%           source specifies whether the marker data are in the 
%               model CS ('model'), or
%               Gillette's lab CS, facing +X direction ('gillettePosX')
%               Gillette's lab CS, facing -X direction ('gilletteNegX')
%
% Output:   angleSternumVertical is an array of angles, in degrees, 
%             of the sternumVector with respect to the vertical axis 
%               Sign convention: angle is + if sternum is tilted 
%                                posteriorly wrt vertical axis,
%                                consistent with model CS.
%
% ASA, 10-05


% Specify vertical axis depending on the source of the marker data and 
% (if data are from Gillette) whether subject walked in +x or -x direction.
if strcmpi(source, 'model')
    verticalAxis = [0 1 0];   
    sagittalAxis = [0 0 1];
    frontalAxis  = [1 0 0];
elseif strcmpi(source, 'gillettePosX')
    verticalAxis = [0 0 1];   
    sagittalAxis = [0 1 0];
    frontalAxis  = [1 0 0];             
elseif strcmpi(source, 'gilletteNegX')
    verticalAxis = [0 0 1];   
    sagittalAxis = [0 1 0];
    frontalAxis  = [-1 0 0];   
end
    
% Get signed angle between sternumVector and vertical axis 
% in sagittal plane at each sample point.
% NOTE:  If angle between sternum and vertical axis is computed directly, 
%        acos will always return a positive angle.  Instead, find angle  
%        wrt frontal axis, then find signed angle wrt vertical.  
[nRows, nCols] = size(sternumVector);
for ptNum = 1:nRows
    sternumInPlane = ...
      sternumVector(ptNum, :) - dot(sternumVector(ptNum, :), sagittalAxis);
    sternumInPlane = sternumInPlane./norm(sternumInPlane);
    
    % Compute angle between sternum vector and frontal axis.
    angleSternumFrontal = acosd(dot(sternumInPlane, frontalAxis));
    
    % Compute angle between sternum vector and vertical axis.
    % Sign convention:  angle is + if sternum is tilted posteriorly.
    angleSternumVertical(ptNum) = angleSternumFrontal - 90;
end
return;