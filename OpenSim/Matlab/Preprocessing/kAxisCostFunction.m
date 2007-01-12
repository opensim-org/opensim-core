function F = kAxisCostFunction(offsetAngle, kf, kv)
% Purpose:  Returns my vector-valued function for estimating
%           malalignment of the knee flexion/extension axis:
%               F(nPts) = corrected knee var/val angle 
%
%           NOTE:
%           When used with lsqnonlin from Matlab's Optimization Toolbox,
%           the cost function that is minimized is:
%               J = sum[(knee var/val angle)^2] over simulateable segment
%
% Inputs:   offsetAngle is the estimated malalignment (in degrees). 
%           kf and kv are arrays of 'measured' knee flex/ext and 
%             var/val angles, in a right handed coordinate system 
%             (in degrees).
% 
%           Sign Conventions:  
%           The sign conventions for the joint angles are based on a 
%             right-handed coordinate system with X anterior, Y left, 
%             and Z superior.
%           For a R limb, flex+, var+, int+ (consistent w/ gait lab).
%           For a L limb, flex+, val+, ext+ (inconsistent w/ gait lab).
%           offsetAngle is a positive rotation about the vertical Z axis.
%
% ASA, 7-05, rev 10-05


% Get sine and cosine of offset (in degrees).
So = sind(offsetAngle);
Co = cosd(offsetAngle);

% Get necessary sines, cosines of kf and kv (arrays, in degrees).
Sf = sind(kf);
Sv = sind(kv);
Cv = cosd(kv);

% Compute knee var/val angles 'corrected' by offsetAngle.
npts = length(kf);
for i = 1:npts
    kvCorrected(i) = asind(Sv(i)*Co - Sf(i)*Cv(i)*So);
end

% Return vector-valued function.
F = kvCorrected;
return;
