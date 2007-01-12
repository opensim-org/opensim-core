function kAxisOffset = get_kAxisOffset(measuredAngles, limb)
% Purpose:  Estimates the degree of malalignment of the knee flex/ext axis
%           based on 'measured' knee angles, extracted from the C3D file
%           of a Gillette subject, for the limb of interest.
%
%           The knee flex/ext axis offset is determined from minimizing
%           the squared magnitude of the knee var/val angle over the 
%           simulateable segment.
%
% Input:    measuredAngles is a structure comprised of the following:
%               *.kf - array of 'measured' knee flex/ext angles
%                .kv - array of 'measured' knee var/val angles
%                .kr - array of 'measured' knee rotation angles
%                .hr - array of 'measured' hip rotation angles
%               NOTE:  These are the angles as reported in C3D file; 
%                  if limb == 'L', these angles must be converted to
%                  a right-handed coordinate system before determining
%                  the corrections as formulated here.
%           limb specifies whether the 'L' or 'R' limb is to be analyzed
%
% Output:   kAxisOffset returns my 'best guess' degree of malalignment
%           Sign Convention:  a positive offset is a positive rotation
%                             about the Z axis, where Z points superior.
%
% Called Functions:
%       lsqnonlin, optimset from Matlab's Optimization Toolbox
%       kAxisCostFunction(kAxisOffset, kf, kv)
% 
% ASA, 10-05


% Get knee flex/ext and var/val angles, and convert to a 
% right handed coordinate system with X anterior, Y left, and Z superior.
%   For a R limb, flex+, var+, int+ (consistent w/ gait lab).
%   For a L limb, flex+, val+, ext+ (inconsistent w/ gait lab).
if strcmpi(limb, 'R')
    kf = measuredAngles.kf;
    kv = measuredAngles.kv;
elseif strcmpi(limb, 'L')
    kf = measuredAngles.kf;
    kv = -1 * measuredAngles.kv;
end

% Search for optimal kAxisOffset.
% NOTE: Since lsqunonlin returns a local minimum, and since the function
%       to be minimized is highly nonlinear, the optimization is performed
%       for a range of initial guesses, and the solution with the smallest
%       residual is selected.
resMin = 1e6;               
initialGuess = -20:1:20;                
for j = 1:length(initialGuess)

    % Call lsqnonlin from Matlab's Optimization Toolbox to find the 
    % angular offset that minimizes the squared magnitude of the 
    % knee var/val angle over the simulateable segment, 
    % as defined by kAxisCostFunction.
    % NOTE:  set optimset('Display', 'iter') to display output
    %        set optimset('Display', 'off') to suppress display
    options = optimset('LargeScale', 'off', 'Display', 'off');
    [offsetAngle, resnorm] = ...
       lsqnonlin(@(offsetAngle) kAxisCostFunction(offsetAngle, kf, kv), ...
                  initialGuess(j), [], [], options);
              
   % If residual at initialGuess(j) < current minimum, 
   % update 'best' solution.
   if resnorm < resMin         
        kAxisOffset = offsetAngle;
        resMin = resnorm;
    end
end
return;

