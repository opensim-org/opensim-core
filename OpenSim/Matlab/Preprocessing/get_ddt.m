function [dxdt, td] = get_ddt(x, t)
% Purpose:  Computes three-point derivative, dx/dt, of the input array x.
% Output:   get_ddt returns the following arrays:
%               dxdt is an array of dxdt values
%               td is an array of time values corresponding to dxdt
%
% ASA, 7-05
% Revised 8-22-05


% Compute three-point derivative dx/dt.
nPts = length(x);
dx = x(3:nPts) - x(1:(nPts-2));
dt = t(3:nPts) - t(1:(nPts-2));
dxdt = dx ./ dt;
td = t(2:(nPts-1));     % new time array corresponding to dxdt

% Correct for high frequency artifacts at beginning and end of trial;
% this facilitates plotting derivatives of marker trajectories.
tol = 6000;
for i = 1:length(dxdt)
    if i == 1 & abs(dxdt(1)) > tol
        dxdt(1) = 0;
    elseif i > 1 & abs(dxdt(i)) > tol
        dxdt(i) = dxdt(i-1);
    end
end
return;
    
