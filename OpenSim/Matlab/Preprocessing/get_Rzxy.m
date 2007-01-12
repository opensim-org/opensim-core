function R = get_Rzxy(rz_deg, rx_deg, ry_deg)
% Purpose: Returns the rotation matrix R, where 
%               R = [RotZ(rz)]*[RotX(rx)]*[RoxY(ry)]
%
% ASA, 3-05

% Solution from David Eberly, Magic Software Inc.
% http://www.magic-software.com


% Convert degrees to radians.
deg2rad = pi/180.0;
rx_rad = rx_deg * deg2rad;
ry_rad = ry_deg * deg2rad;
rz_rad = rz_deg * deg2rad;

% Compute transformation matrix from angles.
cx = cos(rx_rad);
cy = cos(ry_rad);
cz = cos(rz_rad);
sx = sin(rx_rad);
sy = sin(ry_rad);
sz = sin(rz_rad);

R = [cy*cz-sx*sy*sz, -cx*sz,  cz*sy+cy*sx*sz; ...
     cz*sx*sy+cy*sz,  cx*cz, -cy*cz*sx+sy*sz; ...
     -cx*sy,          sx,     cx*cy];    
return;           