function [rz_deg rx_deg ry_deg] = get_RzxyAngles(R)
% Purpose: Returns the angles rz_deg, rx_deg, and ry_deg corresponding to 
%          rotation matrix R, where R = [RotZ(rz)]*[RotX(rx)]*[RoxY(ry)]
%         
%          Angles are returned in units of degrees.
%
% ASA, 3-05

% Solution from David Eberly, Magic Software Inc.
% http://www.magic-software.com

rx = asin(R(3, 2));
if rx < pi/2 
    if rx > -pi/2   % non-degenerate case
        rz = atan2(-R(1, 2), R(2, 2));
        ry = atan2(-R(3, 1), R(3, 3));
        
    else            % degenerate case, rx = -90 degrees
        rz = -atan2(R(1, 3), R(1, 1));
        ry = 0;
    end
else                % degenerate case, rx = +90 degrees
    rz = atan2(R(1, 3), R(1, 1));
    ry = 0;
end

% Convert radians to degrees.
rad2deg = 180.0/pi;
rx_deg = rx*rad2deg;
ry_deg = ry*rad2deg;
rz_deg = rz*rad2deg;

% % Re-compute original transformation matrix from angles as a check.
% cx = cos(rx);
% cy = cos(ry);
% cz = cos(rz);
% sx = sin(rx);
% sy = sin(ry);
% sz = sin(rz);
% R = R
% R_check = [cy*cz-sx*sy*sz, -cx*sz,  cz*sy+cy*sx*sz; ...
%            cz*sx*sy+cy*sz,  cx*cz, -cy*cz*sx+sy*sz; ...
%            -cx*sy,          sx,     cx*cy]
      
return;           