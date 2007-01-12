function R = get_RfromKtheta(K, theta_deg)
% Purpose: Returns the rotation matrix R corresponding to 
%          a rotation theta (in degrees) about a vector K.
%
% ASA, 3-05

% Convert degrees to radians
theta_rad = theta_deg * pi/180.0;

% Get sine, cosine, and versine of theta_rad
s = sin(theta_rad);
c = cos(theta_rad);
v = 1.0 - c;

% Get elements of vector K
kx = K(1);
ky = K(2);
kz = K(3);

% Compute rotation matrix R
R = [kx*kx*v+c     ky*kx*v-kz*s  kz*kx*v+ky*s; ...
     kx*ky*v+kz*s  ky*ky*v+c     kz*ky*v-kx*s; ...
     kx*kz*v-ky*s  ky*kz*v+kx*s  kz*kz*v+c];

return;


