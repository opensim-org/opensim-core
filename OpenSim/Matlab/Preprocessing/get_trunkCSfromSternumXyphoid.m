function R = get_trunkCSfromSternumXyphoid(landmarks)
% Purpose:  Returns the rotation matrix that defines the orientation 
%           of a trunk reference frame from the coordinates of the 
%           sternal notch, zyphoid process, and subclavicular markers.
%
% Input:    landmarks is a structure with the following format:
%           landmarks.rSubclav      = 1x3 vector of xyz marker coordinates
%           landmarks.lSubclav      = "
%           landmarks.sternalnotch  = "
%           landmarks.xyphoid  = "
%

% Output:   R returns a 3x3 rotation matrix that describes the orientation
%           of the trunk CS with respect to the CS used to define the 
%           trunk landmarks.
%           
%           NOTE: The directions of the X,Y,Z axes are consistent with 
%                 Darryl's model, with: 
%                       +X anterior, +Y superior, +Z to the right
%
% ASA, 10-05


% Z axis (ML, R+) is the unit vector from lSubclav to rSubclav.
  Zaxis = landmarks.rSubclav - landmarks.lSubclav;
  Zaxis = Zaxis./norm(Zaxis);

% Y axis (SI, S+) is the unit vector Z x (xyphoid - sternalnotch).
  Yaxis = cross(Zaxis, (landmarks.xyphoid - landmarks.sternalnotch));
  Yaxis = Yaxis./norm(Yaxis);

% X axis (AP, A+) is the unit vector Y x Z.
  Xaxis = cross(Yaxis, Zaxis);
  Xaxis = Xaxis./norm(Xaxis);
  
% Return rotation matrix.
  R = [Xaxis' Yaxis' Zaxis'];
  return;