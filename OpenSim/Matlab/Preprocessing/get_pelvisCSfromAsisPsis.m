function R = get_pelvisCSfromAsisPsis(landmarks)
% Purpose:  Returns the rotation matrix that defines the orientation 
%           of a pelvis reference frame from the right and left 
%           ASIS and PSIS markers.
%
% Input:    landmarks is a structure with the following format:
%           landmarks.rASIS      = 1x3 vector of xyz marker coordinates
%           landmarks.lASIS      = "
%           landmarks.rPSIS      = "
%           landmarks.lPSIS      = "
%
% Output:   R returns a 3x3 rotation matrix that describes the orientation
%           of the pelvis CS with respect to the CS used to define the 
%           pelvis landmarks.
%           
%           NOTE: The directions of the X,Y,Z axes are consistent with 
%                 Darryl's model, with: 
%                       +X anterior, +Y superior, +Z to the right
%
% ASA, 10-05


% Z axis (ML, R+) is the unit vector from lASIS to rASIS.
  Zaxis = landmarks.rASIS - landmarks.lASIS;
  Zaxis = Zaxis./norm(Zaxis);

% Y axis (SI, S+) is the unit vector Z x (midptASISs - midptPSISs).
  midASIS = (landmarks.rASIS + landmarks.lASIS)./2;
  midPSIS = (landmarks.rPSIS + landmarks.lPSIS)./2;
  Yaxis = cross(Zaxis, (midASIS - midPSIS));
  Yaxis = Yaxis./norm(Yaxis);

% X axis (AP, A+) is the unit vector Y x Z.
  Xaxis = cross(Yaxis, Zaxis);
  Xaxis = Xaxis./norm(Xaxis);
  
% Return rotation matrix.
  R = [Xaxis' Yaxis' Zaxis'];
  return;