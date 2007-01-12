function f = get_c3dForcePlatform(itf);
% Purpose:  This function uses the C3D Server from Motion Lab Systems to
%           retrieve FORCE_PLATFORM information from a Gillette C3D file 
%           relevant for generating a subject-specific simulation. 
%
%           The C3D Server must be activated as a COM object using 
%           Matlab's ActiveX interface PRIOR to calling this function.  
%
% Input:    itf is the name given to the instance of the C3D Server 
%           activated by the calling function. 
%
% Output:   f returns a structure with the following format: 
%             f(FP#).corners(corner#, 3) - stores xyz coordinates of the 
%                   4 corners of each FP, in the lab CS
%             f(FP#).origin - stores xyz elements of the vector from the
%                   center of the FP surface to the FP origin, in the
%                   FP coordinate system (Type 2, AMTI plates).
%               
% ASA 9-05
% Retrieve number of FPs from FORCE_PLATFORM:USED.
parameter_index = itf.GetParameterIndex('FORCE_PLATFORM', 'USED');
item_index = 0;                     % index corresponding to 1st item
nFP = itf.GetParameterValue(parameter_index, item_index);
% Retrieve coordinates of FP corners from FORCE_PLATFORM:CORNERS.
parameter_index = itf.GetParameterIndex('FORCE_PLATFORM', 'CORNERS');
for fpIndex = 1:nFP
    for cornerIndex = 1:4
        item_index = 3*4*(fpIndex - 1) + 3*(cornerIndex - 1);
                                    % C3D Server uses 0-based index 
        xCoord = itf.GetParameterValue(parameter_index, item_index);
        yCoord = itf.GetParameterValue(parameter_index, item_index + 1);
        zCoord = itf.GetParameterValue(parameter_index, item_index + 2);
        corners(cornerIndex, 1) = xCoord;
        corners(cornerIndex, 2) = yCoord;
        corners(cornerIndex, 3) = zCoord;
    end
    f(fpIndex).corners = corners;
end
% Retrieve vector from FP surface to FP origin from FORCE_PLATFORM:ORIGIN.
for fpIndex = 1:nFP
    parameter_index = ...
        itf.GetParameterIndex('FORCE_PLATFORM', 'ORIGIN');
    item_index = 3*(fpIndex - 1);        % C3D Server uses 0-based index 
    xCoord = itf.GetParameterValue(parameter_index, item_index);
    yCoord = itf.GetParameterValue(parameter_index, item_index + 1);
    zCoord = itf.GetParameterValue(parameter_index, item_index + 2);
    f(fpIndex).origin = [xCoord yCoord zCoord];
end
return;
