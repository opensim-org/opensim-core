function table_rotated = osimRotateTableData(table, axisString, value)
% Utility function for for rotating Vec3 TableData elements
% about an axisString by value (in degrees). 
% table         Vec3 dataTable
% axisString    string 'x', 'y', or 'z'
% value         double, in degrees
%  
% Example: rotate all (Vec3) elements in t by 90 degrees about the x axisString.                                     
% t_r = rotateTableData(t, 'x', -90) 

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% Author: James Dunne

%% import java libraries
import org.opensim.modeling.*

%% set up the transform
if strcmp(axisString, 'x')
    axis = CoordinateAxis(0);
elseif strcmp(axisString, 'y')
    axis = CoordinateAxis(1);
elseif strcmp(axisString, 'z')
    axis = CoordinateAxis(2);
else
    error(['Axis must be either x,y or z'])
end

%% instantiate a transform object
R = Rotation( deg2rad(value) , axis ) ;

%% rotate the elements in each row
% clone the table.
table_rotated = table.clone();

for iRow = 0 : table_rotated.getNumRows() - 1
    % get a row from the table
    rowVec = table_rotated.getRowAtIndex(iRow);
    % rotate each Vec3 element of row vector, rowVec, at once
    rowVec_rotated = R.multiply(rowVec);
    % overwrite row with rotated row
    table_rotated.setRowAtIndex(iRow,rowVec_rotated)
end

end







