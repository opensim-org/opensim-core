%% // Example script for use in OpenSim 4.0
% (1) Reading c3d files into opensim table format
% (2) 'flattenning' a vec3 table to a table of doubles
% (3) Rotating table data and writing to a new table
% (4) Writing marker (.trc) and force (.mot) data to file

% Author: James Dunne, Shrinidhi K. Lakshmikanth, Chris Dembia, Tom Uchida,
% Ajay Seth, Ayman Habib, Jen Hicks. 


import org.opensim.modeling.*

%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter();

tables = adapter.read('test_walking.c3d');

%% get the marker data
markers = tables.get('markers');
% get the number of markers and rows
nMarkers = markers.getNumColumns();
markers_nRows = markers.getNumRows();
% get the array of the metadata key names
metadDataKeys_markers = markers.getTableMetaDataKeys();
markerrate = str2num(markers.getTableMetaDataAsString(metadDataKeys_markers.get(0)));

%% get the force data
forces = tables.get('forces');
% get the numner of forces and rows
nForces = forces.getNumColumns();
forces_nRows = forces.getNumRows();
% get the array of the metadata key names
metaDataKeys_forces = forces.getTableMetaDataKeys();
forcerate = str2num(forces.getTableMetaDataAsString(metaDataKeys_forces.get(2)));

%% Define a rotation matix
Rot = 90;          
rotationMatrix = [1,0,0;0,cos(Rot*pi/180),-(sin(Rot*pi/180));0,sin(Rot*pi/180),cos(Rot*pi/180)];
        
%% Rotate marker data
% make a clean copuy to alter
markers_rotated = markers();

for iMarker = 0 : nMarkers - 1

    % get the column data for the marker
    makrer = markers_rotated.updDependentColumnAtIndex(iMarker);

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : markers_rotated.getNumRows - 1
        % get Matlab vector marker position
        vectorData = [marker.getElt(0,iRow).get(0)...
                      marker.getElt(0,iRow).get(1)...
                      marker.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix'*vectorData']';

        % Write the rotated data back to the Vec3TimesSeriesTable
        elem = Vec3(rotatedData(1),rotatedData(2),rotatedData(3));
        % set the value of the element
        marker.set(iRow, elem);
    end
end

%% Rotate Force data
% make a clean copuy to alter
forces_rotated = forces();

for iForce = 0 : nForces - 1

    % get the column data for the marker
    force = forces_rotated.updDependentColumnAtIndex( iForce   );

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : forces_nRows - 1
        % get Matlab vector marker position
        vectorData = [force.getElt(0,iRow).get(0)...
                      force.getElt(0,iRow).get(1)...
                      force.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix'*vectorData']';

        % Write the rotated data back to the Vec3TimesSeriesTable
        elem = Vec3(rotatedData(1),rotatedData(2),rotatedData(3));
        % set the value of the element
        force.set(iRow, elem);

    end
end
%% Print the rotated markers to trc file

% make trc adapter and write marker tables to file. 
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');
trcfileadapter.write(markers_rotated,'test_walking_rotated.trc');

%% Print the force data
% make postfix string vector for naming colomns
postfix = StdVectorString();
postfix.add('_x');
postfix.add('_y');
postfix.add('_z');

% flatten the Vec3 table to a doubles table
forces_flattened = forces.flatten(postfix);
forces_rot_flattened = forces_rotated.flatten(postfix);

% make a sto adapter and write the forces table to file.
stofileadapater = STOFileAdapter();
stofileadapater.write(forces_flattened,'test_walking_grf.mot');
stofileadapater.write(forces_rot_flattened,'test_walking_grf_rotated.mot');


% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
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



