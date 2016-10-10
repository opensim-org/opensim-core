%%  Script to test and show;
% (1) Reading c3d files into opensim table format
% (2) 'flattenning' a vec3 table to a table of doubles
% (3) Rotating table data and writing to a new table
% (4) Writing marker (.trc) and force (.mot) data to file

% Script written and edited by J. Dunne
% Data: Oct 2016
% Updated: Oct 2016

import org.opensim.modeling.*

%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter();

tables = adapter.read('test_walking.c3d');

%% get the Markers
markers = tables.get('markers');

% Print the (unrotated) markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');

%% Get the force and convert the times series table to type double
forces = tables.get('forces');

% make a vector of strings to be used for a custom suffix
suffixes = StdVectorString();
suffixes.add('_x'); 
suffixes.add('_y'); 
suffixes.add('_z');

% Flatten (Vec3 to double) the data to a times series table 
forces_double = forces.flatten(suffixes);

% Write flattened forces table to .mot file format
stofileadapter = STOFileAdapter();
stofileadapter.write(forces_double,'test_walking_grf.sto')

%% Define a rotation matix
Rot = 90;
rotationMatrix = [1,0,0;...
                  0,cos(Rot*pi/180),-(sin(Rot*pi/180));...
                  0,sin(Rot*pi/180),cos(Rot*pi/180)];

%% make a copy of the marker and grf tables
markerTable_rotated = markers;
forcesTable_rotated = forces;

%% Rotate marker data
for iMarker = 0 : markers.getNumColumns - 1
    % get the column data for the marker
    marker = markerTable_rotated.getDependentColumnAtIndex(iMarker);
    
    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : markers.getNumRows - 1
       % get Matlab vector marker position
       vectorData = [marker.getElt(0,iRow).get(0)...
                      marker.getElt(0,iRow).get(1)...
                      marker.getElt(0,iRow).get(2)];

       % rotate the marker data
       rotatedData = [rotationMatrix'*vectorData']';
       % write the rotated data to the table copy
       marker.set(iRow, Vec3(rotatedData(1),rotatedData(2),rotatedData(3)));        
    end
end

%% Print the rotated markers to trc file
trcfileadapter.write(markerTable_rotated,'test_walking_rotated.trc');

%% Rotate force data
for iforces = 0 : forces.getNumColumns - 1
    % get the column data for the marker
    force = forcesTable_rotated.getDependentColumnAtIndex(iforces);
    
    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : forces.getNumRows - 1
       % get Matlab vector marker position
       vectorData = [force.getElt(0,iRow).get(0)...
                      force.getElt(0,iRow).get(1)...
                      force.getElt(0,iRow).get(2)];

       % rotate the force data
       rotatedData = [rotationMatrix'*vectorData']';
       % write the rotated data to the table copy
       force.set(iRow, Vec3(rotatedData(1),rotatedData(2),rotatedData(3)));        
    end
end

%% Print the rotated forces to trc file
% Flatten (Vec3 to double) the data to a times series table 
forcesTable_double = forcesTable_rotated.flatten(suffixes);
% Write flattened forces table to .mot file format
stofileadapter.write(forcesTable_double,'test_walking_grf_rotated.mot')
