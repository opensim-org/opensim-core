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
adapter = C3DFileAdapter()

tables = adapter.read('test_walking.c3d');

%% get the Markers
markers = tables.get('markers');

% Print the (unrotated) markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');

%% Get the force and convert the times series table to type double
forces = tables.get('forces');

% Flatten the data to a times series table
% THE BELOW LINE GENERATES AN EXCEPTION
forcesdouble = forces.flatten()  ;

% Write flattened forces table to .mot file format
stofileadapter = STOFileAdapter();
stofileadapter.write(forces,'test_walking_grf.mot')

%% Define a rotation matix
Rot = 90;
rotationMatrix = [1,0,0;...
                  0,cos(Rot*pi/180),-(sin(Rot*pi/180));...
                  0,sin(Rot*pi/180),cos(Rot*pi/180)];


%% Rotate marker data
% MAKE A COPY OF THE TABLE

for iMarker = 0 : length(markerdata) - 1

    % get the column data for the marker
    marker = markers.updDependentColumn(markerlabels(iMarker+1) );

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : markers.getNumRows - 1
        % get Matlab vector marker position
        vectorData = [marker.getElt(0,iRow).get(0)...
                      marker.getElt(0,iRow).get(1)...
                      marker.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix'*vectorData']';

        % WRITE THE ROTATED DATA TO THE TABLE COPY

    end
end

%% Print the rotated markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking_rotated.trc');
