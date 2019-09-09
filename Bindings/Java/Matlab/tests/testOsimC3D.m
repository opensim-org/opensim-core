%% clear working space
clear all;close all;clc;
c3dpath = fullfile(cd,'walking2.c3d');

%% import opensim libraries
import org.opensim.modeling.*

% Integer value for representation of force from plate
% 0 = forceplate orgin, 1 = COP, 2 = Point Of Wrench Application
ForceLocation = 0;

%% Get path to c3d file in tests. 
c3dpath = fullfile(cd,'walking2.c3d');
c3d = osimC3D(c3dpath,ForceLocation);
% Get tables directly from c3d file adapter
c3dAdapter = C3DFileAdapter();
c3dAdapter.setLocationForForceExpression(ForceLocation);
tables = c3dAdapter.read('walking2.c3d');
markers = c3dAdapter.getMarkersTable(tables);
forces = c3dAdapter.getForcesTable(tables);

%% Test getAsStructs
[markerRef, forceRef] = c3d.getAsStructs;

mlabels = fieldnames(markerRef);
flabels = fieldnames(forceRef);

% Get the number of labels (minus time column)
nMlabels = length(mlabels)-1;
nFlabels = length(flabels)-1;

% Number of Columns in osim tables must equal number in structures
assert(markers.getNumColumns() == nMlabels, 'Number or markers from osimC3D is not equal to number in osim table');
assert(forces.getNumColumns()  == nFlabels, 'Number or forces from osimC3D is not equal to number in osim table');


%% Perform Table Rotations
% Rotate Data 180 degrees around X and check that Y and Z data are
% mirrored
c3d.rotateData('x',-90);c3d.rotateData('x',-90);
% For each marker, Y and Z data should be mirrored when rotating 180
% about x. 
markers_rotated = c3d.getTable_markers();
forces_rotated = c3d.getTable_forces();

%% Test Rotations
% Compare the values between original and rotated data. Use random row and
% col numbers.

% Test Marker Rotations
randomRows = randsample(markers.getNumRows(),10)-1;
randomCols = randsample(markers.getNumColumns(),10)-1;
for i = 1 : 10
    % Get the values in the original and rotated marker tables
    loc = osimVec3ToArray(markers.getRowAtIndex(randomRows(i)).get(randomCols(i)));
    loc_rotated = osimVec3ToArray(markers_rotated.getRowAtIndex(randomRows(i)).get(randomCols(i)));
    % Compute the difference in the marker locations. The Y and Z values
    % should be mirroed and result in 0 when added. Round to 8 decimal
    % places. 
    diff_array = round(loc + loc_rotated,8);
    % Compare the difference
    if isnan(diff_array)
        % If the value is a nan then move the next value
        continue
    elseif diff_array(2) ~= 0 || diff_array(3) ~= 0
        error(['Error in rotateData(). Marker rotations about X are incorrect'])
    end
end

% Test Force Rotations
randomRows = randsample(forces.getNumRows(),6)-1;
randomCols = randsample(forces.getNumColumns(),6)-1;
for i = 1 : 6
    % Get the values in the original and rotated marker tables
    val = osimVec3ToArray(forces.getRowAtIndex(randomRows(i)).get(randomCols(i)));
    val_rotated = osimVec3ToArray(forces_rotated.getRowAtIndex(randomRows(i)).get(randomCols(i)));
    % Compute the difference in the marker locations. The Y and Z values
    % should be mirroed and result in 0 when added. Round to 8 decimal
    % places. 
    diff_array = round(val + val_rotated,8);
    % Compare the difference
    if isnan(diff_array)
        % If the value is a nan then move the next value
        continue
    elseif diff_array(2) ~= 0 || diff_array(3) ~= 0
        error(['Error in rotateData(). Force rotations about X are incorrect'])
    end
end

%% Test mm to M conversion for forces
c3d.convertMillimeters2Meters();
forces_m = c3d.getTable_forces();

% Forces should remain unchanged, point and moments should be in meters
% (divided by 1000)
randomRows = randsample(forces.getNumRows(),101)-1;
for i =  0 : forces_m.getNumColumns() - 1
    
    col =  forces_rotated.getDependentColumnAtIndex(i);
    col_m = forces_m.getDependentColumnAtIndex(i);
    label = forces_m.getColumnLabel(i);
    
    for u = 1 : length(randomRows)
        if startsWith(char(forces_m.getColumnLabels().get(i)),'f')
            % Get the values
            d   = col.get(randomRows(u));
            d_m = col_m.get(randomRows(u));
            
            assert(d.get(0)==d_m.get(0),'Force Data has been incorrectly altered');
            assert(d.get(1)==d_m.get(1),'Force Data has been incorrectly altered')
            assert(d.get(2)==d_m.get(2),'Force Data has been incorrectly altered')
        else
            % Get the values
            d   = col.get(randomRows(u));
            d_m = col_m.get(randomRows(u));
            
            assert(d.get(0)/1000==d_m.get(0),'Point or Moment Data has been incorrectly altered');
            assert(d.get(1)/1000==d_m.get(1),'Point or Moment Data has been incorrectly altered');
            assert(d.get(2)/1000==d_m.get(2),'Point or Moment Data has been incorrectly altered');
        end
    end
end
%% Test Writing Methods
c3d.writeTRC()
if isempty(exist( fullfile(cd, 'walking2.trc') ,'file'))
    error('c3d.writeTRC() did not write walking2.trc to c3d dir')
end

c3d.writeTRC(['testing.trc'])
if isempty(exist( fullfile(cd, 'testing.trc') ,'file'))
    error('c3d.writeTRC(fileName) did not write testing.trc to c3d dir')
end

c3d.writeTRC( fullfile(cd,'testing2.trc'))
if isempty(exist( fullfile(cd, 'testing2.trc') ,'file'))
    error('c3d.writeTRC(fullpath) did not write testing2.trc to c3d dir')
end

c3d.writeMOT()
if isempty(exist( fullfile(cd, 'walking2.mot') ,'file'))
    error('c3d.writeMOT() did not write walking2.mot to c3d dir')
end

c3d.writeMOT(['testing.mot'])
if isempty(exist( fullfile(cd, 'testing.mot') ,'file'))
    error('c3d.writeMOT(fileName) did not write testing.mot to c3d dir')
end

c3d.writeMOT( fullfile(cd,'testing2.mot'))
if isempty(exist( fullfile(cd, 'testing2.mot') ,'file'))
    error('c3d.writeMOT(fullpath) did not write testing2.mot to c3d dir')
end

%% Test that TRC and MOT are readable by osim classes
trcTable = TimeSeriesTableVec3(fullfile(cd, 'walking2.trc'));
motTable = TimeSeriesTable(fullfile(cd, 'walking2.mot'));

% Test if readable by Storage
sto  = Storage(fullfile(cd, 'walking2.trc'));
sto  = Storage(fullfile(cd, 'walking2.mot'));

%%
disp('Tests Passed!')
