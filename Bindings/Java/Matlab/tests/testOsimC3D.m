%% clear working space
clear all;close all;clc;
c3dpath = fullfile(cd,'walking2.c3d');

%% import opensim libraries
import org.opensim.modeling.*

%% Get path to c3d file in tests. 
c3dpath = fullfile(cd,'walking2.c3d');
c3d = osimC3D(c3dpath,1);
% Get tables directly from c3d file adapter
c3dAdapter = C3DFileAdapter();
data = c3dAdapter.read('walking2.c3d');
markers = data.get('markers');
forces = data.get('forces');

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


%% Test Rotations
% Rotate Data 180 degrees around X and check that Y and Z data are
% mirrored
c3d.rotateData('x',-90);c3d.rotateData('x',-90);
[markerRef180X, forceRef180X] = c3d.getAsStructs;
% For each marker, Y and Z data should be mirrored when rotating 180
% about x. 
for u = 1 : nMlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(markerRef.(mlabels{u}) + markerRef180X.(mlabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(2),5) ~= 0 | round(p(3),5) ~= 0
        error(['Error in rotateData(). Rotations about X are incorrect'])
    end
end
for u = 1 : nFlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(forceRef.(flabels{u}) + forceRef180X.(flabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(2),5) ~= 0 | round(p(3),5) ~= 0
        error(['Error in rotateData(). Rotations about X are incorrect'])
    end
end

% Rotate Data 180 degrees around Y and check that Z and X data are
% mirrored
c3d.rotateData('y',-90);c3d.rotateData('y',-90);
[markerRef180Y, forceRef180Y] = c3d.getAsStructs;
% For each marker, Y and Z data should be mirrored when rotating 180
% about x. 
for u = 1 : nMlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(markerRef180X.(mlabels{u}) + markerRef180Y.(mlabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(1),5) ~= 0 | round(p(3),5) ~= 0
        error(['Error in rotateData(). Rotations about Y are incorrect'])
    end
end
for u = 1 : nFlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(forceRef180X.(flabels{u}) + forceRef180Y.(flabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(1),5) ~= 0 | round(p(3),5) ~= 0
        error(['Error in rotateData(). Rotations about Y are incorrect'])
    end
 end

% Rotate Data 180 degrees around Z and check that X and Y data are
% mirrored
c3d.rotateData('z',-90);c3d.rotateData('z',-90);
[markerRef180Z, forceRef180Z] = c3d.getAsStructs;
% For each marker, Y and Z data should be mirrored when rotating 180
% about x. 
for u = 1 : nMlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(markerRef180Z.(mlabels{u}) + markerRef180Y.(mlabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(1),5) ~= 0 | round(p(2),5) ~= 0
        error(['Error in rotateData(). Rotations about Z are incorrect'])
    end
end
for u = 1 : nFlabels
    % Get the sum of the differences between original and rotated data
    p = nansum(forceRef180Z.(flabels{u}) + forceRef180Y.(flabels{u}));
    % Sum should be at or near zero. Round to 5 decimal places 
    if round(p(1),5) ~= 0 | round(p(2),5) ~= 0
        error(['Error in rotateData(). Rotations about Z are incorrect'])
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
trcTable = TRCFileAdapter.read(fullfile(cd, 'walking2.trc'));
motTable = STOFileAdapter.read(fullfile(cd, 'walking2.mot'));

% Test if readable by Storage
sto  = Storage(fullfile(cd, 'walking2.trc'));
sto  = Storage(fullfile(cd, 'walking2.mot'));

%%

