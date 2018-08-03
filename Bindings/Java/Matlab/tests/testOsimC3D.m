%% clear working space
clear all;close all;clc;

%% import opensim libraries
import org.opensim.modeling.*

%% Get path to c3d file in tests. 
c3dpath = fullfile(cd,'walking2.c3d');
c3d = osimC3D(c3dpath,1);

% Get tables directly from c3d file adapter
data = C3DFileAdapter.read(c3dpath,1);
markers = data.get('markers');
forces = data.get('forces');

%% Test getAsStructs

[markerRef, forceRef] = c3d.getAsStructs;

mlables = fieldnames(markerRef);
flables = fieldnames(forceRef);





%% Test Rotations

for i = 1 : 4 
    c3d.rotateData('x',-90)
    
    if i == 2
        % Rotation should be 180 degrees 
        [markerRef180, forceRef180] = c3d.getAsStructs;
        
        marker
        
    
    
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

