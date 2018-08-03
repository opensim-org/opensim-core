%% clear working space
clear all;close all;clc;

%% import opensim libraries
import org.opensim.modeling.*

%% Get path to c3d file in tests. 
c3dpath = fullfile(cd,'walking2.c3d');
c3d = osimC3D(c3dpath,1);

%% Test TRC trc output
c3d.writeTRC()
% Determine the trc file name
trcName = fullfile(filepath, [name '.trc']);
% Test if readable by TRCFileAdapter
trcTable = TRCFileAdapter.read(trcName);
% Test if readable by Storage
sto  = Storage(trcName);


%% test mot output
c3d.writeMOT()
% Determine the mot file name
motName = fullfile(filepath, [name '.mot']);
% Test if readable by TRCFileAdapter
motTable = STOFileAdapter.read(motName);
% Test if readable by Storage
sto  = Storage(motName);
