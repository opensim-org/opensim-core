%% clear working space
clear all;close all;clc;

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Instantiate a model from file
model = Model('../../../../OpenSim/Tests/shared/arm26.osim');

%% Test Body list
% Get the number of components of type 'Body'
nBodies = osimList.getNumComponents(model, 'Body');
% Get all the Body names as a cell of strings
names = osimList.getComponentNames(model, 'Body');
% Get the Pelvis Body
BodyReference = osimList.getComponent(model, 'Body', names{1});

%% Muscle List 
% Get the number of components of type 'Body'
nMuscles = osimList.getNumComponents(model, 'Muscle');
% Get all the Body names as a cell of strings
names = osimList.getComponentNames(model, 'Muscle');
% Get the BRA Muscle
MuscleReference = osimList.getComponent(model, 'Muscle', names{nMuscles});










