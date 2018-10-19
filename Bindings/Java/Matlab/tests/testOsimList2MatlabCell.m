%% clear working space
clear all;close all;clc;

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Instantiate a model from file
model = Model('../../../../OpenSim/Tests/shared/arm26.osim');

%% Test Body list
% Get a cell array of references to all the bodies in a model
Bodies = osimList2MatlabCell(model, 'Body');

% Get the names of all the bodies
names = {};
for i = 1 : length(Bodies)
    names{i,1} = char(Bodies{i}.getName());
end

% Get the individual reference to a Body
b = Bodies{3};
% Get the inertia of the body as a Vec3()
in = Bodies{3}.get_inertia;
 









