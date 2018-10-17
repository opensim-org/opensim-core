%% clear working space
clear all;close all;clc;

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Instantiate a model from file
model = Model('../../../../OpenSim/Tests/shared/arm26.osim');

%% Test Body list
% Get a Matlab BodyList object
bodylist = osimList(model,'Body');
% Get the size of the list 
n = bodylist.getSize();  
% Get all the Body names as a cell of strings
names = bodylist.getNames(); 
% Get all the component Outputs as a cell of strings
outputnames = bodylist.getOutputNames(); 
% Get a reference to one of the Bodies in the list
body = bodylist.getByName(names{1}); 
body = bodylist.getByIndex(1); 

%% Muscle List 
% get a Matlab MuscleList object
musclelist = osimList(model, 'Muscle');
% get the number of muscles
nm = musclelist.getSize();
% Get all the Muscle names as a cell of strings
names = musclelist.getNames();
% Get all the component Outputs as a cell of strings
outputnames = musclelist.getOutputNames();
% Get a reference to one of the Muscles in the list
muscle = musclelist.getByName(names{1});
muscle = musclelist.getByIndex(2);










