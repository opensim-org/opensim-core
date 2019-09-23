%% buildWalker.m
%   Script to programmatically build a Passive Dynamic Walker osimModel
%   For Tutorial 'From the Ground Up: Building a Passive Dynamic Walker'

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): James Dunne, Andrew Miller, Jeff Reinbolt, Ajay Seth...    %
%            Daniel A. Jacobs, Chris Dembia. Ayman Habib                %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

%% Clear Workspace
clear all; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%% Define key model variables
pelvisWidth = 0.20;
thighLength = 0.40;
shankLength = 0.435;

%% Instantiate an (empty) OpenSim Model
osimModel = Model();
osimModel.setName('DynamicWalkerModel');

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
osimModel.setGravity(Vec3(0, -9.80665, 0));

% TODO: Construct Bodies and Joints Here
% ********** BEGIN CODE **********


% **********  END CODE  **********

% TODO: Construct ContactGeometry and HuntCrossleyForces Here
% ********** BEGIN CODE **********


% **********  END CODE  **********

% TODO: Construct CoordinateLimitForces Here
% ********** BEGIN CODE **********


% **********  END CODE  **********

%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
osimModel.print('DynamicWalkerModel.osim');
display(['DynamicWalkerModel.osim printed!']);
