%% buildWalker.m
%   Script to programmatically build a Passive Dynamic Walker osimModel
%   For Tutorial 'From the Ground Up: Building a Passive Dynamic Walker'

% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal osimModeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2019 Stanford University and the Authors                %
% Author(s): James Dunne, Tom Uchida, Chris Dembia                           %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% Author(s): James Dunne, Andrew Miller, Jeff Reinbold, Ajay Seth...
%            Daniel A. Jacobs, Chris Dembia.

%% Clear Workspace
clear all; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%% Instantiate an (empty) OpenSim Model
osimModel = Model();
% Name the Model
osimModel.setName('DynamicWalkerModel');
 
% Define the acceleration due to gravity
osimModel.setGravity( Vec3(0, -9.80665, 0) );

% Get handle to the Ground
ground = osimModel.getGround();

%% TODO: Add Bodies, Joints, & Display Geometry 
% Make and add a Platform Body
% Make and add a Weld joint for the Platform Body

% Make and add a Pelvis Body
% Make and add a Planar joint for the Pelvis Body

% Make and add a Right Thigh Body
% Make and add a Pin joint for the Right Thigh Body

% Make and add a Left Thigh Body
% Make and add a Pin joint for the Left Thigh Body

% Make and add a Right Shank Body
% Make and add a Pin joint for the Right Shank Body

% Make and add a Left Shank Body
% Make and add a Pin joint for the Left Shank Body

% Make and add a Right Foot Body
% Make and add a Weld joint for the Right Foot Body

% Make and add a Left Foot Body
% Make and add a Weld joint for the Left Foot Body

%% TODO: Add ContactGeometry 
% Make a Contact Half Space
% Make a Right Hip Contact
% Make a Left Hip Contact
% Make a Right Knee Contact
% Make a Left Knee Contact
% Make a Right Foot Contact Sphere
% Make a Left Foot Contact Sphere


%% TODO: Add Hunt Crossley Forces
% Define Contact Force Parameters
% Make a Hunt Crossley Force for Right Hip and update parameters
% Make a Hunt Crossley Force for Left Hip and update parameters
% Make a Hunt Crossley Force for Right Knee and update parameters
% Make a Hunt Crossley Force for Left Knee and update parameters
% Make a Hunt Crossley Force for Right Foot and update parameters
% Make a Hunt Crossley Force for Left Foot and update parameters


%% TODO: Add CoordinateLimitForces 
% Define Coordinate Limit Force Parameters
% Make a Right Hip Coordinate Limit Force
% Make a Left Hip Coordinate Limit Force
% Make a Right Knee Coordinate Limit Force
% Make a Left Knee Coordinate Limit Force

%% Initialize the System (checks model consistency). 
osimModel.initSystem();

%% Print osimModel to file
osimModel.print('Walker_osimModel.osim');
display(['osimModel Walker_osimModel.osim printed!']);
