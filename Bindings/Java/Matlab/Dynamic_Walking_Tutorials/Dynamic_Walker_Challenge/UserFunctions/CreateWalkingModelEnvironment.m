%% Script to generate a Passive Dynamic Walking Model Course.
%  Adds contact Spheres to a Passive Dynamic Walker plateform. Placement is
%  done randomly.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs
% Contributor(s): Hannah O'Day, Chris Dembia
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% http://www.apache.org/licenses/LICENSE-2.0.
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
% implied. See the License for the specific language governing
% permissions and limitations under the License.
% -----------------------------------------------------------------------

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Get a path to one.
[filename,pathname] = uigetfile('*.osim', 'Select an OpenSim Passive Dynamic Walker Model File (.osim)');
display('Loading the model...');

modelpath = fullfile(pathname, filename);

% Model Body Parameters
PlatformLength  = 10;
PlatformHeight   = 0.05;
PlatformOffset  = 0.5;
PelvisWidth = 0.20;

% Add Obstacles
addObstacles = true;

% Obstacle Contact Parameters
numSpheres = 40;
minObstacleRadius = 0.05;
maxObstacleRadius = 0.08;
startingPoint = 3.0; % Min X Value for Spheres
endingPoint = PlatformLength; % Max X Value for Spheres
distance = endingPoint - startingPoint;

% Set the Matlab random number generator to use Mersenne Twister
rng(0, 'twister');

%% Instantiate Passive Dynamic Walker Model
osimModel = Model(modelpath);

%% Get a reference to the platform body
platform = osimModel.getBodySet().get('Platform');

%% Add Hunt Crossley Forces
% Contact Parameters
stiffness           = 1.0E6;
dissipation         = 1.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;

% Create Hunt Crossley Obstacle Forces
ObstacleForces = HuntCrossleyForce();
ObstacleForces.setName('ObstacleForces');
ObstacleForces.setStiffness(stiffness);
ObstacleForces.setDissipation(dissipation);
ObstacleForces.setStaticFriction(staticFriction);
ObstacleForces.setDynamicFriction(dynamicFriction);
ObstacleForces.setViscousFriction(viscousFriction);
ObstacleForces.setTransitionVelocity(0.2);
ObstacleForces.addGeometry('LFootContact');
ObstacleForces.addGeometry('RFootContact');

%% Add Contact Spheres
for i = 1:1:numSpheres
    radius = minObstacleRadius+(maxObstacleRadius - minObstacleRadius)*rand(1,1);
    locX = endingPoint-distance*abs(.30*randn(1,1))-PlatformOffset;
    locY = PlatformHeight/2-radius*0.8;
    locZ = PelvisWidth/2*(sign(-0.5+rand(1,1)));
    sphere = ContactSphere(radius, Vec3(locX,locY,locZ), platform);
    name = ['Obstacle',num2str(i)];
    sphere.setName(name);
    osimModel.addContactGeometry(sphere);
    ObstacleForces.addGeometry(name);
end

%% Add Force to the Model
osimModel.addForce(ObstacleForces);

%% Ensure there are no errors with how the model was built.
osimModel.initSystem();

%% Print Model to file.
% Get the file parts
[pathname,filename,ext] = fileparts(modelpath);
% Make the print path
printPath = fullfile(pathname, [filename 'withTerrain.osim']);
% Print the actuators xml file
osimModel.print(printPath);
%% Display printed file
display(['Printed model to ' printPath])
