% This script demonstrates how to add a PathSpring to a model.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Jen Hicks
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

% NOTE: In this sample code, we've used arbitrary parameters. Tweak them to get
% your desired result!

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
walkerModel = Model(model_path);
% Change the name
walkerModel.setName('WalkerModelTerrain_PathSpring');

%% Create a Path Spring on the right leg
restLength = 1.0;
stiffness = 100;
dissipation = 0.01;
rightSpring = PathSpring('path_spring_r',restLength,stiffness,dissipation);

% Define the geometry path
rightShankBody = walkerModel.getBodySet().get('RightShank');
rightThighBody = walkerModel.getBodySet().get('RightThigh');
rightSpring.updGeometryPath().appendNewPathPoint('right_shank',rightShankBody,Vec3(0,0,0));
rightSpring.updGeometryPath().appendNewPathPoint('right_thigh',rightThighBody,Vec3(0,0,0));

% Add the force to the model
walkerModel.addForce(rightSpring);

%% Create a Path Spring on the left leg
restLength = 1.0;
stiffness = 1000;
dissipation = 0.01;
leftSpring = PathSpring('path_spring_l',restLength,stiffness,dissipation);

% Define the geometry path
leftShankBody = walkerModel.getBodySet().get('LeftShank');
leftThighBody = walkerModel.getBodySet().get('LeftThigh');
leftSpring.updGeometryPath().appendNewPathPoint('left_shank',leftShankBody,Vec3(0,0,0));
leftSpring.updGeometryPath().appendNewPathPoint('left_thigh',leftThighBody,Vec3(0,0,0));

% Add the force to the model
walkerModel.addForce(leftSpring);

%% Finalize connections
walkerModel.finalizeConnections()

%% Print a new model file
newFilename = strrep(model_path, '.osim', '_PathSpring.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
