% This script demonstrates how to add a Millard2012EquilibriumMuscle
% to a model.

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
walkerModel.setName('WalkerModelTerrain_MillardMuscle');

%% Create a muscle on the right leg
maxIsometricForce = 500;
optimalFiberLength = 0.15;
tendonSlackLength = 0.1;
pennationAngle = 0.3;
rightMuscle = Millard2012EquilibriumMuscle('muscle_r',maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

% Define the geometry path
rightShankBody = walkerModel.getBodySet().get('RightShank');
rightThighBody = walkerModel.getBodySet().get('RightThigh');
rightMuscle.updGeometryPath().appendNewPathPoint('right_shank_attachment',rightShankBody,Vec3(0,0,0));
rightMuscle.updGeometryPath().appendNewPathPoint('right_thigh_attachment',rightThighBody,Vec3(0,0,0));

% Add the force to the model
walkerModel.addForce(rightMuscle);

% Create a muscle on the left leg using the same params
leftMuscle = Millard2012EquilibriumMuscle('muscle_l',maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

% Define the geometry path
leftShankBody = walkerModel.getBodySet().get('LeftShank');
leftThighBody = walkerModel.getBodySet().get('LeftThigh');
leftMuscle.updGeometryPath().appendNewPathPoint('left_shank_attachment',leftShankBody,Vec3(0,0,0));
leftMuscle.updGeometryPath().appendNewPathPoint('left_thigh_attachment',leftThighBody,Vec3(0,0,0));

% Add the force to the model
walkerModel.addForce(leftMuscle);

%% Finalize connections
walkerModel.finalizeConnections()

%% Print a new model file
newFilename = strrep(model_path, '.osim', '_MillardMuscle.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
