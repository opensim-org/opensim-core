% This script demonstrates how to add a SpringGeneralizedForce to a model.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): The OpenSim Team
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
walkerModel.setName('WalkerModelTerrain_SpringGeneralizedForce');

%% Create the springs
rightSpring = SpringGeneralizedForce('RHip_rz');
leftSpring = SpringGeneralizedForce('LHip_rz');

% Set names of the forces.
rightSpring.setName('spring_right_hip');
leftSpring.setName('spring_left_hip');

% Set the params
stiffness = 10;
restLength = 0.01;
viscosity = 0.01;
rightSpring.setStiffness(stiffness);
rightSpring.setRestLength(restLength);
rightSpring.setViscosity(viscosity);
leftSpring.setStiffness(stiffness);
leftSpring.setRestLength(restLength);
leftSpring.setViscosity(viscosity);

% Add the forces to the model
walkerModel.addForce(rightSpring);
walkerModel.addForce(leftSpring);

%% Finalize connections
walkerModel.finalizeConnections()

%% Print a new model file
newFilename = strrep(model_path, '.osim', '_SpringGeneralizedForce.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
