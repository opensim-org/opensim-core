% This script demonstrates how to add a ExpressionBasedPointToPointForce
% to a model.
%
% The ExpressionPointToPointForce calculates the relative translation and
% relative velocity between two points in the global frame and allows the
% user to specify a force based on a string of the symbols d and ddot.

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
% Author(s): Daniel A. Jacobs                                             %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

%% Import OpenSim Libraries
import org.opensim.modeling.*

%%  Define some variables
% Define the point location in each body's coordinate frame
locThigh = Vec3(0.05,-0.2, 0);
locShank = Vec3(0.05, 0.2, 0);

% Define the body Names for the ExpressionBasedPointToPointForce
parentBodyName = 'RightThigh';
childBodyName  = 'RightShank';

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
walkerModel = Model(model_path);
% Change the name
walkerModel.setName('WalkerModelTerrain_KneeMagnet');

%% Create an ExpressionBasedBushingForce for the Right Knee Coordinate
rightKneeMagnet = ExpressionBasedPointToPointForce();
rightKneeMagnet.setName('RightKneeMagnet')

% Get references to model bodies
parentBody = walkerModel.getBodySet().get(parentBodyName);
childBody  = walkerModel.getBodySet().get(childBodyName);

% Set body references and point locations for the ExpressionBasedBushingForce
rightKneeMagnet.setBody1Name( parentBody.getName() );
rightKneeMagnet.setPoint1(locThigh);
rightKneeMagnet.setBody2Name( childBody.getName() );
rightKneeMagnet.setPoint2(locShank);

% Set the expression to represent a magnet (0.01/d^2) force.
rightKneeMagnet.setExpression('0.01/d^2');

% Add the Right Knee Magnet force to the Model
walkerModel.addForce(rightKneeMagnet);

%% TODO: Create an ExpressionBasedBushingForce for the Left Knee Coordinate

% TODO: Set body references and point locations for the ExpressionBasedBushingForce

% TODO: Set the expression to represent a magnet (0.01/d^2) force.

% TODO: Add the Left Knee Magnet force to the Model

%% Finalize connections
walkerModel.finalizeConnections()

%% Print the model to file.
newFilename = strrep(model_path, '.osim', '_KneeMagnet.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
