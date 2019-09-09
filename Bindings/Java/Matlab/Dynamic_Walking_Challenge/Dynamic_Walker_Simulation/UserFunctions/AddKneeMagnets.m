% This script demonstraties how to add a ExpressionBasedPointToPointForce
% to a model.
%
% The ExpressionPointToPointForce calculates the relative translation and
% relative velocity between two points in the global frame and allows the
% user to specify a force based on a string of the symbols d and ddot.

%-----------------------------------------------------------------------  %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
% Author(s): Daniel A. Jacobs
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

%% Instantiate the Model
model_path = '../Model/WalkerModelTerrain.osim';
walkerModel = Model(model_path);
% Change the name
walkerModel.setName('WalkerModelTerrain_KneeMagnet');

% Create an ExpressionBasedPointToPointForce for the magnets
leftKneeMagnet = ExpressionBasedPointToPointForce();
leftKneeMagnet.setName('LeftKneeMagnet');
rightKneeMagnet = ExpressionBasedPointToPointForce();
rightKneeMagnet.setName('RightKneeMagnet')

% Define the point locations for where the PointToPointForce will attach
locThigh = Vec3(0.05,-0.2, 0);
locShank = Vec3(0.05, 0.2, 0);

% Set the first body and the reference frame properties
leftKneeMagnet.setBody1Name('LeftThigh');
leftKneeMagnet.setPoint1(locThigh);
rightKneeMagnet.setBody1Name('RightThigh');
rightKneeMagnet.setPoint1(locThigh);

% Set the second body and the location of the reference point
leftKneeMagnet.setBody2Name('LeftShank');
leftKneeMagnet.setPoint2(locShank);
rightKneeMagnet.setBody2Name('RightShank');
rightKneeMagnet.setPoint2(locShank);

% Set the expression to represent a magnet (0.01/d^2) force.
% d is the distance between the two points of ExpressionBasedPointToPointForce
leftKneeMagnet.setExpression('0.01/d^2');
rightKneeMagnet.setExpression('0.01/d^2');

% Add the force to the model
walkerModel.addComponent(leftKneeMagnet);
walkerModel.addComponent(rightKneeMagnet);

% Finalize connections
walkerModel.finalizeConnections()

%% Print a new model file
newFilename = strrep(model_path, '.osim', '_KneeMagnet.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
