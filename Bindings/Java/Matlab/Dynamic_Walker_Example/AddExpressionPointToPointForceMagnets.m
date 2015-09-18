% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2013 Stanford University and the Authors             %
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
% This script uses a ExpressionBasedPointToPointForce to realize a magnet 
% force model.  The ExpressionPointToPointForce calculates the relative 
% translation and relative velocity between two points in the global frame 
% and allows the user to specify a force based on a string of the symbols 
% d and ddot. 

% Import Java Library 
import org.opensim.modeling.*

% Open the model
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Change the name
walkerModel.setName('DW2013_WalkerModelTerrainAddMagnet');

% Define the body and points location in each body's coordinate frame
locBody1 = Vec3(0.05,-0.2, 0);
locBody2 = Vec3(0.05, 0.2, 0);

% Create an ExpressionBasedBushingForce for the magnets
leftKneeMagnet = ExpressionBasedPointToPointForce();
leftKneeMagnet.setName('LeftKneeMagnet');
rightKneeMagnet = ExpressionBasedPointToPointForce();
rightKneeMagnet.setName('RightKneeMagnet')

% Set the first body and the reference frame properties
% ***** WRITE YOUR CODE HERE:  *****
% Task: Navigate to your local doxygen and find the methods to set the name
% of Body1 and set the location of Point1.  


% Set the second body and the location of the reference point
% ***** WRITE YOUR CODE HERE:  *****
% Task: Set the name of Body2 and the location of Point2.


% Set the expression to represent a magnet (1/d^2) force
% ***** WRITE YOUR CODE HERE:  *****
% Task: Use the setExpression method for each magnet and pass an char type 
% argument of the desired function (0.01/d^2).

% ***** WRITE YOUR CODE HERE:  *****
% Task: Uncomment these lines when you are finished with the above tasks.
% Add the force to the model
% walkerModel.addForce(leftKneeMagnet);
% walkerModel.addForce(rightKneeMagnet);

% Print a new model file
walkerModel.print('../Model/DW2013_WalkerModelTerrainAddMagnet.osim');