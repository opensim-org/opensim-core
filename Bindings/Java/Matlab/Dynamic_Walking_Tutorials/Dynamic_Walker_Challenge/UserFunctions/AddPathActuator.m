% This script demonstrates how to add a PathActuator to a model.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs, Tom Uchida
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

%% Import OpenSim Libraries.
import org.opensim.modeling.*

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
walkerModel = Model(model_path);
% Change the name.
walkerModel.setName('WalkerModelTerrain_PathActuator');

%% Display names of all bodies in the model.
numBodies = walkerModel.getNumBodies();
fprintf('There are %d bodies in the model:\n',numBodies);
for i=0:numBodies-1
    fprintf('\t(%d) %s\n',i,char(walkerModel.getBodySet().get(i)));
end

%% Create and configure a PathActuator that spans the left knee.
pathAct = PathActuator();
pathAct.setName('pathAct_LK');              % Name
pathAct.setOptimalForce(10.0);              % Maximum generalized force
pathAct.setMinControl(-inf);                % Minimum control signal allowed
pathAct.setMaxControl(inf);                 % Maximum control signal allowed

% Attach one end of the PathActuator to the origin of the left thigh.
body1 = walkerModel.getBodySet().get('LeftThigh');
point1 = Vec3(0,0,0);
pathAct.addNewPathPoint('pathAct_point1',body1,point1);

% Attach the other end of the PathActuator to the origin of the left shank.
body2 = walkerModel.getBodySet().get('LeftShank');
point2 = Vec3(0,0,0);
pathAct.addNewPathPoint('pathAct_point2',body2,point2);

% Add the force to the model.
walkerModel.addComponent(pathAct);

%% Finalize connections
walkerModel.finalizeConnections()

%% Print the model to file.
newFilename = strrep(model_path, '.osim', '_PathActuator.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
