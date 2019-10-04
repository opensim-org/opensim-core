% This file demonstrates how to add a CoordinateActuator to the model. In
% this case, the Left Knee of a Passive Dynamic Walker Model.

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

%% Import OpenSim Libraries.
import org.opensim.modeling.*

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
walkerModel = Model(model_path);
% Change the name
walkerModel.setName('WalkerModelTerrain_CoordinateActuator');

%% Display all coordinates in the model.
numCoords = walkerModel.getNumCoordinates();
fprintf('There are %d coordinates in the model:\n',numCoords);
for i=0:numCoords-1
    fprintf('\t(%d) %s\n',i,char(walkerModel.getCoordinateSet().get(i)));
end

%% Create and configure a CoordinateActuator for the left knee.
coordinateName = 'LKnee_rz';
fprintf('Adding CoordinateActuator to %s.\n',coordinateName);
coordAct = CoordinateActuator(coordinateName);
coordAct.setName('coordAct_LK');            % Name
coordAct.setOptimalForce(10.0);             % Maximum generalized force
coordAct.setMinControl(-inf);               % Minimum control signal allowed
coordAct.setMaxControl(inf);                % Maximum control signal allowed

% Add the force to the model.
walkerModel.addForce(coordAct);

%% Finalize connections
walkerModel.finalizeConnections()

%% Print the model to file.
newFilename = strrep(model_path, '.osim', '_CoordinateActuator.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
