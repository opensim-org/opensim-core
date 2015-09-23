% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2013 Stanford University and the Authors
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
% This script adds a CoordinateActuator to the left knee.
% simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1CoordinateActuator.html

% Import Java library.
import org.opensim.modeling.*

% Open the model.
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Change the name.
walkerModel.setName('DW2013_WalkerModelTerrain_CoordAct');

% Display all coordinates in the model.
numCoords = walkerModel.getNumCoordinates();
fprintf('There are %d coordinates in the model:\n',numCoords);
for i=0:numCoords-1
    fprintf('\t(%d) %s\n',i,char(walkerModel.getCoordinateSet().get(i)));
end

% Create and configure a CoordinateActuator for the left knee.
jointName = 'LKnee_rz';
fprintf('Adding CoordinateActuator to %s.\n',jointName);
coordAct = CoordinateActuator(jointName);
coordAct.setName('coordAct_LK');            % Name
coordAct.setOptimalForce(10.0);             % Maximum generalized force
coordAct.setMinControl(-inf);               % Minimum control signal allowed
coordAct.setMaxControl(inf);                % Maximum control signal allowed

% Add the force to the model.
walkerModel.addForce(coordAct);

% Save the new model file.
modelFile_new = '../Model/DW2013_WalkerModelTerrain_CoordAct.osim';
walkerModel.print(modelFile_new);
fprintf('Model saved to %s\n',modelFile_new);
