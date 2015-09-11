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
% This script adds a PathActuator that spans the left knee.
% https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1PathActuator.html

clc;

% Import Java library.
import org.opensim.modeling.*

% Open the model.
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Change the name.
walkerModel.setName('DW2013_WalkerModelTerrain_CoordAct');

% Display all bodies in the model.
numBodies = walkerModel.getNumBodies();
fprintf('There are %d bodies in the model:\n',numBodies);
for i=0:numBodies-1
    fprintf('\t(%d) %s\n',i,char(walkerModel.getBodySet().get(i)));
end

% Create and configure a PathActuator that spans the left knee.
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
walkerModel.addForce(pathAct);

% Save the new model file.
modelFile_new = '../Model/DW2013_WalkerModelTerrain_PathAct.osim';
walkerModel.print(modelFile_new);
fprintf('Model saved to %s\n',modelFile_new);
