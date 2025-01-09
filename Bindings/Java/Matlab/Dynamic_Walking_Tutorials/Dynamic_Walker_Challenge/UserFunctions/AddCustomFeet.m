% This file demonstrates how to add feet to the model using a custom
% object.
%
% The model will look for .object files in two locations
% 1) The model's local directory
% 2) The Models directory in the OpenSim Installation Directory
% e.g. (<OpenSim_Home>\Models)
%
% The allowed object extensions are .vtp, .stl, .obj

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs
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

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
walkerModel = Model(model_path);
% Change the name
walkerModel.setName('WalkerModelTerrain_CustomFeet');

%% Define the foot mesh name
meshPath = 'ThinHalfCylinder100mmby50mm.obj';
% If the foot mesh isn't in the Model folder, make a copy. Otherwise mesh
% will not be usable when Model is openned in the OpenSim GUI
if ~exist('../Model/ThinHalfCylinder100mmby50mm.obj','file')
    copyfile 'ThinHalfCylinder100mmby50mm.obj' '../Model/ThinHalfCylinder100mmby50mm.obj'
end

%% Remove the current foot forces
forceNames = [{'LFootForce'} {'RFootForce'}];

% Check tosee if the model has feet forces
for i = 1 : length(forceNames)
    idx = walkerModel.getForceSet().getIndex( forceNames{i} );
    if (idx < 0), error(['Force, ' forceNames{i} ', does not exist in the Model']); end
    isSuccessful = walkerModel.updForceSet().remove(idx);
    if (~isSuccessful), error(['The Force, ' forceNames{i} ', as not able to be removed from Model']); end
end

% Remove the current (foot) contact geometry
geometryNames = [{'LFootContact'} {'RFootContact'}];
for i = 1 : length(geometryNames)

    idx = walkerModel.getContactGeometrySet().getIndex(geometryNames{i});
    if (idx < 0), error(['COntact Geometry, ' geometryNames{i} ', does not exist in the Model']); end
    isSuccessful = walkerModel.updContactGeometrySet().remove(idx);
    if (~isSuccessful), error(['The Contact Geometry, ' geometryNames{i} ',was not able to be removed from Model']); end
end

%% Add Bodies and joints for two feet
% Make feet Bodies
mass = 0.0001;
massCenter = Vec3(0);
inertia = Inertia(1,1,.0001,0,0,0);

leftFoot = Body('LeftFoot', mass , massCenter, inertia);
rightFoot = Body('RightFoot', mass , massCenter, inertia );

% Add visual objects of the feet
leftFoot.attachGeometry( Mesh(meshPath) );
rightFoot.attachGeometry( Mesh(meshPath) );

% Get a reference to the shank bodies
leftShank= walkerModel.getBodySet().get('LeftShank');
rightShank = walkerModel.getBodySet().get('RightShank');

% Make Weld Joints for the feet
locationInParent = Vec3(0.075,-0.2,0);
orientationInParent = Vec3(0);
locationInChild = Vec3(0);
orientationInChild = Vec3(0);

ankle_l = WeldJoint('ankle_l',leftShank, locationInParent, orientationInParent, leftFoot, locationInChild, orientationInChild);
ankle_r = WeldJoint('ankle_r',rightShank, locationInParent,orientationInParent, rightFoot, locationInChild, orientationInChild);


% Add the bodies and joints to the model
walkerModel.addBody(leftFoot);
walkerModel.addBody(rightFoot);
walkerModel.addJoint(ankle_l);
walkerModel.addJoint(ankle_r);

% Make a contact mesh for each foot
contact_l = ContactMesh(meshPath,Vec3(0,0,0), Vec3(0,0,0), leftFoot, 'LFootContact');
contact_r = ContactMesh(meshPath,Vec3(0,0,0), Vec3(0,0,0), rightFoot, 'RFootContact');

% Add contact geometry to the model
walkerModel.addContactGeometry(contact_l);
walkerModel.addContactGeometry(contact_r);

%% Make elastic foundation forces for both feet
elasticforce_l = ElasticFoundationForce();
elasticforce_r = ElasticFoundationForce();

% Set elastic foundation force names
elasticforce_l.setName('LFootForce');
elasticforce_r.setName('RFootForce');

% Define the force parameters
stiffness           = 1.0E6;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
translationVelocity = 0.1;

% set the contact parameters
elasticforce_l.addGeometry('LFootContact');
elasticforce_l.addGeometry('PlatformContact');
elasticforce_l.setStiffness(stiffness);
elasticforce_l.setDissipation(dissipation);
elasticforce_l.setStaticFriction(staticFriction);
elasticforce_l.setDynamicFriction(dynamicFriction);
elasticforce_l.setViscousFriction(viscousFriction);
elasticforce_l.setTransitionVelocity(translationVelocity);

elasticforce_r.addGeometry('RFootContact');
elasticforce_r.addGeometry('PlatformContact');
elasticforce_r.setStiffness(stiffness);
elasticforce_r.setDissipation(dissipation);
elasticforce_r.setStaticFriction(staticFriction);
elasticforce_r.setDynamicFriction(dynamicFriction);
elasticforce_r.setViscousFriction(viscousFriction);
elasticforce_r.setTransitionVelocity(translationVelocity);

% Add forces.
walkerModel.addForce(elasticforce_l);
walkerModel.addForce(elasticforce_r);

%% finalize connections
walkerModel.finalizeConnections()

%% Print the model to file.
newFilename = strrep(model_path, '.osim', '_CustomFeet.osim');
isSuccessful = walkerModel.print(newFilename);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');
