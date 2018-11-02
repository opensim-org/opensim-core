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

% Import Java Library
import org.opensim.modeling.*

% model file
modelPath = '../Model/WalkerModel.osim';

% open the model
model = Model(modelPath);

% change the name
model.setName( [model.getName().toCharArray()' '_CustomFeet']);

% remove the current foot forces
forceNames = [{'LFootForce'} {'RFootForce'}];

for i = 1 : length(forceNames)
    idx = model.getForceSet().getIndex( forceNames{i} );
    if (idx < 0), error(['Force, ' forceNames{i} ', does not exist in the Model']); end
    isSuccessful = model.updForceSet().remove(idx);
    if (~isSuccessful), error(['The Force, ' forceNames{i} ', as not able to be removed from Model']); end
end

% remove the current (foot) contact geometry
geometryNames = [{'LFootContact'} {'RFootContact'}];
for i = 1 : length(geometryNames)
    
    idx = model.getContactGeometrySet().getIndex(geometryNames{i}); 
    if (idx < 0), error(['COntact Geometry, ' geometryNames{i} ', does not exist in the Model']); end
    isSuccessful = model.updContactGeometrySet().remove(idx);
    if (~isSuccessful), error(['The Contact Geometry, ' geometryNames{i} ',was not able to be removed from Model']); end
end

% make rigid bodies for feet
leftFoot = Body('LeftFoot', 0.0001 , Vec3(0), Inertia(1,1,.0001,0,0,0) );
rightFoot = Body('RightFoot', 0.0001 , Vec3(0), Inertia(1,1,.0001,0,0,0) );

% get a reference to each shank
leftShank= model.getBodySet().get('LeftShank');
rightShank = model.getBodySet().get('RightShank');

% weld feet to shanks
ankle_l = WeldJoint('ankle_l',leftShank, Vec3(0.075,-0.2,0), Vec3(0,0,0), leftFoot, Vec3(0,0,0), Vec3(0,0,0));
ankle_r = WeldJoint('ankle_r',rightShank, Vec3(0.075,-0.2,0),Vec3(0,0,0), rightFoot, Vec3(0,0,0), Vec3(0,0,0));

% add the visual object
leftFoot.attachGeometry( Mesh('ThinHalfCylinder100mmby50mm.obj') );
rightFoot.attachGeometry( Mesh('ThinHalfCylinder100mmby50mm.obj') );

% add the bodies and joints to the model
model.addBody(leftFoot);
model.addBody(rightFoot);
model.addJoint(ankle_l);
model.addJoint(ankle_r);

% make a contact mesh for each foot
contact_l = ContactMesh('ThinHalfCylinder100mmby50mm.obj',Vec3(0,0,0), Vec3(0,0,0), leftFoot, 'LFootContact');
contact_r = ContactMesh('ThinHalfCylinder100mmby50mm.obj',Vec3(0,0,0), Vec3(0,0,0), rightFoot, 'RFootContact');

% add contact geometry
model.addComponent(contact_l);
model.addComponent(contact_r);

% make an elastic foundation force for both feet
elasticforce_l = ElasticFoundationForce();
elasticforce_r = ElasticFoundationForce();

% set names
elasticforce_l.setName('LFootForce');
elasticforce_r.setName('RFootForce');

% set transition velocity
elasticforce_l.setTransitionVelocity(0.1);
elasticforce_r.setTransitionVelocity(0.1);

% define contact parameters
stiffness           = 1.0E6;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;

% set the contact parameters
elasticforce_l.addGeometry('LFootContact');
elasticforce_l.addGeometry('PlatformContact');
elasticforce_l.setStiffness(stiffness);
elasticforce_l.setDissipation(dissipation);
elasticforce_l.setStaticFriction(staticFriction);
elasticforce_l.setDynamicFriction(dynamicFriction);
elasticforce_l.setViscousFriction(viscousFriction);

elasticforce_r.addGeometry('RFootContact');
elasticforce_r.addGeometry('PlatformContact');
elasticforce_r.setStiffness(stiffness);
elasticforce_r.setDissipation(dissipation);
elasticforce_r.setStaticFriction(staticFriction);
elasticforce_r.setDynamicFriction(dynamicFriction);
elasticforce_r.setViscousFriction(viscousFriction);

% add forces.
model.addForce(elasticforce_l);
model.addForce(elasticforce_r);

% finalize connections
model.finalizeConnections()

% print new model to file.
newFilename = 'WalkerModel_customFeet.osim';
isSuccessful = model.print(['../Model/',newFilename]);
if (~isSuccessful), error('Model printed to file failed'); end
fprintf('Model printed to file successfully\n');