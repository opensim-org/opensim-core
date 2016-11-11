% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2013 Stanford University and the Authors             
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
%----------------------------------------------------------------
% User Inputs 
%Model File
modelDirectory = '../Model/';
modelFile = 'WalkerModelTerrain.osim';
resultsDirectory = '../Model/';
%----------------------------------------------------------------
% Import Java Library 
import org.opensim.modeling.*

% Open the model
walkerModel = Model([modelDirectory, modelFile]);

% Change the name
modelPrefix = walkerModel.getName();
modelName = [modelPrefix.toCharArray()','AddCustomFeet'];
walkerModel.setName(modelName);

% Remove the current foot forces
lFootIndex = walkerModel.getForceSet().getIndex('LFootForce');
walkerModel.updForceSet().remove(lFootIndex);
rFootIndex = walkerModel.getForceSet().getIndex('RFootForce');
walkerModel.updForceSet().remove(rFootIndex);

% Remove the current foot contact geo
lContactIndex = walkerModel.getContactGeometrySet().getIndex('LFootContact');
walkerModel.updContactGeometrySet().remove(lContactIndex);
rContactIndex = walkerModel.getContactGeometrySet().getIndex('RFootContact');
walkerModel.updContactGeometrySet().remove(rContactIndex);
% ----------------------------------------------------------------------- 
% Setup the new foot paramters
footMass    = 0.0001;
footInertia = Inertia(1,1,.0001,0,0,0);

% Create the leftFoot and rightFoot bodies
leftFoot = Body();
leftFoot.setName('LeftFoot');
leftFoot.setMass(footMass);
leftFoot.setInertia(footInertia);

rightFoot = Body();
rightFoot.setName('RightFoot');
rightFoot.setMass(footMass);
rightFoot.setInertia(footInertia);

% Set up the parameters for the Weld Joint connecting the feet to the
% shanks
locationInParent    = Vec3(0.075,-0.2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);

% Get a reference to each shank
leftShankBody = walkerModel.updBodySet().get('LeftShank');
rightShankBody = walkerModel.updBodySet().get('RightShank');

% Create Weld Joint
lFootToLShank    = WeldJoint('LFootToLShank', leftShankBody, locationInParent, ...
    orientationInParent, leftFoot, locationInChild, orientationInChild, false);
rFootToRShank    = WeldJoint('RFootToRShank', rightShankBody, locationInParent, ...
    orientationInParent, rightFoot, locationInChild, orientationInChild, false);

% Add the Visual Object
leftFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');
rightFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');

% Add the body to the Model
walkerModel.addBody(leftFoot);
walkerModel.addBody(rightFoot);
% ----------------------------------------------------------------------- 
% Create a ContactMesh for each foot
footMeshLocation    = Vec3(0,0,0);
footMeshOrientation = Vec3(0,0,0);
leftFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
    footMeshLocation, footMeshOrientation, leftFoot, 'LFootContact');
rightFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
    footMeshLocation, footMeshOrientation, rightFoot, 'RFootContact');

% Add ContactGeometry
walkerModel.addContactGeometry(leftFootContact);
walkerModel.addContactGeometry(rightFootContact);
% ----------------------------------------------------------------------- 
% Create an elastic foundation force for both feet
leftElasticFootForce = ElasticFoundationForce();
rightElasticFootForce = ElasticFoundationForce();

% Set Names
leftElasticFootForce.setName('LFootForce');
rightElasticFootForce.setName('RFootForce');

% Set transition velocity
leftElasticFootForce.setTransitionVelocity(0.1);
rightElasticFootForce.setTransitionVelocity(0.1);

% Define Contact Parameters
stiffness           = 1.0E6;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;

% Set the Contact parameters for the forces 
leftElasticFootForce.addGeometry('LFootContact');
leftElasticFootForce.addGeometry('PlatformContact');
leftElasticFootForce.setStiffness(stiffness);
leftElasticFootForce.setDissipation(dissipation);
leftElasticFootForce.setStaticFriction(staticFriction);
leftElasticFootForce.setDynamicFriction(dynamicFriction);
leftElasticFootForce.setViscousFriction(viscousFriction);

rightElasticFootForce.addGeometry('RFootContact');
rightElasticFootForce.addGeometry('PlatformContact');
rightElasticFootForce.setStiffness(stiffness);
rightElasticFootForce.setDissipation(dissipation);
rightElasticFootForce.setStaticFriction(staticFriction);
rightElasticFootForce.setDynamicFriction(dynamicFriction);
rightElasticFootForce.setViscousFriction(viscousFriction);

% Add Forces
walkerModel.addForce(leftElasticFootForce);
walkerModel.addForce(rightElasticFootForce);
% ----------------------------------------------------------------------- 
% Save the new model
walkerModel.print([resultsDirectory,modelName,'.osim']);
status = 0;                 