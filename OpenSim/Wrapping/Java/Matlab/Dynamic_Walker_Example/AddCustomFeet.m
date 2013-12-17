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

% Open the model
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Change the name
walkerModel.setName('DW2013_WalkerModelTerrainCustomFeet');

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
% ***** WRITE YOUR CODE HERE:  *****
% Task: Declare a leftFoot and a rightFoot variable and assign them a
% object with the constructor Body().  Use the "set" functions to set the
% name, the mass and the inertia.


% Set up the parameters for the Weld Joint connecting the feet to the
% shanks
% ***** WRITE YOUR CODE HERE:  *****
% Task: Create the Vec3 for the location and orientation of the parent and 
% child using the constructor Vec3(x,y,z)

% Get a reference to each shank
leftShankBody = walkerModel.updBodySet().get('LeftShank');
rightShankBody = walkerModel.updBodySet().get('RightShank');

% ***** WRITE YOUR CODE HERE:  *****
% Task: Uncomment everything below these lines when you are finished with the above tasks.
% Create Weld Joint
% lFootToLShank    = WeldJoint('LFootToLShank', leftShankBody, locationInParent, ...
%     orientationInParent, leftFoot, locationInChild, orientationInChild, false);
% rFootToRShank    = WeldJoint('RFootToRShank', rightShankBody, locationInParent, ...
%     orientationInParent, rightFoot, locationInChild, orientationInChild, false);
% 
% % Add the Visual Object
% leftFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');
% rightFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');
% 
% % Add the body to the Model
% walkerModel.addBody(leftFoot);
% walkerModel.addBody(rightFoot);
% ----------------------------------------------------------------------- 
% % Create a ContactMesh for each foot
% footMeshLocation    = Vec3(0,0,0);
% footMeshOrientation = Vec3(0,0,0);
% leftFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
%     footMeshLocation, footMeshOrientation, leftFoot, 'LFootContact');
% rightFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
%     footMeshLocation, footMeshOrientation, rightFoot, 'RFootContact');
% 
% % Add ContactGeometry
% walkerModel.addContactGeometry(leftFootContact);
% walkerModel.addContactGeometry(rightFootContact);
% % ----------------------------------------------------------------------- 
% % Create an elastic foundation force for both feet
% leftElasticFootForce = ElasticFoundationForce();
% rightElasticFootForce = ElasticFoundationForce();
% 
% % Set Names
% leftElasticFootForce.setName('LeftElasticFootForce');
% rightElasticFootForce.setName('RightElasticFootForce');
% 
% % Set transition velocity
% leftElasticFootForce.setTransitionVelocity(0.1);
% rightElasticFootForce.setTransitionVelocity(0.1);
% 
% % Define Contact Parameters
% stiffness           = 1.0E6;
% dissipation         = 2.0;
% staticFriction      = 0.8;
% dynamicFriction     = 0.4;
% viscousFriction     = 0.4;
% 
% % Set the Contact parameters for the forces 
% leftElasticFootForce.addGeometry('LFootContact');
% leftElasticFootForce.addGeometry('PlatformContact');
% leftElasticFootForce.setStiffness(stiffness);
% leftElasticFootForce.setDissipation(dissipation);
% leftElasticFootForce.setStaticFriction(staticFriction);
% leftElasticFootForce.setDynamicFriction(dynamicFriction);
% leftElasticFootForce.setViscousFriction(viscousFriction);
% 
% rightElasticFootForce.addGeometry('RFootContact');
% rightElasticFootForce.addGeometry('PlatformContact');
% rightElasticFootForce.setStiffness(stiffness);
% rightElasticFootForce.setDissipation(dissipation);
% rightElasticFootForce.setStaticFriction(staticFriction);
% rightElasticFootForce.setDynamicFriction(dynamicFriction);
% rightElasticFootForce.setViscousFriction(viscousFriction);
% 
% % Add Forces
% walkerModel.addForce(leftElasticFootForce);
% walkerModel.addForce(rightElasticFootForce);
% % ----------------------------------------------------------------------- 
% % Save the new model
% walkerModel.print('../Model/DW2013_WalkerModelTerrainAddCustomFeet.osim');                  