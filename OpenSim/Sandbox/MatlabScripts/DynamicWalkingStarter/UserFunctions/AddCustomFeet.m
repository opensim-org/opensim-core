% Import Java Library 
import org.opensim.modeling.*

% Model File
modelDirectory = '../Model/WalkerModel.osim';

% Open the model
model = Model(modelDirectory);

% Change the name
model.setName( [model.getName().toCharArray()' '_CustomFeet']);

% Remove the current foot forces
model.updForceSet().remove( model.getForceSet.get('LFootForce') );
model.updForceSet().remove( model.getForceSet.get('RFootForce') );

% Remove the current foot contact geo
model.updContactGeometrySet().remove( model.getContactGeometrySet().get('LFootContact') );
model.updContactGeometrySet().remove( model.getContactGeometrySet().get('RFootContact') );

% make new the feet bodies
leftFoot = Body('LeftFoot', 0.0001 , Vec3(0), Inertia(1,1,.0001,0,0,0) );
rightFoot = Body('RightFoot', 0.0001 , Vec3(0), Inertia(1,1,.0001,0,0,0) );


% Get a reference to each shank
leftShank= model.updBodySet().get('LeftShank');
rightShank = model.updBodySet().get('RightShank');

% make weld joints
ankle_l = WeldJoint('ankle_l',leftShank, Vec3(0.075,-0.2,0), ...
                    Vec3(0,0,0), leftFoot, Vec3(0,0,0), Vec3(0,0,0));
ankle_r = WeldJoint('ankle_r',rightShank, Vec3(0.075,-0.2,0), ...
                    Vec3(0,0,0), rightFoot, Vec3(0,0,0), Vec3(0,0,0));

% Add the Visual Object
leftFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');
rightFoot.addDisplayGeometry('ThinHalfCylinder100mmby50mm.obj');

% Add the body to the Model
model.addComponent(leftFoot);
model.addComponent(rightFoot);
% ----------------------------------------------------------------------- 
% make a ContactMesh for each foot
footMeshLocation    = Vec3(0,0,0);
footMeshOrientation = Vec3(0,0,0);
leftFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
    footMeshLocation, footMeshOrientation, leftFoot, 'LFootContact');
rightFootContact = ContactMesh('ThinHalfCylinder100mmby50mm.obj', ...
    footMeshLocation, footMeshOrientation, rightFoot, 'RFootContact');

% Add ContactGeometry
model.addContactGeometry(leftFootContact);
model.addContactGeometry(rightFootContact);
% ----------------------------------------------------------------------- 
% make an elastic foundation force for both feet
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
model.addForce(leftElasticFootForce);
model.addForce(rightElasticFootForce);
% ----------------------------------------------------------------------- 
% Save the new model
model.print([resultsDirectory,modelName,'.osim']);
status = 0;     

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