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
%CreateWalkingModelAndEnvironment
%   This script creates a planar kneed walking model and randomly generates
%   40 spheres for obstacles
% -----------------------------------------------------------------------     

% User Define Constants for Model
% Body Parameters
PlatformLength  = 10;
PlatformWidth   = 0.05;
PlatformOffset  = 0.5;

PelvisMass      = 0.5;
PelvisWidth     = 0.4;

ThighMass       = 0.5;
ThighLength     = 0.5;
ThighIZZ        = 1/12*ThighMass*ThighLength^2;

ShankMass       = 0.05;
ShankLength     = 0.5;
ShankIZZ        = 1/12*ShankMass*ShankLength^2;

DEG2RAD         = pi/180;
RAD2DEG         = 1/DEG2RAD;

% Obstacle Contact Parameters
numSpheres = 40;
minObstacleRadius = 0.05;
maxObstacleRadius = 0.08;
startingPoint = 3.0; % Min X Value for Spheres
endingPoint = PlatformLength; % Max X Value for Spheres
distance = endingPoint - startingPoint;
% -----------------------------------------------------------------------
% Import Matlab Classes
import org.opensim.modeling.*

% Open Model
osimModel = Model();
osimModel.setName('DW2013_WalkerModelTerrain');
osimModel.setAuthors('Daniel A. Jacobs, Ajay Seth');

% Get a Handle to Ground Body
ground = osimModel.getGroundBody();

% -----------------------------------------------------------------------
% Platform
platform            = Body();
platform.setName('Platform');
platform.setMass(0);
platform.setInertia(Inertia(0,0,0,0,0,0));

% Create Weld Joint
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,-10*DEG2RAD);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
platformToGround    = WeldJoint('PlatformToGround', ground, locationInParent, ...
    orientationInParent, platform, locationInChild, orientationInChild, false);

% Add Visible Object for GUI
platform.addDisplayGeometry('box.vtp');
platform.updDisplayer().setScaleFactors([PlatformLength, PlatformWidth, 1]);
platform.updDisplayer().translate(Vec3(PlatformLength/2-PlatformOffset,0,0));

% Add Body to Model
osimModel.addBody(platform);
% -----------------------------------------------------------------------
% Create a massless body for one DOF of the Pelvis
pelvisMassless      = Body();
pelvisMassless.setName('Pelvis_massless');
pelvisMassless.setMass(0);
pelvisMassless.setInertia(Inertia(0,0,0,0,0,0));

% Create Slider Joint
locationInParent    = Vec3(0,PlatformWidth/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
pelvisMasslessToPlatform    = SliderJoint('PelvisMasslessToPlatform', platform, ...
    locationInParent, orientationInParent, pelvisMassless, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet  = pelvisMasslessToPlatform.getCoordinateSet();
jointCoordinateSet.get(0).setRange([0, 100]);
jointCoordinateSet.get(0).setName('Pelvis_tx');
jointCoordinateSet.get(0).setDefaultValue(0);

% Add Body to Model
osimModel.addBody(pelvisMassless);
% -----------------------------------------------------------------------
% Create a massive body for the last DOF of the pelvis
pelvis              = Body();
pelvis.setName('Pelvis');
pelvis.setMass(PelvisMass);
pelvis.setInertia(Inertia(1,1,1,0,0,0));

% Create a Slider Joint
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,pi/2);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,pi/2);
pelvisToPelvisMassless    = SliderJoint('PelvisToPelvisMassless', pelvisMassless, ...
    locationInParent, orientationInParent, pelvis, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet  = pelvisToPelvisMassless.getCoordinateSet();
jointCoordinateSet.get(0).setRange([0, 5]);
jointCoordinateSet.get(0).setName('Pelvis_ty');
jointCoordinateSet.get(0).setDefaultValue(1.0);

% Add Visible Object for GUI
pelvis.addDisplayGeometry('sphere.vtp');
pelvis.getDisplayer().setScaleFactors([PelvisWidth/5, PelvisWidth/5, PelvisWidth]);

% Add Body to Model
osimModel.addBody(pelvis);
% -----------------------------------------------------------------------
% Create the Left Thigh
leftThigh = Body();
leftThigh.setName('LeftThigh');
leftThigh.setMass(ThighMass);
leftThigh.setInertia(Inertia(1,1,ThighIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,-PelvisWidth/2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ThighLength/2,0);
orientationInChild  = Vec3(0,0,0);
LThighToPelvis = PinJoint('LThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, leftThigh, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet = LThighToPelvis.getCoordinateSet();
jointCoordinateSet.get(0).setRange([-100*DEG2RAD, 100*DEG2RAD]);
jointCoordinateSet.get(0).setName('LHip_rz');
jointCoordinateSet.get(0).setDefaultValue(60*DEG2RAD);

% Add Visible Object for GUI
leftThigh.addDisplayGeometry('sphere.vtp');
leftThigh.getDisplayer().setScaleFactors([ThighLength/10, ThighLength, ThighLength/10]);

% Add Body to Model
osimModel.addBody(leftThigh);
% -----------------------------------------------------------------------
% Create the Right Thigh
rightThigh = Body();
rightThigh.setName('RightThigh');
rightThigh.setMass(ThighMass);
rightThigh.setInertia(Inertia(1,1,ThighIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,PelvisWidth/2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ThighLength/2,0);
orientationInChild  = Vec3(0,0,0);
RThighToPelvis = PinJoint('RThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, rightThigh, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet = RThighToPelvis.getCoordinateSet();
jointCoordinateSet.get(0).setRange([-100*DEG2RAD, 100*DEG2RAD]);
jointCoordinateSet.get(0).setName('RHip_rz');
jointCoordinateSet.get(0).setDefaultValue(0*DEG2RAD);

% Add Visible Object for GUI
rightThigh.addDisplayGeometry('sphere.vtp');
rightThigh.getDisplayer().setScaleFactors([ThighLength/10, ThighLength, ThighLength/10]);

% Add Body to Model
osimModel.addBody(rightThigh);
% -----------------------------------------------------------------------
% Create the Left Shank
leftShank = Body();
leftShank.setName('LeftShank');
leftShank.setMass(ShankMass);
leftShank.setInertia(Inertia(1,1,ShankIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,-ThighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ShankLength/2,0);
orientationInChild  = Vec3(0,0,0);
LShankToThigh = PinJoint('LShankToLThigh', leftThigh, locationInParent,  ...
    orientationInParent, leftShank, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet = LShankToThigh.getCoordinateSet();
jointCoordinateSet.get(0).setRange([-140*DEG2RAD, 0]);
jointCoordinateSet.get(0).setName('LKnee_rz');
jointCoordinateSet.get(0).setDefaultValue(0*DEG2RAD);

% Add Visible Object for GUI
leftShank.addDisplayGeometry('sphere.vtp');
leftShank.getDisplayer().setScaleFactors([ShankLength/10, ShankLength, ShankLength/10]);

% Add Body to Model
osimModel.addBody(leftShank);
% -----------------------------------------------------------------------
% Create the Right Shank
rightShank = Body();
rightShank.setName('RightShank');
rightShank.setMass(ShankMass);
rightShank.setInertia(Inertia(1,1,ShankIZZ,0,0,0));

% Rod Visuals
rightShank.addDisplayGeometry('sphere.vtp');
rightShank.getDisplayer().setScaleFactors([ShankLength/10, ShankLength, ShankLength/10]);

% Create a Pin joint
locationInParent    = Vec3(0,-ThighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ShankLength/2,0);
orientationInChild  = Vec3(0,0,0);
RShankToThigh = PinJoint('RShankToRThigh', rightThigh, locationInParent, orientationInParent, rightShank, locationInChild, orientationInChild, false);

% Update the coordinates of the new joint
jointCoordinateSet = RShankToThigh.getCoordinateSet();
jointCoordinateSet.get(0).setRange([-140*DEG2RAD, 0]);
jointCoordinateSet.get(0).setName('RKnee_rz');
jointCoordinateSet.get(0).setDefaultValue(0*DEG2RAD);

% Add Body to Model
osimModel.addBody(rightShank);
% -----------------------------------------------------------------------
% Add Contact Elements to the Shank and Pelvis
footSphereRadius    = 0.05;

LeftFootContactSphere = ContactSphere(footSphereRadius, Vec3(0,-ShankLength/2+footSphereRadius,0), leftShank);
LeftFootContactSphere.setName('LFootContact');
LeftFootContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(LeftFootContactSphere);

RightFootContactSphere = ContactSphere(footSphereRadius, Vec3(0,-ShankLength/2+footSphereRadius,0), rightShank);
RightFootContactSphere.setName('RFootContact');
RightFootContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(RightFootContactSphere);

LeftKneeContactSphere = ContactSphere(footSphereRadius, Vec3(0,ShankLength/2,0), leftShank);
LeftKneeContactSphere.setName('LKneeContact');
LeftKneeContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(LeftKneeContactSphere);

RightKneeContactSphere = ContactSphere(footSphereRadius, Vec3(0,ShankLength/2,0), rightShank);
RightKneeContactSphere.setName('RKneeContact');
RightKneeContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(RightKneeContactSphere);

LeftPelvisContactSphere = ContactSphere(footSphereRadius, Vec3(0,0,-PelvisWidth/2), pelvis);
LeftPelvisContactSphere.setName('LHipContact');
LeftPelvisContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(LeftPelvisContactSphere);

RightPelvisContactSphere = ContactSphere(footSphereRadius, Vec3(0,0,PelvisWidth/2), pelvis);
RightPelvisContactSphere.setName('RHipContact');
RightPelvisContactSphere.setDisplayPreference(4);
osimModel.addContactGeometry(RightPelvisContactSphere);

groundContactLoc = Vec3(0,PlatformWidth/2,0);
groundContactOri = Vec3(0,0,-pi/2);
groundContactSpace = ContactHalfSpace (groundContactLoc, groundContactOri, platform);
groundContactSpace.setName('PlatformContact');
groundContactSpace.setDisplayPreference(4);
osimModel.addContactGeometry(groundContactSpace);
% -----------------------------------------------------------------------
% Add Hunt Crossley Forces
% Contact Parameters
stiffness           = 1.0E6;
dissipation         = 1.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;

% Create Hunt Crossley Left Foot
HuntCrossleyLeftFoot = HuntCrossleyForce();
HuntCrossleyLeftFoot.setName('LFootForce');

% Set Contact Bodies and Parameters
HuntCrossleyLeftFoot.addGeometry('LFootContact');
HuntCrossleyLeftFoot.addGeometry('PlatformContact');
HuntCrossleyLeftFoot.setStiffness(stiffness);
HuntCrossleyLeftFoot.setDissipation(dissipation);
HuntCrossleyLeftFoot.setStaticFriction(staticFriction);
HuntCrossleyLeftFoot.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftFoot.setViscousFriction(viscousFriction);
HuntCrossleyLeftFoot.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyLeftFoot);
% -----------------------------------------------------------------------
% Create Hunt Crossley Right Foot
HuntCrossleyRightFoot = HuntCrossleyForce();
HuntCrossleyRightFoot.setName('RFootForce');

% Set Contact Bodies and Parameters
HuntCrossleyRightFoot.addGeometry('RFootContact');
HuntCrossleyRightFoot.addGeometry('PlatformContact');
HuntCrossleyRightFoot.setStiffness(stiffness);
HuntCrossleyRightFoot.setDissipation(dissipation);
HuntCrossleyRightFoot.setStaticFriction(staticFriction);
HuntCrossleyRightFoot.setDynamicFriction(dynamicFriction);
HuntCrossleyRightFoot.setViscousFriction(viscousFriction);
HuntCrossleyRightFoot.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyRightFoot);
% -----------------------------------------------------------------------
% Create Hunt Crossley Left Knee
HuntCrossleyLeftKnee = HuntCrossleyForce();
HuntCrossleyLeftKnee.setName('LKneeForce');

% Set Contact Bodies and Parameters
HuntCrossleyLeftKnee.addGeometry('LKneeContact');
HuntCrossleyLeftKnee.addGeometry('PlatformContact');
HuntCrossleyLeftKnee.setStiffness(stiffness);
HuntCrossleyLeftKnee.setDissipation(dissipation);
HuntCrossleyLeftKnee.setStaticFriction(staticFriction);
HuntCrossleyLeftKnee.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftKnee.setViscousFriction(viscousFriction);
HuntCrossleyLeftKnee.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyLeftKnee);
% -----------------------------------------------------------------------
% Create Hunt Crossley Right Knee
HuntCrossleyRightKnee = HuntCrossleyForce();
HuntCrossleyRightKnee.setName('RKneeForce');

% Set Contact Bodies and Parameters
HuntCrossleyRightKnee.addGeometry('RKneeContact');
HuntCrossleyRightKnee.addGeometry('PlatformContact');
HuntCrossleyRightKnee.setStiffness(stiffness);
HuntCrossleyRightKnee.setDissipation(dissipation);
HuntCrossleyRightKnee.setStaticFriction(staticFriction);
HuntCrossleyRightKnee.setDynamicFriction(dynamicFriction);
HuntCrossleyRightKnee.setViscousFriction(viscousFriction);
HuntCrossleyRightKnee.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyRightKnee);
% -----------------------------------------------------------------------
% Create Hunt Crossley Left Hip
HuntCrossleyLeftHip = HuntCrossleyForce();
HuntCrossleyLeftHip.setName('LHipForce');

% Set Contact Bodies and Parameters
HuntCrossleyLeftHip.addGeometry('LHipContact');
HuntCrossleyLeftHip.addGeometry('PlatformContact');
HuntCrossleyLeftHip.setStiffness(stiffness);
HuntCrossleyLeftHip.setDissipation(dissipation);
HuntCrossleyLeftHip.setStaticFriction(staticFriction);
HuntCrossleyLeftHip.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftHip.setViscousFriction(viscousFriction);
HuntCrossleyLeftHip.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyLeftHip);
% -----------------------------------------------------------------------
% Create Hunt Crossley Right Hip
HuntCrossleyRightHip = HuntCrossleyForce();
HuntCrossleyRightHip.setName('RHipForce');

% Set Contact Bodies and Parameters
HuntCrossleyRightHip.addGeometry('RHipContact');
HuntCrossleyRightHip.addGeometry('PlatformContact');
HuntCrossleyRightHip.setStiffness(stiffness);
HuntCrossleyRightHip.setDissipation(dissipation);
HuntCrossleyRightHip.setStaticFriction(staticFriction);
HuntCrossleyRightHip.setDynamicFriction(dynamicFriction);
HuntCrossleyRightHip.setViscousFriction(viscousFriction);
HuntCrossleyRightHip.setTransitionVelocity(0.2);

% Add Force to the Model
osimModel.addForce(HuntCrossleyRightHip);
% -----------------------------------------------------------------------
% Add Coordinate Limit Forces
% Note this force takes input in Degrees to match coordinate panel in GUI
upperStiffness = 0.5;
lowerStiffness = 0.5;
damping = 0.025;
transition = 5;

kneeUpperLimit = 0;
kneeLowerLimit = -140;
hipUpperLimit = 100;
hipLowerLimit = -100;

LKneeLimitTorque = CoordinateLimitForce('LKnee_rz', kneeUpperLimit, upperStiffness, kneeLowerLimit, lowerStiffness, damping, transition);
LKneeLimitTorque.setName('LKneeLimitTorque');
osimModel.addForce(LKneeLimitTorque);

RKneeLimitTorque = CoordinateLimitForce('RKnee_rz', kneeUpperLimit, upperStiffness, kneeLowerLimit, lowerStiffness, damping, transition);
RKneeLimitTorque.setName('RKneeLimitTorque');
osimModel.addForce(RKneeLimitTorque);

LHipLimitTorque = CoordinateLimitForce('LHip_rz', hipUpperLimit, upperStiffness, hipLowerLimit, lowerStiffness, damping, transition);
LHipLimitTorque.setName('LHipLimitTorque');
osimModel.addForce(LHipLimitTorque);

RHipLimitTorque = CoordinateLimitForce('RHip_rz', hipUpperLimit, upperStiffness, hipLowerLimit, lowerStiffness, damping, transition);
RHipLimitTorque.setName('RHipLimitTorque');
osimModel.addForce(RHipLimitTorque);
% -----------------------------------------------------------------------
% Create Hunt Crossley Obstacle Forces
ObstacleForces = HuntCrossleyForce();
ObstacleForces.setName('ObstacleForces');
ObstacleForces.setStiffness(stiffness);
ObstacleForces.setDissipation(dissipation);
ObstacleForces.setStaticFriction(staticFriction);
ObstacleForces.setDynamicFriction(dynamicFriction);
ObstacleForces.setViscousFriction(viscousFriction);
ObstacleForces.setTransitionVelocity(0.2);
ObstacleForces.addGeometry('LFootContact');
ObstacleForces.addGeometry('RFootContact');

% Add Spheres
% This way of setting the seed was added in Matlab2011a as an error fix.
% Do not use the older versions to set the seed and state in rand or randn.
if(exist('rng', 'file'))
    rng(0, 'twister');
else
   warning('CreateWalkingModelAndEnvironment:RNG', ...
           ['\tYou are using a version of Matlab before 2011a and do', ...
           'not have the rng function.  The location of your terrain', ...
           'obstacles will be different than the base model.']); 
end
% You can comment this line out if necessary. However, your obstacles will 
% be in a different place than the first model.  
for i = 1:1:numSpheres
    radius = minObstacleRadius+(maxObstacleRadius - minObstacleRadius)*rand(1,1);
    locX = endingPoint-distance*abs(.30*randn(1,1))-PlatformOffset;
    locY = PlatformWidth/2-radius*0.8;
    locZ = PelvisWidth/2*(sign(-0.5+rand(1,1)));
    sphere = ContactSphere(radius, Vec3(locX,locY,locZ), platform);
    name = ['Obstacle',num2str(i)];
    sphere.setName(name);
    osimModel.addContactGeometry(sphere);
    ObstacleForces.addGeometry(name);
end

% Add Force to the Model
osimModel.addForce(ObstacleForces);
% -----------------------------------------------------------------------
osimModel.print('../Model/DW2013_WalkerModelTerrainUserEdit.osim');