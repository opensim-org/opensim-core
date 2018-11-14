% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2019 Stanford University and the Authors             
% Author(s): Daniel A. Jacobs                                             
% Contributor(s): Hannah O'Day, Chris Dembia
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

% User Section - Adjust these parameters at will
outputPath = '../Model/';
outputModelName = 'WalkerModel';

% Model Body Parameters
PlatformLength  = 10;
PlatformHeight   = 0.05;
PlatformOffset  = 0.5;

PelvisMass      = 0.5;
PelvisWidth     = 0.4;
PelvisIYY       = 1/12*PelvisMass*PelvisWidth^2;
PelvisIZZ       = 1/12*PelvisMass*PelvisWidth^2;

ThighMass       = 0.05;
ThighLength     = 0.40;
ThighIZZ        = 1/12*ThighMass*ThighLength^2;

ShankMass       = 0.05;
ShankLength     = 0.435;
ShankIZZ        = 1/12*ShankMass*ShankLength^2;

smallInertia    = 1E-4; 

% Add Obstacles
addObstacles = true;

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
if addObstacles
    outputModelName = [outputModelName, 'Terrain'];
end
osimModel.setName(outputModelName);
osimModel.setAuthors('Daniel A. Jacobs, Ajay Seth');

% Get a Handle to Ground Body
ground = osimModel.getGround();

% -----------------------------------------------------------------------
% Platform
platform            = Body();
platform.setName('Platform');
platform.setMass(1);
platform.setInertia(Inertia(1,1,1,0,0,0));

% Create Weld Joint
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,deg2rad(-10));
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
platformToGround    = WeldJoint('PlatformToGround', ...
    ground, locationInParent, orientationInParent, ...
    platform, locationInChild, orientationInChild);

% Add geometry to display in the GUI
platformGeomOffset = PhysicalOffsetFrame(platform, ...
    Transform(...
        Vec3(PlatformLength/2 - PlatformOffset, 0, 0)));
platformGeomOffset.attachGeometry(...
    Brick(Vec3(0.5 * PlatformLength, 0.5 * PlatformHeight, 0.5)));
platform.addComponent(platformGeomOffset);

% Add Body to Model
osimModel.addBody(platform);
osimModel.addJoint(platformToGround);
% -----------------------------------------------------------------------
% Create a massive body for the 6 DOF of the pelvis
pelvis              = Body();
pelvis.setName('Pelvis');
pelvis.setMass(PelvisMass);
pelvis.setInertia(Inertia(smallInertia,PelvisIYY,smallInertia,0,0,0));

% Create Planar Joint
pelvisToPlatform    = PlanarJoint('PelvisToPlatform', platform, pelvis);

% Update the coordinates of the new joint
Pelvis_rz = pelvisToPlatform.upd_coordinates(0);
Pelvis_rz.setRange([-pi, pi]);
Pelvis_rz.setName('Pelvis_rz');
Pelvis_rz.setDefaultValue(0);

Pelvis_tx = pelvisToPlatform.upd_coordinates(1);
Pelvis_tx.setRange([-10, 10]);
Pelvis_tx.setName('Pelvis_tx');
Pelvis_tx.setDefaultValue(0);

Pelvis_ty = pelvisToPlatform.upd_coordinates(2);
Pelvis_ty.setRange([-1, 2]);
Pelvis_ty.setName('Pelvis_ty');
Pelvis_ty.setDefaultValue(1.0);

% Add Body to Model
osimModel.addBody(pelvis);
osimModel.addJoint(pelvisToPlatform);

% Add geometry to display in the GUI
pelvis.attachGeometry(...
    Ellipsoid(PelvisWidth/4.0, PelvisWidth/4.0, PelvisWidth/2.0));

% -----------------------------------------------------------------------
% Create the Left Thigh
leftThigh = Body();
leftThigh.setName('LeftThigh');
leftThigh.setMass(ThighMass);
leftThigh.setInertia(Inertia(smallInertia,smallInertia,ThighIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,-PelvisWidth/2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ThighLength/2,0);
orientationInChild  = Vec3(0,0,0);
LThighToPelvis = PinJoint('LThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, leftThigh, locationInChild, orientationInChild);

% Update the coordinates of the new joint
LHip_rz = LThighToPelvis.updCoordinate();
LHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
LHip_rz.setName('LHip_rz');
LHip_rz.setDefaultValue(deg2rad(-10));

% Add geometry to display in the GUI
leftThigh.attachGeometry(...
    Ellipsoid(ThighLength/20, ThighLength/2, ThighLength/20));

% Add Body to Model
osimModel.addBody(leftThigh);
osimModel.addJoint(LThighToPelvis);
% -----------------------------------------------------------------------
% Create the Right Thigh
rightThigh = Body();
rightThigh.setName('RightThigh');
rightThigh.setMass(ThighMass);
rightThigh.setInertia(Inertia(smallInertia,smallInertia,ThighIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,PelvisWidth/2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ThighLength/2,0);
orientationInChild  = Vec3(0,0,0);
RThighToPelvis = PinJoint('RThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, rightThigh, locationInChild, orientationInChild);

% Update the coordinates of the new joint
RHip_rz = RThighToPelvis.updCoordinate();
RHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
RHip_rz.setName('RHip_rz');
RHip_rz.setDefaultValue(deg2rad(30));

% Add geometry to display in the GUI
rightThigh.attachGeometry(...
    Ellipsoid(ThighLength/20, ThighLength/2, ThighLength/20));

% Add Body to Model
osimModel.addBody(rightThigh);
osimModel.addJoint(RThighToPelvis);
% -----------------------------------------------------------------------
% Create the Left Shank
leftShank = Body();
leftShank.setName('LeftShank');
leftShank.setMass(ShankMass);
leftShank.setInertia(Inertia(smallInertia,smallInertia,ShankIZZ,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,-ThighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ShankLength/2,0);
orientationInChild  = Vec3(0,0,0);
LShankToThigh = PinJoint('LShankToLThigh', leftThigh, locationInParent,  ...
    orientationInParent, leftShank, locationInChild, orientationInChild);

% Update the coordinates of the new joint
LKnee_rz = LShankToThigh.updCoordinate();
LKnee_rz.setRange([deg2rad(-100), 0]);
LKnee_rz.setName('LKnee_rz');
LKnee_rz.setDefaultValue(deg2rad(-30));

% Add geometry to display in the GUI
leftShank.attachGeometry(...
    Ellipsoid(ShankLength/20, ShankLength/2, ShankLength/20));

% Add Body to Model
osimModel.addBody(leftShank);
osimModel.addJoint(LShankToThigh);
% -----------------------------------------------------------------------
% Create the Right Shank
rightShank = Body();
rightShank.setName('RightShank');
rightShank.setMass(ShankMass);
rightShank.setInertia(Inertia(smallInertia,smallInertia,ShankIZZ,0,0,0));

% Add geometry to display in the GUI
rightShank.attachGeometry(...
    Ellipsoid(ShankLength/20, ShankLength/2, ShankLength/20));

% Create a Pin joint
locationInParent    = Vec3(0,-ThighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,ShankLength/2,0);
orientationInChild  = Vec3(0,0,0);
RShankToThigh = PinJoint('RShankToRThigh', ...
    rightThigh, locationInParent, orientationInParent, ...
    rightShank, locationInChild, orientationInChild);

% Update the coordinates of the new joint
RKnee_rz = RShankToThigh.updCoordinate();
RKnee_rz.setRange([deg2rad(-100), 0]);
RKnee_rz.setName('RKnee_rz');
RKnee_rz.setDefaultValue(deg2rad(-30));

% Add Body to Model
osimModel.addBody(rightShank);
osimModel.addJoint(RShankToThigh);
% -----------------------------------------------------------------------
% Add Contact Elements to the Shank and Pelvis
footSphereRadius    = 0.05;

LeftFootContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,-ShankLength/2+footSphereRadius,0), leftShank);
LeftFootContactSphere.setName('LFootContact');
osimModel.addContactGeometry(LeftFootContactSphere);

RightFootContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,-ShankLength/2+footSphereRadius,0), rightShank);
RightFootContactSphere.setName('RFootContact');
osimModel.addContactGeometry(RightFootContactSphere);

LeftKneeContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,ShankLength/2,0), leftShank);
LeftKneeContactSphere.setName('LKneeContact');
osimModel.addContactGeometry(LeftKneeContactSphere);

RightKneeContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,ShankLength/2,0), rightShank);
RightKneeContactSphere.setName('RKneeContact');
osimModel.addContactGeometry(RightKneeContactSphere);

LeftPelvisContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,0,-PelvisWidth/2), pelvis);
LeftPelvisContactSphere.setName('LHipContact');
osimModel.addContactGeometry(LeftPelvisContactSphere);

RightPelvisContactSphere = ContactSphere(footSphereRadius, ...
    Vec3(0,0,PelvisWidth/2), pelvis);
RightPelvisContactSphere.setName('RHipContact');
osimModel.addContactGeometry(RightPelvisContactSphere);

groundContactLoc = Vec3(0,PlatformHeight/2,0);
groundContactOri = Vec3(0,0,-pi/2);
groundContactSpace = ...
    ContactHalfSpace(groundContactLoc, groundContactOri, platform);
groundContactSpace.setName('PlatformContact');
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

LKneeLimitTorque = CoordinateLimitForce('LKnee_rz', kneeUpperLimit, ...
    upperStiffness, kneeLowerLimit, lowerStiffness, damping, transition);
LKneeLimitTorque.setName('LKneeLimitTorque');
osimModel.addForce(LKneeLimitTorque);

RKneeLimitTorque = CoordinateLimitForce('RKnee_rz', kneeUpperLimit, ...
    upperStiffness, kneeLowerLimit, lowerStiffness, damping, transition);
RKneeLimitTorque.setName('RKneeLimitTorque');
osimModel.addForce(RKneeLimitTorque);

LHipLimitTorque = CoordinateLimitForce('LHip_rz', hipUpperLimit, ...
    upperStiffness, hipLowerLimit, lowerStiffness, damping, transition);
LHipLimitTorque.setName('LHipLimitTorque');
osimModel.addForce(LHipLimitTorque);

RHipLimitTorque = CoordinateLimitForce('RHip_rz', hipUpperLimit, ...
    upperStiffness, hipLowerLimit, lowerStiffness, damping, transition);
RHipLimitTorque.setName('RHipLimitTorque');
osimModel.addForce(RHipLimitTorque);
% -----------------------------------------------------------------------
% Create Hunt Crossley Obstacle Forces
if addObstacles
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
    if exist('rng', 'file')
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
        locY = PlatformHeight/2-radius*0.8;
        locZ = PelvisWidth/2*(sign(-0.5+rand(1,1)));
        sphere = ContactSphere(radius, Vec3(locX,locY,locZ), platform);
        name = ['Obstacle',num2str(i)];
        sphere.setName(name);
        osimModel.addContactGeometry(sphere);
        ObstacleForces.addGeometry(name);
    end

    % Add Force to the Model
    osimModel.addForce(ObstacleForces);

end

% -----------------------------------------------------------------------
% Ensure there are no errors with how the model was built.
osimModel.initSystem();
osimModel.print([outputPath, outputModelName, '.osim']);
display(['Model ' outputModelName ', printed!']);
