%% buildWalker.m
%   Script to programmatically build a Passive Dynamic Walker osimModel
%   For Tutorial 'From the Ground Up: Building a Passive Dynamic Walker'

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): James Dunne, Andrew Miller, Jeff Reinbolt, Ajay Seth...    %
%            Daniel A. Jacobs, Chris Dembia.                            %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

%% Clear Workspace
clear all; close all; clc;

%% Import OpenSim Libraries into Matlab
import org.opensim.modeling.*

%% Define key model variables
pelvisWidth = 0.20;
thighLength = 0.40;
shankLength = 0.435;

%% Instantiate an (empty) OpenSim Model
osimModel = Model();
osimModel.setName('DynamicWalkerModel');

% Get a reference to the ground object
ground = osimModel.getGround();

% Define the acceleration of gravity
osimModel.setGravity(Vec3(0, -9.80665, 0));

% TODO: Construct Bodies and Joints Here
% ********** BEGIN CODE **********
platform = Body();
platform.setName('Platform');
platform.setMass(1);
platform.setInertia( Inertia(1,1,1,0,0,0) );

% Add geometry to the body
platformGeometry = Brick(Vec3(10,0.01,1));
platform.attachGeometry(platformGeometry);

% Add Body to the Model
osimModel.addBody(platform);

% Section: Create the Platform Joint
% Make and add a Pin joint for the Platform Body
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
platformToGround    = PinJoint('PlatformToGround',...  % Joint Name
                                ground,...             % Parent Frame
                                locationInParent,...   % Translation in Parent Frame
                                orientationInParent,...% Orientation in Parent Frame
                                platform,...           % Child Frame
                                locationInChild,...    % Translation in Child Frame
                                orientationInChild);   % Orientation in Child Frame

% Update the coordinates of the new joint
platform_rz = platformToGround.upd_coordinates(0);
platform_rz.setRange([deg2rad(-100), deg2rad(100)]);
platform_rz.setName('platform_rz');
platform_rz.setDefaultValue(deg2rad(-10));
platform_rz.setDefaultSpeedValue(0);
platform_rz.setDefaultLocked(true)

% Add Joint to the Model
osimModel.addJoint(platformToGround);

% Make and add a Pelvis Body
pelvis = Body();
pelvis.setName('Pelvis');
pelvis.setMass(1);
pelvis.setInertia(Inertia(1,1,1,0,0,0));
% Add geometry for display
pelvis.attachGeometry(Sphere(pelvisWidth));
% Add Body to the Model
osimModel.addBody(pelvis);

% Make and add a Planar joint for the Pelvis Body
pelvisToPlatform = PlanarJoint('PelvisToPlatform', platform, pelvis);
% Update the coordinates of the new joint
Pelvis_rz = pelvisToPlatform.upd_coordinates(0); % Rotation about z
Pelvis_rz.setRange([-pi, pi]);
Pelvis_rz.setName('Pelvis_rz');
Pelvis_rz.setDefaultValue(0);

Pelvis_tx = pelvisToPlatform.upd_coordinates(1); % T about x
Pelvis_tx.setRange([0, 10]);
Pelvis_tx.setName('Pelvis_tx');
Pelvis_tx.setDefaultValue(0);
Pelvis_tx.setDefaultSpeedValue(0)

Pelvis_ty = pelvisToPlatform.upd_coordinates(2); % Translation about y
Pelvis_ty.setRange([-5,5]);
Pelvis_ty.setName('Pelvis_ty');
Pelvis_ty.setDefaultValue(thighLength + shankLength);
Pelvis_ty.setDefaultSpeedValue(0)
% Add Joint to model
osimModel.addJoint(pelvisToPlatform)


% Make and add a Right Thigh Body
rightThigh = Body();
rightThigh.setName('RightThigh');
rightThigh.setMass(1);
rightThigh.setMassCenter( Vec3(0,0,0) )
rightThigh.setInertia(Inertia(2,2,0.02,0,0,0));
% Add geometry for display
rightThigh.attachGeometry(Ellipsoid(thighLength/10, thighLength/2, thighLength/10));
% Add Body to the Model
osimModel.addBody(rightThigh);

% Make and add a Pin joint for the Right Thigh Body
locationInParent    = Vec3(0,0,pelvisWidth);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,thighLength/2,0);
orientationInChild  = Vec3(0,0,0);
RightThighToPelvis = PinJoint('RThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, rightThigh, locationInChild, orientationInChild);

% Update the coordinates of the new joint
RHip_rz = RightThighToPelvis.upd_coordinates(0);
RHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
RHip_rz.setName('RHip_rz');
RHip_rz.setDefaultValue(deg2rad(30));
RHip_rz.setDefaultSpeedValue(0 );
% Add Joint to the Model
osimModel.addJoint(RightThighToPelvis);


% Make and add a Left Thigh Body
leftThigh = Body();
leftThigh.setName('LeftThigh');
leftThigh.setMass(1);
rightThigh.setMassCenter( Vec3(0,0,0) )
leftThigh.setInertia(Inertia(2,2,0.02,0,0,0));
% Add geometry for display
leftThigh.attachGeometry(Ellipsoid(thighLength/10, thighLength/2, thighLength/10));
% Add Body to the Model
osimModel.addBody(leftThigh);

% Make and add a Pin joint for the Left Thigh Body
locationInParent    = Vec3(0,0,-pelvisWidth);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,thighLength/2,0);
orientationInChild  = Vec3(0,0,0);
LThighToPelvis = PinJoint('LThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, leftThigh, locationInChild, orientationInChild);
% Update the coordinates of the new joint
LHip_rz = LThighToPelvis.upd_coordinates(0);
LHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
LHip_rz.setName('LHip_rz');
LHip_rz.setDefaultValue(deg2rad(-10));
LHip_rz.setDefaultSpeedValue(0);
% Add Body to the Model
osimModel.addJoint(LThighToPelvis);

% Make and add a Right Shank Body
rightShank = Body();
rightShank.setName('RightShank');
rightShank.setMass(1);
rightShank.setMassCenter( Vec3(0,0,0) )
rightShank.setInertia(Inertia(2,2,0.002,0,0,0));
% Add geometry for display
rightShank.attachGeometry(Ellipsoid(shankLength/10, shankLength/2, shankLength/10));
% Add Body to the Model
osimModel.addBody(rightShank);
% Make and add a Pin joint for the Right Shank Body
locationInParent    = Vec3(0,-thighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,shankLength/2,0);
orientationInChild  = Vec3(0,0,0);
RShankToThigh = PinJoint('RShankToRThigh', ...
    rightThigh, locationInParent, orientationInParent, ...
    rightShank, locationInChild, orientationInChild);
% Update the coordinates of the new joint
RKnee_rz = RShankToThigh.updCoordinate();
RKnee_rz.setRange([deg2rad(-100), 0]);
RKnee_rz.setName('RKnee_rz');
RKnee_rz.setDefaultValue(deg2rad(-30));
RKnee_rz.setDefaultSpeedValue(0);
% Add Joint to the Model
osimModel.addJoint(RShankToThigh);

% Make and add a Left Shank Body
leftShank = Body();
leftShank.setName('LeftShank');
leftShank.setMass(0.1);
leftShank.setInertia(Inertia(2,2,0.002,0,0,0));
% Add geometry for display
leftShank.attachGeometry(Ellipsoid(shankLength/10, shankLength/2, shankLength/10));
% Add Body to the Model
osimModel.addBody(leftShank);
% Make and add a Pin joint for the Left Shank Body
locationInParent    = Vec3(0,-thighLength/2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,shankLength/2,0);
orientationInChild  = Vec3(0,0,0);
LShankToThigh = PinJoint('LShankToLThigh', leftThigh, locationInParent,...
    orientationInParent, leftShank, locationInChild, orientationInChild);
% Update the coordinates of the new joint
LKnee_rz = LShankToThigh.updCoordinate();
LKnee_rz.setRange([deg2rad(-100), 0]);
LKnee_rz.setName('LKnee_rz');
LKnee_rz.setDefaultValue(deg2rad(-30));
LKnee_rz.setDefaultSpeedValue(0)
% Add Joint to the Model
osimModel.addJoint(LShankToThigh);
% **********  END CODE  **********

% TODO: Construct ContactGeometry and HuntCrossleyForces Here
% ********** BEGIN CODE **********
% TODO: Construct ContactGeometry and HuntCrossleyForces Here
% ********** BEGIN CODE **********
contactSphereRadius = 0.05;

% Make a Contact Half Space
groundContactLocation = Vec3(0,0.025,0);
groundContactOrientation = Vec3(0,0,-1.57);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       platform);
groundContactSpace.setName('PlatformContact');
osimModel.addContactGeometry(groundContactSpace);


% Make a Right Hip Contact
RightHipContactSphere = ContactSphere();
RightHipContactSphere.setRadius(contactSphereRadius);
RightHipContactSphere.setLocation( Vec3(0,0,pelvisWidth) );
RightHipContactSphere.setFrame(pelvis)
RightHipContactSphere.setName('RHipContact');
osimModel.addContactGeometry(RightHipContactSphere);


% Make a Left Hip Contact
LeftHipContactSphere = ContactSphere();
LeftHipContactSphere.setRadius(contactSphereRadius);
LeftHipContactSphere.setLocation( Vec3(0,0,-pelvisWidth) );
LeftHipContactSphere.setFrame(pelvis)
LeftHipContactSphere.setName('LHipContact');
osimModel.addContactGeometry(LeftHipContactSphere);


% Make a Right Knee Contact
RightKneeContactSphere = ContactSphere();
RightKneeContactSphere.setRadius(contactSphereRadius);
RightKneeContactSphere.setLocation( Vec3(0,thighLength/2,0) );
RightKneeContactSphere.setFrame(rightShank)
RightKneeContactSphere.setName('RKneeContact');
osimModel.addContactGeometry(RightKneeContactSphere);

% Make a Left Knee Contact
LeftKneeContactSphere = ContactSphere();
LeftKneeContactSphere.setRadius(contactSphereRadius);
LeftKneeContactSphere.setLocation(Vec3(0,thighLength/2,0));
LeftKneeContactSphere.setFrame(leftShank)
LeftKneeContactSphere.setName('LKneeContact');
osimModel.addContactGeometry(LeftKneeContactSphere);

% Make a Right Foot Contact Sphere
RightFootContactSphere = ContactSphere();
RightFootContactSphere.setRadius(contactSphereRadius);
RightFootContactSphere.setLocation( Vec3(0,-shankLength/2,0) );
RightFootContactSphere.setFrame(rightShank)
RightFootContactSphere.setName('RFootContact');
osimModel.addContactGeometry(RightFootContactSphere);

% Make a Left Foot Contact Sphere
LeftFootContactSphere = ContactSphere();
LeftFootContactSphere.setRadius(contactSphereRadius);
LeftFootContactSphere.setLocation( Vec3(0,-shankLength/2,0) );
LeftFootContactSphere.setFrame(leftShank)
LeftFootContactSphere.setName('LFootContact');
osimModel.addContactGeometry(LeftFootContactSphere);

% **********  END CODE  **********

% Define Contact Force Parameters
stiffness           = 1000000;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;

% Make a Hunt Crossley Force for Right Hip and update parameters
HuntCrossleyRightHip = HuntCrossleyForce();
HuntCrossleyRightHip.setName('RHipForce');
HuntCrossleyRightHip.addGeometry('RHipContact');
HuntCrossleyRightHip.addGeometry('PlatformContact');
HuntCrossleyRightHip.setStiffness(stiffness);
HuntCrossleyRightHip.setDissipation(dissipation);
HuntCrossleyRightHip.setStaticFriction(staticFriction);
HuntCrossleyRightHip.setDynamicFriction(dynamicFriction);
HuntCrossleyRightHip.setViscousFriction(viscousFriction);
HuntCrossleyRightHip.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyRightHip);


% Make a Hunt Crossley Force for Left Hip and update parameters
HuntCrossleyLeftHip = HuntCrossleyForce();
HuntCrossleyLeftHip.setName('LHipForce');
HuntCrossleyLeftHip.addGeometry('LHipContact');
HuntCrossleyLeftHip.addGeometry('PlatformContact');
HuntCrossleyLeftHip.setStiffness(stiffness);
HuntCrossleyLeftHip.setDissipation(dissipation);
HuntCrossleyLeftHip.setStaticFriction(staticFriction);
HuntCrossleyLeftHip.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftHip.setViscousFriction(viscousFriction);
HuntCrossleyLeftHip.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyLeftHip);


% Make a Hunt Crossley Force for Right Knee and update parameters
HuntCrossleyRightKnee = HuntCrossleyForce();
HuntCrossleyRightKnee.setName('RKneeForce');
HuntCrossleyRightKnee.addGeometry('RKneeContact');
HuntCrossleyRightKnee.addGeometry('PlatformContact');
HuntCrossleyRightKnee.setStiffness(stiffness);
HuntCrossleyRightKnee.setDissipation(dissipation);
HuntCrossleyRightKnee.setStaticFriction(staticFriction);
HuntCrossleyRightKnee.setDynamicFriction(dynamicFriction);
HuntCrossleyRightKnee.setViscousFriction(viscousFriction);
HuntCrossleyRightKnee.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyRightKnee);


% Make a Hunt Crossley Force for Left Knee and update parameters
HuntCrossleyLeftKnee = HuntCrossleyForce();
HuntCrossleyLeftKnee.setName('LKneeForce');
HuntCrossleyLeftKnee.addGeometry('LKneeContact');
HuntCrossleyLeftKnee.addGeometry('PlatformContact');
HuntCrossleyLeftKnee.setStiffness(stiffness);
HuntCrossleyLeftKnee.setDissipation(dissipation);
HuntCrossleyLeftKnee.setStaticFriction(staticFriction);
HuntCrossleyLeftKnee.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftKnee.setViscousFriction(viscousFriction);
HuntCrossleyLeftKnee.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyLeftKnee);


% Make a Hunt Crossley Force for Right Foot and update parameters
HuntCrossleyRightFoot = HuntCrossleyForce();
HuntCrossleyRightFoot.setName('RFootForce');
HuntCrossleyRightFoot.addGeometry('RFootContact');
HuntCrossleyRightFoot.addGeometry('PlatformContact');
HuntCrossleyRightFoot.setStiffness(stiffness);
HuntCrossleyRightFoot.setDissipation(dissipation);
HuntCrossleyRightFoot.setStaticFriction(staticFriction);
HuntCrossleyRightFoot.setDynamicFriction(dynamicFriction);
HuntCrossleyRightFoot.setViscousFriction(viscousFriction);
HuntCrossleyRightFoot.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyRightFoot);


% Make a Hunt Crossley Force for Left Foot and update parameters
HuntCrossleyLeftFoot = HuntCrossleyForce();
HuntCrossleyLeftFoot.setName('LFootForce');
HuntCrossleyLeftFoot.addGeometry('LFootContact');
HuntCrossleyLeftFoot.addGeometry('PlatformContact');
HuntCrossleyLeftFoot.setStiffness(stiffness);
HuntCrossleyLeftFoot.setDissipation(dissipation);
HuntCrossleyLeftFoot.setStaticFriction(staticFriction);
HuntCrossleyLeftFoot.setDynamicFriction(dynamicFriction);
HuntCrossleyLeftFoot.setViscousFriction(viscousFriction);
HuntCrossleyLeftFoot.setTransitionVelocity(transitionVelocity);
osimModel.addForce(HuntCrossleyLeftFoot);

% TODO: Construct CoordinateLimitForces Here
% ********** BEGIN CODE **********
% Define Coordinate Limit Force Parameters
upperStiffness = 0.5;
lowerStiffness = 0.5;
kneeUpperLimit = 0;
kneeLowerLimit = -140;
hipUpperLimit = 100;
hipLowerLimit = -100;
damping = 0.025;
transition = 5;

% Make a Right Hip Coordinate Limit Force
RHipLimitTorque = CoordinateLimitForce();
RHipLimitTorque.setName('RHipLimitTorque')
RHipLimitTorque.set_coordinate('RHip_rz');
RHipLimitTorque.setUpperStiffness(upperStiffness);
RHipLimitTorque.setLowerStiffness(lowerStiffness);
RHipLimitTorque.setUpperLimit(hipUpperLimit);
RHipLimitTorque.setLowerLimit(hipLowerLimit);
RHipLimitTorque.setDamping(damping);
RHipLimitTorque.setTransition(transition)
osimModel.addForce(RHipLimitTorque);


% Make a Left Hip Coordinate Limit Force
LHipLimitTorque = CoordinateLimitForce();
LHipLimitTorque.setName('LHipLimitTorque')
LHipLimitTorque.set_coordinate('LHip_rz');
LHipLimitTorque.setUpperStiffness(upperStiffness);
LHipLimitTorque.setLowerStiffness(lowerStiffness);
LHipLimitTorque.setUpperLimit(hipUpperLimit);
LHipLimitTorque.setLowerLimit(hipLowerLimit);
LHipLimitTorque.setDamping(damping);
LHipLimitTorque.setTransition(transition)
osimModel.addForce(LHipLimitTorque);

% Make a Right Knee Coordinate Limit Force
RKneeLimitTorque = CoordinateLimitForce();
RKneeLimitTorque.setName('RKneeLimitTorque')
RKneeLimitTorque.set_coordinate('RKnee_rz');
RKneeLimitTorque.setUpperStiffness(upperStiffness);
RKneeLimitTorque.setLowerStiffness(lowerStiffness);
RKneeLimitTorque.setUpperLimit(kneeUpperLimit);
RKneeLimitTorque.setLowerLimit(kneeLowerLimit);
RKneeLimitTorque.setDamping(damping);
RKneeLimitTorque.setTransition(transition)
osimModel.addForce(RKneeLimitTorque);

% Make a Left Knee Coordinate Limit Force
LKneeLimitTorque = CoordinateLimitForce();
LKneeLimitTorque.setName('LKneeLimitTorque')
LKneeLimitTorque.set_coordinate('LKnee_rz');
LKneeLimitTorque.setUpperStiffness(upperStiffness);
LKneeLimitTorque.setLowerStiffness(lowerStiffness);
LKneeLimitTorque.setUpperLimit(kneeUpperLimit);
LKneeLimitTorque.setLowerLimit(kneeLowerLimit);
LKneeLimitTorque.setDamping(damping);
LKneeLimitTorque.setTransition(transition)
osimModel.addForce(LKneeLimitTorque);
% **********  END CODE  **********

%% Initialize the System (checks model consistency).
osimModel.initSystem();

% Save the model to a file
osimModel.print('DynamicWalkerModel.osim');
display(['DynamicWalkerModel.osim printed!']);
