%% buildWalker.m
%   Script to programmatically build a Passive Dynamic Walker Model
%   For Tutorial 'From the Ground Up: Building a Passive Dynamic Walker'

% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2019 Stanford University and the Authors                %
% Author(s): James Dunne, Tom Uchida, Chris Dembia                           %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% Author(s): James Dunne, Andrew Miller, Jeff Reinbold, Ajay Seth...
%            Daniel A. Jacobs, Hannah O'Day, Chris Dembia.

%% Clear Workspace
clear all; close all; clc;

%% Import Matlab Classes
import org.opensim.modeling.*

%% Open Model
model = Model();

%% Add bodies
% Get handle to the Ground
ground = model.getGround();

% Platform
platform = Body();
platform.setName('Platform');
platform.setMass(1);
platform.setInertia( Inertia(1,1,1,0,0,0) );

% Platform Weld Joint
locationInParent    = Vec3(0,0,0);
orientationInParent = Vec3(0,0,deg2rad(-1.5));
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
platformToGround    = WeldJoint('PlatformToGround', ...
    ground, locationInParent, orientationInParent, ...
    platform, locationInChild, orientationInChild);

% Add geometry to display in the GUI
% Instantiate a mesh to be used as display geometry
m = Mesh('box.vtp');
m.set_scale_factors( Vec3(10, 0.05, 1) );

% Add geometry mesh to the body
platform.append_attached_geometry(m);

%% Create Pelvis
pelvis = Body();
pelvis.setName('Pelvis');
pelvis.setMass(1);
pelvis.setInertia(Inertia(1,1,1,0,0,0));

% Create Planar Joint
pelvisToPlatform    = PlanarJoint('PelvisToPlatform', platform, pelvis);

% Update the coordinates of the new joint
Pelvis_rz = pelvisToPlatform.upd_coordinates(0);
Pelvis_rz.setRange([0, 0]);
Pelvis_rz.setName('Pelvis_rz');
Pelvis_rz.setDefaultValue(0);

Pelvis_tx = pelvisToPlatform.upd_coordinates(1);
Pelvis_tx.setRange([0, 10]);
Pelvis_tx.setName('Pelvis_tx');
Pelvis_tx.setDefaultValue(0.20);
Pelvis_tx.setDefaultSpeedValue(1.5)

Pelvis_ty = pelvisToPlatform.upd_coordinates(2);
Pelvis_ty.setRange([0, 5]);
Pelvis_ty.setName('Pelvis_ty');
Pelvis_ty.setDefaultValue(0.96);
Pelvis_ty.setDefaultSpeedValue(-2)

% Add geometry to display in the GUI
pelvis.attachGeometry(Sphere(0.2));

%% Create a Left Thigh
leftThigh = Body();
leftThigh.setName('LeftThigh');
leftThigh.setMass(1);
leftThigh.setInertia(Inertia(2,2,0.02,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,-0.2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0.25,0);
orientationInChild  = Vec3(0,0,0);
LThighToPelvis = PinJoint('LThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, leftThigh, locationInChild, orientationInChild);

% Update the coordinates of the new joint
LHip_rz = LThighToPelvis.updCoordinate();
LHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
LHip_rz.setName('LHip_rz');
LHip_rz.setDefaultValue(0.312511671);
LHip_rz.setDefaultSpeedValue(-0.245407488);

% Add geometry to display in the GUI
ThighLength = 0.5;
leftThigh.attachGeometry(Ellipsoid(ThighLength/20, ThighLength/2, ThighLength/20));

%% Create a Right Thigh
rightThigh = Body();
rightThigh.setName('RightThigh');
rightThigh.setMass(1);
rightThigh.setInertia(Inertia(2,2,0.02,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,0,0.2);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0.25,0);
orientationInChild  = Vec3(0,0,0);
RThighToPelvis = PinJoint('RThighToPelvis', pelvis, locationInParent, ...
    orientationInParent, rightThigh, locationInChild, orientationInChild);

% Update the coordinates of the new joint
RHip_rz = RThighToPelvis.updCoordinate();
RHip_rz.setRange([deg2rad(-100), deg2rad(100)]);
RHip_rz.setName('RHip_rz');
RHip_rz.setDefaultValue(-0.15);
RHip_rz.setDefaultSpeedValue(-0.6 );

% Add geometry to display in the GUI
rightThigh.attachGeometry(Ellipsoid(ThighLength/20, ThighLength/2, ThighLength/20));

%% Create a Left Shank
leftShank = Body();
leftShank.setName('LeftShank');
leftShank.setMass(0.1);
leftShank.setInertia(Inertia(2,2,0.002,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0,-0.25,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0.25,0);
orientationInChild  = Vec3(0,0,0);
LShankToThigh = PinJoint('LShankToLThigh', leftThigh, locationInParent,...
    orientationInParent, leftShank, locationInChild, orientationInChild);

% Update the coordinates of the new joint
LKnee_rz = LShankToThigh.updCoordinate();
LKnee_rz.setRange([deg2rad(-100), 0]);
LKnee_rz.setName('LKnee_rz');
LKnee_rz.setDefaultValue(0.03);
LKnee_rz.setDefaultSpeedValue(-0.5)

% Add geometry to display in the GUI
ShankLength = 0.5;
leftShank.attachGeometry(Ellipsoid(ShankLength/20, ShankLength/2, ShankLength/20));

%% Create a Right Shank
rightShank = Body();
rightShank.setName('RightShank');
rightShank.setMass(0.1);
rightShank.setInertia(Inertia(2,2,0.002,0,0,0));

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
RKnee_rz.setDefaultValue(0.05);
RKnee_rz.setDefaultSpeedValue(0.02);

% Add geometry to display in the GUI
rightShank.attachGeometry(Ellipsoid(ShankLength/20, ShankLength/2, ShankLength/20));

%% Create a Left Foot
leftFoot = Body();
leftFoot.setName('leftFoot');
leftFoot.setMass(0.0002);
leftFoot.setInertia(Inertia(2,2,0.0002,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0.05,-0.2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
LShankToFoot = WeldJoint('LeftShankToFoot', ...
    leftShank, locationInParent, orientationInParent, ...
    leftFoot, locationInChild, orientationInChild);

%% Create a Right Foot
rightFoot = Body();
rightFoot.setName('RightFoot');
rightFoot.setMass(0.0002);
rightFoot.setInertia(Inertia(2,2,0.0002,0,0,0));

% Create a Pin joint
locationInParent    = Vec3(0.05,-0.2,0);
orientationInParent = Vec3(0,0,0);
locationInChild     = Vec3(0,0,0);
orientationInChild  = Vec3(0,0,0);
RShankToFoot = WeldJoint('RightShankToFoot', ...
    rightShank, locationInParent, orientationInParent, ...
    rightFoot, locationInChild, orientationInChild);

%% Add the Bodies and Joints to the Model
% Add Bodies to the Model
model.addBody(platform);
model.addBody(pelvis);
model.addBody(leftThigh);
model.addBody(rightThigh);
model.addBody(leftShank);
model.addBody(rightShank);
model.addBody(leftFoot);
model.addBody(rightFoot);

% Add joints to the Model
model.addJoint(platformToGround);
model.addJoint(pelvisToPlatform);
model.addJoint(LThighToPelvis);
model.addJoint(RThighToPelvis);
model.addJoint(LShankToThigh);
model.addJoint(RShankToThigh);
model.addJoint(LShankToFoot);
model.addJoint(RShankToFoot);

%% Add Contact Elements to the Shank and Pelvis

%% Contact Half Space
groundContactLocation = Vec3(0,0.025,0);
groundContactOrientation = Vec3(0,0,-1.5707963267949001);
groundContactSpace = ContactHalfSpace(groundContactLocation,...
                                       groundContactOrientation,...
                                       platform);
groundContactSpace.setName('PlatformContact');

%% Left Foot Contact Sphere
LeftFootContactSphere = ContactSphere();
LeftFootContactSphere.setRadius(0.1);
LeftFootContactSphere.setLocation( Vec3(0,0.05,0) );
LeftFootContactSphere.setFrame(leftFoot)
LeftFootContactSphere.setName('LFootContact');

%% Right Foot Contact Sphere
RightFootContactSphere = ContactSphere();
RightFootContactSphere.setRadius(0.1);
RightFootContactSphere.setLocation( Vec3(0,0.05,0) );
RightFootContactSphere.setFrame(rightFoot)
RightFootContactSphere.setName('RFootContact');

%% Left Knee Contact
LeftKneeContactSphere = ContactSphere();
LeftKneeContactSphere.setRadius(0.05);
LeftKneeContactSphere.setLocation(Vec3(0,0.25,0));
LeftKneeContactSphere.setFrame(leftShank)
LeftKneeContactSphere.setName('LKneeContact');

%% Right Knee Contact
RightKneeContactSphere = ContactSphere();
RightKneeContactSphere.setRadius(0.05);
RightKneeContactSphere.setLocation( Vec3(0,0.25,0) );
RightKneeContactSphere.setFrame(rightShank)
RightKneeContactSphere.setName('RKneeContact');

%% Left Hip Contact
LeftPelvisContactSphere = ContactSphere();
LeftPelvisContactSphere.setRadius(0.05);
LeftPelvisContactSphere.setLocation( Vec3(0,0,-0.2) );
LeftPelvisContactSphere.setFrame(pelvis)
LeftPelvisContactSphere.setName('LHipContact');

%% Right Hip Contact
RightPelvisContactSphere = ContactSphere();
RightPelvisContactSphere.setRadius(0.05);
RightPelvisContactSphere.setLocation( Vec3(0,0,0.2) );
RightPelvisContactSphere.setFrame(pelvis)
RightPelvisContactSphere.setName('RHipContact');

%% Add contract Geometry to the model
model.addContactGeometry(groundContactSpace);
model.addContactGeometry(LeftFootContactSphere);
model.addContactGeometry(RightFootContactSphere);
model.addContactGeometry(LeftKneeContactSphere);
model.addContactGeometry(RightKneeContactSphere);
model.addContactGeometry(LeftPelvisContactSphere);
model.addContactGeometry(RightPelvisContactSphere);

%% Add Hunt Crossley Forces
% Contact Parameters
stiffness           = 1000000;
dissipation         = 2.0;
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
HuntCrossleyLeftFoot.setTransitionVelocity(0.1);

%% Create Hunt Crossley Right Foot
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
HuntCrossleyRightFoot.setTransitionVelocity(0.1);

%% Create Hunt Crossley Left Knee
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

%% Create Hunt Crossley Right Knee
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

%% Create Hunt Crossley Left Hip
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

%% Create Hunt Crossley Right Hip
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

%% Add Forces to the Model
model.addForce(HuntCrossleyLeftFoot);
model.addForce(HuntCrossleyRightFoot);
model.addForce(HuntCrossleyLeftKnee);
model.addForce(HuntCrossleyRightKnee);
model.addForce(HuntCrossleyLeftHip);
model.addForce(HuntCrossleyRightHip);

%% Add Coordinate Limit Forces
% Note this force takes input in Degrees to match coordinate panel in GUI
upperStiffness = 0.5;
lowerStiffness = 0.5;

kneeUpperLimit = 0;
kneeLowerLimit = -140;
hipUpperLimit = 100;
hipLowerLimit = -100;

damping = 0.025;
transition = 5;

% Create Left Knee Coordinate Limit Force
LKneeLimitTorque = CoordinateLimitForce();
LKneeLimitTorque.setName('LKneeLimitTorque')
LKneeLimitTorque.set_coordinate('LKnee_rz');
LKneeLimitTorque.setUpperStiffness(upperStiffness);
LKneeLimitTorque.setLowerStiffness(lowerStiffness);
LKneeLimitTorque.setUpperLimit(kneeUpperLimit);
LKneeLimitTorque.setLowerLimit(kneeLowerLimit);
LKneeLimitTorque.setDamping(damping);
LKneeLimitTorque.setTransition(transition)


% Create Right Knee Coordinate Limit Force
RKneeLimitTorque = CoordinateLimitForce();
RKneeLimitTorque.setName('RKneeLimitTorque')
RKneeLimitTorque.set_coordinate('RKnee_rz');
RKneeLimitTorque.setUpperStiffness(upperStiffness);
RKneeLimitTorque.setLowerStiffness(lowerStiffness);
RKneeLimitTorque.setUpperLimit(kneeUpperLimit);
RKneeLimitTorque.setLowerLimit(kneeLowerLimit);
RKneeLimitTorque.setDamping(damping);
RKneeLimitTorque.setTransition(transition)

% Create Left Hip Coordinate Limit Force
LHipLimitTorque = CoordinateLimitForce();
LHipLimitTorque.setName('LHipLimitTorque')
LHipLimitTorque.set_coordinate('LHip_rz');
LHipLimitTorque.setUpperStiffness(upperStiffness);
LHipLimitTorque.setLowerStiffness(lowerStiffness);
LHipLimitTorque.setUpperLimit(hipUpperLimit);
LHipLimitTorque.setLowerLimit(hipLowerLimit);
LHipLimitTorque.setDamping(damping);
LHipLimitTorque.setTransition(transition)

% Create Right Hip Coordinate Limit Force
RHipLimitTorque = CoordinateLimitForce();
RHipLimitTorque.setName('RHipLimitTorque')
RHipLimitTorque.set_coordinate('RHip_rz');
RHipLimitTorque.setUpperStiffness(upperStiffness);
RHipLimitTorque.setLowerStiffness(lowerStiffness);
RHipLimitTorque.setUpperLimit(hipUpperLimit);
RHipLimitTorque.setLowerLimit(hipLowerLimit);
RHipLimitTorque.setDamping(damping);
RHipLimitTorque.setTransition(transition)

%% Add Coordinate Limit Forces
model.addForce(LKneeLimitTorque);
model.addForce(RKneeLimitTorque);
model.addForce(LHipLimitTorque);
model.addForce(RHipLimitTorque);

%% Finalize connections
model.initSystem();

%% Print Model to file
model.print('Walker_Model.osim');
display(['Model Walker_Model.osim printed!']);
