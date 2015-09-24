% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
%                                                                         %   
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         % 
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% OpenSimCreateTugOfWarModel.m - script to perform the same model building and
% simulation tasks as the MainExample from the SDK examples

% This example script creates a model similar to the TugofWar API example.
% Two muscles are created and attached to a block. Linear controllers are
% defined for the muscles.

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

%///////////////////////////////////////////
%// DEFINE BODIES AND JOINTS OF THE MODEL //
%///////////////////////////////////////////

% Create a blank model
model = Model();
model.setName('TugOfWar')

% Convenience - create a zero value Vec3
zeroVec3 = ArrayDouble.createVec3(0);

% Set gravity as 0 since there is no ground contact model in this version
% of the example
model.setGravity(zeroVec3);

% GROUND BODY

% Get a reference to the model's ground body
ground = model.getGroundBody();

% Add display geometry to the ground to visualize in the GUI
ground.addDisplayGeometry('ground.vtp');
ground.addDisplayGeometry('anchor1.vtp');
ground.addDisplayGeometry('anchor2.vtp');

% "BLOCK" BODY

% Create a block Body with associate dimensions, mass properties, and DisplayGeometry
block = Body();
block.setName('Block');
block.setMass(20);
block.setMassCenter(zeroVec3);
% Need to set inertia
block.addDisplayGeometry('block.vtp');

% FREE JOINT

% Create a new free joint with 6 degrees-of-freedom (coordinates) between the block and ground bodies
blockSideLength      = 0.1;
locationInParentVec3 = ArrayDouble.createVec3([0, blockSideLength/2, 0]);
blockToGround        = FreeJoint('blockToGround', ground, locationInParentVec3, zeroVec3, block, zeroVec3, zeroVec3, false);

% Set bounds on coordinates
jointCoordinateSet=blockToGround.getCoordinateSet();
angleRange 	  = [-pi/2, pi/2];
positionRange = [-1, 1];
jointCoordinateSet.get(0).setRange(angleRange);
jointCoordinateSet.get(1).setRange(angleRange);
jointCoordinateSet.get(2).setRange(angleRange);
jointCoordinateSet.get(3).setRange(positionRange);
jointCoordinateSet.get(4).setRange(positionRange);
jointCoordinateSet.get(5).setRange(positionRange);

% Add the block body to the model
model.addBody(block)

%///////////////////////////////////////
%// DEFINE FORCES ACTING ON THE MODEL //
%///////////////////////////////////////

% Set muscle parameters
maxIsometricForce  = 1000.0;
optimalFiberLength = 0.25;
tendonSlackLength  = 0.1;
pennationAngle 	   = 0.0;

% Create new muscles
muscle1 = Thelen2003Muscle();
muscle1.setName('muscle1')
muscle1.setMaxIsometricForce(maxIsometricForce)
muscle1.setOptimalFiberLength(optimalFiberLength)
muscle1.setTendonSlackLength(tendonSlackLength);
muscle1.setPennationAngleAtOptimalFiberLength(pennationAngle)

% Path for muscle 1
muscle1.addNewPathPoint('muscle1-point1', ground, ArrayDouble.createVec3([0.0,0.05,-0.35]))
muscle1.addNewPathPoint('muscle1-point2', block, ArrayDouble.createVec3([0.0,0.0,-0.05]))

% Repeat for Muscle 2
muscle2 = Thelen2003Muscle();
muscle2.setName('muscle2');
muscle2.setMaxIsometricForce(maxIsometricForce)
muscle2.setOptimalFiberLength(optimalFiberLength)
muscle2.setTendonSlackLength(tendonSlackLength)
muscle2.setPennationAngleAtOptimalFiberLength(pennationAngle)

% Path for muscle 2
muscle2.addNewPathPoint('muscle2-point1', ground, ArrayDouble.createVec3([0.0,0.05,0.35]))
muscle2.addNewPathPoint('muscle2-point2', block, ArrayDouble.createVec3([0.0,0.0,0.05]))

% Add the two muscles (as forces) to the model
model.addForce(muscle1)
model.addForce(muscle2);

%Set up Controller
initialTime = 0.0;
finalTime = 3.0;

muscleController = PrescribedController();
muscleController.setName('LinearRamp Controller')
muscleController.setActuators(model.updActuators())

% Define linear functions for the control values for the two muscles
slopeAndIntercept1=ArrayDouble(0.0, 2);
slopeAndIntercept2=ArrayDouble(0.0, 2);

% Muscle1 control has slope of -1 starting 1 at t = 0
slopeAndIntercept1.setitem(0, -1.0/(finalTime-initialTime));
slopeAndIntercept1.setitem(1,  1.0);

% Muscle2 control has slope of 0.95 starting 0.05 at t = 0
slopeAndIntercept2.setitem(0, 0.95/(finalTime-initialTime));
slopeAndIntercept2.setitem(1, 0.05);

% Set the indiviudal muscle control functions for the prescribed muscle controller
muscleController.prescribeControlForActuator('muscle1', LinearFunction(slopeAndIntercept1));
muscleController.prescribeControlForActuator('muscle2', LinearFunction(slopeAndIntercept2));

% Add the control set controller to the model
model.addController(muscleController);

model.disownAllComponents();
model.print('tug_of_war_muscles_controller.osim');