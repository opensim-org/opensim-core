%% OpenSimInstantiateTugOfWarModel.m
%   Script builds a TugofWar model having Two muscles, attached to a block.
%   Linear Controller functions are defined for the muscles.

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
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

%% Import OpenSim libraries
import org.opensim.modeling.*

%% Instantiate an empty model
model = Model();
model.setName('TugOfWar')
% Convenience - Instantiate a zero value Vec3
zeroVec3 = Vec3(0);
% Set gravity as 0 since we are not concerned with ground contact
model.setGravity(zeroVec3);

%% Define Bodies and Joints in the Model
% Get a reference to the model's ground body
ground = model.getGround();

%% Attach Anchor geometry to the Ground
% Add offset frames so we can position the geometry
anchor1Offset = PhysicalOffsetFrame('anchor1', ground, Transform(Vec3(0,0.05,0.4)));
anchor1Offset.attachGeometry(Brick(Vec3(0.2,0.05,0.05)));
anchor2Offset = PhysicalOffsetFrame('anchor1', ground, Transform(Vec3(0,0.05,-0.4)));
anchor2Offset.attachGeometry(Brick(Vec3(0.2,0.05,0.05)));
% Add the frames and Geometry
model.addComponent(anchor1Offset)
model.addComponent(anchor2Offset)

% Instantiate a Body with mass, inertia, and a display geometry
block = Body();
block.setName('Block');
block.setMass(20);
block.setMassCenter(zeroVec3);
block.setInertia(Inertia(0.133,0.133,0.133,0,0,0));
% Add display geometry for the block
block.attachGeometry(Brick(Vec3(0.05)));


% Instantiate a Free Joint (6 DoF) that connects the block and ground.
blockSideLength      = 0.1;
locationInParentVec3 = Vec3(0, blockSideLength/2, 0);
blockToGround        = FreeJoint('blockToGround', ...
                            ground, locationInParentVec3, zeroVec3, ...
                            block, zeroVec3, zeroVec3);

% Set bounds on the 6 coordinates of the Free Joint.
angleRange 	  = [-pi/2, pi/2];
positionRange = [-1, 1];
for i=0:2, blockToGround.upd_coordinates(i).setRange(angleRange); end
for i=3:5, blockToGround.upd_coordinates(i).setRange(positionRange); end


% Add the block body and joint to the model
model.addBody(block);
model.addJoint(blockToGround);

%% Define Muscles in the Model
% Define parameters for a Muscle
maxIsometricForce  = 1000.0;
optimalFiberLength = 0.25;
tendonSlackLength  = 0.1;
pennationAngle 	   = 0.0;

% Instantiate a Muscle
muscle1 = Thelen2003Muscle();
muscle1.setName('muscle1')
muscle1.setMaxIsometricForce(maxIsometricForce)
muscle1.setOptimalFiberLength(optimalFiberLength)
muscle1.setTendonSlackLength(tendonSlackLength);
muscle1.setPennationAngleAtOptimalFiberLength(pennationAngle)

% Add Path points to muscle 1
muscle1.addNewPathPoint('muscle1-point1', ground, Vec3(0.0,0.05,-0.35))
muscle1.addNewPathPoint('muscle1-point2', block, Vec3(0.0,0.0,-0.05))

% Instantiate a second Muscle
muscle2 = Thelen2003Muscle();
muscle2.setName('muscle2');
muscle2.setMaxIsometricForce(maxIsometricForce)
muscle2.setOptimalFiberLength(optimalFiberLength)
muscle2.setTendonSlackLength(tendonSlackLength)
muscle2.setPennationAngleAtOptimalFiberLength(pennationAngle)

% Add Path points to  muscle 2
muscle2.addNewPathPoint('muscle2-point1', ground, Vec3(0.0,0.05,0.35))
muscle2.addNewPathPoint('muscle2-point2', block, Vec3(0.0,0.0,0.05))

% Add the two muscles (as forces) to the model
model.addForce(muscle1)
model.addForce(muscle2);

%% Define a Controller to the Model
initialTime = 0.0;
finalTime = 3.0;

muscleController = PrescribedController();
muscleController.setName('LinearRamp_Controller')
muscleController.setActuators(model.updActuators())

% Define linear functions for the control values for the two muscles
slopeAndIntercept1=ArrayDouble(0.0, 2);
slopeAndIntercept2=ArrayDouble(0.0, 2);

% Set the Muscle1 control coefficients
slopeAndIntercept1.setitem(0, -1.0/(finalTime-initialTime));
slopeAndIntercept1.setitem(1,  1.0);

% Set the Muscle1 control coefficients
slopeAndIntercept2.setitem(0, 0.95/(finalTime-initialTime));
slopeAndIntercept2.setitem(1, 0.05);

% Set the indiviudal muscle control functions for the prescribed muscle controller
muscleController.prescribeControlForActuator('muscle1', LinearFunction(slopeAndIntercept1));
muscleController.prescribeControlForActuator('muscle2', LinearFunction(slopeAndIntercept2));

% Add the controller to the model
model.addController(muscleController);

%% Finalize connections so that sockets connectees are correctly saved
model.finalizeConnections();

%% Print the model to a XML file (.osim)
modelPath = fullfile(cd, 'tug_of_war_muscles_controller.osim');
model.print(modelPath);
disp(['Model has been written to file: ' modelPath]);
