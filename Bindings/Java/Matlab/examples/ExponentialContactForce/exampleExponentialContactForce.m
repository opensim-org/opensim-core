% -------------------------------------------------------------------------- %
%                 OpenSim:  exampleExponentialContactForce.m                 %
% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2026 Stanford University and the Authors                     %
% Author(s): Nicholas Bianco, F. C. Anderson                                 %
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

% This example demonstrates how to simulate a bouncing block using the
% ExponentialContactForce class.
%
% ExponentialContactForce uses a normal force model based on exponential
% spring functions. The exponential springs avoid expensive contact
% detection algorithms by always applying a normal force to the body,
% becoming negligibly small as soon as the body is more than a few
% centimeters above the contact plane. Additional speed up is achieved
% through a spring-based frictional model that includes a
% velocity-dependent damping term and a position-dependent elastic term,
% the latter eliminating drift entirely to maintain large integration step
% sizes.

import org.opensim.modeling.*

% Create a model with a single body representing a block that can move
% relative to the ground via a free joint.
model = Model();
model.setName('bouncing_block');

% Block body.
block = Body();
block.setName('block');
block.set_mass(10.0);
block.set_mass_center(Vec3(0));
block.setInertia(Inertia(1));
% Block geometry.
block.attachGeometry(Brick(Vec3(0.1)));

% Ground-to-block free joint.
freeJoint = FreeJoint('free_joint', ...
    model.getGround(), Vec3(0), Vec3(0), ...
    block, Vec3(0), Vec3(0));
model.addBody(block);
model.addJoint(freeJoint);

% Create the transform that defines the ground plane for the contact
% force. The transform rotates about the x-axis by -90 degrees so that
% the positive z-axis of the contact plane (i.e., the normal direction)
% aligns with the positive y-axis of the ground frame, which is the
% upward direction in OpenSim.
floorRotation = Rotation(-pi/2, Vec3(1, 0, 0));
floorTransform = Transform(floorRotation, Vec3(0, 0, 0));

% Add an ExponentialContactForce at each corner of the block.
hs = 0.1; % half the side length of the block
corners = { ...
    Vec3( hs, -hs,  hs), Vec3( hs, -hs, -hs), ...
    Vec3(-hs, -hs, -hs), Vec3(-hs, -hs,  hs), ...
    Vec3( hs,  hs,  hs), Vec3( hs,  hs, -hs), ...
    Vec3(-hs,  hs, -hs), Vec3(-hs,  hs,  hs)};
for i = 1:8
    esf = ExponentialContactForce(floorTransform, block, corners{i});
    esf.setName(['ExpSprForce' num2str(i - 1)]);
    model.addForce(esf);
end

% Finalize the model and initialize the system.
model.finalizeConnections();
state = model.initSystem();

% Set the block initial conditions.
freeJoint.get_coordinates(3).setValue(state, -1.5);      % tx
freeJoint.get_coordinates(4).setValue(state, 2.0 * hs);  % ty
freeJoint.get_coordinates(5).setValue(state,  1.0);      % tz
freeJoint.get_coordinates(3).setSpeedValue(state, -1.0); % tx speed
freeJoint.get_coordinates(2).setSpeedValue(state, 2*pi); % rz speed (wz)

% Simulate the model.
manager = Manager(model);
manager.setIntegratorAccuracy(1e-4);
manager.initialize(state);
manager.setWriteToStorage(true);
state = manager.integrate(5.0);

% Visualize the results.
statesTable = manager.getStatesTable();
VisualizerUtilities.showMotion(model, statesTable);
