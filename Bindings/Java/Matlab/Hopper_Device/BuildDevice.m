%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Thomas Uchida, Chris Dembia, Carmichael Ong, Nick Bianco,  %
%            Shrinidhi K. Lakshmikanth, Ajay Seth, James Dunne          %
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
function device = BuildDevice()
% Build a model of a device, consisting of a PathActuator, a proportional
% myoelectric controller, and two bodies.

import org.opensim.modeling.*;

% Create the device.
% TODO talk about Container class (if it remains).
% TODO try making this a Model (done: did not work; two Grounds).
device = Container();
device.setName('device')

% The device's mass is distributed between two identical cuffs that attach to
% the hopper via WeldJoints (to be added below).
deviceMass = 2.0;
cuffA = Body('cuffA', deviceMass/2., Vec3(0), Inertia(0.5));
cuffB = Body('cuffB', deviceMass/2., Vec3(0), Inertia(0.5));
device.addComponent(cuffA);
device.addComponent(cuffB);

% Attach a sphere to each cuff for visualization.
sphere = Sphere(0.01);
sphere.setName('sphere');
sphere.setColor(Vec3(1, 0, 0));
cuffA.attachGeometry(sphere);
cuffB.attachGeometry(sphere.clone());

% Create a WeldJoint to anchor cuffA to the hopper.
anchorA = WeldJoint();
anchorA.setName('anchorA');
% Connect the 'child_frame' (a PhysicalFrame) Socket of anchorA to cuffA.
% Note that only the child frame is connected now; the parent frame will be
% connected in a different file (TODO).
anchorA.connectSocket_child_frame(cuffA);
device.addComponent(anchorA);

% Create a WeldJoint to anchor cuffB to the hopper. Connect the 'child_frame'
% Socket of anchorB to cuffB and add anchorB to the device.
anchorB = WeldJoint();
anchorB.setName('anchorB');
anchorB.connectSocket_child_frame(cuffB);
device.addComponent(anchorB);

% Attach a PathActuator between the two cuffs.
pathActuator = PathActuator();
pathActuator.setName('cableAtoB');
pathActuator.updGeometryPath().setName('geompath');
pathActuator.set_optimal_force(1000.0);
pathActuator.addNewPathPoint('pointA', cuffA, Vec3(0));
pathActuator.addNewPathPoint('pointB', cuffB, Vec3(0));
device.addComponent(pathActuator);

% Create a proportional myoelectric controller.
controller = ToyPropMyoController();
controller.setName('controller');
controller.set_gain(1.0);
% Connect the controller's 'actuator' Socket to pathActuator.
controller.connectSocket_actuator(pathActuator);
device.addComponent(controller);

end
