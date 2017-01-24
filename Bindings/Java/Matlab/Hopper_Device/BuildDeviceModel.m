% TODO license.

import org.opensim.modeling.*;

% Create the device.
% TODO talk about Container class (if it remains).
device = Container();
device.setName('device')

% The device's mass is distributed between two identical cuffs that attach to
% the hopper via WeldJoints (to be added below).
deviceMass = 2.0;
cuffA = Body('cuffA', deviceMass/2., Vec3(0), Inertia(0.5));
cuffB = Body('cuffA', deviceMass/2., Vec3(0), Inertia(0.5));
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
pathActuator.set_optimal_force(4000.0);
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
