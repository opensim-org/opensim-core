% Builds a testbed for testing the device before attaching it to the hopper. We
% will attach one end of the device to ground ('/testbed/ground') and the other
% end to a sprung load ('/testbed/load').
% TODO license

import org.opensim.modeling.*;

% Create a new OpenSim model.
testbed = Model();
testbed.setName('testbed');
testbed.setGravity(Vec3(0));

% Create a 2500 kg load and add geometry for visualization.
load = Body('load', 2500, Vec3(0), Inertia(1));
sphere = Sphere(0.02); % radius of 2 cm.
sphere.setFrame(load);
sphere.setOpacity(0.5);
sphere.setColor(Vec3(0, 0, 1)); % blue.
load.attachGeometry(sphere);
testbed.addBody(load);

% Attach the load to ground with a FreeJoint and set the location of the load
% to (1, 0, 0).
gndToLoad = FreeJoint('gndToLoad', testbed.getGround(), load);
gndToLoad.upd_coordinates(3).setDefaultValue(1.0); % TODO use enum
testbed.addJoint(gndToLoad);

% Add a spring between the ground's origin and the load.
spring = PointToPointSpring(...
    testbed.getGround(), Vec3(0), ... % frame G and location in G of point 1.
    load, Vec3(0), ...                % frame F and location in F of point 2.
    5000, 1);                         % stiffness and rest length.
testbed.addForce(spring);

