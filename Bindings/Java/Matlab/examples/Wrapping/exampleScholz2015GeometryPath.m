% -------------------------------------------------------------------------- %
%                OpenSim:  exampleScholz2015GeometryPath.m                   %
% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2025 Stanford University and the Authors                %
% Author(s): Nicholas Bianco                                                 %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the 'License') you may     %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an 'AS IS' BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% This example demonstrates the basic elements of creating a geometry-based
% path using the Scholz2015GeometryPath class. The path is used to define the
% length and speed of a PathSpring, which applies forces to the bodies of a
% double pendulum model. The path contains three path points and wraps around
% a cylindrical obstacle.

function exampleScholz2015GeometryPath()

import org.opensim.modeling.*;

model = ModelFactory.createDoublePendulum();
model.setUseVisualizer(true);

% Create a PathSpring with a Scholz2015GeometryPath.
spring = PathSpring();
spring.setName('path_spring');
spring.setRestingLength(0.25);
spring.setDissipation(0.75);
spring.setStiffness(10.0);
spring.set_path(Scholz2015GeometryPath());
model.addComponent(spring);

% Configure the Scholz2015GeometryPath. We will update the path after
% adding it to the PathSpring, so that the Socket connections in each
% Station (i.e., path point) remain valid.
path = Scholz2015GeometryPath.safeDownCast(spring.updPath());
path.setName('path');

% Add a path point connected to ground. Since this is the first path point,
% it defines the origin of the path.
path.appendPathPoint(model.getGround(), Vec3(0.05, 0.05, 0.));

% Append a second path point, creating a straight line segment between the
% ground and body "b0".
path.appendPathPoint(model.getBodySet().get('b0'), Vec3(-0.5, 0.1, 0.));

% Create a ContactCylinder to use as a wrapping obstacle to the path. The
% cylinder has radius 0.15 m and is attached to body "b0".
obstacle = ContactCylinder(0.15, ...
    Vec3(-0.2, 0.2, 0.), Vec3(0), ...
    model.getBodySet().get('b0'));
model.addComponent(obstacle);

% Before we add the obstacle to the path, we must provide a "contact hint"
% to initialize the wrapping solver. The contact hint is a point on the
% surface of the obstacle, expressed in the obstacle's frame. The point
% does not have to lie on the contact geometry's surface, nor does it have
% to belong to a valid cable path. The choice of the contact hint will
% determine which side of the cylinder the path wraps around.
contact_hint = Vec3(0., 0.15, 0.);
path.appendObstacle(obstacle, contact_hint);

% At least one path point must follow an obstacle (or list of obstacles)
% in a Scholz2015GeometryPath. Since this is the last path point we are
% adding, it defines the insertion of the path.
path.appendPathPoint(model.getBodySet().get('b1'), Vec3(-0.5, 0.1, 0.));

% Initialize the system.
state = model.initSystem();
model.updVisualizer().updSimbodyVisualizer().setBackgroundTypeByInt(2);
transform = Transform(Vec3(0.2, -0.8, 3.0));
model.updVisualizer().updSimbodyVisualizer().setCameraTransform(transform);
model.updVisualizer().updSimbodyVisualizer().setCameraFieldOfView(0.9);

% Simulate.
manager = Manager(model);
manager.initialize(state);
manager.integrate(20.0);

end
