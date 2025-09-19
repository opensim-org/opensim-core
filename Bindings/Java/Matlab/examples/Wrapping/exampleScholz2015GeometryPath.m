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
% double pendulum model. The path wraps around a cylindrical obstacle and
% contains a via point.

function exampleScholz2015GeometryPath()

import org.opensim.modeling.*;

model = ModelFactory.createDoublePendulum();
model.setUseVisualizer(true);

% Create a PathSpring with a Scholz2015GeometryPath.
spring = PathSpring();
spring.setName('path_spring');
spring.setRestingLength(0.5);
spring.setDissipation(0.5);
spring.setStiffness(25.0);
spring.set_path(Scholz2015GeometryPath());
model.addComponent(spring);

% Configure the Scholz2015GeometryPath.
path = Scholz2015GeometryPath.safeDownCast(spring.updPath());
path.setName('path');

% The origin and insertion points are stored using the 'origin' and
% 'insertion' properties of Scholz2015GeometryPath, which are of type
% Station. We must update these properties after the Scholz2015GeometryPath
% has been added to the PathSpring, so that the Socket connections in each
% Station remain valid.
path.setOrigin(model.getGround(), Vec3(0.05, 0.05, 0.));
path.setInsertion(model.getBodySet().get('b1'), Vec3(-0.25, 0.1, 0.));

% Add a ContactCylinder wrapping obstacle to the path.
obstacle = ContactCylinder(0.1, ...
    Vec3(-0.5, 0.1, 0.), Vec3(0), ...
    model.getBodySet().get('b0'));
model.addComponent(obstacle);
path.addObstacle(obstacle, Vec3(0., 0.1, 0.));

% Add a via point to the path.
path.addViaPoint(model.getBodySet().get('b1'), Vec3(-0.75, 0.1, 0.));

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
