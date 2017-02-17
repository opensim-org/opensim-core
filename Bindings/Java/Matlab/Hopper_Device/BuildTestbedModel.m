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

% Builds a testbed for testing the device before attaching it to the hopper. We
% will attach one end of the device to ground ('/testbed/ground') and the other
% end to a sprung load ('/testbed/load').

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

