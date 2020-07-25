% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2020 Stanford University and the Authors             %
% Author(s): Christopher Dembia                                           %
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
%
% This basic example shows how to create a point-mass model, simulate the model,
% analyze the simulation, and modify the model.
% This example was created as a companion to the OpenSim webinar
% "Automating-OpenSim-Workflows-An-Intro-to-the-OpenSim-API-in-Matlab".
% https://opensim.stanford.edu/support/event_details.php?id=245&title=OpenSim-Webinar-Automating-OpenSim-Workflows-An-Intro-to-the-OpenSim-API-in-Matlab

import org.opensim.modeling.*;

% Build a model.
model = Model();
body = Body('body', 1.0, Vec3(0), Inertia(0));
body.attachGeometry(Sphere(0.1));
model.addBody(body);

joint = SliderJoint('joint', model.getGround(), body);
coord = joint.updCoordinate();
coord.setName('translation');
model.addJoint(joint);

actuator = CoordinateActuator('translation');
actuator.setName('actuator');
model.addForce(actuator);

controller = PrescribedController();
% methodsview(controller);
controller.addActuator(actuator);
controller.prescribeControlForActuator('actuator', Sine());
model.addController(controller);

model.finalizeConnections();
model.print('pointmass.osim');

% Simulate a model.
model.setUseVisualizer(true);
initState = model.initSystem();
disp(initState.getY());
finalState = opensimSimulation.simulate(model, initState, 1.5);

% Analyze a simulation.
disp(coord.getValue(finalState));
model.realizePosition(finalState);
disp(model.calcMassCenterPosition(finalState));
model.realizeAcceleration(finalState);
disp(joint.calcReactionOnParentExpressedInGround(finalState));

% Inspect and modify a model.
clear all;
import org.opensim.modeling.*;
model = Model('pointmass.osim');
body = model.updBodySet().get('body');
body.setMass(2 * body.getMass());
force = model.updForceSet().get('actuator');
disp(class(force));
actuator = CoordinateActuator.safeDownCast(force);
disp(class(actuator));
actuator.setOptimalForce(4 * actuator.getOptimalForce());

% Simulate the modified model.
initState = model.initSystem();
disp(initState.getY());
finalState = opensimSimulation.simulate(model, initState, 1.5);
disp(model.getCoordinateSet().get('translation').getValue(finalState));
