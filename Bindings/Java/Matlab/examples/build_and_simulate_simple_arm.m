%% build_and_simulate_simple_arm.m
% This API example demonstrates building and simulating a simple arm model
% consisting of two bodies, two joints, a muscle, and a controller. The API
% visualizer and a reporter are used to display the simulation results.

% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2019 Stanford University and the Authors                %
% Author(s): James Dunne, Chris Dembia, Tom Uchida                           %
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

% Author(s): James Dunne, Chris Dembia, Tom Uchida                           %

%% Import Java libraries
import org.opensim.modeling.*

%% Instantiate model
model = Model();
model.setUseVisualizer(true);

% Create two links, each with a mass of 1 kg, center of mass at the body's
% origin, and moments and products of inertia of zero.
humerus = Body('humerus', 1, Vec3(0), Inertia(0));
radius  = Body('radius',  1, Vec3(0), Inertia(0));

% Connect the bodies with pin joints. Assume each body is 1 m long.
shoulder = PinJoint('shoulder', ...
                    model.getGround(), ...  % Parent body
                    Vec3(0,2,0), ...        % Location in parent
                    Vec3(0), ...            % Orientation in parent
                    humerus, ...            % Child body
                    Vec3(0, 1, 0), ...      % Location in child
                    Vec3(0));               % Orientation in child
elbow = PinJoint('elbow', ...
                 humerus, Vec3(0), Vec3(0), radius, Vec3(0, 1, 0), Vec3(0));

% Add a muscle that flexes the elbow.
biceps = Millard2012EquilibriumMuscle('biceps', 200, 0.6, 0.55, 0);
biceps.addNewPathPoint('origin',    humerus, Vec3(0, 0.8, 0));
biceps.addNewPathPoint('insertion', radius,  Vec3(0, 0.7, 0));

% Add a controller that specifies the excitation of the muscle.
brain = PrescribedController();
brain.addActuator(biceps);
% Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
brain.prescribeControlForActuator('biceps', StepFunction(0.5, 3, 0.3, 1));

% Add components to the model.
model.addBody(humerus);    
model.addBody(radius);
model.addJoint(elbow);
model.addJoint(shoulder);  
model.addForce(biceps);
model.addController(brain);

% Add a console reporter to print the muscle fiber force and elbow angle. The
% output will be written to the log file (out.log) in the current directory.
reporter = ConsoleReporter();
reporter.set_report_time_interval(1.0);
reporter.addToReport(biceps.getOutput('fiber_force'));
reporter.addToReport(elbow.getCoordinate().getOutput('value'), 'elbow_angle');
model.addComponent(reporter);

% Add display geometry.
bodyGeometry = Ellipsoid(0.1, 0.5, 0.1);
bodyGeometry.setColor(Vec3(0.5));  % Gray

% Attach an ellipsoid to a frame located at the center of each body.
humerusCenter = PhysicalOffsetFrame();
humerusCenter.setName('humerusCenter');
humerusCenter.setParentFrame(humerus);
humerusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
humerus.addComponent(humerusCenter);
humerusCenter.attachGeometry(bodyGeometry.clone());

radiusCenter = PhysicalOffsetFrame();
radiusCenter.setName('radiusCenter');
radiusCenter.setParentFrame(radius);
radiusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
radius.addComponent(radiusCenter);
radiusCenter.attachGeometry(bodyGeometry.clone());

% Configure the model.
state = model.initSystem();
% Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, true);
elbow.getCoordinate().setValue(state, 0.5 * pi);
model.equilibrateMuscles(state);

% Simulate.
manager = Manager(model);
finalTime = 10;
manager.initialize( state );
manager.integrate( finalTime );
