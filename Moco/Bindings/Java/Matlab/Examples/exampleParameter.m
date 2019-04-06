% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleParameter.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2017 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Christopher Dembia                                              %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

stiffness = 100.0;
mass = 5.0;
final_time = pi * sqrt(mass / stiffness);


import org.opensim.modeling.*;

model = Model();
model.setName('oscillator');
model.set_gravity(Vec3(0, 0, 0));
body = Body('body', 0.5*mass, Vec3(0), Inertia(0));
model.addComponent(body);
marker = Marker('marker', body, Vec3(0, 0, 0));
model.addMarker(marker);


% Allows translation along x.
joint = SliderJoint('slider', model.getGround(), body);
coord = joint.updCoordinate();
coord.setName('position');
model.addComponent(joint);

spring = SpringGeneralizedForce();
spring.set_coordinate('position');
spring.setRestLength(0.0);
spring.setStiffness(stiffness);
spring.setViscosity(0.0);
model.addComponent(spring);

N = 25;

% Create MocoTool.
% ================
moco = MocoTool();
moco.setName('oscillator_spring_stiffness');

% Define the optimal control problem.
% ===================================
problem = moco.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% Initial time must be 0, final time is FINAL_TIME.
problem.setTimeBounds(0, final_time);

% Initial position must be -0.5, final position must be in [0.25, 0.75]
problem.setStateInfo('/slider/position/value', [-5.0, 5.0], -0.5, [0.25, 0.75]);

% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-20, 20], [0], [0]);

%Add Parameter
problem.addParameter(MocoParameter('oscillator_mass', 'body', 'mass', MocoBounds(0, 10)));

endpointCost = MocoMarkerEndpointCost();
endpointCost.setPointName('/markerset/marker');
endpointCost.setReferenceLocation(Vec3(0.5, 0, 0));

problem.addCost(endpointCost);


solver = moco.initCasADiSolver();
solver.set_num_mesh_points(N);

sol = moco.solve();
sol.write('parameter_solution.sto');