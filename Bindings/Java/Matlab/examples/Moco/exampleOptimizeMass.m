% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleOptimizeMass.m                                        %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Noah Gordon                                                     %
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

% Optimize the mass of a simple harmonic oscillator such that it follows the
% correct trajectory specified by the state bounds 
% and the MocoMarkerFinalGoal.

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

% Create MocoStudy.
% ================
study = MocoStudy();
study.setName('oscillator_spring_stiffness');

% Define the optimal control problem.
% ===================================
problem = study.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% Initial time must be 0, final time is final_time.
problem.setTimeBounds(0, final_time);

% Position must be within [-5, 5] throughout the motion.
% Initial position must be -0.5, final position must be in [0.25, 0.75].
problem.setStateInfo('/slider/position/value', [-5.0, 5.0], -0.5, [0.25, 0.75]);

% Speed must be within [-20, 20] throughout the motion.
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-20, 20], [0], [0]);

% Add Parameter. The default initial guess for a parameter is the midpoint of
% its bounds, *not* the value of the property in the model.
problem.addParameter(MocoParameter('oscillator_mass', 'body', 'mass',... 
    MocoBounds(0, 10)));

endpointCost = MocoMarkerFinalGoal();
endpointCost.setPointName('/markerset/marker');
endpointCost.setReferenceLocation(Vec3(0.5, 0, 0));

problem.addGoal(endpointCost);

solver = study.initTropterSolver();

study.print('optimize_mass.omoco');

sol = study.solve();
sol.write('optimize_mass_solution.sto');
