% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleSlidingMass.m                                         %
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

import org.opensim.modeling.*;

model = Model();
model.setName('sliding_mass');
model.set_gravity(Vec3(0, 0, 0));
body = Body('body', 2.0, Vec3(0), Inertia(0));
model.addComponent(body);

% Allows translation along x.
joint = SliderJoint('slider', model.getGround(), body);
coord = joint.updCoordinate();
coord.setName('position');
model.addComponent(joint);

actu = CoordinateActuator();
actu.setCoordinate(coord);
actu.setName('actuator');
actu.setOptimalForce(1);
model.addComponent(actu);

body.attachGeometry(Sphere(0.05));

model.finalizeConnections();

% Create MocoStudy.
% ================
study = MocoStudy();
study.setName('sliding_mass');

% Define the optimal control problem.
% ===================================
problem = study.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% Initial time must be 0, final time can be within [0, 5].
problem.setTimeBounds(MocoInitialBounds(0.), MocoFinalBounds(0., 5.));

% Position must be within [-5, 5] throughout the motion.
% Initial position must be 0, final position must be 1.
problem.setStateInfo('/slider/position/value', MocoBounds(-5, 5), ...
    MocoInitialBounds(0), MocoFinalBounds(1));
% Speed must be within [-50, 50] throughout the motion.
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-50, 50], [0], [0]);

% Applied force must be between -50 and 50.
problem.setControlInfo('/actuator', MocoBounds(-50, 50));

% Cost.
% -----
problem.addGoal(MocoFinalTimeGoal('final_time'));



% Configure the solver.
% =====================
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);

% Now that we've finished setting up the tool, print it to a file.
study.print('sliding_mass.omoco');

% Solve the problem.
% ==================
solution = study.solve();

solution.write('sliding_mass_solution.sto');

% The following environment variable is set during automated testing.
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    study.visualize(solution);
    plot(solution.getTimeMat(), solution.getStatesTrajectoryMat());
    xlabel('time (s)');
    ylabel('states');
    legend('/slider/position/value', '/slider/position/speed');
end
