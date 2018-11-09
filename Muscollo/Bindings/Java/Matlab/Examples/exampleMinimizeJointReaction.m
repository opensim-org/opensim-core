% -------------------------------------------------------------------------- %
% OpenSim Muscollo: exampleMinimizeJointReaction.m                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2017 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Nicholas Bianco                                                 %
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

function exampleMinimizeJointReaction()

import org.opensim.modeling.*;

controlEffortSolution = minimizeControlEffort();
figure(1)
tic
plot(controlEffortSolution.getTimeMat(),...
     controlEffortSolution.getStatesTrajectoryMat())
xlabel('time (s)')
ylabel('states')
legend('pin/angle/value', 'slider/angle/speed')
toc
% title('minimize control effort')

tic
jointReactionSolution = minimizeJointReactionLoads();
toc
figure(2)
plot(jointReactionSolution.getTimeMat(),...
     jointReactionSolution.getStatesTrajectoryMat())
xlabel('time (s)')
ylabel('states')
legend('pin/angle/value', 'slider/angle/speed')
% title('minimize joint reaction loads')

end

function model = createInvertedPendulumModel() 

import org.opensim.modeling.*;

model = Model();
model.setName('inverted_pendulum');
body = Body('body', 1.0, Vec3(0), Inertia(0));
model.addComponent(body);

joint = PinJoint('pin', model.getGround(), Vec3(0), Vec3(0), body, ...
                  Vec3(-1, 0, 0), Vec3(0));
coord = joint.updCoordinate();
coord.setName('angle');
model.addComponent(joint);

actu = CoordinateActuator();
actu.setCoordinate(coord);
actu.setName('actuator');
actu.setOptimalForce(1);
model.addComponent(actu);

geom = Ellipsoid(0.5, 0.1, 0.1);
transform = Transform(Vec3(-0.5, 0, 0));
bodyCenter = PhysicalOffsetFrame('body_center', 'body', transform);
body.addComponent(bodyCenter);
bodyCenter.attachGeometry(geom);

end

function solution = minimizeControlEffort() 

import org.opensim.modeling.*;

muco = MucoTool();
muco.setName('minimize_control_effort');
problem = muco.updProblem();
problem.setModel(createInvertedPendulumModel());

problem.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(1));
problem.setStateInfo('pin/angle/value', MucoBounds(-10, 10), ...
    MucoInitialBounds(0), MucoFinalBounds(pi));
problem.setStateInfo('pin/angle/speed', MucoBounds(-50, 50), ...
    MucoInitialBounds(0), MucoFinalBounds(0));
problem.setControlInfo('actuator', MucoBounds(-100, 100));

effort = MucoControlCost();
problem.addCost(effort);

solver = muco.initSolver();
solver.set_num_mesh_points(50);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1E-3);
solver.set_optim_hessian_approximation('exact');
solver.setGuess('bounds');

solution = muco.solve();
solution.write('pendulum_control_effort_solution.sto');
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    muco.visualize(solution);
end

end

function solution = minimizeJointReactionLoads()

import org.opensim.modeling.*;

muco = MucoTool();
muco.setName('minimize_joint_reaction_loads');
problem = muco.updProblem();
problem.setModel(createInvertedPendulumModel());

problem.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(1));
problem.setStateInfo('pin/angle/value', MucoBounds(-10, 10), ...
    MucoInitialBounds(0), MucoFinalBounds(pi));
problem.setStateInfo('pin/angle/speed', MucoBounds(-50, 50), ...
    MucoInitialBounds(0), MucoFinalBounds(0));
problem.setControlInfo('actuator', MucoBounds(-100, 100));

reaction = MucoJointReactionNormCost();
reaction.setJointPath('pin');
problem.addCost(reaction);

solver = muco.initSolver();
solver.set_num_mesh_points(50);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1E-3);
solver.set_optim_hessian_approximation('exact');
solver.setGuess('bounds');

solution = muco.solve();
solution.write('pendulum_control_effort_solution.sto');
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    muco.visualize(solution);
end

end
