% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMinimizeJointReaction.m                               %
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

% Control effort minimization problem.
% ====================================
% This problem minimizes the squared control effort, integrated over the phase.
effort = MocoControlGoal();
runInvertedPendulumProblem('minimize_control_effort', effort);

% Joint reaction load minimization problem.
% =========================================
% This problem minimizes the reaction loads on the rotating body at the pin 
% joint. Specifically, the norm of the reaction forces and moments integrated
% over the phase is minimized.
reaction = MocoJointReactionGoal();
reaction.setJointPath('pin');
runInvertedPendulumProblem('minimize_joint_reaction_loads', reaction);

end

function solution = runInvertedPendulumProblem(name, goal)

import org.opensim.modeling.*;

% Create the inverted pendulum model.
% ===================================
model = createInvertedPendulumModel();

% Create MocoStudy.
% ================
study = MocoStudy();
% The name of the study is used as part of the file name for the solution.
study.setName(name);

% Define the optimal control problem.
% ===================================
problem = study.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% Initial time must be zero, final time must be 1.
problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(1));

% Initial position must be 0, final position must be 180 degrees.
problem.setStateInfo('/pin/angle/value', MocoBounds(-10, 10), ...
    MocoInitialBounds(0), MocoFinalBounds(pi));
% Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/pin/angle/speed', [-50, 50], [0], [0]);

% Applied moment must be between -100 and 100 N-m.
problem.setControlInfo('/forceset/actuator', MocoBounds(-100, 100));

% Goal.
% -----
problem.addGoal(goal);

% Configure the solver.
% =====================
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);
solver.set_optim_convergence_tolerance(1e-3);

% Now that we've finished setting up the tool, print it to a file.
study.print([name '.omoco']);

% Solve the problem.
% ==================
solution = study.solve();
solution.write([name '_solution.sto']);

% Visualize.
% ==========
% The following environment variable is set during automated testing.
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    study.visualize(solution);
end

% Plot results.
% =============
figure;

% Plot states trajectory solution.
% --------------------------------
subplot(3,1,1);
time = solution.getTimeMat();
plot(time, solution.getStatesTrajectoryMat())
xlabel('time (s)')
ylabel('states')
legend('pin/angle/value', 'pin/angle/speed')
title(strrep(name, '_', ' '))

% Plot controls trajectory solution.
% ----------------------------------
subplot(3,1,2)
plot(time, solution.getControlsTrajectoryMat())
xlabel('time (s)')
ylabel('control')
legend('actuator')

% Plot trajectory of the norm of the joint reaction loads.
% --------------------------------------------------------
statesTraj = solution.exportToStatesTrajectory(problem);

% This function adds the solution controls as prescribed controllers to the 
% model. This ensures that the correct reaction loads are computed when 
% realizing to acceleration.
opensimMoco.prescribeControlsToModel(solution, model);
model.initSystem();

% Compute reaction loads.
jointReactionNorm = zeros(size(time));
for i = 1:length(time)
    state = statesTraj.get(i-1);
    
    % Need to realize to acceleration for the reaction loads.
    model.realizeAcceleration(state);
    joint = PinJoint().safeDownCast(model.getComponent('pin'));
    reactionLoadsSpatialVec = ...
        joint.calcReactionOnChildExpressedInGround(state);
    
    % Convert the reaction load SpatialVec to a usable format.
    reactionLoadsFlat = flattenSpatialVec(reactionLoadsSpatialVec);
    
    % Compute the norm.
    jointReactionNorm(i) = norm(reactionLoadsFlat);
end

subplot(3,1,3);
plot(time, jointReactionNorm)
integralJointReactionNorm = trapz(time, jointReactionNorm)
xlabel('time (s)')
ylabel('norm reaction loads')

end

function model = createInvertedPendulumModel()

import org.opensim.modeling.*;

model = Model();
model.setName('inverted_pendulum');
body = Body('body', 1.0, Vec3(0), Inertia(1));
model.addComponent(body);

joint = PinJoint('pin', model.getGround(), Vec3(0), Vec3(0), ...
                        body,              Vec3(-1, 0, 0), Vec3(0));
coord = joint.updCoordinate();
coord.setName('angle');
model.addComponent(joint);

actu = CoordinateActuator();
actu.setCoordinate(coord);
actu.setName('actuator');
actu.setOptimalForce(1);
model.addForce(actu);

geom = Ellipsoid(0.5, 0.1, 0.1);
transform = Transform(Vec3(-0.5, 0, 0));
body_center = PhysicalOffsetFrame('body_center', body, transform);
body.addComponent(body_center);
body_center.attachGeometry(geom);

model.finalizeConnections();

end

function spatialVecFlat = flattenSpatialVec(spatialVec)

spatialVecFlat = zeros(6,1);
spatialVecFlat(1) = spatialVec.get(0).get(0);
spatialVecFlat(2) = spatialVec.get(0).get(1);
spatialVecFlat(3) = spatialVec.get(0).get(2);
spatialVecFlat(4) = spatialVec.get(1).get(0);
spatialVecFlat(5) = spatialVec.get(1).get(1);
spatialVecFlat(6) = spatialVec.get(1).get(2);

end
