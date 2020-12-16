% -------------------------------------------------------------------------- %
% OpenSim Moco: examplePrototypeCustomGoal.m                                 %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
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

function value = examplePrototypeCustomGoal
% This example allows Matlab users of Moco to prototype custom costs/goals
% to send to OpenSim/Moco developers for implementing in C++.
% If you can write your goal in the "...GoalIntegrand" and "...GoalValue"
% functions below and you can invoke evaluateCustomGoal() (shown below)
% without error, then OpenSim/Moco developers may be able to convert your goal
% into a proper C++ goal. Once your prototype works, create an issue
% on GitHub. We cannot guarantee that OpenSim/Moco developers will have the
% time to prioritize creating this goal in C++. See the C++ Moco example
% exampleMocoCustomEffortGoal to see how you could create a C++ plugin for a
% custom goal on your own.
%
% In C++, MocoGoals can be in cost mode or endpoint constraint mode. In this
% example, we expect that your goal is in cost mode.
%
% In this example, the custom goal is minimizing the sum of squared controls
% divided by the time duration of the phase.

import org.opensim.modeling.*;

study = MocoStudy();
problem = study.updProblem();
problem.setModel(ModelFactory.createPendulum());
problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/j0/q0/value', [], 0, 0.25 * pi);
problem.setStateInfo('/jointset/j0/q0/speed', [], 0, 0);
problem.setControlInfo('/tau0', [-2, 5]);
solver = study.initCasADiSolver();
trajectory = solver.createGuess();

value = evaluateCustomGoal(problem, trajectory, ...
        @calcMyCustomEffortGoalIntegrand, @calcMyCustomEffortGoalValue);

end

function integrand = calcMyCustomEffortGoalIntegrand(model, state)
% Compute the integrand for the integral portion of the cost goal.

model.realizeVelocity(state);
% This returns all model controls as a SimTK::Vector.
controls = model.getControls(state);
% This computes the sum of squared element values (squared 2-norm).
integrand = 0;
for i = 0:controls.size() - 1
    integrand = integrand + controls.get(i)^2;
end

end

function value = calcMyCustomEffortGoalValue(...
                        model, initial_state, final_state, integral)
% Compute the goal value from the phase's initial and final states, and from
% the integral of the integrand function over the phase. The integration is
% performed for you by evaluateCustomGoal().
value = integral / (final_state.getTime() - initial_state.getTime());
end

function goalValue = evaluateCustomGoal(...
                        problem, mocoTraj, integrandFunc, goalFunc)
% Test a custom goal, defined by integrand and goal functions, on the
% provided problem and MocoTrajectory.
model = problem.getPhase(0).getModelProcessor().process();
org.opensim.modeling.opensimMoco.prescribeControlsToModel(mocoTraj, model);
statesTraj = mocoTraj.exportToStatesTrajectory(problem);
model.initSystem();
N = statesTraj.getSize();
integrand = zeros(N, 1);
for i = 0:(N - 1)
    integrand(i + 1) = integrandFunc(model, statesTraj.get(i));
end

integral = trapz(integrand);
goalValue = goalFunc(model, statesTraj.front(), statesTraj.back(), integral);

end

