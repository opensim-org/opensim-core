function exampleSitToStand

%% Part 0: Load the OpenSim and Moco libraries.
import org.opensim.modeling.*;

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Get a pre-configured MocoTool and set a torque-driven model on
% the underlying MocoProblem.


% Part 1b: Add a MocoControlCost to the problem.


% Part 1c: Update the underlying MocoCasADiSolver with the new problem.


% Part 1d: Solve, write the solution to file, and visualize.


%% Part 2: Torque-driven Tracking Problem
% Part 2a: Get a fresh MocoTool and set the torque-driven model on the
% problem.


% Part 2b: Add a MocoStateTrackingCost() to the problem using the states
% from the predictive problem, and set weights to zero for states associated 
% with the dependent coordinate in the model's knee CoordinateCoupler
% constraint.


% Part 2c: Update the underlying MocoCasADiSolver with the new problem.


% Part 2d: Set the initial guess using the predictive problem solution.


% Part 2e: Solve, write the solution to file, and visualize.


%% Part 3: Compare Predictive and Tracking Solutions
% This is a convenience function provided for you. See below for the 
% implementation details.
compareSolutions(predictSolution, trackingSolution) 

end

function [moco] = configureMocoTool()

import org.opensim.modeling.*;

% Create a new MocoTool.
moco = MocoTool();

% Configure the solver.
solver = moco.initCasADiSolver();
solver.set_num_mesh_points(25);
solver.set_dynamics_mode('implicit');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_solver('ipopt');
solver.set_transcription_scheme('hermite-simpson');
solver.set_enforce_constraint_derivatives(true);
solver.set_optim_hessian_approximation('limited-memory');
solver.set_optim_finite_difference_scheme('forward');

% Set bounds on the problem.
problem = moco.updProblem();
problem.setTimeBounds(0, 1);
% The position bounds specify that the model should start in a crouch and
% finish standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', ...
    MocoBounds(-2, 0.5), MocoInitialBounds(-2), MocoFinalBounds(0));
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', ...
    [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);
% The velocity bounds specify that the model coordinates should start and
% end at zero.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/speed', ...
    MocoBounds(-50, 50), MocoInitialBounds(0), MocoFinalBounds(0));
problem.setStateInfo('/jointset/knee_r/knee_angle_r/speed', ...
    [-50, 50], 0, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/speed', ...
    [-50, 50], 0, 0);

end

function addCoordinateActuator(model, coordName, optForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);

model.addComponent(actu);

end

function [model] = getTorqueDrivenModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('sitToStand_3dof9musc.osim');

% Remove the muscles in the model.
model.updForceSet().clearAndDestroy();
model.initSystem();

% Add CoordinateActuators to the model degrees-of-freedom.
addCoordinateActuator(model, 'hip_flexion_r', 100);
addCoordinateActuator(model, 'knee_angle_r', 300);
addCoordinateActuator(model, 'ankle_angle_r', 100);

end

function [model] = getMuscleDrivenModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('sitToStand_3dof9musc.osim');
model.finalizeConnections();

% Replace the muscles in the model with muscles from DeGroote, Fregly, 
% et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem 
% Formulations for Solving the Muscle Redundancy Problem". These muscles
% have the same properties as the original muscles but their characteristic
% curves are optimized for direct collocation (i.e. no discontinuities, 
% twice differentiable, etc).
DeGrooteFregly2016Muscle().replaceMuscles(model);

% Turn off activation dynamics and muscle-tendon dynamics to keep the
% problem simple.
for m = 0:model.getMuscles().getSize()-1
    musc = model.updMuscles().get(m);
    musc.set_ignore_activation_dynamics(true);
    musc.set_ignore_tendon_compliance(true);
end

end

function compareSolutions(predictSolution, trackingSolution) 

%% States.
figure(1);
stateNames = predictSolution.getStateNames();
numStates = stateNames.size();
dim = ceil(sqrt(numStates));
for i = 0:numStates-1
    subplot(dim, dim, i+1);
    plot(predictSolution.getTimeMat(), ...
         predictSolution.getStateMat(stateNames.get(i)), '-r', ...
         'linewidth', 3);
    hold on
    plot(trackingSolution.getTimeMat(), ...
         trackingSolution.getStateMat(stateNames.get(i)), '--b', ...
         'linewidth', 2.5);
    hold off
    stateName = string(stateNames.get(i).toCharArray');
    title(stateName(10:end), 'Interpreter', 'none')
    xlabel('time (s)')
    if contains(stateName, 'value')
        ylabel('position (rad)')
    else
        ylabel('speed (rad/s)')
    end
    if i == 0
       legend('predict', 'track')
    end
end

%% Controls.
figure(2);
controlNames = predictSolution.getControlNames();
numControls = controlNames.size();
dim = ceil(sqrt(numControls));
for i = 0:numControls-1
    subplot(dim, dim, i+1);
    plot(predictSolution.getTimeMat(), ...
         predictSolution.getControlMat(controlNames.get(i)), '-r', ...
         'linewidth', 3);
    hold on
    plot(trackingSolution.getTimeMat(), ...
         trackingSolution.getControlMat(controlNames.get(i)), '--b', ...
         'linewidth', 2.5);
    hold off
    title(controlNames.get(i).toCharArray', 'Interpreter', 'none')        
    xlabel('time (s)')
    ylabel('value')
    if i == 0
       legend('predict', 'track')
    end
end

end
