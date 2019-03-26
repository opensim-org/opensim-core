function exampleSitToStand_answers

%% Part 0: Load the OpenSim and Moco libraries.
import org.opensim.modeling.*;

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Get a pre-configured MocoTool and set a torque-driven model on
% the underlying MocoProblem.
moco = configureMocoTool();
problem = moco.updProblem();
problem.setModelCopy(getTorqueDrivenModel());

% Part 1b: Add a MocoControlCost to the problem.
problem.addCost(MocoControlCost('effort'));

% Part 1c: Update the underlying MocoCasADiSolver with the new problem.
solver = MocoCasADiSolver.safeDownCast(moco.updSolver());
solver.resetProblem(problem);
solver.setGuess('bounds'); % This is also the default setting.

% Part 1d: Solve, write the solution to file, and visualize.
predictSolution = moco.solve();
predictSolution.write('predictSolution.sto');
moco.visualize(predictSolution);

%% Part 2: Torque-driven Tracking Problem
% Part 2a: Get a fresh MocoTool and set the torque-driven model on the
% problem.
moco = configureMocoTool();
problem = moco.updProblem();
problem.setModelCopy(getTorqueDrivenModel());

% Part 2b: Add a MocoStateTrackingCost() to the problem using the states
% from the predictive problem, and set weights to zero for states associated
% with the dependent coordinate in the model's knee CoordinateCoupler
% constraint.
tracking = MocoStateTrackingCost();
tracking.setName('tracking');
tracking.setReferenceFile('predictSolution.sto');
tracking.setAllowUnusedReferences(true);
tracking.setWeight('/jointset/patellofemoral_r/knee_angle_r_beta/value', 0);
tracking.setWeight('/jointset/patellofemoral_r/knee_angle_r_beta/speed', 0);
problem.addCost(tracking);

% Part 2c: Add a MocoControlCost() with a low weight to avoid oscillations
% in the solution.
problem.addCost(MocoControlCost('effort', 0.01));

% Part 2d: Update the underlying MocoCasADiSolver with the new problem.
solver = MocoCasADiSolver.safeDownCast(moco.updSolver());
solver.resetProblem(problem);

% Part 2e: Set the initial guess using the predictive problem solution.
solver.setGuess(predictSolution);

% Part 2f: Solve, write the solution to file, and visualize.
trackingSolution = moco.solve();
trackingSolution.write('trackingSolution.sto');
moco.visualize(trackingSolution);

%% Part 3: Compare Predictive and Tracking Solutions
% This is a convenience function provided for you. See below for the
% implementation details.
compareSolutions(predictSolution, trackingSolution)

%% Part 4: Muscle-driven Inverse Problem
inverse = MocoInverse();
inverse.setModel(getMuscleDrivenModel());
inverse.setKinematicsFile('predictSolution.sto');
inverse.set_kinematics_allow_extra_columns(true);
inverse.set_lowpass_cutoff_frequency_for_kinematics(6);
inverse.set_mesh_interval(0.05);
inverse.set_create_reserve_actuators(2);
inverse.set_minimize_sum_squared_states(true);
inverse.set_tolerance(1e-4);
inverse.append_output_paths('.*normalized_fiber_length');
inverse.append_output_paths('.*passive_force_multiplier');
inverseSolution = inverse.solve();
inverseSolution.getMocoSolution().write('inverseSolution.sto');
inverseOutputs = inverseSolution.getOutputs();
STOFileAdapter.write(inverseOutputs, 'muscle_outputs.sto');
fprintf('Cost without device: %f\n', ...
        inverseSolution.getMocoSolution().getObjective());

%% Part 5: Add an assistive device to the knee.
model = getMuscleDrivenModel();
device = SpringGeneralizedForce('knee_angle_r');
device.setStiffness(50);
device.setRestLength(0);
device.setViscosity(0);
model.addForce(device);
inverse.setModel(model);
inverseDeviceSolution = inverse.solve();
inverseDeviceSolution.getMocoSolution().write('inverseDeviceSolution.sto');
fprintf('Cost with device: %f\n', ...
        inverseDeviceSolution.getMocoSolution().getObjective());

%% Part 6: Compare unassisted and assisted Inverse Problems.
compareInverseSolutions(inverseSolution, inverseDeviceSolution);

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
addCoordinateActuator(model, 'hip_flexion_r', 150);
addCoordinateActuator(model, 'knee_angle_r', 300);
addCoordinateActuator(model, 'ankle_angle_r', 150);

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
    musc.set_ignore_tendon_compliance(true);
    musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
    dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
    dgf.set_active_force_width_scale(1.5);
    if strcmp(char(musc.getName()), 'soleus_r')
        % Soleus has a very long tendon, so modeling its tendon as rigid
        % causes the fiber to be unrealistically long and generate
        % excessive passive fiber force.
        dgf.set_ignore_passive_fiber_force(true);
    end
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
    stateName = stateNames.get(i).toCharArray';
    title(stateName(11:end), 'Interpreter', 'none')
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

function compareInverseSolutions(unassistedSolution, assistedSolution)

unassistedSolution = unassistedSolution.getMocoSolution();
assistedSolution = assistedSolution.getMocoSolution();
figure(3);
stateNames = unassistedSolution.getStateNames();
numStates = stateNames.size();
dim = ceil(sqrt(numStates));
for i = 0:numStates-1
    subplot(dim, dim, i+1);
    plot(unassistedSolution.getTimeMat(), ...
         unassistedSolution.getStateMat(stateNames.get(i)), '-r', ...
         'linewidth', 3);
    hold on
    plot(assistedSolution.getTimeMat(), ...
         assistedSolution.getStateMat(stateNames.get(i)), '--b', ...
         'linewidth', 2.5);
    hold off
    stateName = stateNames.get(i).toCharArray';
    plotTitle = stateName;
    plotTitle = strrep(plotTitle, '/forceset/', '');
    plotTitle = strrep(plotTitle, '/activation', '');
    title(plotTitle, 'Interpreter', 'none');
    xlabel('time (s)');
    ylabel('activation (-)');
    ylim([0, 1]);
    if i == 0
       legend('unassisted', 'assisted');
    end
end

end
