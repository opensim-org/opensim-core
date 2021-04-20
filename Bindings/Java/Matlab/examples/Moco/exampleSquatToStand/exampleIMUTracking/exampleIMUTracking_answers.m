function exampleIMUTracking_answers

%% Part 0: Load the Moco libraries and pre-configured Models.
import org.opensim.modeling.*;

% These models are provided for you (i.e., they are not part of Moco).
model = getTorqueDrivenModel();
% model = getMuscleDrivenModel();

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Create a new MocoStudy.
study = MocoStudy();

% Part 1b: Initialize the problem and set the model.
problem = study.updProblem();
problem.setModel(model);

% Part 1c: Set bounds on the problem.
%
% problem.setTimeBounds(initial_bounds, final_bounds)
% problem.setStateInfo(path, trajectory_bounds, inital_bounds, final_bounds)
%
% All *_bounds arguments can be set to a range, [lower upper], or to a
% single value (equal lower and upper bounds). Empty brackets, [], indicate
% using default bounds (if they exist). You may set multiple state infos at
% once using setStateInfoPattern():
%
% problem.setStateInfoPattern(pattern, trajectory_bounds, inital_bounds, ...
%       final_bounds)
%
% This function supports regular expressions in the 'pattern' argument;
% use '.*' to match any substring of the state/control path
% For example, the following will set all coordinate value state infos:
%
% problem.setStateInfoPattern('/path/to/states/.*/value', ...)

% Time bounds
problem.setTimeBounds(0, 1);

% Position bounds: the model should start in a squat and finish 
% standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', ...
    [-2, 0.5], -2, 0);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', ...
    [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);

% Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);

% Part 1d: Add a MocoControlCost to the problem.
problem.addGoal(MocoControlGoal('myeffort'));

% Part 1e: Configure the solver.
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(25);
solver.set_optim_convergence_tolerance(1e-2);
solver.set_optim_constraint_tolerance(1e-4);

if ~exist('predictSolution.sto', 'file')
% Part 1f: Solve! Write the solution to file, and visualize.
predictSolution = study.solve();
predictSolution.write('predictSolution.sto');
study.visualize(predictSolution);
end

%% Part 2: Add IMU tracking frames to the model 

%% Part 3: Create "synthetic" IMU acceleration signals
predictSolution = MocoTrajectory('predictSolution.sto');
framePaths = StdVectorString();
framePaths.add('/bodyset/torso');
framePaths.add('/bodyset/pelvis');
framePaths.add('/bodyset/femur_r');
framePaths.add('/bodyset/tibia_r');
torqueDrivenModel.initSystem();
accelerationsReference = ... 
    opensimSimulation.createSyntheticIMUAccelerationSignals(model, ...
        predictSolution.exportToStatesTable(), ...
        predictSolution.exportToControlsTable(), framePaths);

%% Part 4: IMU tracking problem 
accelerationIMUTracking = MocoAccelerationTrackingGoal('imu_tracking');
accelerationIMUTracking.setFramePaths(framePaths);
accelerationIMUTracking.setAccelerationReference(accelerationsReference);
accelerationIMUTracking.setGravityOffset(true);
accelerationIMUTracking.setExpressAccelerationsInTrackingFrames(true);
problem.addGoal(accelerationIMUTracking);

% Part 4a: Reduce the control cost weight so it now acts as a regularization 
% term.
problem.updGoal('myeffort').setWeight(0);

% Part 4b: Set the initial guess using the predictive problem solution.
% Tighten convergence tolerance to ensure smooth controls.
%solver.setGuessFile('predictSolution.sto');
%solver.set_optim_convergence_tolerance(1e-6);

if ~exist('trackingSolution.sto', 'file')
% Part 4c: Solve! Write the solution to file, and visualize.
trackingSolution = study.solve();
trackingSolution.write('trackingSolution.sto');
study.visualize(trackingSolution);
end


%% Part 5: Compare tracking solution to original prediction
% This is a convenience function provided for you. See mocoPlotTrajectory.m
mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto', ...
        'predict', 'track');


end

%% Model Creation and Plotting Convenience Functions 

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
model = Model('squatToStand_3dof9musc.osim');

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
model = Model('squatToStand_3dof9musc.osim');
model.finalizeConnections();

% Replace the muscles in the model with muscles from DeGroote, Fregly,
% et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem
% Formulations for Solving the Muscle Redundancy Problem". These muscles
% have the same properties as the original muscles but their characteristic
% curves are optimized for direct collocation (i.e. no discontinuities,
% twice differentiable, etc).
DeGrooteFregly2016Muscle().replaceMuscles(model);

% Make problems easier to solve by strengthening the model and widening the
% active force-length curve.
for m = 0:model.getMuscles().getSize()-1
    musc = model.updMuscles().get(m);
    musc.setMinControl(0.01);
    musc.set_ignore_activation_dynamics(false);
    musc.set_ignore_tendon_compliance(true);
    musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
    dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
    dgf.set_active_force_width_scale(1.5);
    dgf.set_ignore_passive_fiber_force(true);
end

end