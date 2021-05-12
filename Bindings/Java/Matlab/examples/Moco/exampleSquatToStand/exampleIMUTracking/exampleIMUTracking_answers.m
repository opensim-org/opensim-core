function exampleIMUTracking_answers

%% Part 0: Load the Moco libraries and pre-configured Models.
addpath('../'); % Add the directory above to access mocoPlotTrajectory.m
import org.opensim.modeling.*;

% Load a torque-driven, 3 degree-of-freedom model with a single leg and
% foot welded to the floor. See the function definition at the bottom of
% this file to see how the model is loaded and constructed.
model = getTorqueDrivenSquatToStandModel();

%% Part 1: Solve an effort minimization predictive problem
% Generate a simulation of a squat-to-stand motion that minimizes control
% effort. We'll use the results from this simulation to compute a set of 
% "synthetic" accelerometer signals later.

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

% Part 1d: Add a MocoControlGoal to the problem.
problem.addGoal(MocoControlGoal('myeffort'));

% Part 1e: Configure the solver.
solver = study.initCasADiSolver();
% A reasonably tight tolerance for the constraints and appropriately dense
% mesh are important to ensure that the model dynamics are enforced 
% accurately.
solver.set_num_mesh_intervals(50);
solver.set_optim_constraint_tolerance(1e-6);
% The convergence tolerance can be less tight, as long as the goal of the
% objective function is achieved.
solver.set_optim_convergence_tolerance(1e-2);

% TODO minimizing accelerations smoothes the solution during noisy tracking
% but only because the acceleration term dominates the cost function.
solver.set_multibody_dynamics_mode('implicit');
solver.set_minimize_implicit_multibody_accelerations(true);
% solver.set_implicit_multibody_accelerations_weight(1e0);

if ~exist('predictSolution.sto', 'file')
% Part 1f: Solve! Write the solution to file, and visualize.
predictSolution = study.solve();
predictSolution.write('predictSolution.sto');
% study.visualize(predictSolution);
end

%% Part 2: Add IMU tracking frames to the model 
% TODO should we have a step to add frames to the model that will be
% tracked? Depends on Ayman's SyntheticIMU component.

%% Part 3: Create synthetic accelerometer signals
% In this step, we'll create a set of fake, or "synthetic", accelerometer
% signals that mimic the output of an IMU sensor. To do this, we need to 
% compute the accelerations from our IMU frames, subtract the gravitational
% acceleration vector, and re-express the accelerations in the IMU frames.
% The <TODO insert SimulationReporter here> will help us do this.

% Part 3a: Load the prediction solution.
predictSolution = MocoTrajectory('predictSolution.sto');

% Part 3b: Create a vector containing paths to the IMU frame we will track.
framePaths = StdVectorString();
framePaths.add('/bodyset/torso');
framePaths.add('/bodyset/pelvis');
framePaths.add('/bodyset/femur_r');
framePaths.add('/bodyset/tibia_r');

% Part 3c: Compute the accelerometer signals. TODO this will change when we
% switch to Ayman's SyntheticIMUReporter.
model.initSystem();
accelerationReference = ... 
    opensimSimulation.createSyntheticIMUAccelerationSignals(model, ...
        predictSolution.exportToStatesTable(), ...
        predictSolution.exportToControlsTable(), framePaths);
    
% Part 3d: Plot the synthetic acceleration signals.
plotAccelerationSignals(accelerationReference);

%% Part 4: Synthetic acceleration tracking problem 
% Part 4a: TODO
accelerationIMUTracking = MocoAccelerationTrackingGoal('acceleration_tracking');
accelerationIMUTracking.setFramePaths(framePaths);
accelerationIMUTracking.setAccelerationReference(accelerationReference);
accelerationIMUTracking.setGravityOffset(true);
accelerationIMUTracking.setExpressAccelerationsInTrackingFrames(true);
problem.addGoal(accelerationIMUTracking);

% Part 4b: Reduce the control cost weight so it now acts as a regularization 
% term.
problem.updGoal('myeffort').setWeight(0.001);

if ~exist('trackingSolution.sto', 'file')
% Part 4c: Solve! Write the solution to file, and visualize.
trackingSolution = study.solve();
trackingSolution.write('trackingSolution.sto');
% study.visualize(trackingSolution);
end

%% Part 5: Compare tracking solution to original prediction
% This is a convenience function provided for you. See mocoPlotTrajectory.m
mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto', ...
        'predict', 'track');
    
%% Part 6: Noisy synthetic acceleration tracking problem 
% Part 6a: Add white noise to the synthetic accelerations signals. Set the 
% magnitude of the noise to 1% of the maximum original acceleration
% signals.
accelerationMat = accelerationReference.flatten().getMatrix().getAsMat();
noiseMagnitude = 0.01 * max(max(accelerationMat));
accelerationReferenceNoisy = ...
    addAccelerometerNoise(accelerationReference, noiseMagnitude);

% Part 6b: Update the acceleration tracking goal.
accelerationIMUTracking = MocoAccelerationTrackingGoal().safeDownCast(...
    problem.updGoal('acceleration_tracking'));
accelerationIMUTracking.setAccelerationReference(...
    accelerationReferenceNoisy);

% Part 6c: Plot the synthetic acceleration signals.
plotAccelerationSignals(accelerationReference, accelerationReferenceNoisy);

if ~exist('noisyTrackingSolution.sto', 'file')
% Part 6d: Solve! Write the solution to file, and visualize.
noisyTrackingSolution = study.solve();
noisyTrackingSolution.write('noisyTrackingSolution.sto');
% study.visualize(noisyTrackingSolution);
end

%% Part 7: Compare noisy tracking solution to previous tracking solution
mocoPlotTrajectory('trackingSolution.sto', 'noisyTrackingSolution.sto', ...
        'track', 'track (noisy)');

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

% Add to ForceSet
model.addForce(actu);

end

function [model] = getTorqueDrivenSquatToStandModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('../squatToStand_3dof9musc.osim');

% Remove the muscles in the model.
model.updForceSet().clearAndDestroy();
model.initSystem();

% Add CoordinateActuators to the model degrees-of-freedom.
addCoordinateActuator(model, 'hip_flexion_r', 150);
addCoordinateActuator(model, 'knee_angle_r', 300);
addCoordinateActuator(model, 'ankle_angle_r', 150);

end

function [model] = getMuscleDrivenSquatToStandModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('../squatToStand_3dof9musc.osim');
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

function [accelerationReferenceNoisy] = ...
        addAccelerometerNoise(accelerationReference, noiseMagnitude)
import org.opensim.modeling.*;

% Initial table to hold noisy acceleration signals
accelerationReferenceNoisy = ...
    TimeSeriesTableVec3(accelerationReference.getIndependentColumn());

nrows = accelerationReference.getNumRows();
ncols = accelerationReference.getNumColumns();
labels = accelerationReference.getColumnLabels();
for icol = 1:ncols
    % Get the original acceleration column 
    label = labels.get(icol-1);    
    % Here, a column is of type VectorVec3
    col = accelerationReference.getDependentColumn(label);    
    
    % Add white noise to this column
    colMat = col.getAsMat();
    noise = noiseMagnitude * randn(size(colMat));
    newCol = VectorVec3(nrows, Vec3(0.0));
    for irow = 1:nrows
        newElt = Vec3(0.0);
        newElt.set(0, colMat(irow, 1) + noise(irow, 1));
        newElt.set(1, colMat(irow, 2) + noise(irow, 2));
        newElt.set(2, colMat(irow, 3) + noise(irow, 3));
        newCol.set(irow-1, newElt);
    end
    
    % Append noisy column to the table
    accelerationReferenceNoisy.appendColumn(label, newCol);
end

end

function plotAccelerationSignals(varargin)

import org.opensim.modeling.*;

accelerationsReference = varargin{1};
if nargin == 2
    accelerationsReferenceNoisy = varargin{2};
end

% Create a time vector that can be used for plotting
timeVec = accelerationsReference.getIndependentColumn();
time = zeros(timeVec.size(),1);
for i = 1:timeVec.size()
   time(i) = timeVec.get(i-1);
end

figure;
% Plot the torso accelerations
subplot(2,2,1)
torso = accelerationsReference.getDependentColumn(...
    '/bodyset/torso').getAsMat();
plot(time, torso(:,1), 'linewidth', 2, 'color', 'black')
if nargin == 2
hold on
torsoNoisy = accelerationsReferenceNoisy.getDependentColumn(...
    '/bodyset/torso').getAsMat();
plot(time, torsoNoisy(:,1), 'linewidth', 2, 'color', 'red')
end
if nargin == 2
    legend('original', 'noise added', 'location', 'best');
end
title('torso')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

% Plot the pelvis accelerations
subplot(2,2,2)
pelvis = accelerationsReference.getDependentColumn(...
    '/bodyset/torso').getAsMat();
plot(time, pelvis(:,1), 'linewidth', 2, 'color', 'black')
if nargin == 2
hold on
pelvisNoisy = accelerationsReferenceNoisy.getDependentColumn(...
    '/bodyset/torso').getAsMat();
plot(time, pelvisNoisy(:,1), 'linewidth', 2, 'color', 'red')
end
title('pelvis')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

% Plot the femur accelerations
subplot(2,2,3)
femur = accelerationsReference.getDependentColumn(...
    '/bodyset/femur_r').getAsMat();
plot(time, femur(:,1), 'linewidth', 2, 'color', 'black')
if nargin == 2
hold on
femurNoisy = accelerationsReferenceNoisy.getDependentColumn(...
    '/bodyset/femur_r').getAsMat();
plot(time, femurNoisy(:,1), 'linewidth', 2, 'color', 'red')
end
title('femur')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

% Plot the tibia accelerations
subplot(2,2,4)
tibia = accelerationsReference.getDependentColumn(...
    '/bodyset/tibia_r').getAsMat();
plot(time, tibia(:,1), 'linewidth', 2, 'color', 'black')
if nargin == 2
hold on
tibiaNoisy = accelerationsReferenceNoisy.getDependentColumn(...
    '/bodyset/tibia_r').getAsMat();
plot(time, tibiaNoisy(:,1), 'linewidth', 2, 'color', 'red')
end
title('tibia')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

end