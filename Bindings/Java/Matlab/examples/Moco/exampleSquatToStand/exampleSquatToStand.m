function exampleSquatToStand

%% Part 0: Load the Moco libraries and pre-configured Models.
import org.opensim.modeling.*;
% These models are provided for you (i.e., they are not part of Moco).
torqueDrivenModel = getTorqueDrivenModel();
muscleDrivenModel = getMuscleDrivenModel();

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Create a new MocoStudy.


% Part 1b: Initialize the problem and set the model.


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
problem.setTimeBounds( );

% Position bounds: the model should start in a squat and finish 
% standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', );
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', );
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', );

% Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', );

% Part 1d: Add a MocoControlGoal to the problem.


% Part 1e: Configure the solver.
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals( );
solver.set_optim_convergence_tolerance( );
solver.set_optim_constraint_tolerance( );

if ~exist('predictSolution.sto', 'file')
% Part 1f: Solve! Write the solution to file, and visualize.


end

%% Part 2: Torque-driven Tracking Problem
% Part 2a: Construct a tracking reference TimeSeriesTable using filtered 
% data from the previous solution. Use a TableProcessor, which accepts a 
% base table and allows appending operations to modify the table.


% Part 2b: Add a MocoStateTrackingCost to the problem using the states
% from the predictive problem (via the TableProcessor we just created). 
% Enable the setAllowUnusedReferences() setting to ignore the controls in
% the predictive solution.


% Part 2c: Reduce the control goal weight so it now acts as a regularization 
% term.


% Part 2d: Set the initial guess using the predictive problem solution.
% Tighten convergence tolerance to ensure smooth controls.


if ~exist('trackingSolution.sto', 'file')
% Part 2e: Solve! Write the solution to file, and visualize.


end

%% Part 3: Compare Predictive and Tracking Solutions
% This is a convenience function provided for you. See mocoPlotTrajectory.m
mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto', ...
        'predict', 'track');

%% Part 4: Muscle-driven Inverse Problem
% Create a MocoInverse tool instance.


% Part 4a: Provide the model via a ModelProcessor. Similar to the TableProcessor,
% you can add operators to modify the base model.


% Part 4b: Set the reference kinematics using the same TableProcessor we used
% in the tracking problem.


% Part 4c: Set the time range, mesh interval, and convergence tolerance.
inverse.set_initial_time( );
inverse.set_final_time( );
inverse.set_mesh_interval( );
inverse.set_convergence_tolerance( );
inverse.set_constraint_tolerance( );

% Allow extra (unused) columns in the kinematics and minimize activations.
inverse.set_kinematics_allow_extra_columns(true);
inverse.set_minimize_sum_squared_activations(true);

% Append additional outputs path for quantities that are calculated
% post-hoc using the inverse problem solution.
inverse.append_output_paths('.*normalized_fiber_length');
inverse.append_output_paths('.*passive_force_multiplier');

% Part 4d: Solve! Write the MocoSolution to file.


% Part 4e: Get the outputs we calculated from the inverse solution.


%% Part 5: Muscle-driven Inverse Problem with Passive Assistance
% Part 5a: Create a new muscle-driven model, now adding a SpringGeneralizedForce 
% about the knee coordinate.


% Part 5b: Create a ModelProcessor similar to the previous one, using the same
% reserve actuator strength so we can compare muscle activity accurately.


% Part 5c: Solve! Write solution.


%% Part 6: Compare unassisted and assisted Inverse Problems.
fprintf('Cost without device: %f\n', ...
        inverseSolution.getMocoSolution().getObjective());
fprintf('Cost with device: %f\n', ...
        inverseDeviceSolution.getMocoSolution().getObjective());
% This is a convenience function provided for you. See below for the
% implementation.
compareInverseSolutions(inverseSolution, inverseDeviceSolution);

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
    musc.setMinControl(0);
    musc.set_ignore_activation_dynamics(false);
    musc.set_ignore_tendon_compliance(false);
    musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
    dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
    dgf.set_active_force_width_scale(1.5);
    dgf.set_tendon_compliance_dynamics_mode('implicit');
    if strcmp(char(musc.getName()), 'soleus_r')
        % Soleus has a very long tendon, so modeling its tendon as rigid
        % causes the fiber to be unrealistically long and generate
        % excessive passive fiber force.
        dgf.set_ignore_passive_fiber_force(true);
    end
end

end

function compareInverseSolutions(unassistedSolution, assistedSolution)

unassistedSolution = unassistedSolution.getMocoSolution();
assistedSolution = assistedSolution.getMocoSolution();
figure;
stateNames = unassistedSolution.getStateNames();
numStates = stateNames.size();
dim = 3;
iplot = 0;
for i = 0:numStates-1
    if contains(char(stateNames.get(i)), 'activation')
        iplot = iplot + 1;
        subplot(dim, dim, iplot);
        plot(unassistedSolution.getTimeMat(), ...
             unassistedSolution.getStateMat(stateNames.get(i)), '-r', ...
             'linewidth', 3);
        hold on
        plot(assistedSolution.getTimeMat(), ...
             assistedSolution.getStateMat(stateNames.get(i)), '--b', ...
             'linewidth', 2.5);
        hold off
        stateName = stateNames.get(i);
        plotTitle = stateName;
        plotTitle = strrep(plotTitle, '/forceset/', '');
        plotTitle = strrep(plotTitle, '/activation', '');
        title(plotTitle, 'Interpreter', 'none');
        xlabel('time (s)');
        ylabel('activation (-)');
        ylim([0, 1]);
        if iplot == 0
           legend('unassisted', 'assisted');
        end
    end
end

end
