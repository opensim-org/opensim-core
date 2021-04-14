function exampleIMUTracking_answers

%% Part 0: Load the Moco libraries and pre-configured Models.
import org.opensim.modeling.*;
% These models are provided for you (i.e., they are not part of Moco).
torqueDrivenModel = getTorqueDrivenModel();
muscleDrivenModel = getMuscleDrivenModel();

%% Part 1: Torque-driven Predictive Problem
% Part 1a: Create a new MocoStudy.
study = MocoStudy();

% Part 1b: Initialize the problem and set the model.
problem = study.updProblem();
problem.setModel(torqueDrivenModel);

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
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);

if ~exist('predictSolution.sto', 'file')
% Part 1f: Solve! Write the solution to file, and visualize.
predictSolution = study.solve();
predictSolution.write('predictSolution.sto');
study.visualize(predictSolution);
end

%% Part 2: Add IMU tracking frames to the model 

%% Part 3: Create "synthetic" IMU acceleration signals

%% Part 4: IMU tracking problem 

%% Part 5: Compare tracking solution to original prediction




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
        stateName = stateNames.get(i).toCharArray';
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